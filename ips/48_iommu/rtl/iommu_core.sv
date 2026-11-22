// SPDX-License-Identifier: Apache-2.0
// iommu_core.sv - Protocol-Agnostic Hardware IOMMU / SMMU-Lite Core (Sv32 + 16-Entry TLB)

module iommu_core
  import bus_pkg::*;
  import iommu_pkg::*;
#(
  parameter int unsigned NUM_STREAMS          = IOMMU_NUM_STREAMS,
  parameter int unsigned TLB_ENTRIES          = IOMMU_TLB_ENTRIES,
  parameter int unsigned FAULT_FIFO_DEPTH     = IOMMU_FAULT_DEPTH,
  parameter bit          USE_AXI_ID_AS_STREAM = 1'b0
) (
  input  logic       clk_i,
  input  logic       rst_ni,

  // Generic CSR Control Slave Interface
  input  csr_req_t   csr_req_i,
  output csr_rsp_t   csr_rsp_o,

  // Inline AXI4 Device Slave Interface (Untranslated IOVA)
  input  axi4_m2s_t  s_axi_dev_i,
  output axi4_s2m_t  s_axi_dev_o,
  input  logic [3:0] aw_stream_id_i,
  input  logic [3:0] ar_stream_id_i,

  // Downstream AXI4 Memory Master Interface (Translated PA + Muxed PTW Reads)
  output axi4_m2s_t  m_axi_mem_o,
  input  axi4_s2m_t  m_axi_mem_i,

  // Fault Interrupt Output
  output logic       irq_o
);

  localparam int unsigned FAULT_CNT_W = $clog2(FAULT_FIFO_DEPTH + 1);
  localparam int unsigned FAULT_REC_W = $bits(fault_rec_t);

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_LOOKUP,
    ST_PTW_WAIT,
    ST_MEM_WR_AW,
    ST_MEM_WR_B,
    ST_MEM_RD_AR,
    ST_MEM_RD_R,
    ST_FAULT_WR_W,
    ST_FAULT_WR_B,
    ST_FAULT_RD_R
  } core_state_e;

  // -------------------------------------------------------------------------
  // Control & Status Registers
  // -------------------------------------------------------------------------
  logic        ctrl_en_q;
  logic        ad_check_en_q;
  logic        sid_from_axi_id_q;
  logic [1:0]  irq_en_q;
  logic [1:0]  irq_stat_q;
  logic        fault_ovf_q;

  // Performance counters
  logic [31:0] perf_tlb_hit_q;
  logic [31:0] perf_tlb_miss_q;
  logic [31:0] perf_ptw_cyc_q;
  logic [31:0] perf_faults_q;

  // Per-stream device context table (16 streams)
  stream_ctx_t stream_ctx_q [0:NUM_STREAMS-1];

  // -------------------------------------------------------------------------
  // Data-path Transaction State
  // -------------------------------------------------------------------------
  core_state_e state_q;
  logic        rr_rd_prio_q;
  logic        is_write_q;
  logic [31:0] addr_q;
  logic [31:0] pa_q;
  logic [7:0]  len_q;
  logic [7:0]  beats_rem_q;
  logic [2:0]  size_q;
  logic [1:0]  burst_q;
  logic [3:0]  id_q;
  logic [3:0]  qos_q;
  logic [2:0]  prot_q;
  logic [3:0]  stream_id_q;
  logic        aw_done_q;
  logic        w_done_q;

  // -------------------------------------------------------------------------
  // TLB Instance
  // -------------------------------------------------------------------------
  stream_ctx_t cur_ctx;
  assign cur_ctx = stream_ctx_q[stream_id_q];

  logic       tlb_hit;
  tlb_entry_t tlb_hit_entry;
  logic       tlb_insert_valid;
  tlb_entry_t tlb_insert_entry;
  logic       tlb_inv_all;
  logic       tlb_inv_stream;
  logic       tlb_inv_iova;
  logic [3:0] tlb_inv_sid;
  logic [19:0] tlb_inv_vpn;

  iommu_tlb #(
    .ENTRIES (TLB_ENTRIES)
  ) u_tlb (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .lookup_stream_id_i (stream_id_q),
    .lookup_asid_i      (cur_ctx.asid),
    .lookup_vpn_i       (addr_q[31:12]),
    .hit_o              (tlb_hit),
    .hit_entry_o        (tlb_hit_entry),
    .insert_valid_i     (tlb_insert_valid),
    .insert_entry_i     (tlb_insert_entry),
    .inv_all_i          (tlb_inv_all),
    .inv_stream_i       (tlb_inv_stream),
    .inv_stream_id_i    (tlb_inv_sid),
    .inv_iova_i         (tlb_inv_iova),
    .inv_vpn_i          (tlb_inv_vpn)
  );

  // -------------------------------------------------------------------------
  // Hardware Page-Table Walker (Sv32)
  // -------------------------------------------------------------------------
  logic         ptw_start;
  logic         ptw_busy;
  logic         ptw_done;
  logic         ptw_fault;
  fault_cause_e ptw_fault_cause;
  logic         ptw_is_superpage;
  logic [21:0]  ptw_ppn;
  logic         ptw_perm_r;
  logic         ptw_perm_w;
  logic         ptw_flag_a;
  logic         ptw_flag_d;
  axi4_m2s_t    ptw_axi_o;
  axi4_s2m_t    ptw_axi_i;

  iommu_ptw u_ptw (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .start_i        (ptw_start),
    .is_write_i     (is_write_q),
    .ad_check_en_i  (ad_check_en_q),
    .root_ppn_i     (cur_ctx.root_ppn),
    .vpn_i          (addr_q[31:12]),
    .busy_o         (ptw_busy),
    .done_o         (ptw_done),
    .fault_o        (ptw_fault),
    .fault_cause_o  (ptw_fault_cause),
    .is_superpage_o (ptw_is_superpage),
    .ppn_o          (ptw_ppn),
    .perm_r_o       (ptw_perm_r),
    .perm_w_o       (ptw_perm_w),
    .flag_a_o       (ptw_flag_a),
    .flag_d_o       (ptw_flag_d),
    .m_axi_ptw_o    (ptw_axi_o),
    .m_axi_ptw_i    (ptw_axi_i)
  );

  // -------------------------------------------------------------------------
  // Fault Recording Queue (FAULT_FIFO, FWFT)
  // -------------------------------------------------------------------------
  logic                   fault_fifo_clr;
  logic                   fault_push;
  fault_rec_t             fault_push_rec;
  logic                   fault_pop;
  fault_rec_t             fault_head_rec;
  logic                   fault_fifo_full;
  logic                   fault_fifo_empty;
  logic [FAULT_CNT_W-1:0] fault_fifo_level;

  sync_fifo #(
    .DATA_W (FAULT_REC_W),
    .DEPTH  (FAULT_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_fault_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fault_fifo_clr),
    .push_i   (fault_push),
    .wdata_i  (fault_push_rec),
    .full_o   (fault_fifo_full),
    .afull_o  (),
    .pop_i    (fault_pop),
    .rdata_o  (fault_head_rec),
    .empty_o  (fault_fifo_empty),
    .aempty_o (),
    .level_o  (fault_fifo_level)
  );

  assign irq_o = (irq_en_q[0] && (irq_stat_q[0] || !fault_fifo_empty)) ||
                 (irq_en_q[1] && irq_stat_q[1]);

  // -------------------------------------------------------------------------
  // CSR Decode & Combinational Signals
  // -------------------------------------------------------------------------
  wire [11:0] csr_off = csr_req_i.addr[11:0];
  wire        csr_wr  = csr_req_i.valid && csr_req_i.write;
  wire        sw_rst_cmd = csr_wr && (csr_off == IOMMU_REG_CTRL) &&
                           csr_req_i.wstrb[0] && csr_req_i.wdata[3];

  assign fault_fifo_clr = sw_rst_cmd;
  assign fault_pop      = csr_wr && (csr_off == IOMMU_REG_FAULT_POP);
  assign tlb_inv_all    = sw_rst_cmd || (csr_wr && (csr_off == IOMMU_REG_TLB_INV_ALL));
  assign tlb_inv_stream = csr_wr && (csr_off == IOMMU_REG_TLB_INV_STREAM);
  assign tlb_inv_iova   = csr_wr && (csr_off == IOMMU_REG_TLB_INV_IOVA);
  assign tlb_inv_sid    = csr_req_i.wdata[3:0];
  assign tlb_inv_vpn    = csr_req_i.wdata[31:12];

  // CSR Read Mux
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;

    if (csr_off >= IOMMU_STREAM_BASE &&
        csr_off < (IOMMU_STREAM_BASE + 12'(NUM_STREAMS * 8))) begin
      automatic logic [3:0] sid;
      sid = csr_off[6:3];
      if (!csr_off[2]) begin
        csr_rsp_o.rdata = {16'd0, stream_ctx_q[sid].asid, 6'd0, stream_ctx_q[sid].mode};
      end else begin
        csr_rsp_o.rdata = {10'd0, stream_ctx_q[sid].root_ppn};
      end
    end else begin
      unique case (csr_off)
        IOMMU_REG_CTRL: begin
          csr_rsp_o.rdata = {28'd0, 1'b0, sid_from_axi_id_q, ad_check_en_q, ctrl_en_q};
        end
        IOMMU_REG_STATUS: begin
          csr_rsp_o.rdata = {
            8'd0,
            8'(TLB_ENTRIES),
            8'(NUM_STREAMS),
            4'(fault_fifo_level),
            fault_ovf_q,
            fault_fifo_full,
            !fault_fifo_empty,
            (state_q != ST_IDLE) || ptw_busy
          };
        end
        IOMMU_REG_IRQ_EN:   csr_rsp_o.rdata = {30'd0, irq_en_q};
        IOMMU_REG_IRQ_STAT: csr_rsp_o.rdata = {30'd0, irq_stat_q};
        IOMMU_REG_CAP_ID:   csr_rsp_o.rdata = IOMMU_ID_VALUE;
        IOMMU_REG_FAULT_IOVA: begin
          csr_rsp_o.rdata = fault_fifo_empty ? 32'd0 : fault_head_rec.iova;
        end
        IOMMU_REG_FAULT_INFO: begin
          if (fault_fifo_empty) begin
            csr_rsp_o.rdata = 32'd0;
          end else begin
            csr_rsp_o.rdata = {
              1'b1,
              7'd0,
              fault_head_rec.asid,
              5'd0,
              fault_head_rec.cause,
              3'd0,
              fault_head_rec.is_write,
              fault_head_rec.stream_id
            };
          end
        end
        IOMMU_REG_PERF_TLB_HIT:  csr_rsp_o.rdata = perf_tlb_hit_q;
        IOMMU_REG_PERF_TLB_MISS: csr_rsp_o.rdata = perf_tlb_miss_q;
        IOMMU_REG_PERF_PTW_CYC:  csr_rsp_o.rdata = perf_ptw_cyc_q;
        IOMMU_REG_PERF_FAULTS:   csr_rsp_o.rdata = perf_faults_q;
        default:                 csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

  // -------------------------------------------------------------------------
  // Combinational AXI4 Data-Path & PTW Muxing
  // -------------------------------------------------------------------------
  wire take_rd = (state_q == ST_IDLE) && s_axi_dev_i.arvalid &&
                 (!s_axi_dev_i.awvalid || rr_rd_prio_q);
  wire take_wr = (state_q == ST_IDLE) && s_axi_dev_i.awvalid && !take_rd;

  // Lookup evaluation helpers
  logic         lkp_is_bypass;
  logic         lkp_is_blocked;
  logic         lkp_tlb_ok;
  logic         lkp_tlb_fault;
  fault_cause_e lkp_tlb_fault_cause;
  logic [31:0]  lkp_tlb_pa;

  always_comb begin
    lkp_is_bypass       = !ctrl_en_q || (cur_ctx.mode == STREAM_MODE_BYPASS);
    lkp_is_blocked      = ctrl_en_q && ((cur_ctx.mode == STREAM_MODE_BLOCK) ||
                                        (cur_ctx.mode == STREAM_MODE_RSVD));
    lkp_tlb_ok          = 1'b0;
    lkp_tlb_fault       = 1'b0;
    lkp_tlb_fault_cause = FAULT_NONE;
    lkp_tlb_pa          = addr_q;

    if (tlb_hit) begin
      lkp_tlb_pa = tlb_hit_entry.is_superpage
                 ? {tlb_hit_entry.ppn[19:10], addr_q[21:0]}
                 : {tlb_hit_entry.ppn[19:0], addr_q[11:0]};

      if (ad_check_en_q && !tlb_hit_entry.flag_a) begin
        lkp_tlb_fault       = 1'b1;
        lkp_tlb_fault_cause = FAULT_PTE_INVALID;
      end else if (!is_write_q && !tlb_hit_entry.perm_r) begin
        lkp_tlb_fault       = 1'b1;
        lkp_tlb_fault_cause = FAULT_PERM_RD;
      end else if (is_write_q && (!tlb_hit_entry.perm_w || (ad_check_en_q && !tlb_hit_entry.flag_d))) begin
        lkp_tlb_fault       = 1'b1;
        lkp_tlb_fault_cause = FAULT_PERM_WR;
      end else begin
        lkp_tlb_ok = 1'b1;
      end
    end
  end

  // PTW start / TLB fill / Fault FIFO push combinational control
  always_comb begin
    ptw_start        = 1'b0;
    tlb_insert_valid = 1'b0;
    tlb_insert_entry = '0;
    fault_push       = 1'b0;
    fault_push_rec   = '0;

    if (state_q == ST_LOOKUP) begin
      if (lkp_is_blocked) begin
        fault_push               = 1'b1;
        fault_push_rec.iova      = addr_q;
        fault_push_rec.stream_id = stream_id_q;
        fault_push_rec.is_write  = is_write_q;
        fault_push_rec.cause     = FAULT_STREAM_BLOCKED;
        fault_push_rec.asid      = cur_ctx.asid;
      end else if (!lkp_is_bypass) begin
        if (tlb_hit) begin
          if (lkp_tlb_fault) begin
            fault_push               = 1'b1;
            fault_push_rec.iova      = addr_q;
            fault_push_rec.stream_id = stream_id_q;
            fault_push_rec.is_write  = is_write_q;
            fault_push_rec.cause     = lkp_tlb_fault_cause;
            fault_push_rec.asid      = cur_ctx.asid;
          end
        end else begin
          ptw_start = 1'b1;
        end
      end
    end else if (state_q == ST_PTW_WAIT) begin
      if (ptw_done) begin
        if (ptw_fault) begin
          fault_push               = 1'b1;
          fault_push_rec.iova      = addr_q;
          fault_push_rec.stream_id = stream_id_q;
          fault_push_rec.is_write  = is_write_q;
          fault_push_rec.cause     = ptw_fault_cause;
          fault_push_rec.asid      = cur_ctx.asid;
        end else begin
          tlb_insert_valid              = 1'b1;
          tlb_insert_entry.valid        = 1'b1;
          tlb_insert_entry.stream_id    = stream_id_q;
          tlb_insert_entry.asid         = cur_ctx.asid;
          tlb_insert_entry.vpn          = addr_q[31:12];
          tlb_insert_entry.is_superpage = ptw_is_superpage;
          tlb_insert_entry.ppn          = ptw_ppn;
          tlb_insert_entry.perm_r       = ptw_perm_r;
          tlb_insert_entry.perm_w       = ptw_perm_w;
          tlb_insert_entry.flag_a       = ptw_flag_a;
          tlb_insert_entry.flag_d       = ptw_flag_d;
        end
      end
    end
  end

  // AXI4 Slave & Master Port Driving
  always_comb begin
    s_axi_dev_o = '0;
    m_axi_mem_o = '0;
    ptw_axi_i   = '0;

    unique case (state_q)
      ST_IDLE: begin
        s_axi_dev_o.arready = take_rd;
        s_axi_dev_o.awready = take_wr;
      end

      ST_PTW_WAIT: begin
        m_axi_mem_o = ptw_axi_o;
        ptw_axi_i   = m_axi_mem_i;
      end

      ST_MEM_WR_AW: begin
        m_axi_mem_o.awid    = id_q;
        m_axi_mem_o.awaddr  = pa_q;
        m_axi_mem_o.awlen   = len_q;
        m_axi_mem_o.awsize  = size_q;
        m_axi_mem_o.awburst = burst_q;
        m_axi_mem_o.awprot  = prot_q;
        m_axi_mem_o.awqos   = qos_q;
        m_axi_mem_o.awvalid = !aw_done_q;

        m_axi_mem_o.wdata   = s_axi_dev_i.wdata;
        m_axi_mem_o.wstrb   = s_axi_dev_i.wstrb;
        m_axi_mem_o.wlast   = s_axi_dev_i.wlast;
        m_axi_mem_o.wvalid  = s_axi_dev_i.wvalid && !w_done_q;
        s_axi_dev_o.wready  = m_axi_mem_i.wready && !w_done_q;
      end

      ST_MEM_WR_B: begin
        m_axi_mem_o.bready = s_axi_dev_i.bready;
        s_axi_dev_o.bvalid = m_axi_mem_i.bvalid;
        s_axi_dev_o.bid    = m_axi_mem_i.bid;
        s_axi_dev_o.bresp  = m_axi_mem_i.bresp;
      end

      ST_MEM_RD_AR: begin
        m_axi_mem_o.arid    = id_q;
        m_axi_mem_o.araddr  = pa_q;
        m_axi_mem_o.arlen   = len_q;
        m_axi_mem_o.arsize  = size_q;
        m_axi_mem_o.arburst = burst_q;
        m_axi_mem_o.arprot  = prot_q;
        m_axi_mem_o.arqos   = qos_q;
        m_axi_mem_o.arvalid = 1'b1;
      end

      ST_MEM_RD_R: begin
        m_axi_mem_o.rready = s_axi_dev_i.rready;
        s_axi_dev_o.rvalid = m_axi_mem_i.rvalid;
        s_axi_dev_o.rid    = m_axi_mem_i.rid;
        s_axi_dev_o.rdata  = m_axi_mem_i.rdata;
        s_axi_dev_o.rresp  = m_axi_mem_i.rresp;
        s_axi_dev_o.rlast  = m_axi_mem_i.rlast;
      end

      ST_FAULT_WR_W: begin
        s_axi_dev_o.wready = 1'b1;
      end

      ST_FAULT_WR_B: begin
        s_axi_dev_o.bvalid = 1'b1;
        s_axi_dev_o.bid    = id_q;
        s_axi_dev_o.bresp  = AXI_RESP_SLVERR;
      end

      ST_FAULT_RD_R: begin
        s_axi_dev_o.rvalid = 1'b1;
        s_axi_dev_o.rid    = id_q;
        s_axi_dev_o.rdata  = 32'd0;
        s_axi_dev_o.rresp  = AXI_RESP_SLVERR;
        s_axi_dev_o.rlast  = (beats_rem_q == 8'd0);
      end

      default: ;
    endcase
  end

  // -------------------------------------------------------------------------
  // Sequential Control, CSR Writes, Counters, and Data-Path FSM
  // -------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_en_q         <= 1'b0;
      ad_check_en_q     <= 1'b1;
      sid_from_axi_id_q <= USE_AXI_ID_AS_STREAM;
      irq_en_q          <= 2'b00;
      irq_stat_q        <= 2'b00;
      fault_ovf_q       <= 1'b0;

      perf_tlb_hit_q    <= 32'd0;
      perf_tlb_miss_q   <= 32'd0;
      perf_ptw_cyc_q    <= 32'd0;
      perf_faults_q     <= 32'd0;

      for (int i = 0; i < int'(NUM_STREAMS); i++) begin
        stream_ctx_q[i].mode     <= STREAM_MODE_BYPASS;
        stream_ctx_q[i].asid     <= 8'd0;
        stream_ctx_q[i].root_ppn <= 22'd0;
      end

      state_q      <= ST_IDLE;
      rr_rd_prio_q <= 1'b0;
      is_write_q   <= 1'b0;
      addr_q       <= 32'd0;
      pa_q         <= 32'd0;
      len_q        <= 8'd0;
      beats_rem_q  <= 8'd0;
      size_q       <= 3'd2;
      burst_q      <= AXI_BURST_INCR;
      id_q         <= 4'd0;
      qos_q        <= 4'd0;
      prot_q       <= 3'd0;
      stream_id_q  <= 4'd0;
      aw_done_q    <= 1'b0;
      w_done_q     <= 1'b0;
    end else begin
      // 1. CSR Writes
      if (csr_wr) begin
        if (csr_off >= IOMMU_STREAM_BASE &&
            csr_off < (IOMMU_STREAM_BASE + 12'(NUM_STREAMS * 8))) begin
          automatic logic [3:0] sid;
          sid = csr_off[6:3];
          if (!csr_off[2]) begin
            if (csr_req_i.wstrb[0]) stream_ctx_q[sid].mode <= stream_mode_e'(csr_req_i.wdata[1:0]);
            if (csr_req_i.wstrb[1]) stream_ctx_q[sid].asid <= csr_req_i.wdata[15:8];
          end else begin
            if (csr_req_i.wstrb[0]) stream_ctx_q[sid].root_ppn[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) stream_ctx_q[sid].root_ppn[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) stream_ctx_q[sid].root_ppn[21:16] <= csr_req_i.wdata[21:16];
          end
        end else begin
          unique case (csr_off)
            IOMMU_REG_CTRL: begin
              if (csr_req_i.wstrb[0]) begin
                ctrl_en_q         <= csr_req_i.wdata[0];
                ad_check_en_q     <= csr_req_i.wdata[1];
                sid_from_axi_id_q <= csr_req_i.wdata[2];
              end
            end
            IOMMU_REG_IRQ_EN: begin
              if (csr_req_i.wstrb[0]) irq_en_q <= csr_req_i.wdata[1:0];
            end
            IOMMU_REG_IRQ_STAT: begin
              if (csr_req_i.wstrb[0]) begin
                if (csr_req_i.wdata[0]) irq_stat_q[0] <= 1'b0;
                if (csr_req_i.wdata[1]) begin
                  irq_stat_q[1] <= 1'b0;
                  fault_ovf_q   <= 1'b0;
                end
              end
            end
            IOMMU_REG_PERF_TLB_HIT:  perf_tlb_hit_q  <= csr_req_i.wdata;
            IOMMU_REG_PERF_TLB_MISS: perf_tlb_miss_q <= csr_req_i.wdata;
            IOMMU_REG_PERF_PTW_CYC:  perf_ptw_cyc_q  <= csr_req_i.wdata;
            IOMMU_REG_PERF_FAULTS:   perf_faults_q   <= csr_req_i.wdata;
            default: ;
          endcase
        end
      end

      // Soft reset clears stats, IRQ status, overflow, and counters
      if (sw_rst_cmd) begin
        irq_stat_q      <= 2'b00;
        fault_ovf_q     <= 1'b0;
        perf_tlb_hit_q  <= 32'd0;
        perf_tlb_miss_q <= 32'd0;
        perf_ptw_cyc_q  <= 32'd0;
        perf_faults_q   <= 32'd0;
      end else if (fault_push) begin
        irq_stat_q[0] <= 1'b1;
        perf_faults_q <= perf_faults_q + 32'd1;
        if (fault_fifo_full) begin
          irq_stat_q[1] <= 1'b1;
          fault_ovf_q   <= 1'b1;
        end
      end

      // 2. Main AXI4 Translation & Data-Path FSM
      unique case (state_q)
        ST_IDLE: begin
          if (take_rd) begin
            is_write_q   <= 1'b0;
            addr_q       <= s_axi_dev_i.araddr;
            len_q        <= s_axi_dev_i.arlen;
            beats_rem_q  <= s_axi_dev_i.arlen;
            size_q       <= s_axi_dev_i.arsize;
            burst_q      <= s_axi_dev_i.arburst;
            id_q         <= s_axi_dev_i.arid;
            qos_q        <= s_axi_dev_i.arqos;
            prot_q       <= s_axi_dev_i.arprot;
            stream_id_q  <= sid_from_axi_id_q ? s_axi_dev_i.arid : ar_stream_id_i;
            rr_rd_prio_q <= 1'b0;
            state_q      <= ST_LOOKUP;
          end else if (take_wr) begin
            is_write_q   <= 1'b1;
            addr_q       <= s_axi_dev_i.awaddr;
            len_q        <= s_axi_dev_i.awlen;
            beats_rem_q  <= s_axi_dev_i.awlen;
            size_q       <= s_axi_dev_i.awsize;
            burst_q      <= s_axi_dev_i.awburst;
            id_q         <= s_axi_dev_i.awid;
            qos_q        <= s_axi_dev_i.awqos;
            prot_q       <= s_axi_dev_i.awprot;
            stream_id_q  <= sid_from_axi_id_q ? s_axi_dev_i.awid : aw_stream_id_i;
            aw_done_q    <= 1'b0;
            w_done_q     <= 1'b0;
            rr_rd_prio_q <= 1'b1;
            state_q      <= ST_LOOKUP;
          end
        end

        ST_LOOKUP: begin
          if (lkp_is_bypass) begin
            pa_q    <= addr_q;
            state_q <= is_write_q ? ST_MEM_WR_AW : ST_MEM_RD_AR;
          end else if (lkp_is_blocked) begin
            state_q <= is_write_q ? ST_FAULT_WR_W : ST_FAULT_RD_R;
          end else if (tlb_hit) begin
            if (!sw_rst_cmd) begin
              perf_tlb_hit_q <= perf_tlb_hit_q + 32'd1;
            end
            if (lkp_tlb_ok) begin
              pa_q    <= lkp_tlb_pa;
              state_q <= is_write_q ? ST_MEM_WR_AW : ST_MEM_RD_AR;
            end else begin
              state_q <= is_write_q ? ST_FAULT_WR_W : ST_FAULT_RD_R;
            end
          end else begin
            if (!sw_rst_cmd) begin
              perf_tlb_miss_q <= perf_tlb_miss_q + 32'd1;
            end
            state_q <= ST_PTW_WAIT;
          end
        end

        ST_PTW_WAIT: begin
          if (!sw_rst_cmd) begin
            perf_ptw_cyc_q <= perf_ptw_cyc_q + 32'd1;
          end
          if (ptw_done) begin
            if (ptw_fault) begin
              state_q <= is_write_q ? ST_FAULT_WR_W : ST_FAULT_RD_R;
            end else begin
              pa_q <= ptw_is_superpage
                    ? {ptw_ppn[19:10], addr_q[21:0]}
                    : {ptw_ppn[19:0], addr_q[11:0]};
              state_q <= is_write_q ? ST_MEM_WR_AW : ST_MEM_RD_AR;
            end
          end
        end

        ST_MEM_WR_AW: begin
          automatic logic aw_hs, w_hs, w_last_beat;
          aw_hs       = m_axi_mem_o.awvalid && m_axi_mem_i.awready;
          w_hs        = m_axi_mem_o.wvalid  && m_axi_mem_i.wready;
          w_last_beat = w_hs && (beats_rem_q == 8'd0);

          if (aw_hs) aw_done_q <= 1'b1;
          if (w_hs && (beats_rem_q != 8'd0)) begin
            beats_rem_q <= beats_rem_q - 8'd1;
          end
          if (w_last_beat) w_done_q <= 1'b1;

          if ((aw_done_q || aw_hs) && (w_done_q || w_last_beat)) begin
            state_q <= ST_MEM_WR_B;
          end
        end

        ST_MEM_WR_B: begin
          if (m_axi_mem_i.bvalid && s_axi_dev_i.bready) begin
            state_q <= ST_IDLE;
          end
        end

        ST_MEM_RD_AR: begin
          if (m_axi_mem_i.arready) begin
            state_q <= ST_MEM_RD_R;
          end
        end

        ST_MEM_RD_R: begin
          if (m_axi_mem_i.rvalid && s_axi_dev_i.rready && m_axi_mem_i.rlast) begin
            state_q <= ST_IDLE;
          end
        end

        ST_FAULT_WR_W: begin
          if (s_axi_dev_i.wvalid) begin
            if (beats_rem_q == 8'd0) begin
              state_q <= ST_FAULT_WR_B;
            end else begin
              beats_rem_q <= beats_rem_q - 8'd1;
            end
          end
        end

        ST_FAULT_WR_B: begin
          if (s_axi_dev_i.bready) begin
            state_q <= ST_IDLE;
          end
        end

        ST_FAULT_RD_R: begin
          if (s_axi_dev_i.rready) begin
            if (beats_rem_q == 8'd0) begin
              state_q <= ST_IDLE;
            end else begin
              beats_rem_q <= beats_rem_q - 8'd1;
            end
          end
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : iommu_core
