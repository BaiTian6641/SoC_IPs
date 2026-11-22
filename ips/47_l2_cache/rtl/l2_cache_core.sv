// SPDX-License-Identifier: Apache-2.0
// l2_cache_core.sv - 4-Way Set-Associative Write-Back AXI4 L2 Cache Controller
//                    with SECDED(39,32) ECC, PLRU, Dirty Victim Write-Back Buffer,
//                    Hardware Maintenance CSRs, Way Locking, and Uncacheable Bypass

module l2_cache_core
  import bus_pkg::*;
  import l2_cache_pkg::*;
#(
  parameter int unsigned WAYS       = 4,
  parameter int unsigned NUM_SETS   = 64,
  parameter int unsigned LINE_BYTES = 32,
  parameter bit          ECC_EN     = 1'b1
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Protocol-Agnostic CSR Control Port
  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // Upstream AXI4 Slave Port
  input  axi4_m2s_t   s_axi_mem_i,
  output axi4_s2m_t   s_axi_mem_o,
  input  logic [3:0]  s_axi_awcache_i,
  input  logic [3:0]  s_axi_arcache_i,

  // Downstream AXI4 Master Port
  output axi4_m2s_t   m_axi_mem_o,
  input  axi4_s2m_t   m_axi_mem_i,
  output logic [3:0]  m_axi_awcache_o,
  output logic [3:0]  m_axi_arcache_o,

  // Interrupt Output
  output logic        irq_o
);

  // ---------------------------------------------------------------------------
  // Derived Geometry Parameters
  // ---------------------------------------------------------------------------
  localparam int unsigned LINE_WORDS  = LINE_BYTES / 4;
  localparam int unsigned WORD_OFF_W  = (LINE_WORDS <= 1) ? 1 : $clog2(LINE_WORDS);
  localparam int unsigned OFFSET_W    = WORD_OFF_W + 2;
  localparam int unsigned SET_W       = (NUM_SETS <= 1) ? 1 : $clog2(NUM_SETS);
  localparam int unsigned TAG_W       = 32 - SET_W - OFFSET_W;
  localparam int unsigned WAY_W       = (WAYS <= 1) ? 1 : $clog2(WAYS);
  localparam int unsigned DATA_ADDR_W = SET_W + WORD_OFF_W;
  localparam int unsigned SRAM_W      = ECC_EN ? 39 : 32;
  localparam int unsigned SRAM_MASK_W = (SRAM_W + 7) / 8;

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] apply_wstrb(
    input logic [31:0] old_val,
    input logic [31:0] new_val,
    input logic [3:0]  wstrb
  );
    logic [31:0] res;
    res = old_val;
    if (wstrb[0]) res[7:0]   = new_val[7:0];
    if (wstrb[1]) res[15:8]  = new_val[15:8];
    if (wstrb[2]) res[23:16] = new_val[23:16];
    if (wstrb[3]) res[31:24] = new_val[31:24];
    return res;
  endfunction

  function automatic logic [WAY_W-1:0] onehot_to_idx(input logic [WAYS-1:0] vec);
    logic [WAY_W-1:0] idx;
    idx = '0;
    for (int w = 0; w < WAYS; w++) begin
      if (vec[w]) idx = WAY_W'(w);
    end
    return idx;
  endfunction

  // ---------------------------------------------------------------------------
  // CSR State Registers
  // ---------------------------------------------------------------------------
  logic                  cfg_cache_en_q;
  logic                  cfg_ecc_chk_en_q;
  logic                  cfg_irq_en_q;
  logic                  cfg_uncache_win_en_q;
  logic [WAYS-1:0]       cfg_way_lock_q;
  l2c_maint_op_e         maint_op_q;
  logic                  maint_pend_q;
  logic [31:0]           maint_addr_q;
  logic [31:0]           cfg_uncache_base_q;
  logic [31:0]           cfg_uncache_mask_q;
  logic [3:0]            irq_status_q;
  logic [3:0]            irq_en_q;
  logic [31:0]           ecc_ce_cnt_q;
  logic [31:0]           ecc_ce_addr_q;
  logic [31:0]           ecc_ue_cnt_q;
  logic [31:0]           ecc_ue_addr_q;
  logic [31:0]           ecc_inj_lo_q;
  logic [6:0]            ecc_inj_hi_q;
  logic                  ecc_inj_target_q;
  logic                  ecc_inj_oneshot_q;
  logic                  ecc_inj_arm_q;
  logic                  perf_en_q;
  logic [31:0]           perf_rd_hit_q;
  logic [31:0]           perf_rd_miss_q;
  logic [31:0]           perf_wr_hit_q;
  logic [31:0]           perf_wr_miss_q;
  logic [31:0]           perf_evicts_q;
  logic [31:0]           perf_bypass_q;

  // ---------------------------------------------------------------------------
  // Cache Metadata Arrays (Packed FFs for clean reset and 1-cycle INV_ALL)
  // ---------------------------------------------------------------------------
  logic [NUM_SETS-1:0][WAYS-1:0] valid_q;
  logic [NUM_SETS-1:0][WAYS-1:0] dirty_q;
  logic [NUM_SETS-1:0][2:0]      plru_q;

  // ---------------------------------------------------------------------------
  // Tag & Data SRAM Instances (tech_sram_sp, clock-only, no async reset)
  // ---------------------------------------------------------------------------
  logic [WAYS-1:0]        tag_en;
  logic [WAYS-1:0]        tag_we;
  logic [SET_W-1:0]       tag_sram_addr;
  logic [31:0]            tag_enc_in;
  logic [6:0]             tag_enc_ecc;
  logic [SRAM_W-1:0]      tag_sram_wdata;
  logic [SRAM_W-1:0]      tag_sram_rdata [WAYS];

  logic [WAYS-1:0]        data_en;
  logic [WAYS-1:0]        data_we;
  logic [DATA_ADDR_W-1:0] data_sram_addr;
  logic [31:0]            data_enc_in;
  logic [6:0]             data_enc_ecc;
  logic [SRAM_W-1:0]      data_sram_wdata;
  logic [SRAM_W-1:0]      data_sram_rdata [WAYS];

  logic [31:0]            tag_dec_data  [WAYS];
  logic [WAYS-1:0]        tag_ce;
  logic [WAYS-1:0]        tag_ue;
  logic [31:0]            data_dec_data [WAYS];
  logic [WAYS-1:0]        data_ce;
  logic [WAYS-1:0]        data_ue;

  logic [WAYS-1:0]        eff_tag_ce;
  logic [WAYS-1:0]        eff_tag_ue;
  logic [WAYS-1:0]        eff_data_ce;
  logic [WAYS-1:0]        eff_data_ue;

  if (ECC_EN) begin : gen_ecc
    l2_cache_ecc_enc u_tag_enc (
      .data_i (tag_enc_in),
      .ecc_o  (tag_enc_ecc)
    );
    l2_cache_ecc_enc u_data_enc (
      .data_i (data_enc_in),
      .ecc_o  (data_enc_ecc)
    );

    always_comb begin
      automatic logic [38:0] inj_mask;
      inj_mask = {ecc_inj_hi_q, ecc_inj_lo_q};
      tag_sram_wdata  = {tag_enc_ecc, tag_enc_in} ^
                        ((ecc_inj_arm_q && ecc_inj_target_q) ? inj_mask : 39'd0);
      data_sram_wdata = {data_enc_ecc, data_enc_in} ^
                        ((ecc_inj_arm_q && !ecc_inj_target_q) ? inj_mask : 39'd0);
    end
  end else begin : gen_no_ecc
    assign tag_enc_ecc     = 7'd0;
    assign data_enc_ecc    = 7'd0;
    assign tag_sram_wdata  = tag_enc_in;
    assign data_sram_wdata = data_enc_in;
  end

  genvar g_w;
  generate
    for (g_w = 0; g_w < WAYS; g_w++) begin : gen_ways
      tech_sram_sp #(
        .DATA_W (SRAM_W),
        .DEPTH  (NUM_SETS),
        .BYTE_W (8)
      ) u_tag_sram (
        .clk_i   (clk_i),
        .en_i    (tag_en[g_w]),
        .we_i    (tag_we[g_w]),
        .wstrb_i ({SRAM_MASK_W{1'b1}}),
        .addr_i  (tag_sram_addr),
        .wdata_i (tag_sram_wdata),
        .rdata_o (tag_sram_rdata[g_w])
      );

      tech_sram_sp #(
        .DATA_W (SRAM_W),
        .DEPTH  (NUM_SETS * LINE_WORDS),
        .BYTE_W (8)
      ) u_data_sram (
        .clk_i   (clk_i),
        .en_i    (data_en[g_w]),
        .we_i    (data_we[g_w]),
        .wstrb_i ({SRAM_MASK_W{1'b1}}),
        .addr_i  (data_sram_addr),
        .wdata_i (data_sram_wdata),
        .rdata_o (data_sram_rdata[g_w])
      );

      if (ECC_EN) begin : gen_way_dec
        l2_cache_ecc_dec u_tag_dec (
          .data_i (tag_sram_rdata[g_w][31:0]),
          .ecc_i  (tag_sram_rdata[g_w][38:32]),
          .data_o (tag_dec_data[g_w]),
          .ce_o   (tag_ce[g_w]),
          .ue_o   (tag_ue[g_w])
        );
        l2_cache_ecc_dec u_data_dec (
          .data_i (data_sram_rdata[g_w][31:0]),
          .ecc_i  (data_sram_rdata[g_w][38:32]),
          .data_o (data_dec_data[g_w]),
          .ce_o   (data_ce[g_w]),
          .ue_o   (data_ue[g_w])
        );
      end else begin : gen_way_no_dec
        assign tag_dec_data[g_w]  = tag_sram_rdata[g_w];
        assign tag_ce[g_w]        = 1'b0;
        assign tag_ue[g_w]        = 1'b0;
        assign data_dec_data[g_w] = data_sram_rdata[g_w];
        assign data_ce[g_w]       = 1'b0;
        assign data_ue[g_w]       = 1'b0;
      end

      assign eff_tag_ce[g_w]  = cfg_ecc_chk_en_q && tag_ce[g_w];
      assign eff_tag_ue[g_w]  = cfg_ecc_chk_en_q && tag_ue[g_w];
      assign eff_data_ce[g_w] = cfg_ecc_chk_en_q && data_ce[g_w];
      assign eff_data_ue[g_w] = cfg_ecc_chk_en_q && data_ue[g_w];
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Dirty Victim Write-Back Buffer & Background AXI4 Write Engine
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    WB_IDLE,
    WB_AW_W,
    WB_W,
    WB_B
  } wb_state_e;

  wb_state_e                    wb_state_q;
  logic [LINE_WORDS-1:0][31:0]  wb_buf_data_q;
  logic [31:0]                  wb_buf_addr_q;
  logic [WORD_OFF_W-1:0]        wb_beat_q;
  logic                         wb_aw_done_q;
  logic                         wb_w_done_q;

  // ---------------------------------------------------------------------------
  // Main Cache & AXI4 Slave/Master FSM
  // ---------------------------------------------------------------------------
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_LOOKUP_REQ,
    ST_LOOKUP_RSP,
    ST_VICTIM_WAIT_WB,
    ST_VICTIM_DRAIN,
    ST_REFILL_AR,
    ST_REFILL_R,
    ST_RD_RESP,
    ST_WR_NEXT_BEAT,
    ST_WR_RESP,
    ST_MISS_BYP_WAIT,
    ST_MISS_BYP_REQ,
    ST_MISS_BYP_RSP,
    ST_WAIT_WB_BYP,
    ST_BYP_WR_AW_W,
    ST_BYP_WR_W,
    ST_BYP_WR_B,
    ST_BYP_RD_AR,
    ST_BYP_RD_R,
    ST_MAINT_PREP,
    ST_MAINT_ADDR_RSP,
    ST_MAINT_WALK,
    ST_MAINT_WALK_TAG,
    ST_MAINT_DRAIN,
    ST_MAINT_WAIT_DONE
  } main_state_e;

  main_state_e           main_state_q;

  // Decoupled AW/W capture registers
  logic                  have_aw_q;
  logic [3:0]            aw_id_q;
  logic [31:0]           aw_addr_q;
  logic [7:0]            aw_len_q;
  logic [2:0]            aw_size_q;
  logic [1:0]            aw_burst_q;
  logic [2:0]            aw_prot_q;
  logic [3:0]            aw_qos_q;
  logic [3:0]            aw_cache_q;

  logic                  have_w_q;
  logic [31:0]           w_data_q;
  logic [3:0]            w_strb_q;
  logic                  w_last_q;
  logic                  rr_prio_q;

  // Active request state
  logic                  is_write_q;
  logic [3:0]            req_id_q;
  logic [31:0]           cur_addr_q;
  logic [7:0]            req_len_q;
  logic [7:0]            beats_rem_q;
  logic [2:0]            req_size_q;
  logic [1:0]            req_burst_q;
  logic [2:0]            req_prot_q;
  logic [3:0]            req_qos_q;
  logic [3:0]            req_cache_q;
  logic [31:0]           cur_wdata_q;
  logic [3:0]            cur_wstrb_q;
  logic                  cur_wlast_q;

  logic [31:0]           rd_data_q;
  logic                  beat_err_q;
  logic                  err_acc_q;

  logic [WAY_W-1:0]      victim_way_q;
  logic [31:0]           wb_pending_addr_q;
  logic [WORD_OFF_W:0]   drain_idx_q;
  logic [WAY_W-1:0]      drain_way_q;
  logic [SET_W-1:0]      drain_set_q;

  logic [WORD_OFF_W-1:0] refill_beat_q;
  logic                  refill_err_q;

  logic                  byp_aw_done_q;
  logic                  byp_w_done_q;

  // Maintenance tracking
  logic [SET_W-1:0]      maint_set_q;
  logic [WAY_W-1:0]      maint_way_q;
  logic                  maint_from_walk_q;

  // Combinational event signals from FSMs to CSR/Stats/WB Engine
  logic                  wb_start;
  logic [31:0]           wb_start_addr;
  logic                  wb_buf_wr_en;
  logic [WORD_OFF_W-1:0] wb_buf_wr_idx;
  logic [31:0]           wb_buf_wr_data;

  logic                  ev_ecc_ce;
  logic [31:0]           ev_ecc_ce_addr;
  logic                  ev_ecc_ue;
  logic [31:0]           ev_ecc_ue_addr;
  logic                  ev_axi_err;
  logic                  ev_maint_done;
  logic                  ev_rd_hit;
  logic                  ev_rd_miss;
  logic                  ev_wr_hit;
  logic                  ev_wr_miss;
  logic                  ev_evict;
  logic                  ev_bypass;
  logic                  ev_inj_tag_used;
  logic                  ev_inj_data_used;

  // Current address decomposition
  wire [TAG_W-1:0]       cur_tag  = cur_addr_q[31 : OFFSET_W + SET_W];
  wire [SET_W-1:0]       cur_set  = cur_addr_q[OFFSET_W +: SET_W];
  wire [WORD_OFF_W-1:0]  cur_word = cur_addr_q[2 +: WORD_OFF_W];

  wire [TAG_W-1:0]       m_tag    = maint_addr_q[31 : OFFSET_W + SET_W];
  wire [SET_W-1:0]       m_set    = maint_addr_q[OFFSET_W +: SET_W];

  function automatic logic is_addr_uncacheable(
    input logic [31:0] addr,
    input logic [3:0]  axcache
  );
    logic win_hit;
    win_hit = cfg_uncache_win_en_q && (cfg_uncache_mask_q != 32'd0) &&
              ((addr & cfg_uncache_mask_q) == (cfg_uncache_base_q & cfg_uncache_mask_q));
    return !cfg_cache_en_q || !axcache[1] || win_hit;
  endfunction

  // ---------------------------------------------------------------------------
  // CSR Read/Write Combinational Interface
  // ---------------------------------------------------------------------------
  logic csr_wr_valid;
  logic csr_addr_err;

  always_comb begin
    csr_rsp_o    = '0;
    csr_rsp_o.ready = 1'b1;
    csr_wr_valid = csr_req_i.valid && csr_req_i.write;
    csr_addr_err = 1'b0;

    if (csr_req_i.valid) begin
      if (csr_req_i.addr[1:0] != 2'b00) begin
        csr_addr_err = 1'b1;
      end else begin
        unique case (csr_req_i.addr[7:0])
          L2C_REG_CTRL: begin
            csr_rsp_o.rdata = {28'd0, cfg_uncache_win_en_q, cfg_irq_en_q,
                               (ECC_EN ? cfg_ecc_chk_en_q : 1'b0), cfg_cache_en_q};
          end
          L2C_REG_STATUS: begin
            csr_rsp_o.rdata = {
              8'(SET_W),
              8'(LINE_BYTES),
              8'(WAYS),
              3'd0,
              ECC_EN,
              (main_state_q == ST_IDLE) && !maint_pend_q && (wb_state_q == WB_IDLE),
              (wb_state_q != WB_IDLE),
              maint_pend_q,
              cfg_cache_en_q
            };
          end
          L2C_REG_CACHE_INFO: begin
            csr_rsp_o.rdata = {8'(LINE_BYTES), 8'(WAYS), 16'(NUM_SETS)};
          end
          L2C_REG_WAY_LOCK: begin
            csr_rsp_o.rdata = {{(32-WAYS){1'b0}}, cfg_way_lock_q};
          end
          L2C_REG_MAINT_CMD: begin
            csr_rsp_o.rdata = {maint_pend_q, 28'd0, maint_op_q};
          end
          L2C_REG_MAINT_ADDR:   csr_rsp_o.rdata = maint_addr_q;
          L2C_REG_UNCACHE_BASE: csr_rsp_o.rdata = cfg_uncache_base_q;
          L2C_REG_UNCACHE_MASK: csr_rsp_o.rdata = cfg_uncache_mask_q;
          L2C_REG_IRQ_STATUS:   csr_rsp_o.rdata = {28'd0, irq_status_q};
          L2C_REG_IRQ_EN:       csr_rsp_o.rdata = {28'd0, irq_en_q};
          L2C_REG_ECC_CE_CNT:   csr_rsp_o.rdata = ecc_ce_cnt_q;
          L2C_REG_ECC_CE_ADDR:  csr_rsp_o.rdata = ecc_ce_addr_q;
          L2C_REG_ECC_UE_CNT:   csr_rsp_o.rdata = ecc_ue_cnt_q;
          L2C_REG_ECC_UE_ADDR:  csr_rsp_o.rdata = ecc_ue_addr_q;
          L2C_REG_ECC_INJ_LO:   csr_rsp_o.rdata = ecc_inj_lo_q;
          L2C_REG_ECC_INJ_HI: begin
            csr_rsp_o.rdata = {22'd0, ecc_inj_arm_q, ecc_inj_oneshot_q,
                               ecc_inj_target_q, ecc_inj_hi_q};
          end
          L2C_REG_PERF_CTRL:    csr_rsp_o.rdata = {31'd0, perf_en_q};
          L2C_REG_PERF_RD_HIT:  csr_rsp_o.rdata = perf_rd_hit_q;
          L2C_REG_PERF_RD_MISS: csr_rsp_o.rdata = perf_rd_miss_q;
          L2C_REG_PERF_WR_HIT:  csr_rsp_o.rdata = perf_wr_hit_q;
          L2C_REG_PERF_WR_MISS: csr_rsp_o.rdata = perf_wr_miss_q;
          L2C_REG_PERF_EVICTS:  csr_rsp_o.rdata = perf_evicts_q;
          L2C_REG_PERF_BYPASS:  csr_rsp_o.rdata = perf_bypass_q;
          L2C_REG_ID_VER:       csr_rsp_o.rdata = L2C_ID_VER_VAL;
          default:              csr_addr_err    = 1'b1;
        endcase
      end
      csr_rsp_o.err = csr_addr_err;
    end
  end

  assign irq_o = cfg_irq_en_q && (|(irq_status_q & irq_en_q));

  // ---------------------------------------------------------------------------
  // Main Datapath & AXI4 Combinational Logic
  // ---------------------------------------------------------------------------
  logic [WAYS-1:0]       lookup_hit_vec;
  logic                  lookup_is_hit;
  logic [WAY_W-1:0]      lookup_hit_way;
  logic                  lookup_any_tag_ue;
  logic [WAYS-1:0]       lookup_tag_ce_valid;
  logic [WAY_W-1:0]      lookup_victim_way;

  logic [WAYS-1:0]       maint_hit_vec;
  logic                  maint_is_hit;
  logic [WAY_W-1:0]      maint_hit_way;
  logic                  maint_any_tag_ue;
  logic [WAYS-1:0]       maint_tag_ce_valid;

  always_comb begin
    // Lookup hit/ECC evaluation
    lookup_hit_vec      = '0;
    lookup_tag_ce_valid = '0;
    lookup_any_tag_ue   = 1'b0;
    for (int w = 0; w < WAYS; w++) begin
      if (valid_q[cur_set][w]) begin
        if (eff_tag_ue[w]) begin
          lookup_any_tag_ue = 1'b1;
        end else begin
          if (eff_tag_ce[w]) begin
            lookup_tag_ce_valid[w] = 1'b1;
          end
          if (tag_dec_data[w][TAG_W-1:0] == cur_tag) begin
            lookup_hit_vec[w] = 1'b1;
          end
        end
      end
    end
    lookup_is_hit     = |lookup_hit_vec;
    lookup_hit_way    = onehot_to_idx(lookup_hit_vec);
    lookup_victim_way = plru_select_victim(plru_q[cur_set], valid_q[cur_set], cfg_way_lock_q);

    // Maintenance address lookup evaluation
    maint_hit_vec      = '0;
    maint_tag_ce_valid = '0;
    maint_any_tag_ue   = 1'b0;
    for (int w = 0; w < WAYS; w++) begin
      if (valid_q[m_set][w]) begin
        if (eff_tag_ue[w]) begin
          maint_any_tag_ue = 1'b1;
        end else begin
          if (eff_tag_ce[w]) begin
            maint_tag_ce_valid[w] = 1'b1;
          end
          if (tag_dec_data[w][TAG_W-1:0] == m_tag) begin
            maint_hit_vec[w] = 1'b1;
          end
        end
      end
    end
    maint_is_hit  = |maint_hit_vec;
    maint_hit_way = onehot_to_idx(maint_hit_vec);
  end

  // Uncacheable window + global enable + AxCACHE[1] check for incoming requests
  wire s_rd_can_go = !maint_pend_q && !have_aw_q && !have_w_q && s_axi_mem_i.arvalid &&
                     (!(s_axi_mem_i.awvalid || s_axi_mem_i.wvalid) || rr_prio_q);

  wire wb_busy = (wb_state_q != WB_IDLE);
  wire refill_line_hazard = wb_busy &&
                            (wb_buf_addr_q[31:OFFSET_W] == cur_addr_q[31:OFFSET_W]);

  always_comb begin
    // Defaults
    s_axi_mem_o     = '0;
    m_axi_mem_o     = '0;
    m_axi_awcache_o = 4'b0010;
    m_axi_arcache_o = 4'b0010;

    tag_en          = '0;
    tag_we          = '0;
    tag_sram_addr   = cur_set;
    tag_enc_in      = '0;

    data_en         = '0;
    data_we         = '0;
    data_sram_addr  = {cur_set, cur_word};
    data_enc_in     = '0;

    wb_start        = 1'b0;
    wb_start_addr   = wb_pending_addr_q;
    wb_buf_wr_en    = 1'b0;
    wb_buf_wr_idx   = '0;
    wb_buf_wr_data  = '0;

    ev_ecc_ce        = 1'b0;
    ev_ecc_ce_addr   = cur_addr_q;
    ev_ecc_ue        = 1'b0;
    ev_ecc_ue_addr   = cur_addr_q;
    ev_axi_err       = 1'b0;
    ev_maint_done    = 1'b0;
    ev_rd_hit        = 1'b0;
    ev_rd_miss       = 1'b0;
    ev_wr_hit        = 1'b0;
    ev_wr_miss       = 1'b0;
    ev_evict         = 1'b0;
    ev_bypass        = 1'b0;
    ev_inj_tag_used  = 1'b0;
    ev_inj_data_used = 1'b0;

    // -------------------------------------------------------------------------
    // Background Dirty Victim Write-Back AXI4 Write Engine (AW/W/B)
    // -------------------------------------------------------------------------
    unique case (wb_state_q)
      WB_IDLE: ;
      WB_AW_W: begin
        m_axi_mem_o.awid    = 4'd1;
        m_axi_mem_o.awaddr  = wb_buf_addr_q;
        m_axi_mem_o.awlen   = 8'(LINE_WORDS - 1);
        m_axi_mem_o.awsize  = 3'd2;
        m_axi_mem_o.awburst = AXI_BURST_INCR;
        m_axi_mem_o.awvalid = !wb_aw_done_q;
        m_axi_awcache_o     = 4'b0011;

        m_axi_mem_o.wdata   = wb_buf_data_q[0];
        m_axi_mem_o.wstrb   = 4'hF;
        m_axi_mem_o.wlast   = (LINE_WORDS == 1);
        m_axi_mem_o.wvalid  = !wb_w_done_q;
      end
      WB_W: begin
        m_axi_mem_o.wdata  = wb_buf_data_q[wb_beat_q];
        m_axi_mem_o.wstrb  = 4'hF;
        m_axi_mem_o.wlast  = (wb_beat_q == WORD_OFF_W'(LINE_WORDS - 1));
        m_axi_mem_o.wvalid = 1'b1;
      end
      WB_B: begin
        m_axi_mem_o.bready = 1'b1;
        if (m_axi_mem_i.bvalid &&
            (m_axi_mem_i.bresp != AXI_RESP_OKAY) &&
            (m_axi_mem_i.bresp != AXI_RESP_EXOKAY)) begin
          ev_axi_err = 1'b1;
        end
      end
      default: ;
    endcase

    // -------------------------------------------------------------------------
    // Main Cache / Passthrough / Maintenance FSM
    // -------------------------------------------------------------------------
    unique case (main_state_q)
      ST_IDLE: begin
        if (!maint_pend_q) begin
          if (s_rd_can_go) begin
            s_axi_mem_o.arready = 1'b1;
          end else begin
            s_axi_mem_o.awready = !have_aw_q;
            s_axi_mem_o.wready  = !have_w_q;
          end
        end
      end

      ST_LOOKUP_REQ: begin
        tag_en         = {WAYS{1'b1}};
        tag_we         = '0;
        tag_sram_addr  = cur_set;
        data_en        = {WAYS{1'b1}};
        data_we        = '0;
        data_sram_addr = {cur_set, cur_word};
      end

      ST_LOOKUP_RSP: begin
        if (lookup_any_tag_ue) begin
          ev_ecc_ue      = 1'b1;
          ev_ecc_ue_addr = cur_addr_q;
        end else if (lookup_is_hit) begin
          // Scrub tag if 1-bit CE occurred on hitting way
          if (lookup_tag_ce_valid[lookup_hit_way]) begin
            ev_ecc_ce             = 1'b1;
            ev_ecc_ce_addr        = cur_addr_q;
            tag_en[lookup_hit_way] = 1'b1;
            tag_we[lookup_hit_way] = 1'b1;
            tag_sram_addr          = cur_set;
            tag_enc_in             = {{(32-TAG_W){1'b0}}, cur_tag};
            ev_inj_tag_used        = 1'b1;
          end

          if (!is_write_q) begin
            ev_rd_hit = 1'b1;
            if (eff_data_ue[lookup_hit_way]) begin
              ev_ecc_ue      = 1'b1;
              ev_ecc_ue_addr = cur_addr_q;
            end else if (eff_data_ce[lookup_hit_way]) begin
              ev_ecc_ce               = 1'b1;
              ev_ecc_ce_addr          = cur_addr_q;
              data_en[lookup_hit_way] = 1'b1;
              data_we[lookup_hit_way] = 1'b1;
              data_sram_addr          = {cur_set, cur_word};
              data_enc_in             = data_dec_data[lookup_hit_way];
              ev_inj_data_used        = 1'b1;
            end
          end else begin
            ev_wr_hit = 1'b1;
            if ((cur_wstrb_q != 4'hF) && eff_data_ue[lookup_hit_way]) begin
              ev_ecc_ue      = 1'b1;
              ev_ecc_ue_addr = cur_addr_q;
            end else begin
              if ((cur_wstrb_q != 4'hF) && eff_data_ce[lookup_hit_way]) begin
                ev_ecc_ce      = 1'b1;
                ev_ecc_ce_addr = cur_addr_q;
              end
              data_en[lookup_hit_way] = 1'b1;
              data_we[lookup_hit_way] = 1'b1;
              data_sram_addr          = {cur_set, cur_word};
              data_enc_in             = apply_wstrb(data_dec_data[lookup_hit_way],
                                                    cur_wdata_q, cur_wstrb_q);
              ev_inj_data_used        = 1'b1;
            end
          end
        end else begin
          // Cache Miss
          if (is_write_q) ev_wr_miss = 1'b1;
          else            ev_rd_miss = 1'b1;

          if (cfg_way_lock_q != {WAYS{1'b1}}) begin
            if (valid_q[cur_set][lookup_victim_way]) begin
              ev_evict = 1'b1;
              if (eff_tag_ce[lookup_victim_way]) begin
                ev_ecc_ce      = 1'b1;
                ev_ecc_ce_addr = {tag_dec_data[lookup_victim_way][TAG_W-1:0], cur_set, {OFFSET_W{1'b0}}};
              end
            end
          end
        end
      end

      ST_VICTIM_WAIT_WB: ;

      ST_VICTIM_DRAIN: begin
        if (drain_idx_q < (WORD_OFF_W + 1)'(LINE_WORDS)) begin
          data_en[drain_way_q] = 1'b1;
          data_we[drain_way_q] = 1'b0;
          data_sram_addr       = {drain_set_q, drain_idx_q[WORD_OFF_W-1:0]};
        end
        if (drain_idx_q > '0) begin
          automatic logic [WORD_OFF_W:0]   cap_idx_ext;
          automatic logic [WORD_OFF_W-1:0] cap_idx;
          cap_idx_ext   = drain_idx_q - (WORD_OFF_W + 1)'(1);
          cap_idx       = cap_idx_ext[WORD_OFF_W-1:0];
          wb_buf_wr_en  = 1'b1;
          wb_buf_wr_idx = cap_idx;
          wb_buf_wr_data = data_dec_data[drain_way_q];
          if (eff_data_ue[drain_way_q]) begin
            ev_ecc_ue      = 1'b1;
            ev_ecc_ue_addr = {wb_pending_addr_q[31:OFFSET_W], cap_idx, 2'b00};
          end else if (eff_data_ce[drain_way_q]) begin
            ev_ecc_ce      = 1'b1;
            ev_ecc_ce_addr = {wb_pending_addr_q[31:OFFSET_W], cap_idx, 2'b00};
          end
        end
        if (drain_idx_q == (WORD_OFF_W + 1)'(LINE_WORDS)) begin
          wb_start      = 1'b1;
          wb_start_addr = wb_pending_addr_q;
        end
      end

      ST_REFILL_AR: begin
        if (!refill_line_hazard) begin
          m_axi_mem_o.arid    = 4'd0;
          m_axi_mem_o.araddr  = {cur_tag, cur_set, {OFFSET_W{1'b0}}};
          m_axi_mem_o.arlen   = 8'(LINE_WORDS - 1);
          m_axi_mem_o.arsize  = 3'd2;
          m_axi_mem_o.arburst = AXI_BURST_INCR;
          m_axi_mem_o.arvalid = 1'b1;
          m_axi_arcache_o     = 4'b0010;
        end
      end

      ST_REFILL_R: begin
        m_axi_mem_o.rready = 1'b1;
        if (m_axi_mem_i.rvalid) begin
          automatic logic err_now;
          automatic logic [31:0] store_word;
          err_now = (m_axi_mem_i.rresp != AXI_RESP_OKAY) &&
                    (m_axi_mem_i.rresp != AXI_RESP_EXOKAY);
          if (err_now) ev_axi_err = 1'b1;

          store_word = (is_write_q && (refill_beat_q == cur_word))
                       ? apply_wstrb(m_axi_mem_i.rdata, cur_wdata_q, cur_wstrb_q)
                       : m_axi_mem_i.rdata;

          data_en[victim_way_q] = 1'b1;
          data_we[victim_way_q] = 1'b1;
          data_sram_addr        = {cur_set, refill_beat_q};
          data_enc_in           = store_word;
          ev_inj_data_used      = 1'b1;

          if ((m_axi_mem_i.rlast || (refill_beat_q == WORD_OFF_W'(LINE_WORDS - 1))) &&
              !(refill_err_q || err_now)) begin
            tag_en[victim_way_q] = 1'b1;
            tag_we[victim_way_q] = 1'b1;
            tag_sram_addr        = cur_set;
            tag_enc_in           = {{(32-TAG_W){1'b0}}, cur_tag};
            ev_inj_tag_used      = 1'b1;
          end
        end
      end

      ST_RD_RESP: begin
        s_axi_mem_o.rvalid = 1'b1;
        s_axi_mem_o.rid    = req_id_q;
        s_axi_mem_o.rdata  = rd_data_q;
        s_axi_mem_o.rresp  = beat_err_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
        s_axi_mem_o.rlast  = (beats_rem_q == 8'd0);
      end

      ST_WR_NEXT_BEAT: begin
        if (beats_rem_q != 8'd0) begin
          s_axi_mem_o.wready = 1'b1;
        end
      end

      ST_WR_RESP: begin
        s_axi_mem_o.bvalid = 1'b1;
        s_axi_mem_o.bid    = req_id_q;
        s_axi_mem_o.bresp  = err_acc_q ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
      end

      // Single-beat fallback when all 4 ways are locked and a beat misses
      ST_MISS_BYP_WAIT: ;

      ST_MISS_BYP_REQ: begin
        if (is_write_q) begin
          m_axi_mem_o.awid    = req_id_q;
          m_axi_mem_o.awaddr  = cur_addr_q;
          m_axi_mem_o.awlen   = 8'd0;
          m_axi_mem_o.awsize  = req_size_q;
          m_axi_mem_o.awburst = AXI_BURST_INCR;
          m_axi_mem_o.awprot  = req_prot_q;
          m_axi_mem_o.awqos   = req_qos_q;
          m_axi_mem_o.awvalid = !byp_aw_done_q;
          m_axi_awcache_o     = 4'b0000;

          m_axi_mem_o.wdata   = cur_wdata_q;
          m_axi_mem_o.wstrb   = cur_wstrb_q;
          m_axi_mem_o.wlast   = 1'b1;
          m_axi_mem_o.wvalid  = !byp_w_done_q;
        end else begin
          m_axi_mem_o.arid    = req_id_q;
          m_axi_mem_o.araddr  = cur_addr_q;
          m_axi_mem_o.arlen   = 8'd0;
          m_axi_mem_o.arsize  = req_size_q;
          m_axi_mem_o.arburst = AXI_BURST_INCR;
          m_axi_mem_o.arprot  = req_prot_q;
          m_axi_mem_o.arqos   = req_qos_q;
          m_axi_mem_o.arvalid = 1'b1;
          m_axi_arcache_o     = 4'b0000;
        end
      end

      ST_MISS_BYP_RSP: begin
        if (is_write_q) begin
          m_axi_mem_o.bready = 1'b1;
          if (m_axi_mem_i.bvalid &&
              (m_axi_mem_i.bresp != AXI_RESP_OKAY) &&
              (m_axi_mem_i.bresp != AXI_RESP_EXOKAY)) begin
            ev_axi_err = 1'b1;
          end
        end else begin
          m_axi_mem_o.rready = 1'b1;
          if (m_axi_mem_i.rvalid &&
              (m_axi_mem_i.rresp != AXI_RESP_OKAY) &&
              (m_axi_mem_i.rresp != AXI_RESP_EXOKAY)) begin
            ev_axi_err = 1'b1;
          end
        end
      end

      // Full-transaction Uncacheable Passthrough
      ST_WAIT_WB_BYP: begin
        if (!wb_busy) begin
          ev_bypass = 1'b1;
        end
      end

      ST_BYP_WR_AW_W: begin
        m_axi_mem_o.awid    = req_id_q;
        m_axi_mem_o.awaddr  = cur_addr_q;
        m_axi_mem_o.awlen   = req_len_q;
        m_axi_mem_o.awsize  = req_size_q;
        m_axi_mem_o.awburst = req_burst_q;
        m_axi_mem_o.awprot  = req_prot_q;
        m_axi_mem_o.awqos   = req_qos_q;
        m_axi_mem_o.awvalid = !byp_aw_done_q;
        m_axi_awcache_o     = req_cache_q;

        m_axi_mem_o.wdata   = cur_wdata_q;
        m_axi_mem_o.wstrb   = cur_wstrb_q;
        m_axi_mem_o.wlast   = cur_wlast_q;
        m_axi_mem_o.wvalid  = !byp_w_done_q;
      end

      ST_BYP_WR_W: begin
        m_axi_mem_o.wvalid = s_axi_mem_i.wvalid;
        m_axi_mem_o.wdata  = s_axi_mem_i.wdata;
        m_axi_mem_o.wstrb  = s_axi_mem_i.wstrb;
        m_axi_mem_o.wlast  = s_axi_mem_i.wlast;
        s_axi_mem_o.wready = m_axi_mem_i.wready;
      end

      ST_BYP_WR_B: begin
        s_axi_mem_o.bvalid = m_axi_mem_i.bvalid;
        s_axi_mem_o.bid    = m_axi_mem_i.bid;
        s_axi_mem_o.bresp  = m_axi_mem_i.bresp;
        m_axi_mem_o.bready = s_axi_mem_i.bready;
        if (m_axi_mem_i.bvalid && s_axi_mem_i.bready &&
            (m_axi_mem_i.bresp != AXI_RESP_OKAY) &&
            (m_axi_mem_i.bresp != AXI_RESP_EXOKAY)) begin
          ev_axi_err = 1'b1;
        end
      end

      ST_BYP_RD_AR: begin
        m_axi_mem_o.arid    = req_id_q;
        m_axi_mem_o.araddr  = cur_addr_q;
        m_axi_mem_o.arlen   = req_len_q;
        m_axi_mem_o.arsize  = req_size_q;
        m_axi_mem_o.arburst = req_burst_q;
        m_axi_mem_o.arprot  = req_prot_q;
        m_axi_mem_o.arqos   = req_qos_q;
        m_axi_mem_o.arvalid = 1'b1;
        m_axi_arcache_o     = req_cache_q;
      end

      ST_BYP_RD_R: begin
        s_axi_mem_o.rvalid = m_axi_mem_i.rvalid;
        s_axi_mem_o.rid    = m_axi_mem_i.rid;
        s_axi_mem_o.rdata  = m_axi_mem_i.rdata;
        s_axi_mem_o.rresp  = m_axi_mem_i.rresp;
        s_axi_mem_o.rlast  = m_axi_mem_i.rlast;
        m_axi_mem_o.rready = s_axi_mem_i.rready;
        if (m_axi_mem_i.rvalid && s_axi_mem_i.rready &&
            (m_axi_mem_i.rresp != AXI_RESP_OKAY) &&
            (m_axi_mem_i.rresp != AXI_RESP_EXOKAY)) begin
          ev_axi_err = 1'b1;
        end
      end

      // Hardware Cache Maintenance FSM
      ST_MAINT_PREP: begin
        if ((maint_op_q == MAINT_OP_CLEAN_ADDR) ||
            (maint_op_q == MAINT_OP_INV_ADDR) ||
            (maint_op_q == MAINT_OP_CLEAN_INV_ADDR)) begin
          if (!wb_busy) begin
            tag_en        = {WAYS{1'b1}};
            tag_we        = '0;
            tag_sram_addr = m_set;
          end
        end
      end

      ST_MAINT_ADDR_RSP: begin
        if (maint_any_tag_ue) begin
          ev_ecc_ue      = 1'b1;
          ev_ecc_ue_addr = maint_addr_q;
        end else if (|maint_tag_ce_valid) begin
          ev_ecc_ce      = 1'b1;
          ev_ecc_ce_addr = maint_addr_q;
        end
      end

      ST_MAINT_WALK: begin
        if (valid_q[maint_set_q][maint_way_q] && dirty_q[maint_set_q][maint_way_q] && !wb_busy) begin
          tag_en[maint_way_q] = 1'b1;
          tag_we[maint_way_q] = 1'b0;
          tag_sram_addr       = maint_set_q;
        end
      end

      ST_MAINT_WALK_TAG: begin
        if (eff_tag_ue[maint_way_q]) begin
          ev_ecc_ue      = 1'b1;
          ev_ecc_ue_addr = { {(TAG_W){1'b0}}, maint_set_q, {OFFSET_W{1'b0}} };
        end else if (eff_tag_ce[maint_way_q]) begin
          ev_ecc_ce      = 1'b1;
          ev_ecc_ce_addr = { tag_dec_data[maint_way_q][TAG_W-1:0], maint_set_q, {OFFSET_W{1'b0}} };
        end
      end

      ST_MAINT_DRAIN: begin
        if (drain_idx_q < (WORD_OFF_W + 1)'(LINE_WORDS)) begin
          data_en[drain_way_q] = 1'b1;
          data_we[drain_way_q] = 1'b0;
          data_sram_addr       = {drain_set_q, drain_idx_q[WORD_OFF_W-1:0]};
        end
        if (drain_idx_q > '0) begin
          automatic logic [WORD_OFF_W:0]   cap_idx_ext;
          automatic logic [WORD_OFF_W-1:0] cap_idx;
          cap_idx_ext    = drain_idx_q - (WORD_OFF_W + 1)'(1);
          cap_idx        = cap_idx_ext[WORD_OFF_W-1:0];
          wb_buf_wr_en   = 1'b1;
          wb_buf_wr_idx  = cap_idx;
          wb_buf_wr_data = data_dec_data[drain_way_q];
          if (eff_data_ue[drain_way_q]) begin
            ev_ecc_ue      = 1'b1;
            ev_ecc_ue_addr = {wb_pending_addr_q[31:OFFSET_W], cap_idx, 2'b00};
          end else if (eff_data_ce[drain_way_q]) begin
            ev_ecc_ce      = 1'b1;
            ev_ecc_ce_addr = {wb_pending_addr_q[31:OFFSET_W], cap_idx, 2'b00};
          end
        end
        if (drain_idx_q == (WORD_OFF_W + 1)'(LINE_WORDS)) begin
          wb_start      = 1'b1;
          wb_start_addr = wb_pending_addr_q;
        end
      end

      ST_MAINT_WAIT_DONE: begin
        if (!wb_busy) begin
          ev_maint_done = 1'b1;
        end
      end

      default: ;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Sequential State Updates
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // CSR Reset State
      cfg_cache_en_q       <= 1'b0;
      cfg_ecc_chk_en_q     <= ECC_EN;
      cfg_irq_en_q         <= 1'b1;
      cfg_uncache_win_en_q <= 1'b1;
      cfg_way_lock_q       <= '0;
      maint_op_q           <= MAINT_OP_NOP;
      maint_pend_q         <= 1'b0;
      maint_addr_q         <= 32'd0;
      cfg_uncache_base_q   <= 32'd0;
      cfg_uncache_mask_q   <= 32'd0;
      irq_status_q         <= 4'd0;
      irq_en_q             <= 4'd0;
      ecc_ce_cnt_q         <= 32'd0;
      ecc_ce_addr_q        <= 32'd0;
      ecc_ue_cnt_q         <= 32'd0;
      ecc_ue_addr_q        <= 32'd0;
      ecc_inj_lo_q         <= 32'd0;
      ecc_inj_hi_q         <= 7'd0;
      ecc_inj_target_q     <= 1'b0;
      ecc_inj_oneshot_q    <= 1'b1;
      ecc_inj_arm_q        <= 1'b0;
      perf_en_q            <= 1'b1;
      perf_rd_hit_q        <= 32'd0;
      perf_rd_miss_q       <= 32'd0;
      perf_wr_hit_q        <= 32'd0;
      perf_wr_miss_q       <= 32'd0;
      perf_evicts_q        <= 32'd0;
      perf_bypass_q        <= 32'd0;

      // Cache Metadata Arrays
      valid_q              <= '0;
      dirty_q              <= '0;
      plru_q               <= '0;

      // Write-Back Engine
      wb_state_q           <= WB_IDLE;
      wb_buf_data_q        <= '0;
      wb_buf_addr_q        <= 32'd0;
      wb_beat_q            <= '0;
      wb_aw_done_q         <= 1'b0;
      wb_w_done_q          <= 1'b0;

      // Main FSM & AXI4 Slave Staging
      main_state_q         <= ST_IDLE;
      have_aw_q            <= 1'b0;
      aw_id_q              <= 4'd0;
      aw_addr_q            <= 32'd0;
      aw_len_q             <= 8'd0;
      aw_size_q            <= 3'd2;
      aw_burst_q           <= AXI_BURST_INCR;
      aw_prot_q            <= 3'd0;
      aw_qos_q             <= 4'd0;
      aw_cache_q           <= 4'b0010;
      have_w_q             <= 1'b0;
      w_data_q             <= 32'd0;
      w_strb_q             <= 4'd0;
      w_last_q             <= 1'b0;
      rr_prio_q            <= 1'b0;

      is_write_q           <= 1'b0;
      req_id_q             <= 4'd0;
      cur_addr_q           <= 32'd0;
      req_len_q            <= 8'd0;
      beats_rem_q          <= 8'd0;
      req_size_q           <= 3'd2;
      req_burst_q          <= AXI_BURST_INCR;
      req_prot_q           <= 3'd0;
      req_qos_q            <= 4'd0;
      req_cache_q          <= 4'b0010;
      cur_wdata_q          <= 32'd0;
      cur_wstrb_q          <= 4'd0;
      cur_wlast_q          <= 1'b0;
      rd_data_q            <= 32'd0;
      beat_err_q           <= 1'b0;
      err_acc_q            <= 1'b0;
      victim_way_q         <= '0;
      wb_pending_addr_q    <= 32'd0;
      drain_idx_q          <= '0;
      drain_way_q          <= '0;
      drain_set_q          <= '0;
      refill_beat_q        <= '0;
      refill_err_q         <= 1'b0;
      byp_aw_done_q        <= 1'b0;
      byp_w_done_q         <= 1'b0;
      maint_set_q          <= '0;
      maint_way_q          <= '0;
      maint_from_walk_q    <= 1'b0;
    end else begin
      // -----------------------------------------------------------------------
      // CSR Writes
      // -----------------------------------------------------------------------
      automatic logic perf_clr_pulse;
      automatic logic ce_cnt_clr_pulse;
      automatic logic ue_cnt_clr_pulse;
      automatic logic [3:0] irq_w1c_mask;
      automatic logic [3:0] irq_set_vec;

      perf_clr_pulse   = 1'b0;
      ce_cnt_clr_pulse = 1'b0;
      ue_cnt_clr_pulse = 1'b0;
      irq_w1c_mask     = 4'd0;

      if (csr_wr_valid && !csr_addr_err) begin
        unique case (csr_req_i.addr[7:0])
          L2C_REG_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              cfg_cache_en_q       <= csr_req_i.wdata[L2C_CTRL_CACHE_EN_BIT];
              cfg_ecc_chk_en_q     <= ECC_EN ? csr_req_i.wdata[L2C_CTRL_ECC_CHK_EN_BIT] : 1'b0;
              cfg_irq_en_q         <= csr_req_i.wdata[L2C_CTRL_IRQ_EN_BIT];
              cfg_uncache_win_en_q <= csr_req_i.wdata[L2C_CTRL_UNCACHE_WIN_EN_BIT];
            end
          end
          L2C_REG_WAY_LOCK: begin
            if (csr_req_i.wstrb[0]) begin
              cfg_way_lock_q <= csr_req_i.wdata[WAYS-1:0];
            end
          end
          L2C_REG_MAINT_CMD: begin
            if (!maint_pend_q) begin
              if (csr_req_i.wstrb[0]) begin
                maint_op_q <= l2c_maint_op_e'(csr_req_i.wdata[2:0]);
              end
              if (csr_req_i.wstrb[3] && csr_req_i.wdata[31]) begin
                maint_pend_q <= 1'b1;
              end
            end
          end
          L2C_REG_MAINT_ADDR: begin
            maint_addr_q <= apply_wstrb(maint_addr_q, csr_req_i.wdata, csr_req_i.wstrb);
          end
          L2C_REG_UNCACHE_BASE: begin
            cfg_uncache_base_q <= apply_wstrb(cfg_uncache_base_q, csr_req_i.wdata, csr_req_i.wstrb);
          end
          L2C_REG_UNCACHE_MASK: begin
            cfg_uncache_mask_q <= apply_wstrb(cfg_uncache_mask_q, csr_req_i.wdata, csr_req_i.wstrb);
          end
          L2C_REG_IRQ_STATUS: begin
            if (csr_req_i.wstrb[0]) irq_w1c_mask = csr_req_i.wdata[3:0];
          end
          L2C_REG_IRQ_EN: begin
            if (csr_req_i.wstrb[0]) irq_en_q <= csr_req_i.wdata[3:0];
          end
          L2C_REG_ECC_CE_CNT: ce_cnt_clr_pulse = 1'b1;
          L2C_REG_ECC_UE_CNT: ue_cnt_clr_pulse = 1'b1;
          L2C_REG_ECC_INJ_LO: begin
            ecc_inj_lo_q <= apply_wstrb(ecc_inj_lo_q, csr_req_i.wdata, csr_req_i.wstrb);
          end
          L2C_REG_ECC_INJ_HI: begin
            if (csr_req_i.wstrb[0]) begin
              ecc_inj_hi_q     <= csr_req_i.wdata[6:0];
              ecc_inj_target_q <= csr_req_i.wdata[L2C_INJ_HI_TARGET_BIT];
            end
            if (csr_req_i.wstrb[1]) begin
              ecc_inj_oneshot_q <= csr_req_i.wdata[L2C_INJ_HI_ONESHOT_BIT];
              ecc_inj_arm_q     <= csr_req_i.wdata[L2C_INJ_HI_ARM_BIT];
            end
          end
          L2C_REG_PERF_CTRL: begin
            if (csr_req_i.wstrb[0]) begin
              perf_en_q      <= csr_req_i.wdata[0];
              perf_clr_pulse  = csr_req_i.wdata[1];
            end
          end
          default: ;
        endcase
      end

      // Auto-disarm one-shot DFT ECC fault injection
      if (ecc_inj_arm_q && ecc_inj_oneshot_q &&
          ((ecc_inj_target_q && ev_inj_tag_used) ||
           (!ecc_inj_target_q && ev_inj_data_used))) begin
        ecc_inj_arm_q <= 1'b0;
      end

      // IRQ Status (W1C + HW set)
      irq_set_vec = {ev_axi_err, ev_maint_done, ev_ecc_ue, ev_ecc_ce};
      irq_status_q <= (irq_status_q & ~irq_w1c_mask) | irq_set_vec;

      // ECC Telemetry Counters & Latched Fault Addresses
      if (ce_cnt_clr_pulse) begin
        ecc_ce_cnt_q <= 32'd0;
      end else if (ev_ecc_ce) begin
        ecc_ce_cnt_q  <= ecc_ce_cnt_q + 32'd1;
        ecc_ce_addr_q <= ev_ecc_ce_addr;
      end

      if (ue_cnt_clr_pulse) begin
        ecc_ue_cnt_q <= 32'd0;
      end else if (ev_ecc_ue) begin
        ecc_ue_cnt_q  <= ecc_ue_cnt_q + 32'd1;
        ecc_ue_addr_q <= ev_ecc_ue_addr;
      end

      // Performance Counters
      if (perf_clr_pulse) begin
        perf_rd_hit_q  <= 32'd0;
        perf_rd_miss_q <= 32'd0;
        perf_wr_hit_q  <= 32'd0;
        perf_wr_miss_q <= 32'd0;
        perf_evicts_q  <= 32'd0;
        perf_bypass_q  <= 32'd0;
      end else if (perf_en_q) begin
        if (ev_rd_hit)  perf_rd_hit_q  <= perf_rd_hit_q  + 32'd1;
        if (ev_rd_miss) perf_rd_miss_q <= perf_rd_miss_q + 32'd1;
        if (ev_wr_hit)  perf_wr_hit_q  <= perf_wr_hit_q  + 32'd1;
        if (ev_wr_miss) perf_wr_miss_q <= perf_wr_miss_q + 32'd1;
        if (ev_evict)   perf_evicts_q  <= perf_evicts_q  + 32'd1;
        if (ev_bypass)  perf_bypass_q  <= perf_bypass_q  + 32'd1;
      end

      // -----------------------------------------------------------------------
      // Victim Write-Back Buffer Load & Background AXI4 Write FSM
      // -----------------------------------------------------------------------
      if (wb_buf_wr_en) begin
        wb_buf_data_q[wb_buf_wr_idx] <= wb_buf_wr_data;
      end

      unique case (wb_state_q)
        WB_IDLE: begin
          if (wb_start) begin
            wb_buf_addr_q <= wb_start_addr;
            wb_beat_q     <= '0;
            wb_aw_done_q  <= 1'b0;
            wb_w_done_q   <= 1'b0;
            wb_state_q    <= WB_AW_W;
          end
        end
        WB_AW_W: begin
          automatic logic aw_ok;
          automatic logic w_ok;
          aw_ok = wb_aw_done_q || m_axi_mem_i.awready;
          w_ok  = wb_w_done_q  || m_axi_mem_i.wready;
          wb_aw_done_q <= aw_ok;
          wb_w_done_q  <= w_ok;
          if (aw_ok && w_ok) begin
            wb_beat_q  <= WORD_OFF_W'(1);
            wb_state_q <= (LINE_WORDS == 1) ? WB_B : WB_W;
          end
        end
        WB_W: begin
          if (m_axi_mem_i.wready) begin
            if (wb_beat_q == WORD_OFF_W'(LINE_WORDS - 1)) begin
              wb_state_q <= WB_B;
            end else begin
              wb_beat_q <= wb_beat_q + WORD_OFF_W'(1);
            end
          end
        end
        WB_B: begin
          if (m_axi_mem_i.bvalid) begin
            wb_state_q <= WB_IDLE;
          end
        end
        default: wb_state_q <= WB_IDLE;
      endcase

      // -----------------------------------------------------------------------
      // Main Cache / Passthrough / Maintenance FSM
      // -----------------------------------------------------------------------
      unique case (main_state_q)
        ST_IDLE: begin
          if (maint_pend_q) begin
            main_state_q <= ST_MAINT_PREP;
          end else if (s_rd_can_go) begin
            rr_prio_q   <= 1'b0;
            is_write_q  <= 1'b0;
            req_id_q    <= s_axi_mem_i.arid;
            cur_addr_q  <= s_axi_mem_i.araddr;
            req_len_q   <= s_axi_mem_i.arlen;
            beats_rem_q <= s_axi_mem_i.arlen;
            req_size_q  <= s_axi_mem_i.arsize;
            req_burst_q <= s_axi_mem_i.arburst;
            req_prot_q  <= s_axi_mem_i.arprot;
            req_qos_q   <= s_axi_mem_i.arqos;
            req_cache_q <= s_axi_arcache_i;
            err_acc_q   <= 1'b0;
            beat_err_q  <= 1'b0;
            if (is_addr_uncacheable(s_axi_mem_i.araddr, s_axi_arcache_i)) begin
              main_state_q <= ST_WAIT_WB_BYP;
            end else begin
              main_state_q <= ST_LOOKUP_REQ;
            end
          end else begin
            automatic logic aw_hs;
            automatic logic w_hs;
            automatic logic aw_now;
            automatic logic w_now;

            aw_hs  = s_axi_mem_i.awvalid && s_axi_mem_o.awready;
            w_hs   = s_axi_mem_i.wvalid  && s_axi_mem_o.wready;
            aw_now = have_aw_q || aw_hs;
            w_now  = have_w_q  || w_hs;

            if (aw_hs) begin
              aw_id_q    <= s_axi_mem_i.awid;
              aw_addr_q  <= s_axi_mem_i.awaddr;
              aw_len_q   <= s_axi_mem_i.awlen;
              aw_size_q  <= s_axi_mem_i.awsize;
              aw_burst_q <= s_axi_mem_i.awburst;
              aw_prot_q  <= s_axi_mem_i.awprot;
              aw_qos_q   <= s_axi_mem_i.awqos;
              aw_cache_q <= s_axi_awcache_i;
            end
            if (w_hs) begin
              w_data_q <= s_axi_mem_i.wdata;
              w_strb_q <= s_axi_mem_i.wstrb;
              w_last_q <= s_axi_mem_i.wlast;
            end

            if (aw_now && w_now) begin
              automatic logic [31:0] eff_addr;
              automatic logic [3:0]  eff_cache;
              eff_addr  = aw_hs ? s_axi_mem_i.awaddr : aw_addr_q;
              eff_cache = aw_hs ? s_axi_awcache_i    : aw_cache_q;

              have_aw_q    <= 1'b0;
              have_w_q     <= 1'b0;
              rr_prio_q    <= 1'b1;
              is_write_q   <= 1'b1;
              req_id_q     <= aw_hs ? s_axi_mem_i.awid    : aw_id_q;
              cur_addr_q   <= eff_addr;
              req_len_q    <= aw_hs ? s_axi_mem_i.awlen   : aw_len_q;
              beats_rem_q  <= aw_hs ? s_axi_mem_i.awlen   : aw_len_q;
              req_size_q   <= aw_hs ? s_axi_mem_i.awsize  : aw_size_q;
              req_burst_q  <= aw_hs ? s_axi_mem_i.awburst : aw_burst_q;
              req_prot_q   <= aw_hs ? s_axi_mem_i.awprot  : aw_prot_q;
              req_qos_q    <= aw_hs ? s_axi_mem_i.awqos   : aw_qos_q;
              req_cache_q  <= eff_cache;
              cur_wdata_q  <= w_hs  ? s_axi_mem_i.wdata   : w_data_q;
              cur_wstrb_q  <= w_hs  ? s_axi_mem_i.wstrb   : w_strb_q;
              cur_wlast_q  <= w_hs  ? s_axi_mem_i.wlast   : w_last_q;
              err_acc_q    <= 1'b0;
              beat_err_q   <= 1'b0;
              byp_aw_done_q <= 1'b0;
              byp_w_done_q  <= 1'b0;

              if (is_addr_uncacheable(eff_addr, eff_cache)) begin
                main_state_q <= ST_WAIT_WB_BYP;
              end else begin
                main_state_q <= ST_LOOKUP_REQ;
              end
            end else begin
              have_aw_q <= aw_now;
              have_w_q  <= w_now;
            end
          end
        end

        ST_LOOKUP_REQ: begin
          main_state_q <= ST_LOOKUP_RSP;
        end

        ST_LOOKUP_RSP: begin
          if (lookup_any_tag_ue) begin
            for (int w = 0; w < WAYS; w++) begin
              if (valid_q[cur_set][w] && eff_tag_ue[w]) begin
                valid_q[cur_set][w] <= 1'b0;
                dirty_q[cur_set][w] <= 1'b0;
              end
            end
            beat_err_q   <= 1'b1;
            err_acc_q    <= 1'b1;
            main_state_q <= is_write_q ? ST_WR_NEXT_BEAT : ST_RD_RESP;
          end else if (lookup_is_hit) begin
            plru_q[cur_set] <= plru_update(plru_q[cur_set], 2'(lookup_hit_way));
            if (!is_write_q) begin
              if (eff_data_ue[lookup_hit_way]) begin
                rd_data_q  <= 32'd0;
                beat_err_q <= 1'b1;
                err_acc_q  <= 1'b1;
              end else begin
                rd_data_q  <= data_dec_data[lookup_hit_way];
                beat_err_q <= 1'b0;
              end
              main_state_q <= ST_RD_RESP;
            end else begin
              if ((cur_wstrb_q != 4'hF) && eff_data_ue[lookup_hit_way]) begin
                beat_err_q <= 1'b1;
                err_acc_q  <= 1'b1;
              end else begin
                dirty_q[cur_set][lookup_hit_way] <= 1'b1;
                beat_err_q <= 1'b0;
              end
              main_state_q <= ST_WR_NEXT_BEAT;
            end
          end else begin
            // Cache Miss
            if (cfg_way_lock_q == {WAYS{1'b1}}) begin
              byp_aw_done_q <= 1'b0;
              byp_w_done_q  <= 1'b0;
              main_state_q  <= ST_MISS_BYP_WAIT;
            end else begin
              victim_way_q <= lookup_victim_way;
              if (valid_q[cur_set][lookup_victim_way] && dirty_q[cur_set][lookup_victim_way]) begin
                wb_pending_addr_q <= {tag_dec_data[lookup_victim_way][TAG_W-1:0],
                                      cur_set, {OFFSET_W{1'b0}}};
                valid_q[cur_set][lookup_victim_way] <= 1'b0;
                dirty_q[cur_set][lookup_victim_way] <= 1'b0;
                drain_way_q  <= lookup_victim_way;
                drain_set_q  <= cur_set;
                drain_idx_q  <= '0;
                main_state_q <= wb_busy ? ST_VICTIM_WAIT_WB : ST_VICTIM_DRAIN;
              end else begin
                valid_q[cur_set][lookup_victim_way] <= 1'b0;
                dirty_q[cur_set][lookup_victim_way] <= 1'b0;
                main_state_q <= ST_REFILL_AR;
              end
            end
          end
        end

        ST_VICTIM_WAIT_WB: begin
          if (!wb_busy) begin
            drain_idx_q  <= '0;
            main_state_q <= ST_VICTIM_DRAIN;
          end
        end

        ST_VICTIM_DRAIN: begin
          if (drain_idx_q == (WORD_OFF_W + 1)'(LINE_WORDS)) begin
            main_state_q <= ST_REFILL_AR;
          end else begin
            drain_idx_q <= drain_idx_q + (WORD_OFF_W + 1)'(1);
          end
        end

        ST_REFILL_AR: begin
          if (!refill_line_hazard && m_axi_mem_i.arready) begin
            refill_beat_q <= '0;
            refill_err_q  <= 1'b0;
            main_state_q  <= ST_REFILL_R;
          end
        end

        ST_REFILL_R: begin
          if (m_axi_mem_i.rvalid) begin
            automatic logic err_now;
            automatic logic err_total;
            err_now   = (m_axi_mem_i.rresp != AXI_RESP_OKAY) &&
                        (m_axi_mem_i.rresp != AXI_RESP_EXOKAY);
            err_total = refill_err_q || err_now;
            refill_err_q <= err_total;

            if (!is_write_q && (refill_beat_q == cur_word)) begin
              rd_data_q <= m_axi_mem_i.rdata;
            end

            if (m_axi_mem_i.rlast || (refill_beat_q == WORD_OFF_W'(LINE_WORDS - 1))) begin
              if (!err_total) begin
                valid_q[cur_set][victim_way_q] <= 1'b1;
                dirty_q[cur_set][victim_way_q] <= is_write_q;
                plru_q[cur_set] <= plru_update(plru_q[cur_set], 2'(victim_way_q));
              end else begin
                err_acc_q <= 1'b1;
              end
              beat_err_q   <= err_total;
              main_state_q <= is_write_q ? ST_WR_NEXT_BEAT : ST_RD_RESP;
            end else begin
              refill_beat_q <= refill_beat_q + WORD_OFF_W'(1);
            end
          end
        end

        ST_RD_RESP: begin
          if (s_axi_mem_i.rready) begin
            if (beats_rem_q == 8'd0) begin
              main_state_q <= ST_IDLE;
            end else begin
              beats_rem_q  <= beats_rem_q - 8'd1;
              cur_addr_q   <= axi_next_addr(cur_addr_q, req_size_q, req_burst_q, req_len_q);
              main_state_q <= ST_LOOKUP_REQ;
            end
          end
        end

        ST_WR_NEXT_BEAT: begin
          if (beats_rem_q == 8'd0) begin
            main_state_q <= ST_WR_RESP;
          end else if (s_axi_mem_i.wvalid) begin
            cur_wdata_q  <= s_axi_mem_i.wdata;
            cur_wstrb_q  <= s_axi_mem_i.wstrb;
            cur_wlast_q  <= s_axi_mem_i.wlast;
            beats_rem_q  <= beats_rem_q - 8'd1;
            cur_addr_q   <= axi_next_addr(cur_addr_q, req_size_q, req_burst_q, req_len_q);
            main_state_q <= ST_LOOKUP_REQ;
          end
        end

        ST_WR_RESP: begin
          if (s_axi_mem_i.bready) begin
            main_state_q <= ST_IDLE;
          end
        end

        ST_MISS_BYP_WAIT: begin
          if (!wb_busy) begin
            main_state_q <= ST_MISS_BYP_REQ;
          end
        end

        ST_MISS_BYP_REQ: begin
          if (is_write_q) begin
            automatic logic aw_ok;
            automatic logic w_ok;
            aw_ok = byp_aw_done_q || m_axi_mem_i.awready;
            w_ok  = byp_w_done_q  || m_axi_mem_i.wready;
            byp_aw_done_q <= aw_ok;
            byp_w_done_q  <= w_ok;
            if (aw_ok && w_ok) begin
              main_state_q <= ST_MISS_BYP_RSP;
            end
          end else if (m_axi_mem_i.arready) begin
            main_state_q <= ST_MISS_BYP_RSP;
          end
        end

        ST_MISS_BYP_RSP: begin
          if (is_write_q) begin
            if (m_axi_mem_i.bvalid) begin
              if ((m_axi_mem_i.bresp != AXI_RESP_OKAY) &&
                  (m_axi_mem_i.bresp != AXI_RESP_EXOKAY)) begin
                err_acc_q <= 1'b1;
              end
              main_state_q <= ST_WR_NEXT_BEAT;
            end
          end else begin
            if (m_axi_mem_i.rvalid) begin
              rd_data_q <= m_axi_mem_i.rdata;
              beat_err_q <= (m_axi_mem_i.rresp != AXI_RESP_OKAY) &&
                            (m_axi_mem_i.rresp != AXI_RESP_EXOKAY);
              main_state_q <= ST_RD_RESP;
            end
          end
        end

        ST_WAIT_WB_BYP: begin
          if (!wb_busy) begin
            byp_aw_done_q <= 1'b0;
            byp_w_done_q  <= 1'b0;
            main_state_q  <= is_write_q ? ST_BYP_WR_AW_W : ST_BYP_RD_AR;
          end
        end

        ST_BYP_WR_AW_W: begin
          automatic logic aw_ok;
          automatic logic w_ok;
          aw_ok = byp_aw_done_q || m_axi_mem_i.awready;
          w_ok  = byp_w_done_q  || m_axi_mem_i.wready;
          byp_aw_done_q <= aw_ok;
          byp_w_done_q  <= w_ok;
          if (aw_ok && w_ok) begin
            if ((req_len_q == 8'd0) || cur_wlast_q) begin
              main_state_q <= ST_BYP_WR_B;
            end else begin
              main_state_q <= ST_BYP_WR_W;
            end
          end
        end

        ST_BYP_WR_W: begin
          if (s_axi_mem_i.wvalid && m_axi_mem_i.wready && s_axi_mem_i.wlast) begin
            main_state_q <= ST_BYP_WR_B;
          end
        end

        ST_BYP_WR_B: begin
          if (m_axi_mem_i.bvalid && s_axi_mem_i.bready) begin
            main_state_q <= ST_IDLE;
          end
        end

        ST_BYP_RD_AR: begin
          if (m_axi_mem_i.arready) begin
            main_state_q <= ST_BYP_RD_R;
          end
        end

        ST_BYP_RD_R: begin
          if (m_axi_mem_i.rvalid && s_axi_mem_i.rready && m_axi_mem_i.rlast) begin
            main_state_q <= ST_IDLE;
          end
        end

        // ---------------------------------------------------------------------
        // Hardware Cache Maintenance FSM
        // ---------------------------------------------------------------------
        ST_MAINT_PREP: begin
          if (maint_op_q == MAINT_OP_INV_ALL) begin
            if (!wb_busy) begin
              valid_q      <= '0;
              dirty_q      <= '0;
              main_state_q <= ST_MAINT_WAIT_DONE;
            end
          end else if ((maint_op_q == MAINT_OP_CLEAN_ALL) ||
                       (maint_op_q == MAINT_OP_CLEAN_INV_ALL)) begin
            maint_set_q  <= '0;
            maint_way_q  <= '0;
            main_state_q <= ST_MAINT_WALK;
          end else if ((maint_op_q == MAINT_OP_CLEAN_ADDR) ||
                       (maint_op_q == MAINT_OP_INV_ADDR) ||
                       (maint_op_q == MAINT_OP_CLEAN_INV_ADDR)) begin
            if (!wb_busy) begin
              main_state_q <= ST_MAINT_ADDR_RSP;
            end
          end else begin
            main_state_q <= ST_MAINT_WAIT_DONE;
          end
        end

        ST_MAINT_ADDR_RSP: begin
          if (maint_any_tag_ue) begin
            for (int w = 0; w < WAYS; w++) begin
              if (valid_q[m_set][w] && eff_tag_ue[w]) begin
                valid_q[m_set][w] <= 1'b0;
                dirty_q[m_set][w] <= 1'b0;
              end
            end
            main_state_q <= ST_MAINT_WAIT_DONE;
          end else if (maint_is_hit) begin
            if (maint_op_q == MAINT_OP_INV_ADDR) begin
              valid_q[m_set][maint_hit_way] <= 1'b0;
              dirty_q[m_set][maint_hit_way] <= 1'b0;
              main_state_q <= ST_MAINT_WAIT_DONE;
            end else if (dirty_q[m_set][maint_hit_way]) begin
              dirty_q[m_set][maint_hit_way] <= 1'b0;
              if (maint_op_q == MAINT_OP_CLEAN_INV_ADDR) begin
                valid_q[m_set][maint_hit_way] <= 1'b0;
              end
              wb_pending_addr_q <= {m_tag, m_set, {OFFSET_W{1'b0}}};
              drain_way_q       <= maint_hit_way;
              drain_set_q       <= m_set;
              drain_idx_q       <= '0;
              maint_from_walk_q <= 1'b0;
              main_state_q      <= ST_MAINT_DRAIN;
            end else begin
              if (maint_op_q == MAINT_OP_CLEAN_INV_ADDR) begin
                valid_q[m_set][maint_hit_way] <= 1'b0;
              end
              main_state_q <= ST_MAINT_WAIT_DONE;
            end
          end else begin
            main_state_q <= ST_MAINT_WAIT_DONE;
          end
        end

        ST_MAINT_WALK: begin
          if (valid_q[maint_set_q][maint_way_q] && dirty_q[maint_set_q][maint_way_q]) begin
            if (!wb_busy) begin
              main_state_q <= ST_MAINT_WALK_TAG;
            end
          end else begin
            if (maint_op_q == MAINT_OP_CLEAN_INV_ALL) begin
              valid_q[maint_set_q][maint_way_q] <= 1'b0;
            end
            if ((maint_set_q == SET_W'(NUM_SETS - 1)) && (maint_way_q == WAY_W'(WAYS - 1))) begin
              main_state_q <= ST_MAINT_WAIT_DONE;
            end else if (maint_way_q == WAY_W'(WAYS - 1)) begin
              maint_way_q <= '0;
              maint_set_q <= maint_set_q + SET_W'(1);
            end else begin
              maint_way_q <= maint_way_q + WAY_W'(1);
            end
          end
        end

        ST_MAINT_WALK_TAG: begin
          dirty_q[maint_set_q][maint_way_q] <= 1'b0;
          if (maint_op_q == MAINT_OP_CLEAN_INV_ALL) begin
            valid_q[maint_set_q][maint_way_q] <= 1'b0;
          end

          if (eff_tag_ue[maint_way_q]) begin
            valid_q[maint_set_q][maint_way_q] <= 1'b0;
            if ((maint_set_q == SET_W'(NUM_SETS - 1)) && (maint_way_q == WAY_W'(WAYS - 1))) begin
              main_state_q <= ST_MAINT_WAIT_DONE;
            end else if (maint_way_q == WAY_W'(WAYS - 1)) begin
              maint_way_q  <= '0;
              maint_set_q  <= maint_set_q + SET_W'(1);
              main_state_q <= ST_MAINT_WALK;
            end else begin
              maint_way_q  <= maint_way_q + WAY_W'(1);
              main_state_q <= ST_MAINT_WALK;
            end
          end else begin
            wb_pending_addr_q <= {tag_dec_data[maint_way_q][TAG_W-1:0],
                                  maint_set_q, {OFFSET_W{1'b0}}};
            drain_way_q       <= maint_way_q;
            drain_set_q       <= maint_set_q;
            drain_idx_q       <= '0;
            maint_from_walk_q <= 1'b1;
            main_state_q      <= ST_MAINT_DRAIN;
          end
        end

        ST_MAINT_DRAIN: begin
          if (drain_idx_q == (WORD_OFF_W + 1)'(LINE_WORDS)) begin
            if (maint_from_walk_q) begin
              if ((maint_set_q == SET_W'(NUM_SETS - 1)) && (maint_way_q == WAY_W'(WAYS - 1))) begin
                main_state_q <= ST_MAINT_WAIT_DONE;
              end else if (maint_way_q == WAY_W'(WAYS - 1)) begin
                maint_way_q  <= '0;
                maint_set_q  <= maint_set_q + SET_W'(1);
                main_state_q <= ST_MAINT_WALK;
              end else begin
                maint_way_q  <= maint_way_q + WAY_W'(1);
                main_state_q <= ST_MAINT_WALK;
              end
            end else begin
              main_state_q <= ST_MAINT_WAIT_DONE;
            end
          end else begin
            drain_idx_q <= drain_idx_q + (WORD_OFF_W + 1)'(1);
          end
        end

        ST_MAINT_WAIT_DONE: begin
          if (!wb_busy) begin
            maint_pend_q <= 1'b0;
            main_state_q <= ST_IDLE;
          end
        end

        default: main_state_q <= ST_IDLE;
      endcase
    end
  end

endmodule : l2_cache_core
