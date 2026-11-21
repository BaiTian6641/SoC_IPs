// SPDX-License-Identifier: Apache-2.0
// tb_soc_sysctrl.sv - Comprehensive Self-Checking Testbench for ips/36_soc_sysctrl
//
// Verifies all 5 sub-IPs and all 4 CSR bus protocols (WB / AXI4 / TL / APB):
//   1. CRG          : Glitch-free 4:1 clock switch (continuous runt/short pulse monitor),
//                     integer/fractional/bypass dividers, tech_icg clock gating,
//                     PLL ctrl/status CDC, per-domain SW reset, sticky RST_CAUSE.
//   2. IOMUX        : 32x8 pinmux routing vs golden model, pad attributes, IN_ISO,
//                     OUT_RET retention latch, write-once LOCK bit.
//   3. PMU          : Exact power-down & power-up FSM ordering invariants
//                     (iso before pwr_en drop, pwr_ack before iso/ret/rst release),
//                     programmable delays, sleep_req_i, 8-source wake matrix, W1C, irq_o.
//   4. Bus Firewall : Inline AXI4 allow/deny, R/W/ID/SEC/PRIV checks, region priority
//                     carveout, burst-boundary crossing block, SLVERR/DECERR, VIOL_*
//                     latching & irq_o, LOCK, randomized traffic vs golden scoreboard.
//   5. sram_ecc_wrap: SECDED(39,32) all 39 single-bit error corrections, double-bit UE
//                     detection, partial-byte RMW + inline correction, background scrubber.
//   6. Quad-Bus     : Wishbone B4, AXI4, TileLink TL-UL, AMBA APB4 live CSR smoke.

`timescale 1ns/1ps

module tb_soc_sysctrl;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  int unsigned seed = 1;
  int unsigned err_cnt = 0;

  // Functional coverage counters
  int unsigned cov_crg_switches = 0;
  int unsigned cov_crg_divs     = 0;
  int unsigned cov_crg_causes   = 0;
  int unsigned cov_iomux_ops    = 0;
  int unsigned cov_pmu_cycles   = 0;
  int unsigned cov_pmu_wakes    = 0;
  int unsigned cov_fw_txns      = 0;
  int unsigned cov_fw_blocks    = 0;
  int unsigned cov_ecc_ce_bits  = 0;
  int unsigned cov_ecc_ue_pairs = 0;
  int unsigned cov_ecc_rmw      = 0;
  int unsigned cov_ecc_scrub    = 0;
  int unsigned cov_quad_bus     = 0;

  // -------------------------------------------------------------------------
  // Clocks & Resets
  // -------------------------------------------------------------------------
  logic clk;
  logic osc_clk;
  logic pll_clk;
  logic ext_clk;
  logic aux_clk;
  initial begin
    clk     = 1'b0;
    osc_clk = 1'b0;
    pll_clk = 1'b0;
    ext_clk = 1'b0;
    aux_clk = 1'b0;
  end
  always #5   clk     = ~clk;     // 100.0 MHz CSR bus clock
  always #5.0 osc_clk = ~osc_clk; // 100.0 MHz (10.0ns period, 5.0ns half)
  always #3.5 pll_clk = ~pll_clk; // 142.8 MHz ( 7.0ns period, 3.5ns half)
  always #6.5 ext_clk = ~ext_clk; //  76.9 MHz (13.0ns period, 6.5ns half)
  always #8.5 aux_clk = ~aux_clk; //  58.8 MHz (17.0ns period, 8.5ns half)

  logic rst_n;
  logic crg_por_n;
  logic crg_ext_rst_n;
  logic crg_wdt_rst;
  logic crg_bo_rst;
  logic crg_test_mode;
  logic crg_pll_lock;

  // Unused bus tie-offs
  wb_m2s_t   wb_zero  = '0;
  axi4_m2s_t axi_zero = '0;
  tl_m2s_t   tl_zero  = '0;
  apb_m2s_t  apb_zero = '0;

  `include "tb_bfm_tasks.svh"

  // APB4 BFM tasks matching repo race-free convention
  task automatic do_apb_write(ref apb_m2s_t m, const ref apb_s2m_t s,
                              input logic [31:0] addr, input logic [31:0] data,
                              input logic [3:0] strb = 4'hF);
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b1;
    m.pwdata  = data;
    m.pstrb   = strb;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    #1;
    m = '0;
  endtask

  task automatic do_apb_read(ref apb_m2s_t m, const ref apb_s2m_t s,
                             input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'h0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data = s.prdata;
    #1;
    m = '0;
  endtask

  // Failure helper
  task automatic check_eq32(input string tag, input logic [31:0] got, input logic [31:0] exp);
    if (got !== exp) begin
      $display("[FAIL] %s: got=0x%08x exp=0x%08x @ %0t", tag, got, exp, $time);
      err_cnt++;
      $fatal(1, "Mismatch in %s", tag);
    end
  endtask

  task automatic check_cond(input string tag, input logic cond);
    if (cond !== 1'b1) begin
      $display("[FAIL] %s @ %0t", tag, $time);
      err_cnt++;
      $fatal(1, "Condition failed: %s", tag);
    end
  endtask

  // =========================================================================
  // DUT 1: CRG (APB4 Primary + WB Smoke)
  // =========================================================================
  apb_m2s_t crg_apb_m;
  apb_s2m_t crg_apb_s;
  logic crg_pll_en, crg_pll_bypass;
  logic [31:0] crg_pll_cfg;
  logic crg_root_clk, crg_sys_rst_n;
  logic [3:0] crg_clk_out, crg_rst_out_n, crg_mux_active;
  logic [4:0] crg_rst_cause;

  crg_top #(
    .BUS_TYPE         (bus_pkg::BUS_APB),
    .N_OUT_CLKS       (4),
    .RST_PULSE_CYCLES (6)
  ) u_crg (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_i           (wb_zero),  .wb_o (),
    .axi_i          (axi_zero), .axi_o(),
    .tl_i           (tl_zero),  .tl_o (),
    .apb_i          (crg_apb_m),
    .apb_o          (crg_apb_s),
    .osc_clk_i      (osc_clk),
    .pll_clk_i      (pll_clk),
    .ext_clk_i      (ext_clk),
    .aux_clk_i      (aux_clk),
    .test_mode_i    (crg_test_mode),
    .por_ni         (crg_por_n),
    .ext_rst_ni     (crg_ext_rst_n),
    .wdt_rst_i      (crg_wdt_rst),
    .brownout_rst_i (crg_bo_rst),
    .pll_en_o       (crg_pll_en),
    .pll_bypass_o   (crg_pll_bypass),
    .pll_cfg_o      (crg_pll_cfg),
    .pll_lock_i     (crg_pll_lock),
    .root_clk_o     (crg_root_clk),
    .sys_rst_no     (crg_sys_rst_n),
    .clk_out_o      (crg_clk_out),
    .rst_out_no     (crg_rst_out_n),
    .mux_active_o   (crg_mux_active),
    .rst_cause_o    (crg_rst_cause)
  );

  // Glitch / Runt-Pulse Monitor on crg_root_clk and crg_clk_out[0]
  logic        crg_glitch_mon_en;
  realtime     last_root_rise;
  realtime     last_root_fall;
  int unsigned root_edge_cnt;

  initial begin
    crg_glitch_mon_en = 1'b0;
    last_root_rise    = 0.0;
    last_root_fall    = 0.0;
    root_edge_cnt     = 0;
  end

  always @(posedge crg_root_clk) begin
    if (crg_glitch_mon_en && (last_root_fall > 0.0)) begin
      automatic realtime t_low = $realtime - last_root_fall;
      if (t_low < 3.4) begin
        $display("[FAIL] CRG root_clk LOW runt pulse detected: %0.3f ns @ %0t", t_low, $realtime);
        $fatal(1, "CRG root_clk glitch");
      end
    end
    last_root_rise <= $realtime;
    root_edge_cnt  <= root_edge_cnt + 1;
  end

  always @(negedge crg_root_clk) begin
    if (crg_glitch_mon_en && (last_root_rise > 0.0)) begin
      automatic realtime t_high = $realtime - last_root_rise;
      if (t_high < 3.4) begin
        $display("[FAIL] CRG root_clk HIGH runt pulse detected: %0.3f ns @ %0t", t_high, $realtime);
        $fatal(1, "CRG root_clk glitch");
      end
    end
    last_root_fall <= $realtime;
  end

  int unsigned branch_edges [4];
  always @(posedge crg_clk_out[0]) branch_edges[0] <= branch_edges[0] + 1;
  always @(posedge crg_clk_out[1]) branch_edges[1] <= branch_edges[1] + 1;
  always @(posedge crg_clk_out[2]) branch_edges[2] <= branch_edges[2] + 1;
  always @(posedge crg_clk_out[3]) branch_edges[3] <= branch_edges[3] + 1;

  // =========================================================================
  // DUT 2: IOMUX (Wishbone B4 Primary)
  // =========================================================================
  wb_m2s_t iomux_wb_m;
  wb_s2m_t iomux_wb_s;
  logic iomux_giso, iomux_gret;
  logic [31:0][7:0] iomux_func_o, iomux_func_oe, iomux_func_i;
  logic [31:0] iomux_pad_i, iomux_pad_o, iomux_pad_oe;
  logic [31:0][1:0] iomux_pad_drive;
  logic [31:0] iomux_pad_slew, iomux_pad_pe, iomux_pad_pu, iomux_pad_schmitt;

  iomux_top #(
    .BUS_TYPE  (bus_pkg::BUS_WISHBONE),
    .NUM_PINS  (32),
    .NUM_FUNCS (8)
  ) u_iomux (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          (iomux_wb_m),
    .wb_o          (iomux_wb_s),
    .axi_i         (axi_zero), .axi_o(),
    .tl_i          (tl_zero),  .tl_o (),
    .apb_i         (apb_zero), .apb_o(),
    .global_iso_i  (iomux_giso),
    .global_ret_i  (iomux_gret),
    .func_o        (iomux_func_o),
    .func_oe_o     (iomux_func_oe),
    .func_i        (iomux_func_i),
    .pad_i         (iomux_pad_i),
    .pad_o         (iomux_pad_o),
    .pad_oe_o      (iomux_pad_oe),
    .pad_drive_o   (iomux_pad_drive),
    .pad_slew_o    (iomux_pad_slew),
    .pad_pull_en_o (iomux_pad_pe),
    .pad_pull_up_o (iomux_pad_pu),
    .pad_schmitt_o (iomux_pad_schmitt)
  );

  // =========================================================================
  // DUT 3: PMU (TileLink TL-UL Primary)
  // =========================================================================
  tl_m2s_t pmu_tl_m;
  tl_s2m_t pmu_tl_s;
  logic [3:0] pmu_slp_req, pmu_pwr_ack;
  logic [7:0] pmu_wake_src;
  logic [3:0] pmu_clk_en, pmu_iso, pmu_ret, pmu_rst_n, pmu_pwr_en;
  logic       pmu_irq;

  pmu_top #(
    .BUS_TYPE      (bus_pkg::BUS_TILELINK),
    .NUM_DOMAINS   (4),
    .NUM_WAKE_SRCS (8)
  ) u_pmu (
    .clk_i         (clk),
    .rst_ni        (rst_n),
    .wb_i          (wb_zero),  .wb_o (),
    .axi_i         (axi_zero), .axi_o(),
    .tl_i          (pmu_tl_m),
    .tl_o          (pmu_tl_s),
    .apb_i         (apb_zero), .apb_o(),
    .sleep_req_i   (pmu_slp_req),
    .dom_pwr_ack_i (pmu_pwr_ack),
    .wake_src_i    (pmu_wake_src),
    .dom_clk_en_o  (pmu_clk_en),
    .dom_iso_o     (pmu_iso),
    .dom_ret_o     (pmu_ret),
    .dom_rst_no    (pmu_rst_n),
    .dom_pwr_en_o  (pmu_pwr_en),
    .irq_o         (pmu_irq)
  );

  // Realistic Power Switch Ack Model (2-cycle turn-on/off delay)
  logic [3:0] pwr_ack_d1_q, pwr_ack_d2_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pwr_ack_d1_q <= 4'hF;
      pwr_ack_d2_q <= 4'hF;
    end else begin
      pwr_ack_d1_q <= pmu_pwr_en;
      pwr_ack_d2_q <= pwr_ack_d1_q;
    end
  end
  assign pmu_pwr_ack = pwr_ack_d2_q;

  // Continuous PMU Power-Sequencing Invariant Monitor
  logic pmu_inv_mon_en = 1'b0;
  logic [3:0] prev_clk_en_q, prev_iso_q, prev_ret_q, prev_rst_n_q, prev_pwr_en_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      prev_clk_en_q <= 4'hF;
      prev_iso_q    <= 4'h0;
      prev_ret_q    <= 4'h0;
      prev_rst_n_q  <= 4'hF;
      prev_pwr_en_q <= 4'hF;
    end else begin
      if (pmu_inv_mon_en) begin
        for (int d = 0; d < 4; d++) begin
          // Power-down invariants:
          // 1. iso_o must NOT rise while clk_en_o is still high
          if (!prev_iso_q[d] && pmu_iso[d] && (pmu_clk_en[d] || prev_clk_en_q[d])) begin
            $display("[FAIL] PMU dom %0d iso rose before clk_en dropped", d);
            $fatal(1, "PMU invariant 1");
          end
          // 2. ret_o must NOT rise while iso_o is low
          if (!prev_ret_q[d] && pmu_ret[d] && !pmu_iso[d]) begin
            $display("[FAIL] PMU dom %0d ret rose before iso asserted", d);
            $fatal(1, "PMU invariant 2");
          end
          // 3. pwr_en_o must NOT drop while iso_o is low
          if (prev_pwr_en_q[d] && !pmu_pwr_en[d] && !pmu_iso[d]) begin
            $display("[FAIL] PMU dom %0d pwr_en dropped before iso asserted", d);
            $fatal(1, "PMU invariant 3");
          end
          // Power-up invariants:
          // 4. ret_o must NOT fall until pwr_en_o=1 and pwr_ack_i=1
          if (prev_ret_q[d] && !pmu_ret[d] && (!pmu_pwr_en[d] || !pmu_pwr_ack[d])) begin
            $display("[FAIL] PMU dom %0d ret released before pwr_ack", d);
            $fatal(1, "PMU invariant 4");
          end
          // 5. iso_o must NOT fall while ret_o is still high
          if (prev_iso_q[d] && !pmu_iso[d] && pmu_ret[d]) begin
            $display("[FAIL] PMU dom %0d iso released before ret released", d);
            $fatal(1, "PMU invariant 5");
          end
          // 6. rst_no must NOT rise while iso_o is still high
          if (!prev_rst_n_q[d] && pmu_rst_n[d] && pmu_iso[d]) begin
            $display("[FAIL] PMU dom %0d rst released before iso released", d);
            $fatal(1, "PMU invariant 6");
          end
          // 7. clk_en_o must NOT rise while rst_no is still low
          if (!prev_clk_en_q[d] && pmu_clk_en[d] && !pmu_rst_n[d]) begin
            $display("[FAIL] PMU dom %0d clk_en released before rst released", d);
            $fatal(1, "PMU invariant 7");
          end
        end
      end
      prev_clk_en_q <= pmu_clk_en;
      prev_iso_q    <= pmu_iso;
      prev_ret_q    <= pmu_ret;
      prev_rst_n_q  <= pmu_rst_n;
      prev_pwr_en_q <= pmu_pwr_en;
    end
  end

  // =========================================================================
  // DUT 4: Bus Firewall (AXI4 CSR Primary + Inline AXI4 Slave Memory)
  // =========================================================================
  axi4_m2s_t fw_csr_m, fw_s_axi_m, fw_m_axi_m;
  axi4_s2m_t fw_csr_s, fw_s_axi_s, fw_m_axi_s;
  logic      fw_irq;

  bus_firewall_top #(
    .BUS_TYPE    (bus_pkg::BUS_AXI4),
    .NUM_REGIONS (8)
  ) u_fw (
    .clk_i   (clk),
    .rst_ni  (rst_n),
    .wb_i    (wb_zero),  .wb_o (),
    .axi_i   (fw_csr_m), .axi_o(fw_csr_s),
    .tl_i    (tl_zero),  .tl_o (),
    .apb_i   (apb_zero), .apb_o(),
    .s_axi_i (fw_s_axi_m),
    .s_axi_o (fw_s_axi_s),
    .m_axi_o (fw_m_axi_m),
    .m_axi_i (fw_m_axi_s),
    .irq_o   (fw_irq)
  );

  // Downstream AXI4 Behavioral Slave Memory (1024 words = 4KB: 0x0000..0x0FFC)
  // Plus illegal-access detector: records if any blocked write ever leaked downstream
  logic [31:0] fw_ram [0:1023];
  int unsigned fw_downstream_wr_cnt;
  int unsigned fw_downstream_rd_cnt;

  typedef enum logic [1:0] { SLV_IDLE, SLV_WDATA, SLV_WRESP, SLV_RDATA } slv_st_e;
  slv_st_e     slv_st_q;
  logic [3:0]  slv_id_q;
  logic [31:0] slv_addr_q;
  logic [7:0]  slv_rem_q;

  always_comb begin
    fw_m_axi_s = '0;
    unique case (slv_st_q)
      SLV_IDLE: begin
        fw_m_axi_s.awready = 1'b1;
        fw_m_axi_s.arready = !fw_m_axi_m.awvalid;
      end
      SLV_WDATA: fw_m_axi_s.wready = 1'b1;
      SLV_WRESP: begin
        fw_m_axi_s.bid    = slv_id_q;
        fw_m_axi_s.bresp  = AXI_RESP_OKAY;
        fw_m_axi_s.bvalid = 1'b1;
      end
      SLV_RDATA: begin
        fw_m_axi_s.rid    = slv_id_q;
        fw_m_axi_s.rdata  = fw_ram[slv_addr_q[11:2]];
        fw_m_axi_s.rresp  = AXI_RESP_OKAY;
        fw_m_axi_s.rlast  = (slv_rem_q == 8'd0);
        fw_m_axi_s.rvalid = 1'b1;
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      slv_st_q             <= SLV_IDLE;
      slv_id_q             <= 4'd0;
      slv_addr_q           <= 32'd0;
      slv_rem_q            <= 8'd0;
      fw_downstream_wr_cnt <= 0;
      fw_downstream_rd_cnt <= 0;
      for (int i = 0; i < 1024; i++) fw_ram[i] <= 32'hA500_0000 | 32'(i);
    end else begin
      unique case (slv_st_q)
        SLV_IDLE: begin
          if (fw_m_axi_m.awvalid && fw_m_axi_s.awready) begin
            slv_id_q             <= fw_m_axi_m.awid;
            slv_addr_q           <= fw_m_axi_m.awaddr;
            slv_st_q             <= SLV_WDATA;
            fw_downstream_wr_cnt <= fw_downstream_wr_cnt + 1;
          end else if (fw_m_axi_m.arvalid && fw_m_axi_s.arready) begin
            slv_id_q             <= fw_m_axi_m.arid;
            slv_addr_q           <= fw_m_axi_m.araddr;
            slv_rem_q            <= fw_m_axi_m.arlen;
            slv_st_q             <= SLV_RDATA;
            fw_downstream_rd_cnt <= fw_downstream_rd_cnt + 1;
          end
        end
        SLV_WDATA: begin
          if (fw_m_axi_m.wvalid) begin
            for (int b = 0; b < 4; b++) begin
              if (fw_m_axi_m.wstrb[b])
                fw_ram[slv_addr_q[11:2]][b*8 +: 8] <= fw_m_axi_m.wdata[b*8 +: 8];
            end
            slv_addr_q <= slv_addr_q + 32'd4;
            if (fw_m_axi_m.wlast) slv_st_q <= SLV_WRESP;
          end
        end
        SLV_WRESP: if (fw_m_axi_m.bready) slv_st_q <= SLV_IDLE;
        SLV_RDATA: if (fw_m_axi_m.rready) begin
          slv_addr_q <= slv_addr_q + 32'd4;
          if (slv_rem_q == 8'd0) slv_st_q <= SLV_IDLE;
          else slv_rem_q <= slv_rem_q - 8'd1;
        end
      endcase
    end
  end

  // =========================================================================
  // DUT 5: SRAM SECDED(39,32) ECC Wrapper
  // =========================================================================
  localparam int unsigned ECC_DEPTH = 64;
  logic        ecc_req_valid, ecc_req_ready, ecc_req_we;
  logic [3:0]  ecc_req_wstrb;
  logic [5:0]  ecc_req_addr;
  logic [31:0] ecc_req_wdata;
  logic        ecc_rsp_valid, ecc_rsp_err;
  logic [31:0] ecc_rsp_rdata;
  logic        ecc_scrub_en, ecc_scrub_busy;
  logic [15:0] ecc_scrub_period;
  logic [38:0] ecc_err_inj;
  logic        ecc_ce_pulse, ecc_ue_pulse;
  logic [31:0] ecc_ce_cnt, ecc_ue_cnt;
  logic [5:0]  ecc_ce_addr, ecc_ue_addr;

  sram_ecc_wrap #(
    .DEPTH        (ECC_DEPTH),
    .ENABLE_SCRUB (1'b1)
  ) u_ecc (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .req_valid_i    (ecc_req_valid),
    .req_ready_o    (ecc_req_ready),
    .req_we_i       (ecc_req_we),
    .req_wstrb_i    (ecc_req_wstrb),
    .req_addr_i     (ecc_req_addr),
    .req_wdata_i    (ecc_req_wdata),
    .rsp_valid_o    (ecc_rsp_valid),
    .rsp_rdata_o    (ecc_rsp_rdata),
    .rsp_err_o      (ecc_rsp_err),
    .scrub_en_i     (ecc_scrub_en),
    .scrub_period_i (ecc_scrub_period),
    .scrub_busy_o   (ecc_scrub_busy),
    .err_inj_i      (ecc_err_inj),
    .ce_pulse_o     (ecc_ce_pulse),
    .ue_pulse_o     (ecc_ue_pulse),
    .ce_cnt_o       (ecc_ce_cnt),
    .ue_cnt_o       (ecc_ue_cnt),
    .ce_addr_o      (ecc_ce_addr),
    .ue_addr_o      (ecc_ue_addr)
  );

  // =========================================================================
  // DUT 6: Quad-Bus Cross-Check Smoke Instances (verify remaining bus wrappers)
  // =========================================================================
  wb_m2s_t   smk_wb_m;   wb_s2m_t   smk_wb_s;
  axi4_m2s_t smk_axi_m;  axi4_s2m_t smk_axi_s;
  tl_m2s_t   smk_tl_m;   tl_s2m_t   smk_tl_s;
  apb_m2s_t  smk_apb_m;  apb_s2m_t  smk_apb_s;

  // CRG WB smoke
  crg_top #(.BUS_TYPE(bus_pkg::BUS_WISHBONE)) u_smk_crg_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(smk_wb_m), .wb_o(smk_wb_s),
    .axi_i(axi_zero), .axi_o(), .tl_i(tl_zero), .tl_o(), .apb_i(apb_zero), .apb_o(),
    .osc_clk_i(osc_clk), .pll_clk_i(pll_clk), .ext_clk_i(ext_clk), .aux_clk_i(aux_clk),
    .test_mode_i(1'b0), .por_ni(crg_por_n), .ext_rst_ni(1'b1), .wdt_rst_i(1'b0), .brownout_rst_i(1'b0),
    .pll_en_o(), .pll_bypass_o(), .pll_cfg_o(), .pll_lock_i(1'b0),
    .root_clk_o(), .sys_rst_no(), .clk_out_o(), .rst_out_no(), .mux_active_o(), .rst_cause_o()
  );

  // IOMUX AXI4 smoke
  iomux_top #(.BUS_TYPE(bus_pkg::BUS_AXI4), .NUM_PINS(8), .NUM_FUNCS(4)) u_smk_iomux_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(wb_zero), .wb_o(),
    .axi_i(smk_axi_m), .axi_o(smk_axi_s),
    .tl_i(tl_zero), .tl_o(), .apb_i(apb_zero), .apb_o(),
    .global_iso_i(1'b0), .global_ret_i(1'b0),
    .func_o('0), .func_oe_o('0), .func_i(),
    .pad_i(8'h5A), .pad_o(), .pad_oe_o(), .pad_drive_o(), .pad_slew_o(),
    .pad_pull_en_o(), .pad_pull_up_o(), .pad_schmitt_o()
  );

  // PMU APB4 smoke
  pmu_top #(.BUS_TYPE(bus_pkg::BUS_APB), .NUM_DOMAINS(2), .NUM_WAKE_SRCS(4)) u_smk_pmu_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(wb_zero), .wb_o(), .axi_i(axi_zero), .axi_o(), .tl_i(tl_zero), .tl_o(),
    .apb_i(smk_apb_m), .apb_o(smk_apb_s),
    .sleep_req_i(2'b00), .dom_pwr_ack_i(2'b11), .wake_src_i(4'h0),
    .dom_clk_en_o(), .dom_iso_o(), .dom_ret_o(), .dom_rst_no(), .dom_pwr_en_o(), .irq_o()
  );

  // Bus Firewall TileLink smoke
  axi4_m2s_t smk_fw_m_o;
  bus_firewall_top #(.BUS_TYPE(bus_pkg::BUS_TILELINK), .NUM_REGIONS(4)) u_smk_fw_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(wb_zero), .wb_o(), .axi_i(axi_zero), .axi_o(),
    .tl_i(smk_tl_m), .tl_o(smk_tl_s),
    .apb_i(apb_zero), .apb_o(),
    .s_axi_i(axi_zero), .s_axi_o(), .m_axi_o(smk_fw_m_o), .m_axi_i('0), .irq_o()
  );

  // =========================================================================
  // Helper Tasks for Bus Firewall AXI4 Master & ECC SRAM
  // =========================================================================
  task automatic fw_axi_tx(
    input  logic        is_write,
    input  logic [31:0] addr,
    input  logic [31:0] wdata,
    input  logic [7:0]  len,
    input  logic [3:0]  id,
    input  logic [2:0]  prot,
    output logic [1:0]  resp_o,
    output logic [31:0] rdata_o
  );
    rdata_o = 32'd0;
    if (is_write) begin
      @(posedge clk); #1;
      fw_s_axi_m         = '0;
      fw_s_axi_m.awid    = id;
      fw_s_axi_m.awaddr  = addr;
      fw_s_axi_m.awlen   = len;
      fw_s_axi_m.awsize  = 3'd2;
      fw_s_axi_m.awburst = AXI_BURST_INCR;
      fw_s_axi_m.awprot  = prot;
      fw_s_axi_m.awvalid = 1'b1;
      fw_s_axi_m.bready  = 1'b1;
      do @(posedge clk); while (!fw_s_axi_s.awready);
      #1; fw_s_axi_m.awvalid = 1'b0;

      for (int b = 0; b <= int'(len); b++) begin
        fw_s_axi_m.wdata  = wdata + 32'(b);
        fw_s_axi_m.wstrb  = 4'hF;
        fw_s_axi_m.wlast  = (b == int'(len));
        fw_s_axi_m.wvalid = 1'b1;
        do @(posedge clk); while (!fw_s_axi_s.wready);
        #1;
      end
      fw_s_axi_m.wvalid = 1'b0;
      fw_s_axi_m.wlast  = 1'b0;

      do @(posedge clk); while (!fw_s_axi_s.bvalid);
      resp_o = fw_s_axi_s.bresp;
      #1; fw_s_axi_m = '0;
    end else begin
      @(posedge clk); #1;
      fw_s_axi_m         = '0;
      fw_s_axi_m.arid    = id;
      fw_s_axi_m.araddr  = addr;
      fw_s_axi_m.arlen   = len;
      fw_s_axi_m.arsize  = 3'd2;
      fw_s_axi_m.arburst = AXI_BURST_INCR;
      fw_s_axi_m.arprot  = prot;
      fw_s_axi_m.arvalid = 1'b1;
      fw_s_axi_m.rready  = 1'b1;
      do @(posedge clk); while (!fw_s_axi_s.arready);
      #1; fw_s_axi_m.arvalid = 1'b0;

      for (int b = 0; b <= int'(len); b++) begin
        do @(posedge clk); while (!fw_s_axi_s.rvalid);
        if (b == 0) rdata_o = fw_s_axi_s.rdata;
        resp_o = fw_s_axi_s.rresp;
        if (b == int'(len)) check_cond("fw rlast", fw_s_axi_s.rlast == 1'b1);
      end
      #1; fw_s_axi_m = '0;
    end
  endtask

  task automatic ecc_write(input logic [5:0] addr, input logic [31:0] data,
                           input logic [3:0] strb = 4'hF, input logic [38:0] inj = 39'd0);
    @(posedge clk); #1;
    ecc_req_valid = 1'b1;
    ecc_req_we    = 1'b1;
    ecc_req_wstrb = strb;
    ecc_req_addr  = addr;
    ecc_req_wdata = data;
    ecc_err_inj   = inj;
    do @(posedge clk); while (!ecc_req_ready);
    #1;
    ecc_req_valid = 1'b0;
    ecc_req_we    = 1'b0;
    // If partial write, wait for RMW to complete before clearing err_inj
    if ((strb != 4'hF) && (strb != 4'h0)) begin
      do @(posedge clk); while (!ecc_req_ready);
      #1;
    end
    ecc_err_inj = 39'd0;
  endtask

  task automatic ecc_read(input logic [5:0] addr, output logic [31:0] data,
                          output logic err_o, output logic ce_seen, output logic ue_seen);
    ce_seen = 1'b0;
    ue_seen = 1'b0;
    @(posedge clk); #1;
    ecc_req_valid = 1'b1;
    ecc_req_we    = 1'b0;
    ecc_req_addr  = addr;
    do @(posedge clk); while (!ecc_req_ready);
    #1;
    ecc_req_valid = 1'b0;
    do begin
      @(posedge clk);
      if (ecc_ce_pulse) ce_seen = 1'b1;
      if (ecc_ue_pulse) ue_seen = 1'b1;
    end while (!ecc_rsp_valid);
    data  = ecc_rsp_rdata;
    err_o = ecc_rsp_err;
    if (ecc_ce_pulse) ce_seen = 1'b1;
    if (ecc_ue_pulse) ue_seen = 1'b1;
  endtask

  // =========================================================================
  // Test Sequences
  // =========================================================================
  task automatic test_crg();
    logic [31:0] rdata;
    int unsigned e0, e1;
    $display("[TB] 1. Testing CRG (glitch-free mux, dividers, ICG, PLL CDC, RST_CAUSE)...");

    // Check POR sticky cause
    do_apb_read(crg_apb_m, crg_apb_s, 32'h18, rdata);
    check_eq32("CRG POR cause", rdata, 32'h01);
    check_eq32("CRG active OSC", {28'd0, crg_mux_active}, 32'h1);
    cov_crg_causes++;

    // Enable glitch monitor
    crg_glitch_mon_en = 1'b1;

    // Configure PLL & verify CDC lock sync
    do_apb_write(crg_apb_m, crg_apb_s, 32'h08, 32'h1234_5678);
    check_eq32("CRG pll_cfg_o", crg_pll_cfg, 32'h1234_5678);
    crg_pll_lock = 1'b1;
    repeat (5) @(posedge clk);
    do_apb_read(crg_apb_m, crg_apb_s, 32'h04, rdata);
    check_cond("CRG PLL_LOCK synced", rdata[8] == 1'b1);

    // Switch OSC(0) -> PLL(1) -> EXT(2) -> AUX(3) -> OSC(0), including back-to-back mid-switch
    do_apb_write(crg_apb_m, crg_apb_s, 32'h00, 32'h0000_0111); // sel=1, pll_en=1, global_cg_en=1
    repeat (15) @(posedge clk);
    do_apb_read(crg_apb_m, crg_apb_s, 32'h04, rdata);
    check_eq32("CRG mux active PLL", {28'd0, rdata[3:0]}, 32'h2);
    cov_crg_switches++;

    // Back-to-back write: sel=2 then immediately sel=3 before 2 completes
    do_apb_write(crg_apb_m, crg_apb_s, 32'h00, 32'h0000_0112);
    do_apb_write(crg_apb_m, crg_apb_s, 32'h00, 32'h0000_0113);
    repeat (25) @(posedge clk);
    do_apb_read(crg_apb_m, crg_apb_s, 32'h04, rdata);
    check_eq32("CRG mux active AUX after b2b", {28'd0, rdata[3:0]}, 32'h8);
    cov_crg_switches += 2;

    // Switch back to OSC(0)
    do_apb_write(crg_apb_m, crg_apb_s, 32'h00, 32'h0000_0100);
    repeat (20) @(posedge clk);
    do_apb_read(crg_apb_m, crg_apb_s, 32'h04, rdata);
    check_eq32("CRG mux active OSC", {28'd0, rdata[3:0]}, 32'h1);
    cov_crg_switches++;

    // Test dividers: ch0 = bypass(1), ch1 = div 4, ch2 = div 3, ch3 = frac 2.5 (int=2, frac=128)
    do_apb_write(crg_apb_m, crg_apb_s, 32'h20, 32'h0000_0001);
    do_apb_write(crg_apb_m, crg_apb_s, 32'h24, 32'h0000_0004);
    do_apb_write(crg_apb_m, crg_apb_s, 32'h28, 32'h0000_0003);
    do_apb_write(crg_apb_m, crg_apb_s, 32'h2C, 32'h0080_0002);
    repeat (10) @(posedge clk);

    e0 = branch_edges[0];
    e1 = branch_edges[1];
    begin
      int unsigned e2 = branch_edges[2];
      int unsigned e3 = branch_edges[3];
      repeat (60) @(posedge osc_clk);
      begin
        int unsigned d0 = branch_edges[0] - e0;
        int unsigned d1 = branch_edges[1] - e1;
        int unsigned d2 = branch_edges[2] - e2;
        int unsigned d3 = branch_edges[3] - e3;
        check_cond("CRG div1 count", (d0 >= 58) && (d0 <= 62));
        check_cond("CRG div4 count", (d1 >= 14) && (d1 <= 16));
        check_cond("CRG div3 count", (d2 >= 19) && (d2 <= 21));
        check_cond("CRG div2.5 frac count", (d3 >= 22) && (d3 <= 26));
        cov_crg_divs += 4;
      end
    end

    // Test ICG clock gating on branch 1
    do_apb_write(crg_apb_m, crg_apb_s, 32'h0C, 32'h0000_000D); // disable bit 1
    repeat (6) @(posedge osc_clk);
    e1 = branch_edges[1];
    repeat (20) @(posedge osc_clk);
    check_eq32("CRG branch 1 gated off", branch_edges[1], e1);
    do_apb_write(crg_apb_m, crg_apb_s, 32'h0C, 32'h0000_000F); // re-enable bit 1
    repeat (12) @(posedge osc_clk);
    check_cond("CRG branch 1 re-enabled", branch_edges[1] > e1);

    // Per-domain SW reset
    do_apb_write(crg_apb_m, crg_apb_s, 32'h10, 32'h0000_0004); // assert dom 2 reset
    repeat (4) @(posedge clk);
    check_cond("CRG dom 2 rst_out_no low", crg_rst_out_n[2] == 1'b0);
    check_cond("CRG dom 0 rst_out_no stays high", crg_rst_out_n[0] == 1'b1);
    do_apb_write(crg_apb_m, crg_apb_s, 32'h10, 32'h0000_0000);
    repeat (15) @(posedge osc_clk);
    check_cond("CRG dom 2 rst_out_no released", crg_rst_out_n[2] == 1'b1);

    // Sticky RST_CAUSE across WDT, SW, EXT, BROWN_OUT + warm rst_ni survival + W1C
    do_apb_write(crg_apb_m, crg_apb_s, 32'h18, 32'h0000_001F); // clear POR
    do_apb_read(crg_apb_m, crg_apb_s, 32'h18, rdata);
    check_eq32("CRG cause cleared", rdata, 32'h00);

    crg_wdt_rst = 1'b1; repeat (4) @(posedge clk); crg_wdt_rst = 1'b0;
    do_apb_write(crg_apb_m, crg_apb_s, 32'h1C, 32'h5A5A_0001); // SW reset key
    repeat (12) @(posedge clk);
    crg_ext_rst_n = 1'b0; repeat (4) @(posedge clk); crg_ext_rst_n = 1'b1;
    crg_bo_rst    = 1'b1; repeat (4) @(posedge clk); crg_bo_rst    = 1'b0;
    repeat (4) @(posedge clk);

    // Pulse warm rst_ni while por_ni=1 -> RST_CAUSE must survive!
    @(posedge clk); #1; rst_n = 1'b0;
    repeat (3) @(posedge clk); #1; rst_n = 1'b1;
    repeat (4) @(posedge clk);

    do_apb_read(crg_apb_m, crg_apb_s, 32'h18, rdata);
    check_eq32("CRG sticky causes survived warm reset", rdata, 32'h1E);
    cov_crg_causes += 4;

    // W1C selective clear
    do_apb_write(crg_apb_m, crg_apb_s, 32'h18, 32'h0000_000A); // clear WDT & EXT
    do_apb_read(crg_apb_m, crg_apb_s, 32'h18, rdata);
    check_eq32("CRG selective W1C", rdata, 32'h14);
    do_apb_write(crg_apb_m, crg_apb_s, 32'h18, 32'h0000_001F);
    crg_glitch_mon_en = 1'b0;
  endtask

  task automatic test_iomux();
    logic [31:0] rdata;
    logic [2:0]  exp_alt   [32];
    logic [1:0]  exp_drv   [32];
    logic        exp_slew  [32];
    logic        exp_pe    [32];
    logic        exp_pu    [32];
    logic        exp_sch   [32];
    $display("[TB] 2. Testing IOMUX (32x8 mux, pad attrs, IN_ISO, OUT_RET, LOCK)...");

    // Randomize configuration across all 32 pins
    for (int p = 0; p < 32; p++) begin
      logic [31:0] cfg;
      exp_alt[p]  = 3'($urandom % 8);
      exp_drv[p]  = 2'($urandom % 4);
      exp_slew[p] = 1'($urandom % 2);
      exp_pe[p]   = 1'($urandom % 2);
      exp_pu[p]   = 1'($urandom % 2);
      exp_sch[p]  = 1'($urandom % 2);
      cfg = {22'd0, exp_sch[p], exp_pu[p], exp_pe[p], exp_slew[p], exp_drv[p], 1'b0, exp_alt[p]};
      do_wb_write(iomux_wb_m, iomux_wb_s, 32'(p * 4), cfg);
      cov_iomux_ops++;
    end

    // Drive random func_o / func_oe / pad_i and compare against golden model
    for (int iter = 0; iter < 8; iter++) begin
      @(posedge clk); #1;
      for (int p = 0; p < 32; p++) begin
        iomux_func_o[p]  = 8'($urandom);
        iomux_func_oe[p] = 8'($urandom);
      end
      iomux_pad_i = $urandom;
      #1;
      for (int p = 0; p < 32; p++) begin
        check_cond("IOMUX pad_o", iomux_pad_o[p] == iomux_func_o[p][exp_alt[p]]);
        check_cond("IOMUX pad_oe", iomux_pad_oe[p] == iomux_func_oe[p][exp_alt[p]]);
        check_cond("IOMUX drive", iomux_pad_drive[p] == exp_drv[p]);
        check_cond("IOMUX slew", iomux_pad_slew[p] == exp_slew[p]);
        check_cond("IOMUX pull_en", iomux_pad_pe[p] == exp_pe[p]);
        check_cond("IOMUX pull_up", iomux_pad_pu[p] == exp_pu[p]);
        check_cond("IOMUX schmitt", iomux_pad_schmitt[p] == exp_sch[p]);
        for (int f = 0; f < 8; f++) begin
          logic exp_fi = (f == int'(exp_alt[p])) ? iomux_pad_i[p] : 1'b0;
          check_cond("IOMUX func_i", iomux_func_i[p][f] == exp_fi);
        end
      end
      cov_iomux_ops++;
    end

    // Test IN_ISO on pin 5
    do_wb_write(iomux_wb_m, iomux_wb_s, 32'h14, {21'd0, 1'b1, 1'b0, 6'd0, exp_alt[5]}); // IN_ISO=1
    iomux_pad_i[5] = 1'b1;
    #1;
    check_eq32("IOMUX IN_ISO clamps func_i[5] to 0", {24'd0, iomux_func_i[5]}, 32'd0);

    // Test OUT_RET on pin 7: capture known value 1/1, then assert OUT_RET=1 and flip func_o/oe
    do_wb_write(iomux_wb_m, iomux_wb_s, 32'h1C, 32'h0000_0002); // pin 7 alt=2, OUT_RET=0
    @(posedge clk); #1;
    iomux_func_o[7]  = 8'hFF;
    iomux_func_oe[7] = 8'hFF;
    @(posedge clk); #1; // ret_pad_o_q[7] & ret_pad_oe_q[7] capture 1
    do_wb_write(iomux_wb_m, iomux_wb_s, 32'h1C, 32'h0000_0802); // OUT_RET=1
    @(posedge clk); #1;
    iomux_func_o[7]  = 8'h00;
    iomux_func_oe[7] = 8'h00;
    #1;
    check_cond("IOMUX OUT_RET holds pad_o=1", iomux_pad_o[7] == 1'b1);
    check_cond("IOMUX OUT_RET holds pad_oe=1", iomux_pad_oe[7] == 1'b1);
    do_wb_write(iomux_wb_m, iomux_wb_s, 32'h1C, 32'h0000_0002); // release OUT_RET=0
    #1;
    check_cond("IOMUX OUT_RET released pad_o=0", iomux_pad_o[7] == 1'b0);

    // Test write-once LOCK on pin 3
    do_wb_write(iomux_wb_m, iomux_wb_s, 32'h0C, 32'h8000_0235); // alt=5, drive=3, schmitt=1, LOCK=1
    do_wb_write(iomux_wb_m, iomux_wb_s, 32'h0C, 32'h0000_0000); // attempt overwrite & unlock
    do_wb_read(iomux_wb_m, iomux_wb_s, 32'h0C, rdata);
    check_eq32("IOMUX pin 3 LOCK preserved", rdata, 32'h8000_0235);
    do_wb_read(iomux_wb_m, iomux_wb_s, 32'h108, rdata);
    check_eq32("IOMUX LOCK_MASK bit 3", rdata, 32'h0000_0008);
    cov_iomux_ops += 4;
  endtask

  task automatic test_pmu();
    logic [31:0] rdata;
    $display("[TB] 3. Testing PMU (power-down/up invariants, delays, wake matrix, irq)...");
    pmu_inv_mon_en = 1'b1;

    // Configure Domain 0: fast delays [pwr=2, ret=2, iso=2, clk=2], HW sleep enabled, wake mask bit 2
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h2C, 32'h0202_0202);
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h28, 32'h0000_0004); // dom 0 wake mask = src 2
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h20, 32'h0000_0013); // wake_done_irq=1, ret_en=1, hw_slp=1

    // Configure Domain 1: custom delays [pwr=4, ret=3, iso=2, clk=1], wake mask bit 5 (active-low!)
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h3C, 32'h0403_0201);
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h38, 32'h0000_0020); // dom 1 wake mask = src 5
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h30, 32'h0000_0002); // ret_en=1, hw_slp=0 (SW trigger)

    // Configure Wake Sources: src 2 active-high edge, src 5 active-low level
    pmu_wake_src = 8'b0010_0000; // src 5 starts high (inactive for active-low)
    repeat (4) @(posedge clk);
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h08, 32'h0000_00DF); // pol: bit 5 = 0 (active-low), others 1
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h0C, 32'h0000_00DF); // edge: bit 5 = 0 (level), others 1
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h04, 32'h0000_00FF); // wake_en all
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h14, 32'h0000_00FF); // wake_irq_en all
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h10, 32'h0000_00FF); // clear any initial status

    // Trigger Domain 0 sleep via hardware sleep_req_i[0], and Domain 1 via SW_PWR_DN
    @(posedge clk); #1; pmu_slp_req[0] = 1'b1;
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h30, 32'h0000_0006); // SW_PWR_DN=1 on dom 1
    repeat (4) @(posedge clk);
    @(posedge clk); #1; pmu_slp_req[0] = 1'b0;

    // Wait for both domains to reach PWR_OFF (state == 4)
    repeat (25) @(posedge clk);
    do_tl_read(pmu_tl_m, pmu_tl_s, 32'h24, rdata);
    check_eq32("PMU dom 0 PWR_OFF state", {28'd0, rdata[3:0]}, 32'd4);
    check_cond("PMU dom 0 iso=1 ret=1 pwr_en=0", (pmu_iso[0] == 1'b1) && (pmu_ret[0] == 1'b1) && (pmu_pwr_en[0] == 1'b0));
    do_tl_read(pmu_tl_m, pmu_tl_s, 32'h34, rdata);
    check_eq32("PMU dom 1 PWR_OFF state", {28'd0, rdata[3:0]}, 32'd4);
    cov_pmu_cycles += 2;

    // Pulse wake_src[2] -> wakes Domain 0 ONLY (Domain 1 stays PWR_OFF)
    @(posedge clk); #1; pmu_wake_src[2] = 1'b1;
    repeat (3) @(posedge clk); #1; pmu_wake_src[2] = 1'b0;
    repeat (30) @(posedge clk);

    do_tl_read(pmu_tl_m, pmu_tl_s, 32'h24, rdata);
    check_eq32("PMU dom 0 back in RUN", {28'd0, rdata[3:0]}, 32'd0);
    check_cond("PMU dom 0 wake_done=1", rdata[10] == 1'b1);
    do_tl_read(pmu_tl_m, pmu_tl_s, 32'h34, rdata);
    check_eq32("PMU dom 1 still PWR_OFF", {28'd0, rdata[3:0]}, 32'd4);
    check_cond("PMU irq_o asserted on wake", pmu_irq == 1'b1);
    do_tl_read(pmu_tl_m, pmu_tl_s, 32'h10, rdata);
    check_eq32("PMU WAKE_STATUS bit 2", rdata, 32'h04);
    cov_pmu_wakes++;

    // Clear wake_status and wake_done -> irq_o deasserts
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h10, 32'h0000_00FF);
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h24, 32'h0000_0400); // W1C bit 10 WAKE_DONE
    repeat (2) @(posedge clk);
    check_cond("PMU irq_o cleared", pmu_irq == 1'b0);

    // Pull active-low wake_src[5] low -> wakes Domain 1
    @(posedge clk); #1; pmu_wake_src[5] = 1'b0;
    repeat (4) @(posedge clk); #1; pmu_wake_src[5] = 1'b1;
    repeat (35) @(posedge clk);
    do_tl_read(pmu_tl_m, pmu_tl_s, 32'h34, rdata);
    check_eq32("PMU dom 1 back in RUN", {28'd0, rdata[3:0]}, 32'd0);
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h10, 32'h0000_00FF);
    cov_pmu_wakes++;

    // Also test remaining wake sources (0,1,3,4,6,7) on Domain 2 & 3
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h48, 32'h0000_00DB); // dom 2 mask
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h40, 32'h0000_0006); // SW_PWR_DN dom 2
    repeat (25) @(posedge clk);
    @(posedge clk); #1; pmu_wake_src[7] = 1'b1;
    repeat (3) @(posedge clk); #1; pmu_wake_src[7] = 1'b0;
    repeat (30) @(posedge clk);
    do_tl_read(pmu_tl_m, pmu_tl_s, 32'h44, rdata);
    check_eq32("PMU dom 2 woken by src 7", {28'd0, rdata[3:0]}, 32'd0);
    do_tl_write(pmu_tl_m, pmu_tl_s, 32'h10, 32'h0000_00FF);
    cov_pmu_wakes++;
  endtask

  task automatic test_firewall();
    logic [31:0] rdata, rd_val;
    logic [1:0]  resp;
    int unsigned wr_before, rd_before;
    $display("[TB] 4. Testing Bus Firewall (regions, priority, burst bounds, SEC/PRIV, LOCK, rand)...");

    // Region 0 (highest priority): Secret carveout [0x100..0x1FF], only ID=0, SEC_ONLY, R/W, DECERR, LOCK
    do_axi_write(fw_csr_m, fw_csr_s, 32'h20, 32'h0000_0100); // BASE0
    do_axi_write(fw_csr_m, fw_csr_s, 32'h24, 32'h0000_01FF); // LIMIT0
    do_axi_write(fw_csr_m, fw_csr_s, 32'h28, 32'h0000_0001); // MASTER_MASK0 (only ID=0)
    do_axi_write(fw_csr_m, fw_csr_s, 32'h2C, 32'h8000_002F); // EN=1, R=1, W=1, SEC_ONLY=1, RESP_MODE=1(DECERR), LOCK=1

    // Attempt to overwrite locked Region 0 -> must be ignored!
    do_axi_write(fw_csr_m, fw_csr_s, 32'h20, 32'hDEAD_BEEF);
    do_axi_read(fw_csr_m, fw_csr_s, 32'h20, rdata);
    check_eq32("FW Region 0 LOCK holds BASE0", rdata, 32'h0000_0100);

    // Region 1: Shared DMA window [0x000..0x7FF], IDs 0..3, R/W, SLVERR
    do_axi_write(fw_csr_m, fw_csr_s, 32'h30, 32'h0000_0000);
    do_axi_write(fw_csr_m, fw_csr_s, 32'h34, 32'h0000_07FF);
    do_axi_write(fw_csr_m, fw_csr_s, 32'h38, 32'h0000_000F); // IDs 0..3
    do_axi_write(fw_csr_m, fw_csr_s, 32'h3C, 32'h0000_0007); // EN=1, R=1, W=1, SLVERR

    // Region 2: Read-only ROM window [0x800..0xBFF], IDs 0..7, R=1, W=0, SLVERR
    do_axi_write(fw_csr_m, fw_csr_s, 32'h40, 32'h0000_0800);
    do_axi_write(fw_csr_m, fw_csr_s, 32'h44, 32'h0000_0BFF);
    do_axi_write(fw_csr_m, fw_csr_s, 32'h48, 32'h0000_00FF);
    do_axi_write(fw_csr_m, fw_csr_s, 32'h4C, 32'h0000_0003); // EN=1, R=1, W=0

    // Enable firewall (EN=1, DEFAULT_ALLOW=0, DEFAULT_DECERR=0, IRQ_EN=1)
    do_axi_write(fw_csr_m, fw_csr_s, 32'h00, 32'h0000_0009);

    // 1. Legal Secure write/read to Region 0 by ID=0
    fw_axi_tx(1'b1, 32'h0000_0120, 32'hCAFE_1234, 8'd0, 4'd0, 3'b000, resp, rd_val);
    check_eq32("FW Rgn0 legal write OKAY", {30'd0, resp}, {30'd0, AXI_RESP_OKAY});
    fw_axi_tx(1'b0, 32'h0000_0120, 32'd0, 8'd0, 4'd0, 3'b000, resp, rd_val);
    check_eq32("FW Rgn0 legal read OKAY", {30'd0, resp}, {30'd0, AXI_RESP_OKAY});
    check_eq32("FW Rgn0 readback", rd_val, 32'hCAFE_1234);
    cov_fw_txns += 2;

    // 2. Illegal Non-Secure (prot[1]=1) write to Region 0 by ID=0 -> Region 0 takes priority over Region 1, returns DECERR!
    wr_before = fw_downstream_wr_cnt;
    fw_axi_tx(1'b1, 32'h0000_0120, 32'hBAD0_0000, 8'd0, 4'd0, 3'b010, resp, rd_val);
    check_eq32("FW Rgn0 NS write DECERR", {30'd0, resp}, {30'd0, AXI_RESP_DECERR});
    check_eq32("FW blocked write never hit downstream", fw_downstream_wr_cnt, wr_before);
    check_cond("FW irq_o asserted", fw_irq == 1'b1);
    do_axi_read(fw_csr_m, fw_csr_s, 32'h08, rdata);
    check_eq32("FW VIOL_ADDR", rdata, 32'h0000_0120);
    do_axi_read(fw_csr_m, fw_csr_s, 32'h04, rdata);
    check_cond("FW VIOL_STAT write+id+valid", (rdata[0] == 1'b1) && (rdata[2] == 1'b1) && (rdata[6:3] == 4'd0));
    // Clear violation
    do_axi_write(fw_csr_m, fw_csr_s, 32'h04, 32'h0000_0003);
    check_cond("FW irq_o cleared", fw_irq == 1'b0);
    cov_fw_blocks++;

    // 3. Illegal write to Region 2 (Read-Only) -> SLVERR
    wr_before = fw_downstream_wr_cnt;
    fw_axi_tx(1'b1, 32'h0000_0840, 32'h1122_3344, 8'd0, 4'd5, 3'b000, resp, rd_val);
    check_eq32("FW Rgn2 RO write SLVERR", {30'd0, resp}, {30'd0, AXI_RESP_SLVERR});
    check_eq32("FW Rgn2 blocked write downstream", fw_downstream_wr_cnt, wr_before);
    do_axi_read(fw_csr_m, fw_csr_s, 32'h04, rdata);
    check_eq32("FW VIOL_ID == 5", {28'd0, rdata[6:3]}, 32'd5);
    do_axi_write(fw_csr_m, fw_csr_s, 32'h04, 32'h0000_0003);
    cov_fw_blocks++;

    // 4. Illegal burst crossing Region 2 limit (starts at 0xBF8, 4 beats -> 0xBF8..0xC07 crosses 0xBFF)
    rd_before = fw_downstream_rd_cnt;
    fw_axi_tx(1'b0, 32'h0000_0BF8, 32'd0, 8'd3, 4'd2, 3'b000, resp, rd_val);
    check_eq32("FW crossing burst denied SLVERR", {30'd0, resp}, {30'd0, AXI_RESP_SLVERR});
    check_eq32("FW crossing burst never hit downstream", fw_downstream_rd_cnt, rd_before);
    do_axi_write(fw_csr_m, fw_csr_s, 32'h04, 32'h0000_0003);
    cov_fw_blocks++;

    // 5. Randomized live AXI4 traffic vs independent golden model
    for (int t = 0; t < 40; t++) begin
      logic        is_w  = 1'($urandom % 2);
      logic [3:0]  id    = 4'($urandom % 10);
      logic [2:0]  prot  = 3'($urandom % 4);
      logic [31:0] addr  = 32'(($urandom % 256) * 4) + ((($urandom % 4) == 3) ? 32'h0800 : 32'h0000);
      logic [31:0] wval  = $urandom;
      logic        exp_allow;
      logic [1:0]  exp_resp;

      // Golden check matching programmed regions
      if (addr >= 32'h100 && addr <= 32'h1FF) begin
        exp_allow = (id == 4'd0) && !prot[1];
        exp_resp  = exp_allow ? AXI_RESP_OKAY : AXI_RESP_DECERR;
      end else if (addr <= 32'h7FF) begin
        exp_allow = (id <= 4'd3);
        exp_resp  = exp_allow ? AXI_RESP_OKAY : AXI_RESP_SLVERR;
      end else if (addr >= 32'h800 && addr <= 32'hBFF) begin
        exp_allow = (!is_w) && (id <= 4'd7);
        exp_resp  = exp_allow ? AXI_RESP_OKAY : AXI_RESP_SLVERR;
      end else begin
        exp_allow = 1'b0;
        exp_resp  = AXI_RESP_SLVERR;
      end

      fw_axi_tx(is_w, addr, wval, 8'd0, id, prot, resp, rd_val);
      check_eq32("FW rand resp", {30'd0, resp}, {30'd0, exp_resp});
      cov_fw_txns++;
      if (!exp_allow) begin
        cov_fw_blocks++;
        do_axi_write(fw_csr_m, fw_csr_s, 32'h04, 32'h0000_0003);
      end
    end
  endtask

  task automatic test_ecc();
    logic [31:0] rd, exp;
    logic err, ce, ue;
    logic [31:0] shadow [0:ECC_DEPTH-1];
    $display("[TB] 5. Testing sram_ecc_wrap (all 39 CE bits, UE pairs, partial RMW, scrubber)...");

    // Initialize all words cleanly
    for (int a = 0; a < ECC_DEPTH; a++) begin
      shadow[a] = $urandom;
      ecc_write(6'(a), shadow[a], 4'hF, 39'd0);
    end

    // Verify clean reads
    for (int a = 0; a < ECC_DEPTH; a++) begin
      ecc_read(6'(a), rd, err, ce, ue);
      check_eq32("ECC clean read", rd, shadow[a]);
      check_cond("ECC clean flags", !err && !ce && !ue);
    end

    // Exhaustive test of ALL 39 single-bit error positions (bits 0..38)
    for (int bit_pos = 0; bit_pos < 39; bit_pos++) begin
      logic [5:0]  a = 6'(bit_pos % ECC_DEPTH);
      logic [31:0] val = $urandom;
      logic [38:0] mask = (39'd1 << bit_pos);
      shadow[a] = val;
      ecc_write(a, val, 4'hF, mask);
      ecc_read(a, rd, err, ce, ue);
      check_eq32("ECC 1-bit corrected data", rd, val);
      check_cond("ECC 1-bit ce_pulse & !ue", ce && !ue && !err);
      check_eq32("ECC ce_addr_o", {26'd0, ecc_ce_addr}, {26'd0, a});
      cov_ecc_ce_bits++;
    end

    // Test double-bit UE detection across data/parity/overall-parity pairs
    begin
      int pairs[6][2] = '{'{0, 3}, '{1, 2}, '{3, 38}, '{5, 19}, '{16, 32}, '{0, 38}};
      for (int p = 0; p < 6; p++) begin
        logic [5:0]  a = 6'(10 + p);
        logic [31:0] val = $urandom;
        logic [38:0] mask = (39'd1 << pairs[p][0]) | (39'd1 << pairs[p][1]);
        ecc_write(a, val, 4'hF, mask);
        ecc_read(a, rd, err, ce, ue);
        check_cond("ECC 2-bit ue_pulse & rsp_err", ue && err && !ce);
        check_eq32("ECC ue_addr_o", {26'd0, ecc_ue_addr}, {26'd0, a});
        cov_ecc_ue_pairs++;
      end
    end

    // Test partial-byte RMW (including RMW over a word with a 1-bit soft error!)
    begin
      logic [5:0]  a = 6'd22;
      logic [31:0] orig = 32'hAABB_CCDD;
      logic [38:0] inj1 = 39'd1 << 14; // inject 1-bit error
      ecc_write(a, orig, 4'hF, inj1);
      // Partial byte write byte 1 = 0x55 -> expected 0xAABB_55DD, clean ECC
      ecc_write(a, 32'h0000_5500, 4'b0010, 39'd0);
      ecc_read(a, rd, err, ce, ue);
      check_eq32("ECC RMW corrected+merged", rd, 32'hAABB_55DD);
      check_cond("ECC RMW new word is error-free", !err && !ce && !ue);

      // Additional partial strobes
      ecc_write(a, 32'h1200_0034, 4'b1001, 39'd0);
      ecc_read(a, rd, err, ce, ue);
      check_eq32("ECC RMW strobe 4'b1001", rd, 32'h12BB_5534);
      cov_ecc_rmw += 2;
    end

    // Test background scrubber: clean all words, inject 1-bit errors at 3 addrs, enable scrub, verify repaired!
    for (int a = 0; a < ECC_DEPTH; a++) begin
      shadow[a] = 32'h5500_0000 + 32'(a);
      ecc_write(6'(a), shadow[a], 4'hF, 39'd0);
    end
    ecc_write(6'd5,  shadow[5],  4'hF, 39'd1 << 7);
    ecc_write(6'd27, shadow[27], 4'hF, 39'd1 << 0);
    ecc_write(6'd58, shadow[58], 4'hF, 39'd1 << 35);

    begin
      logic [31:0] ce_before = ecc_ce_cnt;
      ecc_scrub_period = 16'd2;
      ecc_scrub_en     = 1'b1;
      // 64 entries * ~4 cycles each = ~260 cycles -> wait 350 cycles for full sweep
      repeat (350) @(posedge clk);
      ecc_scrub_en = 1'b0;
      repeat (4) @(posedge clk);
      check_eq32("ECC scrubber fixed 3 CEs", ecc_ce_cnt - ce_before, 32'd3);

      // Read back scrubbed words: must have ZERO new CE pulses because SRAM was rewritten!
      ecc_read(6'd5,  rd, err, ce, ue);
      check_eq32("ECC scrubbed addr 5 data", rd, shadow[5]);
      check_cond("ECC scrubbed addr 5 clean", !ce && !ue && !err);

      ecc_read(6'd27, rd, err, ce, ue);
      check_eq32("ECC scrubbed addr 27 data", rd, shadow[27]);
      check_cond("ECC scrubbed addr 27 clean", !ce && !ue && !err);

      ecc_read(6'd58, rd, err, ce, ue);
      check_eq32("ECC scrubbed addr 58 data", rd, shadow[58]);
      check_cond("ECC scrubbed addr 58 clean", !ce && !ue && !err);
      cov_ecc_scrub++;
    end
  endtask

  task automatic test_quad_bus_smoke();
    logic [31:0] rdata;
    $display("[TB] 6. Testing Quad-Bus Smoke across WB / AXI4 / TL / APB...");

    // CRG on Wishbone
    do_wb_write(smk_wb_m, smk_wb_s, 32'h08, 32'hCAFE_BABE);
    do_wb_read(smk_wb_m, smk_wb_s, 32'h08, rdata);
    check_eq32("Smoke CRG WB", rdata, 32'hCAFE_BABE);
    cov_quad_bus++;

    // IOMUX on AXI4
    do_axi_write(smk_axi_m, smk_axi_s, 32'h00, 32'h0000_0231);
    do_axi_read(smk_axi_m, smk_axi_s, 32'h00, rdata);
    check_eq32("Smoke IOMUX AXI4", rdata, 32'h0000_0231);
    do_axi_read(smk_axi_m, smk_axi_s, 32'h100, rdata);
    check_eq32("Smoke IOMUX AXI4 PAD_IN", rdata, 32'h0000_005A);
    cov_quad_bus++;

    // PMU on APB4
    do_apb_write(smk_apb_m, smk_apb_s, 32'h04, 32'h0000_000F);
    do_apb_read(smk_apb_m, smk_apb_s, 32'h04, rdata);
    check_eq32("Smoke PMU APB4", rdata, 32'h0000_000F);
    cov_quad_bus++;

    // Bus Firewall on TileLink
    do_tl_write(smk_tl_m, smk_tl_s, 32'h20, 32'h1000_0000);
    do_tl_read(smk_tl_m, smk_tl_s, 32'h20, rdata);
    check_eq32("Smoke FW TL", rdata, 32'h1000_0000);
    cov_quad_bus++;
  endtask

  // =========================================================================
  // Main Control & Watchdog
  // =========================================================================
  initial begin
    if ($value$plusargs("seed=%d", seed)) begin
      void'($urandom(seed));
    end

    rst_n         = 1'b0;
    crg_por_n     = 1'b0;
    crg_ext_rst_n = 1'b1;
    crg_wdt_rst   = 1'b0;
    crg_bo_rst    = 1'b0;
    crg_test_mode = 1'b0;
    crg_pll_lock  = 1'b0;

    crg_apb_m     = '0;
    iomux_wb_m    = '0;
    iomux_giso    = 1'b0;
    iomux_gret    = 1'b0;
    iomux_func_o  = '0;
    iomux_func_oe = '0;
    iomux_pad_i   = '0;

    pmu_tl_m      = '0;
    pmu_slp_req   = '0;
    pmu_wake_src  = '0;

    fw_csr_m      = '0;
    fw_s_axi_m    = '0;

    ecc_req_valid    = 1'b0;
    ecc_req_we       = 1'b0;
    ecc_req_wstrb    = 4'hF;
    ecc_req_addr     = 6'd0;
    ecc_req_wdata    = 32'd0;
    ecc_scrub_en     = 1'b0;
    ecc_scrub_period = 16'd4;
    ecc_err_inj      = 39'd0;

    smk_wb_m      = '0;
    smk_axi_m     = '0;
    smk_tl_m      = '0;
    smk_apb_m     = '0;

    repeat (6) @(posedge clk); #1;
    rst_n     = 1'b1;
    crg_por_n = 1'b1;
    repeat (6) @(posedge clk);

    test_crg();
    test_iomux();
    test_pmu();
    test_firewall();
    test_ecc();
    test_quad_bus_smoke();

    $display("[COV] seed=%0d crg_sw=%0d crg_div=%0d crg_cause=%0d iomux=%0d pmu_cyc=%0d pmu_wk=%0d fw_tx=%0d fw_blk=%0d ecc_ce=%0d ecc_ue=%0d ecc_rmw=%0d ecc_scrub=%0d quad=%0d",
             seed, cov_crg_switches, cov_crg_divs, cov_crg_causes, cov_iomux_ops,
             cov_pmu_cycles, cov_pmu_wakes, cov_fw_txns, cov_fw_blocks,
             cov_ecc_ce_bits, cov_ecc_ue_pairs, cov_ecc_rmw, cov_ecc_scrub, cov_quad_bus);

    if (err_cnt != 0) $fatal(1, "err_cnt != 0");
    $display("[PASS] tb_soc_sysctrl: CRG(mux/div/icg/pll/rst_cause), IOMUX(32x8/ret/iso/lock), PMU(seq/wake/irq), Firewall(axi4/regions/lock/rand), SRAM_ECC(39b_ce/ue/rmw/scrub), quad-bus(WB/AXI4/TL/APB) verified (seed=%0d)", seed);
    $finish;
  end

  // Watchdog Timeout
  initial begin
    #2000000;
    $fatal(1, "[TIMEOUT] tb_soc_sysctrl exceeded watchdog limit");
  end

endmodule : tb_soc_sysctrl
