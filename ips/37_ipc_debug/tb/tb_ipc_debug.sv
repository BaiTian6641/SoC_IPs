// SPDX-License-Identifier: Apache-2.0
// tb_ipc_debug.sv - Self-Checking Verification Suite for 37_ipc_debug
//   1. ipc_mailbox_top : 32-lock atomic test-and-set contention/release, multi-core doorbells,
//                        message FIFOs & per-core irq_o, quad-bus WB/AXI4/TL/APB
//   2. riscv_dm_top    : DMI dmactive reset, 4-hart halt/resume/ndmreset, abstract commands +
//                        postincrement + cmderr, SBA 8/16/32-bit R/W + sbautoincrement +
//                        sbreadonaddr + bus error capture against tb_sim_ram, quad-bus CSR monitor
//   3. adc_pdm_top     : Behavioral SAR-ADC macro (scan, 16x oversampling exact avg, ext_trgo_i,
//                        AWD high/low IRQs, DMA); 1-bit sigma-delta modulator -> Sinc3 CIC
//                        (DC gain, HPF zero-offset convergence, sine SNR, stereo L/R), quad-bus

`timescale 1ns/1ps

module tb_ipc_debug;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  // ---------------------------------------------------------------------------
  // Clock, Reset, Seed & Watchdog
  // ---------------------------------------------------------------------------
  logic clk, rst_n;
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  int unsigned seed = 1;
  initial begin
    void'($value$plusargs("seed=%d", seed));
    void'($urandom(seed));
  end

  // Watchdog timeout
  initial begin
    #20_000_000;
    $fatal(1, "[FAIL] tb_ipc_debug: Watchdog timeout!");
  end

  // Coverage counters
  int cov_mbox_locks_tested     = 0;
  int cov_mbox_db_irqs          = 0;
  int cov_mbox_fifo_msgs        = 0;
  int cov_dm_hart_ops           = 0;
  int cov_dm_abs_cmds           = 0;
  int cov_dm_sba_xfers          = 0;
  int cov_dm_sba_errs           = 0;
  int cov_adc_scans             = 0;
  int cov_adc_awd_trips         = 0;
  int cov_pdm_frames            = 0;
  int cov_quad_bus_checks       = 0;

  // ---------------------------------------------------------------------------
  // Include standard WB/AXI/TL BFM tasks + APB4 BFM tasks
  // ---------------------------------------------------------------------------
  `include "tb_bfm_tasks.svh"

  task automatic do_apb_write_err(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, input logic [31:0] data,
    input logic [3:0] strb, output logic err_o
  );
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
    err_o = s.pslverr;
    #1;
    m = '0;
  endtask

  task automatic do_apb_read_err(
    ref apb_m2s_t m, const ref apb_s2m_t s,
    input logic [31:0] addr, output logic [31:0] data, output logic err_o
  );
    @(posedge clk); #1;
    m         = '0;
    m.paddr   = addr;
    m.psel    = 1'b1;
    m.penable = 1'b0;
    m.pwrite  = 1'b0;
    m.pstrb   = 4'b0000;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    data  = s.prdata;
    err_o = s.pslverr;
    #1;
    m = '0;
  endtask

  task automatic do_apb_write(ref apb_m2s_t m, const ref apb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    logic e;
    do_apb_write_err(m, s, addr, data, 4'hF, e);
    if (e) $fatal(1, "[FAIL] APB unexpected pslverr on write addr=0x%08x", addr);
  endtask

  task automatic do_apb_read(ref apb_m2s_t m, const ref apb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    logic e;
    do_apb_read_err(m, s, addr, data, e);
    if (e) $fatal(1, "[FAIL] APB unexpected pslverr on read addr=0x%08x", addr);
  endtask

  // ===========================================================================
  // SECTION 1: IPC Mailbox & Spinlocks DUTs (Primary APB + WB/AXI4/TL Smoke)
  // ===========================================================================
  localparam int unsigned MBOX_CORES = 4;
  localparam int unsigned MBOX_CH    = 4;
  localparam int unsigned MBOX_LOCKS = 32;

  apb_m2s_t  mbox_apb_m; apb_s2m_t  mbox_apb_s;
  wb_m2s_t   mbox_wb_m;  wb_s2m_t   mbox_wb_s;
  axi4_m2s_t mbox_axi_m; axi4_s2m_t mbox_axi_s;
  tl_m2s_t   mbox_tl_m;  tl_s2m_t   mbox_tl_s;
  logic [MBOX_CORES-1:0] mbox_irq_apb, mbox_irq_wb, mbox_irq_axi, mbox_irq_tl;

  ipc_mailbox_top #(.BUS_TYPE(BUS_APB), .NUM_CORES(MBOX_CORES), .NUM_CH(MBOX_CH), .NUM_LOCKS(MBOX_LOCKS)) u_mbox_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(),
    .apb_i(mbox_apb_m), .apb_o(mbox_apb_s), .irq_o(mbox_irq_apb)
  );
  ipc_mailbox_top #(.BUS_TYPE(BUS_WISHBONE)) u_mbox_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(mbox_wb_m), .wb_o(mbox_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(),
    .apb_i('0), .apb_o(), .irq_o(mbox_irq_wb)
  );
  ipc_mailbox_top #(.BUS_TYPE(BUS_AXI4)) u_mbox_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(mbox_axi_m), .axi_o(mbox_axi_s), .tl_i('0), .tl_o(),
    .apb_i('0), .apb_o(), .irq_o(mbox_irq_axi)
  );
  ipc_mailbox_top #(.BUS_TYPE(BUS_TILELINK)) u_mbox_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(mbox_tl_m), .tl_o(mbox_tl_s),
    .apb_i('0), .apb_o(), .irq_o(mbox_irq_tl)
  );

  // ===========================================================================
  // SECTION 2: RISC-V Debug Module DUTs (Primary AXI4 + WB/TL/APB)
  // ===========================================================================
  localparam int unsigned DM_HARTS = 4;

  // Primary DM (AXI4 slave + AXI4 SBA master)
  axi4_m2s_t dm_s_axi_m; axi4_s2m_t dm_s_axi_s;
  axi4_m2s_t dm_m_axi_m; axi4_s2m_t dm_m_axi_s;
  logic      dm_dmi_req_v, dm_dmi_req_r;
  logic [6:0]  dm_dmi_req_a;
  logic [1:0]  dm_dmi_req_op;
  logic [31:0] dm_dmi_req_d;
  logic      dm_dmi_rsp_v, dm_dmi_rsp_r;
  logic [31:0] dm_dmi_rsp_d;
  logic [1:0]  dm_dmi_rsp_op;
  logic [DM_HARTS-1:0] dm_halted, dm_res_ack, dm_halt_req, dm_res_req;
  logic      dm_ndmreset, dm_irq;
  logic      dm_abs_req_v, dm_abs_req_w;
  logic [1:0]  dm_abs_hartsel;
  logic [15:0] dm_abs_regno;
  logic [31:0] dm_abs_wdata;
  logic        dm_abs_rsp_v, dm_abs_rsp_err;
  logic [31:0] dm_abs_rsp_rdata;

  riscv_dm_top #(.BUS_TYPE(BUS_AXI4), .NUM_HARTS(DM_HARTS)) u_dm_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i(dm_s_axi_m), .s_axi_o(dm_s_axi_s),
    .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(dm_m_axi_m), .m_axi_i(dm_m_axi_s),
    .m_tl_o(), .m_tl_i('0),
    .dmi_req_valid_i(dm_dmi_req_v), .dmi_req_ready_o(dm_dmi_req_r),
    .dmi_req_addr_i(dm_dmi_req_a), .dmi_req_op_i(dm_dmi_req_op), .dmi_req_data_i(dm_dmi_req_d),
    .dmi_rsp_valid_o(dm_dmi_rsp_v), .dmi_rsp_ready_i(dm_dmi_rsp_r),
    .dmi_rsp_data_o(dm_dmi_rsp_d), .dmi_rsp_op_o(dm_dmi_rsp_op),
    .halted_i(dm_halted), .resume_ack_i(dm_res_ack),
    .halt_req_o(dm_halt_req), .resume_req_o(dm_res_req), .ndmreset_o(dm_ndmreset),
    .abs_req_valid_o(dm_abs_req_v), .abs_req_write_o(dm_abs_req_w),
    .abs_req_hartsel_o(dm_abs_hartsel), .abs_req_regno_o(dm_abs_regno), .abs_req_wdata_o(dm_abs_wdata),
    .abs_rsp_valid_i(dm_abs_rsp_v), .abs_rsp_rdata_i(dm_abs_rsp_rdata), .abs_rsp_err_i(dm_abs_rsp_err),
    .irq_o(dm_irq)
  );

  // Address-decoded RAM + Fault Slave for Primary DM AXI4 SBA Master
  axi4_m2s_t dm_ram_axi_m;
  axi4_s2m_t dm_ram_axi_s;
  wire dm_axi_is_err_aw = dm_m_axi_m.awvalid && (dm_m_axi_m.awaddr[31] == 1'b1);
  wire dm_axi_is_err_ar = dm_m_axi_m.arvalid && (dm_m_axi_m.araddr[31] == 1'b1);
  logic dm_err_b_q, dm_err_r_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dm_err_b_q <= 1'b0;
      dm_err_r_q <= 1'b0;
    end else begin
      if (dm_axi_is_err_aw && dm_m_axi_m.wvalid) dm_err_b_q <= 1'b1;
      else if (dm_err_b_q && dm_m_axi_m.bready)  dm_err_b_q <= 1'b0;
      if (dm_axi_is_err_ar)                      dm_err_r_q <= 1'b1;
      else if (dm_err_r_q && dm_m_axi_m.rready)  dm_err_r_q <= 1'b0;
    end
  end

  always_comb begin
    dm_ram_axi_m = dm_m_axi_m;
    if (dm_m_axi_m.awaddr[31]) dm_ram_axi_m.awvalid = 1'b0;
    if (dm_m_axi_m.awaddr[31]) dm_ram_axi_m.wvalid  = 1'b0;
    if (dm_m_axi_m.araddr[31]) dm_ram_axi_m.arvalid = 1'b0;

    dm_m_axi_s = dm_ram_axi_s;
    if (dm_axi_is_err_aw) dm_m_axi_s.awready = 1'b1;
    if (dm_axi_is_err_aw) dm_m_axi_s.wready  = 1'b1;
    if (dm_axi_is_err_ar) dm_m_axi_s.arready = 1'b1;
    if (dm_err_b_q) begin
      dm_m_axi_s.bvalid = 1'b1;
      dm_m_axi_s.bresp  = AXI_RESP_SLVERR;
    end
    if (dm_err_r_q) begin
      dm_m_axi_s.rvalid = 1'b1;
      dm_m_axi_s.rresp  = AXI_RESP_SLVERR;
      dm_m_axi_s.rlast  = 1'b1;
      dm_m_axi_s.rdata  = 32'hDEAD_FA11;
    end
  end

  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(1024)) u_dm_ram_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(dm_ram_axi_m), .axi_o(dm_ram_axi_s), .tl_i('0), .tl_o()
  );

  // Secondary DM instances for WB, TL, APB quad-bus + SBA verification
  wb_m2s_t dm_s_wb_m, dm_m_wb_m; wb_s2m_t dm_s_wb_s, dm_m_wb_s;
  tl_m2s_t dm_s_tl_m, dm_m_tl_m; tl_s2m_t dm_s_tl_s, dm_m_tl_s;
  apb_m2s_t dm_s_apb_m; apb_s2m_t dm_s_apb_s;
  axi4_m2s_t dm_apb_m_axi_m; axi4_s2m_t dm_apb_m_axi_s;

  logic      dm_wb_dmi_rv, dm_wb_dmi_rr, dm_wb_dmi_sv, dm_wb_dmi_sr;
  logic [6:0]  dm_wb_dmi_a; logic [1:0] dm_wb_dmi_op, dm_wb_dmi_sop; logic [31:0] dm_wb_dmi_wd, dm_wb_dmi_rd;

  logic      dm_tl_dmi_rv, dm_tl_dmi_rr, dm_tl_dmi_sv, dm_tl_dmi_sr;
  logic [6:0]  dm_tl_dmi_a; logic [1:0] dm_tl_dmi_op, dm_tl_dmi_sop; logic [31:0] dm_tl_dmi_wd, dm_tl_dmi_rd;

  logic      dm_apb_dmi_rv, dm_apb_dmi_rr, dm_apb_dmi_sv, dm_apb_dmi_sr;
  logic [6:0]  dm_apb_dmi_a; logic [1:0] dm_apb_dmi_op, dm_apb_dmi_sop; logic [31:0] dm_apb_dmi_wd, dm_apb_dmi_rd;

  riscv_dm_top #(.BUS_TYPE(BUS_WISHBONE)) u_dm_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i(dm_s_wb_m), .s_wb_o(dm_s_wb_s), .s_axi_i('0), .s_axi_o(),
    .s_tl_i('0), .s_tl_o(), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(dm_m_wb_m), .m_wb_i(dm_m_wb_s), .m_axi_o(), .m_axi_i('0), .m_tl_o(), .m_tl_i('0),
    .dmi_req_valid_i(dm_wb_dmi_rv), .dmi_req_ready_o(dm_wb_dmi_rr),
    .dmi_req_addr_i(dm_wb_dmi_a), .dmi_req_op_i(dm_wb_dmi_op), .dmi_req_data_i(dm_wb_dmi_wd),
    .dmi_rsp_valid_o(dm_wb_dmi_sv), .dmi_rsp_ready_i(dm_wb_dmi_sr),
    .dmi_rsp_data_o(dm_wb_dmi_rd), .dmi_rsp_op_o(dm_wb_dmi_sop),
    .halted_i(4'b0), .resume_ack_i(4'b0), .halt_req_o(), .resume_req_o(), .ndmreset_o(),
    .abs_req_valid_o(), .abs_req_write_o(), .abs_req_hartsel_o(), .abs_req_regno_o(), .abs_req_wdata_o(),
    .abs_rsp_valid_i(1'b0), .abs_rsp_rdata_i(32'd0), .abs_rsp_err_i(1'b0), .irq_o()
  );
  tb_sim_ram #(.BUS_TYPE(BUS_WISHBONE), .WORDS(256)) u_dm_ram_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(dm_m_wb_m), .wb_o(dm_m_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o()
  );

  riscv_dm_top #(.BUS_TYPE(BUS_TILELINK)) u_dm_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(),
    .s_tl_i(dm_s_tl_m), .s_tl_o(dm_s_tl_s), .s_apb_i('0), .s_apb_o(),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(), .m_axi_i('0), .m_tl_o(dm_m_tl_m), .m_tl_i(dm_m_tl_s),
    .dmi_req_valid_i(dm_tl_dmi_rv), .dmi_req_ready_o(dm_tl_dmi_rr),
    .dmi_req_addr_i(dm_tl_dmi_a), .dmi_req_op_i(dm_tl_dmi_op), .dmi_req_data_i(dm_tl_dmi_wd),
    .dmi_rsp_valid_o(dm_tl_dmi_sv), .dmi_rsp_ready_i(dm_tl_dmi_sr),
    .dmi_rsp_data_o(dm_tl_dmi_rd), .dmi_rsp_op_o(dm_tl_dmi_sop),
    .halted_i(4'b0), .resume_ack_i(4'b0), .halt_req_o(), .resume_req_o(), .ndmreset_o(),
    .abs_req_valid_o(), .abs_req_write_o(), .abs_req_hartsel_o(), .abs_req_regno_o(), .abs_req_wdata_o(),
    .abs_rsp_valid_i(1'b0), .abs_rsp_rdata_i(32'd0), .abs_rsp_err_i(1'b0), .irq_o()
  );
  tb_sim_ram #(.BUS_TYPE(BUS_TILELINK), .WORDS(256)) u_dm_ram_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(dm_m_tl_m), .tl_o(dm_m_tl_s)
  );

  riscv_dm_top #(.BUS_TYPE(BUS_APB)) u_dm_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .s_wb_i('0), .s_wb_o(), .s_axi_i('0), .s_axi_o(),
    .s_tl_i('0), .s_tl_o(), .s_apb_i(dm_s_apb_m), .s_apb_o(dm_s_apb_s),
    .m_wb_o(), .m_wb_i('0), .m_axi_o(dm_apb_m_axi_m), .m_axi_i(dm_apb_m_axi_s), .m_tl_o(), .m_tl_i('0),
    .dmi_req_valid_i(dm_apb_dmi_rv), .dmi_req_ready_o(dm_apb_dmi_rr),
    .dmi_req_addr_i(dm_apb_dmi_a), .dmi_req_op_i(dm_apb_dmi_op), .dmi_req_data_i(dm_apb_dmi_wd),
    .dmi_rsp_valid_o(dm_apb_dmi_sv), .dmi_rsp_ready_i(dm_apb_dmi_sr),
    .dmi_rsp_data_o(dm_apb_dmi_rd), .dmi_rsp_op_o(dm_apb_dmi_sop),
    .halted_i(4'b0), .resume_ack_i(4'b0), .halt_req_o(), .resume_req_o(), .ndmreset_o(),
    .abs_req_valid_o(), .abs_req_write_o(), .abs_req_hartsel_o(), .abs_req_regno_o(), .abs_req_wdata_o(),
    .abs_rsp_valid_i(1'b0), .abs_rsp_rdata_i(32'd0), .abs_rsp_err_i(1'b0), .irq_o()
  );
  tb_sim_ram #(.BUS_TYPE(BUS_AXI4), .WORDS(256)) u_dm_ram_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(dm_apb_m_axi_m), .axi_o(dm_apb_m_axi_s), .tl_i('0), .tl_o()
  );

  // Behavioral Multi-Hart Model for Primary DM (Run-control + Abstract GPRs)
  logic [31:0] hart_regs [0:DM_HARTS-1][0:31];
  logic        hart_abs_inject_err = 1'b0;
  logic [1:0]  hart_conv_wait_q [0:DM_HARTS-1];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dm_halted        <= 4'b0;
      dm_res_ack       <= 4'b0;
      dm_abs_rsp_v     <= 1'b0;
      dm_abs_rsp_err   <= 1'b0;
      dm_abs_rsp_rdata <= 32'd0;
      for (int h = 0; h < DM_HARTS; h++) begin
        hart_conv_wait_q[h] <= 2'd0;
        for (int r = 0; r < 32; r++) begin
          hart_regs[h][r] <= 32'(h * 32'h100 + r);
        end
      end
    end else begin
      dm_res_ack   <= 4'b0;
      dm_abs_rsp_v <= 1'b0;
      for (int h = 0; h < DM_HARTS; h++) begin
        if (dm_ndmreset) begin
          dm_halted[h] <= dm_halt_req[h];
        end else begin
          if (dm_halt_req[h]) dm_halted[h] <= 1'b1;
          else if (dm_res_req[h] && dm_halted[h]) begin
            dm_halted[h]  <= 1'b0;
            dm_res_ack[h] <= 1'b1;
          end
        end
      end

      if (dm_abs_req_v && !dm_abs_rsp_v) begin
        dm_abs_rsp_v <= 1'b1;
        if (hart_abs_inject_err || (dm_abs_regno < 16'h1000) || (dm_abs_regno > 16'h101F)) begin
          dm_abs_rsp_err   <= 1'b1;
          dm_abs_rsp_rdata <= 32'd0;
        end else begin
          automatic logic [4:0] ridx = dm_abs_regno[4:0];
          dm_abs_rsp_err <= 1'b0;
          if (dm_abs_req_w) begin
            hart_regs[dm_abs_hartsel][ridx] <= dm_abs_wdata;
            dm_abs_rsp_rdata                <= 32'd0;
          end else begin
            dm_abs_rsp_rdata <= hart_regs[dm_abs_hartsel][ridx];
          end
        end
      end
    end
  end

  // ===========================================================================
  // SECTION 3: SAR-ADC + PDM-to-PCM DUTs (Primary WB + AXI4/TL/APB)
  // ===========================================================================
  wb_m2s_t   adc_wb_m;  wb_s2m_t   adc_wb_s;
  axi4_m2s_t adc_axi_m; axi4_s2m_t adc_axi_s;
  tl_m2s_t   adc_tl_m;  tl_s2m_t   adc_tl_s;
  apb_m2s_t  adc_apb_m; apb_s2m_t  adc_apb_s;

  logic        adc_start, adc_eoc, ext_trgo;
  logic [3:0]  adc_ch_sel;
  logic [11:0] adc_data;
  logic        pdm_clk, pdm_data;
  logic        dma_adc_req, dma_pdm_req, awd_irq, adc_irq;

  adc_pdm_top #(.BUS_TYPE(BUS_WISHBONE)) u_adc_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i(adc_wb_m), .wb_o(adc_wb_s), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .adc_start_o(adc_start), .adc_ch_sel_o(adc_ch_sel), .adc_eoc_i(adc_eoc), .adc_data_i(adc_data),
    .ext_trgo_i(ext_trgo), .pdm_clk_o(pdm_clk), .pdm_data_i(pdm_data),
    .dma_adc_req_o(dma_adc_req), .dma_pdm_req_o(dma_pdm_req), .awd_irq_o(awd_irq), .irq_o(adc_irq)
  );

  adc_pdm_top #(.BUS_TYPE(BUS_AXI4)) u_adc_axi (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i(adc_axi_m), .axi_o(adc_axi_s), .tl_i('0), .tl_o(), .apb_i('0), .apb_o(),
    .adc_start_o(), .adc_ch_sel_o(), .adc_eoc_i(1'b0), .adc_data_i(12'd0),
    .ext_trgo_i(1'b0), .pdm_clk_o(), .pdm_data_i(1'b0),
    .dma_adc_req_o(), .dma_pdm_req_o(), .awd_irq_o(), .irq_o()
  );

  adc_pdm_top #(.BUS_TYPE(BUS_TILELINK)) u_adc_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i(adc_tl_m), .tl_o(adc_tl_s), .apb_i('0), .apb_o(),
    .adc_start_o(), .adc_ch_sel_o(), .adc_eoc_i(1'b0), .adc_data_i(12'd0),
    .ext_trgo_i(1'b0), .pdm_clk_o(), .pdm_data_i(1'b0),
    .dma_adc_req_o(), .dma_pdm_req_o(), .awd_irq_o(), .irq_o()
  );

  adc_pdm_top #(.BUS_TYPE(BUS_APB)) u_adc_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_i('0), .wb_o(), .axi_i('0), .axi_o(), .tl_i('0), .tl_o(), .apb_i(adc_apb_m), .apb_o(adc_apb_s),
    .adc_start_o(), .adc_ch_sel_o(), .adc_eoc_i(1'b0), .adc_data_i(12'd0),
    .ext_trgo_i(1'b0), .pdm_clk_o(), .pdm_data_i(1'b0),
    .dma_adc_req_o(), .dma_pdm_req_o(), .awd_irq_o(), .irq_o()
  );

  // Behavioral SAR-ADC Macro Model (3-cycle conversion, per-channel programmable values)
  logic [11:0] sar_ch_base [0:15];
  logic        sar_use_ramp = 1'b0;
  logic [2:0]  sar_cnt_q;
  logic [3:0]  sar_sample_idx [0:15];
  logic        sar_busy_q;
  logic [3:0]  sar_lat_ch_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sar_busy_q <= 1'b0;
      sar_cnt_q  <= 3'd0;
      adc_eoc    <= 1'b0;
      adc_data   <= 12'd0;
      sar_lat_ch_q <= 4'd0;
      for (int c = 0; c < 16; c++) sar_sample_idx[c] <= 4'd0;
    end else begin
      adc_eoc <= 1'b0;
      if (adc_start && !sar_busy_q) begin
        sar_busy_q   <= 1'b1;
        sar_cnt_q    <= 3'd2;
        sar_lat_ch_q <= adc_ch_sel;
      end else if (sar_busy_q) begin
        if (sar_cnt_q == 3'd0) begin
          sar_busy_q <= 1'b0;
          adc_eoc    <= 1'b1;
          if (sar_use_ramp) begin
            adc_data <= sar_ch_base[sar_lat_ch_q] + {8'd0, sar_sample_idx[sar_lat_ch_q]};
          end else begin
            adc_data <= sar_ch_base[sar_lat_ch_q];
          end
          sar_sample_idx[sar_lat_ch_q] <= sar_sample_idx[sar_lat_ch_q] + 4'd1;
        end else begin
          sar_cnt_q <= sar_cnt_q - 3'd1;
        end
      end
    end
  end

  // Behavioral Independent 1-bit Sigma-Delta PDM Modulator (Mono / Stereo)
  // Modes:
  //   0: const_bits (pdm_const_l / pdm_const_r)
  //   1: periodic pattern (pdm_pat_l / pdm_pat_r)
  //   2: 1st-order Sigma-Delta Modulator driven by real target_l / target_r in [-1.0, +1.0]
  int     pdm_mode = 0;
  logic   pdm_const_l = 1'b1, pdm_const_r = 1'b0;
  logic [3:0] pdm_pat_l = 4'b1110, pdm_pat_r = 4'b0001;
  int     pdm_pat_idx_l, pdm_pat_idx_r;
  real    pdm_dc_l, pdm_dc_r;
  real    pdm_amp_l, pdm_amp_r;
  real    pdm_freq_l, pdm_freq_r; // normalized cycles per PDM bit
  real    sdm_acc_l, sdm_acc_r;
  int     sdm_tick_l, sdm_tick_r;
  logic   pdm_bit_l_q, pdm_bit_r_q;
  logic   pdm_clk_d;

  function automatic logic step_sdm(
    inout real acc,
    inout int  tick,
    input real dc,
    input real amp,
    input real freq
  );
    real x, y;
    logic b;
    x = dc + amp * $sin(6.283185307179586 * freq * real'(tick));
    tick = tick + 1;
    acc = acc + x;
    if (acc >= 0.0) begin
      b = 1'b1;
      y = 1.0;
    end else begin
      b = 1'b0;
      y = -1.0;
    end
    acc = acc - y;
    return b;
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pdm_clk_d     <= 1'b0;
      pdm_bit_l_q   <= 1'b1;
      pdm_bit_r_q   <= 1'b0;
      pdm_pat_idx_l <= 0;
      pdm_pat_idx_r <= 0;
      sdm_acc_l     <= 0.0;
      sdm_acc_r     <= 0.0;
      sdm_tick_l    <= 0;
      sdm_tick_r    <= 0;
    end else begin
      pdm_clk_d <= pdm_clk;
      if (pdm_mode != 2) begin
        sdm_acc_l  <= 0.0;
        sdm_acc_r  <= 0.0;
        sdm_tick_l <= 0;
        sdm_tick_r <= 0;
      end
      // On rising edge of pdm_clk: update Ch0 (Left) so it's valid before falling edge!
      if (pdm_clk && !pdm_clk_d) begin
        if (pdm_mode == 0) pdm_bit_l_q <= pdm_const_l;
        else if (pdm_mode == 1) begin
          pdm_bit_l_q   <= pdm_pat_l[pdm_pat_idx_l[1:0]];
          pdm_pat_idx_l <= (pdm_pat_idx_l + 1) & 3;
        end else begin
          pdm_bit_l_q <= step_sdm(sdm_acc_l, sdm_tick_l, pdm_dc_l, pdm_amp_l, pdm_freq_l);
        end
      end
      // On falling edge of pdm_clk: update Ch1 (Right) so it's valid before rising edge!
      if (!pdm_clk && pdm_clk_d) begin
        if (pdm_mode == 0) pdm_bit_r_q <= pdm_const_r;
        else if (pdm_mode == 1) begin
          pdm_bit_r_q   <= pdm_pat_r[pdm_pat_idx_r[1:0]];
          pdm_pat_idx_r <= (pdm_pat_idx_r + 1) & 3;
        end else begin
          pdm_bit_r_q <= step_sdm(sdm_acc_r, sdm_tick_r, pdm_dc_r, pdm_amp_r, pdm_freq_r);
        end
      end
    end
  end

  // Multiplexed stereo PDM wire: when pdm_clk==1 -> Ch0; when pdm_clk==0 -> Ch1
  assign pdm_data = pdm_clk ? pdm_bit_l_q : pdm_bit_r_q;

  // ===========================================================================
  // Helper DMI Transaction Tasks
  // ===========================================================================
  task automatic do_dmi_op(
    input  logic [6:0]  addr,
    input  logic [1:0]  op,
    input  logic [31:0] wdata,
    output logic [31:0] rdata,
    output logic [1:0]  rsp_op
  );
    @(posedge clk); #1;
    dm_dmi_req_v  = 1'b1;
    dm_dmi_req_a  = addr;
    dm_dmi_req_op = op;
    dm_dmi_req_d  = wdata;
    dm_dmi_rsp_r  = 1'b1;
    do @(posedge clk); while (!dm_dmi_req_r);
    #1; dm_dmi_req_v = 1'b0;
    while (!dm_dmi_rsp_v) @(posedge clk);
    rdata  = dm_dmi_rsp_d;
    rsp_op = dm_dmi_rsp_op;
    #1; dm_dmi_rsp_r = 1'b0;
  endtask

  task automatic dmi_wr(input logic [6:0] addr, input logic [31:0] data);
    logic [31:0] dummy;
    logic [1:0]  op_rsp;
    do_dmi_op(addr, 2'd2, data, dummy, op_rsp);
    if (op_rsp != 2'd0) $fatal(1, "[FAIL] DMI write failed addr=0x%02x op_rsp=%0d", addr, op_rsp);
  endtask

  task automatic dmi_rd(input logic [6:0] addr, output logic [31:0] data);
    logic [1:0] op_rsp;
    do_dmi_op(addr, 2'd1, 32'd0, data, op_rsp);
    if (op_rsp != 2'd0) $fatal(1, "[FAIL] DMI read failed addr=0x%02x op_rsp=%0d", addr, op_rsp);
  endtask

  // Generic DMI op for secondary DM tops (WB=0, TL=1, APB=2)
  task automatic dmi_sec_wr(input int which, input logic [6:0] addr, input logic [31:0] data);
    logic [1:0] sop;
    @(posedge clk); #1;
    if (which == 0) begin
      dm_wb_dmi_rv = 1'b1; dm_wb_dmi_a = addr; dm_wb_dmi_op = 2'd2; dm_wb_dmi_wd = data; dm_wb_dmi_sr = 1'b1;
      do @(posedge clk); while (!dm_wb_dmi_rr);
      #1; dm_wb_dmi_rv = 1'b0;
      while (!dm_wb_dmi_sv) @(posedge clk);
      sop = dm_wb_dmi_sop; #1; dm_wb_dmi_sr = 1'b0;
    end else if (which == 1) begin
      dm_tl_dmi_rv = 1'b1; dm_tl_dmi_a = addr; dm_tl_dmi_op = 2'd2; dm_tl_dmi_wd = data; dm_tl_dmi_sr = 1'b1;
      do @(posedge clk); while (!dm_tl_dmi_rr);
      #1; dm_tl_dmi_rv = 1'b0;
      while (!dm_tl_dmi_sv) @(posedge clk);
      sop = dm_tl_dmi_sop; #1; dm_tl_dmi_sr = 1'b0;
    end else begin
      dm_apb_dmi_rv = 1'b1; dm_apb_dmi_a = addr; dm_apb_dmi_op = 2'd2; dm_apb_dmi_wd = data; dm_apb_dmi_sr = 1'b1;
      do @(posedge clk); while (!dm_apb_dmi_rr);
      #1; dm_apb_dmi_rv = 1'b0;
      while (!dm_apb_dmi_sv) @(posedge clk);
      sop = dm_apb_dmi_sop; #1; dm_apb_dmi_sr = 1'b0;
    end
    if (sop != 2'd0) $fatal(1, "[FAIL] Secondary DMI write error which=%0d addr=0x%02x", which, addr);
  endtask

  task automatic dmi_sec_rd(input int which, input logic [6:0] addr, output logic [31:0] data);
    logic [1:0] sop;
    @(posedge clk); #1;
    if (which == 0) begin
      dm_wb_dmi_rv = 1'b1; dm_wb_dmi_a = addr; dm_wb_dmi_op = 2'd1; dm_wb_dmi_wd = 32'd0; dm_wb_dmi_sr = 1'b1;
      do @(posedge clk); while (!dm_wb_dmi_rr);
      #1; dm_wb_dmi_rv = 1'b0;
      while (!dm_wb_dmi_sv) @(posedge clk);
      data = dm_wb_dmi_rd; sop = dm_wb_dmi_sop; #1; dm_wb_dmi_sr = 1'b0;
    end else if (which == 1) begin
      dm_tl_dmi_rv = 1'b1; dm_tl_dmi_a = addr; dm_tl_dmi_op = 2'd1; dm_tl_dmi_wd = 32'd0; dm_tl_dmi_sr = 1'b1;
      do @(posedge clk); while (!dm_tl_dmi_rr);
      #1; dm_tl_dmi_rv = 1'b0;
      while (!dm_tl_dmi_sv) @(posedge clk);
      data = dm_tl_dmi_rd; sop = dm_tl_dmi_sop; #1; dm_tl_dmi_sr = 1'b0;
    end else begin
      dm_apb_dmi_rv = 1'b1; dm_apb_dmi_a = addr; dm_apb_dmi_op = 2'd1; dm_apb_dmi_wd = 32'd0; dm_apb_dmi_sr = 1'b1;
      do @(posedge clk); while (!dm_apb_dmi_rr);
      #1; dm_apb_dmi_rv = 1'b0;
      while (!dm_apb_dmi_sv) @(posedge clk);
      data = dm_apb_dmi_rd; sop = dm_apb_dmi_sop; #1; dm_apb_dmi_sr = 1'b0;
    end
    if (sop != 2'd0) $fatal(1, "[FAIL] Secondary DMI read error which=%0d addr=0x%02x", which, addr);
  endtask

  // ===========================================================================
  // TEST 1: IPC Mailbox & Spinlocks
  // ===========================================================================
  task automatic run_test_ipc_mailbox();
    logic [31:0] rd;
    logic        err;
    logic [31:0] expected_locks;

    $display("[TB] --- Test 1: ipc_mailbox_top ---");

    // 1.1 Check CAPS & error response on unmapped address
    do_apb_read(mbox_apb_m, mbox_apb_s, 32'h004, rd);
    if (rd !== 32'h0420_0404) $fatal(1, "[FAIL] MBOX CAPS mismatch: 0x%08x", rd);
    do_apb_read_err(mbox_apb_m, mbox_apb_s, 32'hFFC, rd, err);
    if (!err) $fatal(1, "[FAIL] MBOX expected pslverr on unmapped offset");

    // 1.2 Spinlocks: acquire all 32 locks atomically, verify contention returns 1
    for (int k = 0; k < 32; k++) begin
      do_apb_read(mbox_apb_m, mbox_apb_s, 32'h400 + 32'(k) * 4, rd);
      if (rd !== 32'd0) $fatal(1, "[FAIL] LOCK[%0d] first acquire returned %0d (expected 0)", k, rd);
      do_apb_read(mbox_apb_m, mbox_apb_s, 32'h400 + 32'(k) * 4, rd);
      if (rd !== 32'd1) $fatal(1, "[FAIL] LOCK[%0d] contend read returned %0d (expected 1)", k, rd);
      cov_mbox_locks_tested++;
    end
    do_apb_read(mbox_apb_m, mbox_apb_s, 32'h008, rd);
    if (rd !== 32'hFFFF_FFFF) $fatal(1, "[FAIL] LOCK_STAT_LO expected all 1s, got 0x%08x", rd);

    // Release & randomized contention across 32 locks
    expected_locks = 32'hFFFF_FFFF;
    for (int iter = 0; iter < 64; iter++) begin
      int k = $urandom_range(0, 31);
      if ($urandom_range(0, 1) == 0) begin
        // Release
        do_apb_write(mbox_apb_m, mbox_apb_s, 32'h400 + 32'(k) * 4, 32'd0);
        expected_locks[k] = 1'b0;
      end else begin
        // Attempt acquire
        do_apb_read(mbox_apb_m, mbox_apb_s, 32'h400 + 32'(k) * 4, rd);
        if (rd !== {31'd0, expected_locks[k]}) begin
          $fatal(1, "[FAIL] LOCK[%0d] TAS mismatch: got %0d exp %0d", k, rd, expected_locks[k]);
        end
        expected_locks[k] = 1'b1;
      end
      do_apb_read(mbox_apb_m, mbox_apb_s, 32'h008, rd);
      if (rd !== expected_locks) $fatal(1, "[FAIL] LOCK_STAT_LO mismatch: 0x%08x vs 0x%08x", rd, expected_locks);
    end

    // 1.3 Multi-core Doorbells & Per-Core IRQs
    if (mbox_irq_apb !== 4'b0000) $fatal(1, "[FAIL] Expected initial mbox_irq=0");
    for (int c = 0; c < MBOX_CORES; c++) begin
      logic [31:0] pat = 32'hA500_0001 << c;
      do_apb_write(mbox_apb_m, mbox_apb_s, 32'h040 + 32'(c) * 4, pat);
      #1;
      if (mbox_irq_apb !== (4'd1 << c)) begin
        $fatal(1, "[FAIL] Doorbell IRQ isolation failed core=%0d irq=4'b%04b", c, mbox_irq_apb);
      end
      do_apb_read(mbox_apb_m, mbox_apb_s, 32'h0C0 + 32'(c) * 4, rd);
      if (rd !== pat) $fatal(1, "[FAIL] DB_STAT[%0d] mismatch: 0x%08x", c, rd);
      do_apb_write(mbox_apb_m, mbox_apb_s, 32'h080 + 32'(c) * 4, pat);
      #1;
      if (mbox_irq_apb !== 4'b0000) $fatal(1, "[FAIL] Doorbell IRQ failed to clear core=%0d", c);
      cov_mbox_db_irqs++;
    end

    // 1.4 Message Payload FIFOs & Routing + Overflow/Underflow
    // Route Ch 0 -> Core 2, Ch 1 -> Core 3, push depth=4, check overflow on 5th
    do_apb_write(mbox_apb_m, mbox_apb_s, 32'h208, 32'h01AA_0004); // Ch0: IRQ_EN=1, SRC=0xAA, DST=Core2
    do_apb_write(mbox_apb_m, mbox_apb_s, 32'h218, 32'h01BB_0008); // Ch1: IRQ_EN=1, SRC=0xBB, DST=Core3

    for (int i = 0; i < 4; i++) begin
      do_apb_write(mbox_apb_m, mbox_apb_s, 32'h200, 32'hCAFE_0000 + 32'(i));
      cov_mbox_fifo_msgs++;
    end
    #1;
    if (mbox_irq_apb !== 4'b0100) $fatal(1, "[FAIL] Ch0->Core2 IRQ mismatch: 4'b%04b", mbox_irq_apb);
    // 5th write triggers overflow
    do_apb_write(mbox_apb_m, mbox_apb_s, 32'h200, 32'hDEAD_BEEF);
    do_apb_read(mbox_apb_m, mbox_apb_s, 32'h204, rd);
    if (!rd[1] || !rd[8] || (rd[7:2] !== 6'd4)) begin
      $fatal(1, "[FAIL] Ch0 MBOX_STAT full/ovf mismatch: 0x%08x", rd);
    end
    // Non-destructive peek
    do_apb_read(mbox_apb_m, mbox_apb_s, 32'h20C, rd);
    if (rd !== 32'hCAFE_0000) $fatal(1, "[FAIL] Ch0 MBOX_PEEK mismatch: 0x%08x", rd);
    // Pop 4 words in FIFO order
    for (int i = 0; i < 4; i++) begin
      do_apb_read(mbox_apb_m, mbox_apb_s, 32'h200, rd);
      if (rd !== (32'hCAFE_0000 + 32'(i))) $fatal(1, "[FAIL] Ch0 FIFO pop[%0d]=0x%08x", i, rd);
    end
    #1;
    if (mbox_irq_apb !== 4'b0000) $fatal(1, "[FAIL] Ch0 IRQ should deassert when empty");
    // Underflow check
    do_apb_read(mbox_apb_m, mbox_apb_s, 32'h200, rd);
    do_apb_read(mbox_apb_m, mbox_apb_s, 32'h204, rd);
    if (!rd[0] || !rd[9]) $fatal(1, "[FAIL] Ch0 underflow/empty flag missing: 0x%08x", rd);

    // 1.5 Quad-bus smoke on WB, AXI4, TL instances
    do_wb_read(mbox_wb_m, mbox_wb_s, 32'h400, rd);
    if (rd !== 32'd0) $fatal(1, "[FAIL] MBOX WB LOCK[0] acq failed");
    do_wb_read(mbox_wb_m, mbox_wb_s, 32'h400, rd);
    if (rd !== 32'd1) $fatal(1, "[FAIL] MBOX WB LOCK[0] busy failed");
    do_wb_write(mbox_wb_m, mbox_wb_s, 32'h044, 32'h1); #1;
    if (mbox_irq_wb !== 4'b0010) $fatal(1, "[FAIL] MBOX WB IRQ failed");

    do_axi_read(mbox_axi_m, mbox_axi_s, 32'h404, rd);
    if (rd !== 32'd0) $fatal(1, "[FAIL] MBOX AXI LOCK[1] acq failed");
    do_axi_write(mbox_axi_m, mbox_axi_s, 32'h048, 32'h2); #1;
    if (mbox_irq_axi !== 4'b0100) $fatal(1, "[FAIL] MBOX AXI IRQ failed");

    do_tl_read(mbox_tl_m, mbox_tl_s, 32'h408, rd);
    if (rd !== 32'd0) $fatal(1, "[FAIL] MBOX TL LOCK[2] acq failed");
    do_tl_write(mbox_tl_m, mbox_tl_s, 32'h04C, 32'h4); #1;
    if (mbox_irq_tl !== 4'b1000) $fatal(1, "[FAIL] MBOX TL IRQ failed");
    cov_quad_bus_checks += 4;
  endtask

  // ===========================================================================
  // TEST 2: RISC-V Debug Module (DMI + Run-Control + Abstract + SBA + CSR)
  // ===========================================================================
  task automatic sba_wait_idle(output logic [31:0] sbcs_out);
    logic [31:0] s;
    int guard = 0;
    do begin
      dmi_rd(7'h38, s);
      guard++;
      if (guard > 200) $fatal(1, "[FAIL] SBA busy timeout sbcs=0x%08x", s);
    end while (s[21]);
    sbcs_out = s;
  endtask

  task automatic abs_wait_idle(output logic [31:0] acs_out);
    logic [31:0] a;
    int guard = 0;
    do begin
      dmi_rd(7'h16, a);
      guard++;
      if (guard > 200) $fatal(1, "[FAIL] Abstract busy timeout acs=0x%08x", a);
    end while (a[12]);
    acs_out = a;
  endtask

  task automatic run_test_riscv_dm();
    logic [31:0] rd, sbcs, acs;
    logic [31:0] shadow_ram [0:63];

    $display("[TB] --- Test 2: riscv_dm_top ---");

    // 2.1 dmactive reset check
    dmi_rd(7'h10, rd);
    if (rd[0] !== 1'b0) $fatal(1, "[FAIL] dmactive should reset to 0");
    dmi_wr(7'h10, 32'h0000_0001); // dmactive=1
    dmi_rd(7'h11, rd);
    if (rd[3:0] !== 4'd2 || !rd[7]) $fatal(1, "[FAIL] dmstatus version/auth mismatch: 0x%08x", rd);
    if (!rd[11] || rd[8]) $fatal(1, "[FAIL] dmstatus expected allrunning=1 anyhalted=0: 0x%08x", rd);

    // 2.2 Multi-hart halt / resume & ndmreset
    for (int h = 0; h < DM_HARTS; h++) begin
      // Select hart h + haltreq=1
      dmi_wr(7'h10, (32'(h) << 16) | 32'h8000_0001);
      repeat (2) @(posedge clk);
      dmi_rd(7'h11, rd);
      if (!rd[9] || !rd[8] || rd[10]) $fatal(1, "[FAIL] Hart %0d halt status failed: 0x%08x", h, rd);
      cov_dm_hart_ops++;
    end
    dmi_rd(7'h40, rd); // haltsum0
    if (rd[3:0] !== 4'b1111) $fatal(1, "[FAIL] haltsum0 expected 4'b1111, got 0x%08x", rd);

    // Resume harts 0..3 via hasel=1 + hawindow
    for (int h = 0; h < DM_HARTS; h++) begin
      dmi_wr(7'h10, (32'(h) << 16) | 32'h4000_0001); // haltreq=0, resumereq=1
      repeat (3) @(posedge clk);
      dmi_rd(7'h11, rd);
      if (!rd[17] || !rd[11]) $fatal(1, "[FAIL] Hart %0d resume failed: 0x%08x", h, rd);
      cov_dm_hart_ops++;
    end

    // Nonexistent hart check (hartsel = 5)
    dmi_wr(7'h10, (32'd5 << 16) | 32'h0000_0001);
    dmi_rd(7'h11, rd);
    if (!rd[14] || !rd[15]) $fatal(1, "[FAIL] Expected nonexistent hart flags for hartsel=5");

    // ndmreset + havereset + ackhavereset
    dmi_wr(7'h10, 32'h0000_0003); // dmactive=1, ndmreset=1, hartsel=0
    #1;
    if (!dm_ndmreset) $fatal(1, "[FAIL] ndmreset_o did not assert");
    dmi_wr(7'h10, 32'h1000_0001); // clear ndmreset, ackhavereset=1
    #1;
    if (dm_ndmreset) $fatal(1, "[FAIL] ndmreset_o did not deassert");
    dmi_rd(7'h11, rd);
    if (rd[18] || rd[19]) $fatal(1, "[FAIL] havereset did not clear on ackhavereset");

    // 2.3 Abstract Commands: cmderr=4 (running), cmderr=2 (unsupported), cmderr=3 (exception),
    //     GPR read/write + aarpostincrement + abstractauto
    // Attempt Access Register while hart 0 is running -> cmderr=4
    dmi_wr(7'h17, 32'h0022_1005); // cmdtype=0, size=2(32b), transfer=1, write=0, regno=0x1005
    abs_wait_idle(acs);
    if (acs[10:8] !== 3'd4) $fatal(1, "[FAIL] Expected cmderr=4 when running, got %0d", acs[10:8]);
    dmi_wr(7'h16, 32'h0000_0700); // W1C cmderr

    // Halt hart 1
    dmi_wr(7'h10, (32'd1 << 16) | 32'h8000_0001);
    repeat (2) @(posedge clk);

    // Unsupported aarsize=3 (64-bit) -> cmderr=2
    dmi_wr(7'h17, 32'h0032_1000);
    abs_wait_idle(acs);
    if (acs[10:8] !== 3'd2) $fatal(1, "[FAIL] Expected cmderr=2 for aarsize=3, got %0d", acs[10:8]);
    dmi_wr(7'h16, 32'h0000_0700);

    // Exception from hart -> cmderr=3
    hart_abs_inject_err = 1'b1;
    dmi_wr(7'h17, 32'h0022_1001);
    abs_wait_idle(acs);
    if (acs[10:8] !== 3'd3) $fatal(1, "[FAIL] Expected cmderr=3 on exception, got %0d", acs[10:8]);
    hart_abs_inject_err = 1'b0;
    dmi_wr(7'h16, 32'h0000_0700);

    // Write x5..x7 on hart 1 using aarpostincrement=1 + abstractauto
    dmi_wr(7'h18, 32'h0000_0001); // autoexecdata[0]=1
    dmi_wr(7'h04, 32'h1122_3344); // data0 (autoexec off? wait: autoexec triggers command_q, so set command_q first!)
    dmi_wr(7'h18, 32'h0000_0000);
    dmi_wr(7'h04, 32'h1122_3344);
    dmi_wr(7'h17, 32'h002B_1005); // aarsize=2, postinc=1, xfer=1, write=1, regno=0x1005
    abs_wait_idle(acs);
    if (acs[10:8] !== 3'd0) $fatal(1, "[FAIL] Abstract write x5 failed cmderr=%0d", acs[10:8]);
    dmi_rd(7'h17, rd);
    if (rd[15:0] !== 16'h1006) $fatal(1, "[FAIL] Postincrement failed: regno=0x%04x", rd[15:0]);
    cov_dm_abs_cmds++;

    // Enable autoexecdata[0]=1 and write x6, x7 automatically
    dmi_wr(7'h18, 32'h0000_0001);
    dmi_wr(7'h04, 32'h5566_7788);
    abs_wait_idle(acs);
    dmi_wr(7'h04, 32'h99AA_BBCC);
    abs_wait_idle(acs);
    dmi_wr(7'h18, 32'h0000_0000);
    cov_dm_abs_cmds += 2;

    // Read back x5, x6, x7 via abstract read + postincrement
    dmi_wr(7'h17, 32'h002A_1005); // write=0, postinc=1, regno=0x1005
    abs_wait_idle(acs);
    dmi_rd(7'h04, rd);
    if (rd !== 32'h1122_3344) $fatal(1, "[FAIL] Abstract read x5=0x%08x", rd);
    dmi_wr(7'h18, 32'h0000_0001); // autoexecdata[0] on read
    dmi_rd(7'h04, rd); // returns current data0 and triggers read of x6
    abs_wait_idle(acs);
    dmi_rd(7'h04, rd); // returns x6 and triggers read of x7
    if (rd !== 32'h5566_7788) $fatal(1, "[FAIL] Abstract read x6=0x%08x", rd);
    abs_wait_idle(acs);
    dmi_wr(7'h18, 32'h0000_0000);
    dmi_rd(7'h04, rd);
    if (rd !== 32'h99AA_BBCC) $fatal(1, "[FAIL] Abstract read x7=0x%08x", rd);
    cov_dm_abs_cmds += 3;

    // 2.4 System Bus Access (SBA) 32-bit, 16-bit, 8-bit R/W + sbautoincrement + sbreadonaddr + sbreadondata
    // sbcs: sbaccess=2 (32-bit), sbautoincrement=1
    dmi_wr(7'h38, (32'd2 << 17) | (32'd1 << 16));
    dmi_wr(7'h39, 32'h0000_0000);
    for (int i = 0; i < 16; i++) begin
      logic [31:0] wval = $urandom();
      shadow_ram[i] = wval;
      dmi_wr(7'h3C, wval);
      sba_wait_idle(sbcs);
      if (sbcs[14:12] !== 3'd0) $fatal(1, "[FAIL] SBA 32b write sberror=%0d", sbcs[14:12]);
      cov_dm_sba_xfers++;
    end
    dmi_rd(7'h39, rd);
    if (rd !== 32'h0000_0040) $fatal(1, "[FAIL] SBA autoincrement addr mismatch: 0x%08x", rd);

    // Enable sbreadonaddr=1 and sbreadondata=1, read back all 16 words!
    dmi_wr(7'h38, (32'd1 << 20) | (32'd2 << 17) | (32'd1 << 16) | (32'd1 << 15));
    dmi_wr(7'h39, 32'h0000_0000); // triggers first read
    for (int i = 0; i < 16; i++) begin
      sba_wait_idle(sbcs);
      if (i == 15) begin
        // Disable sbreadondata before final read
        dmi_wr(7'h38, (32'd2 << 17) | (32'd1 << 16));
      end
      dmi_rd(7'h3C, rd);
      if (rd !== shadow_ram[i]) $fatal(1, "[FAIL] SBA 32b read[%0d] got 0x%08x exp 0x%08x", i, rd, shadow_ram[i]);
      cov_dm_sba_xfers++;
    end

    // 16-bit SBA writes & reads (sbaccess=1, sbautoincrement=1)
    dmi_wr(7'h38, (32'd1 << 17) | (32'd1 << 16));
    dmi_wr(7'h39, 32'h0000_0100);
    dmi_wr(7'h3C, 32'h0000_BEEF); sba_wait_idle(sbcs);
    dmi_wr(7'h3C, 32'h0000_DEAD); sba_wait_idle(sbcs);
    dmi_wr(7'h38, (32'd1 << 20) | (32'd1 << 17) | (32'd1 << 16)); // sbreadonaddr=1
    dmi_wr(7'h39, 32'h0000_0100); sba_wait_idle(sbcs);
    dmi_rd(7'h3C, rd);
    if (rd !== 32'h0000_BEEF) $fatal(1, "[FAIL] SBA 16b low halfword=0x%08x", rd);
    dmi_wr(7'h39, 32'h0000_0102); sba_wait_idle(sbcs);
    dmi_rd(7'h3C, rd);
    if (rd !== 32'h0000_DEAD) $fatal(1, "[FAIL] SBA 16b high halfword=0x%08x", rd);
    cov_dm_sba_xfers += 4;

    // 8-bit SBA writes & reads across all 4 byte lanes (sbaccess=0, sbautoincrement=1)
    dmi_wr(7'h38, (32'd0 << 17) | (32'd1 << 16));
    dmi_wr(7'h39, 32'h0000_0120);
    dmi_wr(7'h3C, 32'h11); sba_wait_idle(sbcs);
    dmi_wr(7'h3C, 32'h22); sba_wait_idle(sbcs);
    dmi_wr(7'h3C, 32'h33); sba_wait_idle(sbcs);
    dmi_wr(7'h3C, 32'h44); sba_wait_idle(sbcs);
    dmi_wr(7'h38, (32'd2 << 17)); // 32-bit read of 0x120 to verify byte lanes!
    dmi_wr(7'h38, (32'd1 << 20) | (32'd2 << 17));
    dmi_wr(7'h39, 32'h0000_0120); sba_wait_idle(sbcs);
    dmi_rd(7'h3C, rd);
    if (rd !== 32'h4433_2211) $fatal(1, "[FAIL] SBA 8b lane assembly=0x%08x", rd);
    cov_dm_sba_xfers += 5;

    // SBA Error Capture: alignment error (sberror=3), bus error (sberror=2), unsupported size (sberror=4)
    dmi_wr(7'h38, (32'd2 << 17));
    dmi_wr(7'h39, 32'h0000_0001); // unaligned 32-bit
    dmi_wr(7'h3C, 32'h1234_5678);
    sba_wait_idle(sbcs);
    if (sbcs[14:12] !== 3'd3) $fatal(1, "[FAIL] Expected sberror=3 (align), got %0d", sbcs[14:12]);
    dmi_wr(7'h38, (32'd7 << 12) | (32'd2 << 17)); // clear sberror
    cov_dm_sba_errs++;

    dmi_wr(7'h39, 32'h8000_0000); // unmapped -> SLVERR
    dmi_wr(7'h3C, 32'hCAFE_BABE);
    sba_wait_idle(sbcs);
    if (sbcs[14:12] !== 3'd2) $fatal(1, "[FAIL] Expected sberror=2 (bus err), got %0d", sbcs[14:12]);
    dmi_wr(7'h38, (32'd7 << 12) | (32'd2 << 17));
    cov_dm_sba_errs++;

    // 2.5 CSR Monitor/Progbuf Aperture + dmactive=0 reset
    dmi_wr(7'h20, 32'h0010_0073); // progbuf0 = ebreak
    do_axi_read(dm_s_axi_m, dm_s_axi_s, 32'h020, rd);
    if (rd !== 32'h0010_0073) $fatal(1, "[FAIL] CSR progbuf0 mismatch: 0x%08x", rd);
    do_axi_read(dm_s_axi_m, dm_s_axi_s, 32'h000, rd); // DM_CSR_STAT
    if (!rd[0]) $fatal(1, "[FAIL] DM_CSR_STAT dmactive bit expected 1");

    // Test dmactive=0 reset
    dmi_wr(7'h10, 32'h0000_0000);
    dmi_rd(7'h39, rd);
    if (rd !== 32'd0) $fatal(1, "[FAIL] dmactive=0 should reset sbaddress0");

    // 2.6 Quad-Bus SBA Master Verification (WB, TL, APB->AXI4)
    for (int w = 0; w < 3; w++) begin
      dmi_sec_wr(w, 7'h10, 32'h0000_0001); // dmactive=1
      dmi_sec_wr(w, 7'h38, (32'd2 << 17) | (32'd1 << 16));
      dmi_sec_wr(w, 7'h39, 32'h0000_0010);
      dmi_sec_wr(w, 7'h3C, 32'hA0B0_C0D0 + 32'(w));
      repeat (10) @(posedge clk);
      dmi_sec_wr(w, 7'h38, (32'd1 << 20) | (32'd2 << 17));
      dmi_sec_wr(w, 7'h39, 32'h0000_0010);
      repeat (10) @(posedge clk);
      dmi_sec_rd(w, 7'h3C, rd);
      if (rd !== (32'hA0B0_C0D0 + 32'(w))) begin
        $fatal(1, "[FAIL] Secondary DM SBA which=%0d got 0x%08x", w, rd);
      end
      cov_quad_bus_checks++;
    end

    // Verify DM APB monitor slave
    do_apb_read(dm_s_apb_m, dm_s_apb_s, 32'h004, rd);
    if (rd !== 32'h0000_0010) $fatal(1, "[FAIL] DM APB monitor sbaddr mismatch: 0x%08x", rd);
    do_wb_read(dm_s_wb_m, dm_s_wb_s, 32'h004, rd);
    if (rd !== 32'h0000_0010) $fatal(1, "[FAIL] DM WB monitor sbaddr mismatch: 0x%08x", rd);
    do_tl_read(dm_s_tl_m, dm_s_tl_s, 32'h004, rd);
    if (rd !== 32'h0000_0010) $fatal(1, "[FAIL] DM TL monitor sbaddr mismatch: 0x%08x", rd);
  endtask

  // ===========================================================================
  // TEST 3: SAR-ADC Digital Sequencer & PDM-to-PCM (DFSDM) Filter
  // ===========================================================================
  task automatic run_test_adc_pdm();
    logic [31:0] rd;
    int          guard;

    $display("[TB] --- Test 3: adc_pdm_top ---");

    // 3.1 SAR-ADC Multi-Channel Scan + 16x Oversampling Exact Average
    // Sequence: 4 slots -> ch 3, 7, 1, 12
    for (int c = 0; c < 16; c++) sar_ch_base[c] = 12'(c * 16'd200 + 16'd100);
    sar_use_ramp = 1'b1;

    do_wb_write(adc_wb_m, adc_wb_s, 32'h028, 32'h0000_C173); // slots 0..3: ch3, ch7, ch1, ch12
    do_wb_write(adc_wb_m, adc_wb_s, 32'h008, 32'h0000_010D); // IRQ_EN: ADC_DONE, AWD_LOW, AWD_HIGH
    // ADC_CTRL: EN=1, SW_START=1, SEQ_LEN=3(4 slots), OVS_RATIO=4(16x), OVS_SHIFT=4, DMA_WM=4
    do_wb_write(adc_wb_m, adc_wb_s, 32'h020, 32'h0004_2439);

    guard = 0;
    do begin
      @(posedge clk);
      guard++;
      if (guard > 2000) $fatal(1, "[FAIL] ADC 16x scan timeout");
    end while (!adc_irq);

    if (!dma_adc_req) $fatal(1, "[FAIL] Expected dma_adc_req_o asserted when level==4");
    cov_adc_scans++;

    // Verify exact 16x average for each slot: base + mean(0..15) = base + 7 (integer floor(120/16)=7!)
    begin
      logic [3:0] exp_chs [4] = '{4'd3, 4'd7, 4'd1, 4'd12};
      for (int s = 0; s < 4; s++) begin
        logic [11:0] base_v = 12'(exp_chs[s] * 16'd200 + 16'd100);
        logic [15:0] exp_avg = (16'(base_v) * 16'd16 + 16'd120) >> 4;
        do_wb_read(adc_wb_m, adc_wb_s, 32'h038, rd);
        if (rd[19:16] !== exp_chs[s] || rd[15:0] !== exp_avg) begin
          $fatal(1, "[FAIL] ADC slot %0d ch=%0d val=%0d (exp ch=%0d val=%0d)",
                 s, rd[19:16], rd[15:0], exp_chs[s], exp_avg);
        end
      end
    end
    do_wb_write(adc_wb_m, adc_wb_s, 32'h004, 32'h1); // W1C ADC_DONE

    // 3.2 ext_trgo_i Hardware Trigger + Analog Watchdog High & Low Trips
    sar_use_ramp   = 1'b0;
    sar_ch_base[0] = 12'h100; // Below AWD_LOW=0x200
    sar_ch_base[1] = 12'hE00; // Above AWD_HIGH=0xC00
    do_wb_write(adc_wb_m, adc_wb_s, 32'h028, 32'h0000_0010); // slots 0..1: ch0, ch1
    do_wb_write(adc_wb_m, adc_wb_s, 32'h030, 32'hFFFF_0001); // AWD_EN=1, all channels
    do_wb_write(adc_wb_m, adc_wb_s, 32'h034, 32'h0C00_0200); // AWD_HIGH=0xC00, AWD_LOW=0x200
    // ADC_CTRL: EN=1, EXT_TRG_EN=1, SEQ_LEN=1(2 slots), OVS=0(1x), SHIFT=0, DMA_WM=1
    do_wb_write(adc_wb_m, adc_wb_s, 32'h020, 32'h0001_0015);

    // Pulse ext_trgo_i
    @(posedge clk); #1; ext_trgo = 1'b1;
    repeat (2) @(posedge clk);
    #1; ext_trgo = 1'b0;

    guard = 0;
    do begin
      @(posedge clk);
      guard++;
      if (guard > 500) $fatal(1, "[FAIL] ADC ext_trgo scan timeout");
    end while (!adc_irq || !awd_irq);
    // Wait a few cycles for slot 1 to finish if slot 0 tripped awd_irq first
    do begin
      do_wb_read(adc_wb_m, adc_wb_s, 32'h004, rd);
      guard++;
      if (guard > 600) $fatal(1, "[FAIL] ADC ext_trgo scan done timeout");
    end while (!rd[0]);
    if (!rd[2] || !rd[3]) $fatal(1, "[FAIL] Expected both AWD_LOW and AWD_HIGH IRQ bits: 0x%08x", rd);
    do_wb_read(adc_wb_m, adc_wb_s, 32'h038, rd);
    if (!rd[20] || rd[15:0] !== 16'h0100) $fatal(1, "[FAIL] Expected AWD_LOW flag on ch0: 0x%08x", rd);
    do_wb_read(adc_wb_m, adc_wb_s, 32'h038, rd);
    if (!rd[21] || rd[15:0] !== 16'h0E00) $fatal(1, "[FAIL] Expected AWD_HIGH flag on ch1: 0x%08x", rd);
    do_wb_write(adc_wb_m, adc_wb_s, 32'h004, 32'h1F); // Clear sticky IRQs
    #1;
    if (awd_irq) $fatal(1, "[FAIL] awd_irq_o did not clear after W1C");
    cov_adc_scans++;
    cov_adc_awd_trips += 2;

    // 3.3 PDM-to-PCM (DFSDM): Exact DC Gain across OSR = 16, 32, 64
    // Pattern 4'b1110 (+1,+1,+1,-1 -> mean +0.5 -> +16384 exact!)
    pdm_mode  = 1;
    pdm_pat_l = 4'b1110;
    for (int osr_sel = 0; osr_sel <= 2; osr_sel++) begin
      // PDM_CTRL: EN=1, STEREO=0, OSR=osr_sel, HPF_EN=0, GAIN=0, FIFO_FLUSH=1, DMA_WM=1, CLK_DIV=1
      do_wb_write(adc_wb_m, adc_wb_s, 32'h060, 32'h0011_8001 | (32'(osr_sel) << 2));
      for (int f = 0; f < 5; f++) begin
        guard = 0;
        do begin
          @(posedge clk);
          guard++;
          if (guard > 5000) $fatal(1, "[FAIL] PDM OSR=%0d frame timeout", osr_sel);
        end while (!dma_pdm_req);
        do_wb_read(adc_wb_m, adc_wb_s, 32'h068, rd);
        cov_pdm_frames++;
        if (f >= 3) begin
          logic signed [15:0] s_pcm = signed'(rd[15:0]);
          if (s_pcm !== 16'sd16384) begin
            $fatal(1, "[FAIL] PDM DC gain OSR_SEL=%0d frame=%0d got %0d (expected 16384)", osr_sel, f, s_pcm);
          end
        end
      end
    end

    // 3.4 PDM HPF Zero-Offset Convergence (constant +1 input -> +32768 -> converges to 0)
    pdm_mode    = 0;
    pdm_const_l = 1'b1;
    // EN=1, OSR=0(16), HPF_EN=1, HPF_SHIFT=2 (fast convergence), GAIN_RSHIFT=1 (+16384 DC), CLK_DIV=1
    do_wb_write(adc_wb_m, adc_wb_s, 32'h060, 32'h0011_8451);
    for (int f = 0; f < 40; f++) begin
      guard = 0;
      do begin
        @(posedge clk);
        guard++;
        if (guard > 2000) $fatal(1, "[FAIL] PDM HPF frame timeout");
      end while (!dma_pdm_req);
      do_wb_read(adc_wb_m, adc_wb_s, 32'h068, rd);
      cov_pdm_frames++;
      if (f == 39) begin
        logic signed [15:0] hpf_pcm = signed'(rd[15:0]);
        logic signed [15:0] dc_est;
        do_wb_read(adc_wb_m, adc_wb_s, 32'h06C, rd);
        dc_est = signed'(rd[15:0]);
        if ((hpf_pcm > 16'sd4) || (hpf_pcm < -16'sd4)) begin
          $fatal(1, "[FAIL] PDM HPF did not converge to zero: pcm=%0d dc_est=%0d", hpf_pcm, dc_est);
        end
      end
    end

    // 3.5 PDM 1-bit Sigma-Delta Sine Wave + DC Offset -> SNR & HPF Verification
    // OSR = 64 (osr_sel=2), 64 PCM samples per sine period -> freq = 1/(64*64) = 1/4096 cycles/bit
    pdm_dc_l   = 0.25;
    pdm_amp_l  = 0.50;
    pdm_freq_l = 1.0 / 4096.0;
    pdm_mode   = 2;
    // EN=1, OSR=2(64), HPF_EN=1, HPF_SHIFT=5, GAIN=0, CLK_DIV=1
    do_wb_write(adc_wb_m, adc_wb_s, 32'h060, 32'h0011_80B9);

    // Discard 256 frames (8*tau) for Sinc3 + HPF DC settling, then capture 64 samples (1 exact sine period)
    begin
      real sig_pow = 0.0, err_pow = 0.0;
      real re_f = 0.0, im_f = 0.0, dc_sum = 0.0;
      real samples [64];
      real snr_db;

      for (int f = 0; f < 320; f++) begin
        guard = 0;
        do begin
          @(posedge clk);
          guard++;
          if (guard > 4000) $fatal(1, "[FAIL] PDM SDM sine timeout");
        end while (!dma_pdm_req);
        do_wb_read(adc_wb_m, adc_wb_s, 32'h068, rd);
        cov_pdm_frames++;
        if (f >= 256) begin
          int idx = f - 256;
          real v = real'(signed'(rd[15:0]));
          real theta = 6.283185307179586 * real'(idx) / 64.0;
          samples[idx] = v;
          dc_sum += v;
          re_f += v * $cos(theta);
          im_f += v * $sin(theta);
        end
      end

      // Single-bin DFT at k=1 extracts fundamental amplitude & phase
      begin
        real amp_est = 2.0 * $sqrt(re_f*re_f + im_f*im_f) / 64.0;
        real ph_est  = $atan2(-im_f, re_f);
        real dc_avg  = dc_sum / 64.0;
        for (int i = 0; i < 64; i++) begin
          real theta = 6.283185307179586 * real'(i) / 64.0;
          real ideal_v = amp_est * $cos(theta + ph_est);
          sig_pow += ideal_v * ideal_v;
          err_pow += (samples[i] - ideal_v) * (samples[i] - ideal_v);
        end
        snr_db = 10.0 * $log10(sig_pow / (err_pow + 1.0e-9));
        $display("[TB] PDM Sinc3 SDM Sine (OSR=64, HPF=ON): amp=%.1f dc=%.1f SNR=%.2f dB", amp_est, dc_avg, snr_db);
        if (amp_est < 12000.0 || amp_est > 18000.0) $fatal(1, "[FAIL] PDM sine amplitude out of range: %.1f", amp_est);
        if (dc_avg > 150.0 || dc_avg < -150.0) $fatal(1, "[FAIL] PDM HPF residual DC too high: %.1f", dc_avg);
        if (snr_db < 35.0) $fatal(1, "[FAIL] PDM Sinc3 SNR too low: %.2f dB", snr_db);
      end
    end

    // 3.6 Stereo PDM L/R Separation Test
    pdm_mode    = 0;
    pdm_const_l = 1'b1; // Ch0 = +16384 (with GAIN_RSHIFT=1)
    pdm_const_r = 1'b0; // Ch1 = -16384 (with GAIN_RSHIFT=1)
    // EN=1, STEREO=1, OSR=0(16), HPF=0, GAIN_RSHIFT=1, CLK_DIV=1
    do_wb_write(adc_wb_m, adc_wb_s, 32'h060, 32'h0011_8403);
    for (int f = 0; f < 5; f++) begin
      guard = 0;
      do begin
        @(posedge clk);
        guard++;
        if (guard > 2000) $fatal(1, "[FAIL] PDM stereo timeout");
      end while (!dma_pdm_req);
      do_wb_read(adc_wb_m, adc_wb_s, 32'h068, rd);
      cov_pdm_frames++;
      if (f == 4) begin
        logic signed [15:0] l_pcm = signed'(rd[15:0]);
        logic signed [15:0] r_pcm = signed'(rd[31:16]);
        if (l_pcm !== 16'sd16384 || r_pcm !== -16'sd16384) begin
          $fatal(1, "[FAIL] PDM stereo mismatch L=%0d R=%0d", l_pcm, r_pcm);
        end
      end
    end

    // 3.7 Quad-bus smoke on AXI4, TL, APB instances
    do_axi_read(adc_axi_m, adc_axi_s, 32'h000, rd);
    if (rd !== 32'h3710_1010) $fatal(1, "[FAIL] ADC AXI CAPS mismatch: 0x%08x", rd);
    do_tl_read(adc_tl_m, adc_tl_s, 32'h000, rd);
    if (rd !== 32'h3710_1010) $fatal(1, "[FAIL] ADC TL CAPS mismatch: 0x%08x", rd);
    do_apb_read(adc_apb_m, adc_apb_s, 32'h000, rd);
    if (rd !== 32'h3710_1010) $fatal(1, "[FAIL] ADC APB CAPS mismatch: 0x%08x", rd);
    cov_quad_bus_checks += 3;
  endtask

  // ===========================================================================
  // Main Test Runner
  // ===========================================================================
  initial begin
    rst_n         = 1'b0;
    mbox_apb_m    = '0; mbox_wb_m = '0; mbox_axi_m = '0; mbox_tl_m = '0;
    dm_s_axi_m    = '0; dm_s_wb_m = '0; dm_s_tl_m  = '0; dm_s_apb_m = '0;
    dm_dmi_req_v  = 1'b0; dm_dmi_req_a = '0; dm_dmi_req_op = '0; dm_dmi_req_d = '0; dm_dmi_rsp_r = 1'b0;
    dm_wb_dmi_rv  = 1'b0; dm_wb_dmi_a  = '0; dm_wb_dmi_op  = '0; dm_wb_dmi_wd = '0; dm_wb_dmi_sr = 1'b0;
    dm_tl_dmi_rv  = 1'b0; dm_tl_dmi_a  = '0; dm_tl_dmi_op  = '0; dm_tl_dmi_wd = '0; dm_tl_dmi_sr = 1'b0;
    dm_apb_dmi_rv = 1'b0; dm_apb_dmi_a = '0; dm_apb_dmi_op = '0; dm_apb_dmi_wd = '0; dm_apb_dmi_sr = 1'b0;
    adc_wb_m      = '0; adc_axi_m = '0; adc_tl_m   = '0; adc_apb_m  = '0;
    ext_trgo      = 1'b0;
    pdm_mode      = 0;
    pdm_dc_l      = 0.0; pdm_dc_r = 0.0;
    pdm_amp_l     = 0.0; pdm_amp_r = 0.0;
    pdm_freq_l    = 0.0; pdm_freq_r = 0.0;
    for (int c = 0; c < 16; c++) sar_ch_base[c] = 12'd0;

    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    run_test_ipc_mailbox();
    run_test_riscv_dm();
    run_test_adc_pdm();

    $display("[COV] seed=%0d locks=%0d db_irqs=%0d mbox_msgs=%0d dm_harts=%0d abs_cmds=%0d sba_xfers=%0d sba_errs=%0d adc_scans=%0d awd=%0d pdm_frames=%0d quad_bus=%0d",
             seed, cov_mbox_locks_tested, cov_mbox_db_irqs, cov_mbox_fifo_msgs,
             cov_dm_hart_ops, cov_dm_abs_cmds, cov_dm_sba_xfers, cov_dm_sba_errs,
             cov_adc_scans, cov_adc_awd_trips, cov_pdm_frames, cov_quad_bus_checks);
    $display("[PASS] tb_ipc_debug: ipc_mailbox(32-lock/DB/FIFO), riscv_dm(4-hart/abstract/SBA 8-16-32b), adc_pdm(SAR-16x-AWD/Sinc3-HPF-Stereo) quad-bus verified");
    $finish;
  end

endmodule : tb_ipc_debug
