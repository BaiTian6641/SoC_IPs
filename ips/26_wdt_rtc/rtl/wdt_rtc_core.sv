// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_core.sv - Windowed watchdog + backup-domain real-time clock (protocol-agnostic CSR core)
//
// Address map (csr_req.addr[9:0]):  0x000..0x0FF  watchdog (wdt_rtc_wdt)
//                                   0x100..0x1FF  RTC      (wdt_rtc_rtcif + wdt_rtc_rtc)
// See ips/26_wdt_rtc/doc/README.md for the complete register description.
//
// Clock / reset domains
//   clk_i / rst_ni    : bus + watchdog domain, system reset
//   por_ni            : power-on reset (reset-cause flags, reset-request hold); tie to rst_ni if the
//                       system has no separate power-on reset
//   rtc_clk_i / rtc_por_ni : backup (battery) domain: ONLY rtc_por_ni resets RTC state

module wdt_rtc_core
  import bus_pkg::*;
  import wdt_rtc_pkg::*;
#(
  parameter int unsigned NUM_TAMPER   = 2,
  parameter int unsigned CAL_WIN_LOG2 = 20,
  parameter int unsigned RTC_TMO_LOG2 = 22
) (
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic                   por_ni,

  input  csr_req_t               csr_req_i,
  output csr_rsp_t               csr_rsp_o,

  // watchdog
  input  logic                   wdt_clk_en_i,
  input  logic                   dbg_freeze_i,
  input  logic                   sleep_i,
  output logic                   wdt_rst_req_o,
  output logic                   wdt_irq_o,

  // RTC (backup domain)
  input  logic                   rtc_clk_i,
  input  logic                   rtc_por_ni,
  input  logic [NUM_TAMPER-1:0]  tamper_i,
  output logic                   rtc_tick_o,
  output logic                   rtc_irq_o,
  output logic                   rtc_wakeup_o,

  output logic                   irq_o
);

  wire sel_wdt = (csr_req_i.addr[9:8] == 2'b00);
  wire sel_rtc = (csr_req_i.addr[9:8] == 2'b01);

  csr_req_t wdt_req, rtc_req;
  always_comb begin
    wdt_req       = csr_req_i;
    rtc_req       = csr_req_i;
    wdt_req.valid = csr_req_i.valid && sel_wdt;
    rtc_req.valid = csr_req_i.valid && sel_rtc;
  end

  logic [31:0] wdt_rdata, rtc_rdata;
  logic        wdt_err, rtc_err, rtc_ready;

  wdt_rtc_wdt u_wdt (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .por_ni       (por_ni),
    .csr_req_i    (wdt_req),
    .rdata_o      (wdt_rdata),
    .err_o        (wdt_err),
    .wdt_clk_en_i (wdt_clk_en_i),
    .dbg_freeze_i (dbg_freeze_i),
    .sleep_i      (sleep_i),
    .rst_req_o    (wdt_rst_req_o),
    .irq_o        (wdt_irq_o)
  );

  // CDC channels between bus domain and RTC domain
  logic              ch_req_tog, ch_rd, ch_ack_tog;
  logic [5:0]        ch_addr;
  logic [31:0]       ch_wdata, ch_rdata;
  logic [3:0]        ch_wstrb;
  logic              snap_req_tog, snap_ack_tog;
  logic [SNAP_W-1:0] snap;
  logic [7:0]        flags, ien;
  logic              initf, inits, unl;

  wdt_rtc_rtcif #(.TMO_LOG2(RTC_TMO_LOG2)) u_rtcif (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .csr_req_i      (rtc_req),
    .ready_o        (rtc_ready),
    .rdata_o        (rtc_rdata),
    .err_o          (rtc_err),
    .ch_req_tog_o   (ch_req_tog),
    .ch_rd_o        (ch_rd),
    .ch_addr_o      (ch_addr),
    .ch_wdata_o     (ch_wdata),
    .ch_wstrb_o     (ch_wstrb),
    .ch_ack_tog_i   (ch_ack_tog),
    .ch_rdata_i     (ch_rdata),
    .snap_req_tog_o (snap_req_tog),
    .snap_ack_tog_i (snap_ack_tog),
    .snap_i         (snap),
    .flags_i        (flags),
    .ien_i          (ien),
    .initf_i        (initf),
    .inits_i        (inits),
    .unlocked_i     (unl),
    .rtc_irq_o      (rtc_irq_o)
  );

  wdt_rtc_rtc #(
    .NUM_TAMPER   (NUM_TAMPER),
    .CAL_WIN_LOG2 (CAL_WIN_LOG2)
  ) u_rtc (
    .rtc_clk_i      (rtc_clk_i),
    .rtc_por_ni     (rtc_por_ni),
    .ch_req_tog_i   (ch_req_tog),
    .ch_rd_i        (ch_rd),
    .ch_addr_i      (ch_addr),
    .ch_wdata_i     (ch_wdata),
    .ch_wstrb_i     (ch_wstrb),
    .ch_ack_tog_o   (ch_ack_tog),
    .ch_rdata_o     (ch_rdata),
    .snap_req_tog_i (snap_req_tog),
    .snap_ack_tog_o (snap_ack_tog),
    .snap_o         (snap),
    .flags_o        (flags),
    .ien_o          (ien),
    .initf_o        (initf),
    .inits_o        (inits),
    .unlocked_o     (unl),
    .tamper_i       (tamper_i),
    .tick_o         (rtc_tick_o),
    .wakeup_o       (rtc_wakeup_o)
  );

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = sel_rtc ? rtc_ready : 1'b1;
    if (sel_wdt) begin
      csr_rsp_o.rdata = wdt_rdata;
      csr_rsp_o.err   = wdt_err;
    end else if (sel_rtc) begin
      csr_rsp_o.rdata = rtc_rdata;
      csr_rsp_o.err   = rtc_err;
    end else begin
      csr_rsp_o.err   = csr_req_i.valid;
    end
  end

  assign irq_o = wdt_irq_o | rtc_irq_o;

endmodule : wdt_rtc_core
