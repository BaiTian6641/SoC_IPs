// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_apb.sv - AMBA APB4 Completer Wrapper for wdt_rtc_core

module wdt_rtc_apb
  import bus_pkg::*;
#(
  parameter int unsigned NUM_TAMPER   = 2,
  parameter int unsigned CAL_WIN_LOG2 = 20,
  parameter int unsigned RTC_TMO_LOG2 = 22
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  apb_m2s_t apb_i,
  output apb_s2m_t apb_o,
  input  logic                   por_ni,
  input  logic                   wdt_clk_en_i,
  input  logic                   dbg_freeze_i,
  input  logic                   sleep_i,
  output logic                   wdt_rst_req_o,
  output logic                   wdt_irq_o,
  input  logic                   rtc_clk_i,
  input  logic                   rtc_por_ni,
  input  logic [NUM_TAMPER-1:0]  tamper_i,
  output logic                   rtc_tick_o,
  output logic                   rtc_irq_o,
  output logic                   rtc_wakeup_o,
  output logic                   irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  apb_to_csr u_apb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .apb_i     (apb_i),
    .apb_o     (apb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  wdt_rtc_core #(
    .NUM_TAMPER   (NUM_TAMPER),
    .CAL_WIN_LOG2 (CAL_WIN_LOG2),
    .RTC_TMO_LOG2 (RTC_TMO_LOG2)
  ) u_core (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .csr_req_i     (csr_req),
    .csr_rsp_o     (csr_rsp),
    .por_ni        (por_ni),
    .wdt_clk_en_i  (wdt_clk_en_i),
    .dbg_freeze_i  (dbg_freeze_i),
    .sleep_i       (sleep_i),
    .wdt_rst_req_o (wdt_rst_req_o),
    .wdt_irq_o     (wdt_irq_o),
    .rtc_clk_i     (rtc_clk_i),
    .rtc_por_ni    (rtc_por_ni),
    .tamper_i      (tamper_i),
    .rtc_tick_o    (rtc_tick_o),
    .rtc_irq_o     (rtc_irq_o),
    .rtc_wakeup_o  (rtc_wakeup_o),
    .irq_o         (irq_o)
  );

endmodule : wdt_rtc_apb
