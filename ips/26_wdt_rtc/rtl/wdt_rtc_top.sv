// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for wdt_rtc_core
//
// Pad / system signals:
//   por_ni        power-on reset (tie to rst_ni if there is no separate one)
//   wdt_clk_en_i  independent watchdog clock-enable tick (async, synchronised inside)
//   dbg_freeze_i  freeze watchdog (synchronous to clk_i);  sleep_i  system sleep indication
//   wdt_rst_req_o reset request toward the reset controller (held WDT_HOLD cycles)
//   rtc_clk_i / rtc_por_ni  backup-domain clock and its ONLY reset
//   tamper_i      tamper / timestamp event inputs (async, synchronised inside the RTC domain)
//   rtc_tick_o    calibrated 1 Hz tick (one rtc_clk_i wide, RTC domain)
//   rtc_wakeup_o  wake-up level (RTC domain)

module wdt_rtc_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned NUM_TAMPER   = 2,
  parameter int unsigned CAL_WIN_LOG2 = 20,
  parameter int unsigned RTC_TMO_LOG2 = 22
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  logic      por_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  input  apb_m2s_t  apb_i,
  output apb_s2m_t  apb_o,

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

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o = '0;
    assign apb_o = '0;
    wdt_rtc_wb #(
      .NUM_TAMPER   (NUM_TAMPER),
      .CAL_WIN_LOG2 (CAL_WIN_LOG2),
      .RTC_TMO_LOG2 (RTC_TMO_LOG2)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    wdt_rtc_tl #(
      .NUM_TAMPER   (NUM_TAMPER),
      .CAL_WIN_LOG2 (CAL_WIN_LOG2),
      .RTC_TMO_LOG2 (RTC_TMO_LOG2)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o = '0;
    assign axi_o = '0;
    assign tl_o = '0;
    wdt_rtc_apb #(
      .NUM_TAMPER   (NUM_TAMPER),
      .CAL_WIN_LOG2 (CAL_WIN_LOG2),
      .RTC_TMO_LOG2 (RTC_TMO_LOG2)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .apb_i   (apb_i),
      .apb_o   (apb_o),
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
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    assign apb_o = '0;
    wdt_rtc_axi #(
      .NUM_TAMPER   (NUM_TAMPER),
      .CAL_WIN_LOG2 (CAL_WIN_LOG2),
      .RTC_TMO_LOG2 (RTC_TMO_LOG2)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i   (axi_i),
      .axi_o   (axi_o),
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
  end

endmodule : wdt_rtc_top
