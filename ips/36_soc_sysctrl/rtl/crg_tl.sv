// SPDX-License-Identifier: Apache-2.0
// crg_tl.sv - TileLink TL-UL / TL-UH Device Wrapper for crg_core

module crg_tl
  import bus_pkg::*;
#(
  parameter int unsigned N_OUT_CLKS       = 4,
  parameter int unsigned RST_PULSE_CYCLES = 8
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  input  tl_m2s_t               tl_i,
  output tl_s2m_t               tl_o,
  input  logic                  osc_clk_i,
  input  logic                  pll_clk_i,
  input  logic                  ext_clk_i,
  input  logic                  aux_clk_i,
  input  logic                  test_mode_i,
  input  logic                  por_ni,
  input  logic                  ext_rst_ni,
  input  logic                  wdt_rst_i,
  input  logic                  brownout_rst_i,
  output logic                  pll_en_o,
  output logic                  pll_bypass_o,
  output logic [31:0]           pll_cfg_o,
  input  logic                  pll_lock_i,
  output logic                  root_clk_o,
  output logic                  sys_rst_no,
  output logic [N_OUT_CLKS-1:0] clk_out_o,
  output logic [N_OUT_CLKS-1:0] rst_out_no,
  output logic [3:0]            mux_active_o,
  output logic [4:0]            rst_cause_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  tl_ul_to_csr u_tl_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .tl_i      (tl_i),
    .tl_o      (tl_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  crg_core #(
    .N_OUT_CLKS       (N_OUT_CLKS),
    .RST_PULSE_CYCLES (RST_PULSE_CYCLES)
  ) u_core (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .csr_req_i      (csr_req),
    .csr_rsp_o      (csr_rsp),
    .osc_clk_i      (osc_clk_i),
    .pll_clk_i      (pll_clk_i),
    .ext_clk_i      (ext_clk_i),
    .aux_clk_i      (aux_clk_i),
    .test_mode_i    (test_mode_i),
    .por_ni         (por_ni),
    .ext_rst_ni     (ext_rst_ni),
    .wdt_rst_i      (wdt_rst_i),
    .brownout_rst_i (brownout_rst_i),
    .pll_en_o       (pll_en_o),
    .pll_bypass_o   (pll_bypass_o),
    .pll_cfg_o      (pll_cfg_o),
    .pll_lock_i     (pll_lock_i),
    .root_clk_o     (root_clk_o),
    .sys_rst_no     (sys_rst_no),
    .clk_out_o      (clk_out_o),
    .rst_out_no     (rst_out_no),
    .mux_active_o   (mux_active_o),
    .rst_cause_o    (rst_cause_o)
  );

endmodule : crg_tl
