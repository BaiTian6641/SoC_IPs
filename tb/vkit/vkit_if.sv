// SPDX-License-Identifier: Apache-2.0
// vkit_if.sv - one thin interface per protocol port.  A port instance is wired either as a
// MASTER-side port (class driver writes m2s, DUT output feeds s2m) or as a SLAVE-side port
// (DUT output feeds m2s, class responder writes s2m).  Monitors/checkers only read both.
//
//   vkit_wb_if  mif[N](clk);   assign dut_m_i[i] = mif[i].m2s;   assign mif[i].s2m = dut_m_o[i];
//   vkit_wb_if  sif[M](clk);   assign sif[j].m2s = dut_s_o[j];   assign dut_s_i[j] = sif[j].s2m;
//
// Interface-array elements cannot be indexed with a run-time variable in Verilator, so a TB
// copies them into `virtual vkit_wb_if` arrays inside a generate loop (see tb_vkit_*).

`timescale 1ns/1ps

interface vkit_wb_if (input logic clk);
  import bus_pkg::*;
  wb_m2s_t m2s = '0;
  wb_s2m_t s2m = '0;
endinterface

interface vkit_axi_if (input logic clk);
  import bus_pkg::*;
  axi4_m2s_t m2s = '0;
  axi4_s2m_t s2m = '0;
endinterface

interface vkit_tl_if (input logic clk);
  import bus_pkg::*;
  tl_m2s_t m2s = '0;
  tl_s2m_t s2m = '0;
endinterface

interface vkit_apb_if (input logic clk);
  import bus_pkg::*;
  apb_m2s_t m2s = '0;
  apb_s2m_t s2m = '0;
endinterface
