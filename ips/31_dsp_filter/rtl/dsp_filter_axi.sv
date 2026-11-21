// SPDX-License-Identifier: Apache-2.0
// dsp_filter_axi.sv - AXI4 / AXI4-Lite Slave Wrapper for dsp_filter_core

module dsp_filter_axi
  import bus_pkg::*;
#(
  parameter int unsigned MAX_TAPS     = 64,
  parameter int unsigned MAX_SECTIONS = 8,
  parameter int unsigned MAX_CHANNELS = 8,
  parameter int unsigned FIFO_DEPTH   = 16,
  localparam int unsigned CH_W        = (MAX_CHANNELS <= 1) ? 1 : $clog2(MAX_CHANNELS)
) (
  input  logic            clk_i,
  input  logic            rst_ni,

  input  axi4_m2s_t       axi_i,
  output axi4_s2m_t       axi_o,

  input  logic            s_axis_tvalid_i,
  output logic            s_axis_tready_o,
  input  logic [31:0]     s_axis_tdata_i,
  input  logic            s_axis_tlast_i,
  input  logic [CH_W-1:0] s_axis_tid_i,

  output logic            m_axis_tvalid_o,
  input  logic            m_axis_tready_i,
  output logic [31:0]     m_axis_tdata_o,
  output logic            m_axis_tlast_o,
  output logic [CH_W-1:0] m_axis_tid_o,

  output logic            dma_in_req_o,
  output logic            dma_out_req_o,
  output logic            irq_o
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  axi_lite_to_csr u_axi_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .axi_i     (axi_i),
    .axi_o     (axi_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  dsp_filter_core #(
    .MAX_TAPS     (MAX_TAPS),
    .MAX_SECTIONS (MAX_SECTIONS),
    .MAX_CHANNELS (MAX_CHANNELS),
    .FIFO_DEPTH   (FIFO_DEPTH)
  ) u_core (
    .clk_i           (clk_i),
    .rst_ni          (rst_ni),
    .csr_req_i       (csr_req),
    .csr_rsp_o       (csr_rsp),
    .s_axis_tvalid_i (s_axis_tvalid_i),
    .s_axis_tready_o (s_axis_tready_o),
    .s_axis_tdata_i  (s_axis_tdata_i),
    .s_axis_tlast_i  (s_axis_tlast_i),
    .s_axis_tid_i    (s_axis_tid_i),
    .m_axis_tvalid_o (m_axis_tvalid_o),
    .m_axis_tready_i (m_axis_tready_i),
    .m_axis_tdata_o  (m_axis_tdata_o),
    .m_axis_tlast_o  (m_axis_tlast_o),
    .m_axis_tid_o    (m_axis_tid_o),
    .dma_in_req_o    (dma_in_req_o),
    .dma_out_req_o   (dma_out_req_o),
    .irq_o           (irq_o)
  );

endmodule : dsp_filter_axi
