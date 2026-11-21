// SPDX-License-Identifier: Apache-2.0
// otp_trng_keymgr_axi.sv - AXI4 / AXI4-Lite Slave Wrapper for otp_trng_keymgr_core

module otp_trng_keymgr_axi
  import bus_pkg::*;
  import otp_trng_keymgr_pkg::*;
#(
  parameter bit          OTP_EMU        = 1'b1,
  parameter bit          ENT_SOFT_EMU   = 1'b1,
  parameter int unsigned RNG_FIFO_DEPTH = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,
  input  axi4_m2s_t             axi_i,
  output axi4_s2m_t             axi_o,

  output logic                  otp_prog_o,
  output logic                  otp_rd_o,
  output logic [OTP_ADDR_W-1:0] otp_addr_o,
  output logic [38:0]           otp_wdata_o,
  input  logic [38:0]           otp_rdata_i,
  input  logic                  otp_ready_i,

  input  logic                  ent_valid_i,
  input  logic [3:0]            ent_data_i,
  output logic                  ent_enable_o,

  output logic                  rng_valid_o,
  output logic [31:0]           rng_data_o,
  input  logic                  rng_ready_i,
  output logic                  dma_req_o,
  output logic                  irq_o,

  output logic                  hw_key_valid_o,
  output logic [2:0]            hw_key_slot_o,
  output logic [255:0]          hw_key_data_o,
  output logic [2:0]            lc_state_o
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

  otp_trng_keymgr_core #(
    .OTP_EMU        (OTP_EMU),
    .ENT_SOFT_EMU   (ENT_SOFT_EMU),
    .RNG_FIFO_DEPTH (RNG_FIFO_DEPTH)
  ) u_core (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),
    .csr_req_i      (csr_req),
    .csr_rsp_o      (csr_rsp),
    .otp_prog_o     (otp_prog_o),
    .otp_rd_o       (otp_rd_o),
    .otp_addr_o     (otp_addr_o),
    .otp_wdata_o    (otp_wdata_o),
    .otp_rdata_i    (otp_rdata_i),
    .otp_ready_i    (otp_ready_i),
    .ent_valid_i    (ent_valid_i),
    .ent_data_i     (ent_data_i),
    .ent_enable_o   (ent_enable_o),
    .rng_valid_o    (rng_valid_o),
    .rng_data_o     (rng_data_o),
    .rng_ready_i    (rng_ready_i),
    .dma_req_o      (dma_req_o),
    .irq_o          (irq_o),
    .hw_key_valid_o (hw_key_valid_o),
    .hw_key_slot_o  (hw_key_slot_o),
    .hw_key_data_o  (hw_key_data_o),
    .lc_state_o     (lc_state_o)
  );

endmodule : otp_trng_keymgr_axi
