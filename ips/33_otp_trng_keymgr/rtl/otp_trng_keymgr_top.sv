// SPDX-License-Identifier: Apache-2.0
// otp_trng_keymgr_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TL / APB4)
//                          for otp_trng_keymgr_core.

module otp_trng_keymgr_top
  import bus_pkg::*;
  import otp_trng_keymgr_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE       = bus_pkg::BUS_AXI4,
  parameter bit                  OTP_EMU        = 1'b1,
  parameter bit                  ENT_SOFT_EMU   = 1'b1,
  parameter int unsigned         RNG_FIFO_DEPTH = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t               wb_i,
  output wb_s2m_t               wb_o,
  input  axi4_m2s_t             axi_i,
  output axi4_s2m_t             axi_o,
  input  tl_m2s_t               tl_i,
  output tl_s2m_t               tl_o,
  input  apb_m2s_t              apb_i,
  output apb_s2m_t              apb_o,

  // External Foundry OTP / eFuse Macro Interface (when OTP_EMU == 0)
  output logic                  otp_prog_o,
  output logic                  otp_rd_o,
  output logic [OTP_ADDR_W-1:0] otp_addr_o,
  output logic [38:0]           otp_wdata_o,
  input  logic [38:0]           otp_rdata_i,
  input  logic                  otp_ready_i,

  // External Analog Entropy-Source Macro Interface
  input  logic                  ent_valid_i,
  input  logic [3:0]            ent_data_i,
  output logic                  ent_enable_o,

  // Conditioned Random Output Hardware Port, DMA, Interrupt
  output logic                  rng_valid_o,
  output logic [31:0]           rng_data_o,
  input  logic                  rng_ready_i,
  output logic                  dma_req_o,
  output logic                  irq_o,

  // Hardware-Only Sideload Key Bus
  output logic                  hw_key_valid_o,
  output logic [2:0]            hw_key_slot_o,
  output logic [255:0]          hw_key_data_o,

  // Lifecycle State Sideband
  output logic [2:0]            lc_state_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign apb_o = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    otp_trng_keymgr_wb #(
      .OTP_EMU        (OTP_EMU),
      .ENT_SOFT_EMU   (ENT_SOFT_EMU),
      .RNG_FIFO_DEPTH (RNG_FIFO_DEPTH)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .wb_i           (wb_i),
      .wb_o           (wb_o),
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
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign apb_o = '0;
    assign wb_o  = '0;
    assign axi_o = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    otp_trng_keymgr_tl #(
      .OTP_EMU        (OTP_EMU),
      .ENT_SOFT_EMU   (ENT_SOFT_EMU),
      .RNG_FIFO_DEPTH (RNG_FIFO_DEPTH)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .tl_i           (tl_i),
      .tl_o           (tl_o),
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
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    otp_trng_keymgr_apb #(
      .OTP_EMU        (OTP_EMU),
      .ENT_SOFT_EMU   (ENT_SOFT_EMU),
      .RNG_FIFO_DEPTH (RNG_FIFO_DEPTH)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .apb_i          (apb_i),
      .apb_o          (apb_o),
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
  end else begin : gen_axi
    assign apb_o = '0;
    assign wb_o  = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    otp_trng_keymgr_axi #(
      .OTP_EMU        (OTP_EMU),
      .ENT_SOFT_EMU   (ENT_SOFT_EMU),
      .RNG_FIFO_DEPTH (RNG_FIFO_DEPTH)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .axi_i          (axi_i),
      .axi_o          (axi_o),
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
  end

endmodule : otp_trng_keymgr_top
