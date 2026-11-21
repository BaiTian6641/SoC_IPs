// SPDX-License-Identifier: Apache-2.0
// adc_pdm_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for adc_pdm_core

module adc_pdm_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE       = bus_pkg::BUS_AXI4,
  parameter int unsigned         ADC_FIFO_DEPTH = 16,
  parameter int unsigned         PDM_FIFO_DEPTH = 16
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t     wb_i,
  output wb_s2m_t     wb_o,
  input  axi4_m2s_t   axi_i,
  output axi4_s2m_t   axi_o,
  input  tl_m2s_t     tl_i,
  output tl_s2m_t     tl_o,
  input  apb_m2s_t    apb_i,
  output apb_s2m_t    apb_o,

  // SAR-ADC Macro Boundary
  output logic        adc_start_o,
  output logic [3:0]  adc_ch_sel_o,
  input  logic        adc_eoc_i,
  input  logic [11:0] adc_data_i,
  input  logic        ext_trgo_i,

  // PDM Microphone Boundary
  output logic        pdm_clk_o,
  input  logic        pdm_data_i,

  // DMA & Interrupts
  output logic        dma_adc_req_o,
  output logic        dma_pdm_req_o,
  output logic        awd_irq_o,
  output logic        irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    adc_pdm_wb #(
      .ADC_FIFO_DEPTH (ADC_FIFO_DEPTH),
      .PDM_FIFO_DEPTH (PDM_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .wb_i          (wb_i),
      .wb_o          (wb_o),
      .adc_start_o   (adc_start_o),
      .adc_ch_sel_o  (adc_ch_sel_o),
      .adc_eoc_i     (adc_eoc_i),
      .adc_data_i    (adc_data_i),
      .ext_trgo_i    (ext_trgo_i),
      .pdm_clk_o     (pdm_clk_o),
      .pdm_data_i    (pdm_data_i),
      .dma_adc_req_o (dma_adc_req_o),
      .dma_pdm_req_o (dma_pdm_req_o),
      .awd_irq_o     (awd_irq_o),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    adc_pdm_tl #(
      .ADC_FIFO_DEPTH (ADC_FIFO_DEPTH),
      .PDM_FIFO_DEPTH (PDM_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .tl_i          (tl_i),
      .tl_o          (tl_o),
      .adc_start_o   (adc_start_o),
      .adc_ch_sel_o  (adc_ch_sel_o),
      .adc_eoc_i     (adc_eoc_i),
      .adc_data_i    (adc_data_i),
      .ext_trgo_i    (ext_trgo_i),
      .pdm_clk_o     (pdm_clk_o),
      .pdm_data_i    (pdm_data_i),
      .dma_adc_req_o (dma_adc_req_o),
      .dma_pdm_req_o (dma_pdm_req_o),
      .awd_irq_o     (awd_irq_o),
      .irq_o         (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    adc_pdm_apb #(
      .ADC_FIFO_DEPTH (ADC_FIFO_DEPTH),
      .PDM_FIFO_DEPTH (PDM_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .apb_i         (apb_i),
      .apb_o         (apb_o),
      .adc_start_o   (adc_start_o),
      .adc_ch_sel_o  (adc_ch_sel_o),
      .adc_eoc_i     (adc_eoc_i),
      .adc_data_i    (adc_data_i),
      .ext_trgo_i    (ext_trgo_i),
      .pdm_clk_o     (pdm_clk_o),
      .pdm_data_i    (pdm_data_i),
      .dma_adc_req_o (dma_adc_req_o),
      .dma_pdm_req_o (dma_pdm_req_o),
      .awd_irq_o     (awd_irq_o),
      .irq_o         (irq_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    adc_pdm_axi #(
      .ADC_FIFO_DEPTH (ADC_FIFO_DEPTH),
      .PDM_FIFO_DEPTH (PDM_FIFO_DEPTH)
    ) u_imp (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .axi_i         (axi_i),
      .axi_o         (axi_o),
      .adc_start_o   (adc_start_o),
      .adc_ch_sel_o  (adc_ch_sel_o),
      .adc_eoc_i     (adc_eoc_i),
      .adc_data_i    (adc_data_i),
      .ext_trgo_i    (ext_trgo_i),
      .pdm_clk_o     (pdm_clk_o),
      .pdm_data_i    (pdm_data_i),
      .dma_adc_req_o (dma_adc_req_o),
      .dma_pdm_req_o (dma_pdm_req_o),
      .awd_irq_o     (awd_irq_o),
      .irq_o         (irq_o)
    );
  end

endmodule : adc_pdm_top
