// SPDX-License-Identifier: Apache-2.0
// timer_pwm_top.sv - Parameterized Quad-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for timer_pwm_core

module timer_pwm_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
  parameter int unsigned CNT_W  = 16,
  parameter int unsigned NUM_CH = 4
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  input  apb_m2s_t  apb_i,
  output apb_s2m_t  apb_o,

  input  logic [NUM_CH-1:0]   ti_i,
  input  logic                etr_i,
  input  logic [1:0]          brk_i,
  input  logic                index_i,
  input  logic [3:0]          itr_i,
  input  logic                dbg_freeze_i,
  output logic [NUM_CH-1:0]   pwm_o,
  output logic [NUM_CH-1:0]   pwm_oe_o,
  output logic [NUM_CH-1:0]   pwm_n_o,
  output logic [NUM_CH-1:0]   pwm_n_oe_o,
  output logic                trgo_o,
  output logic [1:0]          adc_trg_o,
  output logic                brk_active_o,
  output logic [NUM_CH+2:0]   dma_req_o,
  input  logic [NUM_CH+2:0]   dma_ack_i,
  output logic                irq_o,
  output logic [3:0]          irq_vec_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o = '0;
    assign apb_o = '0;
    timer_pwm_wb #(
      .CNT_W  (CNT_W),
      .NUM_CH (NUM_CH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i  (wb_i),
      .wb_o  (wb_o),
      .ti_i          (ti_i),
      .etr_i         (etr_i),
      .brk_i         (brk_i),
      .index_i       (index_i),
      .itr_i         (itr_i),
      .dbg_freeze_i  (dbg_freeze_i),
      .pwm_o         (pwm_o),
      .pwm_oe_o      (pwm_oe_o),
      .pwm_n_o       (pwm_n_o),
      .pwm_n_oe_o    (pwm_n_oe_o),
      .trgo_o        (trgo_o),
      .adc_trg_o     (adc_trg_o),
      .brk_active_o  (brk_active_o),
      .dma_req_o     (dma_req_o),
      .dma_ack_i     (dma_ack_i),
      .irq_o         (irq_o),
      .irq_vec_o     (irq_vec_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    timer_pwm_tl #(
      .CNT_W  (CNT_W),
      .NUM_CH (NUM_CH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i  (tl_i),
      .tl_o  (tl_o),
      .ti_i          (ti_i),
      .etr_i         (etr_i),
      .brk_i         (brk_i),
      .index_i       (index_i),
      .itr_i         (itr_i),
      .dbg_freeze_i  (dbg_freeze_i),
      .pwm_o         (pwm_o),
      .pwm_oe_o      (pwm_oe_o),
      .pwm_n_o       (pwm_n_o),
      .pwm_n_oe_o    (pwm_n_oe_o),
      .trgo_o        (trgo_o),
      .adc_trg_o     (adc_trg_o),
      .brk_active_o  (brk_active_o),
      .dma_req_o     (dma_req_o),
      .dma_ack_i     (dma_ack_i),
      .irq_o         (irq_o),
      .irq_vec_o     (irq_vec_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o = '0;
    assign axi_o = '0;
    assign tl_o = '0;
    timer_pwm_apb #(
      .CNT_W  (CNT_W),
      .NUM_CH (NUM_CH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .apb_i (apb_i),
      .apb_o (apb_o),
      .ti_i          (ti_i),
      .etr_i         (etr_i),
      .brk_i         (brk_i),
      .index_i       (index_i),
      .itr_i         (itr_i),
      .dbg_freeze_i  (dbg_freeze_i),
      .pwm_o         (pwm_o),
      .pwm_oe_o      (pwm_oe_o),
      .pwm_n_o       (pwm_n_o),
      .pwm_n_oe_o    (pwm_n_oe_o),
      .trgo_o        (trgo_o),
      .adc_trg_o     (adc_trg_o),
      .brk_active_o  (brk_active_o),
      .dma_req_o     (dma_req_o),
      .dma_ack_i     (dma_ack_i),
      .irq_o         (irq_o),
      .irq_vec_o     (irq_vec_o)
    );
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    assign apb_o = '0;
    timer_pwm_axi #(
      .CNT_W  (CNT_W),
      .NUM_CH (NUM_CH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
      .ti_i          (ti_i),
      .etr_i         (etr_i),
      .brk_i         (brk_i),
      .index_i       (index_i),
      .itr_i         (itr_i),
      .dbg_freeze_i  (dbg_freeze_i),
      .pwm_o         (pwm_o),
      .pwm_oe_o      (pwm_oe_o),
      .pwm_n_o       (pwm_n_o),
      .pwm_n_oe_o    (pwm_n_oe_o),
      .trgo_o        (trgo_o),
      .adc_trg_o     (adc_trg_o),
      .brk_active_o  (brk_active_o),
      .dma_req_o     (dma_req_o),
      .dma_ack_i     (dma_ack_i),
      .irq_o         (irq_o),
      .irq_vec_o     (irq_vec_o)
    );
  end

endmodule : timer_pwm_top
