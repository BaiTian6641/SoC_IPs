// SPDX-License-Identifier: Apache-2.0
// ipc_mailbox_top.sv - Parameterized Quad-Bus Wrapper (WB / AXI4 / TileLink / APB4) for ipc_mailbox_core

module ipc_mailbox_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE   = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_CORES  = 4,
  parameter int unsigned         NUM_CH     = 4,
  parameter int unsigned         NUM_LOCKS  = 32,
  parameter int unsigned         FIFO_DEPTH = 4
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t              wb_i,
  output wb_s2m_t              wb_o,
  input  axi4_m2s_t            axi_i,
  output axi4_s2m_t            axi_o,
  input  tl_m2s_t              tl_i,
  output tl_s2m_t              tl_o,
  input  apb_m2s_t             apb_i,
  output apb_s2m_t             apb_o,

  output logic [NUM_CORES-1:0] irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};
    ipc_mailbox_wb #(
      .NUM_CORES  (NUM_CORES),
      .NUM_CH     (NUM_CH),
      .NUM_LOCKS  (NUM_LOCKS),
      .FIFO_DEPTH (FIFO_DEPTH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
      .irq_o  (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};
    ipc_mailbox_tl #(
      .NUM_CORES  (NUM_CORES),
      .NUM_CH     (NUM_CH),
      .NUM_LOCKS  (NUM_LOCKS),
      .FIFO_DEPTH (FIFO_DEPTH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
      .irq_o  (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};
    ipc_mailbox_apb #(
      .NUM_CORES  (NUM_CORES),
      .NUM_CH     (NUM_CH),
      .NUM_LOCKS  (NUM_LOCKS),
      .FIFO_DEPTH (FIFO_DEPTH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .apb_i  (apb_i),
      .apb_o  (apb_o),
      .irq_o  (irq_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = '0;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};
    ipc_mailbox_axi #(
      .NUM_CORES  (NUM_CORES),
      .NUM_CH     (NUM_CH),
      .NUM_LOCKS  (NUM_LOCKS),
      .FIFO_DEPTH (FIFO_DEPTH)
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i  (axi_i),
      .axi_o  (axi_o),
      .irq_o  (irq_o)
    );
  end

endmodule : ipc_mailbox_top
