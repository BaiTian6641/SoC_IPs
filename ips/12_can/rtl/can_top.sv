// SPDX-License-Identifier: Apache-2.0
module can_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4
) (
  input  logic      clk_i,
  input  logic      rst_ni,
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  input  apb_m2s_t  apb_i,
  output apb_s2m_t  apb_o,
  output logic     can_tx_o,
  input  logic     can_rx_i,
  output logic     can_stb_no,
  output logic     irq_o
);
  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign apb_o = '0;
    assign axi_o = '0; assign tl_o = '0;
    can_wb u_imp (.clk_i(clk_i), .rst_ni(rst_ni), .wb_i(wb_i), .wb_o(wb_o),
    .can_tx_o   (can_tx_o),
    .can_rx_i   (can_rx_i),
    .can_stb_no (can_stb_no),
    .irq_o      (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign apb_o = '0;
    assign wb_o = '0; assign axi_o = '0;
    can_tl u_imp (.clk_i(clk_i), .rst_ni(rst_ni), .tl_i(tl_i), .tl_o(tl_o),
    .can_tx_o   (can_tx_o),
    .can_rx_i   (can_rx_i),
    .can_stb_no (can_stb_no),
    .irq_o      (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o = '0;
    assign axi_o = '0;
    assign tl_o = '0;
    can_apb u_imp (.clk_i(clk_i), .rst_ni(rst_ni), .apb_i(apb_i), .apb_o(apb_o),
    .can_tx_o   (can_tx_o),
    .can_rx_i   (can_rx_i),
    .can_stb_no (can_stb_no),
    .irq_o      (irq_o)
    );
  end else begin : gen_axi
    assign apb_o = '0;
    assign wb_o = '0; assign tl_o = '0;
    can_axi u_imp (.clk_i(clk_i), .rst_ni(rst_ni), .axi_i(axi_i), .axi_o(axi_o),
    .can_tx_o   (can_tx_o),
    .can_rx_i   (can_rx_i),
    .can_stb_no (can_stb_no),
    .irq_o      (irq_o)
    );
  end
endmodule : can_top
