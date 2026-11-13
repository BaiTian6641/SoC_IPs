// SPDX-License-Identifier: Apache-2.0
// tl_reg_slice.sv - Full-Throughput Zero-Bubble TileLink (Channel A & Channel D) Register Slice

module tl_reg_slice
  import bus_pkg::*;
#(
  parameter bit BYPASS   = 1'b0,
  parameter bit BYPASS_A = BYPASS,
  parameter bit BYPASS_D = BYPASS
) (
  input  logic    clk_i,
  input  logic    rst_ni,

  // Upstream (from Host)
  input  tl_m2s_t s_tl_i,
  output tl_s2m_t s_tl_o,

  // Downstream (to Device)
  output tl_m2s_t m_tl_o,
  input  tl_s2m_t m_tl_i
);

  typedef struct packed {
    logic [2:0]  opcode;
    logic [2:0]  param;
    logic [3:0]  size;
    logic [7:0]  source;
    logic [31:0] address;
    logic [3:0]  mask;
    logic [31:0] data;
    logic        corrupt;
  } tl_a_pay_t;

  typedef struct packed {
    logic [2:0]  opcode;
    logic [1:0]  param;
    logic [3:0]  size;
    logic [7:0]  source;
    logic [3:0]  sink;
    logic        denied;
    logic [31:0] data;
    logic        corrupt;
  } tl_d_pay_t;

  if (BYPASS_A) begin : gen_a_byp
    assign m_tl_o.a_valid   = s_tl_i.a_valid;
    assign m_tl_o.a_opcode  = s_tl_i.a_opcode;
    assign m_tl_o.a_param   = s_tl_i.a_param;
    assign m_tl_o.a_size    = s_tl_i.a_size;
    assign m_tl_o.a_source  = s_tl_i.a_source;
    assign m_tl_o.a_address = s_tl_i.a_address;
    assign m_tl_o.a_mask    = s_tl_i.a_mask;
    assign m_tl_o.a_data    = s_tl_i.a_data;
    assign m_tl_o.a_corrupt = s_tl_i.a_corrupt;
    assign s_tl_o.a_ready   = m_tl_i.a_ready;
  end else begin : gen_a_reg
    tl_a_pay_t a_in, a_out;
    assign a_in = '{opcode: s_tl_i.a_opcode, param: s_tl_i.a_param, size: s_tl_i.a_size,
                    source: s_tl_i.a_source, address: s_tl_i.a_address, mask: s_tl_i.a_mask,
                    data: s_tl_i.a_data, corrupt: s_tl_i.a_corrupt};
    skid_buffer #(.T(tl_a_pay_t)) u_ch_a (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (s_tl_i.a_valid),
      .s_ready_o (s_tl_o.a_ready),
      .s_data_i  (a_in),
      .m_valid_o (m_tl_o.a_valid),
      .m_ready_i (m_tl_i.a_ready),
      .m_data_o  (a_out)
    );
    assign m_tl_o.a_opcode  = a_out.opcode;
    assign m_tl_o.a_param   = a_out.param;
    assign m_tl_o.a_size    = a_out.size;
    assign m_tl_o.a_source  = a_out.source;
    assign m_tl_o.a_address = a_out.address;
    assign m_tl_o.a_mask    = a_out.mask;
    assign m_tl_o.a_data    = a_out.data;
    assign m_tl_o.a_corrupt = a_out.corrupt;
  end

  if (BYPASS_D) begin : gen_d_byp
    assign s_tl_o.d_valid   = m_tl_i.d_valid;
    assign s_tl_o.d_opcode  = m_tl_i.d_opcode;
    assign s_tl_o.d_param   = m_tl_i.d_param;
    assign s_tl_o.d_size    = m_tl_i.d_size;
    assign s_tl_o.d_source  = m_tl_i.d_source;
    assign s_tl_o.d_sink    = m_tl_i.d_sink;
    assign s_tl_o.d_denied  = m_tl_i.d_denied;
    assign s_tl_o.d_data    = m_tl_i.d_data;
    assign s_tl_o.d_corrupt = m_tl_i.d_corrupt;
    assign m_tl_o.d_ready   = s_tl_i.d_ready;
  end else begin : gen_d_reg
    tl_d_pay_t d_in, d_out;
    assign d_in = '{opcode: m_tl_i.d_opcode, param: m_tl_i.d_param, size: m_tl_i.d_size,
                    source: m_tl_i.d_source, sink: m_tl_i.d_sink, denied: m_tl_i.d_denied,
                    data: m_tl_i.d_data, corrupt: m_tl_i.d_corrupt};
    skid_buffer #(.T(tl_d_pay_t)) u_ch_d (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (m_tl_i.d_valid),
      .s_ready_o (m_tl_o.d_ready),
      .s_data_i  (d_in),
      .m_valid_o (s_tl_o.d_valid),
      .m_ready_i (s_tl_i.d_ready),
      .m_data_o  (d_out)
    );
    assign s_tl_o.d_opcode  = d_out.opcode;
    assign s_tl_o.d_param   = d_out.param;
    assign s_tl_o.d_size    = d_out.size;
    assign s_tl_o.d_source  = d_out.source;
    assign s_tl_o.d_sink    = d_out.sink;
    assign s_tl_o.d_denied  = d_out.denied;
    assign s_tl_o.d_data    = d_out.data;
    assign s_tl_o.d_corrupt = d_out.corrupt;
  end

endmodule : tl_reg_slice
