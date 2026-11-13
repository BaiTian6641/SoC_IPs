// SPDX-License-Identifier: Apache-2.0
// axi4_reg_slice.sv - Full-Throughput Zero-Bubble 5-Channel AXI4 Register Slice
// Breaks combinational timing paths on AW, W, B, AR, and R using skid_buffer instances.

module axi4_reg_slice
  import bus_pkg::*;
#(
  parameter bit BYPASS    = 1'b0,
  parameter bit BYPASS_AW = BYPASS,
  parameter bit BYPASS_W  = BYPASS,
  parameter bit BYPASS_B  = BYPASS,
  parameter bit BYPASS_AR = BYPASS,
  parameter bit BYPASS_R  = BYPASS
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Upstream (from Master)
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,

  // Downstream (to Slave)
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i
);

  typedef struct packed {
    logic [3:0]  id;
    logic [31:0] addr;
    logic [7:0]  len;
    logic [2:0]  size;
    logic [1:0]  burst;
    logic [2:0]  prot;
    logic [3:0]  qos;
  } axi_ax_pay_t;

  typedef struct packed {
    logic [31:0] data;
    logic [3:0]  strb;
    logic        last;
  } axi_w_pay_t;

  typedef struct packed {
    logic [3:0]  id;
    logic [1:0]  resp;
  } axi_b_pay_t;

  typedef struct packed {
    logic [3:0]  id;
    logic [31:0] data;
    logic [1:0]  resp;
    logic        last;
  } axi_r_pay_t;

  // AW Channel
  if (BYPASS_AW) begin : gen_aw_byp
    assign m_axi_o.awid    = s_axi_i.awid;
    assign m_axi_o.awaddr  = s_axi_i.awaddr;
    assign m_axi_o.awlen   = s_axi_i.awlen;
    assign m_axi_o.awsize  = s_axi_i.awsize;
    assign m_axi_o.awburst = s_axi_i.awburst;
    assign m_axi_o.awprot  = s_axi_i.awprot;
    assign m_axi_o.awqos   = s_axi_i.awqos;
    assign m_axi_o.awvalid = s_axi_i.awvalid;
    assign s_axi_o.awready = m_axi_i.awready;
  end else begin : gen_aw_reg
    axi_ax_pay_t aw_in, aw_out;
    assign aw_in = '{id: s_axi_i.awid, addr: s_axi_i.awaddr, len: s_axi_i.awlen,
                     size: s_axi_i.awsize, burst: s_axi_i.awburst, prot: s_axi_i.awprot, qos: s_axi_i.awqos};
    skid_buffer #(.T(axi_ax_pay_t)) u_aw (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (s_axi_i.awvalid),
      .s_ready_o (s_axi_o.awready),
      .s_data_i  (aw_in),
      .m_valid_o (m_axi_o.awvalid),
      .m_ready_i (m_axi_i.awready),
      .m_data_o  (aw_out)
    );
    assign m_axi_o.awid    = aw_out.id;
    assign m_axi_o.awaddr  = aw_out.addr;
    assign m_axi_o.awlen   = aw_out.len;
    assign m_axi_o.awsize  = aw_out.size;
    assign m_axi_o.awburst = aw_out.burst;
    assign m_axi_o.awprot  = aw_out.prot;
    assign m_axi_o.awqos   = aw_out.qos;
  end

  // W Channel
  if (BYPASS_W) begin : gen_w_byp
    assign m_axi_o.wdata  = s_axi_i.wdata;
    assign m_axi_o.wstrb  = s_axi_i.wstrb;
    assign m_axi_o.wlast  = s_axi_i.wlast;
    assign m_axi_o.wvalid = s_axi_i.wvalid;
    assign s_axi_o.wready = m_axi_i.wready;
  end else begin : gen_w_reg
    axi_w_pay_t w_in, w_out;
    assign w_in = '{data: s_axi_i.wdata, strb: s_axi_i.wstrb, last: s_axi_i.wlast};
    skid_buffer #(.T(axi_w_pay_t)) u_w (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (s_axi_i.wvalid),
      .s_ready_o (s_axi_o.wready),
      .s_data_i  (w_in),
      .m_valid_o (m_axi_o.wvalid),
      .m_ready_i (m_axi_i.wready),
      .m_data_o  (w_out)
    );
    assign m_axi_o.wdata = w_out.data;
    assign m_axi_o.wstrb = w_out.strb;
    assign m_axi_o.wlast = w_out.last;
  end

  // B Channel
  if (BYPASS_B) begin : gen_b_byp
    assign s_axi_o.bid    = m_axi_i.bid;
    assign s_axi_o.bresp  = m_axi_i.bresp;
    assign s_axi_o.bvalid = m_axi_i.bvalid;
    assign m_axi_o.bready = s_axi_i.bready;
  end else begin : gen_b_reg
    axi_b_pay_t b_in, b_out;
    assign b_in = '{id: m_axi_i.bid, resp: m_axi_i.bresp};
    skid_buffer #(.T(axi_b_pay_t)) u_b (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (m_axi_i.bvalid),
      .s_ready_o (m_axi_o.bready),
      .s_data_i  (b_in),
      .m_valid_o (s_axi_o.bvalid),
      .m_ready_i (s_axi_i.bready),
      .m_data_o  (b_out)
    );
    assign s_axi_o.bid   = b_out.id;
    assign s_axi_o.bresp = b_out.resp;
  end

  // AR Channel
  if (BYPASS_AR) begin : gen_ar_byp
    assign m_axi_o.arid    = s_axi_i.arid;
    assign m_axi_o.araddr  = s_axi_i.araddr;
    assign m_axi_o.arlen   = s_axi_i.arlen;
    assign m_axi_o.arsize  = s_axi_i.arsize;
    assign m_axi_o.arburst = s_axi_i.arburst;
    assign m_axi_o.arprot  = s_axi_i.arprot;
    assign m_axi_o.arqos   = s_axi_i.arqos;
    assign m_axi_o.arvalid = s_axi_i.arvalid;
    assign s_axi_o.arready = m_axi_i.arready;
  end else begin : gen_ar_reg
    axi_ax_pay_t ar_in, ar_out;
    assign ar_in = '{id: s_axi_i.arid, addr: s_axi_i.araddr, len: s_axi_i.arlen,
                     size: s_axi_i.arsize, burst: s_axi_i.arburst, prot: s_axi_i.arprot, qos: s_axi_i.arqos};
    skid_buffer #(.T(axi_ax_pay_t)) u_ar (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (s_axi_i.arvalid),
      .s_ready_o (s_axi_o.arready),
      .s_data_i  (ar_in),
      .m_valid_o (m_axi_o.arvalid),
      .m_ready_i (m_axi_i.arready),
      .m_data_o  (ar_out)
    );
    assign m_axi_o.arid    = ar_out.id;
    assign m_axi_o.araddr  = ar_out.addr;
    assign m_axi_o.arlen   = ar_out.len;
    assign m_axi_o.arsize  = ar_out.size;
    assign m_axi_o.arburst = ar_out.burst;
    assign m_axi_o.arprot  = ar_out.prot;
    assign m_axi_o.arqos   = ar_out.qos;
  end

  // R Channel
  if (BYPASS_R) begin : gen_r_byp
    assign s_axi_o.rid    = m_axi_i.rid;
    assign s_axi_o.rdata  = m_axi_i.rdata;
    assign s_axi_o.rresp  = m_axi_i.rresp;
    assign s_axi_o.rlast  = m_axi_i.rlast;
    assign s_axi_o.rvalid = m_axi_i.rvalid;
    assign m_axi_o.rready = s_axi_i.rready;
  end else begin : gen_r_reg
    axi_r_pay_t r_in, r_out;
    assign r_in = '{id: m_axi_i.rid, data: m_axi_i.rdata, resp: m_axi_i.rresp, last: m_axi_i.rlast};
    skid_buffer #(.T(axi_r_pay_t)) u_r (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .s_valid_i (m_axi_i.rvalid),
      .s_ready_o (m_axi_o.rready),
      .s_data_i  (r_in),
      .m_valid_o (s_axi_o.rvalid),
      .m_ready_i (s_axi_i.rready),
      .m_data_o  (r_out)
    );
    assign s_axi_o.rid   = r_out.id;
    assign s_axi_o.rdata = r_out.data;
    assign s_axi_o.rresp = r_out.resp;
    assign s_axi_o.rlast = r_out.last;
  end

endmodule : axi4_reg_slice
