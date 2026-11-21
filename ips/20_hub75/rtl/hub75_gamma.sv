// SPDX-License-Identifier: Apache-2.0
// hub75_gamma.sv - Three 256x12 gamma LUT RAMs (R,G,B), reset/reload initialisation FSM from ROM presets,
//                  CPU read/write port and a registered pixel-pipeline lookup port.

module hub75_gamma
  import hub75_gamma_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic [1:0]  preset_i,
  input  logic        reload_i,     // 1-cycle pulse: re-run ROM initialisation
  output logic        busy_o,       // ROM initialisation running

  // pixel pipeline lookup (address now, data 1 cycle later)
  input  logic [7:0]  pix_r_i,
  input  logic [7:0]  pix_g_i,
  input  logic [7:0]  pix_b_i,
  output logic [11:0] lut_r_o,
  output logic [11:0] lut_g_o,
  output logic [11:0] lut_b_o,

  // CPU port (writes ignored while busy_o; core stalls the bus)
  input  logic        cpu_we_i,
  input  logic [1:0]  cpu_ch_i,
  input  logic [7:0]  cpu_idx_i,
  input  logic [11:0] cpu_wdata_i,
  input  logic [1:0]  cpu_rch_i,    // read channel/index (data 1 cycle later)
  input  logic [7:0]  cpu_ridx_i,
  output logic [11:0] cpu_rdata_o
);

  logic [8:0] init_idx_q;   // 0..255 writing, bit 8 = done
  logic       busy_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy_q     <= 1'b1;
      init_idx_q <= '0;
    end else if (reload_i) begin
      busy_q     <= 1'b1;
      init_idx_q <= '0;
    end else if (busy_q) begin
      if (init_idx_q == 9'd255) begin
        busy_q     <= 1'b0;
        init_idx_q <= '0;
      end else begin
        init_idx_q <= init_idx_q + 9'd1;
      end
    end
  end

  assign busy_o = busy_q;

  wire [7:0]  w_idx  = busy_q ? init_idx_q[7:0] : cpu_idx_i;
  wire [11:0] w_data = busy_q ? gamma_rom(preset_i, init_idx_q[7:0]) : cpu_wdata_i;
  wire [2:0]  w_en   = busy_q ? 3'b111 : (cpu_we_i ? (3'b001 << cpu_ch_i) : 3'b000);

  logic [11:0] rdb [3];
  assign cpu_rdata_o = (cpu_rch_i == 2'd0) ? rdb[0] : (cpu_rch_i == 2'd1) ? rdb[1] : rdb[2];

  hub75_ram_1w2r #(.W(12), .DEPTH(256)) u_lut_r (
    .clk_i(clk_i), .we_i(w_en[0]), .waddr_i(w_idx), .wdata_i(w_data),
    .raddr_a_i(pix_r_i), .rdata_a_o(lut_r_o), .raddr_b_i(cpu_ridx_i), .rdata_b_o(rdb[0]));
  hub75_ram_1w2r #(.W(12), .DEPTH(256)) u_lut_g (
    .clk_i(clk_i), .we_i(w_en[1]), .waddr_i(w_idx), .wdata_i(w_data),
    .raddr_a_i(pix_g_i), .rdata_a_o(lut_g_o), .raddr_b_i(cpu_ridx_i), .rdata_b_o(rdb[1]));
  hub75_ram_1w2r #(.W(12), .DEPTH(256)) u_lut_b (
    .clk_i(clk_i), .we_i(w_en[2]), .waddr_i(w_idx), .wdata_i(w_data),
    .raddr_a_i(pix_b_i), .rdata_a_o(lut_b_o), .raddr_b_i(cpu_ridx_i), .rdata_b_o(rdb[2]));

endmodule : hub75_gamma
