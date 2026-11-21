// SPDX-License-Identifier: Apache-2.0
// xspi_aes128.sv - Self-contained Compact Iterative AES-128 Forward Cipher (FIPS-197)
//
// Computes 128-bit AES-128 encryption (used as CTR-mode keystream generator for OTFAD XIP).
// 1 round per clock cycle (11 cycles from start_i to valid_o) with on-the-fly key schedule.
// Pure synthesizable IEEE 1800-2017 SystemVerilog (Yosys/FPGA/ASIC friendly).

module xspi_aes128 (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic         start_i,
  input  logic [127:0] key_i,
  input  logic [127:0] block_i,
  output logic         busy_o,
  output logic         valid_o,
  output logic [127:0] block_o
);

  localparam logic [7:0] SBOX [0:255] = '{
    8'h63, 8'h7C, 8'h77, 8'h7B, 8'hF2, 8'h6B, 8'h6F, 8'hC5, 8'h30, 8'h01, 8'h67, 8'h2B, 8'hFE, 8'hD7, 8'hAB, 8'h76,
    8'hCA, 8'h82, 8'hC9, 8'h7D, 8'hFA, 8'h59, 8'h47, 8'hF0, 8'hAD, 8'hD4, 8'hA2, 8'hAF, 8'h9C, 8'hA4, 8'h72, 8'hC0,
    8'hB7, 8'hFD, 8'h93, 8'h26, 8'h36, 8'h3F, 8'hF7, 8'hCC, 8'h34, 8'hA5, 8'hE5, 8'hF1, 8'h71, 8'hD8, 8'h31, 8'h15,
    8'h04, 8'hC7, 8'h23, 8'hC3, 8'h18, 8'h96, 8'h05, 8'h9A, 8'h07, 8'h12, 8'h80, 8'hE2, 8'hEB, 8'h27, 8'hB2, 8'h75,
    8'h09, 8'h83, 8'h2C, 8'h1A, 8'h1B, 8'h6E, 8'h5A, 8'hA0, 8'h52, 8'h3B, 8'hD6, 8'hB3, 8'h29, 8'hE3, 8'h2F, 8'h84,
    8'h53, 8'hD1, 8'h00, 8'hED, 8'h20, 8'hFC, 8'hB1, 8'h5B, 8'h6A, 8'hCB, 8'hBE, 8'h39, 8'h4A, 8'h4C, 8'h58, 8'hCF,
    8'hD0, 8'hEF, 8'hAA, 8'hFB, 8'h43, 8'h4D, 8'h33, 8'h85, 8'h45, 8'hF9, 8'h02, 8'h7F, 8'h50, 8'h3C, 8'h9F, 8'hA8,
    8'h51, 8'hA3, 8'h40, 8'h8F, 8'h92, 8'h9D, 8'h38, 8'hF5, 8'hBC, 8'hB6, 8'hDA, 8'h21, 8'h10, 8'hFF, 8'hF3, 8'hD2,
    8'hCD, 8'h0C, 8'h13, 8'hEC, 8'h5F, 8'h97, 8'h44, 8'h17, 8'hC4, 8'hA7, 8'h7E, 8'h3D, 8'h64, 8'h5D, 8'h19, 8'h73,
    8'h60, 8'h81, 8'h4F, 8'hDC, 8'h22, 8'h2A, 8'h90, 8'h88, 8'h46, 8'hEE, 8'hB8, 8'h14, 8'hDE, 8'h5E, 8'h0B, 8'hDB,
    8'hE0, 8'h32, 8'h3A, 8'h0A, 8'h49, 8'h06, 8'h24, 8'h5C, 8'hC2, 8'hD3, 8'hAC, 8'h62, 8'h91, 8'h95, 8'hE4, 8'h79,
    8'hE7, 8'hC8, 8'h37, 8'h6D, 8'h8D, 8'hD5, 8'h4E, 8'hA9, 8'h6C, 8'h56, 8'hF4, 8'hEA, 8'h65, 8'h7A, 8'hAE, 8'h08,
    8'hBA, 8'h78, 8'h25, 8'h2E, 8'h1C, 8'hA6, 8'hB4, 8'hC6, 8'hE8, 8'hDD, 8'h74, 8'h1F, 8'h4B, 8'hBD, 8'h8B, 8'h8A,
    8'h70, 8'h3E, 8'hB5, 8'h66, 8'h48, 8'h03, 8'hF6, 8'h0E, 8'h61, 8'h35, 8'h57, 8'hB9, 8'h86, 8'hC1, 8'h1D, 8'h9E,
    8'hE1, 8'hF8, 8'h98, 8'h11, 8'h69, 8'hD9, 8'h8E, 8'h94, 8'h9B, 8'h1E, 8'h87, 8'hE9, 8'hCE, 8'h55, 8'h28, 8'hDF,
    8'h8C, 8'hA1, 8'h89, 8'h0D, 8'hBF, 8'hE6, 8'h42, 8'h68, 8'h41, 8'h99, 8'h2D, 8'h0F, 8'hB0, 8'h54, 8'hBB, 8'h16
  };

  function automatic logic [7:0] xtime(input logic [7:0] b);
    return {b[6:0], 1'b0} ^ (b[7] ? 8'h1B : 8'h00);
  endfunction

  function automatic logic [31:0] mix_column(input logic [31:0] col);
    logic [7:0] b0, b1, b2, b3;
    logic [7:0] x0, x1, x2, x3;
    logic [7:0] y0, y1, y2, y3;
    b0 = col[31:24];
    b1 = col[23:16];
    b2 = col[15:8];
    b3 = col[7:0];
    x0 = xtime(b0);
    x1 = xtime(b1);
    x2 = xtime(b2);
    x3 = xtime(b3);
    y0 = x0 ^ (x1 ^ b1) ^ b2 ^ b3;
    y1 = b0 ^ x1 ^ (x2 ^ b2) ^ b3;
    y2 = b0 ^ b1 ^ x2 ^ (x3 ^ b3);
    y3 = (x0 ^ b0) ^ b1 ^ b2 ^ x3;
    return {y0, y1, y2, y3};
  endfunction

  function automatic logic [7:0] rcon_val(input logic [3:0] rnd);
    unique case (rnd)
      4'd1:    return 8'h01;
      4'd2:    return 8'h02;
      4'd3:    return 8'h04;
      4'd4:    return 8'h08;
      4'd5:    return 8'h10;
      4'd6:    return 8'h20;
      4'd7:    return 8'h40;
      4'd8:    return 8'h80;
      4'd9:    return 8'h1B;
      4'd10:   return 8'h36;
      default: return 8'h00;
    endcase
  endfunction

  logic [127:0] state_q, rkey_q;
  logic [3:0]   round_q;
  logic         busy_q, valid_q;

  // SubBytes + ShiftRows
  logic [7:0]   sb [0:15];
  logic [127:0] sr_state;
  logic [127:0] mc_state;
  logic [127:0] next_rk;

  always_comb begin
    automatic logic [31:0] k0, k1, k2, k3, rot, subw, w0, w1, w2, w3;
    for (int i = 0; i < 16; i++) begin
      sb[i] = SBOX[state_q[127 - 8*i -: 8]];
    end
    sr_state = {
      sb[0],  sb[5],  sb[10], sb[15],
      sb[4],  sb[9],  sb[14], sb[3],
      sb[8],  sb[13], sb[2],  sb[7],
      sb[12], sb[1],  sb[6],  sb[11]
    };
    mc_state = {
      mix_column(sr_state[127:96]),
      mix_column(sr_state[95:64]),
      mix_column(sr_state[63:32]),
      mix_column(sr_state[31:0])
    };

    k0   = rkey_q[127:96];
    k1   = rkey_q[95:64];
    k2   = rkey_q[63:32];
    k3   = rkey_q[31:0];
    rot  = {k3[23:0], k3[31:24]};
    subw = {SBOX[rot[31:24]], SBOX[rot[23:16]], SBOX[rot[15:8]], SBOX[rot[7:0]]};
    w0   = k0 ^ subw ^ {rcon_val(round_q), 24'h00_0000};
    w1   = k1 ^ w0;
    w2   = k2 ^ w1;
    w3   = k3 ^ w2;
    next_rk = {w0, w1, w2, w3};
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q <= '0;
      rkey_q  <= '0;
      round_q <= 4'd0;
      busy_q  <= 1'b0;
      valid_q <= 1'b0;
    end else begin
      if (start_i) begin
        state_q <= block_i ^ key_i;
        rkey_q  <= key_i;
        round_q <= 4'd1;
        busy_q  <= 1'b1;
        valid_q <= 1'b0;
      end else if (busy_q) begin
        rkey_q <= next_rk;
        if (round_q == 4'd10) begin
          state_q <= sr_state ^ next_rk;
          busy_q  <= 1'b0;
          valid_q <= 1'b1;
        end else begin
          state_q <= mc_state ^ next_rk;
          round_q <= round_q + 4'd1;
        end
      end
    end
  end

  assign busy_o  = busy_q;
  assign valid_o = valid_q;
  assign block_o = state_q;

endmodule : xspi_aes128
