// SPDX-License-Identifier: Apache-2.0
// hub75_geom.sv - Registered derivation of the geometry-dependent values from the live CSR fields.
//   Free-running 2-stage pipeline: results are valid 2 cycles after the CSR fields are stable.

module hub75_geom #(
  parameter int unsigned LANES      = 1,
  parameter int unsigned MAX_CHAIN  = 512,
  parameter int unsigned MAX_PANELS = 32
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  logic [9:0]  pw_i,
  input  logic [5:0]  gx_i,
  input  logic [5:0]  gy_i,
  input  logic [2:0]  scan_log2_i,
  input  logic [1:0]  k_log2_i,
  input  logic [3:0]  depth_i,
  input  logic [2:0]  fmt_i,

  output logic        valid_o,
  output logic [5:0]  np_o,
  output logic [11:0] lp_o,
  output logic [11:0] lc_o,
  output logic [6:0]  ph_o,
  output logic [15:0] wimg_o,
  output logic [13:0] himg_o,
  output logic [11:0] hlane_o
);

  // stage 1
  logic [11:0] np1_q;
  logic [12:0] lp1_q;
  logic [6:0]  ph1_q;
  logic [15:0] wimg1_q;
  logic        ok1_q;

  wire [3:0] sk = 4'(scan_log2_i) + 4'(k_log2_i);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      np1_q   <= '0;
      lp1_q   <= '0;
      ph1_q   <= '0;
      wimg1_q <= '0;
      ok1_q   <= 1'b0;
    end else begin
      np1_q   <= 12'(gx_i) * 12'(gy_i);
      lp1_q   <= 13'(pw_i) << (k_log2_i - 2'd1);
      ph1_q   <= (sk <= 4'd6) ? 7'(7'd1 << sk) : 7'd0;
      wimg1_q <= 16'(gx_i) * 16'(pw_i);
      ok1_q   <= (pw_i >= 10'd4) && (pw_i <= 10'd512) && (pw_i[1:0] == 2'b00) &&
                 (gx_i != 6'd0) && (gy_i != 6'd0) &&
                 (scan_log2_i >= 3'd2) && (scan_log2_i <= 3'd5) &&
                 (k_log2_i >= 2'd1) && (sk <= 4'd6) &&
                 (depth_i >= 4'd8) && (depth_i <= 4'd12) &&
                 (fmt_i <= 3'd4);
    end
  end

  // stage 2
  logic [24:0] lc2_q;
  logic [18:0] hl2_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      valid_o <= 1'b0;
      np_o    <= '0;
      lp_o    <= '0;
      lc_o    <= '0;
      ph_o    <= '0;
      wimg_o  <= '0;
      himg_o  <= '0;
      hlane_o <= '0;
      lc2_q   <= '0;
      hl2_q   <= '0;
    end else begin
      lc2_q   <= 25'(np1_q) * 25'(lp1_q);
      hl2_q   <= 19'(gy_i) * 19'(ph1_q);
      np_o    <= np1_q[5:0];
      lp_o    <= lp1_q[11:0];
      ph_o    <= ph1_q;
      wimg_o  <= wimg1_q;
      lc_o    <= lc2_q[11:0];
      hlane_o <= hl2_q[11:0];
      himg_o  <= 14'(hl2_q * 19'(LANES));
      valid_o <= ok1_q && (np1_q <= 12'(MAX_PANELS)) && (lc2_q <= 25'(MAX_CHAIN)) &&
                 (lc2_q != 25'd0) && (hl2_q <= 19'd2048);
    end
  end

endmodule : hub75_geom
