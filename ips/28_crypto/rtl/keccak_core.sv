// SPDX-License-Identifier: Apache-2.0
// keccak_core.sv - FIPS 202 Keccak-f[1600] permutation core (1 round/cycle, 24 rounds).
//
// 1600-bit state arranged as 25 x 64-bit lanes (lane idx = x + 5*y, x,y in 0..4).
// Provides a 32-bit half-lane XOR write port for absorbing and a 32-bit read port
// for squeezing (both in little-endian lane order: word 2*k = lane[31:0], 2*k+1 = lane[63:32]).

module keccak_core (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        wipe_i,
  input  logic        clear_i,
  input  logic        xor_en_i,
  input  logic [5:0]  wr_widx_i,  // 0..49 (32-bit word index into 1600-bit state)
  input  logic [31:0] wr_wdata_i, // little-endian half-lane word
  input  logic        start_i,
  output logic        busy_o,
  output logic        done_o,
  input  logic [5:0]  rd_widx_i,  // 0..49
  output logic [31:0] rd_wdata_o
);

  typedef logic [4:0][4:0][63:0] kstate_t;

  // FIPS 202 Sec. 3.2.5 round constants RC[0..23]
  localparam logic [63:0] RC [24] = '{
    64'h0000000000000001, 64'h0000000000008082, 64'h800000000000808a, 64'h8000000080008000,
    64'h000000000000808b, 64'h0000000080000001, 64'h8000000080008081, 64'h8000000000008009,
    64'h000000000000008a, 64'h0000000000000088, 64'h0000000080008009, 64'h000000008000000a,
    64'h000000008000808b, 64'h800000000000008b, 64'h8000000000008089, 64'h8000000000008003,
    64'h8000000000008002, 64'h8000000000000080, 64'h000000000000800a, 64'h800000008000000a,
    64'h8000000080008081, 64'h8000000000008080, 64'h0000000080000001, 64'h8000000080008008
  };

  // FIPS 202 Sec. 3.2.2 rho rotation offsets RHO[x][y]
  function automatic int unsigned rho_off(input int x, input int y);
    unique case ({3'(y), 3'(x)})
      {3'd0, 3'd0}: return  0;
      {3'd0, 3'd1}: return  1;
      {3'd0, 3'd2}: return 62;
      {3'd0, 3'd3}: return 28;
      {3'd0, 3'd4}: return 27;
      {3'd1, 3'd0}: return 36;
      {3'd1, 3'd1}: return 44;
      {3'd1, 3'd2}: return  6;
      {3'd1, 3'd3}: return 55;
      {3'd1, 3'd4}: return 20;
      {3'd2, 3'd0}: return  3;
      {3'd2, 3'd1}: return 10;
      {3'd2, 3'd2}: return 43;
      {3'd2, 3'd3}: return 25;
      {3'd2, 3'd4}: return 39;
      {3'd3, 3'd0}: return 41;
      {3'd3, 3'd1}: return 45;
      {3'd3, 3'd2}: return 15;
      {3'd3, 3'd3}: return 21;
      {3'd3, 3'd4}: return  8;
      {3'd4, 3'd0}: return 18;
      {3'd4, 3'd1}: return  2;
      {3'd4, 3'd2}: return 61;
      {3'd4, 3'd3}: return 56;
      default:      return 14;
    endcase
  endfunction

  function automatic logic [63:0] rotl64(input logic [63:0] v, input int unsigned n);
    if (n == 0) return v;
    else        return (v << n) | (v >> (64 - n));
  endfunction

  kstate_t     st_q, st_nxt;
  logic        busy_q, done_q;
  logic [4:0]  rnd_q;

  // One full Keccak-f[1600] round (theta -> rho+pi -> chi -> iota)
  always_comb begin
    logic [4:0][63:0] c, d;
    kstate_t b;
    for (int x = 0; x < 5; x++)
      c[x] = st_q[x][0] ^ st_q[x][1] ^ st_q[x][2] ^ st_q[x][3] ^ st_q[x][4];
    for (int x = 0; x < 5; x++)
      d[x] = c[(x + 4) % 5] ^ rotl64(c[(x + 1) % 5], 1);
    for (int x = 0; x < 5; x++) begin
      for (int y = 0; y < 5; y++) begin
        b[y][(2 * x + 3 * y) % 5] = rotl64(st_q[x][y] ^ d[x], rho_off(x, y));
      end
    end
    for (int x = 0; x < 5; x++) begin
      for (int y = 0; y < 5; y++) begin
        st_nxt[x][y] = b[x][y] ^ ((~b[(x + 1) % 5][y]) & b[(x + 2) % 5][y]);
      end
    end
    st_nxt[0][0] = st_nxt[0][0] ^ RC[rnd_q];
  end

  // Write index decode (lane = wr_widx_i[5:1], half = wr_widx_i[0])
  logic [4:0] wr_lane;
  assign wr_lane = wr_widx_i[5:1];

  // Read mux (lane = rd_widx_i[5:1], half = rd_widx_i[0])
  logic [4:0] rd_lane;
  assign rd_lane = rd_widx_i[5:1];
  always_comb begin
    logic [63:0] lane_v;
    int rx, ry;
    rx = int'(rd_lane) % 5;
    ry = int'(rd_lane) / 5;
    if (rd_lane < 5'd25) lane_v = st_q[rx][ry];
    else                 lane_v = 64'h0;
    rd_wdata_o = rd_widx_i[0] ? lane_v[63:32] : lane_v[31:0];
  end

  assign busy_o = busy_q;
  assign done_o = done_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q   <= '0;
      busy_q <= 1'b0;
      done_q <= 1'b0;
      rnd_q  <= 5'd0;
    end else begin
      done_q <= 1'b0;
      if (wipe_i) begin
        st_q   <= '0;
        busy_q <= 1'b0;
        rnd_q  <= 5'd0;
      end else if (clear_i) begin
        st_q   <= '0;
        busy_q <= 1'b0;
        rnd_q  <= 5'd0;
      end else if (busy_q) begin
        st_q <= st_nxt;
        if (rnd_q == 5'd23) begin
          busy_q <= 1'b0;
          done_q <= 1'b1;
        end else begin
          rnd_q <= rnd_q + 5'd1;
        end
      end else begin
        if (xor_en_i && (wr_lane < 5'd25)) begin
          int wx, wy;
          wx = int'(wr_lane) % 5;
          wy = int'(wr_lane) / 5;
          if (wr_widx_i[0]) st_q[wx][wy][63:32] <= st_q[wx][wy][63:32] ^ wr_wdata_i;
          else              st_q[wx][wy][31:0]  <= st_q[wx][wy][31:0]  ^ wr_wdata_i;
        end
        if (start_i) begin
          rnd_q  <= 5'd0;
          busy_q <= 1'b1;
        end
      end
    end
  end

endmodule : keccak_core
