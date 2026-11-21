// SPDX-License-Identifier: Apache-2.0
// tb_axis_utils.sv - Self-checking constrained-random testbench for interconnect/axis/*
//
// Structure (portable to UVM later):
//   axs_tb_pkg  : axs_rng (xorshift32, +seed driven), axs_frame (byte-level frame),
//                 axs_sb (scoreboard / reference model: per-key queues of expected frames)
//   axs_src     : driver  - builds frames from the byte model, drives beats with random gaps
//   axs_snk     : monitor + checker - random tready, AXI-Stream protocol checks
//                 (hold-stable under back-pressure, keep legality, id/dest/user constancy),
//                 re-assembles frames from kept bytes and checks against the scoreboard
//   axs_h_*     : one harness per DUT configuration (register, fifo, frame fifo, async
//                 fifo, width converter, arb mux, demux), each with directed + random phases
//   tb_axis_utils: top - seed plumbing (+seed=N), watchdog, PASS line.
//
// The reference model is byte-level and width-agnostic: a frame is the list of kept
// bytes {tstrb,tdata} plus id/dest/user, so every DUT (whatever its beat width) is
// compared against the same oracle; routing/dropping rules are modelled in axs_src.

package axs_tb_pkg;

  class axs_rng;
    bit [31:0] s;
    function new(int unsigned seed);
      s = 32'(seed) ^ 32'h9E3779B9;
      if (s == 32'd0) s = 32'd1;
    endfunction
    function int unsigned next();
      s = s ^ (s << 13);
      s = s ^ (s >> 17);
      s = s ^ (s << 5);
      return int'(s);
    endfunction
    function int unsigned range(int unsigned lo, int unsigned hi);
      return lo + (next() % (hi - lo + 1));
    endfunction
    function bit pct(int unsigned p);
      return (next() % 100) < p;
    endfunction
  endclass

  class axs_frame;
    bit [8:0]    bytes[$];   // {strb, data}
    int unsigned id, dest, user, seq;
    bit          bad;
  endclass

  class axs_sb;
    axs_frame    exp[8][$];
    int unsigned n_put, n_got, n_skipped;

    function void put(int key, axs_frame f);
      exp[key].push_back(f);
      n_put++;
    endfunction

    function int unsigned pending();
      int unsigned t = 0;
      for (int k = 0; k < 8; k++) t += exp[k].size();
      return t;
    endfunction

    function void flush();
      for (int k = 0; k < 8; k++) exp[k].delete();
    endfunction

    function void cmp(string nm, axs_frame e, axs_frame g, bit chk_user);
      if (e.bytes.size() != g.bytes.size())
        $fatal(1, "[%s] frame length mismatch exp=%0d got=%0d (seq=%0d)", nm, e.bytes.size(), g.bytes.size(), e.seq);
      for (int i = 0; i < e.bytes.size(); i++)
        if (e.bytes[i] !== g.bytes[i])
          $fatal(1, "[%s] byte %0d mismatch exp={strb=%0b,%02h} got={strb=%0b,%02h} (seq=%0d)", nm, i,
                 e.bytes[i][8], e.bytes[i][7:0], g.bytes[i][8], g.bytes[i][7:0], e.seq);
      if (e.id != g.id)     $fatal(1, "[%s] tid mismatch exp=%0d got=%0d (seq=%0d)", nm, e.id, g.id, e.seq);
      if (e.dest != g.dest) $fatal(1, "[%s] tdest mismatch exp=%0d got=%0d (seq=%0d)", nm, e.dest, g.dest, e.seq);
      if (chk_user && (e.user != g.user))
        $fatal(1, "[%s] tuser mismatch exp=%0d got=%0d (seq=%0d)", nm, e.user, g.user, e.seq);
    endfunction

    function void check(string nm, int key, axs_frame g, bit chk_user, bit lossy);
      axs_frame e;
      int       hit;
      if (exp[key].size() == 0)
        $fatal(1, "[%s] unexpected frame on key %0d (len=%0d user=%0d)", nm, key, g.bytes.size(), g.user);
      hit = 0;
      if (lossy) begin
        hit = -1;
        for (int i = 0; i < exp[key].size(); i++)
          if (hit < 0 && exp[key][i].user == g.user) hit = i;
        if (hit < 0) $fatal(1, "[%s] frame with seq/user %0d was never sent", nm, g.user);
        for (int i = 0; i < hit; i++) begin
          void'(exp[key].pop_front());
          n_skipped++;
        end
      end
      e = exp[key].pop_front();
      cmp(nm, e, g, chk_user);
      n_got++;
    endfunction
  endclass

  axs_sb sb_tab[128];
  function automatic axs_sb get_sb(int i);
    if (sb_tab[i] == null) sb_tab[i] = new();
    return sb_tab[i];
  endfunction

endpackage : axs_tb_pkg

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------
module axs_src
  import axs_tb_pkg::*;
#(
  parameter int DATA_W   = 32,
  parameter int ID_W     = 1,
  parameter int DEST_W   = 1,
  parameter int USER_W   = 1,
  parameter int SB       = 0,
  parameter int KEY      = 0,
  parameter int KEY_MODE = 0,   // 0 fixed KEY, 1 by tdest (>=KEY_N dropped), 2 tdest % KEY_N
  parameter int KEY_N    = 1,
  parameter int ID_FIXED = -1,
  parameter int DEST_MAX = 0,
  parameter int USER_MODE = 0,  // 0 random per frame, 1 = sequence number, 2 = bad flag in last tuser[0]
  parameter int MAXLEN   = 40,  // bytes
  parameter int OVERSIZE = 0,   // frames with more beats than this are expected to be dropped (0 = off)
  parameter bit DROP_BAD = 1'b0
) (
  input  logic                clk_i,
  input  logic                rst_ni,
  output logic                tvalid_o,
  input  logic                tready_i,
  output logic [DATA_W-1:0]   tdata_o,
  output logic [DATA_W/8-1:0] tkeep_o,
  output logic [DATA_W/8-1:0] tstrb_o,
  output logic                tlast_o,
  output logic [ID_W-1:0]     tid_o,
  output logic [DEST_W-1:0]   tdest_o,
  output logic [USER_W-1:0]   tuser_o,
  input  int unsigned         valid_pct
);
  localparam int BYTES = DATA_W / 8;

  axs_rng      rng;
  axs_sb       sb;
  axs_frame    todo[$];
  axs_frame    cur;
  int unsigned pos;
  int unsigned seq_n, n_added, n_excl, n_beats;
  int unsigned log_seq[$];
  int unsigned log_beats[$];

  task automatic init(input int unsigned s);
    rng = new(s);
    sb  = get_sb(SB);
    cur = null;
  endtask

  function automatic bit is_idle();
    return (todo.size() == 0) && (cur == null) && !tvalid_o;
  endfunction

  // lenmode: 0 mixed, 1 one byte, 2 one beat (random keep), 3 exact full beats,
  //          4 one..five beats, 5 uniform 1..MAXLEN bytes
  task automatic add_frames(input int n, input int lenmode);
    for (int i = 0; i < n; i++) begin
      automatic axs_frame f = new();
      automatic int       len;
      automatic int       beats;
      automatic int       key;
      automatic bit       excl;
      automatic int       sel = rng.range(0, 9);
      case (lenmode)
        1:       len = 1;
        2:       len = rng.range(1, BYTES);
        3:       len = BYTES * rng.range(1, (MAXLEN >= BYTES) ? (MAXLEN / BYTES) : 1);
        4:       len = rng.range(1, 5 * BYTES);
        5:       len = rng.range(1, MAXLEN);
        default: begin
          if (sel < 2)       len = 1;
          else if (sel == 2) len = BYTES;
          else if (sel == 3) len = BYTES * rng.range(1, 4);
          else               len = rng.range(1, MAXLEN);
        end
      endcase
      for (int b = 0; b < len; b++)
        f.bytes.push_back({1'(rng.pct(70)), 8'(rng.next())});
      f.id   = (ID_FIXED >= 0) ? int'(ID_FIXED) : rng.range(0, (1 << ID_W) - 1);
      f.dest = rng.range(0, DEST_MAX);
      f.seq  = seq_n;
      f.bad  = rng.pct(25);
      case (USER_MODE)
        1:       f.user = seq_n & ((1 << USER_W) - 1);
        2:       f.user = 0;
        default: f.user = rng.range(0, (1 << USER_W) - 1);
      endcase
      seq_n++;
      beats = (len + BYTES - 1) / BYTES;
      excl  = 1'b0;
      if (OVERSIZE > 0 && beats > OVERSIZE) excl = 1'b1;
      if (DROP_BAD && f.bad) excl = 1'b1;
      if (KEY_MODE == 1 && f.dest >= KEY_N) excl = 1'b1;
      key = (KEY_MODE == 0) ? KEY : (KEY_MODE == 1) ? int'(f.dest) : int'(f.dest % KEY_N);
      if (excl) n_excl++;
      else      sb.put(key, f);
      n_added++;
      todo.push_back(f);
      log_seq.push_back(f.seq);
      log_beats.push_back(beats);
    end
  endtask

  logic                v_n, l_n;
  logic [DATA_W-1:0]   d_n;
  logic [BYTES-1:0]    k_n, s_n;
  logic [ID_W-1:0]     i_n;
  logic [DEST_W-1:0]   t_n;
  logic [USER_W-1:0]   u_n;

  function automatic void step();
    if (!rst_ni) begin
      v_n = 1'b0;
      cur = null;
      todo.delete();
    end else begin
      if (tvalid_o && tready_i) v_n = 1'b0;
      if (!tvalid_o || tready_i) begin
        if (cur == null && todo.size() > 0) begin
          cur = todo.pop_front();
          pos = 0;
        end
        if (cur != null && rng.pct(valid_pct)) begin
          automatic logic [DATA_W-1:0] d  = '0;
          automatic logic [BYTES-1:0]  k  = '0;
          automatic logic [BYTES-1:0]  st = '0;
          automatic int                rem = int'(cur.bytes.size()) - int'(pos);
          automatic int                nb  = (rem > BYTES) ? BYTES : rem;
          for (int l = 0; l < nb; l++) begin
            d[l*8 +: 8] = cur.bytes[pos + l][7:0];
            st[l]       = cur.bytes[pos + l][8];
            k[l]        = 1'b1;
          end
          v_n = 1'b1;
          d_n = d;
          k_n = k;
          s_n = st;
          l_n = (rem <= BYTES);
          i_n = ID_W'(cur.id);
          t_n = DEST_W'(cur.dest);
          u_n = (USER_MODE == 2) ? USER_W'((rem <= BYTES) && cur.bad) : USER_W'(cur.user);
          pos = pos + nb;
          n_beats++;
          if (rem <= BYTES) cur = null;
        end
      end
    end
  endfunction

  always @(posedge clk_i) begin
    step();
    tvalid_o <= v_n;
    tdata_o  <= d_n;
    tkeep_o  <= k_n;
    tstrb_o  <= s_n;
    tlast_o  <= l_n;
    tid_o    <= i_n;
    tdest_o  <= t_n;
    tuser_o  <= u_n;
  end
endmodule : axs_src

// ---------------------------------------------------------------------------
// Monitor / checker
// ---------------------------------------------------------------------------
module axs_snk
  import axs_tb_pkg::*;
#(
  parameter int DATA_W  = 32,
  parameter int ID_W    = 1,
  parameter int DEST_W  = 1,
  parameter int USER_W  = 1,
  parameter int SB      = 0,
  parameter int KEY     = 0,
  parameter bit KEY_BY_ID = 1'b0,
  parameter bit CHK_USER  = 1'b1,
  parameter bit LOSSY     = 1'b0,
  parameter string NAME   = "snk"
) (
  input  logic                clk_i,
  input  logic                rst_ni,
  input  logic                tvalid_i,
  output logic                tready_o,
  input  logic [DATA_W-1:0]   tdata_i,
  input  logic [DATA_W/8-1:0] tkeep_i,
  input  logic [DATA_W/8-1:0] tstrb_i,
  input  logic                tlast_i,
  input  logic [ID_W-1:0]     tid_i,
  input  logic [DEST_W-1:0]   tdest_i,
  input  logic [USER_W-1:0]   tuser_i,
  input  int unsigned         ready_pct
);
  localparam int BYTES = DATA_W / 8;
  localparam int PW    = DATA_W + 2 * BYTES + 1 + ID_W + DEST_W + USER_W;

  axs_rng      rng;
  axs_sb       sb;
  axs_frame    cur;
  int unsigned beats_in_frame;
  int unsigned n_frames, n_1beat, n_partial, n_beats, n_stall, n_stall_hold, max_len;
  int unsigned rx_seq[$];
  int unsigned rx_id[$];

  logic          p_v, p_r;
  logic [PW-1:0] p_pl;
  logic [PW-1:0] pl;
  assign pl = {tdata_i, tkeep_i, tstrb_i, tlast_i, tid_i, tdest_i, tuser_i};

  task automatic init(input int unsigned s);
    rng = new(s);
    sb  = get_sb(SB);
    cur = null;
  endtask

  function automatic void process_beat();
    automatic logic [BYTES-1:0] kp = tkeep_i;
    automatic int nk = 0;
    for (int l = 0; l < BYTES; l++) if (kp[l]) nk++;
    if ((tstrb_i & ~tkeep_i) != '0)
      $fatal(1, "[%s] tstrb set on a lane without tkeep (%b/%b)", NAME, tstrb_i, tkeep_i);
    if (!tlast_i && tkeep_i != {BYTES{1'b1}})
      $fatal(1, "[%s] non-last beat with partial tkeep %b", NAME, tkeep_i);
    if (tlast_i && (tkeep_i == '0 || ((tkeep_i & (tkeep_i + 1'b1)) != '0)))
      $fatal(1, "[%s] illegal tkeep %b on last beat", NAME, tkeep_i);
    if (cur == null) begin
      cur = new();
      cur.id   = int'(tid_i);
      cur.dest = int'(tdest_i);
      cur.user = int'(tuser_i);
      beats_in_frame = 0;
    end else begin
      if (cur.id != int'(tid_i))
        $fatal(1, "[%s] tid changed inside a frame (%0d -> %0d): frame interleaving", NAME, cur.id, tid_i);
      if (cur.dest != int'(tdest_i))
        $fatal(1, "[%s] tdest changed inside a frame", NAME);
      if (CHK_USER && cur.user != int'(tuser_i))
        $fatal(1, "[%s] tuser changed inside a frame", NAME);
    end
    for (int l = 0; l < BYTES; l++)
      if (kp[l]) cur.bytes.push_back({tstrb_i[l], tdata_i[l*8 +: 8]});
    beats_in_frame++;
    n_beats++;
    if (tlast_i) begin
      automatic int key = KEY_BY_ID ? int'(cur.id) : KEY;
      n_frames++;
      if (beats_in_frame == 1) n_1beat++;
      if (tkeep_i != {BYTES{1'b1}}) n_partial++;
      if (cur.bytes.size() > max_len) max_len = cur.bytes.size();
      rx_seq.push_back(cur.user);
      rx_id.push_back(cur.id);
      sb.check(NAME, key, cur, CHK_USER, LOSSY);
      cur = null;
    end
  endfunction

  logic r_n;

  function automatic void step();
    if (!rst_ni) begin
      r_n = 1'b0;
      cur = null;
      p_v = 1'b0;
      p_r = 1'b1;
    end else begin
      if (p_v && !p_r) begin
        if (!tvalid_i) $fatal(1, "[%s] tvalid dropped before tready (protocol violation)", NAME);
        if (pl != p_pl) $fatal(1, "[%s] payload changed while stalled (protocol violation)", NAME);
        n_stall_hold++;
      end
      p_v  = tvalid_i;
      p_r  = tready_o;
      p_pl = pl;
      if (tvalid_i && !tready_o) n_stall++;
      if (tvalid_i && tready_o) process_beat();
      r_n = rng.pct(ready_pct);
    end
  endfunction

  always @(posedge clk_i) begin
    step();
    tready_o <= r_n;
  end
endmodule : axs_snk

// ---------------------------------------------------------------------------
// Harness: single source / single sink DUTs
//   KIND 0 axis_register (P0 = REG_MODE)
//   KIND 1 axis_fifo beat mode (P0 = DEPTH, P1 = OUT_REG)
//   KIND 2 axis_fifo frame mode (P0 = DEPTH, P1 = OUT_REG, P2 = DROP_WHEN_FULL,
//          P3 = DROP_BAD_FRAME, P4 = run the directed stalled-overflow model)
//   KIND 3 axis_async_fifo (P0 = ADDR_W, P1 = read clock half-period)
//   KIND 4 axis_width_converter (S_W -> M_W)
// ---------------------------------------------------------------------------
module axs_h_pt
  import axs_tb_pkg::*;
#(
  parameter int    KIND = 0,
  parameter int    S_W  = 32,
  parameter int    M_W  = 32,
  parameter int    P0   = 0,
  parameter int    P1   = 0,
  parameter int    P2   = 0,
  parameter int    P3   = 0,
  parameter int    P4   = 0,
  parameter int    SB   = 0,
  parameter int    NFR  = 100,
  parameter string NAME = "pt"
) (
  input  int unsigned seed_i,
  output logic        done_o,
  output int unsigned frames_o
);
  localparam int ID_W   = 4;
  localparam int DEST_W = 3;
  localparam int USER_W = 16;
  localparam int SKW    = S_W / 8;
  localparam int MKW    = M_W / 8;
  localparam int DEPTH  = (KIND == 3) ? (1 << P0) : P0;
  localparam int UMODE  = (KIND == 2 && (P2 == 1 || P4 == 1)) ? 1 : ((KIND == 2 && P3 == 1) ? 2 : 0);
  localparam bit LOSSY  = (KIND == 2 && P2 == 1);
  localparam bit CHKU   = (KIND != 4);

  logic clk, mclk, rst_n, tb_rst_n;
  task automatic set_rst(input logic v);
    rst_n    = v;
    tb_rst_n = v;
  endtask
  logic mclk_s;
  initial begin clk = 1'b0; forever #5 clk = ~clk; end
  initial begin mclk = 1'b0; #3; forever #((P1 > 0) ? P1 : 5) mclk = ~mclk; end
  assign mclk_s = (KIND == 3) ? mclk : clk;

  int unsigned vpct, rpct;

  logic                s_v, s_r, s_l;
  logic [S_W-1:0]      s_d;
  logic [SKW-1:0]      s_k, s_s;
  logic [ID_W-1:0]     s_i;
  logic [DEST_W-1:0]   s_t;
  logic [USER_W-1:0]   s_u;
  logic                m_v, m_r, m_l;
  logic [M_W-1:0]      m_d;
  logic [MKW-1:0]      m_k, m_s;
  logic [ID_W-1:0]     m_i;
  logic [DEST_W-1:0]   m_t;
  logic [USER_W-1:0]   m_u;
  logic                dut_drop;
  logic [$clog2(DEPTH + 1) + 1:0] dut_level;

  axs_src #(.DATA_W(S_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .SB(SB), .KEY(0),
            .DEST_MAX(7), .USER_MODE(UMODE), .MAXLEN((KIND == 2) ? (4 * DEPTH * SKW) : 61),
            .OVERSIZE((KIND == 2 && P2 == 0) ? DEPTH : 0), .DROP_BAD(P3 != 0)) u_src (
    .clk_i(clk), .rst_ni(tb_rst_n), .tvalid_o(s_v), .tready_i(s_r), .tdata_o(s_d), .tkeep_o(s_k),
    .tstrb_o(s_s), .tlast_o(s_l), .tid_o(s_i), .tdest_o(s_t), .tuser_o(s_u), .valid_pct(vpct));

  axs_snk #(.DATA_W(M_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .SB(SB), .KEY(0),
            .CHK_USER(CHKU), .LOSSY(LOSSY), .NAME(NAME)) u_snk (
    .clk_i(mclk_s), .rst_ni(tb_rst_n), .tvalid_i(m_v), .tready_o(m_r), .tdata_i(m_d), .tkeep_i(m_k),
    .tstrb_i(m_s), .tlast_i(m_l), .tid_i(m_i), .tdest_i(m_t), .tuser_i(m_u), .ready_pct(rpct));

  if (KIND == 0) begin : g_dut
    axis_register #(.DATA_W(S_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .REG_MODE(P0)) u_dut (
      .clk_i(clk), .rst_ni(rst_n),
      .s_tvalid_i(s_v), .s_tready_o(s_r), .s_tdata_i(s_d), .s_tkeep_i(s_k), .s_tstrb_i(s_s),
      .s_tlast_i(s_l), .s_tid_i(s_i), .s_tdest_i(s_t), .s_tuser_i(s_u),
      .m_tvalid_o(m_v), .m_tready_i(m_r), .m_tdata_o(m_d), .m_tkeep_o(m_k), .m_tstrb_o(m_s),
      .m_tlast_o(m_l), .m_tid_o(m_i), .m_tdest_o(m_t), .m_tuser_o(m_u));
    assign dut_drop = 1'b0;
    assign dut_level = '0;
  end else if (KIND == 1 || KIND == 2) begin : g_dut
    logic [$clog2(DEPTH + 1)-1:0] lvl;
    axis_fifo #(.DATA_W(S_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .DEPTH(DEPTH),
                .FRAME_FIFO(KIND == 2), .DROP_WHEN_FULL(P2 != 0), .DROP_BAD_FRAME(P3 != 0),
                .OUT_REG(P1 != 0)) u_dut (
      .clk_i(clk), .rst_ni(rst_n),
      .s_tvalid_i(s_v), .s_tready_o(s_r), .s_tdata_i(s_d), .s_tkeep_i(s_k), .s_tstrb_i(s_s),
      .s_tlast_i(s_l), .s_tid_i(s_i), .s_tdest_i(s_t), .s_tuser_i(s_u),
      .m_tvalid_o(m_v), .m_tready_i(m_r), .m_tdata_o(m_d), .m_tkeep_o(m_k), .m_tstrb_o(m_s),
      .m_tlast_o(m_l), .m_tid_o(m_i), .m_tdest_o(m_t), .m_tuser_o(m_u),
      .level_o(lvl), .drop_o(dut_drop));
    assign dut_level = {2'b00, lvl};
  end else if (KIND == 3) begin : g_dut
    axis_async_fifo #(.DATA_W(S_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .ADDR_W(P0)) u_dut (
      .s_clk_i(clk), .s_rst_ni(rst_n),
      .s_tvalid_i(s_v), .s_tready_o(s_r), .s_tdata_i(s_d), .s_tkeep_i(s_k), .s_tstrb_i(s_s),
      .s_tlast_i(s_l), .s_tid_i(s_i), .s_tdest_i(s_t), .s_tuser_i(s_u),
      .m_clk_i(mclk), .m_rst_ni(rst_n),
      .m_tvalid_o(m_v), .m_tready_i(m_r), .m_tdata_o(m_d), .m_tkeep_o(m_k), .m_tstrb_o(m_s),
      .m_tlast_o(m_l), .m_tid_o(m_i), .m_tdest_o(m_t), .m_tuser_o(m_u));
    assign dut_drop = 1'b0;
    assign dut_level = '0;
  end else begin : g_dut
    axis_width_converter #(.S_DATA_W(S_W), .M_DATA_W(M_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W)) u_dut (
      .clk_i(clk), .rst_ni(rst_n),
      .s_tvalid_i(s_v), .s_tready_o(s_r), .s_tdata_i(s_d), .s_tkeep_i(s_k), .s_tstrb_i(s_s),
      .s_tlast_i(s_l), .s_tid_i(s_i), .s_tdest_i(s_t), .s_tuser_i(s_u),
      .m_tvalid_o(m_v), .m_tready_i(m_r), .m_tdata_o(m_d), .m_tkeep_o(m_k), .m_tstrb_o(m_s),
      .m_tlast_o(m_l), .m_tid_o(m_i), .m_tdest_o(m_t), .m_tuser_o(m_u));
    assign dut_drop = 1'b0;
    assign dut_level = '0;
  end

  // ---- DUT-level counters / baselines -------------------------------------
  axs_sb       sbh;
  int unsigned n_drop, b_drop, b_excl, b_skip, b_got, b_add;
  always @(posedge clk) if (tb_rst_n && dut_drop) n_drop <= n_drop + 1;

  task automatic rebase();
    b_drop = n_drop;
    b_excl = u_src.n_excl;
    b_skip = sbh.n_skipped;
    b_got  = sbh.n_got;
    b_add  = u_src.n_added;
  endtask

  // wait until everything sent has either come out or been (legally) dropped
  task automatic drain(input string what);
    automatic int unsigned quiet = 0;
    automatic int unsigned guard = 0;
    forever begin
      @(posedge clk);
      guard++;
      if (guard > 400000) $fatal(1, "[%s] drain timeout in phase '%s' (pending=%0d)", NAME, what, sbh.pending());
      if (KIND == 2) begin
        if (u_src.is_idle() && dut_level == '0 && !m_v) quiet++; else quiet = 0;
      end else begin
        if (u_src.is_idle() && sbh.pending() == 0 && !m_v) quiet++; else quiet = 0;
      end
      if (quiet >= 12) break;
    end
    reconcile(what);
  endtask

  task automatic reconcile(input string what);
    automatic int unsigned excl = u_src.n_excl - b_excl;
    automatic int unsigned skip = sbh.n_skipped - b_skip;
    automatic int unsigned left = sbh.pending();
    automatic int unsigned drp  = n_drop - b_drop;
    if (!LOSSY && (left != 0 || skip != 0))
      $fatal(1, "[%s] phase '%s': %0d expected frame(s) never delivered", NAME, what, left);
    if (KIND == 2 && drp != excl + skip + left)
      $fatal(1, "[%s] phase '%s': drop_o count %0d != modelled drops %0d (excluded=%0d skipped=%0d tail=%0d)",
             NAME, what, drp, excl + skip + left, excl, skip, left);
    sbh.flush();
    rebase();
  endtask

  task automatic phase(input string what, input int n, input int lm, input int unsigned v, input int unsigned r);
    vpct = v;
    rpct = r;
    u_src.add_frames(n, lm);
    drain(what);
  endtask

  task automatic reset_midframe();
    vpct = 100;
    rpct = 25;
    u_src.add_frames(10, 3);
    repeat (u_src.rng.range(6, 20)) @(posedge clk);
    #2;
    set_rst(1'b0);
    sbh.flush();
    #90;
    set_rst(1'b1);
    repeat (4) @(posedge clk);
    if (!u_src.is_idle()) $fatal(1, "[%s] source not idle after reset", NAME);
    rebase();
  endtask

  // Directed model of a stalled frame FIFO: frames are accepted greedily while they fit.
  task automatic stalled_overflow_model();
    automatic int unsigned free_beats = DEPTH;
    automatic int unsigned accepted[$];
    automatic int unsigned base = u_src.log_seq.size();
    automatic int unsigned nacc = 0;
    u_snk.rx_seq.delete();
    vpct = 100;
    rpct = 0;
    u_src.add_frames(60, 4);
    repeat (4) @(posedge clk);
    while (!u_src.is_idle()) @(posedge clk);
    repeat (30) @(posedge clk);
    for (int i = base; i < u_src.log_seq.size(); i++) begin
      if (u_src.log_beats[i] <= free_beats) begin
        free_beats -= u_src.log_beats[i];
        accepted.push_back(u_src.log_seq[i]);
        nacc++;
      end
    end
    if (int'(dut_level) != int'(DEPTH - free_beats))
      $fatal(1, "[%s] stalled FIFO level %0d != model %0d", NAME, dut_level, DEPTH - free_beats);
    if (u_snk.n_frames != b_got_frames_snk)
      $fatal(1, "[%s] frames leaked out of a stalled FIFO", NAME);
    rpct = 100;
    drain("stalled-overflow");
    if (u_snk.rx_seq.size() != accepted.size())
      $fatal(1, "[%s] stalled model: %0d frames out, %0d expected", NAME, u_snk.rx_seq.size(), accepted.size());
    for (int i = 0; i < accepted.size(); i++)
      if (u_snk.rx_seq[i] != (accepted[i] & 32'h0000FFFF))
        $fatal(1, "[%s] stalled model: frame %0d is seq %0d, expected %0d", NAME, i, u_snk.rx_seq[i], accepted[i]);
    cov_overflow_frames += (u_src.log_seq.size() - base) - nacc;
  endtask

  int unsigned b_got_frames_snk;
  int unsigned cov_overflow_frames;

  initial begin
    done_o   = 1'b0;
    frames_o = 0;
    set_rst(1'b0);
    vpct = 100;
    rpct = 100;
    #1;
    sbh = get_sb(SB);
    u_src.init(seed_i ^ (32'h1000 + SB * 7));
    u_snk.init(seed_i ^ (32'h2000 + SB * 13));
    repeat (5) @(posedge clk);
    #2 set_rst(1'b1);
    repeat (4) @(posedge clk);
    rebase();

    // directed corner cases
    phase("1-byte frames",        40, 1, 100, 100);
    phase("1-beat partial keep",  40, 2, 100, 100);
    phase("back-to-back full",    30, 3, 100, 100);
    phase("one..five beats",      NFR, 4, 100, 100);
    // randomised with back-pressure on both sides
    phase("mixed 100/100",        NFR, 0, 100, 100);
    phase("mixed 100/30",         NFR, 0, 100, 30);
    phase("mixed 30/100",         NFR, 0, 30, 100);
    phase("mixed 60/60",          NFR, 0, 60, 60);
    phase("long 20/20",           NFR / 2, 5, 20, 20);
    phase("mixed 90/10",          NFR / 2, 0, 90, 10);
    if (KIND == 2 && P4 != 0) begin
      b_got_frames_snk = u_snk.n_frames;
      stalled_overflow_model();
    end
    // reset in the middle of frames, then prove the DUT recovered
    reset_midframe();
    phase("post-reset mixed",     NFR, 0, 70, 70);
    phase("post-reset one-beat",  30, 1, 100, 100);

    frames_o = u_snk.n_frames;
    $display("[INFO] %-26s frames=%0d 1beat=%0d partial_last=%0d beats=%0d stalls=%0d hold_checks=%0d drops=%0d maxlen=%0dB",
             NAME, u_snk.n_frames, u_snk.n_1beat, u_snk.n_partial, u_snk.n_beats, u_snk.n_stall,
             u_snk.n_stall_hold, n_drop, u_snk.max_len);
    if (u_snk.n_frames == 0) $fatal(1, "[%s] no frames were checked", NAME);
    done_o = 1'b1;
  end
endmodule : axs_h_pt

// ---------------------------------------------------------------------------
// Harness: arbitrating mux (N sources -> 1 sink)
// ---------------------------------------------------------------------------
module axs_h_arb
  import axs_tb_pkg::*;
#(
  parameter int    N          = 4,
  parameter int    ARB_TYPE   = 0,
  parameter bit    FRAME_LOCK = 1'b1,
  parameter bit    OUT_REG    = 1'b0,
  parameter int    SB         = 0,
  parameter int    NFR        = 100,
  parameter string NAME       = "arb"
) (
  input  int unsigned seed_i,
  output logic        done_o,
  output int unsigned frames_o
);
  localparam int DATA_W = 32;
  localparam int KW     = 4;
  localparam int ID_W   = 3;
  localparam int DEST_W = 3;
  localparam int USER_W = 4;
  localparam int SELW   = $clog2(N);

  logic clk, rst_n, tb_rst_n;
  task automatic set_rst(input logic v);
    rst_n    = v;
    tb_rst_n = v;
  endtask
  initial begin clk = 1'b0; forever #5 clk = ~clk; end
  int unsigned vpct, rpct;

  logic [N-1:0]                s_v, s_r, s_l;
  logic [N-1:0][DATA_W-1:0]    s_d;
  logic [N-1:0][KW-1:0]        s_k, s_s;
  logic [N-1:0][ID_W-1:0]      s_i;
  logic [N-1:0][DEST_W-1:0]    s_t;
  logic [N-1:0][USER_W-1:0]    s_u;
  logic                        m_v, m_r, m_l;
  logic [DATA_W-1:0]           m_d;
  logic [KW-1:0]               m_k, m_s;
  logic [ID_W-1:0]             m_i;
  logic [DEST_W-1:0]           m_t;
  logic [USER_W-1:0]           m_u;
  logic [SELW-1:0]             sel;

  logic [N-1:0] idle_v;
  event         cmd_ev;
  int           cmd_n, cmd_lm;
  int unsigned  seed_q;
  for (genvar g = 0; g < N; g++) begin : g_src
    axs_src #(.DATA_W(DATA_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .SB(SB), .KEY(g),
              .ID_FIXED(g), .DEST_MAX(7), .USER_MODE(0), .MAXLEN(45)) u_src (
      .clk_i(clk), .rst_ni(tb_rst_n), .tvalid_o(s_v[g]), .tready_i(s_r[g]), .tdata_o(s_d[g]),
      .tkeep_o(s_k[g]), .tstrb_o(s_s[g]), .tlast_o(s_l[g]), .tid_o(s_i[g]), .tdest_o(s_t[g]),
      .tuser_o(s_u[g]), .valid_pct(vpct));
    always @(posedge clk) idle_v[g] <= u_src.is_idle();
    always @(cmd_ev) u_src.add_frames(cmd_n, cmd_lm);
    initial begin #1; u_src.init(seed_i ^ (32'h3000 + SB * 31 + g * 5)); end
  end
  axs_rng rr;

  axs_snk #(.DATA_W(DATA_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .SB(SB),
            .KEY_BY_ID(1'b1), .CHK_USER(1'b1), .NAME(NAME)) u_snk (
    .clk_i(clk), .rst_ni(tb_rst_n), .tvalid_i(m_v), .tready_o(m_r), .tdata_i(m_d), .tkeep_i(m_k),
    .tstrb_i(m_s), .tlast_i(m_l), .tid_i(m_i), .tdest_i(m_t), .tuser_i(m_u), .ready_pct(rpct));

  axis_arb_mux #(.N(N), .DATA_W(DATA_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W),
                 .ARB_TYPE(ARB_TYPE), .FRAME_LOCK(FRAME_LOCK), .OUT_REG(OUT_REG)) u_dut (
    .clk_i(clk), .rst_ni(rst_n),
    .s_tvalid_i(s_v), .s_tready_o(s_r), .s_tdata_i(s_d), .s_tkeep_i(s_k), .s_tstrb_i(s_s),
    .s_tlast_i(s_l), .s_tid_i(s_i), .s_tdest_i(s_t), .s_tuser_i(s_u),
    .m_tvalid_o(m_v), .m_tready_i(m_r), .m_tdata_o(m_d), .m_tkeep_o(m_k), .m_tstrb_o(m_s),
    .m_tlast_o(m_l), .m_tid_o(m_i), .m_tdest_o(m_t), .m_tuser_o(m_u), .sel_o(sel));

  axs_sb sbh;

  function automatic bit all_idle();
    return (idle_v == '1);
  endfunction

  task automatic add_all(input int n, input int lm);
    cmd_n  = n;
    cmd_lm = lm;
    -> cmd_ev;
    #0;
  endtask

  task automatic drain(input string what);
    automatic int unsigned quiet = 0;
    automatic int unsigned guard = 0;
    forever begin
      @(posedge clk);
      guard++;
      if (guard > 400000) $fatal(1, "[%s] drain timeout in phase '%s' (pending=%0d)", NAME, what, sbh.pending());
      if (all_idle() && sbh.pending() == 0 && !m_v) quiet++; else quiet = 0;
      if (quiet >= 8) break;
    end
  endtask

  task automatic phase(input string what, input int n, input int lm, input int unsigned v, input int unsigned r);
    vpct = v;
    rpct = r;
    add_all(n, FRAME_LOCK ? lm : 2);
    drain(what);
  endtask

  // all sources saturated with one-beat frames: grant order is fully determined
  task automatic order_check();
    automatic int total = N * 24;
    u_snk.rx_id.delete();
    vpct = 100;
    rpct = 100;
    add_all(24, 2);
    drain("order");
    if (u_snk.rx_id.size() != total) $fatal(1, "[%s] order phase: %0d frames, expected %0d", NAME, u_snk.rx_id.size(), total);
    if (ARB_TYPE == 0) begin
      for (int i = 0; i + N <= total; i++)
        for (int a = 0; a < N; a++)
          for (int b = a + 1; b < N; b++)
            if (u_snk.rx_id[i + a] == u_snk.rx_id[i + b])
              $fatal(1, "[%s] round-robin violated: window @%0d repeats source %0d", NAME, i, u_snk.rx_id[i + a]);
    end else begin
      for (int i = 1; i < total; i++)
        if (u_snk.rx_id[i] < u_snk.rx_id[i - 1])
          $fatal(1, "[%s] fixed priority violated at frame %0d (%0d after %0d)", NAME, i, u_snk.rx_id[i], u_snk.rx_id[i - 1]);
    end
  endtask

  task automatic reset_midframe();
    vpct = 100;
    rpct = 25;
    add_all(8, FRAME_LOCK ? 3 : 2);
    repeat (rr.range(6, 20)) @(posedge clk);
    #2;
    set_rst(1'b0);
    sbh.flush();
    #90;
    set_rst(1'b1);
    repeat (4) @(posedge clk);
    if (!all_idle()) $fatal(1, "[%s] sources not idle after reset", NAME);
  endtask

  initial begin
    done_o   = 1'b0;
    frames_o = 0;
    set_rst(1'b0);
    vpct = 100;
    rpct = 100;
    #1;
    sbh = get_sb(SB);
    rr = new(seed_i ^ 32'h7000);
    u_snk.init(seed_i ^ (32'h4000 + SB));
    repeat (5) @(posedge clk);
    #2 set_rst(1'b1);
    repeat (4) @(posedge clk);
    phase("1-beat frames",   30, 2, 100, 100);
    phase("mixed 100/100",   NFR, 0, 100, 100);
    phase("mixed 100/30",    NFR, 0, 100, 30);
    phase("mixed 40/100",    NFR, 0, 40, 100);
    phase("mixed 60/60",     NFR, 0, 60, 60);
    phase("long 30/20",      NFR / 2, 5, 30, 20);
    order_check();
    reset_midframe();
    phase("post-reset mixed", NFR, 0, 70, 70);
    frames_o = u_snk.n_frames;
    $display("[INFO] %-26s frames=%0d 1beat=%0d partial_last=%0d beats=%0d stalls=%0d hold_checks=%0d",
             NAME, u_snk.n_frames, u_snk.n_1beat, u_snk.n_partial, u_snk.n_beats, u_snk.n_stall, u_snk.n_stall_hold);
    if (u_snk.n_frames == 0) $fatal(1, "[%s] no frames were checked", NAME);
    done_o = 1'b1;
  end
endmodule : axs_h_arb

// ---------------------------------------------------------------------------
// Harness: demux (1 source -> N sinks), routing by tdest or by external select
// ---------------------------------------------------------------------------
module axs_h_demux
  import axs_tb_pkg::*;
#(
  parameter int    N           = 4,
  parameter bit    ROUTE_TDEST = 1'b1,
  parameter int    SB          = 0,
  parameter int    NFR         = 100,
  parameter string NAME        = "demux"
) (
  input  int unsigned seed_i,
  output logic        done_o,
  output int unsigned frames_o
);
  localparam int DATA_W = 32;
  localparam int KW     = 4;
  localparam int ID_W   = 3;
  localparam int DEST_W = 3;
  localparam int USER_W = 4;
  localparam int SELW   = $clog2(N);

  logic clk, rst_n, tb_rst_n;
  task automatic set_rst(input logic v);
    rst_n    = v;
    tb_rst_n = v;
  endtask
  initial begin clk = 1'b0; forever #5 clk = ~clk; end
  int unsigned vpct, rpct;

  logic                s_v, s_r, s_l;
  logic [DATA_W-1:0]   s_d;
  logic [KW-1:0]       s_k, s_s;
  logic [ID_W-1:0]     s_i;
  logic [DEST_W-1:0]   s_t;
  logic [USER_W-1:0]   s_u;
  logic [SELW-1:0]     s_sel;
  logic [N-1:0]        m_v, m_r;
  logic                m_l;
  logic [DATA_W-1:0]   m_d;
  logic [KW-1:0]       m_k, m_s;
  logic [ID_W-1:0]     m_i;
  logic [DEST_W-1:0]   m_t;
  logic [USER_W-1:0]   m_u;
  logic                dut_drop;
  logic                inframe_q;
  int unsigned         n_drop, b_drop, b_excl;

  // the select input changes after the first beat (and while beat 0 is stalled)
  // to prove the route is locked both under backpressure and across the frame
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)          inframe_q <= 1'b0;
    else if (s_v && s_r) inframe_q <= !s_l;
    else if (s_v)        inframe_q <= 1'b1;
  end
  assign s_sel = inframe_q ? SELW'((int'(s_t) + 1) % N) : SELW'(int'(s_t) % N);

  axs_src #(.DATA_W(DATA_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .SB(SB),
            .KEY_MODE(ROUTE_TDEST ? 1 : 2), .KEY_N(N), .DEST_MAX(7), .USER_MODE(0), .MAXLEN(45)) u_src (
    .clk_i(clk), .rst_ni(tb_rst_n), .tvalid_o(s_v), .tready_i(s_r), .tdata_o(s_d), .tkeep_o(s_k),
    .tstrb_o(s_s), .tlast_o(s_l), .tid_o(s_i), .tdest_o(s_t), .tuser_o(s_u), .valid_pct(vpct));

  int unsigned nfr_v [N];
  for (genvar g = 0; g < N; g++) begin : g_snk
    axs_snk #(.DATA_W(DATA_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .SB(SB), .KEY(g),
              .CHK_USER(1'b1), .NAME(NAME)) u_snk (
      .clk_i(clk), .rst_ni(tb_rst_n), .tvalid_i(m_v[g]), .tready_o(m_r[g]), .tdata_i(m_d), .tkeep_i(m_k),
      .tstrb_i(m_s), .tlast_i(m_l), .tid_i(m_i), .tdest_i(m_t), .tuser_i(m_u), .ready_pct(rpct));
    always @(posedge clk) nfr_v[g] <= u_snk.n_frames;
    initial begin #1; u_snk.init(seed_i ^ (32'h6000 + SB * 17 + g * 3)); end
  end
  axs_rng rr;

  axis_demux #(.N(N), .DATA_W(DATA_W), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W),
               .ROUTE_TDEST(ROUTE_TDEST)) u_dut (
    .clk_i(clk), .rst_ni(rst_n),
    .s_tvalid_i(s_v), .s_tready_o(s_r), .s_tdata_i(s_d), .s_tkeep_i(s_k), .s_tstrb_i(s_s),
    .s_tlast_i(s_l), .s_tid_i(s_i), .s_tdest_i(s_t), .s_tuser_i(s_u), .s_select_i(s_sel),
    .m_tvalid_o(m_v), .m_tready_i(m_r), .m_tdata_o(m_d), .m_tkeep_o(m_k), .m_tstrb_o(m_s),
    .m_tlast_o(m_l), .m_tid_o(m_i), .m_tdest_o(m_t), .m_tuser_o(m_u), .drop_o(dut_drop));

  axs_sb sbh;
  always @(posedge clk) if (tb_rst_n && dut_drop) n_drop <= n_drop + 1;

  task automatic drain(input string what);
    automatic int unsigned quiet = 0;
    automatic int unsigned guard = 0;
    forever begin
      @(posedge clk);
      guard++;
      if (guard > 400000) $fatal(1, "[%s] drain timeout in phase '%s' (pending=%0d)", NAME, what, sbh.pending());
      if (u_src.is_idle() && sbh.pending() == 0 && m_v == '0) quiet++; else quiet = 0;
      if (quiet >= 8) break;
    end
    if (n_drop - b_drop != u_src.n_excl - b_excl)
      $fatal(1, "[%s] phase '%s': drop_o count %0d != modelled invalid-route frames %0d",
             NAME, what, n_drop - b_drop, u_src.n_excl - b_excl);
    b_drop = n_drop;
    b_excl = u_src.n_excl;
  endtask

  task automatic phase(input string what, input int n, input int lm, input int unsigned v, input int unsigned r);
    vpct = v;
    rpct = r;
    u_src.add_frames(n, lm);
    drain(what);
  endtask

  function automatic int unsigned total_frames();
    automatic int unsigned t = 0;
    for (int i = 0; i < N; i++) t += nfr_v[i];
    return t;
  endfunction

  initial begin
    done_o   = 1'b0;
    frames_o = 0;
    set_rst(1'b0);
    vpct = 100;
    rpct = 100;
    #1;
    sbh = get_sb(SB);
    u_src.init(seed_i ^ (32'h5000 + SB * 3));
    rr = new(seed_i ^ 32'h8000);
    repeat (5) @(posedge clk);
    #2 set_rst(1'b1);
    repeat (4) @(posedge clk);
    phase("1-beat frames", 60, 2, 100, 100);
    phase("mixed 100/100", NFR, 0, 100, 100);
    phase("mixed 100/30",  NFR, 0, 100, 30);
    phase("mixed 40/100",  NFR, 0, 40, 100);
    phase("mixed 60/60",   NFR, 0, 60, 60);
    phase("long 30/20",    NFR / 2, 5, 30, 20);
    // reset mid-frame
    vpct = 100;
    rpct = 25;
    u_src.add_frames(10, 3);
    repeat (rr.range(6, 20)) @(posedge clk);
    #2 set_rst(1'b0);
    sbh.flush();
    #90 set_rst(1'b1);
    repeat (4) @(posedge clk);
    if (!u_src.is_idle()) $fatal(1, "[%s] source not idle after reset", NAME);
    b_drop = n_drop;
    b_excl = u_src.n_excl;
    phase("post-reset mixed", NFR, 0, 70, 70);
    frames_o = total_frames();
    $display("[INFO] %-26s frames=%0d dropped_invalid=%0d", NAME, total_frames(), n_drop);
    if (total_frames() == 0) $fatal(1, "[%s] no frames were checked", NAME);
    done_o = 1'b1;
  end
endmodule : axs_h_demux

// ---------------------------------------------------------------------------
// AXI4 Burst Slave Memory with strict AMBA AXI4 protocol & 4KB checks
// ---------------------------------------------------------------------------
module axs_axi4_slave_mem
  import bus_pkg::*;
  import axs_tb_pkg::*;
#(
  parameter int    MEM_BYTES = 16384,
  parameter string NAME      = "axi_mem"
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  axi4_m2s_t   axi_i,
  output axi4_s2m_t   axi_o,
  input  int unsigned ready_pct,
  input  int unsigned resp_pct,
  input  logic        inj_rerr_i,
  input  logic        inj_berr_i
);
  logic [7:0] mem [0:MEM_BYTES-1];

  axs_rng      rng;
  int unsigned n_ar_bursts, n_aw_bursts, n_4kb_rd_splits, n_4kb_wr_splits;
  int unsigned n_ar_stalls, n_aw_stalls, n_w_stalls;

  // Read channel state
  bit          rd_act;
  logic [3:0]  rd_id;
  logic [31:0] rd_addr;
  int unsigned rd_rem;

  // Write channel state (supports independent AW and W ordering)
  bit          aw_have;
  logic [3:0]  aw_id;
  logic [31:0] aw_addr;
  int unsigned aw_len_beats;
  logic [31:0] wq_data[$];
  logic [3:0]  wq_strb[$];
  bit          wq_last[$];
  bit          b_pend;
  logic [3:0]  b_id;
  logic [1:0]  b_resp;

  // Protocol hold-stable trackers
  bit          p_ar_v, p_ar_r;
  logic [55:0] p_ar_pl;
  bit          p_aw_v, p_aw_r;
  logic [55:0] p_aw_pl;
  bit          p_w_v, p_w_r;
  logic [36:0] p_w_pl;

  axi4_s2m_t   out_n;

  task automatic init(input int unsigned s);
    rng = new(s);
  endtask

  function automatic void check_burst_legal(
    input string       ch,
    input logic [31:0] addr,
    input logic [7:0]  len,
    input logic [2:0]  size,
    input logic [1:0]  burst,
    input bit          is_rd
  );
    automatic int unsigned beats  = int'(len) + 1;
    automatic int unsigned bytes  = beats << int'(size);
    automatic int unsigned offset = int'(addr & 32'h0000_0FFF);
    if (size != 3'd2)
      $fatal(1, "[%s] %s illegal size=%0d (expected 2)", NAME, ch, size);
    if (burst != AXI_BURST_INCR)
      $fatal(1, "[%s] %s illegal burst=%0d (expected INCR)", NAME, ch, burst);
    if (addr[1:0] != 2'b00)
      $fatal(1, "[%s] %s unaligned addr=0x%08h", NAME, ch, addr);
    if (offset + bytes > 4096)
      $fatal(1, "[%s] %s 4KB BOUNDARY VIOLATION: addr=0x%08h len=%0d (offset=%0d + bytes=%0d = %0d > 4096)",
             NAME, ch, addr, len, offset, bytes, offset + bytes);
    if (int'(addr) + bytes > MEM_BYTES)
      $fatal(1, "[%s] %s out-of-range addr=0x%08h bytes=%0d", NAME, ch, addr, bytes);
    if (offset + bytes == 4096) begin
      if (is_rd) n_4kb_rd_splits++;
      else       n_4kb_wr_splits++;
    end
  endfunction

  function automatic void commit_write_burst();
    automatic logic [31:0] a = aw_addr;
    for (int unsigned b = 0; b < aw_len_beats; b++) begin
      automatic logic [31:0] wd = wq_data.pop_front();
      automatic logic [3:0]  ws = wq_strb.pop_front();
      automatic bit          wl = wq_last.pop_front();
      automatic bit          exp_last = (b == aw_len_beats - 1);
      if (wl != exp_last)
        $fatal(1, "[%s] WLAST mismatch at beat %0d/%0d (got %0b exp %0b)", NAME, b, aw_len_beats, wl, exp_last);
      for (int lane = 0; lane < 4; lane++) begin
        if (ws[lane]) mem[int'(a) + lane] = wd[lane*8 +: 8];
      end
      a = a + 32'd4;
    end
    aw_have = 1'b0;
    b_pend  = 1'b1;
    b_id    = aw_id;
    b_resp  = inj_berr_i ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
  endfunction

  function automatic void step();
    automatic logic [55:0] ar_pl = {axi_i.arid, axi_i.araddr, axi_i.arlen, axi_i.arsize, axi_i.arburst, axi_i.arprot, axi_i.arqos};
    automatic logic [55:0] aw_pl = {axi_i.awid, axi_i.awaddr, axi_i.awlen, axi_i.awsize, axi_i.awburst, axi_i.awprot, axi_i.awqos};
    automatic logic [36:0] w_pl  = {axi_i.wdata, axi_i.wstrb, axi_i.wlast};

    if (!rst_ni) begin
      out_n   = '0;
      rd_act  = 1'b0;
      aw_have = 1'b0;
      b_pend  = 1'b0;
      wq_data.delete();
      wq_strb.delete();
      wq_last.delete();
      p_ar_v  = 1'b0; p_ar_r = 1'b1;
      p_aw_v  = 1'b0; p_aw_r = 1'b1;
      p_w_v   = 1'b0; p_w_r  = 1'b1;
    end else begin
      // Hold-stable checks on master outputs
      if (p_ar_v && !p_ar_r) begin
        if (!axi_i.arvalid || ar_pl != p_ar_pl)
          $fatal(1, "[%s] AR channel changed while stalled", NAME);
        n_ar_stalls++;
      end
      if (p_aw_v && !p_aw_r) begin
        if (!axi_i.awvalid || aw_pl != p_aw_pl)
          $fatal(1, "[%s] AW channel changed while stalled", NAME);
        n_aw_stalls++;
      end
      if (p_w_v && !p_w_r) begin
        if (!axi_i.wvalid || w_pl != p_w_pl)
          $fatal(1, "[%s] W channel changed while stalled", NAME);
        n_w_stalls++;
      end

      p_ar_v = axi_i.arvalid; p_ar_r = axi_o.arready; p_ar_pl = ar_pl;
      p_aw_v = axi_i.awvalid; p_aw_r = axi_o.awready; p_aw_pl = aw_pl;
      p_w_v  = axi_i.wvalid;  p_w_r  = axi_o.wready;  p_w_pl  = w_pl;

      // Read channel handshakes
      if (axi_i.arvalid && axi_o.arready) begin
        check_burst_legal("AR", axi_i.araddr, axi_i.arlen, axi_i.arsize, axi_i.arburst, 1'b1);
        rd_act  = 1'b1;
        rd_id   = axi_i.arid;
        rd_addr = axi_i.araddr;
        rd_rem  = int'(axi_i.arlen) + 1;
        n_ar_bursts++;
      end
      if (axi_o.rvalid && axi_i.rready) begin
        out_n.rvalid = 1'b0;
        rd_addr      = rd_addr + 32'd4;
        rd_rem       = rd_rem - 1;
        if (rd_rem == 0) rd_act = 1'b0;
      end

      // Write channel handshakes
      if (axi_i.awvalid && axi_o.awready) begin
        check_burst_legal("AW", axi_i.awaddr, axi_i.awlen, axi_i.awsize, axi_i.awburst, 1'b0);
        aw_have      = 1'b1;
        aw_id        = axi_i.awid;
        aw_addr      = axi_i.awaddr;
        aw_len_beats = int'(axi_i.awlen) + 1;
        n_aw_bursts++;
      end
      if (axi_i.wvalid && axi_o.wready) begin
        wq_data.push_back(axi_i.wdata);
        wq_strb.push_back(axi_i.wstrb);
        wq_last.push_back(axi_i.wlast);
      end
      if (axi_o.bvalid && axi_i.bready) begin
        out_n.bvalid = 1'b0;
      end

      if (aw_have && !b_pend && !out_n.bvalid && (wq_data.size() >= aw_len_beats)) begin
        commit_write_burst();
      end

      // Drive next R beat (hold stable if already valid)
      if (!out_n.rvalid && rd_act && rng.pct(resp_pct)) begin
        out_n.rvalid = 1'b1;
        out_n.rid    = rd_id;
        out_n.rdata  = {mem[int'(rd_addr) + 3], mem[int'(rd_addr) + 2],
                        mem[int'(rd_addr) + 1], mem[int'(rd_addr) + 0]};
        out_n.rresp  = inj_rerr_i ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
        out_n.rlast  = (rd_rem == 1);
      end

      // Drive next B response (hold stable if already valid)
      if (!out_n.bvalid && b_pend && rng.pct(resp_pct)) begin
        out_n.bvalid = 1'b1;
        out_n.bid    = b_id;
        out_n.bresp  = b_resp;
        b_pend       = 1'b0;
      end

      // Randomize ready signals
      out_n.arready = (!rd_act) && rng.pct(ready_pct);
      out_n.awready = (!aw_have && !b_pend && !out_n.bvalid) && rng.pct(ready_pct);
      out_n.wready  = (wq_data.size() < 260) && rng.pct(ready_pct);
    end
  endfunction

  always @(posedge clk_i) begin
    step();
    axi_o <= out_n;
  end
endmodule : axs_axi4_slave_mem

// ---------------------------------------------------------------------------
// Harness: axi4_to_axis (MM2S) + axis_to_axi4 (S2MM) + loopback DMA copy
// ---------------------------------------------------------------------------
module axs_h_axi4_dma
  import bus_pkg::*;
  import axs_tb_pkg::*;
#(
  parameter int    MAX_BURST = 16,
  parameter int    FIFO_DEP  = 32,
  parameter int    SB        = 0,
  parameter string NAME      = "axi4_dma"
) (
  input  int unsigned seed_i,
  output logic        done_o,
  output int unsigned frames_o
);
  localparam int MEM_BYTES = 16384;
  localparam int ID_W      = 4;
  localparam int DEST_W    = 4;
  localparam int USER_W    = 4;

  logic clk, rst_n, tb_rst_n;
  task automatic set_rst(input logic v);
    rst_n    = v;
    tb_rst_n = v;
  endtask
  initial begin clk = 1'b0; forever #5 clk = ~clk; end

  int unsigned r_ready_pct, r_resp_pct, w_ready_pct, w_resp_pct, snk_rpct;
  logic        inj_rerr, inj_berr;
  logic        loopback_en;

  // MM2S command/status
  logic              rd_cmd_v, rd_cmd_r, rd_cmd_last;
  logic [31:0]       rd_cmd_addr, rd_cmd_len;
  logic [ID_W-1:0]   rd_cmd_id;
  logic [DEST_W-1:0] rd_cmd_dest;
  logic [USER_W-1:0] rd_cmd_user;
  logic              rd_sts_v, rd_sts_err, rd_busy;

  // S2MM command/status
  logic              wr_cmd_v, wr_cmd_r, wr_cmd_stop;
  logic [31:0]       wr_cmd_addr, wr_cmd_maxlen;
  logic              wr_sts_v, wr_sts_err, wr_sts_tlast, wr_busy;
  logic [31:0]       wr_sts_bytes;
  logic [ID_W-1:0]   wr_sts_tid;
  logic [DEST_W-1:0] wr_sts_tdest;
  logic [USER_W-1:0] wr_sts_tuser;

  // AXI4 buses
  axi4_m2s_t rd_axi_m2s, wr_axi_m2s;
  axi4_s2m_t rd_axi_s2m, wr_axi_s2m;

  // MM2S stream output
  logic              mm2s_v, mm2s_r, mm2s_l;
  logic [31:0]       mm2s_d;
  logic [3:0]        mm2s_k, mm2s_s;
  logic [ID_W-1:0]   mm2s_i;
  logic [DEST_W-1:0] mm2s_t;
  logic [USER_W-1:0] mm2s_u;

  // Standalone S2MM stream driver signals
  logic              drv_v, drv_l;
  logic [31:0]       drv_d;
  logic [3:0]        drv_k, drv_s;
  logic [ID_W-1:0]   drv_i;
  logic [DEST_W-1:0] drv_t;
  logic [USER_W-1:0] drv_u;

  // Muxed S2MM stream input (standalone driver vs loopback from MM2S)
  logic              s2mm_v, s2mm_r, s2mm_l;
  logic [31:0]       s2mm_d;
  logic [3:0]        s2mm_k, s2mm_s;
  logic [ID_W-1:0]   s2mm_i;
  logic [DEST_W-1:0] s2mm_t;
  logic [USER_W-1:0] s2mm_u;

  // Sink ready (active when not in loopback)
  logic snk_r;
  assign mm2s_r = loopback_en ? s2mm_r : snk_r;

  assign s2mm_v = loopback_en ? mm2s_v : drv_v;
  assign s2mm_d = loopback_en ? mm2s_d : drv_d;
  assign s2mm_k = loopback_en ? mm2s_k : drv_k;
  assign s2mm_s = loopback_en ? mm2s_s : drv_s;
  assign s2mm_l = loopback_en ? mm2s_l : drv_l;
  assign s2mm_i = loopback_en ? mm2s_i : drv_i;
  assign s2mm_t = loopback_en ? mm2s_t : drv_t;
  assign s2mm_u = loopback_en ? mm2s_u : drv_u;

  axs_axi4_slave_mem #(.MEM_BYTES(MEM_BYTES), .NAME({NAME, "_rmem"})) u_rmem (
    .clk_i(clk), .rst_ni(tb_rst_n), .axi_i(rd_axi_m2s), .axi_o(rd_axi_s2m),
    .ready_pct(r_ready_pct), .resp_pct(r_resp_pct), .inj_rerr_i(inj_rerr), .inj_berr_i(1'b0)
  );

  axs_axi4_slave_mem #(.MEM_BYTES(MEM_BYTES), .NAME({NAME, "_wmem"})) u_wmem (
    .clk_i(clk), .rst_ni(tb_rst_n), .axi_i(wr_axi_m2s), .axi_o(wr_axi_s2m),
    .ready_pct(w_ready_pct), .resp_pct(w_resp_pct), .inj_rerr_i(1'b0), .inj_berr_i(inj_berr)
  );

  axi4_to_axis #(
    .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .MAX_BURST_LEN(MAX_BURST),
    .AXI_ID(4'h5), .AXI_PROT(3'b001), .AXI_QOS(4'h3)
  ) u_mm2s (
    .clk_i(clk), .rst_ni(rst_n),
    .cmd_valid_i(rd_cmd_v), .cmd_ready_o(rd_cmd_r), .cmd_addr_i(rd_cmd_addr),
    .cmd_len_i(rd_cmd_len), .cmd_id_i(rd_cmd_id), .cmd_dest_i(rd_cmd_dest),
    .cmd_user_i(rd_cmd_user), .cmd_last_i(rd_cmd_last),
    .sts_valid_o(rd_sts_v), .sts_err_o(rd_sts_err), .busy_o(rd_busy),
    .axi_o(rd_axi_m2s), .axi_i(rd_axi_s2m),
    .m_tvalid_o(mm2s_v), .m_tready_i(mm2s_r), .m_tdata_o(mm2s_d), .m_tkeep_o(mm2s_k),
    .m_tstrb_o(mm2s_s), .m_tlast_o(mm2s_l), .m_tid_o(mm2s_i), .m_tdest_o(mm2s_t),
    .m_tuser_o(mm2s_u)
  );

  axis_to_axi4 #(
    .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .MAX_BURST_LEN(MAX_BURST),
    .FIFO_DEPTH(FIFO_DEP), .AXI_ID(4'hA), .AXI_PROT(3'b010), .AXI_QOS(4'h7)
  ) u_s2mm (
    .clk_i(clk), .rst_ni(rst_n),
    .cmd_valid_i(wr_cmd_v), .cmd_ready_o(wr_cmd_r), .cmd_addr_i(wr_cmd_addr),
    .cmd_max_len_i(wr_cmd_maxlen), .cmd_stop_on_last_i(wr_cmd_stop),
    .sts_valid_o(wr_sts_v), .sts_bytes_o(wr_sts_bytes), .sts_err_o(wr_sts_err),
    .sts_tlast_o(wr_sts_tlast), .sts_tid_o(wr_sts_tid), .sts_tdest_o(wr_sts_tdest),
    .sts_tuser_o(wr_sts_tuser), .busy_o(wr_busy),
    .s_tvalid_i(s2mm_v), .s_tready_o(s2mm_r), .s_tdata_i(s2mm_d), .s_tkeep_i(s2mm_k),
    .s_tstrb_i(s2mm_s), .s_tlast_i(s2mm_l), .s_tid_i(s2mm_i), .s_tdest_i(s2mm_t),
    .s_tuser_i(s2mm_u),
    .axi_o(wr_axi_m2s), .axi_i(wr_axi_s2m)
  );

  axs_snk #(
    .DATA_W(32), .ID_W(ID_W), .DEST_W(DEST_W), .USER_W(USER_W), .SB(SB), .KEY(0),
    .CHK_USER(1'b1), .LOSSY(1'b0), .NAME({NAME, "_mm2s"})
  ) u_snk (
    .clk_i(clk), .rst_ni(tb_rst_n),
    .tvalid_i(loopback_en ? 1'b0 : mm2s_v), .tready_o(snk_r),
    .tdata_i(mm2s_d), .tkeep_i(mm2s_k), .tstrb_i(mm2s_s), .tlast_i(mm2s_l),
    .tid_i(mm2s_i), .tdest_i(mm2s_t), .tuser_i(mm2s_u), .ready_pct(snk_rpct)
  );

  // Latched status pulse monitors
  logic        rd_sts_seen_q, rd_sts_err_q;
  logic        wr_sts_seen_q, wr_sts_err_q, wr_sts_tlast_q;
  logic [31:0] wr_sts_bytes_q;
  logic [ID_W-1:0]   wr_sts_tid_q;
  logic [DEST_W-1:0] wr_sts_tdest_q;
  logic [USER_W-1:0] wr_sts_tuser_q;
  logic        clr_rd_sts, clr_wr_sts;

  always @(posedge clk) begin
    if (!tb_rst_n || clr_rd_sts) begin
      rd_sts_seen_q <= 1'b0;
      rd_sts_err_q  <= 1'b0;
    end else if (rd_sts_v) begin
      rd_sts_seen_q <= 1'b1;
      rd_sts_err_q  <= rd_sts_err;
    end
  end

  always @(posedge clk) begin
    if (!tb_rst_n || clr_wr_sts) begin
      wr_sts_seen_q  <= 1'b0;
      wr_sts_err_q   <= 1'b0;
      wr_sts_tlast_q <= 1'b0;
      wr_sts_bytes_q <= 32'd0;
      wr_sts_tid_q   <= '0;
      wr_sts_tdest_q <= '0;
      wr_sts_tuser_q <= '0;
    end else if (wr_sts_v) begin
      wr_sts_seen_q  <= 1'b1;
      wr_sts_err_q   <= wr_sts_err;
      wr_sts_tlast_q <= wr_sts_tlast;
      wr_sts_bytes_q <= wr_sts_bytes;
      wr_sts_tid_q   <= wr_sts_tid;
      wr_sts_tdest_q <= wr_sts_tdest;
      wr_sts_tuser_q <= wr_sts_tuser;
    end
  end

  axs_rng      rng;
  axs_sb       sbh;
  logic [7:0]  gold_wmem [0:MEM_BYTES-1];
  int unsigned n_mm2s_ops, n_s2mm_ops, n_loop_ops;

  task automatic fill_memories();
    for (int i = 0; i < MEM_BYTES; i++) begin
      u_rmem.mem[i] = 8'(rng.next());
      gold_wmem[i]  = 8'(rng.next());
      u_wmem.mem[i] = gold_wmem[i];
    end
  endtask

  task automatic check_wmem_exact(input string ctx);
    for (int i = 0; i < MEM_BYTES; i++) begin
      if (u_wmem.mem[i] !== gold_wmem[i])
        $fatal(1, "[%s] %s wmem mismatch at byte 0x%04h: got=0x%02h exp=0x%02h",
               NAME, ctx, i, u_wmem.mem[i], gold_wmem[i]);
    end
  endtask

  task automatic issue_rd_cmd(
    input logic [31:0]       addr,
    input logic [31:0]       len,
    input logic [ID_W-1:0]   id,
    input logic [DEST_W-1:0] dest,
    input logic [USER_W-1:0] user,
    input logic              last
  );
    @(posedge clk);
    clr_rd_sts <= 1'b0;
    rd_cmd_v    <= 1'b1;
    rd_cmd_addr <= addr;
    rd_cmd_len  <= len;
    rd_cmd_id   <= id;
    rd_cmd_dest <= dest;
    rd_cmd_user <= user;
    rd_cmd_last <= last;
    do @(posedge clk); while (!rd_cmd_r);
    rd_cmd_v <= 1'b0;
  endtask

  task automatic wait_rd_done(input bit exp_err, input string ctx);
    automatic int unsigned guard = 0;
    while (!rd_sts_seen_q) begin
      @(posedge clk);
      guard++;
      if (guard > 200000) $fatal(1, "[%s] %s MM2S timeout waiting for sts_valid", NAME, ctx);
    end
    if (rd_sts_err_q !== exp_err)
      $fatal(1, "[%s] %s MM2S sts_err=%0b exp=%0b", NAME, ctx, rd_sts_err_q, exp_err);
    clr_rd_sts <= 1'b1;
    @(posedge clk);
    clr_rd_sts <= 1'b0;
    while (rd_busy || sbh.pending() > 0) begin
      @(posedge clk);
      guard++;
      if (guard > 250000) $fatal(1, "[%s] %s MM2S timeout draining stream", NAME, ctx);
    end
  endtask

  task automatic run_mm2s_op(
    input logic [31:0] addr,
    input logic [31:0] len,
    input bit          err_en,
    input string       ctx
  );
    automatic logic [ID_W-1:0]   id   = ID_W'(rng.next());
    automatic logic [DEST_W-1:0] dest = DEST_W'(rng.next());
    automatic logic [USER_W-1:0] user = USER_W'(rng.next());
    automatic logic [31:0]       a    = {addr[31:2], 2'b00};
    inj_rerr = err_en;
    if (len > 0) begin
      automatic axs_frame f = new();
      f.id   = int'(id);
      f.dest = int'(dest);
      f.user = int'(user);
      f.seq  = n_mm2s_ops;
      for (int unsigned b = 0; b < len; b++)
        f.bytes.push_back({1'b1, u_rmem.mem[int'(a) + b]});
      sbh.put(0, f);
    end
    issue_rd_cmd(addr, len, id, dest, user, 1'b1);
    wait_rd_done(err_en && (len > 0), ctx);
    inj_rerr = 1'b0;
    n_mm2s_ops++;
  endtask

  // Push a beat vector onto standalone S2MM driver and update golden memory
  task automatic drive_s2mm_stream(
    input logic [31:0]       dst_addr,
    input logic [31:0]       max_len,
    input bit                stop_on_last,
    input int unsigned       n_beats,
    input logic [31:0]       b_data[],
    input logic [3:0]        b_keep[],
    input logic [3:0]        b_strb[],
    input logic              b_last[],
    input logic [ID_W-1:0]   tid,
    input logic [DEST_W-1:0] tdest,
    input logic [USER_W-1:0] tuser,
    input int unsigned       vpct,
    input bit                err_en,
    input string             ctx
  );
    automatic int unsigned max_beats = (max_len == 0) ? 0 : ((int'(max_len) - 1) / 4) + 1;
    automatic logic [3:0]  lmask     = (max_len[1:0] == 2'd0) ? 4'b1111 :
                                       (max_len[1:0] == 2'd1) ? 4'b0001 :
                                       (max_len[1:0] == 2'd2) ? 4'b0011 : 4'b0111;
    automatic int unsigned exp_bytes = 0;
    automatic bit          exp_tlast = 1'b0;
    automatic int unsigned accepted  = 0;
    automatic int unsigned guard     = 0;
    automatic logic [31:0] a         = {dst_addr[31:2], 2'b00};

    inj_berr = err_en;
    // Issue S2MM command
    @(posedge clk);
    clr_wr_sts    <= 1'b0;
    wr_cmd_v      <= 1'b1;
    wr_cmd_addr   <= dst_addr;
    wr_cmd_maxlen <= max_len;
    wr_cmd_stop   <= stop_on_last;
    do @(posedge clk); while (!wr_cmd_r);
    wr_cmd_v <= 1'b0;

    // Drive beats until command stops accepting (or all beats pushed)
    if (max_len > 0) begin
      automatic int unsigned idx = 0;
      automatic bit          done_drv = 1'b0;
      while (!done_drv) begin
        if (rng.pct(vpct)) begin
          drv_v <= 1'b1;
          drv_d <= b_data[idx];
          drv_k <= b_keep[idx];
          drv_s <= b_strb[idx];
          drv_l <= b_last[idx];
          drv_i <= tid;
          drv_t <= tdest;
          drv_u <= tuser;
          do begin
            @(posedge clk);
            guard++;
            if (guard > 200000) $fatal(1, "[%s] %s S2MM stream timeout at beat %0d", NAME, ctx, idx);
          end while (!s2mm_r);
          // Model accepted beat
          begin
            automatic logic [3:0] ek = ((accepted + 1) == max_beats) ? (b_keep[idx] & lmask) : b_keep[idx];
            automatic logic [3:0] es = ek & b_strb[idx];
            for (int lane = 0; lane < 4; lane++) begin
              if (ek[lane]) exp_bytes++;
              if (es[lane]) gold_wmem[int'(a) + accepted * 4 + lane] = b_data[idx][lane*8 +: 8];
            end
          end
          accepted++;
          if ((stop_on_last && b_last[idx]) || (accepted == max_beats)) begin
            exp_tlast = b_last[idx];
            done_drv  = 1'b1;
          end
          idx++;
          if (!done_drv && idx >= n_beats)
            $fatal(1, "[%s] %s TB ran out of stream beats", NAME, ctx);
        end else begin
          drv_v <= 1'b0;
          @(posedge clk);
        end
      end
      drv_v <= 1'b0;
    end

    // Wait for S2MM completion status
    guard = 0;
    while (!wr_sts_seen_q) begin
      @(posedge clk);
      guard++;
      if (guard > 200000) $fatal(1, "[%s] %s S2MM timeout waiting for sts_valid", NAME, ctx);
    end
    if (wr_sts_bytes_q !== exp_bytes)
      $fatal(1, "[%s] %s S2MM sts_bytes=%0d exp=%0d", NAME, ctx, wr_sts_bytes_q, exp_bytes);
    if (wr_sts_tlast_q !== exp_tlast)
      $fatal(1, "[%s] %s S2MM sts_tlast=%0b exp=%0b", NAME, ctx, wr_sts_tlast_q, exp_tlast);
    if (wr_sts_err_q !== (err_en && (accepted > 0)))
      $fatal(1, "[%s] %s S2MM sts_err=%0b exp=%0b", NAME, ctx, wr_sts_err_q, (err_en && (accepted > 0)));
    if (accepted > 0) begin
      if (wr_sts_tid_q !== tid || wr_sts_tdest_q !== tdest || wr_sts_tuser_q !== tuser)
        $fatal(1, "[%s] %s S2MM metadata mismatch", NAME, ctx);
    end
    clr_wr_sts <= 1'b1;
    @(posedge clk);
    clr_wr_sts <= 1'b0;
    inj_berr = 1'b0;
    check_wmem_exact(ctx);
    n_s2mm_ops++;
  endtask

  task automatic run_random_s2mm(
    input logic [31:0] dst_addr,
    input int unsigned frame_bytes,
    input logic [31:0] max_len,
    input int unsigned vpct,
    input bit          allow_pos_bytes,
    input bit          err_en,
    input string       ctx
  );
    automatic int unsigned nb = (frame_bytes == 0) ? 0 : ((frame_bytes - 1) / 4) + 1;
    automatic logic [31:0] bd[] = new[nb];
    automatic logic [3:0]  bk[] = new[nb];
    automatic logic [3:0]  bs[] = new[nb];
    automatic logic        bl[] = new[nb];
    for (int unsigned i = 0; i < nb; i++) begin
      automatic int rem = int'(frame_bytes) - int'(i * 4);
      automatic int nl  = (rem >= 4) ? 4 : rem;
      bd[i] = rng.next();
      bk[i] = (nl == 4) ? 4'b1111 : (nl == 3) ? 4'b0111 : (nl == 2) ? 4'b0011 : 4'b0001;
      bs[i] = bk[i];
      if (allow_pos_bytes) begin
        for (int lane = 0; lane < nl; lane++)
          if (!rng.pct(75)) bs[i][lane] = 1'b0;
      end
      bl[i] = (i == nb - 1);
    end
    drive_s2mm_stream(dst_addr, max_len, 1'b1, nb, bd, bk, bs, bl,
                      ID_W'(rng.next()), DEST_W'(rng.next()), USER_W'(rng.next()),
                      vpct, err_en, ctx);
  endtask

  task automatic run_loopback_copy(
    input logic [31:0] src_addr,
    input logic [31:0] dst_addr,
    input logic [31:0] len,
    input string       ctx
  );
    automatic logic [31:0] sa = {src_addr[31:2], 2'b00};
    automatic logic [31:0] da = {dst_addr[31:2], 2'b00};
    automatic int unsigned guard = 0;
    loopback_en = 1'b1;
    @(posedge clk);
    // Arm S2MM first, then launch MM2S
    wr_cmd_v      <= 1'b1;
    wr_cmd_addr   <= da;
    wr_cmd_maxlen <= len + 32'd64;
    wr_cmd_stop   <= 1'b1;
    do @(posedge clk); while (!wr_cmd_r);
    wr_cmd_v <= 1'b0;

    issue_rd_cmd(sa, len, 4'h3, 4'h9, 4'h5, 1'b1);

    // Wait for both completions
    while (!rd_sts_seen_q || !wr_sts_seen_q) begin
      @(posedge clk);
      guard++;
      if (guard > 200000) $fatal(1, "[%s] %s loopback timeout", NAME, ctx);
    end
    if (rd_sts_err_q || wr_sts_err_q)
      $fatal(1, "[%s] %s unexpected error in loopback", NAME, ctx);
    if (wr_sts_bytes_q !== len || !wr_sts_tlast_q)
      $fatal(1, "[%s] %s loopback bytes=%0d (exp %0d) tlast=%0b", NAME, ctx, wr_sts_bytes_q, len, wr_sts_tlast_q);

    for (int unsigned b = 0; b < len; b++)
      gold_wmem[int'(da) + b] = u_rmem.mem[int'(sa) + b];
    check_wmem_exact(ctx);

    clr_rd_sts <= 1'b1;
    clr_wr_sts <= 1'b1;
    @(posedge clk);
    clr_rd_sts <= 1'b0;
    clr_wr_sts <= 1'b0;
    loopback_en = 1'b0;
    n_loop_ops++;
  endtask

  initial begin
    done_o      = 1'b0;
    frames_o    = 0;
    set_rst(1'b0);
    rd_cmd_v    = 1'b0; rd_cmd_addr = 32'd0; rd_cmd_len = 32'd0;
    rd_cmd_id   = '0;   rd_cmd_dest = '0;    rd_cmd_user = '0; rd_cmd_last = 1'b1;
    wr_cmd_v    = 1'b0; wr_cmd_addr = 32'd0; wr_cmd_maxlen = 32'd0; wr_cmd_stop = 1'b1;
    drv_v       = 1'b0; drv_d = 32'd0; drv_k = 4'hF; drv_s = 4'hF; drv_l = 1'b0;
    drv_i       = '0;   drv_t = '0;    drv_u = '0;
    clr_rd_sts  = 1'b0; clr_wr_sts = 1'b0;
    inj_rerr    = 1'b0; inj_berr = 1'b0;
    loopback_en = 1'b0;
    r_ready_pct = 100; r_resp_pct = 100;
    w_ready_pct = 100; w_resp_pct = 100;
    snk_rpct    = 100;
    #1;
    rng = new(seed_i ^ (32'hA000 + SB * 19));
    sbh = get_sb(SB);
    u_rmem.init(seed_i ^ (32'hB000 + SB * 23));
    u_wmem.init(seed_i ^ (32'hC000 + SB * 29));
    u_snk.init(seed_i ^ (32'hD000 + SB * 31));
    fill_memories();
    repeat (5) @(posedge clk);
    #2 set_rst(1'b1);
    repeat (4) @(posedge clk);

    // ---- 1. MM2S directed corner cases ----
    run_mm2s_op(32'h0000_0100, 32'd0,    1'b0, "mm2s_zero");
    run_mm2s_op(32'h0000_0100, 32'd1,    1'b0, "mm2s_1B");
    run_mm2s_op(32'h0000_0104, 32'd2,    1'b0, "mm2s_2B");
    run_mm2s_op(32'h0000_0108, 32'd3,    1'b0, "mm2s_3B");
    run_mm2s_op(32'h0000_010C, 32'd4,    1'b0, "mm2s_4B");
    run_mm2s_op(32'h0000_0110, 32'd5,    1'b0, "mm2s_5B");
    // 4KB boundary crossings
    run_mm2s_op(32'h0000_0FFC, 32'd20,   1'b0, "mm2s_4kb_ffc");
    run_mm2s_op(32'h0000_0FF0, 32'd67,   1'b0, "mm2s_4kb_ff0");
    run_mm2s_op(32'h0000_0F00, 32'd515,  1'b0, "mm2s_4kb_f00");
    run_mm2s_op(32'h0000_1F80, 32'd1029, 1'b0, "mm2s_4kb_big");
    // Multi-descriptor stitched frame (cmd_last=0 then cmd_last=1)
    begin
      automatic axs_frame f = new();
      f.id = 6; f.dest = 2; f.user = 9; f.seq = 9999;
      for (int unsigned b = 0; b < 32; b++) f.bytes.push_back({1'b1, u_rmem.mem[32'h0200 + b]});
      for (int unsigned b = 0; b < 19; b++) f.bytes.push_back({1'b1, u_rmem.mem[32'h0FF4 + b]});
      sbh.put(0, f);
      issue_rd_cmd(32'h0000_0200, 32'd32, 4'd6, 4'd2, 4'd9, 1'b0);
      while (!rd_sts_seen_q) @(posedge clk);
      clr_rd_sts <= 1'b1; @(posedge clk); clr_rd_sts <= 1'b0;
      issue_rd_cmd(32'h0000_0FF4, 32'd19, 4'd6, 4'd2, 4'd9, 1'b1);
      wait_rd_done(1'b0, "mm2s_stitch");
      n_mm2s_ops++;
    end
    // SLVERR injection + recovery
    r_ready_pct = 50; r_resp_pct = 50; snk_rpct = 40;
    run_mm2s_op(32'h0000_0FF8, 32'd33, 1'b1, "mm2s_slverr");
    run_mm2s_op(32'h0000_0FF8, 32'd33, 1'b0, "mm2s_post_err");

    // ---- 2. MM2S constrained-random with heavy backpressure ----
    for (int i = 0; i < 40; i++) begin
      automatic logic [31:0] addr;
      automatic logic [31:0] len;
      r_ready_pct = rng.range(30, 100);
      r_resp_pct  = rng.range(30, 100);
      snk_rpct    = rng.range(25, 100);
      if (rng.pct(50)) begin
        // Bias near 4KB boundary
        automatic int page = rng.range(1, 3);
        addr = 32'(page * 4096 - int'(rng.range(1, 32)) * 4);
      end else begin
        addr = 32'(rng.range(0, (MEM_BYTES - 700) / 4) * 4);
      end
      len = 32'(rng.range(1, 600));
      run_mm2s_op(addr, len, 1'b0, "mm2s_rand");
    end

    // ---- 3. S2MM directed corner cases ----
    w_ready_pct = 100; w_resp_pct = 100;
    run_random_s2mm(32'h0000_0400, 0,  32'd0,   100, 1'b0, 1'b0, "s2mm_zero");
    run_random_s2mm(32'h0000_0400, 1,  32'd64,  100, 1'b0, 1'b0, "s2mm_1B");
    run_random_s2mm(32'h0000_0404, 2,  32'd64,  100, 1'b0, 1'b0, "s2mm_2B");
    run_random_s2mm(32'h0000_0408, 3,  32'd64,  100, 1'b0, 1'b0, "s2mm_3B");
    run_random_s2mm(32'h0000_040C, 4,  32'd64,  100, 1'b0, 1'b0, "s2mm_4B");
    run_random_s2mm(32'h0000_0410, 5,  32'd64,  100, 1'b1, 1'b0, "s2mm_5B_pos");
    // 4KB boundary crossings
    run_random_s2mm(32'h0000_0FFC, 64,  32'd128, 100, 1'b0, 1'b0, "s2mm_4kb_ffc");
    run_random_s2mm(32'h0000_0FF8, 131, 32'd256, 70,  1'b1, 1'b0, "s2mm_4kb_ff8");
    run_random_s2mm(32'h0000_1FE0, 261, 32'd512, 80,  1'b0, 1'b0, "s2mm_4kb_1fe0");
    // Truncation at non-4B-aligned max_len (10B of a 24B frame), then drain remaining 12B!
    begin
      automatic logic [31:0] bd[6];
      automatic logic [3:0]  bk[6];
      automatic logic [3:0]  bs[6];
      automatic logic        bl[6];
      automatic logic [31:0] bd2[3];
      automatic logic [3:0]  bk2[3];
      automatic logic [3:0]  bs2[3];
      automatic logic        bl2[3];
      for (int i = 0; i < 6; i++) begin
        bd[i] = rng.next(); bk[i] = 4'hF; bs[i] = 4'hF; bl[i] = (i == 5);
      end
      drive_s2mm_stream(32'h0000_0FF8, 32'd10, 1'b1, 6, bd, bk, bs, bl,
                        4'h2, 4'h5, 4'h1, 80, 1'b0, "s2mm_trunc_part1");
      for (int i = 0; i < 3; i++) begin
        bd2[i] = bd[i + 3]; bk2[i] = bk[i + 3]; bs2[i] = bs[i + 3]; bl2[i] = bl[i + 3];
      end
      drive_s2mm_stream(32'h0000_1004, 32'd64, 1'b1, 3, bd2, bk2, bs2, bl2,
                        4'h2, 4'h5, 4'h1, 80, 1'b0, "s2mm_trunc_part2");
    end
    // stop_on_last = 0 (merge multiple short frames into one contiguous buffer)
    begin
      automatic logic [31:0] bd[5];
      automatic logic [3:0]  bk[5];
      automatic logic [3:0]  bs[5];
      automatic logic        bl[5];
      for (int i = 0; i < 5; i++) begin
        bd[i] = rng.next(); bk[i] = 4'hF; bs[i] = 4'hF; bl[i] = (i == 1 || i == 3 || i == 4);
      end
      drive_s2mm_stream(32'h0000_0FF4, 32'd20, 1'b0, 5, bd, bk, bs, bl,
                        4'h7, 4'h1, 4'h3, 75, 1'b0, "s2mm_nostop_merge");
    end
    // BRESP SLVERR + recovery
    w_ready_pct = 45; w_resp_pct = 45;
    run_random_s2mm(32'h0000_0FF0, 40, 32'd64, 70, 1'b0, 1'b1, "s2mm_slverr");
    run_random_s2mm(32'h0000_0FF0, 40, 32'd64, 70, 1'b0, 1'b0, "s2mm_post_err");

    // ---- 4. S2MM constrained-random with heavy backpressure ----
    for (int i = 0; i < 40; i++) begin
      automatic logic [31:0] addr;
      automatic int unsigned flen;
      w_ready_pct = rng.range(25, 100);
      w_resp_pct  = rng.range(30, 100);
      if (rng.pct(50)) begin
        automatic int page = rng.range(1, 3);
        addr = 32'(page * 4096 - int'(rng.range(1, 32)) * 4);
      end else begin
        addr = 32'(rng.range(0, (MEM_BYTES - 700) / 4) * 4);
      end
      flen = rng.range(1, 520);
      run_random_s2mm(addr, flen, 32'(flen + rng.range(0, 32)), rng.range(35, 100),
                      1'b1, 1'b0, "s2mm_rand");
    end

    // ---- 5. Full MM2S -> AXIS -> S2MM loopback DMA copy ----
    for (int i = 0; i < 20; i++) begin
      automatic logic [31:0] sa = 32'(rng.range(1, 3) * 4096 - int'(rng.range(1, 24)) * 4);
      automatic logic [31:0] da = 32'(rng.range(1, 3) * 4096 - int'(rng.range(1, 24)) * 4);
      automatic logic [31:0] ln = 32'(rng.range(1, 450));
      r_ready_pct = rng.range(35, 100); r_resp_pct = rng.range(35, 100);
      w_ready_pct = rng.range(35, 100); w_resp_pct = rng.range(35, 100);
      run_loopback_copy(sa, da, ln, "loopback");
    end

    // ---- 6. Reset mid-burst recovery ----
    r_ready_pct = 30; r_resp_pct = 30; snk_rpct = 25;
    issue_rd_cmd(32'h0000_0FC0, 32'd256, 4'd1, 4'd2, 4'd3, 1'b1);
    repeat (12) @(posedge clk);
    #2 set_rst(1'b0);
    sbh.flush();
    #90 set_rst(1'b1);
    repeat (4) @(posedge clk);
    fill_memories();
    r_ready_pct = 70; r_resp_pct = 70; w_ready_pct = 70; w_resp_pct = 70; snk_rpct = 70;
    run_mm2s_op(32'h0000_0FFC, 32'd77, 1'b0, "post_rst_mm2s");
    run_random_s2mm(32'h0000_0FFC, 77, 32'd128, 70, 1'b0, 1'b0, "post_rst_s2mm");
    run_loopback_copy(32'h0000_0FF4, 32'h0000_1FF8, 32'd123, "post_rst_loop");

    frames_o = n_mm2s_ops + n_s2mm_ops + n_loop_ops;
    $display("[INFO] %-26s mm2s=%0d s2mm=%0d loop=%0d ar_bursts=%0d(4kb=%0d) aw_bursts=%0d(4kb=%0d) stalls(ar=%0d,aw=%0d,w=%0d)",
             NAME, n_mm2s_ops, n_s2mm_ops, n_loop_ops,
             u_rmem.n_ar_bursts, u_rmem.n_4kb_rd_splits,
             u_wmem.n_aw_bursts, u_wmem.n_4kb_wr_splits,
             u_rmem.n_ar_stalls, u_wmem.n_aw_stalls, u_wmem.n_w_stalls);
    if (u_rmem.n_4kb_rd_splits == 0 || u_wmem.n_4kb_wr_splits == 0)
      $fatal(1, "[%s] failed to exercise 4KB boundary splits", NAME);
    done_o = 1'b1;
  end
endmodule : axs_h_axi4_dma

// ---------------------------------------------------------------------------
// Top
// ---------------------------------------------------------------------------
module tb_axis_utils;
  localparam int NH = 39;

  int unsigned seed;
  logic [NH-1:0] done;
  int unsigned   frames [NH];

  initial begin
    if (!$value$plusargs("seed=%d", seed)) seed = 1;
    $display("[INFO] tb_axis_utils seed=%0d", seed);
  end

  // watchdog
  initial begin
    #400_000_000;
    $fatal(1, "tb_axis_utils: global watchdog expired (done mask=%b)", done);
  end

  // axis_register: all four modes + a narrow and a wide instance
  for (genvar m = 0; m < 4; m++) begin : g_reg
    axs_h_pt #(.KIND(0), .S_W(32), .M_W(32), .P0(m), .SB(m), .NAME($sformatf("register_mode%0d", m))) u_h (
      .seed_i(seed), .done_o(done[m]), .frames_o(frames[m]));
  end
  axs_h_pt #(.KIND(0), .S_W(8),  .M_W(8),  .P0(3), .SB(4), .NAME("register_full_8b")) u_h_r8 (
    .seed_i(seed), .done_o(done[4]), .frames_o(frames[4]));
  axs_h_pt #(.KIND(0), .S_W(64), .M_W(64), .P0(2), .SB(5), .NAME("register_bwd_64b")) u_h_r64 (
    .seed_i(seed), .done_o(done[5]), .frames_o(frames[5]));

  // sync fifo, beat mode
  axs_h_pt #(.KIND(1), .S_W(32), .M_W(32), .P0(16), .P1(0), .SB(6),  .NAME("fifo_d16")) u_h_f0 (
    .seed_i(seed), .done_o(done[6]), .frames_o(frames[6]));
  axs_h_pt #(.KIND(1), .S_W(16), .M_W(16), .P0(5),  .P1(1), .SB(7),  .NAME("fifo_d5_oreg")) u_h_f1 (
    .seed_i(seed), .done_o(done[7]), .frames_o(frames[7]));
  axs_h_pt #(.KIND(1), .S_W(32), .M_W(32), .P0(2),  .P1(0), .SB(8),  .NAME("fifo_d2")) u_h_f2 (
    .seed_i(seed), .done_o(done[8]), .frames_o(frames[8]));

  // frame fifo: oversize drop (back-pressure), drop-when-full lossy + stalled model, bad-frame drop
  axs_h_pt #(.KIND(2), .S_W(32), .M_W(32), .P0(16), .P1(0), .P2(0), .P3(0), .P4(0), .SB(9),  .NAME("framefifo_oversize")) u_h_ff0 (
    .seed_i(seed), .done_o(done[9]), .frames_o(frames[9]));
  axs_h_pt #(.KIND(2), .S_W(32), .M_W(32), .P0(16), .P1(0), .P2(1), .P3(0), .P4(1), .SB(10), .NAME("framefifo_dropfull")) u_h_ff1 (
    .seed_i(seed), .done_o(done[10]), .frames_o(frames[10]));
  axs_h_pt #(.KIND(2), .S_W(32), .M_W(32), .P0(12), .P1(1), .P2(0), .P3(1), .P4(0), .SB(11), .NAME("framefifo_badframe")) u_h_ff2 (
    .seed_i(seed), .done_o(done[11]), .frames_o(frames[11]));
  axs_h_pt #(.KIND(2), .S_W(16), .M_W(16), .P0(7),  .P1(0), .P2(1), .P3(0), .P4(1), .SB(12), .NAME("framefifo_d7_dropfull")) u_h_ff3 (
    .seed_i(seed), .done_o(done[12]), .frames_o(frames[12]));

  // async fifo, read clock faster / slower / equal
  axs_h_pt #(.KIND(3), .S_W(32), .M_W(32), .P0(3), .P1(4),  .SB(13), .NAME("async_fifo_rd_fast")) u_h_a0 (
    .seed_i(seed), .done_o(done[13]), .frames_o(frames[13]));
  axs_h_pt #(.KIND(3), .S_W(32), .M_W(32), .P0(2), .P1(11), .SB(14), .NAME("async_fifo_rd_slow")) u_h_a1 (
    .seed_i(seed), .done_o(done[14]), .frames_o(frames[14]));
  axs_h_pt #(.KIND(3), .S_W(16), .M_W(16), .P0(4), .P1(5),  .SB(15), .NAME("async_fifo_equal")) u_h_a2 (
    .seed_i(seed), .done_o(done[15]), .frames_o(frames[15]));

  // width converters: equal, power-of-two and odd ratios, up and down
  localparam int NW = 12;
  localparam int WS [NW] = '{8,  32, 16, 48, 32, 64, 8, 8,  64, 24, 8,  40};
  localparam int WM [NW] = '{32, 8,  48, 16, 64, 16, 8, 64, 8,  8,  24, 8};
  for (genvar w = 0; w < NW; w++) begin : g_wc
    axs_h_pt #(.KIND(4), .S_W(WS[w]), .M_W(WM[w]), .SB(16 + w), .NAME($sformatf("wconv_%0d_to_%0d", WS[w], WM[w]))) u_h (
      .seed_i(seed), .done_o(done[16 + w]), .frames_o(frames[16 + w]));
  end

  // arbitrating mux
  axs_h_arb #(.N(4), .ARB_TYPE(0), .FRAME_LOCK(1'b1), .OUT_REG(1'b0), .SB(28), .NAME("arbmux_rr4")) u_h_m0 (
    .seed_i(seed), .done_o(done[28]), .frames_o(frames[28]));
  axs_h_arb #(.N(3), .ARB_TYPE(0), .FRAME_LOCK(1'b1), .OUT_REG(1'b1), .SB(29), .NAME("arbmux_rr3_oreg")) u_h_m1 (
    .seed_i(seed), .done_o(done[29]), .frames_o(frames[29]));
  axs_h_arb #(.N(4), .ARB_TYPE(1), .FRAME_LOCK(1'b1), .OUT_REG(1'b0), .SB(30), .NAME("arbmux_fixed4")) u_h_m2 (
    .seed_i(seed), .done_o(done[30]), .frames_o(frames[30]));
  axs_h_arb #(.N(5), .ARB_TYPE(0), .FRAME_LOCK(1'b0), .OUT_REG(1'b0), .SB(31), .NAME("arbmux_rr5_beat")) u_h_m3 (
    .seed_i(seed), .done_o(done[31]), .frames_o(frames[31]));

  // demux
  axs_h_demux #(.N(4), .ROUTE_TDEST(1'b1), .SB(32), .NAME("demux_tdest4")) u_h_d0 (
    .seed_i(seed), .done_o(done[32]), .frames_o(frames[32]));
  axs_h_demux #(.N(3), .ROUTE_TDEST(1'b1), .SB(33), .NAME("demux_tdest3")) u_h_d1 (
    .seed_i(seed), .done_o(done[33]), .frames_o(frames[33]));
  axs_h_demux #(.N(4), .ROUTE_TDEST(1'b0), .SB(34), .NAME("demux_select4")) u_h_d2 (
    .seed_i(seed), .done_o(done[34]), .frames_o(frames[34]));
  axs_h_demux #(.N(3), .ROUTE_TDEST(1'b0), .SB(35), .NAME("demux_select3")) u_h_d3 (
    .seed_i(seed), .done_o(done[35]), .frames_o(frames[35]));

  // DMA adapters: axi4_to_axis (MM2S) + axis_to_axi4 (S2MM) + loopback copy
  axs_h_axi4_dma #(.MAX_BURST(16), .FIFO_DEP(32), .SB(36), .NAME("axi4_dma_b16")) u_h_dma0 (
    .seed_i(seed), .done_o(done[36]), .frames_o(frames[36]));
  axs_h_axi4_dma #(.MAX_BURST(1),  .FIFO_DEP(4),  .SB(37), .NAME("axi4_dma_b1"))  u_h_dma1 (
    .seed_i(seed), .done_o(done[37]), .frames_o(frames[37]));
  axs_h_axi4_dma #(.MAX_BURST(64), .FIFO_DEP(64), .SB(38), .NAME("axi4_dma_b64")) u_h_dma2 (
    .seed_i(seed), .done_o(done[38]), .frames_o(frames[38]));

  initial begin
    automatic int unsigned total = 0;
    #10;
    wait (done == {NH{1'b1}});
    for (int i = 0; i < NH; i++) total += frames[i];
    $display("[PASS] tb_axis_utils: %0d DUT configs (axis_register x6, axis_fifo beat x3 / frame x4, axis_async_fifo x3, axis_width_converter x12, axis_arb_mux x4, axis_demux x4, axi4_to_axis+axis_to_axi4 x3), %0d frames/ops checked, seed=%0d", NH, total, seed);
    $finish;
  end
endmodule : tb_axis_utils
