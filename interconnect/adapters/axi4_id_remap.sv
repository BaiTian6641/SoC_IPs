// SPDX-License-Identifier: Apache-2.0
// axi4_id_remap.sv - AXI4 ID-Width / ID-Remap Table with Per-ID In-Order Enforcement
//
// Maps arbitrary 4-bit upstream AXI4 IDs (0..15) onto a compact downstream ID space
// of width OUT_ID_W (1..4 bits, up to min(2**OUT_ID_W, MAX_OUTSTANDING) concurrent
// outstanding transactions per channel).
//
// Features:
//   * Independent Write (AW/W/B) and Read (AR/R) slot tables.
//   * Guarantees AMBA AXI4 per-ID in-order response ordering (A5.3) even when attached
//     to an out-of-order / beat-interleaving downstream slave: if an upstream ID already
//     has an active transaction in flight, the next request with the same upstream ID
//     waits until its predecessor completes, while requests with distinct upstream IDs
//     proceed in parallel on other downstream slots.
//   * Freezes the allocated downstream slot while AW/AR is stalled (valid && !ready) so
//     m_axi_o.awid / m_axi_o.arid never mutate mid-stall even if a lower slot retires.
//   * Keeps W beats aligned to accepted AWs (w_pend_q) for AXI4 write-data ordering.

module axi4_id_remap
  import bus_pkg::*;
#(
  parameter int unsigned OUT_ID_W        = 2, // 1..4 downstream ID bits
  parameter int unsigned MAX_OUTSTANDING = 4  // 1..16; clamped to <= 2**OUT_ID_W
) (
  input  logic      clk_i,
  input  logic      rst_ni,

  // Upstream AXI4 Slave Port (full 4-bit IDs)
  input  axi4_m2s_t s_axi_i,
  output axi4_s2m_t s_axi_o,

  // Downstream AXI4 Master Port (IDs in range 0 .. NUM_SLOTS-1)
  output axi4_m2s_t m_axi_o,
  input  axi4_s2m_t m_axi_i
);

  localparam int unsigned EFF_ID_W  = (OUT_ID_W < 1) ? 1 : (OUT_ID_W > 4) ? 4 : OUT_ID_W;
  localparam int unsigned ID_CAP    = (1 << EFF_ID_W);
  localparam int unsigned NUM_SLOTS = (MAX_OUTSTANDING < 1)      ? 1      :
                                      (MAX_OUTSTANDING > ID_CAP) ? ID_CAP : MAX_OUTSTANDING;
  localparam int unsigned SLOT_W    = (NUM_SLOTS <= 1) ? 1 : $clog2(NUM_SLOTS);
  localparam int unsigned WCNT_W    = $clog2(NUM_SLOTS + 1);

  // Write slot table
  logic [NUM_SLOTS-1:0]      w_busy_q;
  logic [NUM_SLOTS-1:0][3:0] w_orig_id_q;
  logic [WCNT_W-1:0]         w_pend_q;      // Accepted AWs still awaiting WLAST
  logic                      aw_stall_q;
  logic [SLOT_W-1:0]         aw_stall_slot_q;

  // Read slot table
  logic [NUM_SLOTS-1:0]      r_busy_q;
  logic [NUM_SLOTS-1:0][3:0] r_orig_id_q;
  logic                      ar_stall_q;
  logic [SLOT_W-1:0]         ar_stall_slot_q;

  // Free slot & same-ID collision detection (Write)
  logic              w_free_vld, w_id_clash;
  logic [SLOT_W-1:0] w_free_slot, w_alloc_slot;
  always_comb begin
    w_free_vld  = 1'b0;
    w_free_slot = '0;
    w_id_clash  = 1'b0;
    for (int i = 0; i < NUM_SLOTS; i++) begin
      if (!w_free_vld && !w_busy_q[i]) begin
        w_free_vld  = 1'b1;
        w_free_slot = SLOT_W'(i);
      end
      if (w_busy_q[i] && (w_orig_id_q[i] == s_axi_i.awid)) begin
        w_id_clash = 1'b1;
      end
    end
  end

  assign w_alloc_slot = aw_stall_q ? aw_stall_slot_q : w_free_slot;
  wire   aw_allow     = aw_stall_q || (w_free_vld && !w_id_clash && (w_pend_q < WCNT_W'(NUM_SLOTS)));

  // Free slot & same-ID collision detection (Read)
  logic              r_free_vld, r_id_clash;
  logic [SLOT_W-1:0] r_free_slot, r_alloc_slot;
  always_comb begin
    r_free_vld  = 1'b0;
    r_free_slot = '0;
    r_id_clash  = 1'b0;
    for (int i = 0; i < NUM_SLOTS; i++) begin
      if (!r_free_vld && !r_busy_q[i]) begin
        r_free_vld  = 1'b1;
        r_free_slot = SLOT_W'(i);
      end
      if (r_busy_q[i] && (r_orig_id_q[i] == s_axi_i.arid)) begin
        r_id_clash = 1'b1;
      end
    end
  end

  assign r_alloc_slot = ar_stall_q ? ar_stall_slot_q : r_free_slot;
  wire   ar_allow     = ar_stall_q || (r_free_vld && !r_id_clash);

  // Combinational channel wiring
  wire w_allow = (w_pend_q != '0);

  always_comb begin
    m_axi_o         = s_axi_i;
    m_axi_o.awid    = 4'(w_alloc_slot);
    m_axi_o.awvalid = s_axi_i.awvalid && aw_allow;
    m_axi_o.wvalid  = s_axi_i.wvalid  && w_allow;
    m_axi_o.arid    = 4'(r_alloc_slot);
    m_axi_o.arvalid = s_axi_i.arvalid && ar_allow;

    s_axi_o         = m_axi_i;
    s_axi_o.awready = m_axi_i.awready && aw_allow;
    s_axi_o.wready  = m_axi_i.wready  && w_allow;
    s_axi_o.arready = m_axi_i.arready && ar_allow;

    // Restore original upstream BID / RID from slot tables
    s_axi_o.bid = '0;
    s_axi_o.rid = '0;
    for (int i = 0; i < NUM_SLOTS; i++) begin
      if (m_axi_i.bid == 4'(i)) s_axi_o.bid = w_orig_id_q[i];
      if (m_axi_i.rid == 4'(i)) s_axi_o.rid = r_orig_id_q[i];
    end
  end

  wire aw_hs = m_axi_o.awvalid && m_axi_i.awready;
  wire wlast_hs = m_axi_o.wvalid && m_axi_i.wready && m_axi_o.wlast;
  wire b_hs  = m_axi_i.bvalid  && s_axi_i.bready;
  wire ar_hs = m_axi_o.arvalid && m_axi_i.arready;
  wire rlast_hs = m_axi_i.rvalid && s_axi_i.rready && m_axi_i.rlast;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      w_busy_q        <= '0;
      w_orig_id_q     <= '0;
      w_pend_q        <= '0;
      aw_stall_q      <= 1'b0;
      aw_stall_slot_q <= '0;
      r_busy_q        <= '0;
      r_orig_id_q     <= '0;
      ar_stall_q      <= 1'b0;
      ar_stall_slot_q <= '0;
    end else begin
      // Write channel tracking
      w_pend_q <= w_pend_q + (aw_hs ? WCNT_W'(1) : '0) - (wlast_hs ? WCNT_W'(1) : '0);

      if (aw_hs) begin
        aw_stall_q               <= 1'b0;
        w_busy_q[w_alloc_slot]   <= 1'b1;
        w_orig_id_q[w_alloc_slot] <= s_axi_i.awid;
      end else if (m_axi_o.awvalid && !m_axi_i.awready && !aw_stall_q) begin
        aw_stall_q      <= 1'b1;
        aw_stall_slot_q <= w_free_slot;
      end

      if (b_hs) begin
        for (int i = 0; i < NUM_SLOTS; i++) begin
          if (m_axi_i.bid == 4'(i)) w_busy_q[i] <= 1'b0;
        end
      end

      // Read channel tracking
      if (ar_hs) begin
        ar_stall_q                <= 1'b0;
        r_busy_q[r_alloc_slot]    <= 1'b1;
        r_orig_id_q[r_alloc_slot] <= s_axi_i.arid;
      end else if (m_axi_o.arvalid && !m_axi_i.arready && !ar_stall_q) begin
        ar_stall_q      <= 1'b1;
        ar_stall_slot_q <= r_free_slot;
      end

      if (rlast_hs) begin
        for (int i = 0; i < NUM_SLOTS; i++) begin
          if (m_axi_i.rid == 4'(i)) r_busy_q[i] <= 1'b0;
        end
      end
    end
  end

endmodule : axi4_id_remap
