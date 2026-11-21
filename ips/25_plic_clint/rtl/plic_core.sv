// SPDX-License-Identifier: Apache-2.0
// plic_core.sv - Platform-Level Interrupt Controller (RISC-V PLIC memory map, own implementation)
//
// Offsets are relative to the PLIC base (only addr[25:0] is decoded; word aligned):
//   0x000000 + 4*ID          PRIORITY[ID]       RW  ID = 1..NUM_SOURCES (bits above PRIO_BITS read 0)
//   0x001000 + 4*w           PENDING[w]         RO  bit b of word w = source 32*w+b
//   0x002000 + 0x80*c + 4*w  ENABLE[c][w]       RW  per-context enable bitmaps
//   0x200000 + 0x1000*c      THRESHOLD[c]       RW  (PRIO_BITS)
//   0x200004 + 0x1000*c      CLAIM/COMPLETE[c]  R: claim (returns ID or 0; clears pending, sets in-flight)
//                                               W: complete (ID; ignored unless ID enabled in context c)
// Reserved offsets (and ID 0 / IDs > NUM_SOURCES / contexts >= NUM_CONTEXTS) read as 0, writes ignored.
//
// irq_i[i] is interrupt source ID (i+1).  ext_irq_o[c] is asserted while a pending, enabled source
// with priority > threshold[c] exists for context c.  The claim returns the highest-priority such
// source (lowest ID on ties).
//
// The CSR slave always responds in the cycle of the request (ready = 1, err = 0).

module plic_core
  import bus_pkg::*;
#(
  parameter int unsigned            NUM_SOURCES  = 32,    // 1..1023
  parameter int unsigned            NUM_CONTEXTS = 2,     // 1..32
  parameter int unsigned            PRIO_BITS    = 3,     // 1..16
  parameter logic [NUM_SOURCES-1:0] EDGE_MASK    = '0,    // bit i: source ID i+1 is edge-triggered
  parameter int unsigned            SYNC_STAGES  = 2,     // input synchronizer depth (0 = none)
  parameter bit                     REG_OUT      = 1'b0   // register ext_irq_o (adds 1 cycle)
) (
  input  logic                        clk_i,
  input  logic                        rst_ni,

  input  csr_req_t                    csr_req_i,
  output csr_rsp_t                    csr_rsp_o,

  input  logic [NUM_SOURCES-1:0]      irq_i,
  output logic [NUM_CONTEXTS-1:0]     ext_irq_o
);

  localparam int unsigned N    = NUM_SOURCES;
  localparam int unsigned C    = NUM_CONTEXTS;
  localparam int unsigned ID_W = (N > 1) ? $clog2(N + 1) : 1;
  localparam int unsigned CW   = (C > 1) ? $clog2(C) : 1;
  localparam int unsigned NW   = (N + 32) / 32;   // enable/pending words covering IDs 0..N

  if (N < 1 || N > 1023) begin : g_chk_n
    $fatal(1, "plic_core: NUM_SOURCES must be 1..1023");
  end
  if (C < 1 || C > 32) begin : g_chk_c
    $fatal(1, "plic_core: NUM_CONTEXTS must be 1..32");
  end
  if (PRIO_BITS < 1 || PRIO_BITS > 16) begin : g_chk_p
    $fatal(1, "plic_core: PRIO_BITS must be 1..16");
  end

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  logic [PRIO_BITS-1:0] prio_q [1:N];
  logic [N:0]           en_q   [C];      // bit 0 (ID 0) is constant 0
  logic [PRIO_BITS-1:0] thr_q  [C];

  // ---------------------------------------------------------------------------
  // Input synchronizers + gateways
  // ---------------------------------------------------------------------------
  logic [N-1:0] irq_sync;
  irq_aggregator #(
    .N (N), .SYNC_STAGES (SYNC_STAGES), .MODE ('0), .ACTIVE_LOW ('0)
  ) u_sync (
    .clk_i (clk_i), .rst_ni (rst_ni),
    .irq_i (irq_i), .irq_mask_i ({N{1'b1}}),
    .irq_o (irq_sync), .any_o ()
  );

  logic [N-1:0] ip_w;
  logic         claim_vld, compl_vld;
  logic [ID_W-1:0] claim_id, compl_id;

  plic_gateway #(.N (N), .EDGE_MASK (EDGE_MASK)) u_gw (
    .clk_i (clk_i), .rst_ni (rst_ni),
    .irq_i (irq_sync),
    .claim_vld_i (claim_vld), .claim_id_i (claim_id),
    .compl_vld_i (compl_vld), .compl_id_i (compl_id),
    .ip_o (ip_w), .inflight_o ()
  );

  // ---------------------------------------------------------------------------
  // Per-context selection
  // ---------------------------------------------------------------------------
  logic [N-1:0][PRIO_BITS-1:0] prio_vec;
  always_comb begin
    for (int i = 0; i < N; i++) prio_vec[i] = prio_q[i+1];
  end

  logic [C-1:0]            sel_valid;
  logic [ID_W-1:0]         sel_id   [C];
  logic [PRIO_BITS-1:0]    sel_prio [C];
  logic [C-1:0]            ctx_hit;          // pending-enabled-above-threshold exists
  logic [ID_W-1:0]         ctx_claim_id [C]; // value a claim read would return (0 if none)

  for (genvar c = 0; c < C; c++) begin : g_ctx
    logic [N-1:0] cand;
    assign cand = ip_w & en_q[c][N:1];

    plic_target_select #(.N (N), .PRIO_BITS (PRIO_BITS)) u_sel (
      .cand_i (cand), .prio_i (prio_vec),
      .valid_o (sel_valid[c]), .id_o (sel_id[c]), .prio_o (sel_prio[c])
    );

    assign ctx_hit[c]      = sel_valid[c] && (sel_prio[c] > thr_q[c]);
    assign ctx_claim_id[c] = ctx_hit[c] ? sel_id[c] : '0;
  end

  if (REG_OUT) begin : g_regout
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) ext_irq_o <= '0;
      else         ext_irq_o <= ctx_hit;
    end
  end else begin : g_combout
    assign ext_irq_o = ctx_hit;
  end

  // ---------------------------------------------------------------------------
  // Address decode
  // ---------------------------------------------------------------------------
  wire [13:0] page   = csr_req_i.addr[25:12];
  wire [9:0]  widx   = csr_req_i.addr[11:2];

  wire        reg_prio_pg = (page == 14'd0);
  wire        reg_pend_pg = (page == 14'd1);
  wire        reg_en_pg   = (csr_req_i.addr[25:21] == 5'd0) && (page >= 14'd2) && (page < 14'h200);
  wire        reg_ctx_pg  = (page >= 14'h200);

  wire [13:0] en_ctx_raw  = csr_req_i.addr[20:7] - 14'h40;
  wire [4:0]  en_word     = csr_req_i.addr[6:2];
  wire [13:0] cx_ctx_raw  = page - 14'h200;

  wire        en_ctx_ok   = reg_en_pg  && (en_ctx_raw < 14'(C)) && (32'(en_word) < 32'(NW));
  wire        cx_ctx_ok   = reg_ctx_pg && (cx_ctx_raw < 14'(C));
  wire [CW-1:0] en_ctx    = en_ctx_raw[CW-1:0];
  wire [CW-1:0] cx_ctx    = cx_ctx_raw[CW-1:0];

  wire        prio_ok     = reg_prio_pg && (widx >= 10'd1) && (32'(widx) <= 32'(N));
  wire        pend_ok     = reg_pend_pg && (csr_req_i.addr[11:7] == 5'd0) && (32'(csr_req_i.addr[6:2]) < 32'(NW));
  wire        thr_hit     = cx_ctx_ok && (widx == 10'd0);
  wire        clm_hit     = cx_ctx_ok && (widx == 10'd1);

  wire        rd = csr_req_i.valid && !csr_req_i.write;
  wire        wr = csr_req_i.valid &&  csr_req_i.write;

  function automatic logic [31:0] merge(input logic [31:0] old_v, input logic [31:0] new_v,
                                        input logic [3:0] strb);
    logic [31:0] m;
    m = {{8{strb[3]}}, {8{strb[2]}}, {8{strb[1]}}, {8{strb[0]}}};
    return (old_v & ~m) | (new_v & m);
  endfunction

  // ---------------------------------------------------------------------------
  // Claim / complete side effects
  // ---------------------------------------------------------------------------
  logic [31:0] cmpl_val;
  assign cmpl_val = merge(32'd0, csr_req_i.wdata, csr_req_i.wstrb);

  logic        cmpl_id_ok;
  logic [ID_W-1:0] cmpl_id_w;
  always_comb begin
    cmpl_id_w  = ID_W'(cmpl_val);
    cmpl_id_ok = (cmpl_val >= 32'd1) && (cmpl_val <= 32'(N));
  end

  logic cmpl_en_w;
  always_comb begin
    cmpl_en_w = 1'b0;
    if (cmpl_id_ok) cmpl_en_w = en_q[cx_ctx][cmpl_id_w];
  end

  assign claim_id  = ctx_claim_id[cx_ctx];
  assign claim_vld = rd && clm_hit && (claim_id != '0);
  assign compl_id  = cmpl_id_w;
  assign compl_vld = wr && clm_hit && cmpl_id_ok && cmpl_en_w;

  // ---------------------------------------------------------------------------
  // Register writes
  // ---------------------------------------------------------------------------
  logic [31:0] en_cur;
  logic [31:0] en_new;
  logic [32*NW-1:0] en_pad;
  always_comb begin
    en_pad = '0;
    en_pad[N:0] = en_q[en_ctx];
    en_cur = en_pad[32*en_word +: 32];
    en_new = merge(en_cur, csr_req_i.wdata, csr_req_i.wstrb);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int i = 1; i <= N; i++) prio_q[i] <= '0;
      for (int c = 0; c < C; c++) begin
        en_q[c]  <= '0;
        thr_q[c] <= '0;
      end
    end else if (wr) begin
      if (prio_ok) begin
        prio_q[widx] <= PRIO_BITS'(merge(32'(prio_q[widx]), csr_req_i.wdata, csr_req_i.wstrb));
      end
      if (en_ctx_ok) begin
        for (int b = 0; b < 32; b++) begin
          if ((32 * int'(en_word) + b >= 1) && (32 * int'(en_word) + b <= int'(N)))
            en_q[en_ctx][32 * int'(en_word) + b] <= en_new[b];
        end
      end
      if (thr_hit) begin
        thr_q[cx_ctx] <= PRIO_BITS'(merge(32'(thr_q[cx_ctx]), csr_req_i.wdata, csr_req_i.wstrb));
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Register reads
  // ---------------------------------------------------------------------------
  logic [32*NW-1:0] pend_pad;
  always_comb begin
    pend_pad = '0;
    pend_pad[N:1] = ip_w;
  end

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    if (prio_ok)          csr_rsp_o.rdata = 32'(prio_q[widx]);
    else if (pend_ok)     csr_rsp_o.rdata = pend_pad[32*csr_req_i.addr[6:2] +: 32];
    else if (en_ctx_ok)   csr_rsp_o.rdata = en_cur;
    else if (thr_hit)     csr_rsp_o.rdata = 32'(thr_q[cx_ctx]);
    else if (clm_hit)     csr_rsp_o.rdata = 32'(ctx_claim_id[cx_ctx]);
  end

endmodule : plic_core
