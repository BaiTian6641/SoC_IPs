// SPDX-License-Identifier: Apache-2.0
// clint_core.sv - Core-local interruptor: machine timer (MTIMER), software interrupts (MSWI, SSWI)
//
// This block sees region-relative offsets in csr_req_i.addr[15:0]; the three region selects are
// decoded by plic_clint_core (so the same core serves the legacy single-window CLINT layout and
// the split ACLINT layout).  Region layouts (offset inside the region):
//   MSWI   (16 KiB) : 0x0000 + 4*h        MSIP[h]       RW  bit 0 only
//   MTIMER (32 KiB) : 0x0000 + 8*h        MTIMECMP[h]   RW  64-bit as two 32-bit words (lo, hi)
//                     0x7FF8              MTIME_LO      RW
//                     0x7FFC              MTIME_HI      RW
//   SSWI   (16 KiB) : 0x0000 + 4*h        SETSSIP[h]    W   writing bit0 = 1 emits a 1-cycle pulse on ssip_o[h]; reads 0
// Legacy CLINT layout = MSWI at +0x0000 and MTIMER at +0x4000 of one 64 KiB window
// (msip 0x0000, mtimecmp 0x4000, mtime 0xBFF8).
//
// 64-bit access over the 32-bit bus
//   * MTIME_SNAPSHOT=1 : reading MTIME_LO captures MTIME_HI into a shadow; the next MTIME_HI read
//     returns the shadow (single use), so "read LO, then HI" is torn-free (single bus master).
//     The portable hi/lo/hi retry loop is always safe.
//   * ATOMIC_WRITE=1   : writes to the LO word of MTIMECMP[h] / MTIME are staged and become
//     visible only when the HI word is written, both words at once (stage is per hart for MTIMECMP).
//   * ATOMIC_WRITE=0   : each 32-bit write takes effect immediately; use the standard sequence
//     (write HI=0xFFFFFFFF, write LO, write HI) to avoid a spurious timer interrupt.
// MTIMECMP resets to all-ones (timer interrupt inactive), MTIME to 0.
// timer_irq_o[h] = (MTIME >= MTIMECMP[h]) unsigned 64-bit.

module clint_core
  import bus_pkg::*;
#(
  parameter int unsigned NUM_HARTS      = 1,      // 1..4095
  parameter bit          MTIME_SNAPSHOT = 1'b1,
  parameter bit          ATOMIC_WRITE   = 1'b0,
  parameter bit          USE_TICK_I     = 1'b0,   // 1: mtime counts tick_i pulses, 0: prescaled clk
  parameter int unsigned PRESCALE       = 1,      // clk cycles per mtime tick (USE_TICK_I = 0)
  parameter bit          ENABLE_SSWI    = 1'b0,
  parameter bit          REG_OUT        = 1'b0    // register timer/sw irq outputs
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  input  csr_req_t                csr_req_i,       // addr[15:0] = region-relative offset
  input  logic                    sel_mswi_i,
  input  logic                    sel_mtimer_i,
  input  logic                    sel_sswi_i,
  output csr_rsp_t                csr_rsp_o,

  input  logic                    tick_i,
  input  logic                    halt_i,          // 1 = freeze mtime (debug halt)

  output logic [NUM_HARTS-1:0]    timer_irq_o,
  output logic [NUM_HARTS-1:0]    sw_irq_o,
  output logic [NUM_HARTS-1:0]    ssip_o,
  output logic [63:0]             mtime_o
);

  localparam int unsigned H  = NUM_HARTS;
  localparam int unsigned HW = (NUM_HARTS > 1) ? $clog2(NUM_HARTS) : 1;

  if (H < 1 || H > 4095) begin : g_chk_h
    $fatal(1, "clint_core: NUM_HARTS must be 1..4095");
  end
  if (PRESCALE < 1) begin : g_chk_ps
    $fatal(1, "clint_core: PRESCALE must be >= 1");
  end

  logic [63:0] mtime_q;
  logic [63:0] cmp_q   [H];
  logic [H-1:0] msip_q;
  logic [H-1:0] ssip_q;

  // ---------------------------------------------------------------------------
  // Tick generation
  // ---------------------------------------------------------------------------
  logic tick_w;
  if (USE_TICK_I) begin : g_tick_ext
    assign tick_w = tick_i;
  end else if (PRESCALE == 1) begin : g_tick_clk
    assign tick_w = 1'b1;
  end else begin : g_tick_pre
    localparam int unsigned PW = $clog2(PRESCALE);
    logic [PW-1:0] pcnt_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni)                            pcnt_q <= '0;
      else if (pcnt_q == PW'(PRESCALE - 1))   pcnt_q <= '0;
      else                                    pcnt_q <= pcnt_q + 1'b1;
    end
    assign tick_w = (pcnt_q == PW'(PRESCALE - 1));
  end

  // ---------------------------------------------------------------------------
  // Decode
  // ---------------------------------------------------------------------------
  wire        rd = csr_req_i.valid && !csr_req_i.write;
  wire        wr = csr_req_i.valid &&  csr_req_i.write;
  wire [15:0] off = csr_req_i.addr[15:0];

  // MSWI / SSWI: 4 bytes per hart, window 16 KiB
  wire [11:0] sw_hart = off[13:2];
  wire        sw_in   = (off[15:14] == 2'b00) && (32'(sw_hart) < H);
  wire        mswi_ok = sel_mswi_i && sw_in;
  wire        sswi_ok = ENABLE_SSWI && sel_sswi_i && sw_in;

  // MTIMER: window 32 KiB
  wire [11:0] cmp_hart   = off[14:3];
  wire        mt_in_win  = (off[15] == 1'b0);
  wire        mtime_lo_h = sel_mtimer_i && mt_in_win && (off[14:2] == 13'h1FFE);
  wire        mtime_hi_h = sel_mtimer_i && mt_in_win && (off[14:2] == 13'h1FFF);
  wire        cmp_ok     = sel_mtimer_i && mt_in_win && (off[14:3] < 12'hFFF) && (32'(cmp_hart) < H);
  wire        cmp_hi_sel = off[2];
  wire [HW-1:0] sw_hi  = sw_hart[HW-1:0];   // index form (range-checked by *_ok)
  wire [HW-1:0] cmp_hi = cmp_hart[HW-1:0];

  function automatic logic [31:0] merge(input logic [31:0] old_v, input logic [31:0] new_v,
                                        input logic [3:0] strb);
    logic [31:0] m;
    m = {{8{strb[3]}}, {8{strb[2]}}, {8{strb[1]}}, {8{strb[0]}}};
    return (old_v & ~m) | (new_v & m);
  endfunction

  // ---------------------------------------------------------------------------
  // MTIME read snapshot
  // ---------------------------------------------------------------------------
  logic [31:0] snap_hi_q;
  logic        snap_v_q;

  // ---------------------------------------------------------------------------
  // Staged writes (ATOMIC_WRITE)
  // ---------------------------------------------------------------------------
  logic [31:0]  mt_stage_lo_q;
  logic         mt_stage_v_q;
  logic [31:0]  cmp_stage_lo_q [H];
  logic [H-1:0] cmp_stage_v_q;

  logic [63:0] cmp_cur;
  always_comb begin
    cmp_cur = '1;
    if (cmp_ok) cmp_cur = cmp_q[cmp_hi];
  end

  logic ssip_pulse;
  assign ssip_pulse = wr && sswi_ok && csr_req_i.wstrb[0] && csr_req_i.wdata[0];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      mtime_q       <= '0;
      msip_q        <= '0;
      ssip_q        <= '0;
      snap_hi_q     <= '0;
      snap_v_q      <= 1'b0;
      mt_stage_lo_q <= '0;
      mt_stage_v_q  <= 1'b0;
      cmp_stage_v_q <= '0;
      for (int h = 0; h < H; h++) begin
        cmp_q[h]           <= '1;
        cmp_stage_lo_q[h]  <= '0;
      end
    end else begin
      // time base
      if (tick_w && !halt_i && !(wr && (mtime_lo_h || mtime_hi_h))) mtime_q <= mtime_q + 64'd1;

      // snapshot capture / consume
      if (rd && mtime_lo_h) begin
        snap_hi_q <= mtime_q[63:32];
        snap_v_q  <= 1'b1;
      end else if (rd && mtime_hi_h) begin
        snap_v_q <= 1'b0;
      end

      // single-hart-clear of ssip pulse register
      ssip_q <= '0;
      if (ssip_pulse) ssip_q[sw_hi] <= 1'b1;

      if (wr) begin
        if (mswi_ok && csr_req_i.wstrb[0]) msip_q[sw_hi] <= csr_req_i.wdata[0];

        if (cmp_ok) begin
          if (!cmp_hi_sel) begin
            if (ATOMIC_WRITE) begin
              cmp_stage_lo_q[cmp_hi] <= merge(cmp_q[cmp_hi][31:0], csr_req_i.wdata, csr_req_i.wstrb);
              cmp_stage_v_q[cmp_hi]  <= 1'b1;
            end else begin
              cmp_q[cmp_hi][31:0] <= merge(cmp_q[cmp_hi][31:0], csr_req_i.wdata, csr_req_i.wstrb);
            end
          end else begin
            cmp_q[cmp_hi][63:32] <= merge(cmp_q[cmp_hi][63:32], csr_req_i.wdata, csr_req_i.wstrb);
            if (ATOMIC_WRITE && cmp_stage_v_q[cmp_hi]) begin
              cmp_q[cmp_hi][31:0]   <= cmp_stage_lo_q[cmp_hi];
              cmp_stage_v_q[cmp_hi] <= 1'b0;
            end
          end
        end

        if (mtime_lo_h) begin
          snap_v_q <= 1'b0;
          if (ATOMIC_WRITE) begin
            mt_stage_lo_q <= merge(mtime_q[31:0], csr_req_i.wdata, csr_req_i.wstrb);
            mt_stage_v_q  <= 1'b1;
          end else begin
            mtime_q[31:0] <= merge(mtime_q[31:0], csr_req_i.wdata, csr_req_i.wstrb);
          end
        end
        if (mtime_hi_h) begin
          snap_v_q <= 1'b0;
          if (ATOMIC_WRITE && mt_stage_v_q) begin
            mtime_q      <= {merge(mtime_q[63:32], csr_req_i.wdata, csr_req_i.wstrb), mt_stage_lo_q};
            mt_stage_v_q <= 1'b0;
          end else begin
            mtime_q[63:32] <= merge(mtime_q[63:32], csr_req_i.wdata, csr_req_i.wstrb);
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Read data
  // ---------------------------------------------------------------------------
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    if (mswi_ok)         csr_rsp_o.rdata = {31'd0, msip_q[sw_hi]};
    else if (cmp_ok)     csr_rsp_o.rdata = cmp_hi_sel ? cmp_cur[63:32] : cmp_cur[31:0];
    else if (mtime_lo_h) csr_rsp_o.rdata = mtime_q[31:0];
    else if (mtime_hi_h) csr_rsp_o.rdata = (MTIME_SNAPSHOT && snap_v_q) ? snap_hi_q : mtime_q[63:32];
  end

  // ---------------------------------------------------------------------------
  // Outputs
  // ---------------------------------------------------------------------------
  logic [H-1:0] timer_w;
  always_comb begin
    for (int h = 0; h < H; h++) timer_w[h] = (mtime_q >= cmp_q[h]);
  end

  if (REG_OUT) begin : g_regout
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        timer_irq_o <= '0;
        sw_irq_o    <= '0;
      end else begin
        timer_irq_o <= timer_w;
        sw_irq_o    <= msip_q;
      end
    end
  end else begin : g_combout
    assign timer_irq_o = timer_w;
    assign sw_irq_o    = msip_q;
  end

  assign ssip_o  = ssip_q;
  assign mtime_o = mtime_q;

endmodule : clint_core
