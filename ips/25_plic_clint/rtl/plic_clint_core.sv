// SPDX-License-Identifier: Apache-2.0
// plic_clint_core.sv - PLIC + CLINT/ACLINT in one CSR slave (standard RISC-V platform memory map)
//
// Default map (matches common RISC-V SoC / QEMU-virt style layouts, so unmodified software and
// device trees work):
//   CLINT_BASE = 0x0200_0000 : msip[h] +0x0000+4h, mtimecmp[h] +0x4000+8h, mtime +0xBFF8
//   PLIC_BASE  = 0x0C00_0000 : priority/pending/enable/threshold/claim (see plic_core)
// ACLINT_SPLIT = 1 relocates MSWI / MTIMER / SSWI to independent bases (MSWI_BASE, MTIMER_BASE,
// SSWI_BASE; the offsets inside each device are identical to the legacy CLINT layout).
// A region that is disabled (ENABLE_PLIC/ENABLE_CLINT = 0) returns a bus error like any unmapped
// address.  The full 32-bit address is decoded: if the interconnect strips the base address, use
// plic_core / clint_core directly (they decode offsets only).

module plic_clint_core
  import bus_pkg::*;
#(
  parameter bit                     ENABLE_PLIC    = 1'b1,
  parameter bit                     ENABLE_CLINT   = 1'b1,
  // PLIC
  parameter int unsigned            NUM_SOURCES    = 32,
  parameter int unsigned            NUM_CONTEXTS   = 2,
  parameter int unsigned            PRIO_BITS      = 3,
  parameter logic [NUM_SOURCES-1:0] EDGE_MASK      = '0,
  parameter int unsigned            SYNC_STAGES    = 2,
  // CLINT / ACLINT
  parameter int unsigned            NUM_HARTS      = 1,
  parameter bit                     ACLINT_SPLIT   = 1'b0,
  parameter bit                     ENABLE_SSWI    = 1'b0,   // only meaningful when ACLINT_SPLIT = 1
  parameter bit                     MTIME_SNAPSHOT = 1'b1,
  parameter bit                     ATOMIC_WRITE   = 1'b0,
  parameter bit                     USE_TICK_I     = 1'b0,
  parameter int unsigned            PRESCALE       = 1,
  parameter bit                     REG_OUT        = 1'b0,
  // Address map
  parameter logic [31:0]            CLINT_BASE     = 32'h0200_0000,
  parameter logic [31:0]            PLIC_BASE      = 32'h0C00_0000,
  parameter logic [31:0]            MSWI_BASE      = 32'h0200_0000,
  parameter logic [31:0]            MTIMER_BASE    = 32'h0201_0000,
  parameter logic [31:0]            SSWI_BASE      = 32'h0202_0000
) (
  input  logic                        clk_i,
  input  logic                        rst_ni,

  input  csr_req_t                    csr_req_i,
  output csr_rsp_t                    csr_rsp_o,

  input  logic [NUM_SOURCES-1:0]      irq_src_i,       // [i] = PLIC source ID i+1
  output logic [NUM_CONTEXTS-1:0]     ext_irq_o,       // per context (e.g. ctx 2h = M-mode hart h, 2h+1 = S-mode)

  input  logic                        tick_i,
  input  logic                        halt_i,
  output logic [NUM_HARTS-1:0]        timer_irq_o,
  output logic [NUM_HARTS-1:0]        sw_irq_o,
  output logic [NUM_HARTS-1:0]        ssip_o,
  output logic [63:0]                 mtime_o
);

  wire [31:0] a = csr_req_i.addr;

  // ---------------------------------------------------------------------------
  // Region decode
  // ---------------------------------------------------------------------------
  logic        plic_sel, mswi_sel, mtimer_sel, sswi_sel, clint_res;
  logic [15:0] c_off;

  always_comb begin
    plic_sel   = ENABLE_PLIC && ((a & 32'hFC00_0000) == PLIC_BASE);
    mswi_sel   = 1'b0;
    mtimer_sel = 1'b0;
    sswi_sel   = 1'b0;
    clint_res  = 1'b0;
    c_off      = a[15:0];
    if (ENABLE_CLINT) begin
      if (!ACLINT_SPLIT) begin
        if ((a & 32'hFFFF_0000) == CLINT_BASE) begin
          clint_res = 1'b1;
          if (a[15:14] == 2'b00) begin
            mswi_sel = 1'b1;
            c_off    = {2'b00, a[13:0]};
          end else if (a[15:14] != 2'b11) begin
            mtimer_sel = 1'b1;
            c_off      = a[15:0] - 16'h4000;
          end
        end
      end else begin
        if ((a & 32'hFFFF_C000) == MSWI_BASE) begin
          mswi_sel = 1'b1;
          c_off    = {2'b00, a[13:0]};
        end else if ((a & 32'hFFFF_8000) == MTIMER_BASE) begin
          mtimer_sel = 1'b1;
          c_off      = {1'b0, a[14:0]};
        end else if (ENABLE_SSWI && ((a & 32'hFFFF_C000) == SSWI_BASE)) begin
          sswi_sel = 1'b1;
          c_off    = {2'b00, a[13:0]};
        end
        clint_res = mswi_sel | mtimer_sel | sswi_sel;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Sub-blocks
  // ---------------------------------------------------------------------------
  csr_req_t plic_req, clint_req;
  csr_rsp_t plic_rsp, clint_rsp;

  always_comb begin
    plic_req        = csr_req_i;
    plic_req.valid  = csr_req_i.valid && plic_sel;
    clint_req       = csr_req_i;
    clint_req.valid = csr_req_i.valid && clint_res;
    clint_req.addr  = {16'd0, c_off};
  end

  if (ENABLE_PLIC) begin : g_plic
    plic_core #(
      .NUM_SOURCES (NUM_SOURCES), .NUM_CONTEXTS (NUM_CONTEXTS), .PRIO_BITS (PRIO_BITS),
      .EDGE_MASK (EDGE_MASK), .SYNC_STAGES (SYNC_STAGES), .REG_OUT (REG_OUT)
    ) u_plic (
      .clk_i (clk_i), .rst_ni (rst_ni),
      .csr_req_i (plic_req), .csr_rsp_o (plic_rsp),
      .irq_i (irq_src_i), .ext_irq_o (ext_irq_o)
    );
  end else begin : g_noplic
    assign plic_rsp  = '0;
    assign ext_irq_o = '0;
  end

  if (ENABLE_CLINT) begin : g_clint
    clint_core #(
      .NUM_HARTS (NUM_HARTS), .MTIME_SNAPSHOT (MTIME_SNAPSHOT), .ATOMIC_WRITE (ATOMIC_WRITE),
      .USE_TICK_I (USE_TICK_I), .PRESCALE (PRESCALE),
      .ENABLE_SSWI (ENABLE_SSWI && ACLINT_SPLIT), .REG_OUT (REG_OUT)
    ) u_clint (
      .clk_i (clk_i), .rst_ni (rst_ni),
      .csr_req_i (clint_req),
      .sel_mswi_i (mswi_sel), .sel_mtimer_i (mtimer_sel), .sel_sswi_i (sswi_sel),
      .csr_rsp_o (clint_rsp),
      .tick_i (tick_i), .halt_i (halt_i),
      .timer_irq_o (timer_irq_o), .sw_irq_o (sw_irq_o), .ssip_o (ssip_o), .mtime_o (mtime_o)
    );
  end else begin : g_noclint
    assign clint_rsp   = '0;
    assign timer_irq_o = '0;
    assign sw_irq_o    = '0;
    assign ssip_o      = '0;
    assign mtime_o     = '0;
  end

  // ---------------------------------------------------------------------------
  // Response mux (unmapped -> error)
  // ---------------------------------------------------------------------------
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    if (plic_sel)       csr_rsp_o.rdata = plic_rsp.rdata;
    else if (clint_res) csr_rsp_o.rdata = clint_rsp.rdata;
    else                csr_rsp_o.err   = csr_req_i.valid;
  end

endmodule : plic_clint_core
