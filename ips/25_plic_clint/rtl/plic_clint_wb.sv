// SPDX-License-Identifier: Apache-2.0
// plic_clint_wb.sv - Wishbone B4 Slave Wrapper for plic_clint_core

module plic_clint_wb
  import bus_pkg::*;
#(
  parameter bit                        ENABLE_PLIC     = 1'b1,
  parameter bit                        ENABLE_CLINT    = 1'b1,
  parameter int unsigned               NUM_SOURCES     = 32,
  parameter int unsigned               NUM_CONTEXTS    = 2,
  parameter int unsigned               PRIO_BITS       = 3,
  parameter logic [NUM_SOURCES-1:0]    EDGE_MASK       = '0,
  parameter int unsigned               SYNC_STAGES     = 2,
  parameter int unsigned               NUM_HARTS       = 1,
  parameter bit                        ACLINT_SPLIT    = 1'b0,
  parameter bit                        ENABLE_SSWI     = 1'b0,
  parameter bit                        MTIME_SNAPSHOT  = 1'b1,
  parameter bit                        ATOMIC_WRITE    = 1'b0,
  parameter bit                        USE_TICK_I      = 1'b0,
  parameter int unsigned               PRESCALE        = 1,
  parameter bit                        REG_OUT         = 1'b0,
  parameter logic [31:0]               CLINT_BASE      = 32'h0200_0000,
  parameter logic [31:0]               PLIC_BASE       = 32'h0C00_0000,
  parameter logic [31:0]               MSWI_BASE       = 32'h0200_0000,
  parameter logic [31:0]               MTIMER_BASE     = 32'h0201_0000,
  parameter logic [31:0]               SSWI_BASE       = 32'h0202_0000
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  wb_m2s_t wb_i,
  output wb_s2m_t wb_o,
  input  logic [NUM_SOURCES-1:0]  irq_src_i,    // [i] = PLIC source ID i+1
  output logic [NUM_CONTEXTS-1:0] ext_irq_o,    // PLIC target (context) interrupt outputs
  input  logic                    tick_i,       // mtime tick enable (USE_TICK_I = 1)
  input  logic                    halt_i,       // freeze mtime (debug halt)
  output logic [NUM_HARTS-1:0]    timer_irq_o,  // MTIP per hart
  output logic [NUM_HARTS-1:0]    sw_irq_o,     // MSIP per hart
  output logic [NUM_HARTS-1:0]    ssip_o,       // SSWI set pulse per hart
  output logic [63:0]             mtime_o       // time value (for `time` CSR / debug)
);

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  wb_to_csr u_wb_adapter (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .wb_i      (wb_i),
    .wb_o      (wb_o),
    .csr_req_o (csr_req),
    .csr_rsp_i (csr_rsp)
  );

  plic_clint_core #(
    .ENABLE_PLIC (ENABLE_PLIC),
    .ENABLE_CLINT (ENABLE_CLINT),
    .NUM_SOURCES (NUM_SOURCES),
    .NUM_CONTEXTS (NUM_CONTEXTS),
    .PRIO_BITS (PRIO_BITS),
    .EDGE_MASK (EDGE_MASK),
    .SYNC_STAGES (SYNC_STAGES),
    .NUM_HARTS (NUM_HARTS),
    .ACLINT_SPLIT (ACLINT_SPLIT),
    .ENABLE_SSWI (ENABLE_SSWI),
    .MTIME_SNAPSHOT (MTIME_SNAPSHOT),
    .ATOMIC_WRITE (ATOMIC_WRITE),
    .USE_TICK_I (USE_TICK_I),
    .PRESCALE (PRESCALE),
    .REG_OUT (REG_OUT),
    .CLINT_BASE (CLINT_BASE),
    .PLIC_BASE (PLIC_BASE),
    .MSWI_BASE (MSWI_BASE),
    .MTIMER_BASE (MTIMER_BASE),
    .SSWI_BASE (SSWI_BASE)
  ) u_core (
    .clk_i     (clk_i),
    .rst_ni    (rst_ni),
    .csr_req_i (csr_req),
    .csr_rsp_o (csr_rsp),
    .irq_src_i   (irq_src_i),
    .ext_irq_o   (ext_irq_o),
    .tick_i      (tick_i),
    .halt_i      (halt_i),
    .timer_irq_o (timer_irq_o),
    .sw_irq_o    (sw_irq_o),
    .ssip_o      (ssip_o),
    .mtime_o     (mtime_o)
  );

endmodule : plic_clint_wb
