// SPDX-License-Identifier: Apache-2.0
// plic_clint_top.sv - Parameterized Multi-Bus Wrapper (Wishbone B4 / AXI4 / TileLink / APB4) for plic_clint_core

module plic_clint_top
  import bus_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE = bus_pkg::BUS_AXI4,
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
  input  logic      clk_i,
  input  logic      rst_ni,

  // Selectable Bus Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t   wb_i,
  output wb_s2m_t   wb_o,
  input  axi4_m2s_t axi_i,
  output axi4_s2m_t axi_o,
  input  tl_m2s_t   tl_i,
  output tl_s2m_t   tl_o,
  input  apb_m2s_t  apb_i,
  output apb_s2m_t  apb_o,

  input  logic [NUM_SOURCES-1:0]  irq_src_i,    // [i] = PLIC source ID i+1
  output logic [NUM_CONTEXTS-1:0] ext_irq_o,    // PLIC target (context) interrupt outputs
  input  logic                    tick_i,       // mtime tick enable (USE_TICK_I = 1)
  input  logic                    halt_i,       // freeze mtime (debug halt)
  output logic [NUM_HARTS-1:0]    timer_irq_o,  // MTIP per hart
  output logic [NUM_HARTS-1:0]    sw_irq_o,     // MSIP per hart
  output logic [NUM_HARTS-1:0]    ssip_o,       // SSWI set pulse per hart
  output logic [63:0]             mtime_o       // time value (for `time` CSR / debug)
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign tl_o = '0;
    assign apb_o = '0;
    assign axi_o = '0;
    wire unused_bus_tl = &{1'b0, tl_i};
    wire unused_bus_apb = &{1'b0, apb_i};
    wire unused_bus_axi = &{1'b0, axi_i};
    plic_clint_wb #(
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
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .wb_i   (wb_i),
      .wb_o   (wb_o),
      .irq_src_i   (irq_src_i),
      .ext_irq_o   (ext_irq_o),
      .tick_i      (tick_i),
      .halt_i      (halt_i),
      .timer_irq_o (timer_irq_o),
      .sw_irq_o    (sw_irq_o),
      .ssip_o      (ssip_o),
      .mtime_o     (mtime_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o = '0;
    assign apb_o = '0;
    assign axi_o = '0;
    wire unused_bus_wb = &{1'b0, wb_i};
    wire unused_bus_apb = &{1'b0, apb_i};
    wire unused_bus_axi = &{1'b0, axi_i};
    plic_clint_tl #(
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
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .tl_i   (tl_i),
      .tl_o   (tl_o),
      .irq_src_i   (irq_src_i),
      .ext_irq_o   (ext_irq_o),
      .tick_i      (tick_i),
      .halt_i      (halt_i),
      .timer_irq_o (timer_irq_o),
      .sw_irq_o    (sw_irq_o),
      .ssip_o      (ssip_o),
      .mtime_o     (mtime_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o = '0;
    assign tl_o = '0;
    assign axi_o = '0;
    wire unused_bus_wb = &{1'b0, wb_i};
    wire unused_bus_tl = &{1'b0, tl_i};
    wire unused_bus_axi = &{1'b0, axi_i};
    plic_clint_apb #(
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
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .apb_i   (apb_i),
      .apb_o   (apb_o),
      .irq_src_i   (irq_src_i),
      .ext_irq_o   (ext_irq_o),
      .tick_i      (tick_i),
      .halt_i      (halt_i),
      .timer_irq_o (timer_irq_o),
      .sw_irq_o    (sw_irq_o),
      .ssip_o      (ssip_o),
      .mtime_o     (mtime_o)
    );
  end else begin : gen_axi
    assign wb_o = '0;
    assign tl_o = '0;
    assign apb_o = '0;
    wire unused_bus_wb = &{1'b0, wb_i};
    wire unused_bus_tl = &{1'b0, tl_i};
    wire unused_bus_apb = &{1'b0, apb_i};
    plic_clint_axi #(
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
    ) u_imp (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .axi_i   (axi_i),
      .axi_o   (axi_o),
      .irq_src_i   (irq_src_i),
      .ext_irq_o   (ext_irq_o),
      .tick_i      (tick_i),
      .halt_i      (halt_i),
      .timer_irq_o (timer_irq_o),
      .sw_irq_o    (sw_irq_o),
      .ssip_o      (ssip_o),
      .mtime_o     (mtime_o)
    );
  end

endmodule : plic_clint_top
