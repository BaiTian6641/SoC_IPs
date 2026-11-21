// SPDX-License-Identifier: Apache-2.0
// tb_pio_examples.sv - runs every example program (ips/23_pio_engine/examples/*.hex, produced by tools/pioasm.py)
// on pio_core and checks pin-level waveforms / FIFO data against independent golden models.
//
// Harness conventions (shared by all ex_*.svh files):
//   * clock: 100 MHz style, `clk`, posedge at t=5,15,...  TB drives inputs at posedge+#1.
//   * `cyc` = number of posedges so far, read after the post-edge #1.  A CSR write issued by csr_wr takes
//     effect at edge number `t_csr` (valid after csr_wr returns).  With clkdiv 1.0, the SM executes its first
//     instruction at edge t_csr+1 when written via SM_START (or SM_EN after restart).
//   * History: h_bus/h_o/h_oe[E - h_first] hold the resolved pad value / DUT pad outputs *during the cycle after
//     edge E* (i.e. an instruction executing at edge E that changes a pin shows up in index E).
//   * External drive: pad_drv/pad_drv_en (pad_pull_hi for pull-ups).  bus = DUT drives where pio_oe, else
//     external, else pull value.  Inputs pass the DUT's 2-FF synchroniser: a pad change made right after edge k is
//     seen by an instruction executing at edge k+3 (k+1 for pins in SYNC_BYPASS).

`timescale 1ns/1ps

module tb_pio_examples;
  import bus_pkg::*;
  import pio_tb_pkg::*;

  localparam int NUM_SM = 4;
  localparam int PINS   = 32;
  localparam int DEPTH  = 128;

  logic clk;
  logic rst_ni;
  initial begin clk = 1'b0; rst_ni = 1'b0; end
  always #5 clk = ~clk;

  csr_req_t csr_req;
  csr_rsp_t csr_rsp;

  logic [PINS-1:0] pio_i;
  logic [PINS-1:0] pio_o, pio_oe, pio_pull_en, pio_pull_up;
  logic [NUM_SM-1:0] dma_tx_req, dma_rx_req;
  logic [1:0] irq;

  pio_core #(.NUM_SM(NUM_SM), .PIO_PINS(PINS), .IMEM_DEPTH(DEPTH)) dut (
    .clk_i(clk), .rst_ni(rst_ni), .csr_req_i(csr_req), .csr_rsp_o(csr_rsp),
    .pio_i(pio_i), .pio_o(pio_o), .pio_oe_o(pio_oe), .pio_pull_en_o(pio_pull_en), .pio_pull_up_o(pio_pull_up),
    .dma_tx_req_o(dma_tx_req), .dma_rx_req_o(dma_rx_req), .irq_o(irq)
  );

  // ---------------------------------------------------------------- external pad model
  logic [PINS-1:0] pad_drv, pad_drv_en, pad_pull_hi;
  initial begin pad_drv = '0; pad_drv_en = '0; pad_pull_hi = '0; end
  wire  [PINS-1:0] bus = (pio_oe & pio_o) | (~pio_oe & ((pad_drv_en & pad_drv) | (~pad_drv_en & pad_pull_hi)));
  assign pio_i = bus;

  // ---------------------------------------------------------------- cycle counter + history
  longint unsigned cyc;
  longint unsigned t_csr;
  longint unsigned h_first;
  initial begin cyc = 0; t_csr = 0; h_first = 0; end
  logic [PINS-1:0] h_bus [$];
  logic [PINS-1:0] h_o   [$];
  logic [PINS-1:0] h_oe  [$];
  bit rec_en;
  initial rec_en = 1'b0;

  always @(posedge clk) begin
    if (rec_en) begin
      h_bus.push_back(bus);
      h_o.push_back(pio_o);
      h_oe.push_back(pio_oe);
    end
    cyc <= cyc + 1;
  end

  task automatic hist_start();
    h_bus.delete(); h_o.delete(); h_oe.delete();
    // the next posedge samples index `cyc` (value before the increment of that edge)
    h_first = cyc;
    rec_en = 1'b1;
  endtask

  function automatic logic [PINS-1:0] hbus(input longint unsigned e);
    int idx;
    idx = int'(e - h_first);
    return h_bus[idx];
  endfunction
  function automatic logic [PINS-1:0] ho(input longint unsigned e);
    int idx;
    idx = int'(e - h_first);
    return h_o[idx];
  endfunction
  function automatic logic [PINS-1:0] hoe(input longint unsigned e);
    int idx;
    idx = int'(e - h_first);
    return h_oe[idx];
  endfunction
  function automatic longint unsigned hlen();
    return longint'(h_bus.size());
  endfunction

  // ---------------------------------------------------------------- CSR access (single cycle, ready is always 1)
  task automatic csr_wr(input int addr, input logic [31:0] data);
    @(posedge clk); #1;
    csr_req = '{valid: 1'b1, write: 1'b1, addr: 32'(addr), wdata: data, wstrb: 4'hF};
    @(posedge clk); #1;
    t_csr = cyc;
    csr_req = '0;
  endtask

  task automatic csr_rd(input int addr, output logic [31:0] data);
    @(posedge clk); #1;
    csr_req = '{valid: 1'b1, write: 1'b0, addr: 32'(addr), wdata: '0, wstrb: 4'h0};
    @(posedge clk);
    data = csr_rsp.rdata;     // sampled pre-edge == handshake cycle
    #1;
    csr_req = '0;
  endtask

  task automatic wait_cyc(input int n);
    repeat (n) @(posedge clk);
    #1;
  endtask

  task automatic chk(input bit cond, input string msg);
    if (!cond) begin
      $display("[FAIL] t=%0t cyc=%0d: %s", $time, cyc, msg);
      $fatal(1, "[FAIL] tb_pio_examples: %s", msg);
    end
  endtask

  // ---------------------------------------------------------------- DUT bring-up helpers
  task automatic tb_reset();
    rec_en = 1'b0;
    pad_drv = '0; pad_drv_en = '0; pad_pull_hi = '0;
    csr_req = '0;
    rst_ni = 1'b0;
    repeat (4) @(posedge clk);
    #1 rst_ni = 1'b1;
    repeat (2) @(posedge clk);
    #1;
  endtask

  logic [31:0] pm [0:255];
  // Load a $readmemh file produced by pioasm into instruction RAM (honours @addr lines).
  // Use bit 31 as an unwritten sentinel so 2-state simulators (Verilator) do not collapse 'x to 0.
  task automatic load_prog(input string path);
    for (int i = 0; i < 256; i++) pm[i] = 32'hFFFF_FFFF;
    $readmemh(path, pm);
    for (int i = 0; i < DEPTH; i++) begin
      if (pm[i][31] == 1'b0) csr_wr(IMEM_BASE + 4 * i, pm[i]);
    end
  endtask

  // Convenience: program one SM's static configuration.
  task automatic cfg_sm(input int sm, input int div_int, input int div_frac,
                        input int wrap_bot, input int wrap_top, input int entry,
                        input logic [31:0] shiftctrl, input logic [31:0] pinctrl, input logic [31:0] inctrl);
    csr_wr(sm_base(sm) + S_CLKDIV, fn_clkdiv(div_int, div_frac));
    csr_wr(sm_base(sm) + S_WRAP, fn_wrap(wrap_bot, wrap_top, entry));
    csr_wr(sm_base(sm) + S_SHIFT, shiftctrl);
    csr_wr(sm_base(sm) + S_PINCTRL, pinctrl);
    csr_wr(sm_base(sm) + S_INCTRL, inctrl);
  endtask

  // Inject one instruction (24-bit word) and let it execute (SM may be disabled).
  task automatic sm_exec(input int sm, input logic [23:0] instr);
    csr_wr(sm_base(sm) + S_INSTR, {8'd0, instr});
    wait_cyc(3);
  endtask

  int n_examples_passed;
  initial n_examples_passed = 0;

  // ---------------------------------------------------------------- example tasks (one file per owner)
`include "ips/23_pio_engine/tb/ex_serial.svh"      // ex_uart_tx, ex_uart_rx, ex_spi_master, ex_i2s_tx
`include "ips/23_pio_engine/tb/ex_misc.svh"        // ex_ws2812, ex_quadrature, ex_bus8080, ex_pwm_meas
`include "ips/23_pio_engine/tb/ex_parallel.svh"    // ex_dvp_capture, ex_hub75, ex_sd4

  // ---------------------------------------------------------------- main
  initial begin
    csr_req = '0;
    tb_reset();
    ex_uart_tx();     n_examples_passed++;
    ex_uart_rx();     n_examples_passed++;
    ex_spi_master();  n_examples_passed++;
    ex_i2s_tx();      n_examples_passed++;
    ex_ws2812();      n_examples_passed++;
    ex_quadrature();  n_examples_passed++;
    ex_bus8080();     n_examples_passed++;
    ex_pwm_meas();    n_examples_passed++;
    ex_dvp_capture(); n_examples_passed++;
    ex_hub75();       n_examples_passed++;
    ex_sd4();         n_examples_passed++;
    $display("[PASS] tb_pio_examples: %0d example programs matched their golden models cycle-exactly", n_examples_passed);
    $finish;
  end

  initial begin
    #(500_000_000);
    $fatal(1, "[TIMEOUT] tb_pio_examples: watchdog expired");
  end

endmodule : tb_pio_examples
