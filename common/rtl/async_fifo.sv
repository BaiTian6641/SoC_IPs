// SPDX-License-Identifier: Apache-2.0
// async_fifo.sv - Dual-Clock Gray-Coded Asynchronous CDC FIFO (Power-of-2 Depth)

// Reset: wrst_ni / rrst_ni are independent asynchronous resets.  They MUST be asserted
// together (or the FIFO flushed by asserting both) - resetting only one side leaves the
// pointers inconsistent.  Each reset should be de-asserted synchronously in its own clock
// domain (reset synchroniser outside this module).  wfull_o is 0 / rempty_o is 1 in reset.
module async_fifo #(
  parameter int unsigned DATA_W = 32,
  parameter int unsigned ADDR_W = 4    // Depth = 2**ADDR_W, ADDR_W >= 1
) (
  // Write Domain
  input  logic              wclk_i,
  input  logic              wrst_ni,
  input  logic              wpush_i,
  input  logic [DATA_W-1:0] wdata_i,
  output logic              wfull_o,

  // Read Domain
  input  logic              rclk_i,
  input  logic              rrst_ni,
  input  logic              rpop_i,
  output logic [DATA_W-1:0] rdata_o,
  output logic              rempty_o
);

  localparam int unsigned DEPTH = (1 << ADDR_W);

  localparam logic [ADDR_W:0] GRAY_FULL_MASK = {2'b11, {(ADDR_W-1){1'b0}}};

  logic [DATA_W-1:0] mem [0:DEPTH-1];

  logic [ADDR_W:0] wbin_q, wbin_d;
  logic [ADDR_W:0] wgray_q, wgray_d;
  logic [ADDR_W:0] rgray_wsync1_q, rgray_wsync2_q;

  logic [ADDR_W:0] rbin_q, rbin_d;
  logic [ADDR_W:0] rgray_q, rgray_d;
  logic [ADDR_W:0] wgray_rsync1_q, wgray_rsync2_q;

  // Write pointer & full logic
  assign wbin_d  = wbin_q + (ADDR_W+1)'((wpush_i && !wfull_o) ? 1'b1 : 1'b0);
  assign wgray_d = (wbin_d >> 1) ^ wbin_d;

  always_ff @(posedge wclk_i or negedge wrst_ni) begin
    if (!wrst_ni) begin
      wbin_q         <= '0;
      wgray_q        <= '0;
      wfull_o        <= 1'b0;
      rgray_wsync1_q <= '0;
      rgray_wsync2_q <= '0;
    end else begin
      wbin_q         <= wbin_d;
      wgray_q        <= wgray_d;
      rgray_wsync1_q <= rgray_q;
      rgray_wsync2_q <= rgray_wsync1_q;
      // full when the next write gray pointer equals the read gray pointer with its two MSBs inverted
      wfull_o        <= (wgray_d == (rgray_wsync2_q ^ GRAY_FULL_MASK));
    end
  end

  // Memory write port: plain clocked process (no reset) so that FPGA block-RAM / ASIC SRAM
  // macros can be inferred; the pointers are what is reset.
  always_ff @(posedge wclk_i) begin
    if (wpush_i && !wfull_o) begin
      mem[wbin_q[ADDR_W-1:0]] <= wdata_i;
    end
  end

  // Read pointer & empty logic
  assign rbin_d  = rbin_q + (ADDR_W+1)'((rpop_i && !rempty_o) ? 1'b1 : 1'b0);
  assign rgray_d = (rbin_d >> 1) ^ rbin_d;
  assign rdata_o = mem[rbin_q[ADDR_W-1:0]];

  always_ff @(posedge rclk_i or negedge rrst_ni) begin
    if (!rrst_ni) begin
      rbin_q         <= '0;
      rgray_q        <= '0;
      rempty_o       <= 1'b1;
      wgray_rsync1_q <= '0;
      wgray_rsync2_q <= '0;
    end else begin
      rbin_q         <= rbin_d;
      rgray_q        <= rgray_d;
      wgray_rsync1_q <= wgray_q;
      wgray_rsync2_q <= wgray_rsync1_q;
      rempty_o       <= (rgray_d == wgray_rsync2_q);
    end
  end

endmodule : async_fifo
