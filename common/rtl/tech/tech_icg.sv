// SPDX-License-Identifier: Apache-2.0
// tech_icg.sv - Integrated Clock Gating (ICG) Cell Wrapper
//
// Glitch-free negative-level latch-based behavioral default for Yosys, Verilator,
// and generic FPGA flows, with clean swap hooks for ASIC foundry cells
// (`TARGET_ASIC` + `TECH_ASIC_ICG_*`) and Xilinx `BUFGCE` (`TARGET_FPGA_XILINX`).

module tech_icg (
  input  logic clk_i,
  input  logic en_i,
  input  logic test_en_i = 1'b0,
  output logic gclk_o
);

`ifdef TARGET_ASIC
  `ifdef TECH_ASIC_ICG_SKY130
  sky130_fd_sc_hd__sdlclkp_1 u_icg (
    .CLK  (clk_i),
    .GATE (en_i),
    .SCE  (test_en_i),
    .GCLK (gclk_o)
  );
  `elsif TECH_ASIC_ICG_GF180
  gf180mcu_fd_sc_mcu7t5v0__icgtp_1 u_icg (
    .CLK (clk_i),
    .E   (en_i),
    .TE  (test_en_i),
    .Q   (gclk_o)
  );
  `else
  // Generic ASIC foundry ICG hook: replace with target standard-cell ICG
  // (e.g. CKLMX / PREICG) during synthesis technology mapping.
  (* keep = "true" *) logic en_lat_q;
  always_latch begin
    if (!clk_i) begin
      en_lat_q = en_i | test_en_i;
    end
  end
  assign gclk_o = clk_i & en_lat_q;
  `endif
`elsif TARGET_FPGA_XILINX
  BUFGCE u_bufgce (
    .I  (clk_i),
    .CE (en_i | test_en_i),
    .O  (gclk_o)
  );
`else
  // Behavioral glitch-free latch-based ICG (Yosys / Verilator / generic FPGA).
  // Enable is captured only while clk_i is low, holding gclk_o glitch-free
  // across the entire high phase of clk_i.
  (* keep = "true" *) logic en_lat_q;

  always_latch begin
    if (!clk_i) begin
      en_lat_q = en_i | test_en_i;
    end
  end

  assign gclk_o = clk_i & en_lat_q;
`endif

endmodule : tech_icg
