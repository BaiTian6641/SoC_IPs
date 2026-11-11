// SPDX-License-Identifier: Apache-2.0
// gpio_core.sv - Parameterized Multi-Function GPIO Controller with Debounce & 4-Mode IRQ
//
// Register Map:
//   0x00 DATA_IN    : (RO) Synchronized & debounced pin input value
//   0x04 DATA_OUT   : (RW) Output data register
//   0x08 DIR        : (RW) Output enable direction (1=Output, 0=Input)
//   0x0C OUT_SET    : (WO) Atomic bit-set on DATA_OUT
//   0x10 OUT_CLR    : (WO) Atomic bit-clear on DATA_OUT
//   0x14 OUT_TGL    : (WO) Atomic bit-toggle on DATA_OUT
//   0x18 OPEN_DRAIN : (RW) Per-pin open-drain enable (drives OE only when DATA_OUT=0)
//   0x1C PULL_EN    : (RW) Pad pull resistor enable
//   0x20 PULL_SEL   : (RW) Pad pull direction (1=Pull-Up, 0=Pull-Down)
//   0x24 INT_EN     : (RW) Per-pin interrupt enable
//   0x28 INT_TYPE   : (RW) 1=Edge-triggered, 0=Level-triggered
//   0x2C INT_POL    : (RW) 1=Rising/High, 0=Falling/Low
//   0x30 INT_BOTH   : (RW) 1=Both rising and falling edges (overrides INT_POL when INT_TYPE=1)
//   0x34 INT_STAT   : (W1C) Sticky edge/level interrupt status
//   0x38 DEBOUNCE   : [15:0]=debounce filter sample period cycles, [16]=global debounce enable

module gpio_core
  import bus_pkg::*;
#(
  parameter int unsigned NUM_PINS = 32
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  input  csr_req_t            csr_req_i,
  output csr_rsp_t            csr_rsp_o,

  input  logic [NUM_PINS-1:0] gpio_i,
  output logic [NUM_PINS-1:0] gpio_o,
  output logic [NUM_PINS-1:0] gpio_oe_o,
  output logic [NUM_PINS-1:0] gpio_pull_en_o,
  output logic [NUM_PINS-1:0] gpio_pull_up_o,

  output logic                irq_o
);

  logic [NUM_PINS-1:0] out_q, dir_q, od_q, pull_en_q, pull_up_q;
  logic [NUM_PINS-1:0] int_en_q, int_type_q, int_pol_q, int_both_q, int_stat_q;
  logic [16:0]         dbnc_cfg_q;

  logic [NUM_PINS-1:0] sync_pin;
  edge_detect #(.WIDTH(NUM_PINS), .INIT('0)) u_pin_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(gpio_i), .sync_o(sync_pin), .rise_o(), .fall_o()
  );

  // Programmable Debounce Filter
  logic [15:0]         dbnc_cnt_q;
  logic [NUM_PINS-1:0] dbnc_stage_q, clean_pin_q, prev_pin_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dbnc_cnt_q   <= '0;
      dbnc_stage_q <= '0;
      clean_pin_q  <= '0;
      prev_pin_q   <= '0;
    end else begin
      prev_pin_q <= clean_pin_q;
      if (!dbnc_cfg_q[16]) begin
        clean_pin_q <= sync_pin;
      end else if (dbnc_cnt_q >= dbnc_cfg_q[15:0]) begin
        dbnc_cnt_q   <= '0;
        dbnc_stage_q <= sync_pin;
        for (int i = 0; i < NUM_PINS; i++) begin
          if (sync_pin[i] == dbnc_stage_q[i]) clean_pin_q[i] <= sync_pin[i];
        end
      end else begin
        dbnc_cnt_q <= dbnc_cnt_q + 16'd1;
      end
    end
  end

  wire [NUM_PINS-1:0] pin_rise = clean_pin_q & ~prev_pin_q;
  wire [NUM_PINS-1:0] pin_fall = ~clean_pin_q & prev_pin_q;

  // Output & Open-Drain Logic
  always_comb begin
    for (int i = 0; i < NUM_PINS; i++) begin
      gpio_o[i]    = od_q[i] ? 1'b0 : out_q[i];
      gpio_oe_o[i] = dir_q[i] && (od_q[i] ? !out_q[i] : 1'b1);
    end
  end

  assign gpio_pull_en_o = pull_en_q;
  assign gpio_pull_up_o = pull_up_q;

  wire [7:0] reg_off = csr_req_i.addr[7:0];

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = 32'(clean_pin_q);
      8'h04: csr_rsp_o.rdata = 32'(out_q);
      8'h08: csr_rsp_o.rdata = 32'(dir_q);
      8'h18: csr_rsp_o.rdata = 32'(od_q);
      8'h1C: csr_rsp_o.rdata = 32'(pull_en_q);
      8'h20: csr_rsp_o.rdata = 32'(pull_up_q);
      8'h24: csr_rsp_o.rdata = 32'(int_en_q);
      8'h28: csr_rsp_o.rdata = 32'(int_type_q);
      8'h2C: csr_rsp_o.rdata = 32'(int_pol_q);
      8'h30: csr_rsp_o.rdata = 32'(int_both_q);
      8'h34: csr_rsp_o.rdata = 32'(int_stat_q);
      8'h38: csr_rsp_o.rdata = {15'd0, dbnc_cfg_q};
      8'h0C, 8'h10, 8'h14: csr_rsp_o.rdata = '0;
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      out_q      <= '0;
      dir_q      <= '0;
      od_q       <= '0;
      pull_en_q  <= '0;
      pull_up_q  <= '0;
      int_en_q   <= '0;
      int_type_q <= '0;
      int_pol_q  <= '0;
      int_both_q <= '0;
      int_stat_q <= '0;
      dbnc_cfg_q <= '0;
    end else begin
      // Update per-pin interrupts
      for (int i = 0; i < NUM_PINS; i++) begin
        logic trig;
        if (int_type_q[i]) begin
          trig = int_both_q[i] ? (pin_rise[i] | pin_fall[i])
                               : (int_pol_q[i] ? pin_rise[i] : pin_fall[i]);
        end else begin
          trig = int_pol_q[i] ? clean_pin_q[i] : !clean_pin_q[i];
        end
        if (trig) int_stat_q[i] <= 1'b1;
      end

      if (csr_req_i.valid && csr_req_i.write) begin
        logic [31:0] w;
        w = csr_req_i.wdata;
        unique case (reg_off)
          8'h04: out_q      <= NUM_PINS'(ip_common_pkg::apply_wstrb(32'(out_q), w, csr_req_i.wstrb));
          8'h08: dir_q      <= NUM_PINS'(ip_common_pkg::apply_wstrb(32'(dir_q), w, csr_req_i.wstrb));
          8'h0C: out_q      <= out_q | NUM_PINS'(w);
          8'h10: out_q      <= out_q & ~NUM_PINS'(w);
          8'h14: out_q      <= out_q ^ NUM_PINS'(w);
          8'h18: od_q       <= NUM_PINS'(w);
          8'h1C: pull_en_q  <= NUM_PINS'(w);
          8'h20: pull_up_q  <= NUM_PINS'(w);
          8'h24: int_en_q   <= NUM_PINS'(w);
          8'h28: int_type_q <= NUM_PINS'(w);
          8'h2C: int_pol_q  <= NUM_PINS'(w);
          8'h30: int_both_q <= NUM_PINS'(w);
          8'h34: int_stat_q <= int_stat_q & ~NUM_PINS'(w);
          8'h38: dbnc_cfg_q <= w[16:0];
          default: ;
        endcase
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : gpio_core
