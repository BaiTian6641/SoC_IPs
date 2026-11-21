// SPDX-License-Identifier: Apache-2.0
// sdmmc_cd_filter.sv - 2-FF synchroniser + debounce filter for card-detect / write-protect pins.
//
// The input must remain stable for (deb_i + 1) prescaler ticks (1 tick = 256 core clocks)
// before the output follows.  val_o is the debounced, polarity-corrected value
// (1 = asserted: card present / write-protected).

module sdmmc_cd_filter #(
  parameter bit ACTIVE_LOW = 1'b1
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic [15:0] deb_i,
  input  logic        pin_i,
  output logic        val_o,
  output logic        raw_o
);

  logic        sync1_q, sync2_q;
  logic [7:0]  pre_q;
  logic [15:0] cnt_q;

  wire asserted = ACTIVE_LOW ? !sync2_q : sync2_q;
  wire tick     = (pre_q == 8'hFF);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sync1_q <= !ACTIVE_LOW;
      sync2_q <= !ACTIVE_LOW;
      pre_q   <= '0;
      cnt_q   <= '0;
      val_o   <= 1'b0;
    end else begin
      sync1_q <= pin_i;
      sync2_q <= sync1_q;
      pre_q   <= pre_q + 8'd1;
      if (asserted == val_o) begin
        cnt_q <= '0;
      end else if (tick) begin
        if (cnt_q >= deb_i) begin
          val_o <= asserted;
          cnt_q <= '0;
        end else begin
          cnt_q <= cnt_q + 16'd1;
        end
      end
    end
  end

  assign raw_o = asserted;

endmodule : sdmmc_cd_filter
