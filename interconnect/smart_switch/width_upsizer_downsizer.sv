// SPDX-License-Identifier: Apache-2.0
// width_upsizer_downsizer.sv - Parameterized 64-bit <-> 32-bit CSR/Memory Width Converter
// Splits a 64-bit initiator beat into 1 or 2 32-bit target beats (or steers a 32-bit lane).

module width_upsizer_downsizer (
  input  logic        clk_i,
  input  logic        rst_ni,

  // 64-bit Wide Initiator Side
  input  logic        wide_valid_i,
  output logic        wide_ready_o,
  input  logic        wide_write_i,
  input  logic [31:0] wide_addr_i,
  input  logic [63:0] wide_wdata_i,
  input  logic [7:0]  wide_wstrb_i,
  output logic [63:0] wide_rdata_o,
  output logic        wide_err_o,

  // 32-bit Narrow Target Side
  output logic        narrow_valid_o,
  input  logic        narrow_ready_i,
  output logic        narrow_write_o,
  output logic [31:0] narrow_addr_o,
  output logic [31:0] narrow_wdata_o,
  output logic [3:0]  narrow_wstrb_o,
  input  logic [31:0] narrow_rdata_i,
  input  logic        narrow_err_i
);

  typedef enum logic [1:0] {
    ST_LOW_WORD,
    ST_HIGH_WORD
  } state_e;

  state_e      state_q;
  logic [31:0] low_rdata_q;
  logic        low_err_q;

  wire need_low  = !wide_write_i || (|wide_wstrb_i[3:0]);
  wire need_high = !wide_write_i || (|wide_wstrb_i[7:4]);

  always_comb begin
    narrow_valid_o = 1'b0;
    narrow_write_o = wide_write_i;
    narrow_addr_o  = {wide_addr_i[31:3], 3'b000};
    narrow_wdata_o = wide_wdata_i[31:0];
    narrow_wstrb_o = wide_wstrb_i[3:0];
    wide_ready_o   = 1'b0;
    wide_rdata_o   = {narrow_rdata_i, low_rdata_q};
    wide_err_o     = low_err_q | narrow_err_i;

    unique case (state_q)
      ST_LOW_WORD: begin
        if (wide_valid_i) begin
          if (need_low) begin
            narrow_valid_o = 1'b1;
            narrow_addr_o  = {wide_addr_i[31:3], 3'b000};
            narrow_wdata_o = wide_wdata_i[31:0];
            narrow_wstrb_o = wide_wstrb_i[3:0];
            if (narrow_ready_i && !need_high) begin
              wide_ready_o = 1'b1;
              wide_rdata_o = {32'd0, narrow_rdata_i};
              wide_err_o   = narrow_err_i;
            end
          end else if (need_high) begin
            narrow_valid_o = 1'b1;
            narrow_addr_o  = {wide_addr_i[31:3], 3'b100};
            narrow_wdata_o = wide_wdata_i[63:32];
            narrow_wstrb_o = wide_wstrb_i[7:4];
            if (narrow_ready_i) begin
              wide_ready_o = 1'b1;
              wide_rdata_o = {narrow_rdata_i, 32'd0};
              wide_err_o   = narrow_err_i;
            end
          end else begin
            // Zero strobe write
            wide_ready_o = 1'b1;
            wide_err_o   = 1'b0;
          end
        end
      end

      ST_HIGH_WORD: begin
        narrow_valid_o = wide_valid_i;
        narrow_addr_o  = {wide_addr_i[31:3], 3'b100};
        narrow_wdata_o = wide_wdata_i[63:32];
        narrow_wstrb_o = wide_wstrb_i[7:4];
        if (wide_valid_i && narrow_ready_i) begin
          wide_ready_o = 1'b1;
          wide_rdata_o = {narrow_rdata_i, low_rdata_q};
          wide_err_o   = low_err_q | narrow_err_i;
        end
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q     <= ST_LOW_WORD;
      low_rdata_q <= '0;
      low_err_q   <= 1'b0;
    end else begin
      unique case (state_q)
        ST_LOW_WORD: begin
          if (wide_valid_i && need_low && need_high && narrow_ready_i) begin
            low_rdata_q <= narrow_rdata_i;
            low_err_q   <= narrow_err_i;
            state_q     <= ST_HIGH_WORD;
          end
        end
        ST_HIGH_WORD: begin
          if (wide_valid_i && narrow_ready_i) begin
            state_q   <= ST_LOW_WORD;
            low_err_q <= 1'b0;
          end
        end
        default: state_q <= ST_LOW_WORD;
      endcase
    end
  end

endmodule : width_upsizer_downsizer
