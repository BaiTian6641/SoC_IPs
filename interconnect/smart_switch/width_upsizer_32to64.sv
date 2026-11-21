// SPDX-License-Identifier: Apache-2.0
// width_upsizer_32to64.sv - 32-bit Narrow Initiator to 64-bit Wide Target Converter
// Supports zero-latency lane steering (WRITE_COMBINE=0) and optional 2-word write combining (WRITE_COMBINE=1).

module width_upsizer_32to64 #(
  parameter bit WRITE_COMBINE = 1'b0
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        flush_i = 1'b0,

  // 32-bit Narrow Initiator Side
  input  logic        narrow_valid_i,
  output logic        narrow_ready_o,
  input  logic        narrow_write_i,
  input  logic [31:0] narrow_addr_i,
  input  logic [31:0] narrow_wdata_i,
  input  logic [3:0]  narrow_wstrb_i,
  output logic [31:0] narrow_rdata_o,
  output logic        narrow_err_o,

  // 64-bit Wide Target Side
  output logic        wide_valid_o,
  input  logic        wide_ready_i,
  output logic        wide_write_o,
  output logic [31:0] wide_addr_o,
  output logic [63:0] wide_wdata_o,
  output logic [7:0]  wide_wstrb_o,
  input  logic [63:0] wide_rdata_i,
  input  logic        wide_err_i
);

  if (!WRITE_COMBINE) begin : gen_direct
    always_comb begin
      wide_valid_o   = narrow_valid_i;
      wide_write_o   = narrow_write_i;
      wide_addr_o    = {narrow_addr_i[31:3], 3'b000};
      wide_wdata_o   = narrow_addr_i[2] ? {narrow_wdata_i, 32'd0} : {32'd0, narrow_wdata_i};
      wide_wstrb_o   = narrow_addr_i[2] ? {narrow_wstrb_i, 4'd0}  : {4'd0, narrow_wstrb_i};
      narrow_ready_o = wide_ready_i;
      narrow_rdata_o = narrow_addr_i[2] ? wide_rdata_i[63:32] : wide_rdata_i[31:0];
      narrow_err_o   = wide_err_i;
    end
  end else begin : gen_combine
    logic        buf_vld_q;
    logic [31:0] buf_addr_q;
    logic [31:0] buf_data_q;
    logic [3:0]  buf_strb_q;

    wire can_merge = buf_vld_q && narrow_valid_i && narrow_write_i &&
                     (narrow_addr_i[31:3] == buf_addr_q[31:3]) && narrow_addr_i[2];

    // Never preempt an active/stalled merge on flush_i: merging flushes the buffer and
    // completes the upper lane in one wide beat without mutating wide_wdata_o/wide_wstrb_o (AR-16).
    wire need_flush = buf_vld_q && !can_merge && (flush_i || narrow_valid_i);
    wire start_buf  = !buf_vld_q && narrow_valid_i && narrow_write_i && !narrow_addr_i[2] && !flush_i;

    always_comb begin
      wide_valid_o   = 1'b0;
      wide_write_o   = 1'b0;
      wide_addr_o    = {narrow_addr_i[31:3], 3'b000};
      wide_wdata_o   = narrow_addr_i[2] ? {narrow_wdata_i, 32'd0} : {32'd0, narrow_wdata_i};
      wide_wstrb_o   = narrow_addr_i[2] ? {narrow_wstrb_i, 4'd0}  : {4'd0, narrow_wstrb_i};
      narrow_ready_o = 1'b0;
      narrow_rdata_o = narrow_addr_i[2] ? wide_rdata_i[63:32] : wide_rdata_i[31:0];
      narrow_err_o   = 1'b0;

      if (need_flush) begin
        wide_valid_o   = 1'b1;
        wide_write_o   = 1'b1;
        wide_addr_o    = {buf_addr_q[31:3], 3'b000};
        wide_wdata_o   = {32'd0, buf_data_q};
        wide_wstrb_o   = {4'd0, buf_strb_q};
        narrow_ready_o = 1'b0;
      end else if (can_merge) begin
        wide_valid_o   = 1'b1;
        wide_write_o   = 1'b1;
        wide_addr_o    = {buf_addr_q[31:3], 3'b000};
        wide_wdata_o   = {narrow_wdata_i, buf_data_q};
        wide_wstrb_o   = {narrow_wstrb_i, buf_strb_q};
        narrow_ready_o = wide_ready_i;
        narrow_err_o   = wide_err_i;
      end else if (start_buf) begin
        narrow_ready_o = 1'b1;
        narrow_err_o   = 1'b0;
      end else if (narrow_valid_i) begin
        wide_valid_o   = 1'b1;
        wide_write_o   = narrow_write_i;
        narrow_ready_o = wide_ready_i;
        narrow_err_o   = wide_err_i;
      end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        buf_vld_q  <= 1'b0;
        buf_addr_q <= '0;
        buf_data_q <= '0;
        buf_strb_q <= '0;
      end else begin
        if (need_flush) begin
          if (wide_ready_i) buf_vld_q <= 1'b0;
        end else if (can_merge) begin
          if (wide_ready_i) buf_vld_q <= 1'b0;
        end else if (start_buf) begin
          buf_vld_q  <= 1'b1;
          buf_addr_q <= narrow_addr_i;
          buf_data_q <= narrow_wdata_i;
          buf_strb_q <= narrow_wstrb_i;
        end
      end
    end
  end

endmodule : width_upsizer_32to64
