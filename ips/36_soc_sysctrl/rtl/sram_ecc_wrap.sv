// SPDX-License-Identifier: Apache-2.0
// sram_ecc_wrap.sv - Synthesizable Hamming SECDED(39,32) ECC Wrapper around tech_sram_sp
//
// Features:
//   - Encodes 32-bit data into 39-bit Hamming SECDED codeword (6 Hamming parity
//     bits at powers of 2 [1,2,4,8,16,32] + overall even-parity bit [0]).
//   - Corrects all 39 single-bit errors inline on reads & RMWs (ce_pulse_o, ce_cnt_o, ce_addr_o).
//   - Detects all double-bit errors (ue_pulse_o, ue_cnt_o, ue_addr_o, rsp_err_o).
//   - Automatic 2-cycle Read-Modify-Write (RMW) when req_wstrb_i != 4'hF.
//   - Optional hardware background scrubber (ENABLE_SCRUB=1) that scans SRAM
//     during idle windows and writes back corrected codewords on CE.
//   - DFT fault-injection mask (err_inj_i[38:0]) XORed onto host write codewords.

module sram_ecc_wrap
  import ip_common_pkg::*;
#(
  parameter int unsigned DEPTH        = 256,
  parameter bit          ENABLE_SCRUB = 1'b1,
  localparam int unsigned ADDR_W      = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  // Synchronous Host SRAM Interface
  input  logic              req_valid_i,
  output logic              req_ready_o,
  input  logic              req_we_i,
  input  logic [3:0]        req_wstrb_i,
  input  logic [ADDR_W-1:0] req_addr_i,
  input  logic [31:0]       req_wdata_i,

  output logic              rsp_valid_o,
  output logic [31:0]       rsp_rdata_o,
  output logic              rsp_err_o,

  // Background Scrubber & DFT Fault Injection
  input  logic              scrub_en_i,
  input  logic [15:0]       scrub_period_i,
  output logic              scrub_busy_o,
  input  logic [38:0]       err_inj_i,

  // ECC Error Telemetry
  output logic              ce_pulse_o,
  output logic              ue_pulse_o,
  output logic [31:0]       ce_cnt_o,
  output logic [31:0]       ue_cnt_o,
  output logic [ADDR_W-1:0] ce_addr_o,
  output logic [ADDR_W-1:0] ue_addr_o
);

  // -------------------------------------------------------------------------
  // Hamming SECDED(39,32) Encode / Decode Helpers
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic [31:0] data;
    logic        ce;
    logic        ue;
  } ecc_dec_t;

  function automatic logic [38:0] ecc_encode(input logic [31:0] data);
    logic [38:0] cw;
    int          d_idx;
    cw    = 39'd0;
    d_idx = 0;
    for (int pos = 1; pos <= 38; pos++) begin
      if ((pos & (pos - 1)) != 0) begin
        cw[pos] = data[d_idx];
        d_idx++;
      end
    end
    for (int k = 0; k < 6; k++) begin
      int   p_pos;
      logic p_bit;
      p_pos = (1 << k);
      p_bit = 1'b0;
      for (int pos = 1; pos <= 38; pos++) begin
        if ((pos != p_pos) && ((pos & p_pos) != 0)) begin
          p_bit ^= cw[pos];
        end
      end
      cw[p_pos] = p_bit;
    end
    cw[0] = ^(cw[38:1]);
    return cw;
  endfunction

  function automatic ecc_dec_t ecc_decode(input logic [38:0] rcw);
    ecc_dec_t    res;
    logic [5:0]  syn;
    logic        overall_p;
    logic [38:0] fixed_cw;
    int          d_idx;

    syn = 6'd0;
    for (int k = 0; k < 6; k++) begin
      int   p_pos;
      logic s_bit;
      p_pos = (1 << k);
      s_bit = 1'b0;
      for (int pos = 1; pos <= 38; pos++) begin
        if ((pos & p_pos) != 0) begin
          s_bit ^= rcw[pos];
        end
      end
      syn[k] = s_bit;
    end
    overall_p = ^(rcw[38:0]);

    fixed_cw = rcw;
    res.ce   = 1'b0;
    res.ue   = 1'b0;

    if (overall_p) begin
      if (syn == 6'd0) begin
        fixed_cw[0] = ~rcw[0];
        res.ce      = 1'b1;
      end else if (syn <= 6'd38) begin
        fixed_cw[syn] = ~rcw[syn];
        res.ce        = 1'b1;
      end else begin
        res.ue = 1'b1;
      end
    end else if (syn != 6'd0) begin
      res.ue = 1'b1;
    end

    d_idx    = 0;
    res.data = 32'd0;
    for (int pos = 1; pos <= 38; pos++) begin
      if ((pos & (pos - 1)) != 0) begin
        res.data[d_idx] = fixed_cw[pos];
        d_idx++;
      end
    end
    return res;
  endfunction

  // -------------------------------------------------------------------------
  // Single-Port Inferable SRAM Macro (39-bit Codeword Storage)
  // -------------------------------------------------------------------------
  logic              sram_en;
  logic              sram_we;
  logic [ADDR_W-1:0] sram_addr;
  logic [38:0]       sram_wdata;
  logic [38:0]       sram_rdata;

  tech_sram_sp #(
    .DATA_W (39),
    .DEPTH  (DEPTH),
    .BYTE_W (39)
  ) u_sram (
    .clk_i   (clk_i),
    .en_i    (sram_en),
    .we_i    (sram_we),
    .wstrb_i (1'b1),
    .addr_i  (sram_addr),
    .wdata_i (sram_wdata),
    .rdata_o (sram_rdata)
  );

  // -------------------------------------------------------------------------
  // Controller FSM & Telemetry Registers
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    ECC_IDLE,
    ECC_RMW_WRITE,
    ECC_SCRUB_READ,
    ECC_SCRUB_CHECK
  } ecc_state_e;

  ecc_state_e        state_q;
  logic              rd_pending_q;
  logic [ADDR_W-1:0] rd_addr_q;

  logic [ADDR_W-1:0] rmw_addr_q;
  logic [31:0]       rmw_wdata_q;
  logic [3:0]        rmw_wstrb_q;

  logic [ADDR_W-1:0] scrub_addr_q;
  logic [15:0]       scrub_timer_q;

  logic              rsp_valid_q;
  logic [31:0]       rsp_rdata_q;
  logic              rsp_err_q;

  logic              ce_pulse_q;
  logic              ue_pulse_q;
  logic [31:0]       ce_cnt_q;
  logic [31:0]       ue_cnt_q;
  logic [ADDR_W-1:0] ce_addr_q;
  logic [ADDR_W-1:0] ue_addr_q;

  ecc_dec_t          sram_dec_w;
  wire               scrub_due = ENABLE_SCRUB && scrub_en_i && !req_valid_i && !rd_pending_q &&
                                 (scrub_timer_q >= scrub_period_i);

  always_comb begin
    sram_dec_w   = ecc_decode(sram_rdata);
    sram_en      = 1'b0;
    sram_we      = 1'b0;
    sram_addr    = '0;
    sram_wdata   = 39'd0;
    req_ready_o  = 1'b0;
    scrub_busy_o = (state_q == ECC_SCRUB_READ) || (state_q == ECC_SCRUB_CHECK);

    unique case (state_q)
      ECC_IDLE: begin
        req_ready_o = 1'b1;
        if (req_valid_i) begin
          if (req_we_i) begin
            if (req_wstrb_i == 4'hF) begin
              sram_en    = 1'b1;
              sram_we    = 1'b1;
              sram_addr  = req_addr_i;
              sram_wdata = ecc_encode(req_wdata_i) ^ err_inj_i;
            end else if (req_wstrb_i != 4'h0) begin
              // Partial write: read existing codeword first
              sram_en   = 1'b1;
              sram_we   = 1'b0;
              sram_addr = req_addr_i;
            end
          end else begin
            sram_en   = 1'b1;
            sram_we   = 1'b0;
            sram_addr = req_addr_i;
          end
        end else if (scrub_due) begin
          sram_en   = 1'b1;
          sram_we   = 1'b0;
          sram_addr = scrub_addr_q;
        end
      end

      ECC_RMW_WRITE: begin
        automatic logic [31:0] merged = apply_wstrb(sram_dec_w.data, rmw_wdata_q, rmw_wstrb_q);
        sram_en    = 1'b1;
        sram_we    = 1'b1;
        sram_addr  = rmw_addr_q;
        sram_wdata = ecc_encode(merged) ^ err_inj_i;
      end

      ECC_SCRUB_READ: begin
        sram_en   = 1'b1;
        sram_we   = 1'b0;
        sram_addr = scrub_addr_q;
      end

      ECC_SCRUB_CHECK: begin
        if (sram_dec_w.ce) begin
          // Write back corrected codeword (without fault injection)
          sram_en    = 1'b1;
          sram_we    = 1'b1;
          sram_addr  = scrub_addr_q;
          sram_wdata = ecc_encode(sram_dec_w.data);
        end
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q       <= ECC_IDLE;
      rd_pending_q  <= 1'b0;
      rd_addr_q     <= '0;
      rmw_addr_q    <= '0;
      rmw_wdata_q   <= 32'd0;
      rmw_wstrb_q   <= 4'd0;
      scrub_addr_q  <= '0;
      scrub_timer_q <= 16'd0;
      rsp_valid_q   <= 1'b0;
      rsp_rdata_q   <= 32'd0;
      rsp_err_q     <= 1'b0;
      ce_pulse_q    <= 1'b0;
      ue_pulse_q    <= 1'b0;
      ce_cnt_q      <= 32'd0;
      ue_cnt_q      <= 32'd0;
      ce_addr_q     <= '0;
      ue_addr_q     <= '0;
    end else begin
      rd_pending_q <= 1'b0;
      rsp_valid_q  <= 1'b0;
      ce_pulse_q   <= 1'b0;
      ue_pulse_q   <= 1'b0;

      // Resolve any host read issued in the previous cycle
      if (rd_pending_q) begin
        rsp_valid_q <= 1'b1;
        rsp_rdata_q <= sram_dec_w.data;
        rsp_err_q   <= sram_dec_w.ue;
        if (sram_dec_w.ce) begin
          ce_pulse_q <= 1'b1;
          ce_addr_q  <= rd_addr_q;
          if (ce_cnt_q != 32'hFFFF_FFFF) ce_cnt_q <= ce_cnt_q + 32'd1;
        end
        if (sram_dec_w.ue) begin
          ue_pulse_q <= 1'b1;
          ue_addr_q  <= rd_addr_q;
          if (ue_cnt_q != 32'hFFFF_FFFF) ue_cnt_q <= ue_cnt_q + 32'd1;
        end
      end

      // Scrubber interval counter
      if (ENABLE_SCRUB && scrub_en_i) begin
        if ((state_q == ECC_IDLE) && !scrub_due && (scrub_timer_q < scrub_period_i)) begin
          scrub_timer_q <= scrub_timer_q + 16'd1;
        end
      end else begin
        scrub_timer_q <= 16'd0;
      end

      unique case (state_q)
        ECC_IDLE: begin
          if (req_valid_i) begin
            if (req_we_i) begin
              if ((req_wstrb_i != 4'hF) && (req_wstrb_i != 4'h0)) begin
                rmw_addr_q  <= req_addr_i;
                rmw_wdata_q <= req_wdata_i;
                rmw_wstrb_q <= req_wstrb_i;
                state_q     <= ECC_RMW_WRITE;
              end
            end else begin
              rd_pending_q <= 1'b1;
              rd_addr_q    <= req_addr_i;
            end
          end else if (scrub_due) begin
            scrub_timer_q <= 16'd0;
            state_q       <= ECC_SCRUB_CHECK;
          end
        end

        ECC_RMW_WRITE: begin
          if (sram_dec_w.ce) begin
            ce_pulse_q <= 1'b1;
            ce_addr_q  <= rmw_addr_q;
            if (ce_cnt_q != 32'hFFFF_FFFF) ce_cnt_q <= ce_cnt_q + 32'd1;
          end
          if (sram_dec_w.ue) begin
            ue_pulse_q <= 1'b1;
            ue_addr_q  <= rmw_addr_q;
            if (ue_cnt_q != 32'hFFFF_FFFF) ue_cnt_q <= ue_cnt_q + 32'd1;
          end
          state_q <= ECC_IDLE;
        end

        ECC_SCRUB_READ: begin
          state_q <= ECC_SCRUB_CHECK;
        end

        ECC_SCRUB_CHECK: begin
          if (sram_dec_w.ce) begin
            ce_pulse_q <= 1'b1;
            ce_addr_q  <= scrub_addr_q;
            if (ce_cnt_q != 32'hFFFF_FFFF) ce_cnt_q <= ce_cnt_q + 32'd1;
          end
          if (sram_dec_w.ue) begin
            ue_pulse_q <= 1'b1;
            ue_addr_q  <= scrub_addr_q;
            if (ue_cnt_q != 32'hFFFF_FFFF) ue_cnt_q <= ue_cnt_q + 32'd1;
          end
          scrub_addr_q <= (scrub_addr_q == ADDR_W'(DEPTH - 1)) ? '0 : (scrub_addr_q + ADDR_W'(1));
          state_q      <= ECC_IDLE;
        end

        default: state_q <= ECC_IDLE;
      endcase
    end
  end

  assign rsp_valid_o = rsp_valid_q;
  assign rsp_rdata_o = rsp_rdata_q;
  assign rsp_err_o   = rsp_err_q;
  assign ce_pulse_o  = ce_pulse_q;
  assign ue_pulse_o  = ue_pulse_q;
  assign ce_cnt_o    = ce_cnt_q;
  assign ue_cnt_o    = ue_cnt_q;
  assign ce_addr_o   = ce_addr_q;
  assign ue_addr_o   = ue_addr_q;

endmodule : sram_ecc_wrap
