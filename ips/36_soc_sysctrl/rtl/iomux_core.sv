// SPDX-License-Identifier: Apache-2.0
// iomux_core.sv - Parameterized Pinmux & Pad-Ring Controller Core
//
// Features:
//   - NUM_PINS (default 32) x NUM_FUNCS (default 8) programmable alternate-function
//     crossbar with split pad-ring outputs (no tri-state inside core).
//   - Per-pin CSR config: ALT_FUNC, DRIVE[1:0], SLEW, PULL_EN, PULL_UP, SCHMITT_EN,
//     IN_ISO (clamps func_i to 0), OUT_RET (latches pre-sleep pad_o/pad_oe_o),
//     and write-once LOCK bit (sticky until reset).
//   - Optional hardware global isolation/retention inputs (global_iso_i, global_ret_i).

module iomux_core
  import bus_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned NUM_PINS  = 32,
  parameter int unsigned NUM_FUNCS = 8
) (
  input  logic                                   clk_i,
  input  logic                                   rst_ni,

  input  csr_req_t                               csr_req_i,
  output csr_rsp_t                               csr_rsp_o,

  // Global Low-Power Hardware Overrides (tie 1'b0 if unused)
  input  logic                                   global_iso_i,
  input  logic                                   global_ret_i,

  // Peripheral Function Side
  input  logic [NUM_PINS-1:0][NUM_FUNCS-1:0]     func_o,
  input  logic [NUM_PINS-1:0][NUM_FUNCS-1:0]     func_oe_o,
  output logic [NUM_PINS-1:0][NUM_FUNCS-1:0]     func_i,

  // Pad Ring Side (Split I/O/OE + Pad Attributes)
  input  logic [NUM_PINS-1:0]                    pad_i,
  output logic [NUM_PINS-1:0]                    pad_o,
  output logic [NUM_PINS-1:0]                    pad_oe_o,
  output logic [NUM_PINS-1:0][1:0]               pad_drive_o,
  output logic [NUM_PINS-1:0]                    pad_slew_o,
  output logic [NUM_PINS-1:0]                    pad_pull_en_o,
  output logic [NUM_PINS-1:0]                    pad_pull_up_o,
  output logic [NUM_PINS-1:0]                    pad_schmitt_o
);

  localparam int unsigned ALT_W     = (NUM_FUNCS <= 1) ? 1 : $clog2(NUM_FUNCS);
  localparam int unsigned PIN_IDX_W = (NUM_PINS <= 1)  ? 1 : $clog2(NUM_PINS);

  // Per-pin configuration storage
  logic [2:0]  alt_func_q   [NUM_PINS];
  logic [1:0]  drive_q      [NUM_PINS];
  logic        slew_q       [NUM_PINS];
  logic        pull_en_q    [NUM_PINS];
  logic        pull_up_q    [NUM_PINS];
  logic        schmitt_en_q [NUM_PINS];
  logic        in_iso_q     [NUM_PINS];
  logic        out_ret_q    [NUM_PINS];
  logic        lock_q       [NUM_PINS];

  // Retention latches for pad output & output-enable
  logic [NUM_PINS-1:0] ret_pad_o_q;
  logic [NUM_PINS-1:0] ret_pad_oe_q;

  // Global low-power overrides
  logic sw_global_iso_q;
  logic sw_global_ret_q;
  logic global_lock_q;

  // Combinational mux & retention/isolation routing
  logic [NUM_PINS-1:0] mux_o;
  logic [NUM_PINS-1:0] mux_oe;
  logic [NUM_PINS-1:0] lock_mask_w;

  always_comb begin
    for (int p = 0; p < NUM_PINS; p++) begin
      automatic logic [ALT_W-1:0] sel = ALT_W'(alt_func_q[p]);
      automatic logic             iso_eff = in_iso_q[p] || sw_global_iso_q || global_iso_i;
      automatic logic             ret_eff = out_ret_q[p] || sw_global_ret_q || global_ret_i;

      mux_o[p]  = func_o[p][sel];
      mux_oe[p] = func_oe_o[p][sel];

      pad_o[p]         = ret_eff ? ret_pad_o_q[p]  : mux_o[p];
      pad_oe_o[p]      = ret_eff ? ret_pad_oe_q[p] : mux_oe[p];
      pad_drive_o[p]   = drive_q[p];
      pad_slew_o[p]    = slew_q[p];
      pad_pull_en_o[p] = pull_en_q[p];
      pad_pull_up_o[p] = pull_up_q[p];
      pad_schmitt_o[p] = schmitt_en_q[p];
      lock_mask_w[p]   = lock_q[p];

      for (int f = 0; f < NUM_FUNCS; f++) begin
        if (!iso_eff && (f == int'(sel))) begin
          func_i[p][f] = pad_i[p];
        end else begin
          func_i[p][f] = 1'b0;
        end
      end
    end
  end

  // Capture live pad_o/pad_oe_o whenever retention is NOT asserted; hold once asserted
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ret_pad_o_q  <= '0;
      ret_pad_oe_q <= '0;
    end else begin
      for (int p = 0; p < NUM_PINS; p++) begin
        if (!(out_ret_q[p] || sw_global_ret_q || global_ret_i)) begin
          ret_pad_o_q[p]  <= mux_o[p];
          ret_pad_oe_q[p] <= mux_oe[p];
        end
      end
    end
  end

  // -------------------------------------------------------------------------
  // CSR Decode (0x000..0x0FC: PIN_CFG[p], 0x100..0x10C: Global/Status)
  // -------------------------------------------------------------------------
  wire [8:0] reg_off = csr_req_i.addr[8:0];

  function automatic logic [31:0] pack_pin_cfg(input int p);
    return {
      lock_q[p],
      19'd0,
      out_ret_q[p],
      in_iso_q[p],
      schmitt_en_q[p],
      pull_up_q[p],
      pull_en_q[p],
      slew_q[p],
      drive_q[p],
      1'b0,
      alt_func_q[p]
    };
  endfunction

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = 32'd0;

    if ((reg_off < 9'(4 * NUM_PINS)) && (reg_off[1:0] == 2'b00)) begin
      automatic logic [PIN_IDX_W-1:0] pidx = PIN_IDX_W'(reg_off >> 2);
      csr_rsp_o.rdata = pack_pin_cfg(int'(pidx));
    end else begin
      unique case (reg_off)
        9'h100: csr_rsp_o.rdata = 32'(pad_i);
        9'h104: csr_rsp_o.rdata = {global_lock_q, 29'd0, sw_global_ret_q, sw_global_iso_q};
        9'h108: csr_rsp_o.rdata = 32'(lock_mask_w);
        9'h10C: csr_rsp_o.rdata = {16'(NUM_FUNCS), 16'(NUM_PINS)};
        default: csr_rsp_o.err = csr_req_i.valid;
      endcase
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sw_global_iso_q <= 1'b0;
      sw_global_ret_q <= 1'b0;
      global_lock_q   <= 1'b0;
      for (int p = 0; p < NUM_PINS; p++) begin
        alt_func_q[p]   <= 3'd0;
        drive_q[p]      <= 2'd1;
        slew_q[p]       <= 1'b0;
        pull_en_q[p]    <= 1'b0;
        pull_up_q[p]    <= 1'b0;
        schmitt_en_q[p] <= 1'b0;
        in_iso_q[p]     <= 1'b0;
        out_ret_q[p]    <= 1'b0;
        lock_q[p]       <= 1'b0;
      end
    end else if (csr_req_i.valid && csr_req_i.write) begin
      automatic logic [31:0] w = csr_req_i.wdata;
      automatic logic [3:0]  s = csr_req_i.wstrb;

      if ((reg_off < 9'(4 * NUM_PINS)) && (reg_off[1:0] == 2'b00)) begin
        automatic logic [PIN_IDX_W-1:0] pidx = PIN_IDX_W'(reg_off >> 2);
        if (!lock_q[pidx]) begin
          automatic logic [31:0] cur = pack_pin_cfg(int'(pidx));
          automatic logic [31:0] nxt = apply_wstrb(cur, w, s);
          alt_func_q[pidx]   <= 3'(ALT_W'(nxt[2:0]));
          drive_q[pidx]      <= nxt[5:4];
          slew_q[pidx]       <= nxt[6];
          pull_en_q[pidx]    <= nxt[7];
          pull_up_q[pidx]    <= nxt[8];
          schmitt_en_q[pidx] <= nxt[9];
          in_iso_q[pidx]     <= nxt[10];
          out_ret_q[pidx]    <= nxt[11];
          lock_q[pidx]       <= nxt[31];
        end
      end else if (reg_off == 9'h104) begin
        if (!global_lock_q) begin
          if (s[0]) begin
            sw_global_iso_q <= w[0];
            sw_global_ret_q <= w[1];
          end
          if (s[3]) begin
            global_lock_q <= w[31];
          end
        end
      end
    end
  end

endmodule : iomux_core
