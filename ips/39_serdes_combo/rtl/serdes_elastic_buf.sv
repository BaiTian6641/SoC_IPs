// SPDX-License-Identifier: Apache-2.0
// serdes_elastic_buf.sv - Dual-Clock Elastic Buffer with Automatic SKP Insertion/Deletion

module serdes_elastic_buf (
  // RX Recovered Clock Domain (Write Side)
  input  logic       rx_clk_i,
  input  logic       rst_ni,
  input  logic       rx_valid_i,
  input  logic [7:0] rx_data_i,
  input  logic       rx_k_i,
  input  logic       rx_err_i,

  // Core Clock Domain (Read Side & Control)
  input  logic       clk_i,
  input  logic       ins_en_i,
  input  logic       del_en_i,
  input  logic [7:0] skp_char_i,   // e.g. K28.0 (8'h1C) or K28.3 (8'h7C)
  input  logic [3:0] low_wm_i,     // Insert SKP when fill <= low_wm_i
  input  logic [3:0] high_wm_i,    // Delete SKP when fill >= high_wm_i
  input  logic       cnt_clr_i,

  output logic       rx_valid_o,
  output logic [7:0] rx_data_o,
  output logic       rx_k_o,
  output logic       rx_err_o,
  output logic       skp_ins_pulse_o,
  output logic       skp_del_pulse_o,
  output logic [7:0] skp_ins_cnt_o,
  output logic [7:0] skp_del_cnt_o,
  output logic [7:0] ovf_cnt_o,
  output logic [7:0] unf_cnt_o
);

  function automatic logic [4:0] bin2gray5(input logic [4:0] b);
    return (b >> 1) ^ b;
  endfunction

  function automatic logic [4:0] gray2bin5(input logic [4:0] g);
    logic [4:0] b;
    b[4] = g[4];
    for (int i = 3; i >= 0; i--) begin
      b[i] = b[i+1] ^ g[i];
    end
    return b;
  endfunction

  // ---------------------------------------------------------------------------
  // Synchronized Resets for Both Clock Domains
  // ---------------------------------------------------------------------------
  logic rx_rst_n_sync, core_rst_n_sync;

  tech_rst_sync #(.STAGES(2)) u_rx_rst_sync (
    .clk_i       (rx_clk_i),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (rx_rst_n_sync)
  );

  tech_rst_sync #(.STAGES(2)) u_core_rst_sync (
    .clk_i       (clk_i),
    .arst_ni     (rst_ni),
    .test_mode_i (1'b0),
    .rst_no      (core_rst_n_sync)
  );

  // Sync control bits into rx_clk_i domain
  logic       del_en_rx;
  logic [7:0] skp_char_rx;
  logic [3:0] high_wm_rx;

  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_delen (
    .clk_i(rx_clk_i), .rst_ni(rx_rst_n_sync), .d_i(del_en_i), .q_o(del_en_rx)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(8)) u_cdc_skpchar (
    .clk_i(rx_clk_i), .rst_ni(rx_rst_n_sync), .d_i(skp_char_i), .q_o(skp_char_rx)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(4)) u_cdc_highwm (
    .clk_i(rx_clk_i), .rst_ni(rx_rst_n_sync), .d_i(high_wm_i), .q_o(high_wm_rx)
  );

  // ---------------------------------------------------------------------------
  // Underlying Dual-Clock Async FIFO (16 entries x 11 bits)
  // ---------------------------------------------------------------------------
  logic        fifo_wpush, fifo_wfull;
  logic [10:0] fifo_wdata;
  logic        fifo_rpop, fifo_rempty;
  logic [10:0] fifo_rdata;

  async_fifo #(
    .DATA_W (11),
    .ADDR_W (4)
  ) u_async_fifo (
    .wclk_i   (rx_clk_i),
    .wrst_ni  (rx_rst_n_sync),
    .wpush_i  (fifo_wpush),
    .wdata_i  (fifo_wdata),
    .wfull_o  (fifo_wfull),
    .rclk_i   (clk_i),
    .rrst_ni  (core_rst_n_sync),
    .rpop_i   (fifo_rpop),
    .rdata_o  (fifo_rdata),
    .rempty_o (fifo_rempty)
  );

  // ---------------------------------------------------------------------------
  // Write Domain (rx_clk_i): Fill Tracking & SKP Deletion
  // ---------------------------------------------------------------------------
  logic [4:0] wcnt_q, wcnt_gray_q;
  logic [4:0] rcnt_gray_q, rcnt_gray_w_sync;
  logic [4:0] w_fill;
  logic       del_prev_q;
  logic       skp_del_tog_rx_q, ovf_tog_rx_q;
  logic       in_is_skp;

  assign in_is_skp = rx_k_i && (rx_data_i == skp_char_rx);

  tech_cdc_sync #(.STAGES(2), .WIDTH(5)) u_cdc_rcnt_to_w (
    .clk_i(rx_clk_i), .rst_ni(rx_rst_n_sync), .d_i(rcnt_gray_q), .q_o(rcnt_gray_w_sync)
  );

  assign w_fill = wcnt_q - gray2bin5(rcnt_gray_w_sync);

  wire do_skp_del = rx_valid_i && in_is_skp && del_en_rx &&
                    (w_fill >= {1'b0, high_wm_rx}) && !del_prev_q;

  assign fifo_wpush = rx_valid_i && !do_skp_del;
  assign fifo_wdata = {in_is_skp, rx_err_i, rx_k_i, rx_data_i};

  always_ff @(posedge rx_clk_i or negedge rx_rst_n_sync) begin
    if (!rx_rst_n_sync) begin
      wcnt_q           <= 5'd0;
      wcnt_gray_q      <= 5'd0;
      del_prev_q       <= 1'b0;
      skp_del_tog_rx_q <= 1'b0;
      ovf_tog_rx_q     <= 1'b0;
    end else begin
      if (fifo_wpush && !fifo_wfull) begin
        wcnt_q      <= wcnt_q + 5'd1;
        wcnt_gray_q <= bin2gray5(wcnt_q + 5'd1);
      end
      if (rx_valid_i) begin
        if (do_skp_del) begin
          del_prev_q       <= 1'b1;
          skp_del_tog_rx_q <= ~skp_del_tog_rx_q;
        end else begin
          del_prev_q <= 1'b0;
        end
        if (fifo_wpush && fifo_wfull) begin
          ovf_tog_rx_q <= ~ovf_tog_rx_q;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Read Domain (clk_i): Nominal Start Hold-off, SKP Insertion & Statistics
  // ---------------------------------------------------------------------------
  logic [4:0] rcnt_q;
  logic [4:0] wcnt_gray_r_sync;
  logic [4:0] r_fill;
  logic       started_q;
  logic       ins_pend_q;
  logic       ins_prev_q;

  logic       skp_del_tog_r, skp_del_tog_r_q;
  logic       ovf_tog_r, ovf_tog_r_q;

  tech_cdc_sync #(.STAGES(2), .WIDTH(5)) u_cdc_wcnt_to_r (
    .clk_i(clk_i), .rst_ni(core_rst_n_sync), .d_i(wcnt_gray_q), .q_o(wcnt_gray_r_sync)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_del_tog (
    .clk_i(clk_i), .rst_ni(core_rst_n_sync), .d_i(skp_del_tog_rx_q), .q_o(skp_del_tog_r)
  );
  tech_cdc_sync #(.STAGES(2), .WIDTH(1)) u_cdc_ovf_tog (
    .clk_i(clk_i), .rst_ni(core_rst_n_sync), .d_i(ovf_tog_rx_q), .q_o(ovf_tog_r)
  );

  assign r_fill = gray2bin5(wcnt_gray_r_sync) - rcnt_q;

  wire r_entry_is_skp = fifo_rdata[10];
  wire r_entry_err    = fifo_rdata[9];
  wire r_entry_k      = fifo_rdata[8];
  wire [7:0] r_entry_data = fifo_rdata[7:0];

  wire skp_del_pulse = skp_del_tog_r ^ skp_del_tog_r_q;
  wire ovf_pulse     = ovf_tog_r ^ ovf_tog_r_q;

  // Pop normally once started, unless holding to emit an inserted SKP
  assign fifo_rpop = started_q && !ins_pend_q && !fifo_rempty;

  always_ff @(posedge clk_i or negedge core_rst_n_sync) begin
    if (!core_rst_n_sync) begin
      rcnt_q          <= 5'd0;
      rcnt_gray_q     <= 5'd0;
      started_q       <= 1'b0;
      ins_pend_q      <= 1'b0;
      ins_prev_q      <= 1'b0;
      rx_valid_o      <= 1'b0;
      rx_data_o       <= 8'd0;
      rx_k_o          <= 1'b0;
      rx_err_o        <= 1'b0;
      skp_ins_pulse_o <= 1'b0;
      skp_del_pulse_o <= 1'b0;
      skp_ins_cnt_o   <= 8'd0;
      skp_del_cnt_o   <= 8'd0;
      ovf_cnt_o       <= 8'd0;
      unf_cnt_o       <= 8'd0;
      skp_del_tog_r_q <= 1'b0;
      ovf_tog_r_q     <= 1'b0;
    end else begin
      skp_del_tog_r_q <= skp_del_tog_r;
      ovf_tog_r_q     <= ovf_tog_r;
      skp_ins_pulse_o <= 1'b0;
      skp_del_pulse_o <= skp_del_pulse;

      if (cnt_clr_i) begin
        skp_ins_cnt_o <= 8'd0;
        skp_del_cnt_o <= 8'd0;
        ovf_cnt_o     <= 8'd0;
        unf_cnt_o     <= 8'd0;
      end else begin
        if (skp_del_pulse && (skp_del_cnt_o != 8'hFF)) skp_del_cnt_o <= skp_del_cnt_o + 8'd1;
        if (ovf_pulse && (ovf_cnt_o != 8'hFF))         ovf_cnt_o     <= ovf_cnt_o + 8'd1;
      end

      if (!started_q) begin
        rx_valid_o <= 1'b0;
        if (r_fill >= 5'd7) begin
          started_q <= 1'b1;
        end
      end else if (ins_pend_q) begin
        // Emit extra inserted SKP symbol without popping FIFO
        ins_pend_q      <= 1'b0;
        ins_prev_q      <= 1'b1;
        rx_valid_o      <= 1'b1;
        rx_data_o       <= skp_char_i;
        rx_k_o          <= 1'b1;
        rx_err_o        <= 1'b0;
        skp_ins_pulse_o <= 1'b1;
        if (!cnt_clr_i && (skp_ins_cnt_o != 8'hFF)) begin
          skp_ins_cnt_o <= skp_ins_cnt_o + 8'd1;
        end
      end else if (!fifo_rempty) begin
        rcnt_q      <= rcnt_q + 5'd1;
        rcnt_gray_q <= bin2gray5(rcnt_q + 5'd1);
        rx_valid_o  <= 1'b1;
        rx_data_o   <= r_entry_data;
        rx_k_o      <= r_entry_k;
        rx_err_o    <= r_entry_err;
        ins_prev_q  <= 1'b0;

        if (ins_en_i && (r_fill <= {1'b0, low_wm_i}) && r_entry_is_skp && !ins_prev_q) begin
          ins_pend_q <= 1'b1;
        end
      end else begin
        // FIFO empty after start: emergency SKP insert if enabled, or count underflow
        ins_prev_q <= 1'b0;
        if (ins_en_i) begin
          rx_valid_o      <= 1'b1;
          rx_data_o       <= skp_char_i;
          rx_k_o          <= 1'b1;
          rx_err_o        <= 1'b0;
          skp_ins_pulse_o <= 1'b1;
          if (!cnt_clr_i && (skp_ins_cnt_o != 8'hFF)) begin
            skp_ins_cnt_o <= skp_ins_cnt_o + 8'd1;
          end
        end else begin
          rx_valid_o <= 1'b0;
          if (!cnt_clr_i && (unf_cnt_o != 8'hFF)) begin
            unf_cnt_o <= unf_cnt_o + 8'd1;
          end
        end
      end
    end
  end

endmodule : serdes_elastic_buf
