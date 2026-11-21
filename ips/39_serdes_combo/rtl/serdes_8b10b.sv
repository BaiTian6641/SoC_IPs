// SPDX-License-Identifier: Apache-2.0
// serdes_8b10b.sv - Full IEEE/IBM 8b/10b Encoder, Decoder, and K28.5 Comma Word Aligner

module serdes_8b10b (
  // TX Domain (8b/10b Encoder)
  input  logic       tx_clk_i,
  input  logic       tx_rst_ni,
  input  logic       tx_valid_i,
  input  logic [7:0] tx_data_i,
  input  logic       tx_k_i,
  input  logic       tx_rd_clr_i,
  output logic [9:0] tx_10b_o,     // {a,b,c,d,e,i,f,g,h,j}
  output logic       tx_valid_o,
  output logic       tx_rd_o,

  // RX Domain (Comma Word Aligner + 8b/10b Decoder)
  input  logic       rx_clk_i,
  input  logic       rx_rst_ni,
  input  logic       rx_valid_i,
  input  logic [9:0] rx_raw_10b_i, // {a,b,c,d,e,i,f,g,h,j}
  input  logic       align_en_i,
  input  logic       align_bypass_i,
  input  logic       rx_rd_clr_i,
  output logic       rx_valid_o,
  output logic [7:0] rx_data_o,
  output logic       rx_k_o,
  output logic       rx_disp_err_o,
  output logic       rx_code_err_o,
  output logic       rx_comma_o,
  output logic       rx_aligned_o,
  output logic       rx_bitslip_o,
  output logic [9:0] rx_aligned_10b_o
);

  // ---------------------------------------------------------------------------
  // Combinational 8b/10b Encoder Function
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [9:0] code10;
    logic       rd_out;
  } enc_res_t;

  function automatic enc_res_t encode_8b10b(
    input logic [7:0] din,
    input logic       kin,
    input logic       rd_in
  );
    enc_res_t   res;
    logic [4:0] x_val;
    logic [2:0] y_val;
    logic [5:0] code6_rd0;
    logic       disp6_nz;
    logic [5:0] code6;
    logic       rd_mid;
    logic [3:0] code4;
    logic       rd_out;
    logic       is_k28;
    logic       use_alt7;

    x_val  = din[4:0];
    y_val  = din[7:5];
    is_k28 = kin && (x_val == 5'd28);

    // 5b/6b Lookup (RD- base code & whether disparity is +/-2)
    disp6_nz = 1'b0;
    unique case (x_val)
      5'd0:  begin code6_rd0 = 6'b100111; disp6_nz = 1'b1; end
      5'd1:  begin code6_rd0 = 6'b011101; disp6_nz = 1'b1; end
      5'd2:  begin code6_rd0 = 6'b101101; disp6_nz = 1'b1; end
      5'd3:  begin code6_rd0 = 6'b110001; disp6_nz = 1'b0; end
      5'd4:  begin code6_rd0 = 6'b110101; disp6_nz = 1'b1; end
      5'd5:  begin code6_rd0 = 6'b101001; disp6_nz = 1'b0; end
      5'd6:  begin code6_rd0 = 6'b011001; disp6_nz = 1'b0; end
      5'd7:  begin code6_rd0 = 6'b111000; disp6_nz = 1'b0; end
      5'd8:  begin code6_rd0 = 6'b111001; disp6_nz = 1'b1; end
      5'd9:  begin code6_rd0 = 6'b100101; disp6_nz = 1'b0; end
      5'd10: begin code6_rd0 = 6'b010101; disp6_nz = 1'b0; end
      5'd11: begin code6_rd0 = 6'b110100; disp6_nz = 1'b0; end
      5'd12: begin code6_rd0 = 6'b001101; disp6_nz = 1'b0; end
      5'd13: begin code6_rd0 = 6'b101100; disp6_nz = 1'b0; end
      5'd14: begin code6_rd0 = 6'b011100; disp6_nz = 1'b0; end
      5'd15: begin code6_rd0 = 6'b010111; disp6_nz = 1'b1; end
      5'd16: begin code6_rd0 = 6'b011011; disp6_nz = 1'b1; end
      5'd17: begin code6_rd0 = 6'b100011; disp6_nz = 1'b0; end
      5'd18: begin code6_rd0 = 6'b010011; disp6_nz = 1'b0; end
      5'd19: begin code6_rd0 = 6'b110010; disp6_nz = 1'b0; end
      5'd20: begin code6_rd0 = 6'b001011; disp6_nz = 1'b0; end
      5'd21: begin code6_rd0 = 6'b101010; disp6_nz = 1'b0; end
      5'd22: begin code6_rd0 = 6'b011010; disp6_nz = 1'b0; end
      5'd23: begin code6_rd0 = 6'b111010; disp6_nz = 1'b1; end
      5'd24: begin code6_rd0 = 6'b110011; disp6_nz = 1'b1; end
      5'd25: begin code6_rd0 = 6'b100110; disp6_nz = 1'b0; end
      5'd26: begin code6_rd0 = 6'b010110; disp6_nz = 1'b0; end
      5'd27: begin code6_rd0 = 6'b110110; disp6_nz = 1'b1; end
      5'd28: begin
        if (is_k28) begin code6_rd0 = 6'b001111; disp6_nz = 1'b1; end
        else        begin code6_rd0 = 6'b001110; disp6_nz = 1'b0; end
      end
      5'd29: begin code6_rd0 = 6'b101110; disp6_nz = 1'b1; end
      5'd30: begin code6_rd0 = 6'b011110; disp6_nz = 1'b1; end
      5'd31: begin code6_rd0 = 6'b101011; disp6_nz = 1'b1; end
    endcase

    if (is_k28) begin
      code6  = rd_in ? ~code6_rd0 : code6_rd0;
      rd_mid = ~rd_in;
    end else if (x_val == 5'd7) begin
      code6  = rd_in ? ~code6_rd0 : code6_rd0;
      rd_mid = rd_in;
    end else if (disp6_nz) begin
      code6  = rd_in ? ~code6_rd0 : code6_rd0;
      rd_mid = ~rd_in;
    end else begin
      code6  = code6_rd0;
      rd_mid = rd_in;
    end

    // 3b/4b Encoding
    use_alt7 = kin ||
               (!rd_mid && (x_val == 5'd17 || x_val == 5'd18 || x_val == 5'd20)) ||
               ( rd_mid && (x_val == 5'd11 || x_val == 5'd13 || x_val == 5'd14));

    unique case (y_val)
      3'd0: begin
        code4  = rd_mid ? 4'b0100 : 4'b1011;
        rd_out = ~rd_mid;
      end
      3'd1: begin
        code4  = (is_k28 && !rd_mid) ? 4'b0110 : 4'b1001;
        rd_out = rd_mid;
      end
      3'd2: begin
        code4  = (is_k28 && !rd_mid) ? 4'b1010 : 4'b0101;
        rd_out = rd_mid;
      end
      3'd3: begin
        code4  = rd_mid ? 4'b0011 : 4'b1100;
        rd_out = rd_mid;
      end
      3'd4: begin
        code4  = rd_mid ? 4'b0010 : 4'b1101;
        rd_out = ~rd_mid;
      end
      3'd5: begin
        code4  = (is_k28 && !rd_mid) ? 4'b0101 : 4'b1010;
        rd_out = rd_mid;
      end
      3'd6: begin
        code4  = (is_k28 && !rd_mid) ? 4'b1001 : 4'b0110;
        rd_out = rd_mid;
      end
      3'd7: begin
        if (use_alt7) code4 = rd_mid ? 4'b1000 : 4'b0111;
        else          code4 = rd_mid ? 4'b0001 : 4'b1110;
        rd_out = ~rd_mid;
      end
    endcase

    res.code10 = {code6, code4};
    res.rd_out = rd_out;
    return res;
  endfunction

  // ---------------------------------------------------------------------------
  // Combinational 8b/10b Decoder Function
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [7:0] data;
    logic       k;
    logic       disp_err;
    logic       code_err;
    logic       rd_out;
  } dec_res_t;

  function automatic dec_res_t decode_8b10b(
    input logic [9:0] code10,
    input logic       rd_in
  );
    dec_res_t   res;
    logic [5:0] c6;
    logic [3:0] c4;
    logic [4:0] x_val;
    logic [2:0] y_val;
    logic       is_k28;
    logic       k4;
    logic       c6_ok, c4_ok;
    logic       disp6_err, disp4_err;
    logic       rd_mid, rd_out;

    c6 = code10[9:4];
    c4 = code10[3:0];

    x_val     = 5'd0;
    is_k28    = 1'b0;
    c6_ok     = 1'b1;
    disp6_err = 1'b0;
    rd_mid    = rd_in;

    // 6b -> 5b Decode
    unique case (c6)
      // Disparity 0 (3 ones)
      6'b110001: begin x_val = 5'd3;  rd_mid = rd_in; end
      6'b101001: begin x_val = 5'd5;  rd_mid = rd_in; end
      6'b011001: begin x_val = 5'd6;  rd_mid = rd_in; end
      6'b111000: begin x_val = 5'd7;  rd_mid = 1'b0; disp6_err = rd_in;  end
      6'b000111: begin x_val = 5'd7;  rd_mid = 1'b1; disp6_err = ~rd_in; end
      6'b100101: begin x_val = 5'd9;  rd_mid = rd_in; end
      6'b010101: begin x_val = 5'd10; rd_mid = rd_in; end
      6'b110100: begin x_val = 5'd11; rd_mid = rd_in; end
      6'b001101: begin x_val = 5'd12; rd_mid = rd_in; end
      6'b101100: begin x_val = 5'd13; rd_mid = rd_in; end
      6'b011100: begin x_val = 5'd14; rd_mid = rd_in; end
      6'b100011: begin x_val = 5'd17; rd_mid = rd_in; end
      6'b010011: begin x_val = 5'd18; rd_mid = rd_in; end
      6'b110010: begin x_val = 5'd19; rd_mid = rd_in; end
      6'b001011: begin x_val = 5'd20; rd_mid = rd_in; end
      6'b101010: begin x_val = 5'd21; rd_mid = rd_in; end
      6'b011010: begin x_val = 5'd22; rd_mid = rd_in; end
      6'b100110: begin x_val = 5'd25; rd_mid = rd_in; end
      6'b010110: begin x_val = 5'd26; rd_mid = rd_in; end
      6'b001110: begin x_val = 5'd28; rd_mid = rd_in; end

      // Disparity +2 (4 ones, valid at RD-, exit RD+)
      6'b100111: begin x_val = 5'd0;  rd_mid = 1'b1; disp6_err = rd_in; end
      6'b011101: begin x_val = 5'd1;  rd_mid = 1'b1; disp6_err = rd_in; end
      6'b101101: begin x_val = 5'd2;  rd_mid = 1'b1; disp6_err = rd_in; end
      6'b110101: begin x_val = 5'd4;  rd_mid = 1'b1; disp6_err = rd_in; end
      6'b111001: begin x_val = 5'd8;  rd_mid = 1'b1; disp6_err = rd_in; end
      6'b010111: begin x_val = 5'd15; rd_mid = 1'b1; disp6_err = rd_in; end
      6'b011011: begin x_val = 5'd16; rd_mid = 1'b1; disp6_err = rd_in; end
      6'b111010: begin x_val = 5'd23; rd_mid = 1'b1; disp6_err = rd_in; end
      6'b110011: begin x_val = 5'd24; rd_mid = 1'b1; disp6_err = rd_in; end
      6'b110110: begin x_val = 5'd27; rd_mid = 1'b1; disp6_err = rd_in; end
      6'b001111: begin x_val = 5'd28; rd_mid = 1'b1; disp6_err = rd_in; is_k28 = 1'b1; end
      6'b101110: begin x_val = 5'd29; rd_mid = 1'b1; disp6_err = rd_in; end
      6'b011110: begin x_val = 5'd30; rd_mid = 1'b1; disp6_err = rd_in; end
      6'b101011: begin x_val = 5'd31; rd_mid = 1'b1; disp6_err = rd_in; end

      // Disparity -2 (2 ones, valid at RD+, exit RD-)
      6'b011000: begin x_val = 5'd0;  rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b100010: begin x_val = 5'd1;  rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b010010: begin x_val = 5'd2;  rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b001010: begin x_val = 5'd4;  rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b000110: begin x_val = 5'd8;  rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b101000: begin x_val = 5'd15; rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b100100: begin x_val = 5'd16; rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b000101: begin x_val = 5'd23; rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b001100: begin x_val = 5'd24; rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b001001: begin x_val = 5'd27; rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b110000: begin x_val = 5'd28; rd_mid = 1'b0; disp6_err = ~rd_in; is_k28 = 1'b1; end
      6'b010001: begin x_val = 5'd29; rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b100001: begin x_val = 5'd30; rd_mid = 1'b0; disp6_err = ~rd_in; end
      6'b010100: begin x_val = 5'd31; rd_mid = 1'b0; disp6_err = ~rd_in; end

      default:   begin c6_ok = 1'b0; end
    endcase

    // 4b -> 3b Decode
    y_val     = 3'd0;
    k4        = is_k28;
    c4_ok     = 1'b1;
    disp4_err = 1'b0;
    rd_out    = rd_mid;

    unique case (c4)
      4'b1011: begin y_val = 3'd0; rd_out = 1'b1; disp4_err = rd_mid;  end
      4'b0100: begin y_val = 3'd0; rd_out = 1'b0; disp4_err = ~rd_mid; end

      4'b1001: begin
        rd_out = rd_mid;
        if (is_k28 && !rd_mid) y_val = 3'd6;
        else                   y_val = 3'd1;
      end
      4'b0110: begin
        rd_out = rd_mid;
        if (is_k28 && !rd_mid) y_val = 3'd1;
        else                   y_val = 3'd6;
      end

      4'b1010: begin
        rd_out = rd_mid;
        if (is_k28 && !rd_mid) y_val = 3'd2;
        else                   y_val = 3'd5;
      end
      4'b0101: begin
        rd_out = rd_mid;
        if (is_k28 && !rd_mid) y_val = 3'd5;
        else                   y_val = 3'd2;
      end

      4'b1100: begin y_val = 3'd3; rd_out = 1'b0; disp4_err = rd_mid;  end
      4'b0011: begin y_val = 3'd3; rd_out = 1'b1; disp4_err = ~rd_mid; end

      4'b1101: begin y_val = 3'd4; rd_out = 1'b1; disp4_err = rd_mid;  end
      4'b0010: begin y_val = 3'd4; rd_out = 1'b0; disp4_err = ~rd_mid; end

      4'b1110: begin
        y_val = 3'd7; rd_out = 1'b1; disp4_err = rd_mid;
        if (is_k28) c4_ok = 1'b0;
      end
      4'b0001: begin
        y_val = 3'd7; rd_out = 1'b0; disp4_err = ~rd_mid;
        if (is_k28) c4_ok = 1'b0;
      end

      4'b0111: begin
        y_val = 3'd7; rd_out = 1'b1; disp4_err = rd_mid;
        if (is_k28 || x_val == 5'd23 || x_val == 5'd27 || x_val == 5'd29 || x_val == 5'd30) begin
          k4 = 1'b1;
        end else if (x_val == 5'd17 || x_val == 5'd18 || x_val == 5'd20) begin
          k4 = 1'b0;
        end else begin
          c4_ok = 1'b0;
        end
      end
      4'b1000: begin
        y_val = 3'd7; rd_out = 1'b0; disp4_err = ~rd_mid;
        if (is_k28 || x_val == 5'd23 || x_val == 5'd27 || x_val == 5'd29 || x_val == 5'd30) begin
          k4 = 1'b1;
        end else if (x_val == 5'd11 || x_val == 5'd13 || x_val == 5'd14) begin
          k4 = 1'b0;
        end else begin
          c4_ok = 1'b0;
        end
      end

      default: begin c4_ok = 1'b0; end
    endcase

    res.data     = {y_val, x_val};
    res.k        = k4;
    res.code_err = !(c6_ok && c4_ok);
    res.disp_err = (disp6_err || disp4_err);
    res.rd_out   = rd_out;
    return res;
  endfunction

  // ---------------------------------------------------------------------------
  // TX Encoder Sequential Logic
  // ---------------------------------------------------------------------------
  logic       tx_rd_q;
  enc_res_t   tx_enc_w;

  always_comb begin
    tx_enc_w = encode_8b10b(tx_data_i, tx_k_i, tx_rd_clr_i ? 1'b0 : tx_rd_q);
  end

  always_ff @(posedge tx_clk_i or negedge tx_rst_ni) begin
    if (!tx_rst_ni) begin
      tx_rd_q    <= 1'b0;
      tx_10b_o   <= 10'd0;
      tx_valid_o <= 1'b0;
    end else begin
      tx_valid_o <= tx_valid_i;
      if (tx_rd_clr_i) begin
        tx_rd_q <= 1'b0;
      end else if (tx_valid_i) begin
        tx_10b_o <= tx_enc_w.code10;
        tx_rd_q  <= tx_enc_w.rd_out;
      end
    end
  end

  assign tx_rd_o = tx_rd_q;

  // ---------------------------------------------------------------------------
  // RX Comma (K28.5) 10-bit Word Aligner + 8b/10b Decoder
  // ---------------------------------------------------------------------------
  localparam logic [9:0] COMMA_RD0 = 10'b0011111010; // K28.5 RD-
  localparam logic [9:0] COMMA_RD1 = 10'b1100000101; // K28.5 RD+

  logic [9:0]  prev_10b_q;
  logic [3:0]  align_off_q;
  logic        aligned_q;
  logic [19:0] win_20b;
  logic        comma_found;
  logic [3:0]  comma_off;
  logic [3:0]  eff_off;
  logic [9:0]  aligned_10b;
  logic        rx_rd_q;
  dec_res_t    rx_dec_w;

  assign win_20b = {prev_10b_q, rx_raw_10b_i};

  always_comb begin
    comma_found = 1'b0;
    comma_off   = 4'd0;
    for (int s = 0; s < 10; s++) begin
      automatic logic [9:0] cand;
      cand = win_20b[19 - s -: 10];
      if (!comma_found && (cand == COMMA_RD0 || cand == COMMA_RD1)) begin
        comma_found = 1'b1;
        comma_off   = 4'(s);
      end
    end

    eff_off = (align_en_i && comma_found) ? comma_off : align_off_q;
    if (align_bypass_i) begin
      aligned_10b = rx_raw_10b_i;
    end else begin
      automatic logic [19:0] shifted_20b;
      shifted_20b = win_20b >> (5'd10 - {1'b0, eff_off});
      aligned_10b = shifted_20b[9:0];
    end

    rx_dec_w = decode_8b10b(aligned_10b, rx_rd_clr_i ? 1'b0 : rx_rd_q);
  end

  always_ff @(posedge rx_clk_i or negedge rx_rst_ni) begin
    if (!rx_rst_ni) begin
      prev_10b_q       <= 10'd0;
      align_off_q      <= 4'd0;
      aligned_q        <= 1'b0;
      rx_rd_q          <= 1'b0;
      rx_valid_o       <= 1'b0;
      rx_data_o        <= 8'd0;
      rx_k_o           <= 1'b0;
      rx_disp_err_o    <= 1'b0;
      rx_code_err_o    <= 1'b0;
      rx_comma_o       <= 1'b0;
      rx_bitslip_o     <= 1'b0;
      rx_aligned_10b_o <= 10'd0;
    end else begin
      rx_valid_o   <= rx_valid_i;
      rx_comma_o   <= 1'b0;
      rx_bitslip_o <= 1'b0;

      if (rx_rd_clr_i) begin
        rx_rd_q <= 1'b0;
      end

      if (rx_valid_i) begin
        prev_10b_q <= rx_raw_10b_i;

        if (align_en_i && !align_bypass_i && comma_found) begin
          if (align_off_q != comma_off) begin
            rx_bitslip_o <= 1'b1;
          end
          align_off_q <= comma_off;
          aligned_q   <= 1'b1;
          rx_comma_o  <= 1'b1;
        end else if (align_bypass_i && (rx_raw_10b_i == COMMA_RD0 || rx_raw_10b_i == COMMA_RD1)) begin
          aligned_q  <= 1'b1;
          rx_comma_o <= 1'b1;
        end

        rx_data_o        <= rx_dec_w.data;
        rx_k_o           <= rx_dec_w.k;
        rx_disp_err_o    <= rx_dec_w.disp_err;
        rx_code_err_o    <= rx_dec_w.code_err;
        rx_aligned_10b_o <= aligned_10b;
        if (!rx_rd_clr_i) begin
          rx_rd_q <= rx_dec_w.rd_out;
        end
      end
    end
  end

  assign rx_aligned_o = aligned_q;

endmodule : serdes_8b10b
