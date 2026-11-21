// SPDX-License-Identifier: Apache-2.0
// pdm_cic_hpf.sv - 3rd-Order Sinc3 CIC Decimation Filter + Exact DC-Offset High-Pass Blocker
//
// Decimation ratio OSR: 0->16, 1->32, 2->64, 3->128.
// Input 1-bit PDM stream maps {0,1} -> {-1,+1}.
// Outputs 16-bit signed PCM with programmable gain right-shift and zero-bias IIR DC blocker.

module pdm_cic_hpf (
  input  logic               clk_i,
  input  logic               rst_ni,
  input  logic               clr_i,

  input  logic               sample_en_i,
  input  logic               pdm_bit_i,

  input  logic [1:0]         osr_sel_i,
  input  logic [1:0]         gain_rshift_i,
  input  logic               hpf_en_i,
  input  logic [3:0]         hpf_shift_i,

  output logic               pcm_valid_o,
  output logic signed [15:0] pcm_data_o,
  output logic signed [15:0] dc_est_o
);

  logic [6:0] osr_max_cnt;
  always_comb begin
    unique case (osr_sel_i)
      2'd0:    osr_max_cnt = 7'd15;  // OSR = 16
      2'd1:    osr_max_cnt = 7'd31;  // OSR = 32
      2'd2:    osr_max_cnt = 7'd63;  // OSR = 64
      default: osr_max_cnt = 7'd127; // OSR = 128
    endcase
  end

  // 24-bit signed modulo arithmetic: max R^3 = 128^3 = 2^21 (fits cleanly in signed 24b)
  logic signed [23:0] int1_q, int2_q, int3_q;
  logic signed [23:0] d1_q, d2_q, d3_q;
  logic [6:0]         decim_cnt_q;
  logic               comb_trig_q;
  logic               hpf_trig_q;

  logic signed [23:0] scaled_pcm_q;
  logic signed [35:0] dc_acc_q;
  logic signed [15:0] pcm_data_q;
  logic               pcm_valid_q;

  logic [3:0] eff_hpf_shift;
  assign eff_hpf_shift = (hpf_shift_i == 4'd0) ? 4'd1 : hpf_shift_i;

  logic signed [35:0] dc_acc_shifted;
  assign dc_acc_shifted = dc_acc_q >>> eff_hpf_shift;

  logic signed [23:0] dc_est_24;
  assign dc_est_24 = dc_acc_shifted[23:0];

  assign pcm_valid_o = pcm_valid_q;
  assign pcm_data_o  = pcm_data_q;
  assign dc_est_o    = (dc_est_24 > 24'sd32767)  ? 16'sh7FFF :
                       (dc_est_24 < -24'sd32768) ? 16'sh8000 : dc_est_24[15:0];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      int1_q       <= 24'sd0;
      int2_q       <= 24'sd0;
      int3_q       <= 24'sd0;
      d1_q         <= 24'sd0;
      d2_q         <= 24'sd0;
      d3_q         <= 24'sd0;
      decim_cnt_q  <= 7'd0;
      comb_trig_q  <= 1'b0;
      hpf_trig_q   <= 1'b0;
      scaled_pcm_q <= 24'sd0;
      dc_acc_q     <= 36'sd0;
      pcm_data_q   <= 16'sd0;
      pcm_valid_q  <= 1'b0;
    end else if (clr_i) begin
      int1_q       <= 24'sd0;
      int2_q       <= 24'sd0;
      int3_q       <= 24'sd0;
      d1_q         <= 24'sd0;
      d2_q         <= 24'sd0;
      d3_q         <= 24'sd0;
      decim_cnt_q  <= 7'd0;
      comb_trig_q  <= 1'b0;
      hpf_trig_q   <= 1'b0;
      scaled_pcm_q <= 24'sd0;
      dc_acc_q     <= 36'sd0;
      pcm_data_q   <= 16'sd0;
      pcm_valid_q  <= 1'b0;
    end else begin
      comb_trig_q <= 1'b0;
      hpf_trig_q  <= 1'b0;
      pcm_valid_q <= 1'b0;

      // Stage 1: 3-stage CIC integrators at PDM sample rate
      if (sample_en_i) begin
        automatic logic signed [23:0] x_in = pdm_bit_i ? 24'sd1 : -24'sd1;
        int1_q <= int1_q + x_in;
        int2_q <= int2_q + int1_q;
        int3_q <= int3_q + int2_q;

        if (decim_cnt_q >= osr_max_cnt) begin
          decim_cnt_q <= 7'd0;
          comb_trig_q <= 1'b1;
        end else begin
          decim_cnt_q <= decim_cnt_q + 7'd1;
        end
      end

      // Stage 2: 3-stage CIC differentiators (comb) + OSR normalization
      if (comb_trig_q) begin
        automatic logic signed [23:0] comb1;
        automatic logic signed [23:0] comb2;
        automatic logic signed [23:0] comb3;
        automatic logic signed [23:0] norm_pcm;
        comb1 = int3_q - d1_q;
        comb2 = comb1  - d2_q;
        comb3 = comb2  - d3_q;
        d1_q <= int3_q;
        d2_q <= comb1;
        d3_q <= comb2;

        unique case (osr_sel_i)
          2'd0:    norm_pcm = comb3 <<< 3;
          2'd1:    norm_pcm = comb3;
          2'd2:    norm_pcm = comb3 >>> 3;
          default: norm_pcm = comb3 >>> 6;
        endcase
        scaled_pcm_q <= norm_pcm >>> gain_rshift_i;
        hpf_trig_q   <= 1'b1;
      end

      // Stage 3: Zero-bias IIR DC-offset blocker & 16-bit signed saturation
      if (hpf_trig_q) begin
        automatic logic signed [23:0] hpf_out;
        hpf_out = hpf_en_i ? (scaled_pcm_q - dc_est_24) : scaled_pcm_q;
        if (hpf_en_i) begin
          dc_acc_q <= dc_acc_q + {{12{hpf_out[23]}}, hpf_out};
        end
        if (hpf_out > 24'sd32767) begin
          pcm_data_q <= 16'sh7FFF;
        end else if (hpf_out < -24'sd32768) begin
          pcm_data_q <= 16'sh8000;
        end else begin
          pcm_data_q <= hpf_out[15:0];
        end
        pcm_valid_q <= 1'b1;
      end
    end
  end

endmodule : pdm_cic_hpf
