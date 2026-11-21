// SPDX-License-Identifier: Apache-2.0
// fft_twiddle_rom.sv - Quarter-Wave Compressed Twiddle Factor ROM ((MAX_N/4)+1 entries)
//
// Stores cos(2*pi*i / MAX_N) scaled by 32768 (unsigned 16-bit [0..32768], exact 1.0 = 16'h8000)
// for the first quadrant i in [0 .. MAX_N/4] (4x compression vs full-wave cos, 8x vs full cos+sin).
// Reconstructs full-circle W_N^{+k} = cos(theta) - j*sin(theta) (forward) and
// W_N^{-k} = cos(theta) + j*sin(theta) (inverse) via quadrant sign/swap symmetry.
// Output is registered (1-cycle synchronous read latency matching working SRAM read latency).

module fft_twiddle_rom #(
  parameter int unsigned MAX_LOG2_N = 10 // Supported range: 4 .. 10 (N up to 1024)
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  logic                    en_i,
  input  logic                    inverse_i, // 0: W_N^{+k} (cos - j*sin), 1: W_N^{-k} (cos + j*sin)
  input  logic [MAX_LOG2_N-1:0]   angle_i,   // Full-circle angle index 0 .. MAX_N-1
  output logic signed [16:0]      tw_re_o,   // Signed 17-bit Q2.15 in [-32768, +32768]
  output logic signed [16:0]      tw_im_o    // Signed 17-bit Q2.15 in [-32768, +32768]
);

  localparam int unsigned Q_LOG2   = MAX_LOG2_N - 2;
  localparam int unsigned Q_DEPTH  = (1 << Q_LOG2); // MAX_N / 4
  localparam int unsigned Q_STRIDE = (1 << (10 - MAX_LOG2_N));

  // Master quarter-wave cosine table for MAX_LOG2_N = 10: round(32768 * cos(2*pi*i/1024))
  // Synthesis constant-folds sub-sampled lookups when MAX_LOG2_N < 10 to exact (MAX_N/4)+1 entries.
  localparam logic [15:0] COS_Q10 [0:256] = '{
    16'd32768, 16'd32767, 16'd32766, 16'd32762, 16'd32758, 16'd32753, 16'd32746, 16'd32738,
    16'd32729, 16'd32718, 16'd32706, 16'd32693, 16'd32679, 16'd32664, 16'd32647, 16'd32629,
    16'd32610, 16'd32590, 16'd32568, 16'd32546, 16'd32522, 16'd32496, 16'd32470, 16'd32442,
    16'd32413, 16'd32383, 16'd32352, 16'd32319, 16'd32286, 16'd32251, 16'd32214, 16'd32177,
    16'd32138, 16'd32099, 16'd32058, 16'd32015, 16'd31972, 16'd31927, 16'd31881, 16'd31834,
    16'd31786, 16'd31737, 16'd31686, 16'd31634, 16'd31581, 16'd31527, 16'd31471, 16'd31415,
    16'd31357, 16'd31298, 16'd31238, 16'd31177, 16'd31114, 16'd31050, 16'd30986, 16'd30920,
    16'd30853, 16'd30784, 16'd30715, 16'd30644, 16'd30572, 16'd30499, 16'd30425, 16'd30350,
    16'd30274, 16'd30196, 16'd30118, 16'd30038, 16'd29957, 16'd29875, 16'd29792, 16'd29707,
    16'd29622, 16'd29535, 16'd29448, 16'd29359, 16'd29269, 16'd29178, 16'd29086, 16'd28993,
    16'd28899, 16'd28803, 16'd28707, 16'd28610, 16'd28511, 16'd28411, 16'd28311, 16'd28209,
    16'd28106, 16'd28002, 16'd27897, 16'd27791, 16'd27684, 16'd27576, 16'd27467, 16'd27357,
    16'd27246, 16'd27133, 16'd27020, 16'd26906, 16'd26791, 16'd26674, 16'd26557, 16'd26439,
    16'd26320, 16'd26199, 16'd26078, 16'd25956, 16'd25833, 16'd25708, 16'd25583, 16'd25457,
    16'd25330, 16'd25202, 16'd25073, 16'd24943, 16'd24812, 16'd24680, 16'd24548, 16'd24414,
    16'd24279, 16'd24144, 16'd24008, 16'd23870, 16'd23732, 16'd23593, 16'd23453, 16'd23312,
    16'd23170, 16'd23028, 16'd22884, 16'd22740, 16'd22595, 16'd22449, 16'd22302, 16'd22154,
    16'd22006, 16'd21856, 16'd21706, 16'd21555, 16'd21403, 16'd21251, 16'd21097, 16'd20943,
    16'd20788, 16'd20632, 16'd20475, 16'd20318, 16'd20160, 16'd20001, 16'd19841, 16'd19681,
    16'd19520, 16'd19358, 16'd19195, 16'd19032, 16'd18868, 16'd18703, 16'd18538, 16'd18372,
    16'd18205, 16'd18037, 16'd17869, 16'd17700, 16'd17531, 16'd17361, 16'd17190, 16'd17018,
    16'd16846, 16'd16673, 16'd16500, 16'd16326, 16'd16151, 16'd15976, 16'd15800, 16'd15624,
    16'd15447, 16'd15269, 16'd15091, 16'd14912, 16'd14733, 16'd14553, 16'd14373, 16'd14192,
    16'd14010, 16'd13828, 16'd13646, 16'd13463, 16'd13279, 16'd13095, 16'd12910, 16'd12725,
    16'd12540, 16'd12354, 16'd12167, 16'd11980, 16'd11793, 16'd11605, 16'd11417, 16'd11228,
    16'd11039, 16'd10850, 16'd10660, 16'd10469, 16'd10279, 16'd10088, 16'd 9896, 16'd 9704,
    16'd 9512, 16'd 9319, 16'd 9127, 16'd 8933, 16'd 8740, 16'd 8546, 16'd 8351, 16'd 8157,
    16'd 7962, 16'd 7767, 16'd 7571, 16'd 7376, 16'd 7180, 16'd 6983, 16'd 6787, 16'd 6590,
    16'd 6393, 16'd 6195, 16'd 5998, 16'd 5800, 16'd 5602, 16'd 5404, 16'd 5205, 16'd 5007,
    16'd 4808, 16'd 4609, 16'd 4410, 16'd 4211, 16'd 4011, 16'd 3812, 16'd 3612, 16'd 3412,
    16'd 3212, 16'd 3012, 16'd 2811, 16'd 2611, 16'd 2411, 16'd 2210, 16'd 2009, 16'd 1809,
    16'd 1608, 16'd 1407, 16'd 1206, 16'd 1005, 16'd  804, 16'd  603, 16'd  402, 16'd  201,
    16'd    0
  };

  // Parameterized quarter-wave ROM slice of exact size (MAX_N/4)+1
  function automatic logic [15:0] qrom_lookup(input logic [Q_LOG2:0] idx);
    automatic logic [8:0] master_idx;
    master_idx = 9'(idx) * 9'(Q_STRIDE);
    return COS_Q10[master_idx];
  endfunction

  // Combinational quadrant decomposition + symmetry reconstruction
  logic signed [16:0] tw_re_d;
  logic signed [16:0] tw_im_d;

  always_comb begin
    automatic logic [1:0]        quad;
    automatic logic [Q_LOG2-1:0] off;
    automatic logic [Q_LOG2:0]   idx_c;
    automatic logic [Q_LOG2:0]   idx_s;
    automatic logic [15:0]       c_raw;
    automatic logic [15:0]       s_raw;
    automatic logic signed [16:0] c_pos;
    automatic logic signed [16:0] s_pos;
    automatic logic signed [16:0] cos_val;
    automatic logic signed [16:0] sin_val;

    quad  = angle_i[MAX_LOG2_N-1 : MAX_LOG2_N-2];
    off   = angle_i[Q_LOG2-1:0];
    idx_c = {1'b0, off};
    idx_s = (Q_LOG2+1)'(Q_DEPTH) - {1'b0, off};

    c_raw = qrom_lookup(idx_c);
    s_raw = qrom_lookup(idx_s);
    c_pos = $signed({1'b0, c_raw});
    s_pos = $signed({1'b0, s_raw});

    unique case (quad)
      2'd0: begin // [0, pi/2)
        cos_val =  c_pos;
        sin_val =  s_pos;
      end
      2'd1: begin // [pi/2, pi)
        cos_val = -s_pos;
        sin_val =  c_pos;
      end
      2'd2: begin // [pi, 3*pi/2)
        cos_val = -c_pos;
        sin_val = -s_pos;
      end
      default: begin // [3*pi/2, 2*pi)
        cos_val =  s_pos;
        sin_val = -c_pos;
      end
    endcase

    tw_re_d = cos_val;
    tw_im_d = inverse_i ? sin_val : -sin_val;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tw_re_o <= 17'sd32768; // 1.0 + j*0.0
      tw_im_o <= 17'sd0;
    end else if (en_i) begin
      tw_re_o <= tw_re_d;
      tw_im_o <= tw_im_d;
    end
  end

endmodule : fft_twiddle_rom
