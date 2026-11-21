// SPDX-License-Identifier: Apache-2.0
// otp_trng_keymgr_pkg.sv - Shared types, SECDED Hamming(39,32) ECC, and 256-bit ARX permutation
//                          for ips/33_otp_trng_keymgr.

package otp_trng_keymgr_pkg;

  localparam int unsigned OTP_WORDS  = 64;
  localparam int unsigned OTP_ADDR_W = 6; // $clog2(OTP_WORDS)

  // Partition Word Ranges (64 x 32-bit logical words, 39-bit stored w/ SECDED ECC)
  localparam logic [OTP_ADDR_W-1:0] PART_UID_BASE       = 6'd0;
  localparam logic [OTP_ADDR_W-1:0] PART_UID_END        = 6'd7;
  localparam logic [OTP_ADDR_W-1:0] PART_SW_CFG_BASE    = 6'd8;
  localparam logic [OTP_ADDR_W-1:0] PART_SW_CFG_END     = 6'd23;
  localparam logic [OTP_ADDR_W-1:0] PART_ROOT_KEY0_BASE = 6'd24;
  localparam logic [OTP_ADDR_W-1:0] PART_ROOT_KEY0_END  = 6'd31;
  localparam logic [OTP_ADDR_W-1:0] PART_ROOT_KEY1_BASE = 6'd32;
  localparam logic [OTP_ADDR_W-1:0] PART_ROOT_KEY1_END  = 6'd39;
  localparam logic [OTP_ADDR_W-1:0] PART_LOCK_BASE      = 6'd40;
  localparam logic [OTP_ADDR_W-1:0] PART_LOCK_END       = 6'd47;
  localparam logic [OTP_ADDR_W-1:0] PART_LC_BASE        = 6'd48;
  localparam logic [OTP_ADDR_W-1:0] PART_LC_END         = 6'd55;
  localparam logic [OTP_ADDR_W-1:0] PART_USER_BASE      = 6'd56;
  localparam logic [OTP_ADDR_W-1:0] PART_USER_END       = 6'd63;

  // Individual Lock Word Indices inside PART_LOCK (one word per lock -> 100% SECDED ECC valid)
  localparam logic [OTP_ADDR_W-1:0] LOCK_IDX_UID_W      = 6'd40;
  localparam logic [OTP_ADDR_W-1:0] LOCK_IDX_SW_W       = 6'd41;
  localparam logic [OTP_ADDR_W-1:0] LOCK_IDX_SW_R       = 6'd42;
  localparam logic [OTP_ADDR_W-1:0] LOCK_IDX_KEY0       = 6'd43;
  localparam logic [OTP_ADDR_W-1:0] LOCK_IDX_KEY1       = 6'd44;
  localparam logic [OTP_ADDR_W-1:0] LOCK_IDX_SELF       = 6'd45;

  // Individual Lifecycle Word Indices inside PART_LC (monotonic bit-OR per word)
  localparam logic [OTP_ADDR_W-1:0] LC_IDX_TEST         = 6'd48;
  localparam logic [OTP_ADDR_W-1:0] LC_IDX_DEV          = 6'd49;
  localparam logic [OTP_ADDR_W-1:0] LC_IDX_PROD         = 6'd50;
  localparam logic [OTP_ADDR_W-1:0] LC_IDX_RMA          = 6'd51;

  localparam logic [31:0] LC_FUSE_TOKEN   = 32'hA5A5_5A5A;
  localparam logic [31:0] LOCK_FUSE_TOKEN = 32'h5A5A_A5A5;
  localparam logic [15:0] LC_CMD_MAGIC    = 16'h4C43; // "LC"

  // Lifecycle States (Monotonic progression: RAW -> TEST -> DEV -> PROD -> RMA)
  typedef enum logic [2:0] {
    LC_RAW  = 3'd0,
    LC_TEST = 3'd1,
    LC_DEV  = 3'd2,
    LC_PROD = 3'd3,
    LC_RMA  = 3'd4
  } lc_state_e;

  // Extended Hamming(38,32) + Overall Parity = SECDED(39,32)
  // Parity bits occupy 1-indexed positions {1, 2, 4, 8, 16, 32}; overall parity is bit 38.
  localparam logic [5:0] SECDED_DATA_POS [0:31] = '{
    6'd3,
    6'd5,  6'd6,  6'd7,
    6'd9,  6'd10, 6'd11, 6'd12, 6'd13, 6'd14, 6'd15,
    6'd17, 6'd18, 6'd19, 6'd20, 6'd21, 6'd22, 6'd23, 6'd24,
    6'd25, 6'd26, 6'd27, 6'd28, 6'd29, 6'd30, 6'd31,
    6'd33, 6'd34, 6'd35, 6'd36, 6'd37, 6'd38
  };

  function automatic logic [6:0] secded_parity(input logic [31:0] d);
    logic [6:0] p;
    p = 7'd0;
    for (int i = 0; i < 32; i++) begin
      if (SECDED_DATA_POS[i][0]) p[0] ^= d[i];
      if (SECDED_DATA_POS[i][1]) p[1] ^= d[i];
      if (SECDED_DATA_POS[i][2]) p[2] ^= d[i];
      if (SECDED_DATA_POS[i][3]) p[3] ^= d[i];
      if (SECDED_DATA_POS[i][4]) p[4] ^= d[i];
      if (SECDED_DATA_POS[i][5]) p[5] ^= d[i];
    end
    p[6] = (^d) ^ (^p[5:0]);
    return p;
  endfunction

  function automatic logic [38:0] secded_encode(input logic [31:0] d);
    return {secded_parity(d), d};
  endfunction

  function automatic void secded_decode(
    input  logic [38:0] cw,
    output logic [31:0] data_o,
    output logic        single_err_o,
    output logic        double_err_o
  );
    logic [31:0] rx_d;
    logic [6:0]  rx_p, exp_p;
    logic [5:0]  syn;
    logic        par;
    rx_d = cw[31:0];
    rx_p = cw[38:32];
    exp_p = secded_parity(rx_d);
    syn   = rx_p[5:0] ^ exp_p[5:0];
    par   = (^rx_d) ^ (^rx_p);

    data_o       = rx_d;
    single_err_o = 1'b0;
    double_err_o = 1'b0;

    if ((syn == 6'd0) && (par == 1'b0)) begin
      // Valid codeword
      single_err_o = 1'b0;
      double_err_o = 1'b0;
    end else if (par == 1'b1) begin
      if (syn > 6'd38) begin
        double_err_o = 1'b1;
      end else begin
        single_err_o = 1'b1;
        for (int i = 0; i < 32; i++) begin
          if (syn == SECDED_DATA_POS[i]) begin
            data_o[i] = ~rx_d[i];
          end
        end
      end
    end else begin
      // Non-zero syndrome with even overall parity => 2-bit error
      double_err_o = 1'b1;
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Clean-room 256-bit ARX + Lane-Rotation Permutation (1 round per call)
  // Used iteratively by trng_cond (sponge conditioner) and key_vault (KDF).
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] rotl32(input logic [31:0] x, input int unsigned n);
    return (x << n) | (x >> (32 - n));
  endfunction

  function automatic logic [255:0] perm256_round(
    input logic [255:0] st_i,
    input logic [3:0]   round_i
  );
    logic [31:0] w [0:7];
    logic [31:0] a0, b0, c0, d0;
    logic [31:0] a1, b1, c1, d1;
    logic [31:0] rc;
    logic [255:0] st_o;

    for (int i = 0; i < 8; i++) begin
      w[i] = st_i[i*32 +: 32];
    end

    rc = 32'h9E37_79B9 ^ ({28'd0, round_i} * 32'h0101_0101);

    // Lane 0 ARX quarter-round
    a0 = w[0] + rc;
    b0 = w[1];
    c0 = w[2];
    d0 = w[3];
    a0 = a0 + b0; d0 = rotl32(d0 ^ a0, 16);
    c0 = c0 + d0; b0 = rotl32(b0 ^ c0, 12);
    a0 = a0 + b0; d0 = rotl32(d0 ^ a0, 8);
    c0 = c0 + d0; b0 = rotl32(b0 ^ c0, 7);

    // Lane 1 ARX quarter-round
    a1 = w[4] + (~rc);
    b1 = w[5];
    c1 = w[6];
    d1 = w[7];
    a1 = a1 + b1; d1 = rotl32(d1 ^ a1, 16);
    c1 = c1 + d1; b1 = rotl32(b1 ^ c1, 12);
    a1 = a1 + b1; d1 = rotl32(d1 ^ a1, 8);
    c1 = c1 + d1; b1 = rotl32(b1 ^ c1, 7);

    // Cross-lane cyclic word permutation for full 256-bit diffusion
    st_o[0*32 +: 32] = b0;
    st_o[1*32 +: 32] = c1;
    st_o[2*32 +: 32] = d0;
    st_o[3*32 +: 32] = a1;
    st_o[4*32 +: 32] = b1;
    st_o[5*32 +: 32] = c0;
    st_o[6*32 +: 32] = d1;
    st_o[7*32 +: 32] = a0;

    return st_o;
  endfunction

endpackage : otp_trng_keymgr_pkg
