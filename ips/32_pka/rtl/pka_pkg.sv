// SPDX-License-Identifier: Apache-2.0
// pka_pkg.sv - Package for Public-Key Cryptographic Accelerator (32_pka)
//              Supports RSA (up to 4096-bit), Curve25519/X25519, and NIST P-256.

package pka_pkg;

  // ---------------------------------------------------------------------------
  // Operation Codes (CTRL.OP)
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] {
    OP_MONT_MUL      = 4'h0, // R = A * B * R^-1 mod M
    OP_MOD_EXP       = 4'h1, // R = A^E mod M (constant-time Montgomery ladder)
    OP_MOD_ADD       = 4'h2, // R = (A + B) mod M (constant-time)
    OP_MOD_SUB       = 4'h3, // R = (A - B) mod M (constant-time)
    OP_X25519_SCALAR = 4'h4, // RFC 7748 X25519 scalar mult R = k * u
    OP_ECC_P256_PMUL = 4'h5, // NIST P-256 scalar mult (R, RY) = k * P
    OP_ECC_P256_PADD = 4'h6, // NIST P-256 point add (R, RY) = P1 + P2
    OP_ECC_P256_PDBL = 4'h7  // NIST P-256 point double (R, RY) = 2 * P1
  } pka_op_e;

  // ---------------------------------------------------------------------------
  // Internal ALU Operations (pka_mont_mul)
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    ALU_MONT_MUL = 2'd0,
    ALU_MOD_ADD  = 2'd1,
    ALU_MOD_SUB  = 2'd2,
    ALU_N0_INV   = 2'd3
  } pka_alu_op_e;

  // ---------------------------------------------------------------------------
  // Micro-sequencer Operations & ECC 256-bit Slot IDs (16 slots x 8 words)
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    UOP_MUL  = 2'd0,
    UOP_ADD  = 2'd1,
    UOP_SUB  = 2'd2,
    UOP_DONE = 2'd3
  } pka_uop_e;

  typedef enum logic [3:0] {
    S_X0 = 4'd0,  // P256: R0.X | X25519: x_2
    S_Y0 = 4'd1,  // P256: R0.Y | X25519: x_1 (clamped u)
    S_Z0 = 4'd2,  // P256: R0.Z | X25519: z_2
    S_X1 = 4'd3,  // P256: R1.X (also ECC_QX CSR) | X25519: x_3
    S_Y1 = 4'd4,  // P256: R1.Y (also ECC_QY CSR)
    S_Z1 = 4'd5,  // P256: R1.Z | X25519: z_3
    S_T0 = 4'd6,  // Scratch 0
    S_T1 = 4'd7,  // Scratch 1
    S_T2 = 4'd8,  // Scratch 2
    S_T3 = 4'd9,  // Scratch 3
    S_T4 = 4'd10, // Scratch 4
    S_T5 = 4'd11, // Scratch 5 (RCB X3)
    S_T6 = 4'd12, // Scratch 6 (RCB Y3, also ECC_RY CSR)
    S_T7 = 4'd13, // Scratch 7 (RCB Z3)
    S_C0 = 4'd14, // Constant 0 (a24_mont or b_mont)
    S_C1 = 4'd15  // Constant 1 (ONE_M / R2_M / inversion base)
  } pka_slot_e;

  typedef struct packed {
    pka_uop_e  op;
    pka_slot_e dst;
    pka_slot_e src_a;
    pka_slot_e src_b;
  } pka_uop_t;

  // ---------------------------------------------------------------------------
  // CSR Byte Offsets (4 KB window: 0x000 .. 0xFFF)
  // ---------------------------------------------------------------------------
  localparam logic [11:0] A_ID          = 12'h000;
  localparam logic [11:0] A_CAPS        = 12'h004;
  localparam logic [11:0] A_CTRL        = 12'h008;
  localparam logic [11:0] A_CMD         = 12'h00C;
  localparam logic [11:0] A_STATUS      = 12'h010;
  localparam logic [11:0] A_ERR_STAT    = 12'h014;
  localparam logic [11:0] A_NUM_WORDS   = 12'h018;
  localparam logic [11:0] A_EXP_BITS    = 12'h01C;
  localparam logic [11:0] A_N0_INV      = 12'h020;
  localparam logic [11:0] A_PERF_CYCLES = 12'h024;
  localparam logic [11:0] A_IRQ_STAT    = 12'h028;

  // Operand / Result SRAM Window Bases (each 0x200 bytes = 128 x 32-bit words)
  localparam logic [11:0] BASE_A        = 12'h200; // 0x200..0x3FF (RW)
  localparam logic [11:0] BASE_B        = 12'h400; // 0x400..0x5FF (RW)
  localparam logic [11:0] BASE_M        = 12'h600; // 0x600..0x7FF (RW)
  localparam logic [11:0] BASE_E        = 12'h800; // 0x800..0x9FF (WO, reads 0)
  localparam logic [11:0] BASE_R        = 12'hA00; // 0xA00..0xBFF (RO)
  localparam logic [11:0] BASE_R2       = 12'hC00; // 0xC00..0xDFF (RW -> T3 preloaded R^2)

  // ECC Point2 / Y-result 256-bit windows (8 words = 32 bytes each)
  localparam logic [11:0] BASE_ECC_QX   = 12'hE00; // 0xE00..0xE1F (RW)
  localparam logic [11:0] BASE_ECC_QY   = 12'hE20; // 0xE20..0xE3F (RW)
  localparam logic [11:0] BASE_ECC_RY   = 12'hE40; // 0xE40..0xE5F (RO)

  localparam logic [31:0] PKA_ID_VALUE  = 32'h504B_4101; // "PKA" v1

  // Error Flag Bits
  localparam int unsigned E_BAD_WORDS = 0;
  localparam int unsigned E_EVEN_MOD  = 1;
  localparam int unsigned E_BAD_OP    = 2;
  localparam int unsigned E_BUSY_WR   = 3;
  localparam int unsigned E_ABORTED   = 4;
  localparam int unsigned NUM_ERR_BITS = 5;

  // ---------------------------------------------------------------------------
  // Curve25519 / X25519 Constants (p = 2^255 - 19, R = 2^256)
  // ---------------------------------------------------------------------------
  localparam logic [255:0] P25519_P     = {32'h7fffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffed};
  localparam logic [255:0] P25519_PM2   = {32'h7fffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'hffffffeb};
  localparam logic [255:0] P25519_ONE_M = {32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000026};
  localparam logic [255:0] P25519_R2_M  = {32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h000005a4};
  localparam logic [255:0] P25519_A24_M = {32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00468ba6};
  localparam logic [31:0]  P25519_N0INV = 32'h286bca1b;

  // ---------------------------------------------------------------------------
  // NIST P-256 Constants (p = 2^256 - 2^224 + 2^192 + 2^96 - 1, R = 2^256)
  // ---------------------------------------------------------------------------
  localparam logic [255:0] P256_P       = {32'hffffffff, 32'h00000001, 32'h00000000, 32'h00000000, 32'h00000000, 32'hffffffff, 32'hffffffff, 32'hffffffff};
  localparam logic [255:0] P256_PM2     = {32'hffffffff, 32'h00000001, 32'h00000000, 32'h00000000, 32'h00000000, 32'hffffffff, 32'hffffffff, 32'hfffffffd};
  localparam logic [255:0] P256_ONE_M   = {32'h00000000, 32'hfffffffe, 32'hffffffff, 32'hffffffff, 32'hffffffff, 32'h00000000, 32'h00000000, 32'h00000001};
  localparam logic [255:0] P256_R2_M    = {32'h00000004, 32'hfffffffd, 32'hffffffff, 32'hfffffffe, 32'hfffffffb, 32'hffffffff, 32'h00000000, 32'h00000003};
  localparam logic [255:0] P256_B_M     = {32'hdc30061d, 32'h04874834, 32'he5a220ab, 32'hf7212ed6, 32'hacf005cd, 32'h78843090, 32'hd89cdf62, 32'h29c4bddf};
  localparam logic [255:0] P256_GX_M    = {32'h18905f76, 32'ha53755c6, 32'h79fb732b, 32'h77622510, 32'h75ba95fc, 32'h5fedb601, 32'h79e730d4, 32'h18a9143c};
  localparam logic [255:0] P256_GY_M    = {32'h8571ff18, 32'h25885d85, 32'hd2e88688, 32'hdd21f325, 32'h8b4ab8e4, 32'hba19e45c, 32'hddf25357, 32'hce95560a};
  localparam logic [31:0]  P256_N0INV   = 32'h00000001;

  // Extract 32-bit limb idx (0 = LSW) from a 256-bit constant
  function automatic logic [31:0] get_limb256(input logic [255:0] val, input logic [2:0] idx);
    return val[idx * 32 +: 32];
  endfunction

  // Helper to pack a micro-instruction
  function automatic pka_uop_t mk_uop(
    input pka_uop_e  op,
    input pka_slot_e dst,
    input pka_slot_e src_a,
    input pka_slot_e src_b
  );
    pka_uop_t u;
    u.op    = op;
    u.dst   = dst;
    u.src_a = src_a;
    u.src_b = src_b;
    return u;
  endfunction

  // ---------------------------------------------------------------------------
  // RFC 7748 X25519 Ladder Step Microcode (18 ops + DONE)
  // Inputs: X0=x2, Z0=z2, X1=x3, Z1=z3, Y0=x1, C0=a24_mont
  // ---------------------------------------------------------------------------
  function automatic pka_uop_t get_x25519_uop(input logic [5:0] pc);
    unique case (pc)
      6'd0:  return mk_uop(UOP_ADD, S_T0, S_X0, S_Z0); // A = x2 + z2
      6'd1:  return mk_uop(UOP_SUB, S_T1, S_X0, S_Z0); // B = x2 - z2
      6'd2:  return mk_uop(UOP_ADD, S_T2, S_X1, S_Z1); // C = x3 + z3
      6'd3:  return mk_uop(UOP_SUB, S_T3, S_X1, S_Z1); // D = x3 - z3
      6'd4:  return mk_uop(UOP_MUL, S_X0, S_T0, S_T0); // AA = A^2
      6'd5:  return mk_uop(UOP_MUL, S_Z0, S_T1, S_T1); // BB = B^2
      6'd6:  return mk_uop(UOP_MUL, S_X1, S_T3, S_T0); // DA = D * A
      6'd7:  return mk_uop(UOP_MUL, S_Z1, S_T2, S_T1); // CB = C * B
      6'd8:  return mk_uop(UOP_ADD, S_T0, S_X1, S_Z1); // DA + CB
      6'd9:  return mk_uop(UOP_SUB, S_T1, S_X1, S_Z1); // DA - CB
      6'd10: return mk_uop(UOP_MUL, S_X1, S_T0, S_T0); // x3' = (DA + CB)^2
      6'd11: return mk_uop(UOP_MUL, S_T0, S_T1, S_T1); // (DA - CB)^2
      6'd12: return mk_uop(UOP_MUL, S_Z1, S_Y0, S_T0); // z3' = x1 * (DA - CB)^2
      6'd13: return mk_uop(UOP_SUB, S_T0, S_X0, S_Z0); // E = AA - BB
      6'd14: return mk_uop(UOP_MUL, S_T1, S_C0, S_T0); // a24 * E
      6'd15: return mk_uop(UOP_ADD, S_T2, S_X0, S_T1); // AA + a24 * E
      6'd16: return mk_uop(UOP_MUL, S_X0, S_X0, S_Z0); // x2' = AA * BB
      6'd17: return mk_uop(UOP_MUL, S_Z0, S_T0, S_T2); // z2' = E * (AA + a24 * E)
      default: return mk_uop(UOP_DONE, S_X0, S_X0, S_X0);
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // NIST P-256 RCB Complete Addition Microcode (43 ops + DONE)
  // Inputs: (X0,Y0,Z0), (X1,Y1,Z1) [or (X0,Y0,Z0) twice when dbl_mode=1], C0=b_mont
  // Outputs: T5=X3, T6=Y3, T7=Z3
  // ---------------------------------------------------------------------------
  function automatic pka_uop_t get_p256_add_uop(input logic [5:0] pc, input logic dbl_mode);
    automatic pka_slot_e x2s = dbl_mode ? S_X0 : S_X1;
    automatic pka_slot_e y2s = dbl_mode ? S_Y0 : S_Y1;
    automatic pka_slot_e z2s = dbl_mode ? S_Z0 : S_Z1;
    unique case (pc)
      6'd0:  return mk_uop(UOP_MUL, S_T0, S_X0, x2s);  // t0 = X1 * X2
      6'd1:  return mk_uop(UOP_MUL, S_T1, S_Y0, y2s);  // t1 = Y1 * Y2
      6'd2:  return mk_uop(UOP_MUL, S_T2, S_Z0, z2s);  // t2 = Z1 * Z2
      6'd3:  return mk_uop(UOP_ADD, S_T3, S_X0, S_Y0); // t3 = X1 + Y1
      6'd4:  return mk_uop(UOP_ADD, S_T4, x2s,  y2s);  // t4 = X2 + Y2
      6'd5:  return mk_uop(UOP_MUL, S_T3, S_T3, S_T4); // t3 = t3 * t4
      6'd6:  return mk_uop(UOP_ADD, S_T4, S_T0, S_T1); // t4 = t0 + t1
      6'd7:  return mk_uop(UOP_SUB, S_T3, S_T3, S_T4); // t3 = t3 - t4
      6'd8:  return mk_uop(UOP_ADD, S_T4, S_Y0, S_Z0); // t4 = Y1 + Z1
      6'd9:  return mk_uop(UOP_ADD, S_T5, y2s,  z2s);  // X3 = Y2 + Z2
      6'd10: return mk_uop(UOP_MUL, S_T4, S_T4, S_T5); // t4 = t4 * X3
      6'd11: return mk_uop(UOP_ADD, S_T5, S_T1, S_T2); // X3 = t1 + t2
      6'd12: return mk_uop(UOP_SUB, S_T4, S_T4, S_T5); // t4 = t4 - X3
      6'd13: return mk_uop(UOP_ADD, S_T5, S_X0, S_Z0); // X3 = X1 + Z1
      6'd14: return mk_uop(UOP_ADD, S_T6, x2s,  z2s);  // Y3 = X2 + Z2
      6'd15: return mk_uop(UOP_MUL, S_T5, S_T5, S_T6); // X3 = X3 * Y3
      6'd16: return mk_uop(UOP_ADD, S_T6, S_T0, S_T2); // Y3 = t0 + t2
      6'd17: return mk_uop(UOP_SUB, S_T6, S_T5, S_T6); // Y3 = X3 - Y3
      6'd18: return mk_uop(UOP_MUL, S_T7, S_C0, S_T2); // Z3 = b * t2
      6'd19: return mk_uop(UOP_SUB, S_T5, S_T6, S_T7); // X3 = Y3 - Z3
      6'd20: return mk_uop(UOP_ADD, S_T7, S_T5, S_T5); // Z3 = X3 + X3
      6'd21: return mk_uop(UOP_ADD, S_T5, S_T5, S_T7); // X3 = X3 + Z3
      6'd22: return mk_uop(UOP_SUB, S_T7, S_T1, S_T5); // Z3 = t1 - X3
      6'd23: return mk_uop(UOP_ADD, S_T5, S_T1, S_T5); // X3 = t1 + X3
      6'd24: return mk_uop(UOP_MUL, S_T6, S_C0, S_T6); // Y3 = b * Y3
      6'd25: return mk_uop(UOP_ADD, S_T1, S_T2, S_T2); // t1 = t2 + t2
      6'd26: return mk_uop(UOP_ADD, S_T2, S_T1, S_T2); // t2 = t1 + t2
      6'd27: return mk_uop(UOP_SUB, S_T6, S_T6, S_T2); // Y3 = Y3 - t2
      6'd28: return mk_uop(UOP_SUB, S_T6, S_T6, S_T0); // Y3 = Y3 - t0
      6'd29: return mk_uop(UOP_ADD, S_T1, S_T6, S_T6); // t1 = Y3 + Y3
      6'd30: return mk_uop(UOP_ADD, S_T6, S_T1, S_T6); // Y3 = t1 + Y3
      6'd31: return mk_uop(UOP_ADD, S_T1, S_T0, S_T0); // t1 = t0 + t0
      6'd32: return mk_uop(UOP_ADD, S_T0, S_T1, S_T0); // t0 = t1 + t0
      6'd33: return mk_uop(UOP_SUB, S_T0, S_T0, S_T2); // t0 = t0 - t2
      6'd34: return mk_uop(UOP_MUL, S_T1, S_T4, S_T6); // t1 = t4 * Y3
      6'd35: return mk_uop(UOP_MUL, S_T2, S_T0, S_T6); // t2 = t0 * Y3
      6'd36: return mk_uop(UOP_MUL, S_T6, S_T5, S_T7); // Y3 = X3 * Z3
      6'd37: return mk_uop(UOP_ADD, S_T6, S_T6, S_T2); // Y3 = Y3 + t2
      6'd38: return mk_uop(UOP_MUL, S_T5, S_T3, S_T5); // X3 = t3 * X3
      6'd39: return mk_uop(UOP_SUB, S_T5, S_T5, S_T1); // X3 = X3 - t1
      6'd40: return mk_uop(UOP_MUL, S_T7, S_T4, S_T7); // Z3 = t4 * Z3
      6'd41: return mk_uop(UOP_MUL, S_T1, S_T3, S_T0); // t1 = t3 * t0
      6'd42: return mk_uop(UOP_ADD, S_T7, S_T7, S_T1); // Z3 = Z3 + t1
      default: return mk_uop(UOP_DONE, S_T5, S_T5, S_T5);
    endcase
  endfunction

endpackage : pka_pkg
