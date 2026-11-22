// SPDX-License-Identifier: Apache-2.0
// codec_pkg.sv - Shared Types, CSR Offsets, DCT/Quant/ZigZag LUTs for 46_lz4_jpeg_codec

package codec_pkg;

  // IP Identifier ("LZJP") & Version
  localparam logic [31:0] CODEC_ID_VALUE  = 32'h4C5A_4A50;
  localparam logic [3:0]  CODEC_VERSION   = 4'd1;

  // Engine Operating Modes
  typedef enum logic [1:0] {
    MODE_LZ4_ENC  = 2'd0,
    MODE_LZ4_DEC  = 2'd1,
    MODE_JPEG_8X8 = 2'd2,
    MODE_RSVD     = 2'd3
  } codec_mode_e;

  // JPEG Quantization Table Selection
  typedef enum logic [1:0] {
    QTBL_LUMA_Q50   = 2'd0,
    QTBL_CHROMA_Q50 = 2'd1,
    QTBL_CUSTOM     = 2'd2,
    QTBL_RSVD       = 2'd3
  } jpeg_qtbl_sel_e;

  // CSR Byte Offsets (12-bit offset within IP aperture)
  localparam logic [11:0] A_ID          = 12'h000;
  localparam logic [11:0] A_CAPS        = 12'h004;
  localparam logic [11:0] A_CTRL        = 12'h008;
  localparam logic [11:0] A_CMD         = 12'h00C;
  localparam logic [11:0] A_STATUS      = 12'h010;
  localparam logic [11:0] A_ERR_STAT    = 12'h014;
  localparam logic [11:0] A_IRQ_EN      = 12'h018;
  localparam logic [11:0] A_IRQ_STAT    = 12'h01C;
  localparam logic [11:0] A_FIFO_CFG    = 12'h020;
  localparam logic [11:0] A_IN_LEN      = 12'h024;
  localparam logic [11:0] A_DIN         = 12'h030;
  localparam logic [11:0] A_DIN_LAST    = 12'h034;
  localparam logic [11:0] A_DIN_BYTE    = 12'h038;
  localparam logic [11:0] A_DOUT        = 12'h040;
  localparam logic [11:0] A_DOUT_INFO   = 12'h044;
  localparam logic [11:0] A_BYTES_IN    = 12'h050;
  localparam logic [11:0] A_BYTES_OUT   = 12'h054;
  localparam logic [11:0] A_JPEG_STAT   = 12'h058;
  localparam logic [11:0] A_PERF_CYCLES = 12'h05C;

  // 64-entry Custom Quantization Table SRAM window: 0x200..0x2FC
  localparam logic [11:0] A_QTBL_BASE   = 12'h200;
  localparam logic [11:0] A_QTBL_END    = 12'h2FC;

  // Error Status Bit Positions
  localparam int unsigned ERR_BIT_BAD_OFFSET = 0;
  localparam int unsigned ERR_BIT_TRUNCATED  = 1;
  localparam int unsigned ERR_BIT_LIT_OVF    = 2;
  localparam int unsigned ERR_BIT_IN_OVF     = 3;
  localparam int unsigned ERR_BIT_JPEG_ALIGN = 4;

  // IRQ Bit Positions
  localparam int unsigned IRQ_BIT_DONE   = 0;
  localparam int unsigned IRQ_BIT_ERR    = 1;
  localparam int unsigned IRQ_BIT_IN_WM  = 2;
  localparam int unsigned IRQ_BIT_OUT_WM = 3;

  // Chen 1D DCT Fixed-Point Constants: round(cos(k * pi / 16) * 8192), Q13
  localparam logic signed [15:0] DCT_C1 = 16'sd8035;
  localparam logic signed [15:0] DCT_C2 = 16'sd7568;
  localparam logic signed [15:0] DCT_C3 = 16'sd6811;
  localparam logic signed [15:0] DCT_C4 = 16'sd5793;
  localparam logic signed [15:0] DCT_C5 = 16'sd4551;
  localparam logic signed [15:0] DCT_C6 = 16'sd3135;
  localparam logic signed [15:0] DCT_C7 = 16'sd1598;

  // 4-byte Knuth Multiplicative Hash (returns 10-bit default index)
  function automatic logic [9:0] lz4_hash4(
    input logic [7:0] b0,
    input logic [7:0] b1,
    input logic [7:0] b2,
    input logic [7:0] b3
  );
    logic [31:0] v;
    logic [31:0] prod;
    v    = {b3, b2, b1, b0};
    prod = v * 32'h9E37_79B1;
    return prod[31:22];
  endfunction

  // JPEG ITU-T T.81 Zig-Zag Scan Order: zig-zag index [0..63] -> raster index (y*8 + x)
  function automatic logic [5:0] jpeg_zigzag_addr(input logic [5:0] zz_idx);
    unique case (zz_idx)
      6'd0:  return 6'd0;
      6'd1:  return 6'd1;
      6'd2:  return 6'd8;
      6'd3:  return 6'd16;
      6'd4:  return 6'd9;
      6'd5:  return 6'd2;
      6'd6:  return 6'd3;
      6'd7:  return 6'd10;
      6'd8:  return 6'd17;
      6'd9:  return 6'd24;
      6'd10: return 6'd32;
      6'd11: return 6'd25;
      6'd12: return 6'd18;
      6'd13: return 6'd11;
      6'd14: return 6'd4;
      6'd15: return 6'd5;
      6'd16: return 6'd12;
      6'd17: return 6'd19;
      6'd18: return 6'd26;
      6'd19: return 6'd33;
      6'd20: return 6'd40;
      6'd21: return 6'd48;
      6'd22: return 6'd41;
      6'd23: return 6'd34;
      6'd24: return 6'd27;
      6'd25: return 6'd20;
      6'd26: return 6'd13;
      6'd27: return 6'd6;
      6'd28: return 6'd7;
      6'd29: return 6'd14;
      6'd30: return 6'd21;
      6'd31: return 6'd28;
      6'd32: return 6'd35;
      6'd33: return 6'd42;
      6'd34: return 6'd49;
      6'd35: return 6'd56;
      6'd36: return 6'd57;
      6'd37: return 6'd50;
      6'd38: return 6'd43;
      6'd39: return 6'd36;
      6'd40: return 6'd29;
      6'd41: return 6'd22;
      6'd42: return 6'd15;
      6'd43: return 6'd23;
      6'd44: return 6'd30;
      6'd45: return 6'd37;
      6'd46: return 6'd44;
      6'd47: return 6'd51;
      6'd48: return 6'd58;
      6'd49: return 6'd59;
      6'd50: return 6'd52;
      6'd51: return 6'd45;
      6'd52: return 6'd38;
      6'd53: return 6'd31;
      6'd54: return 6'd39;
      6'd55: return 6'd46;
      6'd56: return 6'd53;
      6'd57: return 6'd60;
      6'd58: return 6'd61;
      6'd59: return 6'd54;
      6'd60: return 6'd47;
      6'd61: return 6'd55;
      6'd62: return 6'd62;
      6'd63: return 6'd63;
      default: return 6'd0;
    endcase
  endfunction

  // ITU-T T.81 Annex K Table K.1 Luminance Q50 Reciprocal Table: {shift[4:0], recip[15:0]}
  function automatic logic [20:0] jpeg_luma_q50_sr(input logic [5:0] idx);
    unique case (idx)
      6'd0:  return {5'd19, 16'd32768}; // Q=16
      6'd1:  return {5'd19, 16'd47663}; // Q=11
      6'd2:  return {5'd19, 16'd52429}; // Q=10
      6'd3:  return {5'd19, 16'd32768}; // Q=16
      6'd4:  return {5'd20, 16'd43691}; // Q=24
      6'd5:  return {5'd21, 16'd52429}; // Q=40
      6'd6:  return {5'd21, 16'd41121}; // Q=51
      6'd7:  return {5'd21, 16'd34380}; // Q=61
      6'd8:  return {5'd19, 16'd43691}; // Q=12
      6'd9:  return {5'd19, 16'd43691}; // Q=12
      6'd10: return {5'd19, 16'd37450}; // Q=14
      6'd11: return {5'd20, 16'd55188}; // Q=19
      6'd12: return {5'd20, 16'd40330}; // Q=26
      6'd13: return {5'd21, 16'd36158}; // Q=58
      6'd14: return {5'd21, 16'd34953}; // Q=60
      6'd15: return {5'd21, 16'd38130}; // Q=55
      6'd16: return {5'd19, 16'd37450}; // Q=14
      6'd17: return {5'd19, 16'd40330}; // Q=13
      6'd18: return {5'd19, 16'd32768}; // Q=16
      6'd19: return {5'd20, 16'd43691}; // Q=24
      6'd20: return {5'd21, 16'd52429}; // Q=40
      6'd21: return {5'd21, 16'd36792}; // Q=57
      6'd22: return {5'd22, 16'd60787}; // Q=69
      6'd23: return {5'd21, 16'd37450}; // Q=56
      6'd24: return {5'd19, 16'd37450}; // Q=14
      6'd25: return {5'd20, 16'd61681}; // Q=17
      6'd26: return {5'd20, 16'd47663}; // Q=22
      6'd27: return {5'd20, 16'd36158}; // Q=29
      6'd28: return {5'd21, 16'd41121}; // Q=51
      6'd29: return {5'd22, 16'd48210}; // Q=87
      6'd30: return {5'd22, 16'd52429}; // Q=80
      6'd31: return {5'd21, 16'd33826}; // Q=62
      6'd32: return {5'd20, 16'd58255}; // Q=18
      6'd33: return {5'd20, 16'd47663}; // Q=22
      6'd34: return {5'd21, 16'd56680}; // Q=37
      6'd35: return {5'd21, 16'd37450}; // Q=56
      6'd36: return {5'd22, 16'd61681}; // Q=68
      6'd37: return {5'd22, 16'd38480}; // Q=109
      6'd38: return {5'd22, 16'd40721}; // Q=103
      6'd39: return {5'd22, 16'd54471}; // Q=77
      6'd40: return {5'd20, 16'd43691}; // Q=24
      6'd41: return {5'd21, 16'd59919}; // Q=35
      6'd42: return {5'd21, 16'd38130}; // Q=55
      6'd43: return {5'd21, 16'd32768}; // Q=64
      6'd44: return {5'd22, 16'd51782}; // Q=81
      6'd45: return {5'd22, 16'd40330}; // Q=104
      6'd46: return {5'd22, 16'd37118}; // Q=113
      6'd47: return {5'd22, 16'd45591}; // Q=92
      6'd48: return {5'd21, 16'd42799}; // Q=49
      6'd49: return {5'd21, 16'd32768}; // Q=64
      6'd50: return {5'd22, 16'd53774}; // Q=78
      6'd51: return {5'd22, 16'd48210}; // Q=87
      6'd52: return {5'd22, 16'd40721}; // Q=103
      6'd53: return {5'd22, 16'd34664}; // Q=121
      6'd54: return {5'd22, 16'd34953}; // Q=120
      6'd55: return {5'd22, 16'd41528}; // Q=101
      6'd56: return {5'd22, 16'd58255}; // Q=72
      6'd57: return {5'd22, 16'd45591}; // Q=92
      6'd58: return {5'd22, 16'd44151}; // Q=95
      6'd59: return {5'd22, 16'd42800}; // Q=98
      6'd60: return {5'd22, 16'd37450}; // Q=112
      6'd61: return {5'd22, 16'd41944}; // Q=100
      6'd62: return {5'd22, 16'd40721}; // Q=103
      6'd63: return {5'd22, 16'd42367}; // Q=99
      default: return {5'd19, 16'd32768};
    endcase
  endfunction

  // ITU-T T.81 Annex K Table K.2 Chrominance Q50 Reciprocal Table: {shift[4:0], recip[15:0]}
  function automatic logic [20:0] jpeg_chroma_q50_sr(input logic [5:0] idx);
    unique case (idx)
      6'd0:  return {5'd20, 16'd61681}; // Q=17
      6'd1:  return {5'd20, 16'd58255}; // Q=18
      6'd2:  return {5'd20, 16'd43691}; // Q=24
      6'd3:  return {5'd21, 16'd44620}; // Q=47
      6'd8:  return {5'd20, 16'd58255}; // Q=18
      6'd9:  return {5'd20, 16'd49932}; // Q=21
      6'd10: return {5'd20, 16'd40330}; // Q=26
      6'd11: return {5'd22, 16'd63551}; // Q=66
      6'd16: return {5'd20, 16'd43691}; // Q=24
      6'd17: return {5'd20, 16'd40330}; // Q=26
      6'd18: return {5'd21, 16'd37450}; // Q=56
      6'd24: return {5'd21, 16'd44620}; // Q=47
      6'd25: return {5'd22, 16'd63551}; // Q=66
      default: return {5'd22, 16'd42367}; // Q=99
    endcase
  endfunction

endpackage : codec_pkg
