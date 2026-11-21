// SPDX-License-Identifier: Apache-2.0
// hub75_pkg.sv - Types, register offsets and constants for the HUB75 LED-matrix controller

package hub75_pkg;

  // ---------------------------------------------------------------------------
  // Hard limits (see doc/README.md "Geometry limits")
  // ---------------------------------------------------------------------------
  localparam int unsigned MAX_LANES_C  = 4;
  localparam int unsigned MAX_PANELS_C = 32;  // panel table depth (chain panels per lane)
  localparam int unsigned INIT_ENTRIES = 16;  // chip-init program depth

  // ---------------------------------------------------------------------------
  // CSR map (byte offsets)
  // ---------------------------------------------------------------------------
  localparam logic [15:0] R_CTRL         = 16'h0000;
  localparam logic [15:0] R_STATUS       = 16'h0004;
  localparam logic [15:0] R_INT_EN       = 16'h0008;
  localparam logic [15:0] R_INT_STAT     = 16'h000C;
  localparam logic [15:0] R_FB_ADDR0     = 16'h0010;
  localparam logic [15:0] R_FB_ADDR1     = 16'h0014;
  localparam logic [15:0] R_FB_STRIDE    = 16'h0018;
  localparam logic [15:0] R_FB_CTRL      = 16'h001C;
  localparam logic [15:0] R_GEOM0        = 16'h0020;
  localparam logic [15:0] R_GEOM1        = 16'h0024;
  localparam logic [15:0] R_BCM          = 16'h0028;
  localparam logic [15:0] R_BCM_BASE     = 16'h002C;
  localparam logic [15:0] R_TIM0         = 16'h0030;
  localparam logic [15:0] R_TIM1         = 16'h0034;
  localparam logic [15:0] R_BRIGHT       = 16'h0038;
  localparam logic [15:0] R_FADE         = 16'h003C;
  localparam logic [15:0] R_TPG_CTRL     = 16'h0040;
  localparam logic [15:0] R_TPG_COLOR    = 16'h0044;
  localparam logic [15:0] R_GAMMA_CTRL   = 16'h0048;
  localparam logic [15:0] R_INIT_CTRL    = 16'h004C;
  localparam logic [15:0] R_BRIGHT_CUR   = 16'h0058;
  localparam logic [15:0] R_FRAME_CNT    = 16'h0060;
  localparam logic [15:0] R_FRAME_CYCLES = 16'h0064;
  localparam logic [15:0] R_FPS          = 16'h0068;
  localparam logic [15:0] R_FPS_WINDOW   = 16'h006C;
  localparam logic [15:0] R_UNDERRUN_CNT = 16'h0070;
  localparam logic [15:0] R_STALL_CYCLES = 16'h0074;
  localparam logic [15:0] R_DMA_ERR_CNT  = 16'h0078;
  localparam logic [15:0] R_ROW_FETCH_MAX= 16'h007C;
  localparam logic [15:0] R_ID           = 16'h0080;
  localparam logic [15:0] R_CAPS         = 16'h0084;
  localparam logic [15:0] PANEL_TBL_BASE = 16'h0100; // 32 x 32-bit
  localparam logic [15:0] INIT_PROG_BASE = 16'h0200; // 16 x 32-bit
  localparam logic [15:0] LUT_BASE       = 16'h1000; // 3 x 0x400 (R,G,B)

  localparam logic [31:0] HUB75_ID_VALUE = 32'h4855_4201; // "HUB" v1

  // CTRL bits
  localparam int CTRL_EN         = 0;
  localparam int CTRL_BLANK      = 1;
  localparam int CTRL_OE_INV     = 2;
  localparam int CTRL_LAT_INV    = 3;
  localparam int CTRL_MSB_FIRST  = 4;
  localparam int CTRL_DITHER     = 5;
  localparam int CTRL_ADDR_SER   = 6;
  localparam int CTRL_MIRROR_X   = 7;
  localparam int CTRL_FLIP_Y     = 8;
  localparam int CTRL_ROT180     = 9;
  localparam int CTRL_AUTO_INIT  = 10;
  localparam int CTRL_CFG_HOLD   = 11;
  localparam int CTRL_CNT_CLR    = 12; // write-1 self-clearing

  // INT bits
  localparam int INT_FRAME    = 0;
  localparam int INT_UNDERRUN = 1;
  localparam int INT_DMA_ERR  = 2;
  localparam int INT_SWAP     = 3;
  localparam int INT_INIT     = 4;
  localparam int INT_FADE     = 5;
  localparam int INT_CFG_ERR  = 6;
  localparam int INT_W        = 7;

  // Pixel formats (FB_CTRL.FMT)
  localparam logic [2:0] FMT_RGB565   = 3'd0;
  localparam logic [2:0] FMT_RGB888P  = 3'd1; // packed 24 bpp, memory byte order B,G,R
  localparam logic [2:0] FMT_ARGB8888 = 3'd2; // 32 bpp, alpha ignored
  localparam logic [2:0] FMT_RGB332   = 3'd3;
  localparam logic [2:0] FMT_RGB444   = 3'd4; // 16 bpp, XRGB4444

  // Multiplexed-panel pixel mapping modes (GEOM1.MUX_MODE)
  localparam logic [1:0] MUX_LINEAR   = 2'd0;
  localparam logic [1:0] MUX_BLOCK    = 2'd1;
  localparam logic [1:0] MUX_STRIPE8  = 2'd2;
  localparam logic [1:0] MUX_SNAKE    = 2'd3;

  // Test patterns (TPG_CTRL.MODE)
  localparam logic [2:0] TPG_OFF      = 3'd0;
  localparam logic [2:0] TPG_BARS     = 3'd1;
  localparam logic [2:0] TPG_GRADIENT = 3'd2;
  localparam logic [2:0] TPG_CHECKER  = 3'd3;
  localparam logic [2:0] TPG_SOLID    = 3'd4;
  localparam logic [2:0] TPG_BORDER   = 3'd5;
  localparam logic [2:0] TPG_PLANE    = 3'd6;

  // Chip-init program entry opcodes (INIT_PROG[n][24:22])
  localparam logic [2:0] IOP_END    = 3'd0;
  localparam logic [2:0] IOP_CONFIG = 3'd1;
  localparam logic [2:0] IOP_WAIT   = 3'd2;
  localparam logic [2:0] IOP_LATP   = 3'd3;

  // ---------------------------------------------------------------------------
  // Frame-latched configuration (shadow set).  Latched by the fetch engine at
  // the start of every frame (unless CTRL.CFG_HOLD).
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic        valid;      // geometry legal
    logic [31:0] fb_base;
    logic [15:0] stride;
    logic [2:0]  fmt;
    logic [7:0]  burst;      // max beats - 1
    logic [9:0]  pw;
    logic [5:0]  gx;
    logic [5:0]  gy;
    logic [2:0]  scan_log2;
    logic [1:0]  k_log2;
    logic [1:0]  mux_mode;
    logic        colmajor;
    logic        snake;
    logic        alt_rot;
    logic        mirror;     // effective (after ROT180 xor)
    logic        flip;       // effective
    logic [3:0]  depth;
    logic        msb_first;
    logic        serial_addr;
    logic [2:0]  tpg_mode;
    logic [3:0]  tpg_plane;
    logic [23:0] tpg_color;
    logic [5:0]  np;         // gx*gy
    logic [11:0] lp;         // chain positions per half per panel
    logic [11:0] lc;         // chain positions per half per lane
    logic [6:0]  ph;         // panel height (rows)
    logic [15:0] wimg;       // gx*pw
    logic [13:0] himg;       // lanes*gy*ph
    logic [11:0] hlane;      // gy*ph
  } cfg_t;

  function automatic logic [2:0] bytes_pp(input logic [2:0] fmt);
    unique case (fmt)
      FMT_RGB565:   return 3'd2;
      FMT_RGB888P:  return 3'd3;
      FMT_ARGB8888: return 3'd4;
      FMT_RGB332:   return 3'd1;
      FMT_RGB444:   return 3'd2;
      default:      return 3'd2;
    endcase
  endfunction

  // Chip-init program templates.  Entry layout:
  //   [15:0] DATA   [21:16] LATCH_N   [24:22] OP   [31:25] LEN8 (shift length / 8, 0 = full chain)
  function automatic logic [31:0] init_entry(
    input logic [15:0] data, input logic [5:0] latch_n, input logic [2:0] op, input logic [6:0] len8);
    return {len8, op, latch_n, data};
  endfunction

  // chip: 0 none, 1 FM6126-class, 2 FM6127-class, 3 ICN2038-class, 4 MBI5124-class, 5 DP3246-class
  function automatic logic [31:0] init_preset(input logic [3:0] chip, input logic [3:0] idx);
    logic [31:0] e;
    e = init_entry(16'h0, 6'd0, IOP_END, 7'd0);
    unique case (chip)
      4'd1: begin // two config registers, LAT high for the last 12 / 13 clocks
        if (idx == 4'd0) e = init_entry(16'h7FFF, 6'd12, IOP_CONFIG, 7'd0);
        if (idx == 4'd1) e = init_entry(16'h0040, 6'd13, IOP_CONFIG, 7'd0);
      end
      4'd2: begin // three-register class: 11 / 12 / 13 latch clocks
        if (idx == 4'd0) e = init_entry(16'h1FFF, 6'd11, IOP_CONFIG, 7'd0);
        if (idx == 4'd1) e = init_entry(16'h0040, 6'd12, IOP_CONFIG, 7'd0);
        if (idx == 4'd2) e = init_entry(16'h0100, 6'd13, IOP_CONFIG, 7'd0);
      end
      4'd3: begin // pre-active LAT pulses, then one config word (14 latch clocks)
        if (idx == 4'd0) e = init_entry(16'h0000, 6'd14, IOP_LATP,   7'd0);
        if (idx == 4'd1) e = init_entry(16'h8008, 6'd14, IOP_CONFIG, 7'd0);
        if (idx == 4'd2) e = init_entry(16'h0010, 6'd1,  IOP_WAIT,   7'd0);
      end
      4'd4: begin // single 16-bit config register, LAT for the last 4 clocks
        if (idx == 4'd0) e = init_entry(16'h2C80, 6'd4, IOP_CONFIG, 7'd0);
      end
      4'd5: begin // two registers, 10/11 latch clocks, short wait in between
        if (idx == 4'd0) e = init_entry(16'h00F0, 6'd10, IOP_CONFIG, 7'd0);
        if (idx == 4'd1) e = init_entry(16'h0004, 6'd1,  IOP_WAIT,   7'd0);
        if (idx == 4'd2) e = init_entry(16'h0A55, 6'd11, IOP_CONFIG, 7'd0);
      end
      default: ;
    endcase
    return e;
  endfunction

endpackage : hub75_pkg
