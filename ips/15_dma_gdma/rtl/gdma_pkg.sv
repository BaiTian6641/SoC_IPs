// SPDX-License-Identifier: Apache-2.0
// gdma_pkg.sv - Register offsets, field positions and helper functions for the GDMA2 controller.
//
// Everything here is documented in ips/15_dma_gdma/doc/README.md (the datasheet is the contract).

package gdma_pkg;

  // ---------------------------------------------------------------------------
  // Global block
  // ---------------------------------------------------------------------------
  localparam logic [11:0] G_CTRL      = 12'h000;
  localparam logic [11:0] G_INT_EN    = 12'h004;
  localparam logic [11:0] G_INT_STAT  = 12'h008;
  localparam logic [11:0] G_STATUS    = 12'h00C;
  localparam logic [11:0] G_ID        = 12'h010;
  localparam logic [11:0] G_CFG       = 12'h014;
  localparam logic [11:0] G_INT_EN_HI = 12'h018;
  localparam logic [11:0] G_INT_ST_HI = 12'h01C;
  localparam logic [11:0] G_PERF_CTRL = 12'h300;
  localparam logic [11:0] G_PERF_CYC  = 12'h304;
  localparam logic [11:0] G_PERF_STL  = 12'h308;
  localparam logic [11:0] G_PERF_RDB  = 12'h30C;
  localparam logic [11:0] G_PERF_WRB  = 12'h310;
  localparam logic [11:0] G_PERF_CHK  = 12'h314;
  localparam logic [11:0] G_PERF_BYT  = 12'h318;
  localparam logic [11:0] G_ERR_ADDR  = 12'h320;
  localparam logic [11:0] G_ERR_INFO  = 12'h324;
  localparam logic [11:0] G_IRQ_PEND  = 12'h328;

  localparam logic [31:0] GDMA_ID     = 32'h4744_4D32;

  // Legacy channel block: 0x020 + 0x20*c
  localparam logic [11:0] LEG_BASE    = 12'h020;
  localparam logic [4:0]  L_CTRL      = 5'h00;
  localparam logic [4:0]  L_SRC       = 5'h04;
  localparam logic [4:0]  L_DST       = 5'h08;
  localparam logic [4:0]  L_LEN       = 5'h0C;
  localparam logic [4:0]  L_DESC      = 5'h10;

  // Extended channel block: 0x400 + 0x80*c
  localparam logic [11:0] EXT_BASE    = 12'h400;
  localparam logic [6:0]  X_CTRL2     = 7'h00;
  localparam logic [6:0]  X_SHAPE     = 7'h04;
  localparam logic [6:0]  X_SPITCH    = 7'h08;
  localparam logic [6:0]  X_DPITCH    = 7'h0C;
  localparam logic [6:0]  X_SPSTR     = 7'h10;
  localparam logic [6:0]  X_DPSTR     = 7'h14;
  localparam logic [6:0]  X_FILL      = 7'h18;
  localparam logic [6:0]  X_CRC       = 7'h1C;
  localparam logic [6:0]  X_STATUS    = 7'h20;
  localparam logic [6:0]  X_IE        = 7'h24;
  localparam logic [6:0]  X_TIMEOUT   = 7'h28;
  localparam logic [6:0]  X_HTTH      = 7'h2C;
  localparam logic [6:0]  X_FLOW      = 7'h30;
  localparam logic [6:0]  X_BYTES     = 7'h34;
  localparam logic [6:0]  X_ERR_ADDR  = 7'h38;
  localparam logic [6:0]  X_ERR_INFO  = 7'h3C;
  localparam logic [6:0]  X_CMD       = 7'h40;
  localparam logic [6:0]  X_PROG      = 7'h44;
  localparam logic [6:0]  X_DESCHEAD  = 7'h48;

  // STATUS flag bit positions
  localparam int F_TC   = 0;
  localparam int F_HT   = 1;
  localparam int F_ERR  = 2;
  localparam int F_TO   = 3;
  localparam int F_NODE = 4;
  localparam int F_CFG  = 5;
  localparam int F_ABRT = 6;

  // Error / stop codes (ERR_INFO[7:4])
  localparam logic [3:0] EC_NONE = 4'd0;
  localparam logic [3:0] EC_RD   = 4'd1;
  localparam logic [3:0] EC_WR   = 4'd2;
  localparam logic [3:0] EC_DESC = 4'd3;
  localparam logic [3:0] EC_CFG  = 4'd4;
  localparam logic [3:0] EC_TO   = 4'd5;

  // ---------------------------------------------------------------------------
  // Checksum helpers. mode: 1 = CRC-32 (IEEE, reflected), 2 = CRC-32C, 3 = byte sum.
  // The CRC register holds the "final" value (state xor xo), see crc_xo().
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] crc_xo(input logic [1:0] mode);
    return (mode == 2'd1 || mode == 2'd2) ? 32'hFFFF_FFFF : 32'h0000_0000;
  endfunction

  function automatic logic [31:0] crc_byte_upd(input logic [1:0] mode, input logic [31:0] st,
                                               input logic [7:0] b);
    logic [31:0] s;
    logic [31:0] poly;
    poly = (mode == 2'd1) ? 32'hEDB8_8320 : 32'h82F6_3B78;
    s    = st ^ {24'd0, b};
    for (int i = 0; i < 8; i++) begin
      s = s[0] ? ((s >> 1) ^ poly) : (s >> 1);
    end
    return (mode == 2'd3) ? (st + {24'd0, b}) : ((mode == 2'd0) ? st : s);
  endfunction

  function automatic logic [7:0] bit_reverse8(input logic [7:0] b);
    logic [7:0] r;
    for (int i = 0; i < 8; i++) r[i] = b[7-i];
    return r;
  endfunction

endpackage : gdma_pkg
