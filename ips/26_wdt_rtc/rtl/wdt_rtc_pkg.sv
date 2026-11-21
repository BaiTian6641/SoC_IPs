// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_pkg.sv - Constants shared by the windowed watchdog + backup-domain RTC IP

package wdt_rtc_pkg;

  // ---------------------------------------------------------------------------
  // Watchdog key words (all written to WDT_KEY with all four byte strobes set)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] WDT_KEY_UNLOCK1 = 32'h5AFE_0001;  // unlock step 1
  localparam logic [31:0] WDT_KEY_UNLOCK2 = 32'hC0DE_0002;  // unlock step 2 (config writes enabled)
  localparam logic [31:0] WDT_KEY_LOCK    = 32'h4C4F_434B;  // "LOCK": config writes disabled
  localparam logic [31:0] WDT_KEY_REFRESH1 = 32'hA5A5_1111; // refresh step 1
  localparam logic [31:0] WDT_KEY_REFRESH2 = 32'h5A5A_2222; // refresh step 2 (refresh event)
  localparam logic [31:0] WDT_SWRST_MAGIC = 32'hDEAD_0B07;  // WDT_SWRST value

  // Reset-cause bit positions (WDT_CAUSE)
  localparam int unsigned CAUSE_POR     = 0;
  localparam int unsigned CAUSE_TIMEOUT = 1;
  localparam int unsigned CAUSE_WINDOW  = 2;
  localparam int unsigned CAUSE_BADKEY  = 3;
  localparam int unsigned CAUSE_SW      = 4;

  // ---------------------------------------------------------------------------
  // RTC write-protect keys (RTC_KEY): KEY1 then KEY2 unlocks; anything else locks
  // ---------------------------------------------------------------------------
  localparam logic [31:0] RTC_KEY1 = 32'h52C1_0001;
  localparam logic [31:0] RTC_KEY2 = 32'h52C2_0002;

  // ---------------------------------------------------------------------------
  // RTC interrupt/wake/status flag indices (RTC_FLAGS / RTC_IEN / RTC_WEN)
  // ---------------------------------------------------------------------------
  localparam int unsigned FLG_ALRA  = 0;
  localparam int unsigned FLG_ALRB  = 1;
  localparam int unsigned FLG_WUT   = 2;
  localparam int unsigned FLG_TS    = 3;
  localparam int unsigned FLG_TSOVF = 4;
  localparam int unsigned FLG_SEC   = 5;
  localparam int unsigned FLG_TAMP  = 6;
  localparam int unsigned FLG_WRERR = 7;

  // ---------------------------------------------------------------------------
  // RTC register offsets (relative to the RTC window base = 0x100), byte addresses
  // ---------------------------------------------------------------------------
  localparam logic [7:0] RTC_O_STAT    = 8'h00;
  localparam logic [7:0] RTC_O_SYNC    = 8'h04;
  localparam logic [7:0] RTC_O_TIME    = 8'h08;
  localparam logic [7:0] RTC_O_DATE    = 8'h0C;
  localparam logic [7:0] RTC_O_SUBSEC  = 8'h10;
  localparam logic [7:0] RTC_O_BIN_LO  = 8'h14;
  localparam logic [7:0] RTC_O_BIN_HI  = 8'h18;
  localparam logic [7:0] RTC_O_FLAGS   = 8'h1C;
  localparam logic [7:0] RTC_O_KEY     = 8'h20;
  localparam logic [7:0] RTC_O_CTRL    = 8'h24;
  localparam logic [7:0] RTC_O_IEN     = 8'h28;
  localparam logic [7:0] RTC_O_WEN     = 8'h2C;
  localparam logic [7:0] RTC_O_PRE     = 8'h30;
  localparam logic [7:0] RTC_O_CAL     = 8'h34;
  localparam logic [7:0] RTC_O_WUT     = 8'h38;
  localparam logic [7:0] RTC_O_WUT_CNT = 8'h3C;
  localparam logic [7:0] RTC_O_ALRA_TIME = 8'h40;
  localparam logic [7:0] RTC_O_ALRA_DATE = 8'h44;
  localparam logic [7:0] RTC_O_ALRA_BIN  = 8'h48;
  localparam logic [7:0] RTC_O_ALRA_BMSK = 8'h4C;
  localparam logic [7:0] RTC_O_ALRB_TIME = 8'h50;
  localparam logic [7:0] RTC_O_ALRB_DATE = 8'h54;
  localparam logic [7:0] RTC_O_ALRB_BIN  = 8'h58;
  localparam logic [7:0] RTC_O_ALRB_BMSK = 8'h5C;
  localparam logic [7:0] RTC_O_TAMP    = 8'h60;
  localparam logic [7:0] RTC_O_TS_TIME = 8'h64;
  localparam logic [7:0] RTC_O_TS_DATE = 8'h68;
  localparam logic [7:0] RTC_O_TS_SUB  = 8'h6C;
  localparam logic [7:0] RTC_O_TS_BLO  = 8'h70;
  localparam logic [7:0] RTC_O_TS_BHI  = 8'h74;
  localparam logic [7:0] RTC_O_TS_SRC  = 8'h78;

  // Width of the coherent time snapshot carried across the clock-domain crossing:
  //   {time[22:0], date[31:0], subsec[21:0], bin[63:0]}
  localparam int unsigned SNAP_W = 23 + 32 + 22 + 64;

  // Offsets that are executed inside the RTC (backup) clock domain through the
  // request/acknowledge channel instead of the bus-domain shadow registers.
  function automatic logic is_chan_rd(input logic [7:0] off);
    return (off == RTC_O_KEY) || (off == RTC_O_CTRL) || (off == RTC_O_IEN) ||
           (off == RTC_O_WEN) || (off == RTC_O_PRE)  || (off == RTC_O_CAL) ||
           (off == RTC_O_WUT) || (off == RTC_O_WUT_CNT) ||
           (off >= RTC_O_ALRA_TIME && off <= RTC_O_ALRB_BMSK && off[1:0] == 2'b00) ||
           (off == RTC_O_TAMP) ||
           (off >= RTC_O_TS_TIME && off <= RTC_O_TS_SRC && off[1:0] == 2'b00);
  endfunction

  function automatic logic is_chan_wr(input logic [7:0] off);
    return (off == RTC_O_TIME) || (off == RTC_O_DATE) || (off == RTC_O_BIN_LO) ||
           (off == RTC_O_BIN_HI) || (off == RTC_O_FLAGS) || (off == RTC_O_KEY) ||
           (off == RTC_O_CTRL) || (off == RTC_O_IEN) || (off == RTC_O_WEN) ||
           (off == RTC_O_PRE)  || (off == RTC_O_CAL) || (off == RTC_O_WUT) ||
           (off >= RTC_O_ALRA_TIME && off <= RTC_O_ALRB_BMSK && off[1:0] == 2'b00) ||
           (off == RTC_O_TAMP);
  endfunction

endpackage : wdt_rtc_pkg
