# hub75 behavioural spec (working draft; merged into README.md at the end)

Core: `hub75_core` (CSR `csr_req_t/csr_rsp_t`, DMA master `mem_req_t/mem_rsp_t`, pads, `irq_o`, `frame_o`).
Parameters: `LANES` (1..4), `MAX_CHAIN` (default 512, max shift positions per half per lane), `MAX_PANELS` (32).
Register offsets/fields: see `rtl/hub75_pkg.sv` (single source of truth) and the register table in README.

## 1. Pads (per lane l = 0..LANES-1)
`hub75_r1_o[l] g1 b1 r2 g2 b2`, shared `hub75_clk_o`, `hub75_lat_o`, `hub75_oe_n_o`, `hub75_addr_o[4:0]`, `hub75_addr_en_o`.
Panel model semantics: a chain is a shift register of **N = LC** stages per colour line (R1,G1,B1,R2,G2,B2 are 6 independent
registers). On every **rising edge of hub75_clk** the bit on the data pin is shifted in at stage 0; after N edges the
first-shifted bit is at stage N-1. `LAT` (after polarity) high at the time of a clk rising edge is a *config-framing* edge (see §7);
LAT **falling** transfers the shift registers into the output latch (stage j -> output j), *unless* LAT was high during
>=1 clk rising edge since it rose (then it is a config write, not a pixel latch). Pixels are displayed while OE is active
(oe_n low by default, polarity CTRL.OE_INV): the panel lights output (half h, stage j) of every colour line whose latched bit is 1, for the
sys-clock cycles OE is active, for the row selected by `hub75_addr_o` (binary, `CTRL.ADDR_SER=0`).
Half 0 = R1/G1/B1, half 1 = R2/G2/B2. Idle (disabled): clk=0, lat=inactive, data=0, addr=0, OE inactive.

## 2. Geometry (all values per lane; every lane has the identical geometry)
* `PW` panel width in pixels (multiple of 4, 4..512), `PH = 2^SCAN_LOG2 * 2^K_LOG2` panel rows (<=64),
  `S = 2^SCAN_LOG2` scan rows (address values, 4/8/16/32), `K = 2^K_LOG2` (2/4/8) = panel rows lit per address.
  Standard 1/16-scan 64x32: S=16,K=2. 1/32-scan 64x64: S=32,K=2. 1/4-scan 32x16: S=4,K=4. 1/8-scan 32x16: S=8,K=2; 1/8-scan 64x32: S=8,K=4.
* Chain: `GX` x `GY` panels, `NP=GX*GY` (<=32). Image width `W=GX*PW`; lane height `HL=GY*PH`; image height `H=LANES*HL`.
  Logical image lives in memory row-major, `W` pixels per row, lane 0 on top, row pitch = `FB_STRIDE` bytes.
* `Lp = PW*K/2` shift positions per half per panel; `LC = NP*Lp` per half per lane (must be <= MAX_CHAIN, else CFG_ERR).
* Chain order: panel index `pi`=0..NP-1, `pi`=0 is *nearest the controller* (its pixels occupy chain stages 0..Lp-1; stage 0 is the one
  nearest the input, shifted LAST). Grid walk (fast index runs fastest): `COLMAJOR=0`: fast=gx, slow=gy; `COLMAJOR=1`: fast=gy, slow=gx.
  `SNAKE=1`: when slow index is odd the fast index runs backwards (fast_eff = lim-1-fast).
* Per-panel table `PANEL_TBL[pi]`: `[2:0] ORDER`, `[3] FLIP_X`, `[4] FLIP_Y`. `ALT_ROT=1` additionally XORs FLIP_X and FLIP_Y for panels with odd slow index.
* Global: `eff_mirror = MIRROR_X ^ ROT180`, `eff_flip = FLIP_Y ^ ROT180`.
* ORDER: logical channel carried on (R-line,G-line,B-line): 0 RGB, 1 RBG, 2 GRB, 3 GBR, 4 BRG, 5 BGR, 6/7 = RGB. Gamma LUT `c` is applied to logical channel `c`
  *before* the reorder.

### Pixel mapping, scan address `a` (0..S-1), lane `L`, chain panel `pi` with grid (gx,gy), panel row group `kp` (0..K-1), physical panel column `xp` (0..PW-1)
```
rev_x = FLIP_X[pi] ^ eff_mirror ^ (ALT_ROT & slow_odd)       rev_y = FLIP_Y[pi] ^ eff_flip ^ (ALT_ROT & slow_odd)
gx_m  = eff_mirror ? GX-1-gx : gx      gy_m = eff_flip ? GY-1-gy : gy      L_m = eff_flip ? LANES-1-L : L
yp    = a + kp*S                       (physical row inside the panel)
ym_in = rev_y ? PH-1-yp : yp
Ymem  = L_m*HL + gy_m*PH + ym_in       Xmem = gx_m*PW + (rev_x ? PW-1-xp : xp)
pixel (physical position) shows memory pixel (Xmem, Ymem)
mux (half h, position p inside the panel), blk:
  MUX_LINEAR : h = kp[0];  blk = kp>>1;                 p = blk*PW + xp
  MUX_BLOCK  : h = kp>=K/2; blk = kp mod (K/2);          p = blk*PW + xp
  MUX_STRIPE8: h = kp[0];  blk = kp>>1;                 p = (xp>>3)*(4*K) + blk*8 + (xp&7)
  MUX_SNAKE  : h = kp[0];  blk = kp>>1;                 p = blk*PW + (blk[0] ? PW-1-xp : xp)
chain stage j = pi*Lp + p ; shift time index t = LC-1-j (t=0 is shifted first)
```
For K=2 (standard panels) all four modes are identical: h=kp, p=xp.

## 3. Pixel formats (FB_CTRL.FMT), little-endian, rows word aligned (base, stride multiples of 4, PW multiple of 4)
0 RGB565 (pixel 0 = bits[15:0] of word); 1 packed RGB888 (3 bytes per pixel, byte order B,G,R); 2 ARGB8888/XRGB8888 (alpha ignored, {A,R,G,B});
3 RGB332 (byte, {R3,G3,B2}); 4 XRGB4444 (16 bit, {X,R4,G4,B4}).
Expansion to 8 bit by bit replication: 565: r={r5,r5[4:2]} g={g6,g6[5:4]} b={b5,b5[4:2]}; 332: r={r3,r3,r3[2:1]} g likewise b={b2,b2,b2,b2}; 444: {x4,x4}.

## 4. Gamma and BCM
Per logical channel LUT (256 x 12 bit, CSR window, reset-initialised from preset `GAMMA_CTRL.PRESET`: 0 gamma2.2, 1 linear, 2 gamma2.8, 3 gamma1.8;
`out = round(4095*(i/255)^g)`). Stored value `v = lut >> (12-DEPTH)` (DEPTH = 8..12, `BCM.DEPTH`).
Plane `p` (0..DEPTH-1) displays bit `p` of `v`. Plane weight `W_p = BCM_BASE << p` sys-clock cycles. `CTRL.MSB_FIRST` reverses the plane *order* (weights unchanged).
Brightness: `S = (b==255) ? 256 : b` where b = current brightness (BRIGHT or fade value); lit time of plane = `(W_p*S + acc) >> 8`;
`acc' = DITHER ? (W_p*S + acc) & 255 : 0` (acc carried across planes/rows/frames in display order, reset when the scan engine restarts).
A plane with lit time 0 produces no OE pulse. **Displayed intensity of a pixel colour = sum over planes of bit_p * lit_time_p** (in sys clocks), per row-refresh.
Each scan address is displayed once per frame (all planes), rows 0..S-1 in order.

## 5. Row/plane sequence (sys-clock cycle accurate; N_x = max(value,1) except where stated)
For every plane of a row (shift of plane data for lines happens while the previous plane is lit):
`WAIT_SHIFT` (OE off until the shift of this plane completed) -> `PREP` (3 cycles, OE off) -> `PRE` (BLANK_PRE cycles, OE off) -> [first plane of a row only: `ADDR`
(address changes; ADDR_SETUP cycles)] -> `LAT` (LAT_W cycles LAT active) -> LAT falls: shifter starts shifting the NEXT plane (or plane 0 of the next row if its line buffer is ready)
-> `POST` (BLANK_POST cycles, OE off) -> `ON` (OE active for lit_time cycles, skipped if 0).
Shift: tick every CLK_DIV+1 sys clocks, hub75_clk period = 2*(CLK_DIV+1) sys clocks (50% duty): per pixel tick A: clk=0 and data pins updated; tick B: clk=1. One extra tick at the end
drops clk to 0. Exactly LC rising edges per plane. Line buffer is double buffered (2 banks): fetch of row r+1 overlaps display of row r.
Underrun = shifter could not start the next row's plane 0 because the row had not been fetched when the OE window of the previous plane ended: display stays dark (OE off) and waits.
Binary address mode: `addr = row` driven in ADDR state. Serial mode (CTRL.ADDR_SER): addr[0]=DIN addr[1]=SCLK (rising edge shifts); at row 0 S+1 pulses (S with DIN=0 then one with DIN=1), rows>0 one pulse DIN=0; each clock phase lasts ADDR_SETUP cycles.
`hub75_addr_en_o` = 1 exactly while OE is active (positive polarity regardless of OE_INV).

## 6. Frame boundary / double buffering
Geometry, format, depth, FB pointer, TPG regs are latched ("shadow") when the fetch engine starts row 0 of a frame (not while CTRL.CFG_HOLD=1).
`FB_CTRL.FB_SEL` selects FB_ADDR0/1 at that moment; `STATUS.FB_ACTIVE` shows the pointer being scanned, `SWAP_PENDING` = FB_SEL != FB_ACTIVE; INT SWAP fires when a swap takes effect.
INT FRAME fires (and `frame_o` pulses one cycle) when the last plane of the last row finished its ON window.

## 7. Chip init (config-register write framing)
`INIT_PROG[n]`: `[15:0] DATA [21:16] LATCH_N [24:22] OP [31:25] LEN8`. OP: 0 END, 1 CONFIG, 2 WAIT, 3 LATP.
CONFIG: shift `LEN = LEN8*8` (LEN8=0: LC from live geometry) clocks on all 6 lines of all lanes (lines not in INIT_CTRL.CHAN_MASK driven 0), data bit at chain stage j is `DATA[j mod 16]` (so
the first shifted bit is DATA[(LEN-1) mod 16]; for LEN a multiple of 16 that is MSB first); LAT is active during the last LATCH_N clock rising edges (stages j < LATCH_N);
LAT falls after the last clock. OE inactive for the whole sequence. Same CLK_DIV timing as pixel shifting; LAT_W/OE irrelevant. A gap of 4 ticks follows every entry.
WAIT: `DATA * 256` sys clocks idle. LATP: LATCH_N LAT pulses (LAT_W cycles high, LAT_W low), no clock.
A panel model recognises a config write when LAT was high during n>=1 clock rising edges, word = stage bits 0..15 (`word[j]` = stage j), register index = n.
`CHIP_TYPE` write (INIT_CTRL[3:0]) loads templates into the program table (see hub75_pkg::init_preset); START (INIT_CTRL[8]) runs the program; `CTRL.AUTO_INIT` runs it on every EN rising edge before scanning.

## 8. Test pattern generator (TPG_CTRL.MODE), replaces DMA fetch, goes through gamma/mapping like memory pixels (coordinates = memory coordinates Xmem,Ymem)
1 bars: 8 bars across W, bar = floor(8*Xmem/W): white, yellow, cyan, green, magenta, red, blue, black. 2 gradient: R=Xmem[7:0], G=Ymem[7:0], B=(Xmem+Ymem)[7:0].
3 checker: 8x8 squares `((Xmem>>3)^(Ymem>>3))&1` -> TPG_COLOR else black. 4 solid TPG_COLOR ({R,G,B}). 5 border: Xmem==0|Xmem==W-1|Ymem==0|Ymem==H-1 -> TPG_COLOR else black.
6 bit-plane: stored value = (TPG_COLOR channel != 0) ? (1<<TPG_PLANE) : 0 on every pixel (bypasses LUT; plane >= DEPTH -> 0).
