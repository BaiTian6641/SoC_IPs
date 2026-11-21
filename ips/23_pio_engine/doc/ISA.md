# PIO Engine ("pioe") — Instruction Set & Execution Semantics (FROZEN v1)

This file is the normative reference for the RTL (`rtl/pio_sm.sv`), the SV reference interpreter used by the
fuzz testbench, and the assembler (`tools/pioasm.py`). All numbers are decimal unless prefixed `0x`/`0b`.

## 1. Instruction word

Instruction memory words are **24 bits** (stored in the low 24 bits of a 32-bit CPU word; bits 31:24 read 0).

```
 23    20 19    16 15    12 11                    0
+--------+--------+--------+-----------------------+
|  DLY   |   SS   |   OP   |          ARG          |
+--------+--------+--------+-----------------------+
```

* `DLY[3:0]` – delay: after the instruction **completes**, the SM idles for `DLY` extra ticks (instruction
  occupies `1 + DLY` ticks when it does not stall).
* `SS[3:0]` – side-set field (see §5). Ignored if the SM has `side_count == 0`.
* `OP[3:0]` – opcode.  `ARG[11:0]` – operands, per opcode below.

### 1.1 Opcode map

| OP  | Mnemonic | ARG layout (bit ranges inside the 24-bit word) |
|-----|----------|------------------------------------------------|
| 0x0 | JMP      | `[11:8]` COND, `[7:0]` ADDR |
| 0x1 | WAIT     | `[11]` POL, `[10:9]` SRC, `[8:7]` REL (IRQ only), `[4:0]` IDX |
| 0x2 | IN       | `[11:8]` SRC, `[4:0]` CNT (0 ⇒ 32) |
| 0x3 | OUT      | `[11:8]` DST, `[4:0]` CNT (0 ⇒ 32) |
| 0x4 | PUSH/PULL| `[11]` PULL (0=PUSH,1=PULL), `[10]` COND (PUSH: IFFULL, PULL: IFEMPTY), `[9]` BLOCK |
| 0x5 | MOV      | `[11:8]` DST, `[7:6]` MOP, `[3:0]` SRC |
| 0x6 | ALUR     | `[11:10]` DST, `[9:7]` ALUOP, `[6:4]` A, `[3:1]` B   (DST = A op B) |
| 0x7 | ALUI     | `[11:10]` DST, `[9:7]` ALUOP, `[6:0]` IMM            (DST = DST op IMM) |
| 0x8 | SET      | `[11:8]` DST, `[7:0]` IMM |
| 0x9 | LOOP     | `[11:8]` LEN (body = LEN+1 instructions), `[7:0]` COUNT (immediate) |
| 0xA | LOOPR    | `[11:8]` LEN, `[7:6]` REG (0=X,1=Y,2=Z) — count taken from `REG[15:0]` |
| 0xB | IRQ      | `[11:10]` MODE, `[9:8]` REL, `[2:0]` IDX |
| 0xC | NOP      | (no ARG) |
| 0xD–0xF | reserved | execute as NOP and set sticky `ILLEGAL` |

Unlisted ARG bits are *reserved, must be 0, ignored* by hardware. Any reserved **enumerated** value
(e.g. IN SRC = 9, SET DST = 7, ALUR DST = 3, LOOPR REG = 3) executes as a NOP (1 tick, DLY and side-set
still honoured) and sets sticky `STATUS.ILLEGAL`.

### 1.2 Enumerations

| Field | Values |
|-------|--------|
| JMP COND | 0 ALWAYS, 1 XZ (X==0), 2 XNZ, 3 YZ, 4 YNZ, 5 XDEC (jump if X!=0, then X=X-1 always), 6 YDEC, 7 XEQY, 8 XNEY, 9 XLTUY (X<Y unsigned), 10 PIN (jmp_pin high), 11 NPIN, 12 OSRE (OSR empty), 13 OSRNE, 14 ISRF (ISR count >= push_thresh), 15 RXF (RX FIFO full) |
| WAIT SRC | 0 GPIO (absolute pin IDX), 1 PIN (in_base+IDX, mod 32), 2 IRQ (flag, IDX[2:0] + REL), 3 XBIT (bit IDX of X) |
| REL (WAIT-IRQ, IRQ) | 0 absolute, 1 `+smid`, 2 `+smid+1`, 3 `+smid-1` — all mod 8 |
| IN SRC | 0 PINS, 1 X, 2 Y, 3 Z, 4 NULL, 5 STATUS, 6 ISR, 7 OSR |
| OUT DST | 0 PINS, 1 X, 2 Y, 3 Z, 4 NULL, 5 PINDIRS, 6 PC, 7 ISR, 8 EXEC |
| MOV DST | 0 PINS, 1 X, 2 Y, 3 Z, 4 PINDIRS, 5 PC, 6 ISR, 7 OSR, 8 EXEC |
| MOV SRC | 0 PINS, 1 X, 2 Y, 3 Z, 4 NULL, 5 STATUS, 6 ISR, 7 OSR |
| MOV MOP | 0 none, 1 invert (`~`), 2 bit-reverse (`rev`), 3 byte-swap (`swap`) |
| SET DST | 0 PINS, 1 X, 2 Y, 3 Z, 4 PINDIRS |
| ALU operand A/B (3 bit) | 0 X, 1 Y, 2 Z, 3 ISR, 4 OSR, 5 PINS, 6 STATUS, 7 NULL |
| ALU DST (2 bit) | 0 X, 1 Y, 2 Z (3 reserved) |
| ALUOP | 0 ADD, 1 SUB, 2 AND, 3 OR, 4 XOR, 5 SHL, 6 SHR, 7 = RSUB (ALUI: `IMM - DST`) / SLTU (ALUR: `A <u B ? 1 : 0`) |
| IRQ MODE | 0 SET, 1 CLEAR, 2 SETWAIT (set, then stall until the flag is cleared), 3 TOGGLE |

ALU details: 32-bit modular arithmetic. ALUI `IMM` is zero-extended 7 bits. SHL/SHR (logical): amount is
`IMM[4:0]` (ALUI) or `B[4:0]` (ALUR). `ALUR`: `DST = A op B`, with A/B read **before** the write.
Operand `PINS` is the input window value (§4.1). `STATUS` is `32'hFFFF_FFFF` if the STATUS condition holds else 0.

## 2. Per-SM state

`pc`, `dly` (remaining delay ticks), `X`,`Y`,`Z` (32), `ISR`,`OSR` (32) with counters `isr_cnt`, `osr_cnt` (0..32),
loop stack (depth 0..2; each entry `start`,`end`,`cnt[15:0]`), injected-instruction slot (`inj_valid`, `inj_instr[23:0]`),
IRQ-wait phase bit, pin level reg `lvl[31:0]` and pin direction reg `oe[31:0]`, sticky `ILLEGAL`, `LOOP_OVF`.

Reset / `SM_RESTART`: `pc = ENTRY`, `dly = 0`, `ISR = 0`, `isr_cnt = 0`, `OSR = 0`, `osr_cnt = 32` (empty),
loop stack empty, inj slot empty, IRQ-wait phase cleared, sticky stall/illegal/loop flags cleared, clkdiv phase reset.
`X`,`Y`,`Z`, `lvl`, `oe` are **not** changed by RESTART (they are 0 after reset).

## 3. Tick, clock divider, issue order

Each SM has a 16.8 fractional divider: `DIV = {INT[15:0], FRAC[7:0]}` (24-bit, unit 1/256 clk); `DIV < 256` behaves as `256` (÷1).
Exact tick rule: after the divider phase was reset (reset, `CLK_RESTART`, `SM_START`, `SM_RESTART`), the tick count after
`n` clocks is `floor(256*n / DIV)`; i.e. clock number `n` (n=1 is the first clock edge after the reset write took effect)
is a tick iff `floor(256*n/DIV) > floor(256*(n-1)/DIV)`. (Implemented as phase accumulator `ph += 256; if ph >= DIV: tick, ph -= DIV`.)
Changing `CLKDIV` takes effect immediately (phase is not reset).

On every clock where `tick` is true **and** (`SM_EN` bit set **or** `inj_valid`) the SM performs one *step*:

1. If `inj_valid`: instruction = `inj_instr`, source = INJ (executes even if the SM is disabled or in delay).
2. Else if `dly != 0`: `dly = dly - 1`; nothing else happens (pins hold; side-set is **not** re-applied).
3. Else instruction = `imem[pc]`, source = MEM.  (Skipped if the SM is disabled.)
4. Execute (§6). The instruction either **stalls** (no architectural change except the idempotent side-set, the
   stall counters, IRQ-wait phase) or **completes**.
5. On completion: `dly = DLY` of the instruction; if source == MEM the PC advances (§7); if source == INJ the PC is
   untouched (except explicit PC writes: JMP/OUT PC/MOV PC) and `inj_valid` is cleared **before** the instruction's own
   effects (so OUT EXEC / MOV EXEC re-arm it).

CPU write to `INSTR` sets `inj_valid=1`, `inj_instr=wdata[23:0]` (replacing any pending slot).
`OUT EXEC` / `MOV EXEC` set the slot to `value[23:0]`; it executes at the **next** step (one tick later, ignoring
`dly`). On completion of an INJ instruction `dly` is loaded from *its* DLY field.

## 4. Pins

`PIO_PINS` pads (param ≤ 32). All pin windows are taken modulo 32 and bits ≥ `PIO_PINS` are dropped.

### 4.1 Input window
`pin_in[31:0]` is the synchronised pad vector (per-pin 2-FF sync, optional bypass / loopback, see register map).
`inwin = ror32(pin_in, in_base) & mask(in_count)` where `in_count==0` means 32. `IN PINS`, `MOV x, pins`, ALU operand
PINS, `WAIT PIN` use the window; `JMP PIN` and `WAIT GPIO` use absolute pins (`jmp_pin`, IDX).

### 4.2 Output windows
* `OUT PINS, n` / `MOV PINS, v`: value bit `i` → pin `(out_base+i)%32` for `i < min(n_or_32, out_count)` (`out_count` 0..32; 0 = none).
  Writes `lvl` (OUT/MOV PINS) or `oe` (PINDIRS).
* `SET PINS/PINDIRS, imm`: imm bit `i` → pin `(set_base+i)%32` for `i < set_count` (0..8).
* Side-set: see §5.

### 4.3 Arbitration (core level)
Pad `p` is driven by the highest-numbered SM `s` with `oe_s[p] && permit_s[p]` (`permit` = per-SM PINMASK register,
reset all-ones); `pio_o[p]` = that SM's `lvl[p]`; `pio_oe_o[p]` = OR of all such `oe`. If two or more SMs qualify the
sticky `CONFLICT[p]` bit is set. Outputs are registered: an instruction executed at clock edge E changes `lvl`/`oe` at E,
so pads change in the cycle after the executing cycle.

## 5. Side-set
Config (PINCTRL): `side_base`, `side_count` (0..4), `side_opt`, `side_pindir`.
* `side_count == 0`: SS ignored.
* `side_opt == 0`: value = `SS & mask(side_count)`, always applied.
* `side_opt == 1` (use `side_count ≤ 3`): `SS[3]` is the enable; if set, value = `SS[2:0] & mask(side_count)` is applied.
Value bit `i` → pin `(side_base+i)%32`; written to `lvl` (or `oe` if `side_pindir`). Side-set is applied on **every** step in which
the instruction executes (including stalled steps — idempotent), **after** the instruction's own pin writes
(so side-set wins on overlap), and **not** on delay ticks. Applies for INJ instructions too.

## 6. Instruction semantics

Let `n = CNT==0 ? 32 : CNT`. `thr_push = push_thresh==0 ? 32 : push_thresh`, `thr_pull` likewise.
`osr_empty = (osr_cnt >= thr_pull)`, `isr_full = (isr_cnt >= thr_push)`.

* **JMP**: evaluate COND using values before the instruction. XDEC/YDEC: condition = (old != 0), register always decremented.
  Taken ⇒ `pc = ADDR[ABITS-1:0]` (counts as a PC write, §7). Not taken ⇒ normal advance.
* **WAIT**: GPIO/PIN/XBIT: stall until selected bit == POL. IRQ: stall until flag == POL; if POL==1, flag is cleared on the step
  that passes (clear loses against a simultaneous set, §8).
* **IN src, n**: `v = src` (PINS = inwin). RIGHT shift (`in_right=1`): `ISR' = (n==32) ? v : {v[n-1:0], ISR[31:n]}`;
  LEFT: `ISR' = (n==32) ? v : (ISR << n) | (v & mask(n))`. `isr_cnt' = min(32, isr_cnt + n)`.
  With `autopush`, if `isr_cnt' >= thr_push`: when the RX FIFO is full the step **stalls with no change**; else `ISR'` is pushed
  to RX FIFO, `ISR = 0`, `isr_cnt = 0` (same step).
* **OUT dst, n**: with `autopull`, if `osr_empty` *before* the shift: when TX FIFO empty ⇒ stall, no change; else
  `OSR = pop(TX)`, `osr_cnt = 0` and the shift below uses the new OSR (same step). Shift: RIGHT (`out_right=1`):
  `v = OSR & mask(n)` , `OSR >>= n`; LEFT: `v = OSR >> (32-n)` (top n bits; n==32: whole OSR), `OSR <<= n`. Zero fill.
  `osr_cnt = min(32, osr_cnt + n)`. Then write `v`: PINS (§4.2), X/Y/Z (zero-extended), NULL (discard), PINDIRS,
  PC (jump to `v[ABITS-1:0]`), ISR (`ISR = v`, `isr_cnt = n`), EXEC (`inj = v[23:0]`).
* **PUSH**: if IFFULL and `!isr_full` ⇒ NOP-complete. Else if RX not full ⇒ push ISR; `ISR=0,isr_cnt=0`. Else (full): BLOCK ⇒ stall;
  non-block ⇒ drop (ISR cleared, count 0) and set sticky `RXDROP`.
* **PULL**: if IFEMPTY and `!osr_empty` ⇒ NOP-complete. Else if TX not empty ⇒ `OSR=pop`, `osr_cnt=0`. Else BLOCK ⇒ stall;
  non-block ⇒ `OSR = X`, `osr_cnt=0`, set sticky `TXUNDER`.
* **MOV dst, [mop] src**: `v = src`; MOP applied (invert / reverse 32-bit bit order / reverse byte order). DST: PINS (§4.2 with
  n=32), X/Y/Z, PINDIRS, PC (jump), ISR (`ISR=v, isr_cnt=0`), OSR (`OSR=v, osr_cnt=0`), EXEC.
* **ALUR/ALUI**: see §1.2. **SET**: PINS/PINDIRS (§4.2) , X/Y/Z = zero-extended IMM.
* **LOOP/LOOPR**: see §7.2. **IRQ**: flag `f = (IDX + off(REL, smid)) & 7`. SET: flag=1. CLEAR: flag=0. TOGGLE: flag ^= 1.
  SETWAIT: first execution sets flag and enters *wait phase* (stall); while in wait phase the step passes (and leaves the
  phase) when flag==0; the flag is not re-set during retries.
* **NOP**, reserved: no effect.

Stalled steps count in `STALLS`; reason codes: 1 WAIT, 2 PULL(block, empty), 3 PUSH/autopush (full), 4 autopull (empty), 5 IRQ SETWAIT.

## 7. Program counter

### 7.1 Advance
`wrap_next(pc) = (pc == wrap_top) ? wrap_bottom : pc + 1` (both `ABITS` wide). A completed MEM instruction sets
`pc = jump_target` if it wrote the PC (taken JMP, OUT PC, MOV PC), else loop-back (§7.2), else `wrap_next(pc)`.

### 7.2 Zero-overhead loops
`LOOP n, len` at address `a` (MEM source only; as INJ it is NOP + ILLEGAL): `start = wrap_next(a)`, `end = (a + 1 + LEN) mod 2^ABITS`.
Count `c = COUNT` (LOOPR: `reg[15:0]`).  If `c == 0`: skip the body: `pc = (a + 2 + LEN) mod 2^ABITS`, no push.
Else push `{start,end,cnt=c}`; `pc = start`. If depth is already 2: set sticky `LOOP_OVF`, treat as NOP (no push, normal advance).
When a MEM instruction at `pc == end(top)` completes **without writing the PC**: `cnt -= 1`; if `cnt != 0` ⇒ `pc = start` else pop and
`pc = wrap_next(pc)`. (Outer loop end check does *not* cascade in the same step: nested loops sharing the same last instruction are
illegal.) A taken PC write whose target lies outside `[start, end]` of the innermost entry pops it, repeating for the next entry;
a target inside keeps the stack. (Range test is plain unsigned `start <= t <= end`, assuming `start <= end` i.e. body does not cross the
end of the address space.) The loop-end check is skipped for INJ instructions.

## 8. IRQ flags (8, shared by all SMs and the CPU)
Same-cycle collisions on one flag: set (SM `IRQ set/toggle-to-1`, CPU `IRQ_SET`) wins over clear (SM clear, WAIT-IRQ consume, CPU `IRQ_FLAGS` W1C).
Normative per-flag update (OR of requests from all SMs and the CPU): `flag' = set ? 1 : (clr ? 0 : (tgl ? ~flag : flag))`. A TOGGLE is a *toggle request*, so two SMs toggling the same flag in one cycle toggle it once.

## 9. FIFOs
Per SM: TX FIFO (CPU→SM) and RX FIFO (SM→CPU), 8 × 32-bit each, first-word-fall-through. `JOIN_TX` makes TX 16 deep and RX 0 deep (RX storage chained in series
behind the CPU write port); `JOIN_RX` the opposite. If both bits are set neither join is active. Changing the join bits flushes both FIFOs.
CPU write to a full TX FIFO is dropped (`TXOVER`), CPU read of an empty RX returns 0 (`RXUNDER`). `STATUS` source condition per INCTRL
(`status_sel`: 0 TXLVL < N, 1 RXLVL < N, 2 any IRQ flag set, 3 never).
DMA/IRQ level: `tx_low = tx_cap != 0 && txlvl < TX_THR`, `rx_high = rx_cap != 0 && rxlvl >= RX_THR`.

## 10. Assembler syntax (normative summary; the manual lives in `doc/ASSEMBLER.md`)

```
; comment   // comment
.program uart_tx
.side_set 1 [opt] [pindirs]     ; side_count[, opt][, pindir]
.origin 0                       ; load address (default 0)
.define BIT_CYCLES 8
.wrap_target                    ; wrap_bottom = next instruction
.wrap                           ; wrap_top = previous instruction (default: last instr / first instr)
.entry label                    ; SM start PC (default = origin)
label:
    set  pins, 1        [3] side 1      ; delay in [], side-set value after 'side'
    jmp  xnz, label                      ; jmp [cond,] target   (cond names in §1.2, lower-case)
    djnz x, label                        ; alias of 'jmp xdec, label' (also y)
    wait 1 gpio 3 | wait 0 pin 2 | wait 1 irq 4 [rel|next|prev] | wait 0 xbit 5
    in   pins, 8 | out x, 4 | push [iffull] [noblock] | pull [ifempty] [noblock]
    mov  x, ~y | mov isr, rev(x) | mov y, swap(osr) | mov exec, osr
    add x, 5 | sub y, 1 | and x, 0x7f | or | xor | shl | shr | rsub     ; immediate form: dst op= imm
    add x, y, z | sltu z, x, y                                           ; register form: dst = a op b (2-operand reg form: dst = dst op b)
    set  pins|x|y|z|pindirs, imm
    loop 8 / ... / endloop                ; assembler computes LEN; also 'loop x' (LOOPR); nesting ≤ 2
    irq  set|clear|setwait|toggle 3 [rel|next|prev]
    nop | halt                            ; halt = 'jmp always, .'  (jump to self)
```
Output: `.hex` (24-bit words as 6 hex digits, one per line, word 0 = `.origin`), C header, SV `localparam` header (`--svh`) with
`WRAP_BOTTOM/WRAP_TOP/ENTRY/SIDE_*` and listing (`--lst`).
