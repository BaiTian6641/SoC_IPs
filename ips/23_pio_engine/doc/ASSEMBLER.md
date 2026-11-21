# `pioasm.py` — PIO engine assembler manual

`tools/pioasm.py` assembles programs for the PIO engine ISA v1 (normative: [ISA.md](ISA.md)) into
24-bit instruction words, and disassembles hex back to source. Pure Python 3 standard library,
no installation. Tests: `python3 tools/test_pioasm.py` (stdlib `unittest`).

```
python3 tools/pioasm.py prog.pio --hex prog.hex --svh prog.svh [--c prog.h] [--lst prog.lst] [--meta]
python3 tools/pioasm.py --disasm prog.hex [--side-set "1 opt pindirs"] [--name n] [-o prog.pio]
```

## 1. Command line

| Option | Meaning |
|--------|---------|
| `input` | `.pio` source (or `.hex` with `--disasm`) |
| `--hex F` | `$readmemh` file: one 6-hex-digit word per line, first word = `.origin`; if origin > 0 a first line `@<origin in hex>` is emitted (the `$readmemh` address syntax, no `0x`) |
| `--c F` | C header: `#define <NAME>_{ORIGIN,LENGTH,WRAP_BOTTOM,WRAP_TOP,ENTRY,SIDE_COUNT,SIDE_OPT,SIDE_PINDIR}` + `static const uint32_t <name>_program[<NAME>_LENGTH]` |
| `--svh F` | SystemVerilog header: `localparam int unsigned <NAME>_{ORIGIN,LENGTH,WRAP_BOTTOM,WRAP_TOP,ENTRY,SIDE_COUNT}`, `localparam bit <NAME>_{SIDE_OPT,SIDE_PINDIR}`, `localparam logic [23:0] <NAME>_INSTR [0:LENGTH-1]`; `<NAME>` = upper-case program name; include-guarded (`` `include `` it inside a module/package/`$unit`) |
| `--lst F` | listing (`addr word line source`) |
| `--meta` | prepend `// .program/.side_set/.wrap_target/.wrap/.entry` comment lines to the hex file (legal in `$readmemh`); `--disasm` reads them back |
| `--disasm` | hex → source (see §6) |
| `--side-set "N [opt] [pindirs]"` | `--disasm`: side-set configuration (otherwise taken from `--meta` comments, otherwise inferred: smallest non-opt count covering all SS values) |
| `-D NAME=VALUE` | global `.define` |
| `--imem-depth N` | instruction memory depth for the `.origin`/length checks (default 256 = 8-bit jump address space) |
| `-q` | suppress warnings |

* `-` as a file name writes to stdout.
* **Several `.program` blocks in one file**: every output file name gets `_<program>` inserted before the
  extension (`--hex o.hex` → `o_uart_tx.hex`, `o_spi.hex`). With exactly one program the name is used verbatim.
* All wrap/entry/label values written to outputs are **absolute instruction-memory addresses** (origin included),
  matching the PC/`WRAP_*`/`ENTRY` registers.
* Exit status 0 = ok; 1 = assembly errors (all of them are listed, **no** output files are written); 2 = usage/IO.

## 2. Source syntax

```
; comment      // comment                      (to end of line)
.define BIT_CYCLES 8
.program uart_tx
.side_set 1 opt pindirs
.origin 0
.wrap_target
label:  mnemonic operands  [delay]  side value
.wrap
.entry label
```

* Mnemonics, registers and keywords are case-insensitive; labels/defines are case-sensitive.
* One instruction per line, any number of labels (`name:`) before it (also on a line of their own).
* Operand separators: `,` between the operands of `jmp/djnz/in/out/mov/set/ALU/loop`; plain spaces for `wait`, `irq`,
  `push`, `pull`.
* Trailing modifiers (either order): `[n]` delay 0..15 (`n` is an expression, extra idle ticks after completion) and
  `side v`.
* Reserved names (not usable as label/define): all mnemonics, `x y z isr osr pins pindirs null status pc exec side rev swap`.

### 2.1 Directives

| Directive | Description |
|-----------|-------------|
| `.program NAME` | start a program (ends the previous). Names must be unique in a file. Everything below belongs to it |
| `.define NAME EXPR` | constant; before the first `.program` it is global, inside it is program-local (shadows a global). Evaluated lazily, may reference other defines and labels, cycles are errors. `.` is not allowed |
| `.side_set N [opt] [pindirs]` | `N` = 0..4 side-set bits. `opt`: `SS[3]` becomes the enable bit; needs `N`=1..3. `pindirs` (or `pindir`): documents `side_pindir` (the PINCTRL bit is set by software; the assembler only reports it in the headers). Must precede the first instruction |
| `.origin EXPR` | load address (default 0); before the first instruction; defines only (no labels) |
| `.wrap_target` | `wrap_bottom` = address of the next instruction (default: first instruction) |
| `.wrap` | `wrap_top` = address of the previous instruction (default: last instruction) |
| `.entry EXPR` | start PC (default: origin); must lie inside the program |
| `.word EXPR` | emit a raw 24-bit word (used by the disassembler for non-canonical/reserved encodings and for tests of the `ILLEGAL` flag) |

### 2.2 Expressions

Integers: decimal, `0x..`, `0b..`, `_` separators allowed. Operators, high → low precedence (C-like):
unary `- + ~`; `* /` (division truncates toward zero); `+ -`; `<< >>`; `&`; `^`; `|`; parentheses.
Names: defines and labels (absolute address); `.` = address of the current instruction (`jmp . + 2`).
Forward label references are fine.

### 2.3 Side-set and delay

* `side v` requires `.side_set`; `0 <= v < 2^N`.
* `.side_set N` (no opt): `SS = v`; when `side` is omitted `SS = 0` — i.e. the side pins are **driven to 0**
  by such an instruction (ISA §5: always applied).
* `.side_set N opt`: with `side v` → `SS = 8 | v` (enable set); without → `SS = 0` (side-set not applied).
* Delay: `DLY = n`, 0..15. Delay and side-set are encoded in separate fields, so nothing is shared (unlike
  other PIO designs).

## 3. Instruction reference

`ENC` is the 24-bit word with `DLY[23:20] SS[19:16] OP[15:12] ARG[11:0]`; only OP/ARG shown.

| Syntax | OP | ARG |
|--------|----|-----|
| `jmp [cond,] target` | 0 | `cond<<8 \| target` (0..255, absolute) |
| `djnz x\|y, target` | 0 | `jmp xdec/ydec, target` (cond 5 / 6) |
| `halt` | 0 | `jmp always, .` (jump to itself; ADDR = own absolute address, must be ≤ 255) |
| `wait POL gpio\|pin\|xbit IDX` | 1 | `POL<<11 \| SRC<<9 \| IDX` (IDX 0..31) |
| `wait POL irq IDX [rel\|next\|prev]` | 1 | `POL<<11 \| 2<<9 \| REL<<7 \| IDX` (IDX 0..7; REL 1 `rel`=+smid, 2 `next`=+smid+1, 3 `prev`=+smid−1) |
| `in SRC, n` | 2 | `SRC<<8 \| n&31` (n 1..32, 32 encodes as 0) |
| `out DST, n` | 3 | `DST<<8 \| n&31` |
| `push [iffull] [block\|noblock]` | 4 | `IFFULL<<10 \| BLOCK<<9` (default `block`) |
| `pull [ifempty] [block\|noblock]` | 4 | `1<<11 \| IFEMPTY<<10 \| BLOCK<<9` |
| `mov DST, [~\|rev()\|swap()] SRC` | 5 | `DST<<8 \| MOP<<6 \| SRC` (`!` accepted as `~`) |
| `add\|sub\|and\|or\|xor\|shl\|shr\|rsub x\|y\|z, imm` | 7 | `DST<<10 \| ALUOP<<7 \| imm` (imm 0..127; shl/shr 0..31) — ALUI, `dst = dst op imm` |
| `add\|sub\|and\|or\|xor\|shl\|shr\|sltu DST, A, B` | 6 | `DST<<10 \| ALUOP<<7 \| A<<4 \| B<<1` — ALUR |
| `OP DST, B` (register) | 6 | same, with `A = DST` |
| `set DST, imm` | 8 | `DST<<8 \| imm` (0..255) |
| `loop N` … `endloop` | 9 | `LEN<<8 \| N` (N 0..255; `LEN` = body length − 1, computed) |
| `loop x\|y\|z` … `endloop` | 10 | `LEN<<8 \| REG<<6` |
| `loop N, LEN` / `loop x, LEN` | 9 / 10 | raw form without `endloop` (LEN 0..15); what the disassembler emits |
| `irq set\|clear\|setwait\|toggle IDX [rel\|next\|prev]` | 11 | `MODE<<10 \| REL<<8 \| IDX` (IDX 0..7) |
| `nop` | 12 | 0 |

Operand/enum tables are exactly ISA §1.2 (lower-case names): jmp conditions `always xz xnz yz ynz xdec ydec xeqy
xney xltuy pin npin osre osrne isrf rxf`; `in` sources `pins x y z null status isr osr`; `out` destinations
`pins x y z null pindirs pc isr exec`; `mov` destinations `pins x y z pindirs pc isr osr exec`, sources as `in`;
`set` destinations `pins x y z pindirs`; ALU operands `x y z isr osr pins status null`.

Notes:

* `jmp cond, target`: a leading identifier is a condition only if followed by `,`; so a label named like a
  condition (`pin`) still works as `jmp pin`.
* ALU: the second operand decides the form: a register name → ALUR, otherwise an expression → ALUI. `rsub` exists
  only as immediate (`dst = imm - dst`), `sltu` only with registers (`dst = a <u b ? 1 : 0`); other combinations
  are errors. ALU dst is x, y or z.
* `out exec`/`mov exec` inject the low 24 bits as an instruction; `.word` is handy to build such values.

### 3.1 Structured loops

```
    loop 8              ; or: loop x   (count taken from X[15:0] when the loop instruction executes)
        out pins, 1 [1]
        nop
    endloop             ; assembler emits nothing for endloop
```

* `LEN` = (number of instruction words between `loop` and `endloop`) − 1; body 1..16 words (nested `loop`
  words are included).
* Nesting ≤ 2 (ISA §7.2). Two loops may **not** end on the same instruction (the outer end check does not cascade):
  put at least one instruction between the inner and outer `endloop`.
* `loop` may carry `[delay]` / `side` (applied when the loop instruction completes).
* Checks that depend on wrap: the `loop` instruction must not be the `.wrap` instruction, `.wrap` must not lie
  *inside* the body (it may be the last body instruction); a `loop <reg>` whose last body instruction is the wrap
  top yields a warning (a zero count skips the body without applying wrap).
* A count of 0 skips the body (hardware semantics).

## 4. Errors

All errors are reported as `file:line: error: message`; the assembler continues after an error and reports every
problem it finds, writes nothing, and exits 1. Typical messages:

| Message | Cause |
|---------|-------|
| `unknown mnemonic 'x'`, `unknown directive '.x'` | typo |
| `instruction outside .program (missing .program?)` | code before `.program` |
| `undefined symbol 'x'` | unknown label/define (`(register/keyword names are not values)` hint for reserved names) |
| `duplicate label/program/.define/.wrap/.entry` | redefinition |
| `'x' is a reserved word and cannot be a label` | reserved name |
| `circular .define involving 'x'` | recursive define |
| `delay 16 out of range 0..15` | delay |
| `'side' used but program has no .side_set`, `side value 4 out of range 0..3 (.side_set 2)` | side-set |
| `.side_set count 5 out of range 0..4`, `.side_set ... opt needs a bit count of 1..3` | directive |
| `jump target 256 out of range 0..255`, `set value 256 out of range 0..255`, `bit count 33 out of range 1..32`, `immediate ...`, `shift amount ...`, `irq flag ...`, `loop count ...` | operand ranges |
| `expected destination (pins, x, ...), got 'status'` | wrong enum for that instruction |
| `expected ',', got ...`, `unexpected 'tok'` | syntax |
| `.origin must come before the first instruction`, `.side_set must come before ...` | ordering |
| `program does not fit: origin O + N words > 256`, `program exceeds instruction memory` | size |
| `.wrap_target (a) is after .wrap (b)`, `.wrap needs a preceding instruction`, `.wrap_target after the last instruction` | wrap |
| `.entry v outside program` | entry |
| `'loop' without matching 'endloop'`, `'endloop' without matching 'loop'`, `empty loop body`, `loop nesting deeper than 2`, `loop body has N instructions (max 16)`, `nested loops must not end on the same instruction` | loops |
| `wrap point (.wrap) lies inside a loop body`, `'loop' sits on the .wrap instruction` | loop/wrap interaction |
| `division by zero in expression` | expression |

## 5. Examples

UART transmitter (8N1, `BIT_CYCLES` ticks per bit, 1 side-set pin = TX line, `pull` blocks while idle):

```
.define BIT_CYCLES 8
.program uart_tx
.side_set 1 opt
        pull block   side 1             ; idle high, wait for data
        set  x, 7    side 0 [BIT_CYCLES-1]   ; start bit
bitloop:
        out  pins, 1                  [BIT_CYCLES-2]
        djnz x, bitloop
        nop          side 1 [BIT_CYCLES-1]   ; stop bit
```

Zero-overhead loop shifting 32 bits, then a handshake:

```
.program shift32
.wrap_target
        pull
        loop 31
            out pins, 1 [3]
        endloop
        out pins, 1
        irq set 0 rel
.wrap
```

Wait-for-edge counter with ALU:

```
.program edge_cnt
        set  x, 0
again:  wait 0 pin 0
        wait 1 pin 0
        add  x, 1
        jmp  again
```

Global defines / multiple programs, outputs named `o_a.hex`, `o_b.hex`:

```
.define N 4
.program a
    set x, N
.program b
    mov y, ~x
```

Build lines:

```
python3 tools/pioasm.py ips/23_pio_engine/prog/uart_tx.pio --hex build/uart_tx.hex --svh build/uart_tx.svh
python3 tools/pioasm.py --disasm build/uart_tx.hex --side-set "1 opt" -o /dev/stdout
```

Testbench use: `$readmemh("uart_tx.hex", imem)` (the `@origin` line positions the words), or
`` `include "uart_tx.svh" `` and use `UART_TX_INSTR`, `UART_TX_WRAP_BOTTOM/TOP`, `UART_TX_ENTRY`.
Remember to program `side_count/side_opt/side_pindir` (PINCTRL) from `UART_TX_SIDE_*`.

## 6. Disassembler

`--disasm file.hex` accepts `$readmemh` text (comments `//`, `#`, `;`; an optional leading `@addr`; several words per
line allowed; `// .directive value` meta lines from `--meta`). It prints source for one program:

* jump targets inside the program get labels `L<addr>`; `jmp always, <self>` prints as `halt`;
* `loop` words print in raw form (`loop 8, 3`, `loop x, 3`), which the assembler accepts directly;
* the side-set configuration is not stored in the words: use `--side-set` (or `--meta` hex, or the inference);
* a word that has **no canonical assembler form** — reserved opcodes (0xD–0xF), reserved enum values, non-zero
  reserved bits, SS bits set while `side_count == 0` or `SS[3]=0` with `opt` — prints as `.word 0x......`.

Round-trip guarantee (checked by `test_pioasm.py` on randomised words): `assemble(disassemble(words)) == words`.

## 7. Python API

```python
import pioasm
progs = pioasm.assemble(text, "file.pio")        # list of Program; raises pioasm.AsmFailure(.errors)
p = progs[0]    # .name .origin .words .wrap_bottom_abs .wrap_top_abs .entry .side (SideCfg) .warnings
text = pioasm.disassemble(p.words, p.origin, p.side, p.name, p.wrap_bottom_abs, p.wrap_top_abs, p.entry)
hexs = pioasm.format_hex(p, meta=True); origin, words, meta = pioasm.parse_hex(hexs)
```
