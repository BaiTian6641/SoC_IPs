#!/usr/bin/env python3
"""Unit tests for tools/pioasm.py (stdlib unittest).

Run from repo root:  python3 -m unittest discover -s tools -p 'test_pioasm.py'
                or:  python3 tools/test_pioasm.py
Expected words are computed here from the ISA.md bit layout, independently of
the assembler's tables.
"""
import io
import os
import random
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pioasm  # noqa: E402


def _r(path):
    with open(path) as f:
        return f.read()


def _w(path, text):
    with open(path, "w") as f:
        f.write(text)


def W(dly=0, ss=0, op=0, arg=0):
    return (dly << 20) | (ss << 16) | (op << 12) | arg


def asm1(line, side=""):
    """Assemble one instruction (plus optional .side_set directive)."""
    ps = pioasm.assemble(".program t\n%s\n%s\n" % (side, line))
    return ps[0].words[0]


def asm_words(src):
    return pioasm.assemble(src)[0].words


def err(src):
    """Return list of (line, msg) for a failing assembly."""
    try:
        pioasm.assemble(src)
    except pioasm.AsmFailure as e:
        return [(l, m) for _f, l, m in e.errors]
    raise AssertionError("expected assembly failure for:\n" + src)


def err1(line, side=""):
    return err(".program t\n%s\n%s\n" % (side, line))


class TestEncoding(unittest.TestCase):
    def test_jmp(self):
        conds = ["always", "xz", "xnz", "yz", "ynz", "xdec", "ydec", "xeqy",
                 "xney", "xltuy", "pin", "npin", "osre", "osrne", "isrf", "rxf"]
        for i, c in enumerate(conds):
            self.assertEqual(asm1("jmp %s, 0" % c), W(op=0, arg=(i << 8)))
            self.assertEqual(asm1("jmp %s, 255" % c), W(op=0, arg=(i << 8) | 255))
        self.assertEqual(asm1("jmp 17"), W(arg=17))
        self.assertEqual(asm1("jmp always, 5"), W(arg=5))
        self.assertEqual(asm1("JMP XNZ, 0x10"), W(arg=0x210))

    def test_djnz_halt_nop(self):
        self.assertEqual(asm1("djnz x, 3"), W(arg=(5 << 8) | 3))
        self.assertEqual(asm1("djnz y, 3"), W(arg=(6 << 8) | 3))
        # halt = jmp always, . ; at address 0 (origin 0) => target 0
        self.assertEqual(asm1("halt"), W(arg=0))
        self.assertEqual(asm_words(".program t\nnop\nhalt\n"), [W(op=0xC), W(arg=1)])
        self.assertEqual(asm_words(".program t\n.origin 9\nhalt\n"), [W(arg=9)])
        self.assertEqual(asm1("nop"), W(op=0xC))
        self.assertEqual(asm1("nop [15] side 1", ".side_set 1"), W(15, 1, 0xC))

    def test_wait(self):
        self.assertEqual(asm1("wait 1 gpio 3"), W(op=1, arg=(1 << 11) | (0 << 9) | 3))
        self.assertEqual(asm1("wait 0 gpio 31"), W(op=1, arg=31))
        self.assertEqual(asm1("wait 0 pin 2"), W(op=1, arg=(1 << 9) | 2))
        self.assertEqual(asm1("wait 1 irq 4"), W(op=1, arg=(1 << 11) | (2 << 9) | 4))
        self.assertEqual(asm1("wait 1 irq 7 rel"), W(op=1, arg=(1 << 11) | (2 << 9) | (1 << 7) | 7))
        self.assertEqual(asm1("wait 0 irq 0 next"), W(op=1, arg=(2 << 9) | (2 << 7)))
        self.assertEqual(asm1("wait 0 irq 0 prev"), W(op=1, arg=(2 << 9) | (3 << 7)))
        self.assertEqual(asm1("wait 0 xbit 5"), W(op=1, arg=(3 << 9) | 5))
        self.assertEqual(asm1("wait 1 xbit 31"), W(op=1, arg=(1 << 11) | (3 << 9) | 31))

    def test_in_out(self):
        src = ["pins", "x", "y", "z", "null", "status", "isr", "osr"]
        for i, s in enumerate(src):
            self.assertEqual(asm1("in %s, 8" % s), W(op=2, arg=(i << 8) | 8))
        self.assertEqual(asm1("in pins, 1"), W(op=2, arg=1))
        self.assertEqual(asm1("in x, 31"), W(op=2, arg=(1 << 8) | 31))
        self.assertEqual(asm1("in x, 32"), W(op=2, arg=(1 << 8)))     # 32 -> 0
        dst = ["pins", "x", "y", "z", "null", "pindirs", "pc", "isr", "exec"]
        for i, d in enumerate(dst):
            self.assertEqual(asm1("out %s, 4" % d), W(op=3, arg=(i << 8) | 4))
        self.assertEqual(asm1("out exec, 32"), W(op=3, arg=8 << 8))

    def test_push_pull(self):
        self.assertEqual(asm1("push"), W(op=4, arg=1 << 9))
        self.assertEqual(asm1("push block"), W(op=4, arg=1 << 9))
        self.assertEqual(asm1("push noblock"), W(op=4, arg=0))
        self.assertEqual(asm1("push iffull"), W(op=4, arg=(1 << 10) | (1 << 9)))
        self.assertEqual(asm1("push iffull noblock"), W(op=4, arg=1 << 10))
        self.assertEqual(asm1("push noblock iffull"), W(op=4, arg=1 << 10))
        self.assertEqual(asm1("pull"), W(op=4, arg=(1 << 11) | (1 << 9)))
        self.assertEqual(asm1("pull noblock"), W(op=4, arg=1 << 11))
        self.assertEqual(asm1("pull ifempty"), W(op=4, arg=(1 << 11) | (1 << 10) | (1 << 9)))
        self.assertEqual(asm1("pull ifempty noblock"), W(op=4, arg=(1 << 11) | (1 << 10)))

    def test_mov(self):
        dst = ["pins", "x", "y", "z", "pindirs", "pc", "isr", "osr", "exec"]
        src = ["pins", "x", "y", "z", "null", "status", "isr", "osr"]
        for i, d in enumerate(dst):
            self.assertEqual(asm1("mov %s, x" % d), W(op=5, arg=(i << 8) | 1))
        for i, s in enumerate(src):
            self.assertEqual(asm1("mov x, %s" % s), W(op=5, arg=(1 << 8) | i))
            self.assertEqual(asm1("mov x, ~%s" % s), W(op=5, arg=(1 << 8) | (1 << 6) | i))
            self.assertEqual(asm1("mov x, ~ %s" % s), W(op=5, arg=(1 << 8) | (1 << 6) | i))
            self.assertEqual(asm1("mov x, rev(%s)" % s), W(op=5, arg=(1 << 8) | (2 << 6) | i))
            self.assertEqual(asm1("mov x, swap(%s)" % s), W(op=5, arg=(1 << 8) | (3 << 6) | i))
        self.assertEqual(asm1("mov exec, osr"), W(op=5, arg=(8 << 8) | 7))
        self.assertEqual(asm1("mov y, swap( osr )"), W(op=5, arg=(2 << 8) | (3 << 6) | 7))

    def test_alu_imm(self):
        ops = ["add", "sub", "and", "or", "xor", "shl", "shr", "rsub"]
        for d, dn in enumerate("xyz"):
            for o, on in enumerate(ops):
                hi = 31 if on in ("shl", "shr") else 127
                self.assertEqual(asm1("%s %s, 0" % (on, dn)), W(op=7, arg=(d << 10) | (o << 7)))
                self.assertEqual(asm1("%s %s, %d" % (on, dn, hi)),
                                 W(op=7, arg=(d << 10) | (o << 7) | hi))
        self.assertEqual(asm1("add x, 5"), W(op=7, arg=5))
        self.assertEqual(asm1("sub y, 1"), W(op=7, arg=(1 << 10) | (1 << 7) | 1))
        self.assertEqual(asm1("and x, 0x7f"), W(op=7, arg=(2 << 7) | 0x7F))

    def test_alu_reg(self):
        regs = ["x", "y", "z", "isr", "osr", "pins", "status", "null"]
        ops = ["add", "sub", "and", "or", "xor", "shl", "shr", "sltu"]
        for o, on in enumerate(ops):
            for a, an in enumerate(regs):
                for b, bn in enumerate(regs):
                    if (a * 8 + b) % 5:   # sample, keeps runtime small
                        continue
                    self.assertEqual(asm1("%s z, %s, %s" % (on, an, bn)),
                                     W(op=6, arg=(2 << 10) | (o << 7) | (a << 4) | (b << 1)))
        # 2-operand register form: dst = dst op b
        self.assertEqual(asm1("add x, y"), W(op=6, arg=(0 << 10) | (0 << 7) | (0 << 4) | (1 << 1)))
        self.assertEqual(asm1("sltu y, osr"), W(op=6, arg=(1 << 10) | (7 << 7) | (1 << 4) | (4 << 1)))
        self.assertEqual(asm1("add x, y, z"), W(op=6, arg=(0 << 7) | (1 << 4) | (2 << 1)))
        self.assertEqual(asm1("sltu z, x, y"), W(op=6, arg=(2 << 10) | (7 << 7) | (0 << 4) | (1 << 1)))

    def test_set(self):
        dst = ["pins", "x", "y", "z", "pindirs"]
        for i, d in enumerate(dst):
            self.assertEqual(asm1("set %s, 0" % d), W(op=8, arg=i << 8))
            self.assertEqual(asm1("set %s, 255" % d), W(op=8, arg=(i << 8) | 255))

    def test_irq(self):
        modes = ["set", "clear", "setwait", "toggle"]
        for m, mn in enumerate(modes):
            self.assertEqual(asm1("irq %s 0" % mn), W(op=11, arg=m << 10))
            self.assertEqual(asm1("irq %s 7" % mn), W(op=11, arg=(m << 10) | 7))
            for r, rn in ((1, "rel"), (2, "next"), (3, "prev")):
                self.assertEqual(asm1("irq %s 3 %s" % (mn, rn)),
                                 W(op=11, arg=(m << 10) | (r << 8) | 3))

    def test_delay_side(self):
        self.assertEqual(asm1("nop [0]"), W(op=12))
        self.assertEqual(asm1("nop [15]"), W(15, 0, 12))
        self.assertEqual(asm1("set pins, 1 [3]"), W(3, 0, 8, 1))
        # no opt: SS = value; unspecified = 0
        for v in range(16):
            self.assertEqual(asm1("nop side %d" % v, ".side_set 4"), W(0, v, 12))
        self.assertEqual(asm1("nop", ".side_set 4"), W(op=12))
        self.assertEqual(asm1("nop side 1 [2]", ".side_set 2"), W(2, 1, 12))
        self.assertEqual(asm1("nop [2] side 3", ".side_set 2"), W(2, 3, 12))
        self.assertEqual(asm1("nop side 3", ".side_set 2 pindirs"), W(0, 3, 12))
        # opt: SS[3] enable
        for v in range(8):
            self.assertEqual(asm1("nop side %d" % v, ".side_set 3 opt"), W(0, 8 | v, 12))
        self.assertEqual(asm1("nop", ".side_set 3 opt"), W(op=12))
        self.assertEqual(asm1("nop side 0", ".side_set 1 opt"), W(0, 8, 12))
        self.assertEqual(asm1("nop side 1 [15]", ".side_set 1 opt pindirs"), W(15, 9, 12))

    def test_loop(self):
        w = asm_words(".program t\nloop 5\nnop\nnop\nendloop\nnop\n")
        self.assertEqual(w, [W(op=9, arg=(1 << 8) | 5), W(op=12), W(op=12), W(op=12)])
        w = asm_words(".program t\nloop 0\nnop\nendloop\nnop\n")
        self.assertEqual(w[0], W(op=9, arg=0))
        w = asm_words(".program t\nloop 255 [3]\n" + "nop\n" * 16 + "endloop\nnop\n")
        self.assertEqual(w[0], W(3, 0, 9, (15 << 8) | 255))
        for r, rn in enumerate("xyz"):
            w = asm_words(".program t\nloop %s\nnop\nendloop\nnop\n" % rn)
            self.assertEqual(w[0], W(op=10, arg=r << 6))
        w = asm_words(".program t\nloop x\nnop\nnop\nnop\nendloop\nnop\n")
        self.assertEqual(w[0], W(op=10, arg=2 << 8))
        # nesting 2, inner LEN
        w = asm_words(".program t\nloop 3\nloop y\nset x, 1\nendloop\nnop\nendloop\nnop\n")
        self.assertEqual(w[0], W(op=9, arg=(2 << 8) | 3))
        self.assertEqual(w[1], W(op=10, arg=(0 << 8) | (1 << 6)))
        # raw form
        w = asm_words(".program t\nloop 7, 3\n" + "nop\n" * 4)
        self.assertEqual(w[0], W(op=9, arg=(3 << 8) | 7))
        w = asm_words(".program t\nloop z, 15\n" + "nop\n" * 16)
        self.assertEqual(w[0], W(op=10, arg=(15 << 8) | (2 << 6)))

    def test_reserved_word_directive(self):
        self.assertEqual(asm_words(".program t\n.word 0xDEADBE & 0xFFFFFF\nnop\n")[0], 0xDEADBE)


class TestExpressions(unittest.TestCase):
    def val(self, expr):
        return asm1("set x, %s" % expr) & 0xFF if False else \
            asm_words(".program t\n.define V (%s)\nset x, V\n" % expr)[0] & 0xFF

    def test_ops(self):
        self.assertEqual(self.val("1+2*3"), 7)
        self.assertEqual(self.val("(1+2)*3"), 9)
        self.assertEqual(self.val("1<<4"), 16)
        self.assertEqual(self.val("0x80>>3"), 16)
        self.assertEqual(self.val("0xF0&0x3C"), 0x30)
        self.assertEqual(self.val("0xF0|0x0F"), 0xFF)
        self.assertEqual(self.val("7 - 2 - 1"), 4)
        self.assertEqual(self.val("100/7"), 14)
        self.assertEqual(self.val("0b1010 ^ 0b0110"), 12)
        self.assertEqual(self.val("1 + 2 << 1"), 6)       # shift lower than +
        self.assertEqual(self.val("1 | 2 & 3"), 3)
        self.assertEqual(self.val("-1 + 3"), 2)
        self.assertEqual(self.val("1_0"), 10)

    def test_defines_and_labels(self):
        w = asm_words(".define A 3\n.program t\n.define B A*2\nset x, B+A\njmp end\nnop\nend: nop\n")
        self.assertEqual(w[0], W(op=8, arg=(1 << 8) | 9))
        self.assertEqual(w[1], W(arg=3))                  # forward label
        w = asm_words(".program t\n.define L2 L1+1\nL1: nop\njmp L2\n")
        self.assertEqual(w[1], W(arg=1))                  # define referencing label

    def test_dot(self):
        w = asm_words(".program t\n.origin 4\nnop\njmp .\njmp . + 2\njmp .-1\n")
        self.assertEqual([x & 0xFF for x in w[1:]], [5, 8, 6])

    def test_labels_with_origin_and_same_line(self):
        w = asm_words(".program t\n.origin 10\na: b: nop\njmp a\njmp b\n")
        self.assertEqual([x & 0xFF for x in w[1:]], [10, 10])


class TestProgramMeta(unittest.TestCase):
    def test_wrap_entry(self):
        p = pioasm.assemble(".program t\n.origin 8\nnop\n.wrap_target\nnop\nnop\n.wrap\nnop\nstart: nop\n.entry start\n")[0]
        self.assertEqual((p.origin, p.wrap_bottom_abs, p.wrap_top_abs, p.entry), (8, 9, 10, 12))
        p = pioasm.assemble(".program t\nnop\nnop\nnop\n")[0]
        self.assertEqual((p.origin, p.wrap_bottom_abs, p.wrap_top_abs, p.entry), (0, 0, 2, 0))
        p = pioasm.assemble(".program t\n.origin 3\nnop\n.wrap_target\nnop\nnop\n")[0]
        self.assertEqual((p.wrap_bottom_abs, p.wrap_top_abs), (4, 5))
        p = pioasm.assemble(".program t\n.origin 3\nnop\nnop\n.wrap\nnop\n")[0]
        self.assertEqual((p.wrap_bottom_abs, p.wrap_top_abs), (3, 4))

    def test_multi_program(self):
        ps = pioasm.assemble(".define G 1\n.program a\nset x, G\n.program b\n.side_set 2 opt\nnop side 1\n")
        self.assertEqual([p.name for p in ps], ["a", "b"])
        self.assertEqual(ps[1].side.count, 2)
        self.assertTrue(ps[1].side.opt)
        self.assertEqual(ps[0].side.count, 0)

    def test_comments(self):
        w = asm_words(".program t ; c\n// full\nnop ; x\nnop // y\n")
        self.assertEqual(len(w), 2)


class TestErrors(unittest.TestCase):
    def assertErr(self, src, line, frag):
        es = err(src)
        self.assertTrue(any(l == line and frag in m for l, m in es),
                        "wanted line %d containing %r, got %r" % (line, frag, es))

    def test_basic(self):
        self.assertErr(".program t\nnop\nbogus x\n", 3, "unknown mnemonic")
        self.assertErr("nop\n", 1, "outside .program")
        self.assertErr(".program t\njmp nowhere\n", 2, "undefined symbol 'nowhere'")
        self.assertErr(".program t\nl: nop\nl: nop\n", 3, "duplicate label")
        self.assertErr(".program t\n.program t\nnop\n", 2, "duplicate program")
        self.assertErr(".program t\n", 1, "no instructions")
        self.assertErr("", 1, "no .program")
        self.assertErr(".program t\n.bogus\nnop\n", 2, "unknown directive")
        self.assertErr(".program t\nset x, 1 $\n", 2, "unexpected character")
        self.assertErr(".program t\nset x 1\n", 2, "expected ','")
        self.assertErr(".program t\nset x, 1 2\n", 2, "unexpected")
        self.assertErr(".program t\nlab: nop\n.define lab 1\n", 3, "clashes")
        self.assertErr(".program t\n.define x 1\n", 2, "reserved")
        self.assertErr(".program t\nsame: nop\n.define a b\n.define b a\nset x, a\n", 5, "circular")

    def test_ranges(self):
        self.assertErr(".program t\nnop [16]\n", 2, "delay 16 out of range 0..15")
        self.assertErr(".program t\nnop [-1]\n", 2, "delay -1")
        self.assertErr(".program t\nnop [1] [2]\n", 2, "duplicate delay")
        self.assertErr(".program t\nnop [1\n", 2, "missing ']'")
        self.assertErr(".program t\nset x, 256\n", 2, "set value 256")
        self.assertErr(".program t\nset x, -1\n", 2, "out of range")
        self.assertErr(".program t\nset q, 1\n", 2, "destination")
        self.assertErr(".program t\njmp 256\n", 2, "jump target")
        self.assertErr(".program t\nin x, 0\n", 2, "bit count")
        self.assertErr(".program t\nin x, 33\n", 2, "bit count")
        self.assertErr(".program t\nin null2, 3\n", 2, "source")
        self.assertErr(".program t\nout status, 3\n", 2, "destination")
        self.assertErr(".program t\nmov status, x\n", 2, "destination")
        self.assertErr(".program t\nmov x, exec\n", 2, "source")
        self.assertErr(".program t\nwait 2 gpio 1\n", 2, "polarity")
        self.assertErr(".program t\nwait 1 gpio 32\n", 2, "index")
        self.assertErr(".program t\nwait 1 irq 8\n", 2, "irq flag")
        self.assertErr(".program t\nwait 1 gpio 3 rel\n", 2, "irq")
        self.assertErr(".program t\nwait 1 foo 3\n", 2, "wait source")
        self.assertErr(".program t\nirq set 8\n", 2, "irq flag")
        self.assertErr(".program t\nirq wiggle 1\n", 2, "irq mode")
        self.assertErr(".program t\nadd x, 128\n", 2, "immediate")
        self.assertErr(".program t\nshl x, 32\n", 2, "shift amount")
        self.assertErr(".program t\nadd x, -1\n", 2, "immediate")
        self.assertErr(".program t\nadd isr, 1\n", 2, "destination")
        self.assertErr(".program t\nrsub x, y\n", 2, "immediate form")
        self.assertErr(".program t\nsltu x, 3\n", 2, "register forms")
        self.assertErr(".program t\nadd x, 1, 2\n", 2, "immediate form")
        self.assertErr(".program t\nloop 256\nnop\nendloop\n", 2, "loop count")
        self.assertErr(".program t\njmp 1/0\n", 2, "division by zero")
        self.assertErr(".program t\n.word 0x1000000\n", 2, "24-bit")
        self.assertErr(".program t\npush iffull iffull\n", 2, "duplicate")
        self.assertErr(".program t\npush ifempty\n", 2, "unexpected")
        self.assertErr(".program t\npull iffull\n", 2, "unexpected")
        self.assertErr(".program t\npush block noblock\n", 2, "duplicate")

    def test_side(self):
        self.assertErr(".program t\nnop side 1\n", 2, "no .side_set")
        self.assertErr(".program t\n.side_set 2\nnop side 4\n", 3, "side value 4")
        self.assertErr(".program t\n.side_set 2\nnop side -1\n", 3, "side value -1")
        self.assertErr(".program t\n.side_set 3 opt\nnop side 8\n", 3, "side value 8")
        self.assertErr(".program t\n.side_set 5\nnop\n", 2, "out of range 0..4")
        self.assertErr(".program t\n.side_set 4 opt\nnop\n", 2, "1..3")
        self.assertErr(".program t\n.side_set 0 opt\nnop\n", 2, "1..3")
        self.assertErr(".program t\n.side_set 1 bogus\nnop\n", 2, "bad .side_set option")
        self.assertErr(".program t\nnop\n.side_set 1\n", 3, "before the first instruction")
        self.assertErr(".program t\n.side_set 1\nnop side 1 side 0\n", 3, "duplicate 'side'")

    def test_directives(self):
        self.assertErr(".program t\nnop\n.origin 1\n", 3, "before the first")
        self.assertErr(".program t\n.origin 256\nnop\n", 2, "out of range")
        self.assertErr(".program t\n.origin 255\nnop\nnop\n", 1, "does not fit")
        self.assertErr(".program t\n.wrap\nnop\n", 2, "preceding instruction")
        self.assertErr(".program t\nnop\n.wrap_target\n", 1, "after the last instruction")
        self.assertErr(".program t\nnop\nnop\n.wrap\n.wrap_target\nnop\n", 1, "after .wrap")
        self.assertErr(".program t\n.wrap_target\n.wrap_target\nnop\n", 3, "duplicate")
        self.assertErr(".program t\nnop\n.entry 5\n", 3, ".entry")
        self.assertErr(".program t\nnop\n.entry nope\n", 3, "undefined symbol")
        self.assertErr(".program t\n.entry 0\n.entry 0\nnop\n", 3, "duplicate")

    def test_loop_structure(self):
        self.assertErr(".program t\nloop 2\nnop\n", 2, "without matching 'endloop'")
        self.assertErr(".program t\nnop\nendloop\n", 3, "without matching 'loop'")
        self.assertErr(".program t\nloop 2\nendloop\nnop\n", 3, "empty loop body")
        self.assertErr(".program t\nloop 1\nloop 2\nloop 3\nnop\nendloop\nendloop\nendloop\n", 4, "nesting")
        self.assertErr(".program t\nloop 1\n" + "nop\n" * 17 + "endloop\nnop\n", 20, "max 16")
        self.assertErr(".program t\nloop 1\nloop 2\nnop\nendloop\nendloop\nnop\n", 6, "same instruction")
        self.assertErr(".program t\nloop 1\nnop\nendloop 3\n", 4, "no operands")
        # wrap interaction
        self.assertErr(".program t\nloop 2\nnop\n.wrap\nnop\nendloop\nnop\n", 2, "wrap")  # wrap_top inside body
        self.assertErr(".program t\nnop\nloop 2\nnop\nendloop\nnop\n.wrap_target\n", 1, "after the last")
        self.assertErr(".program t\nloop 2, 3\nnop\n", 2, "past the last instruction")

    def test_loop_wrap_ok(self):
        # wrap on the last body instruction is legal
        pioasm.assemble(".program t\nloop 2\nnop\nnop\n.wrap\nendloop\nnop\n.wrap_target\n" if False else
                        ".program t\nloop 2\nnop\nnop\nendloop\n.wrap\nnop\n")
        pioasm.assemble(".program t\n.wrap_target\nnop\nloop 2\nnop\nnop\nendloop\nnop\n.wrap\n")

    def test_multiple_errors_reported(self):
        es = err(".program t\nfoo\nset x, 999\nnop [20]\n")
        self.assertEqual([l for l, _ in es], [2, 3, 4])

    def test_imem_overflow(self):
        self.assertErr(".program t\n" + "nop\n" * 257, 258, "exceeds instruction memory")


SAMPLE = """\
.define BIT_CYCLES 8
.program uart_tx
.side_set 1 opt
.origin 2
.wrap_target
start:
    pull block side 0 [7]
    set x, 7 side 1 [BIT_CYCLES-1]
bitloop:
    out pins, 1
    djnz x, bitloop
    loop 3
      nop
      loop y
        irq set 3 next
      endloop
      add x, 1
    endloop
    mov y, ~x
    mov isr, rev(x)
    sltu z, x, y
    add x, y
    wait 1 irq 4 rel
    jmp xnz, start
    mov pc, swap(osr)
.wrap
    halt
.entry bitloop
"""


class TestRoundTrip(unittest.TestCase):
    def test_sample(self):
        p = pioasm.assemble(SAMPLE)[0]
        hx = pioasm.format_hex(p, meta=True)
        origin, words, meta = pioasm.parse_hex(hx)
        self.assertEqual((origin, words), (p.origin, p.words))
        side = pioasm._parse_side_arg(meta["side_set"])
        text = pioasm.disassemble(words, origin, side, "uart_tx",
                                  int(meta["wrap_target"]), int(meta["wrap"]),
                                  int(meta["entry"]))
        q = pioasm.assemble(text)[0]
        self.assertEqual(q.words, p.words)
        self.assertEqual((q.origin, q.wrap_bottom_abs, q.wrap_top_abs, q.entry, q.side.count,
                          q.side.opt),
                         (p.origin, p.wrap_bottom_abs, p.wrap_top_abs, p.entry, 1, True))
        # idempotent text
        self.assertEqual(pioasm.disassemble(q.words, q.origin, q.side, "uart_tx",
                                            q.wrap_bottom_abs, q.wrap_top_abs, q.entry), text)

    def _rt(self, words, side, origin=0):
        text = pioasm.disassemble(words, origin, side)
        q = pioasm._assemble(text, "<rt>", 256, None, False)[0]
        self.assertEqual(q.words, words, text)

    def test_random_words_all_configs(self):
        rnd = random.Random(1234)
        for side in (pioasm.SideCfg(0), pioasm.SideCfg(1), pioasm.SideCfg(3, True),
                     pioasm.SideCfg(2, False, True), pioasm.SideCfg(4), pioasm.SideCfg(1, True)):
            words = []
            for _ in range(3000):
                op = rnd.randrange(16) if rnd.random() < 0.3 else rnd.randrange(13)
                words.append((rnd.randrange(16) << 20) | (rnd.randrange(16) << 16) |
                             (op << 12) | rnd.randrange(4096))
            for i in range(0, len(words), 200):
                self._rt(words[i:i + 200], side, origin=rnd.randrange(0, 50))

    def test_canonical_words_are_not_dot_word(self):
        # every word produced by the assembler must disassemble to real text
        p = pioasm.assemble(SAMPLE)[0]
        for i, w in enumerate(p.words):
            self.assertNotIn(".word", pioasm.disassemble_word(w, p.origin + i, p.side))

    def test_reserved_become_dot_word(self):
        side = pioasm.SideCfg(0)
        for w in (0x00D000, 0x00F123, 0x002900, 0x008500, 0x006C00, 0x00A0C0, 0x00C001,
                  0x010000):
            self.assertTrue(pioasm.disassemble_word(w, 0, side).startswith(".word"), hex(w))

    def test_infer_side(self):
        s = pioasm.infer_side([W(0, 5, 12), W(0, 2, 12)])
        self.assertEqual(s.count, 3)
        self.assertEqual(pioasm.infer_side([W(op=12)]).count, 0)


class TestCLI(unittest.TestCase):
    def run_cli(self, args):
        o, e = io.StringIO(), io.StringIO()
        with redirect_stdout(o), redirect_stderr(e):
            rc = pioasm.main(args)
        return rc, o.getvalue(), e.getvalue()

    def test_outputs(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "a.pio")
            _w(src, SAMPLE)
            rc, _, e = self.run_cli([src, "--hex", d + "/x.hex", "--svh", d + "/x.svh",
                                     "--c", d + "/x.h", "--lst", d + "/x.lst"])
            self.assertEqual(rc, 0, e)
            hx = _r(d + "/x.hex").split()
            self.assertEqual(hx[0], "@2")
            self.assertTrue(all(len(x) == 6 for x in hx[1:]))
            svh = _r(d + "/x.svh")
            for name in ("WRAP_BOTTOM", "WRAP_TOP", "ENTRY", "ORIGIN", "SIDE_COUNT",
                         "SIDE_OPT", "SIDE_PINDIR", "LENGTH", "INSTR"):
                self.assertIn("UART_TX_" + name, svh)
            self.assertIn("UART_TX_SIDE_COUNT  = 1;", svh)
            ch = _r(d + "/x.h")
            for name in ("WRAP_BOTTOM", "WRAP_TOP", "ENTRY", "ORIGIN", "SIDE_COUNT",
                         "SIDE_OPT", "SIDE_PINDIR", "LENGTH"):
                self.assertIn("#define UART_TX_" + name, ch)
            self.assertIn("uint32_t uart_tx_program[UART_TX_LENGTH]", ch)
            self.assertIn("UART_TX_ENTRY        4", ch)
            self.assertIn("addr", _r(d + "/x.lst"))

    def test_origin_zero_no_at_line(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "a.pio")
            _w(src, ".program t\nnop\nhalt\n")
            self.run_cli([src, "--hex", d + "/x.hex"])
            self.assertEqual(_r(d + "/x.hex"), "00C000\n000001\n")

    def test_multi_program_suffix(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "a.pio")
            _w(src, ".program p1\nnop\n.program p2\nhalt\n")
            rc, _, e = self.run_cli([src, "--hex", d + "/o.hex", "--svh", d + "/o.svh"])
            self.assertEqual(rc, 0, e)
            self.assertEqual(sorted(os.listdir(d)),
                             ["a.pio", "o_p1.hex", "o_p1.svh", "o_p2.hex", "o_p2.svh"])
            self.assertIn("P2_ENTRY", _r(d + "/o_p2.svh"))

    def test_error_exit(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "bad.pio")
            _w(src, ".program t\nnop\nfrob\n")
            rc, _, e = self.run_cli([src, "--hex", d + "/x.hex"])
            self.assertEqual(rc, 1)
            self.assertIn("bad.pio:3: error: unknown mnemonic 'frob'", e)
            self.assertFalse(os.path.exists(d + "/x.hex"))

    def test_disasm_cli_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "a.pio")
            _w(src, SAMPLE)
            self.run_cli([src, "--hex", d + "/x.hex", "--meta"])
            rc, out, e = self.run_cli(["--disasm", d + "/x.hex"])
            self.assertEqual(rc, 0, e)
            _w(d + "/b.pio", out)
            self.run_cli([d + "/b.pio", "--hex", d + "/y.hex", "--meta"])
            a = _r(d + "/x.hex").splitlines()
            b = _r(d + "/y.hex").splitlines()
            self.assertEqual([x for x in a if not x.startswith("//")],
                             [x for x in b if not x.startswith("//")])

    def test_define_flag(self):
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "a.pio")
            _w(src, ".program t\nset x, N\n")
            self.assertEqual(self.run_cli([src, "-D", "N=5", "--hex", d + "/x.hex"])[0], 0)
            self.assertEqual(_r(d + "/x.hex"), "008105\n")

    def test_subprocess_from_root(self):
        import subprocess
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with tempfile.TemporaryDirectory() as d:
            src = os.path.join(d, "a.pio")
            _w(src, SAMPLE)
            r = subprocess.run([sys.executable, "tools/pioasm.py", src, "--hex", d + "/x.hex",
                                "--svh", d + "/x.svh"], cwd=root, capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            self.assertTrue(os.path.exists(d + "/x.svh"))


class TestWarnings(unittest.TestCase):
    def test_loopr_on_wrap(self):
        # loop <reg> whose last body instruction is the (default) wrap top
        p = pioasm.assemble(".program t\nloop x\nnop\nnop\nendloop\n")[0]
        self.assertEqual(len(p.warnings), 1)
        # constant-count loop: no warning
        p = pioasm.assemble(".program t\nloop 3\nnop\nnop\nendloop\n")[0]
        self.assertEqual(p.warnings, [])
        # an instruction after the loop: wrap top is not the loop end
        p = pioasm.assemble(".program t\nloop x\nnop\nnop\nendloop\nnop\n")[0]
        self.assertEqual(p.warnings, [])


if __name__ == "__main__":
    unittest.main()
