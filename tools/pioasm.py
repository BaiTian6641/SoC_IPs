#!/usr/bin/env python3
"""pioasm.py - assembler / disassembler for the PIO engine ("pioe") ISA v1.

Normative encoding: ips/23_pio_engine/doc/ISA.md (24-bit words,
DLY[23:20] SS[19:16] OP[15:12] ARG[11:0]).  Manual: doc/ASSEMBLER.md.
Pure Python 3 standard library.

    python3 tools/pioasm.py prog.pio --hex prog.hex --svh prog.svh --c prog.h --lst prog.lst
    python3 tools/pioasm.py --disasm prog.hex [--side-set "1 opt"] [-o prog.pio]
"""
import argparse
import os
import re
import sys

__all__ = ["assemble", "disassemble", "parse_hex", "format_hex", "Program",
           "AsmError", "AsmFailure", "SideCfg"]

# --------------------------------------------------------------------------
# ISA tables (ISA.md section 1.2).  Index in list == field value.
# --------------------------------------------------------------------------
OP_JMP, OP_WAIT, OP_IN, OP_OUT, OP_PP, OP_MOV, OP_ALUR, OP_ALUI = range(8)
OP_SET, OP_LOOP, OP_LOOPR, OP_IRQ, OP_NOP = 8, 9, 10, 11, 12

CONDS = ["always", "xz", "xnz", "yz", "ynz", "xdec", "ydec", "xeqy", "xney",
         "xltuy", "pin", "npin", "osre", "osrne", "isrf", "rxf"]
WAIT_SRC = ["gpio", "pin", "irq", "xbit"]
REL = {"rel": 1, "next": 2, "prev": 3}
REL_NAMES = {1: "rel", 2: "next", 3: "prev"}
IN_SRC = ["pins", "x", "y", "z", "null", "status", "isr", "osr"]
OUT_DST = ["pins", "x", "y", "z", "null", "pindirs", "pc", "isr", "exec"]
MOV_DST = ["pins", "x", "y", "z", "pindirs", "pc", "isr", "osr", "exec"]
MOV_SRC = IN_SRC
SET_DST = ["pins", "x", "y", "z", "pindirs"]
ALU_REG = ["x", "y", "z", "isr", "osr", "pins", "status", "null"]
ALU_DST = ["x", "y", "z"]
ALU_OPS = ["add", "sub", "and", "or", "xor", "shl", "shr"]   # 0..6; 7 = rsub/sltu
IRQ_MODE = ["set", "clear", "setwait", "toggle"]
LOOP_REG = ["x", "y", "z"]
MOP_NAMES = ["", "~", "rev", "swap"]

MNEMONICS = {"jmp", "djnz", "halt", "nop", "wait", "in", "out", "push", "pull",
             "mov", "add", "sub", "and", "or", "xor", "shl", "shr", "rsub",
             "sltu", "set", "loop", "endloop", "irq"}
# names that may never be used for labels / defines
RESERVED = (MNEMONICS | set(IN_SRC) | set(OUT_DST) | set(MOV_DST) |
            {"side", "rev", "swap", "pc", "exec", "pindirs"})

WORD_MASK = 0xFFFFFF
MAX_LOOP_DEPTH = 2
MAX_LOOP_BODY = 16


class AsmError(Exception):
    def __init__(self, msg, line=None):
        Exception.__init__(self, msg)
        self.msg = msg
        self.line = line


class AsmFailure(Exception):
    """Raised by assemble(); .errors is a list of (filename, line, msg)."""

    def __init__(self, errors):
        self.errors = errors
        Exception.__init__(self, "\n".join(
            "%s:%s: error: %s" % (f, l if l else "?", m) for f, l, m in errors))


class SideCfg(object):
    def __init__(self, count=0, opt=False, pindir=False):
        self.count, self.opt, self.pindir = count, opt, pindir

    def __repr__(self):
        return "SideCfg(%d,%s,%s)" % (self.count, self.opt, self.pindir)

    def directive(self):
        if not self.count:
            return ""
        return ".side_set %d%s%s" % (self.count, " opt" if self.opt else "",
                                     " pindirs" if self.pindir else "")


# --------------------------------------------------------------------------
# Tokenizer
# --------------------------------------------------------------------------
_TOK_RE = re.compile(
    r"\s*(?:(?P<num>0[xX][0-9a-fA-F_]+|0[bB][01_]+|\d[\d_]*)"
    r"|(?P<id>[A-Za-z_][A-Za-z0-9_]*)"
    r"|(?P<op><<|>>|[-+*/&|^~()\[\],:.!]))")


class Tok(object):
    __slots__ = ("kind", "text", "col")

    def __init__(self, kind, text, col):
        self.kind, self.text, self.col = kind, text, col

    def __repr__(self):
        return "Tok(%s,%r)" % (self.kind, self.text)


def strip_comment(s):
    cut = len(s)
    for m in (";", "//"):
        i = s.find(m)
        if i >= 0 and i < cut:
            cut = i
    return s[:cut]


def tokenize(s):
    toks, pos = [], 0
    while True:
        m = _TOK_RE.match(s, pos)
        if not m:
            if s[pos:].strip() == "":
                return toks
            bad = s[pos:].lstrip()[0]
            raise AsmError("unexpected character %r" % bad)
        kind = "num" if m.group("num") else ("id" if m.group("id") else "op")
        text = m.group(kind)
        toks.append(Tok(kind, text, m.start(kind)))
        pos = m.end()


def _is_op(t, text):
    return t is not None and t.kind == "op" and t.text == text


# --------------------------------------------------------------------------
# Expressions
# --------------------------------------------------------------------------
class Env(object):
    """Symbol environment for expression evaluation."""

    def __init__(self, gdefines=None, defines=None, labels=None, dot=None):
        self.gdefines = gdefines or {}
        self.defines = defines or {}
        self.labels = labels or {}
        self.dot = dot
        self.cache = {}
        self.busy = set()

    def lookup(self, name):
        if name in self.defines:
            d = self.defines
        elif name in self.gdefines:
            d = self.gdefines
        elif name in self.labels:
            return self.labels[name]
        else:
            hint = ""
            if name.lower() in RESERVED:
                hint = " (register/keyword names are not values)"
            raise AsmError("undefined symbol '%s'%s" % (name, hint))
        key = (id(d), name)
        if key in self.cache:
            return self.cache[key]
        if key in self.busy:
            raise AsmError("circular .define involving '%s'" % name)
        self.busy.add(key)
        try:
            toks, line = d[name]
            sub = Env(self.gdefines, self.defines, self.labels, None)
            sub.cache, sub.busy = self.cache, self.busy
            try:
                v, p = parse_expr(toks, 0, sub)
                if p != len(toks):
                    raise AsmError("junk after expression in .define %s" % name)
            except AsmError as e:
                if e.line is None:
                    e.msg = "in .define %s (line %s): %s" % (name, line, e.msg)
                    e.args = (e.msg,)
                raise
        finally:
            self.busy.discard(key)
        self.cache[key] = v
        return v


def _tdiv(a, b):
    q = abs(a) // abs(b)
    return q if (a < 0) == (b < 0) else -q


class _Expr(object):
    def __init__(self, toks, pos, env):
        self.t, self.p, self.env = toks, pos, env

    def peek(self):
        return self.t[self.p] if self.p < len(self.t) else None

    def _binop(self, sub, ops):
        v = sub()
        while True:
            t = self.peek()
            if t is not None and t.kind == "op" and t.text in ops:
                self.p += 1
                r = sub()
                v = self._apply(t.text, v, r)
            else:
                return v

    @staticmethod
    def _apply(op, a, b):
        if op == "|":
            return a | b
        if op == "^":
            return a ^ b
        if op == "&":
            return a & b
        if op == "+":
            return a + b
        if op == "-":
            return a - b
        if op == "*":
            return a * b
        if op in ("<<", ">>"):
            if b < 0 or b > 1024:
                raise AsmError("bad shift amount %d" % b)
            return a << b if op == "<<" else a >> b
        if op == "/":
            if b == 0:
                raise AsmError("division by zero in expression")
            return _tdiv(a, b)
        raise AsmError("bad operator " + op)

    def or_(self):
        return self._binop(self.xor_, ("|",))

    def xor_(self):
        return self._binop(self.and_, ("^",))

    def and_(self):
        return self._binop(self.shift, ("&",))

    def shift(self):
        return self._binop(self.add, ("<<", ">>"))

    def add(self):
        return self._binop(self.mul, ("+", "-"))

    def mul(self):
        return self._binop(self.unary, ("*", "/"))

    def unary(self):
        t = self.peek()
        if t is not None and t.kind == "op" and t.text in ("-", "+", "~"):
            self.p += 1
            v = self.unary()
            return -v if t.text == "-" else (v if t.text == "+" else ~v)
        return self.primary()

    def primary(self):
        t = self.peek()
        if t is None:
            raise AsmError("expression expected, got end of line")
        self.p += 1
        if t.kind == "num":
            return int(t.text.replace("_", ""), 0) if not re.match(
                r"^0\d", t.text) else int(t.text.replace("_", ""), 10)
        if t.kind == "id":
            return self.env.lookup(t.text)
        if t.text == "(":
            v = self.or_()
            c = self.peek()
            if not _is_op(c, ")"):
                raise AsmError("missing ')' in expression")
            self.p += 1
            return v
        if t.text == ".":
            if self.env.dot is None:
                raise AsmError("'.' (current address) is not allowed here")
            return self.env.dot
        raise AsmError("expression expected, got '%s'" % t.text)


def parse_expr(toks, pos, env):
    e = _Expr(toks, pos, env)
    v = e.or_()
    return v, e.p


# --------------------------------------------------------------------------
# Program model
# --------------------------------------------------------------------------
class Item(object):
    __slots__ = ("kind", "line", "toks", "index", "loop_len", "word", "word_len")

    def __init__(self, kind, line, toks, index):
        self.kind, self.line, self.toks, self.index = kind, line, toks, index
        self.loop_len = None   # structured loop: computed LEN
        self.word = None
        self.word_len = 0


class Program(object):
    def __init__(self, name, line):
        self.name = name
        self.line = line
        self.origin = 0
        self.side = SideCfg()
        self.items = []
        self.labels = {}        # name -> relative index
        self.defines = {}       # name -> (toks, line)
        self.wrap_target = None  # relative index
        self.wrap_top = None
        self.entry_toks = None
        self.entry_line = None
        self.src_lines = []     # (lineno, text) for listing
        self.endloop_lines = []
        self.words = []
        self.wrap_bottom_abs = 0
        self.wrap_top_abs = 0
        self.entry = 0
        self.warnings = []
        self.finished = False

    # convenience used by emitters
    @property
    def length(self):
        return len(self.words)

    @property
    def side_count(self):
        return self.side.count

    @property
    def side_opt(self):
        return self.side.opt

    @property
    def side_pindir(self):
        return self.side.pindir


# --------------------------------------------------------------------------
# Pass 1: structure (labels, directives, loops)
# --------------------------------------------------------------------------
def _split_labels(toks):
    labels = []
    while (len(toks) >= 2 and toks[0].kind == "id" and _is_op(toks[1], ":")):
        labels.append(toks[0].text)
        toks = toks[2:]
    return labels, toks


def _is_directive(toks):
    return (len(toks) >= 2 and _is_op(toks[0], ".") and toks[1].kind == "id"
            and toks[1].col == toks[0].col + 1)


def _check_name(name, what):
    if name.lower() in RESERVED:
        raise AsmError("'%s' is a reserved word and cannot be a %s" % (name, what))


class _Parser1(object):
    def __init__(self, text, imem_depth, cli_defines):
        self.text = text
        self.imem_depth = imem_depth
        self.gdefines = dict(cli_defines)
        self.programs = []
        self.cur = None
        self.errors = []
        self.loop_stack = []
        self.last_endloop = None

    def err(self, line, msg):
        self.errors.append((line, msg))

    def run(self):
        for n, raw in enumerate(self.text.splitlines(), 1):
            try:
                self.line(n, raw)
            except AsmError as e:
                self.err(n, e.msg)
        self.finish_program(len(self.text.splitlines()))
        return self.programs

    def finish_program(self, line):
        p = self.cur
        if p is None:
            return
        for it in self.loop_stack:
            self.err(it.line, "'loop' without matching 'endloop'")
        self.loop_stack = []
        self.last_endloop = None
        p.finished = True
        self.cur = None

    def env_pass1(self):
        p = self.cur
        return Env(self.gdefines, p.defines if p else {}, {}, None)

    def line(self, n, raw):
        toks = tokenize(strip_comment(raw))
        p = self.cur
        if not toks:
            if p is not None:
                p.src_lines.append((n, raw))
            return
        labels, toks = _split_labels(toks)
        if p is not None:
            p.src_lines.append((n, raw))
        if labels:
            if p is None:
                raise AsmError("label '%s' outside .program" % labels[0])
            for lb in labels:
                _check_name(lb, "label")
                if lb in p.labels:
                    raise AsmError("duplicate label '%s'" % lb)
                if lb in p.defines or lb in self.gdefines:
                    raise AsmError("label '%s' clashes with a .define" % lb)
                p.labels[lb] = len(p.items)
        if not toks:
            return
        if _is_directive(toks):
            self.directive(n, raw, toks[1].text.lower(), toks[2:])
            return
        if toks[0].kind != "id":
            raise AsmError("expected instruction, got '%s'" % toks[0].text)
        if p is None:
            raise AsmError("instruction outside .program (missing .program?)")
        mn = toks[0].text.lower()
        if mn not in MNEMONICS:
            raise AsmError("unknown mnemonic '%s'" % toks[0].text)
        if mn == "endloop":
            self.endloop(n, toks)
            return
        item = Item("instr", n, toks, len(p.items))
        if mn == "loop":
            if not any(_is_op(t, ",") for t in toks):
                if len(self.loop_stack) >= MAX_LOOP_DEPTH:
                    raise AsmError("loop nesting deeper than %d" % MAX_LOOP_DEPTH)
                item.loop_len = -1   # fixed up by endloop
                self.loop_stack.append(item)
        p.items.append(item)
        self.check_depth(n)

    def check_depth(self, n):
        if len(self.cur.items) > self.imem_depth:
            raise AsmError("program exceeds instruction memory (%d words)" %
                           self.imem_depth)

    def endloop(self, n, toks):
        p = self.cur
        if len(toks) > 1:
            raise AsmError("'endloop' takes no operands")
        if not self.loop_stack:
            raise AsmError("'endloop' without matching 'loop'")
        it = self.loop_stack.pop()
        body = len(p.items) - it.index - 1
        if body < 1:
            raise AsmError("empty loop body")
        if body > MAX_LOOP_BODY:
            raise AsmError("loop body has %d instructions (max %d)" %
                           (body, MAX_LOOP_BODY))
        if self.last_endloop == len(p.items):
            raise AsmError("nested loops must not end on the same instruction "
                           "(add an instruction before the outer 'endloop')")
        self.last_endloop = len(p.items)
        it.loop_len = body - 1
        p.endloop_lines.append(n)

    # -- directives ---------------------------------------------------------
    def need_prog(self, d):
        if self.cur is None:
            raise AsmError(".%s outside .program" % d)
        return self.cur

    def directive(self, n, raw, d, a):
        if d == "program":
            if len(a) != 1 or a[0].kind != "id":
                raise AsmError(".program expects a name")
            name = a[0].text
            self.finish_program(n)
            if any(q.name == name for q in self.programs):
                raise AsmError("duplicate program name '%s'" % name)
            self.cur = Program(name, n)
            self.cur.src_lines.append((n, raw))
            self.programs.append(self.cur)
            return
        if d == "define":
            if not a or a[0].kind != "id":
                raise AsmError(".define expects NAME EXPR")
            name = a[0].text
            _check_name(name, "define")
            if len(a) < 2:
                raise AsmError(".define %s needs a value" % name)
            scope = self.cur.defines if self.cur else self.gdefines
            if name in scope:
                raise AsmError("duplicate .define '%s'" % name)
            if self.cur is not None and name in self.cur.labels:
                raise AsmError(".define '%s' clashes with a label" % name)
            scope[name] = (a[1:], n)
            return
        p = self.need_prog(d)
        if d == "origin":
            if p.items:
                raise AsmError(".origin must come before the first instruction")
            v, pos = parse_expr(a, 0, self.env_pass1())
            self.expect_end(a, pos, ".origin")
            if v < 0 or v >= self.imem_depth:
                raise AsmError(".origin %d out of range 0..%d" %
                               (v, self.imem_depth - 1))
            p.origin = v
        elif d == "side_set":
            if p.items:
                raise AsmError(".side_set must come before the first instruction")
            if not a:
                raise AsmError(".side_set expects a bit count")
            v, pos = parse_expr(a, 0, self.env_pass1())
            opt = pind = False
            for t in a[pos:]:
                w = t.text.lower() if t.kind == "id" else None
                if w == "opt" and not opt:
                    opt = True
                elif w in ("pindirs", "pindir") and not pind:
                    pind = True
                else:
                    raise AsmError("bad .side_set option '%s' (use opt, pindirs)"
                                   % t.text)
            if v < 0 or v > 4:
                raise AsmError(".side_set count %d out of range 0..4" % v)
            if opt and not 1 <= v <= 3:
                raise AsmError(".side_set ... opt needs a bit count of 1..3 "
                               "(SS[3] is the enable bit)")
            p.side = SideCfg(v, opt, pind)
        elif d == "wrap_target":
            self.expect_end(a, 0, ".wrap_target")
            if p.wrap_target is not None:
                raise AsmError("duplicate .wrap_target")
            p.wrap_target = len(p.items)
        elif d == "wrap":
            self.expect_end(a, 0, ".wrap")
            if p.wrap_top is not None:
                raise AsmError("duplicate .wrap")
            if not p.items:
                raise AsmError(".wrap needs a preceding instruction")
            p.wrap_top = len(p.items) - 1
        elif d == "entry":
            if not a:
                raise AsmError(".entry expects a label or address")
            if p.entry_toks is not None:
                raise AsmError("duplicate .entry")
            p.entry_toks, p.entry_line = a, n
        elif d == "word":
            if not a:
                raise AsmError(".word expects a value")
            p.items.append(Item("word", n, a, len(p.items)))
            self.check_depth(n)
        else:
            raise AsmError("unknown directive '.%s'" % d)

    @staticmethod
    def expect_end(a, pos, name):
        if pos < len(a):
            raise AsmError("unexpected '%s' after %s" % (a[pos].text, name))


# --------------------------------------------------------------------------
# Pass 2: encoding
# --------------------------------------------------------------------------
class _Ops(object):
    """Operand cursor over the token list of one instruction."""

    def __init__(self, toks, env):
        self.t, self.p, self.env = toks, 0, env

    def peek(self, k=0):
        i = self.p + k
        return self.t[i] if i < len(self.t) else None

    def end(self):
        return self.p >= len(self.t)

    def word(self, k=0):
        t = self.peek(k)
        return t.text.lower() if t is not None and t.kind == "id" else None

    def take_id(self, table, what):
        w = self.word()
        if w is None or w not in table:
            t = self.peek()
            got = "end of line" if t is None else "'%s'" % t.text
            raise AsmError("expected %s (%s), got %s" %
                           (what, ", ".join(table), got))
        self.p += 1
        return list(table).index(w)

    def comma(self):
        if not _is_op(self.peek(), ","):
            t = self.peek()
            got = "end of line" if t is None else "'%s'" % t.text
            raise AsmError("expected ',', got %s" % got)
        self.p += 1

    def expr(self):
        v, self.p = parse_expr(self.t, self.p, self.env)
        return v

    def rng(self, what, lo, hi):
        v = self.expr()
        if v < lo or v > hi:
            raise AsmError("%s %d out of range %d..%d" % (what, v, lo, hi))
        return v

    def opt_rel(self):
        w = self.word()
        if w in REL:
            self.p += 1
            return REL[w]
        return 0

    def mods(self, side):
        """Parse trailing [delay] and 'side v'. Returns (delay, ss)."""
        dly = None
        sv = None
        while not self.end():
            t = self.peek()
            if _is_op(t, "["):
                if dly is not None:
                    raise AsmError("duplicate delay")
                self.p += 1
                dly = self.expr()
                if not _is_op(self.peek(), "]"):
                    raise AsmError("missing ']' after delay")
                self.p += 1
                if dly < 0 or dly > 15:
                    raise AsmError("delay %d out of range 0..15" % dly)
            elif self.word() == "side":
                if sv is not None:
                    raise AsmError("duplicate 'side'")
                self.p += 1
                sv = self.expr()
            else:
                raise AsmError("unexpected '%s'" % t.text)
        ss = 0
        if sv is not None:
            if side.count == 0:
                raise AsmError("'side' used but program has no .side_set")
            if sv < 0 or sv >= (1 << side.count):
                raise AsmError("side value %d out of range 0..%d (.side_set %d)"
                               % (sv, (1 << side.count) - 1, side.count))
            ss = sv | (8 if side.opt else 0)
        return (dly or 0), ss

    def finish(self):
        if not self.end():
            raise AsmError("unexpected '%s'" % self.peek().text)


def _arg(op, arg, dly, ss):
    return (dly << 20) | (ss << 16) | (op << 12) | arg


def _enc_instr(item, prog, env, strict):
    toks = item.toks
    mn = toks[0].text.lower()
    o = _Ops(toks[1:], env)
    side = prog.side
    dot = prog.origin + item.index

    def done(op, arg):
        d, ss = o.mods(side)
        return _arg(op, arg, d, ss)

    if mn == "jmp":
        cond = 0
        w = o.word()
        if w in CONDS and _is_op(o.peek(1), ","):
            cond = CONDS.index(w)
            o.p += 2
        tgt = o.rng("jump target", 0, 255)
        return done(OP_JMP, (cond << 8) | tgt)
    if mn == "djnz":
        r = o.take_id(["x", "y"], "register x or y")
        o.comma()
        tgt = o.rng("jump target", 0, 255)
        return done(OP_JMP, ((5 + r) << 8) | tgt)
    if mn == "halt":
        return done(OP_JMP, dot & 0xFF if dot <= 255 else _raise_range(dot))
    if mn == "nop":
        return done(OP_NOP, 0)
    if mn == "wait":
        pol = o.rng("wait polarity", 0, 1)
        src = o.take_id(WAIT_SRC, "wait source")
        if src == 2:
            idx = o.rng("irq flag", 0, 7)
            rel = o.opt_rel()
        else:
            idx = o.rng("pin/bit index", 0, 31)
            rel = 0
            if o.word() in REL:
                raise AsmError("rel/next/prev only valid for 'wait ... irq'")
        return done(OP_WAIT, (pol << 11) | (src << 9) | (rel << 7) | idx)
    if mn in ("in", "out"):
        tbl = IN_SRC if mn == "in" else OUT_DST
        f = o.take_id(tbl, "source" if mn == "in" else "destination")
        o.comma()
        n = o.rng("bit count", 1, 32)
        return done(OP_IN if mn == "in" else OP_OUT, (f << 8) | (n & 31))
    if mn in ("push", "pull"):
        is_pull = mn == "pull"
        cond = None
        block = None
        while True:
            w = o.word()
            if w == ("ifempty" if is_pull else "iffull"):
                if cond is not None:
                    raise AsmError("duplicate '%s'" % w)
                cond = 1
            elif w in ("block", "noblock"):
                if block is not None:
                    raise AsmError("duplicate block/noblock")
                block = 1 if w == "block" else 0
            else:
                break
            o.p += 1
        if block is None:
            block = 1
        return done(OP_PP, (int(is_pull) << 11) | ((cond or 0) << 10) | (block << 9))
    if mn == "mov":
        dst = o.take_id(MOV_DST, "destination")
        o.comma()
        mop = 0
        t = o.peek()
        if _is_op(t, "~") or _is_op(t, "!"):
            mop = 1
            o.p += 1
            src = o.take_id(MOV_SRC, "source")
        elif o.word() in ("rev", "swap") and _is_op(o.peek(1), "("):
            mop = 2 if o.word() == "rev" else 3
            o.p += 2
            src = o.take_id(MOV_SRC, "source")
            if not _is_op(o.peek(), ")"):
                raise AsmError("missing ')'")
            o.p += 1
        else:
            src = o.take_id(MOV_SRC, "source")
        return done(OP_MOV, (dst << 8) | (mop << 6) | src)
    if mn in ALU_OPS or mn in ("rsub", "sltu"):
        dst = o.take_id(ALU_DST, "destination register x, y or z")
        o.comma()
        opn = ALU_OPS.index(mn) if mn in ALU_OPS else 7
        if o.word() in ALU_REG:
            if mn == "rsub":
                raise AsmError("'rsub' has only the immediate form (dst = imm - dst)")
            r1 = o.take_id(ALU_REG, "operand")
            if _is_op(o.peek(), ","):
                o.comma()
                r2 = o.take_id(ALU_REG, "operand")
                a, b = r1, r2
            else:
                a, b = dst, r1
            return done(OP_ALUR, (dst << 10) | (opn << 7) | (a << 4) | (b << 1))
        if mn == "sltu":
            raise AsmError("'sltu' has only register forms (sltu dst, a, b)")
        hi = 31 if mn in ("shl", "shr") else 127
        imm = o.rng("immediate" if hi == 127 else "shift amount", 0, hi)
        if _is_op(o.peek(), ","):
            raise AsmError("immediate form takes 'dst, imm' "
                           "(register form: 'dst, a, b')")
        return done(OP_ALUI, (dst << 10) | (opn << 7) | imm)
    if mn == "set":
        dst = o.take_id(SET_DST, "destination")
        o.comma()
        imm = o.rng("set value", 0, 255)
        return done(OP_SET, (dst << 8) | imm)
    if mn == "irq":
        mode = o.take_id(IRQ_MODE, "irq mode")
        idx = o.rng("irq flag", 0, 7)
        rel = o.opt_rel()
        return done(OP_IRQ, (mode << 10) | (rel << 8) | idx)
    if mn == "loop":
        reg = o.word()
        if reg in LOOP_REG:
            o.p += 1
            op, field = OP_LOOPR, LOOP_REG.index(reg) << 6
        else:
            op, field = OP_LOOP, o.rng("loop count", 0, 255)
        if item.loop_len == -1:           # structure error already reported
            ln = 0
        elif item.loop_len is None:       # raw form: loop N, LEN
            o.comma()
            ln = o.rng("loop LEN", 0, 15)
        else:
            ln = item.loop_len
        return done(op, (ln << 8) | field)
    raise AsmError("internal: unhandled mnemonic " + mn)


def _raise_range(v):
    raise AsmError("halt at address %d does not fit the 8-bit jump address" % v)


def _check_loops(prog, n):
    """Loop/wrap interaction checks (needs wrap and loop info)."""
    wt = prog.wrap_top_abs - prog.origin
    for it in prog.items:
        if it.kind != "instr" or it.toks[0].text.lower() != "loop":
            continue
        ln = it.word_len
        end = it.index + 1 + ln
        if end >= n:
            raise AsmError("loop body extends past the last instruction", it.line)
        if wt == it.index:
            raise AsmError("'loop' sits on the .wrap instruction; wrap would "
                           "redirect the loop entry", it.line)
        if it.index < wt < end:
            raise AsmError("wrap point (.wrap) lies inside a loop body", it.line)
        if wt == end and ((it.word >> 12) & 15) == OP_LOOPR:
            prog.warnings.append(
                "line %d: 'loop <reg>' ends on the .wrap instruction; a zero "
                "count skips the body without wrapping" % it.line)


def assemble_program(prog, gdefines, strict=True):
    errors = []
    n = len(prog.items)
    labels = dict((k, prog.origin + v) for k, v in prog.labels.items())
    words = []
    for it in prog.items:
        env = Env(gdefines, prog.defines, labels, prog.origin + it.index)
        try:
            if it.kind == "word":
                v, pos = parse_expr(it.toks, 0, env)
                if pos != len(it.toks):
                    raise AsmError("unexpected '%s'" % it.toks[pos].text)
                if v < 0 or v > WORD_MASK:
                    raise AsmError(".word value out of 24-bit range")
                it.word = v
            else:
                it.word = _enc_instr(it, prog, env, strict)
            it.word_len = (it.word >> 8) & 15
        except AsmError as e:
            errors.append((it.line, e.msg))
            it.word = 0
            it.word_len = 0
        words.append(it.word)
    prog.words = words
    if n == 0:
        errors.append((prog.line, "program '%s' has no instructions" % prog.name))
        return errors
    if prog.origin + n > 256:
        errors.append((prog.line, "program does not fit: origin %d + %d words "
                       "> 256 (8-bit jump address)" % (prog.origin, n)))
    # wrap
    bottom = prog.wrap_target if prog.wrap_target is not None else 0
    top = prog.wrap_top if prog.wrap_top is not None else n - 1
    if bottom >= n:
        errors.append((prog.line, ".wrap_target after the last instruction"))
        bottom = 0
    if bottom > top:
        errors.append((prog.line, ".wrap_target (%d) is after .wrap (%d)" %
                       (prog.origin + bottom, prog.origin + top)))
    prog.wrap_bottom_abs = prog.origin + bottom
    prog.wrap_top_abs = prog.origin + top
    # entry
    prog.entry = prog.origin
    if prog.entry_toks is not None:
        env = Env(gdefines, prog.defines, labels, None)
        try:
            v, pos = parse_expr(prog.entry_toks, 0, env)
            if pos != len(prog.entry_toks):
                raise AsmError("unexpected '%s' after .entry" %
                               prog.entry_toks[pos].text)
            if not prog.origin <= v < prog.origin + n:
                raise AsmError(".entry %d outside program (%d..%d)" %
                               (v, prog.origin, prog.origin + n - 1))
            prog.entry = v
        except AsmError as e:
            errors.append((prog.entry_line, e.msg))
    if strict and not errors:
        try:
            _check_loops(prog, n)
        except AsmError as e:
            errors.append((e.line, e.msg))
    return errors


def assemble(text, filename="<input>", imem_depth=256, defines=None):
    """Assemble source text. Returns list of Program; raises AsmFailure."""
    return _assemble(text, filename, imem_depth, defines, True)


def _assemble(text, filename, imem_depth, defines, strict):
    cli = {}
    for k, v in (defines or {}).items():
        cli[k] = (tokenize(str(v)), 0)
    p1 = _Parser1(text, imem_depth, cli)
    progs = p1.run()
    errors = [(filename, l, m) for l, m in p1.errors]
    if not progs and not errors:
        errors.append((filename, 1, "no .program found"))
    for p in progs:
        for l, m in assemble_program(p, p1.gdefines, strict):
            errors.append((filename, l, m))
    if errors:
        errors.sort(key=lambda e: (e[1] or 0))
        raise AsmFailure(errors)
    return progs


# --------------------------------------------------------------------------
# Disassembler
# --------------------------------------------------------------------------
def _decode(word, addr, lab):
    """Return instruction text (no delay/side) or None if not decodable."""
    op = (word >> 12) & 15
    arg = word & 0xFFF

    def pick(tbl, v):
        return tbl[v] if v < len(tbl) else None

    if op == OP_JMP:
        cond, tgt = (arg >> 8) & 15, arg & 255
        if cond == 0 and tgt == addr:
            return "halt"
        t = lab(tgt)
        if cond == 0:
            return "jmp " + t
        return "jmp %s, %s" % (CONDS[cond], t)
    if op == OP_WAIT:
        pol, src, rel, idx = (arg >> 11) & 1, (arg >> 9) & 3, (arg >> 7) & 3, arg & 31
        s = "wait %d %s %d" % (pol, WAIT_SRC[src], idx)
        if rel:
            s += " " + REL_NAMES[rel]
        return s
    if op in (OP_IN, OP_OUT):
        f = pick(IN_SRC if op == OP_IN else OUT_DST, (arg >> 8) & 15)
        if f is None:
            return None
        return "%s %s, %d" % ("in" if op == OP_IN else "out", f, (arg & 31) or 32)
    if op == OP_PP:
        pull, cond, blk = (arg >> 11) & 1, (arg >> 10) & 1, (arg >> 9) & 1
        s = "pull" if pull else "push"
        if cond:
            s += " ifempty" if pull else " iffull"
        if not blk:
            s += " noblock"
        return s
    if op == OP_MOV:
        d = pick(MOV_DST, (arg >> 8) & 15)
        s = pick(MOV_SRC, arg & 15)
        mop = (arg >> 6) & 3
        if d is None or s is None:
            return None
        if mop == 1:
            s = "~" + s
        elif mop:
            s = "%s(%s)" % (MOP_NAMES[mop], s)
        return "mov %s, %s" % (d, s)
    if op in (OP_ALUR, OP_ALUI):
        dst = (arg >> 10) & 3
        opn = (arg >> 7) & 7
        if dst > 2:
            return None
        if op == OP_ALUR:
            name = ALU_OPS[opn] if opn < 7 else "sltu"
            return "%s %s, %s, %s" % (name, ALU_DST[dst], ALU_REG[(arg >> 4) & 7],
                                      ALU_REG[(arg >> 1) & 7])
        name = ALU_OPS[opn] if opn < 7 else "rsub"
        return "%s %s, %d" % (name, ALU_DST[dst], arg & 127)
    if op == OP_SET:
        d = pick(SET_DST, (arg >> 8) & 15)
        return None if d is None else "set %s, %d" % (d, arg & 255)
    if op == OP_LOOP:
        return "loop %d, %d" % (arg & 255, (arg >> 8) & 15)
    if op == OP_LOOPR:
        r = pick(LOOP_REG, (arg >> 6) & 3)
        return None if r is None else "loop %s, %d" % (r, (arg >> 8) & 15)
    if op == OP_IRQ:
        s = "irq %s %d" % (IRQ_MODE[(arg >> 10) & 3], arg & 7)
        rel = (arg >> 8) & 3
        return s + (" " + REL_NAMES[rel] if rel else "")
    if op == OP_NOP:
        return "nop"
    return None


def _suffix(word, side):
    dly = (word >> 20) & 15
    ss = (word >> 16) & 15
    s = ""
    if dly:
        s += " [%d]" % dly
    if side.count:
        if side.opt:
            if ss & 8:
                s += " side %d" % (ss & 7)
        elif ss:
            s += " side %d" % ss
    return s


def _verify(text, word, addr, side):
    src = ".program _v\n%s\n.origin %d\n%s\n" % (side.directive(), addr, text)
    try:
        ps = _assemble(src, "<v>", 256, None, False)
    except AsmFailure:
        return False
    return ps[0].words[0] == word


def disassemble_word(word, addr, side, lab=None):
    """Text for one word; '.word 0x......' if it has no canonical form."""
    num = lambda a: str(a)
    body = _decode(word, addr, num)
    if body is not None:
        text = body + _suffix(word, side)
        if _verify(text, word, addr, side):
            if lab is not None:
                return _decode(word, addr, lab) + _suffix(word, side)
            return text
    return ".word 0x%06X" % word


def infer_side(words):
    m = 0
    for w in words:
        m |= (w >> 16) & 15
    return SideCfg(m.bit_length() if m else 0, False, False)


def disassemble(words, origin=0, side=None, name="disasm", wrap_bottom=None,
                wrap_top=None, entry=None):
    """Words -> assembler source text (assembles back to the same words)."""
    if side is None:
        side = infer_side(words)
    n = len(words)
    targets = set()
    for i, w in enumerate(words):
        if ((w >> 12) & 15) == OP_JMP:
            t = w & 255
            if origin <= t < origin + n and not (((w >> 8) & 15) == 0 and t == origin + i):
                targets.add(t)
    lab = lambda a: ("L%d" % a) if a in targets else str(a)
    out = [".program %s" % name]
    if side.count:
        out.append(side.directive())
    if origin:
        out.append(".origin %d" % origin)
    if entry is not None and entry != origin:
        out.append(".entry %d" % entry)
    for i, w in enumerate(words):
        a = origin + i
        if a in targets:
            out.append("L%d:" % a)
        if wrap_bottom is not None and wrap_bottom != origin and a == wrap_bottom:
            out.append(".wrap_target")
        out.append("    " + disassemble_word(w, a, side, lab))
        if wrap_top is not None and wrap_top != origin + n - 1 and a == wrap_top:
            out.append(".wrap")
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------
# Hex I/O
# --------------------------------------------------------------------------
def format_hex(prog, meta=False):
    out = []
    if meta:
        out.append("// .program %s" % prog.name)
        if prog.side.count:
            out.append("// " + prog.side.directive())
        out.append("// .wrap_target %d" % prog.wrap_bottom_abs)
        out.append("// .wrap %d" % prog.wrap_top_abs)
        out.append("// .entry %d" % prog.entry)
    if prog.origin:
        out.append("@%X" % prog.origin)
    out.extend("%06X" % w for w in prog.words)
    return "\n".join(out) + "\n"


def parse_hex(text):
    """Return (origin, words, meta). meta keys: program, side_set, wrap_target,
    wrap, entry (from '// .directive value' comments)."""
    words, origin, meta = [], None, {}
    for n, raw in enumerate(text.splitlines(), 1):
        s = raw.strip()
        m = re.match(r"^//\s*\.(\w+)\s*(.*)$", s)
        if m:
            meta[m.group(1)] = m.group(2).strip()
            continue
        for c in ("//", "#", ";"):
            i = s.find(c)
            if i >= 0:
                s = s[:i]
        s = s.strip()
        if not s:
            continue
        if s.startswith("@"):
            try:
                a = int(s[1:].replace("_", ""), 16)
            except ValueError:
                raise AsmFailure([("<hex>", n, "bad address line '%s'" % s)])
            if words or origin is not None:
                raise AsmFailure([("<hex>", n, "only one leading @address "
                                   "is supported")])
            origin = a
            continue
        for tok in s.split():
            try:
                v = int(tok.replace("_", ""), 16)
            except ValueError:
                raise AsmFailure([("<hex>", n, "bad hex word '%s'" % tok)])
            if v < 0 or v > WORD_MASK:
                raise AsmFailure([("<hex>", n, "word %s exceeds 24 bits" % tok)])
            words.append(v)
    return (origin or 0), words, meta


def _parse_side_arg(s):
    toks = s.replace(",", " ").split()
    if not toks:
        return SideCfg()
    try:
        cnt = int(toks[0], 0)
    except ValueError:
        raise SystemExit("pioasm: bad --side-set '%s'" % s)
    opt = "opt" in toks[1:]
    pind = "pindirs" in toks[1:] or "pindir" in toks[1:]
    return SideCfg(cnt, opt, pind)


# --------------------------------------------------------------------------
# Emitters
# --------------------------------------------------------------------------
def format_c(prog, source_name):
    U = prog.name.upper()
    L = prog.name.lower()
    ws = ",\n".join("    0x%06X" % w for w in prog.words)
    return """/* Generated by pioasm.py from %(src)s (program %(name)s) - do not edit. */
#ifndef PIOASM_%(U)s_H
#define PIOASM_%(U)s_H
#include <stdint.h>

#define %(U)s_ORIGIN       %(origin)d
#define %(U)s_LENGTH       %(length)d
#define %(U)s_WRAP_BOTTOM  %(wb)d
#define %(U)s_WRAP_TOP     %(wt)d
#define %(U)s_ENTRY        %(entry)d
#define %(U)s_SIDE_COUNT   %(sc)d
#define %(U)s_SIDE_OPT     %(so)d
#define %(U)s_SIDE_PINDIR  %(sp)d

/* 24-bit instruction words, first word loads at address %(U)s_ORIGIN */
static const uint32_t %(L)s_program[%(U)s_LENGTH] = {
%(ws)s
};

#endif /* PIOASM_%(U)s_H */
""" % dict(src=source_name, name=prog.name, U=U, L=L, origin=prog.origin,
           length=prog.length, wb=prog.wrap_bottom_abs, wt=prog.wrap_top_abs,
           entry=prog.entry, sc=prog.side.count, so=int(prog.side.opt),
           sp=int(prog.side.pindir), ws=ws)


def format_svh(prog, source_name):
    U = prog.name.upper()
    ws = ",\n".join("  24'h%06X" % w for w in prog.words)
    return """// Generated by pioasm.py from %(src)s (program %(name)s) - do not edit.
`ifndef PIOASM_%(U)s_SVH
`define PIOASM_%(U)s_SVH
localparam int unsigned %(U)s_ORIGIN      = %(origin)d;
localparam int unsigned %(U)s_LENGTH      = %(length)d;
localparam int unsigned %(U)s_WRAP_BOTTOM = %(wb)d;
localparam int unsigned %(U)s_WRAP_TOP    = %(wt)d;
localparam int unsigned %(U)s_ENTRY       = %(entry)d;
localparam int unsigned %(U)s_SIDE_COUNT  = %(sc)d;
localparam bit          %(U)s_SIDE_OPT    = 1'b%(so)d;
localparam bit          %(U)s_SIDE_PINDIR = 1'b%(sp)d;
// instruction words; element 0 loads at address %(U)s_ORIGIN
localparam logic [23:0] %(U)s_INSTR [0:%(last)d] = '{
%(ws)s
};
`endif // PIOASM_%(U)s_SVH
""" % dict(src=source_name, name=prog.name, U=U, origin=prog.origin,
           length=prog.length, wb=prog.wrap_bottom_abs, wt=prog.wrap_top_abs,
           entry=prog.entry, sc=prog.side.count, so=int(prog.side.opt),
           sp=int(prog.side.pindir), ws=ws, last=prog.length - 1)


def format_lst(prog, source_name):
    by_line = {}
    for it in prog.items:
        by_line[it.line] = it
    out = ["; pioasm listing  %s  program %s" % (source_name, prog.name),
           "; origin=%d length=%d wrap_bottom=%d wrap_top=%d entry=%d "
           "side_set=%d%s%s" % (prog.origin, prog.length, prog.wrap_bottom_abs,
                                prog.wrap_top_abs, prog.entry, prog.side.count,
                                " opt" if prog.side.opt else "",
                                " pindirs" if prog.side.pindir else ""),
           "; addr  word    line  source"]
    for n, raw in prog.src_lines:
        it = by_line.get(n)
        if it is not None:
            out.append(" %4d  %06X  %4d  %s" % (prog.origin + it.index, it.word,
                                                  n, raw.rstrip()))
        else:
            out.append(" %4s  %6s  %4d  %s" % ("", "", n, raw.rstrip()))
    return "\n".join(out) + "\n"


def _out_path(base, prog, multi):
    if base == "-" or not multi:
        return base
    root, ext = os.path.splitext(base)
    return "%s_%s%s" % (root, prog.name, ext)


def _write(path, text):
    if path == "-":
        sys.stdout.write(text)
        return
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    with open(path, "w") as f:
        f.write(text)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="pioasm.py",
        description="Assembler/disassembler for the PIO engine ISA v1.")
    ap.add_argument("input", help="source .pio file (or .hex with --disasm)")
    ap.add_argument("--hex", metavar="FILE", help="$readmemh output")
    ap.add_argument("--c", metavar="FILE", help="C header output")
    ap.add_argument("--svh", metavar="FILE", help="SystemVerilog localparam header")
    ap.add_argument("--lst", metavar="FILE", help="listing")
    ap.add_argument("--meta", action="store_true",
                    help="add '// .directive' metadata comments to the hex file")
    ap.add_argument("--disasm", action="store_true",
                    help="treat INPUT as a hex file and print assembler source")
    ap.add_argument("--side-set", metavar="'N [opt] [pindirs]'",
                    help="(--disasm) side-set configuration of the hex words")
    ap.add_argument("--name", default=None, help="(--disasm) program name")
    ap.add_argument("-o", metavar="FILE", help="(--disasm) output file (default stdout)")
    ap.add_argument("-D", action="append", default=[], metavar="NAME=VALUE",
                    help="global define (like .define)")
    ap.add_argument("--imem-depth", type=int, default=256,
                    help="instruction memory depth for range checks (default 256)")
    ap.add_argument("-q", "--quiet", action="store_true", help="suppress warnings")
    a = ap.parse_args(argv)

    try:
        with open(a.input) as f:
            text = f.read()
    except IOError as e:
        sys.stderr.write("pioasm: cannot read %s: %s\n" % (a.input, e))
        return 2

    if a.disasm:
        try:
            origin, words, meta = parse_hex(text)
        except AsmFailure as e:
            sys.stderr.write("%s: %s\n" % (a.input, e))
            return 1
        side = _parse_side_arg(a.side_set) if a.side_set is not None else None
        if side is None and "side_set" in meta:
            side = _parse_side_arg(meta["side_set"])

        def mi(k):
            try:
                return int(meta[k], 0) if k in meta else None
            except ValueError:
                return None
        name = a.name or meta.get("program") or "disasm"
        out = disassemble(words, origin, side, name, mi("wrap_target"),
                          mi("wrap"), mi("entry"))
        _write(a.o or "-", out)
        return 0

    defs = {}
    for d in a.D:
        if "=" not in d:
            sys.stderr.write("pioasm: -D expects NAME=VALUE\n")
            return 2
        k, v = d.split("=", 1)
        defs[k.strip()] = v.strip()
    try:
        progs = assemble(text, a.input, a.imem_depth, defs)
    except AsmFailure as e:
        for f, l, m in e.errors:
            sys.stderr.write("%s:%s: error: %s\n" % (f, l if l else "?", m))
        sys.stderr.write("pioasm: %d error(s), no output written\n" % len(e.errors))
        return 1
    multi = len(progs) > 1
    for p in progs:
        if not a.quiet:
            for w in p.warnings:
                sys.stderr.write("%s: warning: %s\n" % (a.input, w))
        if a.hex:
            _write(_out_path(a.hex, p, multi), format_hex(p, a.meta))
        if a.c:
            _write(_out_path(a.c, p, multi), format_c(p, os.path.basename(a.input)))
        if a.svh:
            _write(_out_path(a.svh, p, multi), format_svh(p, os.path.basename(a.input)))
        if a.lst:
            _write(_out_path(a.lst, p, multi), format_lst(p, os.path.basename(a.input)))
    if not (a.hex or a.c or a.svh or a.lst) and not a.quiet:
        for p in progs:
            sys.stderr.write("%s: program %s: %d words, origin %d, wrap %d..%d, "
                             "entry %d\n" % (a.input, p.name, p.length, p.origin,
                                             p.wrap_bottom_abs, p.wrap_top_abs,
                                             p.entry))
    return 0


if __name__ == "__main__":
    sys.exit(main())
