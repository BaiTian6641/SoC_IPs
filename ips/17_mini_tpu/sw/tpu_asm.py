#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Mini-TPU / NPU v2.0 64-Bit Microcode Assembler & Encoder (Stdlib Python 3).

Supports all 12 opcodes (4'h0..4'hB):
  NOP, CFG, CFG_GSCALE, DMA_LD, DMA_LD_BG, DMA_ST, GEMM, SFU,
  LOOP_INIT, LOOP_NEXT, HALT, BREAK, CONV2D, POOL, VEC_EW, SYNC_BANK

Outputs hex (`--format hex`), 32-bit IMEM word pairs (`--format words32`),
C uint64_t array (`--format c`), or SystemVerilog array (`--format sv`).

Usage:
  python3 ips/17_mini_tpu/sw/tpu_asm.py --self-test
  python3 ips/17_mini_tpu/sw/tpu_asm.py input.asm -f c -o prog.h
"""

import argparse
import re
import struct
import sys
from typing import Dict, List, Optional, Tuple


# -----------------------------------------------------------------------------
# Architectural Constants & Mnemonic Tables
# -----------------------------------------------------------------------------
PREC_MAP: Dict[str, Tuple[int, int, int]] = {
    # name -> (prec_mode[2:0], prec_ext, int4_scale_en)
    "INT8_SYM":    (0, 0, 0), "PREC_INT8_SYM":  (0, 0, 0),
    "INT8_ASYM":   (1, 0, 0), "PREC_INT8_ASYM": (1, 0, 0),
    "FP8_E4M3":    (2, 0, 0), "PREC_FP8_E4M3":  (2, 0, 0),
    "FP8_E5M2":    (3, 0, 0), "PREC_FP8_E5M2":  (3, 0, 0),
    "FP8_MIXED":   (4, 0, 0), "PREC_FP8_MIXED": (4, 0, 0),
    "NVFP4":       (5, 0, 0), "PREC_NVFP4":     (5, 0, 0),
    "BF16":        (6, 0, 0), "PREC_BF16":      (6, 0, 0),
    "FP16":        (7, 0, 0), "PREC_FP16":      (7, 0, 0),
    "INT4":        (0, 1, 0), "PREC_INT4":      (0, 1, 0),
    "INT4_SCALED": (0, 1, 1), "PREC_INT4_SCALED": (0, 1, 1),
}

ACT_MAP: Dict[str, int] = {
    "NONE":       0, "ACT_NONE":       0,
    "RELU":       1, "ACT_RELU":       1,
    "RELU6":      2, "ACT_RELU6":      2,
    "LEAKY_RELU": 3, "ACT_LEAKY_RELU": 3,
    "SILU":       4, "ACT_SILU":       4,
    "GELU":       5, "ACT_GELU":       5,
}

OUT_FMT_MAP: Dict[str, int] = {
    "FP32":     0, "OUT_FP32":     0,
    "INT32":    1, "OUT_INT32":    1,
    "INT8":     2, "OUT_INT8":     2,
    "FP8_E4M3": 3, "OUT_FP8_E4M3": 3,
    "FP8_E5M2": 4, "OUT_FP8_E5M2": 4,
    "NVFP4":    5, "OUT_NVFP4":    5,
    "BF16":     6, "OUT_BF16":     6,
    "FP16":     7, "OUT_FP16":     7,
}

BUF_MAP: Dict[str, int] = {
    "ABUF": 0,
    "WBUF": 1,
    "SBUF": 2,
    "OBUF": 3,
    "IMEM": 4,
}

POOL_FMT_MAP: Dict[str, int] = {
    "FP32": 0,
    "INT8": 1,
    "BF16": 2,
    "FP16": 3,
}

EW_OP_MAP: Dict[str, int] = {
    "ADD":     0, "EW_ADD":     0,
    "SUB":     1, "EW_SUB":     1,
    "MUL":     2, "EW_MUL":     2,
    "RMSNORM": 3, "EW_RMSNORM": 3,
    "SOFTMAX": 4, "EW_SOFTMAX": 4,
}

EW_FMT_MAP: Dict[str, int] = {
    "FP32":  0,
    "BF16":  1,
    "FP16":  2,
    "INT32": 3,
}


# -----------------------------------------------------------------------------
# Bitwise Encoders for All 12 ISA Instructions (4'h0..4'hB)
# -----------------------------------------------------------------------------
def _chk(name: str, val: int, lo: int, hi: int) -> int:
    if val < lo or val > hi:
        raise ValueError(f"Field '{name}' value {val} out of range [{lo}, {hi}]")
    return val


def enc_nop() -> int:
    return 0


def enc_cfg_quant(
    prec: int, act: int, out_fmt: int,
    zp_a: int = 0, zp_b: int = 0, out_zp: int = 0, nvfp4_out_scale: int = 0,
    prec_ext: int = 0, int4_scale_en: int = 0,
) -> int:
    return (
        (1 << 60)
        | (_chk("prec", prec, 0, 7) << 57)
        | (_chk("act", act, 0, 7) << 54)
        | (_chk("out_fmt", out_fmt, 0, 7) << 51)
        | (_chk("prec_ext", prec_ext, 0, 1) << 49)
        | (_chk("int4_scale_en", int4_scale_en, 0, 1) << 48)
        | ((zp_b & 0xFF) << 24)
        | ((zp_a & 0xFF) << 16)
        | ((nvfp4_out_scale & 0xFF) << 8)
        | (out_zp & 0xFF)
    )


def enc_cfg_gscale(
    prec: int, act: int, out_fmt: int, gscale_fp32_bits: int,
    prec_ext: int = 0, int4_scale_en: int = 0,
) -> int:
    return (
        (1 << 60)
        | (_chk("prec", prec, 0, 7) << 57)
        | (_chk("act", act, 0, 7) << 54)
        | (_chk("out_fmt", out_fmt, 0, 7) << 51)
        | (1 << 50)
        | (_chk("prec_ext", prec_ext, 0, 1) << 49)
        | (_chk("int4_scale_en", int4_scale_en, 0, 1) << 48)
        | (gscale_fp32_bits & 0xFFFFFFFF)
    )


def enc_dma_ld(
    base_sel: int, buf_sel: int, buf_word_addr: int,
    num_rows: int, words_per_row: int, ext_row_stride_bytes: int,
    ext_word_offset: int, async_bg: int = 0,
) -> int:
    if async_bg and buf_sel not in (0, 1):
        raise ValueError("async_bg is only valid for ABUF (0) or WBUF (1)")
    return (
        (2 << 60)
        | (_chk("base_sel", base_sel, 0, 1) << 59)
        | (_chk("buf_sel", buf_sel, 0, 4) << 56)
        | (_chk("async_bg", async_bg, 0, 1) << 55)
        | (_chk("buf_word_addr", buf_word_addr, 0, 1023) << 44)
        | (_chk("num_rows", num_rows, 0, 255) << 36)
        | (_chk("words_per_row", words_per_row, 0, 255) << 28)
        | (_chk("ext_row_stride_bytes", ext_row_stride_bytes, 0, 0xFFF) << 16)
        | _chk("ext_word_offset", ext_word_offset, 0, 0xFFFF)
    )


def enc_dma_st(
    base_sel: int, buf_sel: int, buf_word_addr: int,
    num_rows: int, words_per_row: int, ext_row_stride_bytes: int,
    ext_word_offset: int,
) -> int:
    return (
        (3 << 60)
        | (_chk("base_sel", base_sel, 0, 1) << 59)
        | (_chk("buf_sel", buf_sel, 0, 4) << 56)
        | (_chk("buf_word_addr", buf_word_addr, 0, 1023) << 44)
        | (_chk("num_rows", num_rows, 0, 255) << 36)
        | (_chk("words_per_row", words_per_row, 0, 255) << 28)
        | (_chk("ext_row_stride_bytes", ext_row_stride_bytes, 0, 0xFFF) << 16)
        | _chk("ext_word_offset", ext_word_offset, 0, 0xFFFF)
    )


def enc_gemm(
    acc_clear: int, m_tiles: int, n_tiles: int, k_groups: int,
    abuf_base: int, wbuf_base: int, obuf_base: int, sbuf_scale_base: int = 0,
) -> int:
    return (
        (4 << 60)
        | (_chk("acc_clear", acc_clear, 0, 1) << 59)
        | (_chk("m_tiles", m_tiles, 0, 15) << 55)
        | (_chk("n_tiles", n_tiles, 0, 15) << 51)
        | (_chk("k_groups", k_groups, 0, 127) << 44)
        | (_chk("abuf_base", abuf_base, 0, 1023) << 34)
        | (_chk("wbuf_base", wbuf_base, 0, 1023) << 24)
        | (_chk("obuf_base", obuf_base, 0, 1023) << 14)
        | _chk("sbuf_scale_base", sbuf_scale_base, 0, 0x3FFF)
    )


def enc_sfu(
    dst_is_abuf: int, use_per_ch_scale: int, use_bias: int, num_vec4: int,
    src_obuf_base: int, dst_base: int, bias_sbuf_base: int = 0,
    scale_sbuf_base: int = 0, cols4_mod: int = 0,
) -> int:
    return (
        (5 << 60)
        | (_chk("dst_is_abuf", dst_is_abuf, 0, 1) << 59)
        | (_chk("use_per_ch_scale", use_per_ch_scale, 0, 1) << 58)
        | (_chk("use_bias", use_bias, 0, 1) << 57)
        | (_chk("num_vec4", num_vec4, 0, 511) << 48)
        | (_chk("src_obuf_base", src_obuf_base, 0, 1023) << 38)
        | (_chk("dst_base", dst_base, 0, 1023) << 28)
        | (_chk("bias_sbuf_base", bias_sbuf_base, 0, 511) << 19)
        | (_chk("scale_sbuf_base", scale_sbuf_base, 0, 511) << 10)
        | _chk("cols4_mod", cols4_mod, 0, 1023)
    )


def enc_loop_init(loop_id: int, count: int) -> int:
    return (
        (6 << 60)
        | (_chk("loop_id", loop_id, 0, 1) << 59)
        | (1 << 58)
        | _chk("count", count, 0, 0xFFFF)
    )


def enc_loop_next(loop_id: int, target_pc: int) -> int:
    return (
        (6 << 60)
        | (_chk("loop_id", loop_id, 0, 1) << 59)
        | (_chk("target_pc", target_pc, 0, 63) << 16)
    )


def enc_halt(is_break: int = 0) -> int:
    return (7 << 60) | (_chk("is_break", is_break, 0, 1) << 59)


def enc_conv2d(
    acc_clear: int, kh: int, kw: int, stride: int, pad: int,
    h_in: int, w_in: int, cin_groups: int, cout_tiles: int,
    abuf_base: int, wbuf_base: int, obuf_base: int, sbuf_w4_base: int = 0,
) -> int:
    return (
        (8 << 60)
        | (_chk("acc_clear", acc_clear, 0, 1) << 59)
        | ((_chk("kh", kh, 1, 4) - 1) << 57)
        | ((_chk("kw", kw, 1, 4) - 1) << 55)
        | ((_chk("stride", stride, 1, 4) - 1) << 53)
        | (_chk("pad", pad, 0, 3) << 51)
        | ((_chk("h_in", h_in, 1, 16) - 1) << 47)
        | ((_chk("w_in", w_in, 1, 16) - 1) << 43)
        | ((_chk("cin_groups", cin_groups, 1, 8) - 1) << 40)
        | ((_chk("cout_tiles", cout_tiles, 1, 8) - 1) << 37)
        | (_chk("abuf_base", abuf_base, 0, 1023) << 27)
        | (_chk("wbuf_base", wbuf_base, 0, 1023) << 17)
        | (_chk("obuf_base", obuf_base, 0, 1023) << 7)
        | _chk("sbuf_w4_base", sbuf_w4_base, 0, 127)
    )


def enc_pool(
    is_avg: int, win_size: int, stride: int, src_is_abuf: int, dst_is_abuf: int,
    fmt: int, pad: int, h_in: int, w_in: int, c_words: int,
    src_base: int, dst_base: int,
) -> int:
    if win_size not in (2, 3):
        raise ValueError(f"OP_POOL win_size must be 2 or 3, got {win_size}")
    if stride not in (1, 2):
        raise ValueError(f"OP_POOL stride must be 1 or 2, got {stride}")
    return (
        (9 << 60)
        | (_chk("is_avg", is_avg, 0, 1) << 59)
        | ((1 if win_size == 3 else 0) << 58)
        | ((1 if stride == 2 else 0) << 57)
        | (_chk("src_is_abuf", src_is_abuf, 0, 1) << 56)
        | (_chk("dst_is_abuf", dst_is_abuf, 0, 1) << 55)
        | (_chk("fmt", fmt, 0, 3) << 53)
        | (_chk("pad", pad, 0, 3) << 51)
        | ((_chk("h_in", h_in, 1, 16) - 1) << 47)
        | ((_chk("w_in", w_in, 1, 16) - 1) << 43)
        | ((_chk("c_words", c_words, 1, 32) - 1) << 38)
        | (_chk("src_base", src_base, 0, 1023) << 28)
        | (_chk("dst_base", dst_base, 0, 1023) << 18)
    )


def enc_vec_ew(
    ew_op: int, srca_is_abuf: int, srcb_flag: int, dst_is_abuf: int,
    fmt: int, num_rows: int, vec_len: int,
    srca_base: int, srcb_base: int, dst_base: int, aux: int = 0,
) -> int:
    return (
        (10 << 60)
        | (_chk("ew_op", ew_op, 0, 4) << 57)
        | (_chk("srca_is_abuf", srca_is_abuf, 0, 1) << 56)
        | (_chk("srcb_flag", srcb_flag, 0, 1) << 55)
        | (_chk("dst_is_abuf", dst_is_abuf, 0, 1) << 54)
        | (_chk("fmt", fmt, 0, 3) << 52)
        | ((_chk("num_rows", num_rows, 1, 64) - 1) << 46)
        | ((_chk("vec_len", vec_len, 1, 256) - 1) << 38)
        | (_chk("srca_base", srca_base, 0, 1023) << 28)
        | (_chk("srcb_base", srcb_base, 0, 1023) << 18)
        | (_chk("dst_base", dst_base, 0, 1023) << 8)
        | _chk("aux", aux, 0, 255)
    )


def enc_sync_bank(
    swap_abuf: int = 0, swap_wbuf: int = 0, wait_dma: int = 1,
    set_explicit: int = 0, abuf_bank: int = 0, wbuf_bank: int = 0,
    pp_abuf_en: int = 0, pp_wbuf_en: int = 0,
) -> int:
    return (
        (11 << 60)
        | (_chk("swap_abuf", swap_abuf, 0, 1) << 59)
        | (_chk("swap_wbuf", swap_wbuf, 0, 1) << 58)
        | (_chk("set_explicit", set_explicit, 0, 1) << 57)
        | (_chk("wait_dma", wait_dma, 0, 1) << 56)
        | (_chk("abuf_bank", abuf_bank, 0, 1) << 55)
        | (_chk("wbuf_bank", wbuf_bank, 0, 1) << 54)
        | (_chk("pp_abuf_en", pp_abuf_en, 0, 1) << 53)
        | (_chk("pp_wbuf_en", pp_wbuf_en, 0, 1) << 52)
    )


# -----------------------------------------------------------------------------
# Text Assembly Parser
# -----------------------------------------------------------------------------
def _parse_int(tok: str) -> int:
    t = tok.strip()
    if t.endswith(("f", "F")) and ("." in t or "e" in t.lower()):
        val = float(t[:-1])
        return struct.unpack(">I", struct.pack(">f", val))[0]
    return int(t, 0)


def _lookup(tok: str, table: Dict[str, int], kind: str) -> int:
    u = tok.strip().upper()
    if u in table:
        return table[u]
    try:
        return int(tok, 0)
    except ValueError as exc:
        raise ValueError(f"Unknown {kind} token '{tok}'") from exc


def _parse_prec(tok: str) -> Tuple[int, int, int]:
    u = tok.strip().upper()
    if u in PREC_MAP:
        return PREC_MAP[u]
    return (int(tok, 0), 0, 0)


def _split_line(line: str) -> Tuple[Optional[str], Optional[str], List[str]]:
    # Strip comments (# or //)
    line = re.sub(r"(?://|#).*$", "", line).strip()
    if not line:
        return None, None, []
    label = None
    if ":" in line:
        head, rest = line.split(":", 1)
        if re.fullmatch(r"[A-Za-z_]\w*", head.strip()):
            label = head.strip()
            line = rest.strip()
    if not line:
        return label, None, []
    parts = [p.strip() for p in re.split(r"[\s,]+", line) if p.strip()]
    return label, parts[0].upper(), parts[1:]


def assemble(src: str) -> List[int]:
    """Assemble multi-line Mini-TPU assembly text into 64-bit instructions."""
    labels: Dict[str, int] = {}
    parsed: List[Tuple[int, str, List[str]]] = []

    pc = 0
    for lineno, raw in enumerate(src.splitlines(), start=1):
        lbl, op, args = _split_line(raw)
        if lbl is not None:
            if lbl in labels:
                raise ValueError(f"Line {lineno}: duplicate label '{lbl}'")
            labels[lbl] = pc
        if op is not None:
            parsed.append((lineno, op, args))
            pc += 1

    if len(parsed) > 64:
        raise ValueError(f"Program exceeds 64-instruction IMEM capacity ({len(parsed)} insts)")

    out: List[int] = []
    for lineno, op, args in parsed:
        try:
            if op == "NOP":
                out.append(enc_nop())
            elif op == "CFG":
                # CFG prec, act, out_fmt [, zp_a, zp_b, out_zp, nvfp4_out_scale]
                pm, pext, i4s = _parse_prec(args[0])
                act = _lookup(args[1], ACT_MAP, "activation")
                ofmt = _lookup(args[2], OUT_FMT_MAP, "out_fmt")
                zpa = _parse_int(args[3]) if len(args) > 3 else 0
                zpb = _parse_int(args[4]) if len(args) > 4 else 0
                ozp = _parse_int(args[5]) if len(args) > 5 else 0
                nvs = _parse_int(args[6]) if len(args) > 6 else 0
                out.append(enc_cfg_quant(pm, act, ofmt, zpa, zpb, ozp, nvs, pext, i4s))
            elif op == "CFG_GSCALE":
                # CFG_GSCALE prec, act, out_fmt, fp32_val
                pm, pext, i4s = _parse_prec(args[0])
                act = _lookup(args[1], ACT_MAP, "activation")
                ofmt = _lookup(args[2], OUT_FMT_MAP, "out_fmt")
                gs = _parse_int(args[3])
                out.append(enc_cfg_gscale(pm, act, ofmt, gs, pext, i4s))
            elif op in ("DMA_LD", "DMA_LD_BG"):
                # DMA_LD[_BG] base_sel, buf_sel, buf_addr, rows, wpr, stride_b, ext_woff
                bg = 1 if op == "DMA_LD_BG" else 0
                bsel = _parse_int(args[0])
                buf = _lookup(args[1], BUF_MAP, "buffer")
                baddr = _parse_int(args[2])
                rows = _parse_int(args[3])
                wpr = _parse_int(args[4])
                strb = _parse_int(args[5])
                eoff = _parse_int(args[6])
                out.append(enc_dma_ld(bsel, buf, baddr, rows, wpr, strb, eoff, async_bg=bg))
            elif op == "DMA_ST":
                bsel = _parse_int(args[0])
                buf = _lookup(args[1], BUF_MAP, "buffer")
                baddr = _parse_int(args[2])
                rows = _parse_int(args[3])
                wpr = _parse_int(args[4])
                strb = _parse_int(args[5])
                eoff = _parse_int(args[6])
                out.append(enc_dma_st(bsel, buf, baddr, rows, wpr, strb, eoff))
            elif op == "GEMM":
                # GEMM acc_clr, m_tiles, n_tiles, k_groups, abuf, wbuf, obuf [, sbuf_scl]
                vals = [_parse_int(a) for a in args]
                sbuf_s = vals[7] if len(vals) > 7 else 0
                out.append(enc_gemm(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], sbuf_s))
            elif op == "SFU":
                # SFU dst_abuf, pch_scl, use_bias, num_vec4, src_obuf, dst [, bias_sb, scale_sb, cols4_mod]
                vals = [_parse_int(a) for a in args]
                bsb = vals[6] if len(vals) > 6 else 0
                ssb = vals[7] if len(vals) > 7 else 0
                cmod = vals[8] if len(vals) > 8 else 0
                out.append(enc_sfu(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], bsb, ssb, cmod))
            elif op == "LOOP_INIT":
                out.append(enc_loop_init(_parse_int(args[0]), _parse_int(args[1])))
            elif op == "LOOP_NEXT":
                lid = _parse_int(args[0])
                tgt = labels[args[1]] if args[1] in labels else _parse_int(args[1])
                out.append(enc_loop_next(lid, tgt))
            elif op == "HALT":
                out.append(enc_halt(_parse_int(args[0]) if args else 0))
            elif op == "BREAK":
                out.append(enc_halt(1))
            elif op == "CONV2D":
                # CONV2D clr, kh, kw, stride, pad, h_in, w_in, cin_grp, cout_tiles, abuf, wbuf, obuf [, sbuf_w4]
                vals = [_parse_int(a) for a in args]
                sbw4 = vals[12] if len(vals) > 12 else 0
                out.append(enc_conv2d(
                    vals[0], vals[1], vals[2], vals[3], vals[4],
                    vals[5], vals[6], vals[7], vals[8], vals[9], vals[10], vals[11], sbw4,
                ))
            elif op == "POOL":
                # POOL MAX|AVG, win(2|3), stride(1|2), src_abuf, dst_abuf, fmt, pad, h_in, w_in, c_words, src, dst
                is_avg = 1 if args[0].upper() in ("AVG", "AVGPOOL", "1") else 0
                win = _parse_int(args[1])
                stride = _parse_int(args[2])
                sab = _parse_int(args[3])
                dab = _parse_int(args[4])
                fmt = _lookup(args[5], POOL_FMT_MAP, "pool_fmt")
                pad = _parse_int(args[6])
                hin = _parse_int(args[7])
                win_w = _parse_int(args[8])
                cw = _parse_int(args[9])
                s_b = _parse_int(args[10])
                d_b = _parse_int(args[11])
                out.append(enc_pool(is_avg, win, stride, sab, dab, fmt, pad, hin, win_w, cw, s_b, d_b))
            elif op == "VEC_EW":
                # VEC_EW ew_op, srca_abuf, srcb_flag, dst_abuf, fmt, num_rows, vec_len, srca, srcb, dst [, aux]
                eop = _lookup(args[0], EW_OP_MAP, "ew_op")
                sa_ab = _parse_int(args[1])
                sb_fl = _parse_int(args[2])
                d_ab = _parse_int(args[3])
                fmt = _lookup(args[4], EW_FMT_MAP, "ew_fmt")
                nrows = _parse_int(args[5])
                vlen = _parse_int(args[6])
                sa = _parse_int(args[7])
                sb = _parse_int(args[8])
                dst = _parse_int(args[9])
                aux = _parse_int(args[10]) if len(args) > 10 else 0
                out.append(enc_vec_ew(eop, sa_ab, sb_fl, d_ab, fmt, nrows, vlen, sa, sb, dst, aux))
            elif op == "SYNC_BANK":
                # SYNC_BANK swap_abuf, swap_wbuf [, wait_dma]  OR  SYNC_BANK SET, ab, wb, pp_a, pp_w
                if args and args[0].upper() == "SET":
                    out.append(enc_sync_bank(
                        set_explicit=1,
                        abuf_bank=_parse_int(args[1]),
                        wbuf_bank=_parse_int(args[2]),
                        pp_abuf_en=_parse_int(args[3]),
                        pp_wbuf_en=_parse_int(args[4]),
                    ))
                else:
                    sw_a = _parse_int(args[0]) if len(args) > 0 else 1
                    sw_w = _parse_int(args[1]) if len(args) > 1 else 1
                    wdma = _parse_int(args[2]) if len(args) > 2 else 1
                    out.append(enc_sync_bank(swap_abuf=sw_a, swap_wbuf=sw_w, wait_dma=wdma))
            else:
                raise ValueError(f"Unknown opcode '{op}'")
        except (IndexError, ValueError) as exc:
            raise ValueError(f"Line {lineno} ({op}): {exc}") from exc

    return out


def format_output(insts: List[int], fmt: str, sym_name: str = "tpu_prog") -> str:
    if fmt == "hex":
        return "\n".join(f"{w:016x}" for w in insts) + "\n"
    if fmt == "words32":
        lines = []
        for w in insts:
            lines.append(f"{w & 0xFFFFFFFF:08x}")
            lines.append(f"{(w >> 32) & 0xFFFFFFFF:08x}")
        return "\n".join(lines) + "\n"
    if fmt == "c":
        body = ",\n".join(f"  0x{w:016X}ULL" for w in insts)
        return f"static const uint64_t {sym_name}[{len(insts)}] = {{\n{body}\n}};\n"
    if fmt == "sv":
        body = ",\n".join(f"  64'h{w:016X}" for w in insts)
        return f"localparam logic [63:0] {sym_name} [0:{len(insts)-1}] = '{{\n{body}\n}};\n"
    raise ValueError(f"Unsupported output format '{fmt}'")


# -----------------------------------------------------------------------------
# Built-in Unit Self-Test Covering All 12 Opcodes (4'h0..4'hB)
# -----------------------------------------------------------------------------
def run_self_test() -> int:
    demo_asm = """
    // Exercise all 12 opcodes 4'h0..4'hB + v2.0 precisions
    NOP
    CFG        BF16, RELU, OUT_BF16
    CFG_GSCALE INT4_SCALED, NONE, FP32, 1.0f
    DMA_LD     0, ABUF, 0, 1, 16, 0, 0
    DMA_LD_BG  1, WBUF, 0, 1, 16, 0, 0
    SYNC_BANK  1, 1, 1
    CONV2D     1, 3, 3, 1, 1, 4, 4, 1, 1, 0, 0, 0, 0
    POOL       MAX, 2, 2, 0, 0, FP32, 0, 4, 4, 4, 0, 256
    LOOP_INIT  0, 2
loop_body:
    GEMM       1, 1, 1, 1, 0, 0, 0, 0
    SFU        0, 1, 1, 4, 0, 128, 0, 16, 0
    VEC_EW     RMSNORM, 0, 1, 1, FP32, 4, 16, 128, 32, 0, 0
    VEC_EW     SOFTMAX, 1, 0, 0, BF16, 4, 4, 0, 0, 256, 0
    LOOP_NEXT  0, loop_body
    DMA_ST     0, OBUF, 256, 1, 8, 0xFFF, 0
    BREAK
    HALT       0
    """
    insts = assemble(demo_asm)
    assert len(insts) == 17, f"Expected 17 insts, got {len(insts)}"
    opcodes = [(w >> 60) & 0xF for w in insts]
    expected_ops = [0x0, 0x1, 0x1, 0x2, 0x2, 0xB, 0x8, 0x9, 0x6, 0x4, 0x5, 0xA, 0xA, 0x6, 0x3, 0x7, 0x7]
    assert opcodes == expected_ops, f"Opcode mismatch: {[hex(x) for x in opcodes]}"
    # Verify CFG_GSCALE INT4_SCALED set prec_ext=1, int4_scale_en=1, set_gscale=1, 1.0f=0x3F800000
    cfg_gs = insts[2]
    assert ((cfg_gs >> 50) & 1) == 1
    assert ((cfg_gs >> 49) & 1) == 1
    assert ((cfg_gs >> 48) & 1) == 1
    assert (cfg_gs & 0xFFFFFFFF) == 0x3F800000
    # Verify LOOP_NEXT resolved label `loop_body` -> PC 9
    assert ((insts[13] >> 16) & 0xFF) == 9
    # Verify DMA_LD_BG set async_bg bit [55]
    assert ((insts[4] >> 55) & 1) == 1
    # Verify both C and hex formatters work
    c_out = format_output(insts, "c", "v2_demo")
    sv_out = format_output(insts, "sv", "v2_demo")
    assert "0x" in c_out and "64'h" in sv_out
    print(f"[TPU-ASM SELF-TEST OK] Assembled {len(insts)} instructions covering all 12 opcodes (0x0..0xB).")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Mini-TPU / NPU v2.0 64-bit Microcode Assembler")
    parser.add_argument("input", nargs="?", help="Input assembly file (.asm / .s)")
    parser.add_argument("-f", "--format", choices=("hex", "words32", "c", "sv"), default="hex")
    parser.add_argument("-n", "--name", default="tpu_prog", help="Array symbol name for C/SV output")
    parser.add_argument("-o", "--output", help="Output file (default: stdout)")
    parser.add_argument("--self-test", action="store_true", help="Run internal assembler self-test")
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()
    if not args.input:
        parser.error("Provide an input file or --self-test")

    with open(args.input, "r", encoding="utf-8") as fh:
        insts = assemble(fh.read())
    txt = format_output(insts, args.format, args.name)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as fh:
            fh.write(txt)
    else:
        sys.stdout.write(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
