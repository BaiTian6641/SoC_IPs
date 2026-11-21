#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Static Yosys / OpenLane / ASIC synthesis compatibility checker.

Scans RTL files under `common/`, `interconnect/`, and `ips/*/rtl/` for
constructs that break or degrade Yosys, OpenLane, FPGA BRAM inference, or ASIC
synthesis flows:
  1. DELAY_IN_RTL      : `#` timing delays (`#5`, `#1ns`, procedural `#(...)`)
  2. REAL_TYPE         : `real`, `shortreal`, `realtime` types
  3. INITIAL_BLOCK     : `initial` procedural blocks in RTL
  4. TRISTATE_LITERAL  : High-Z `'bz` / `'hz` / `'z` literals inside digital cores
  5. ASYNC_RST_MEM     : Async-reset `for` loops clearing memory arrays
  6. UNSYNTH_SYSTASK   : Unsynthesizable system tasks (`$display`, `$finish`,
                         `$random`, `$readmem*`, `$fopen`, `$fatal`, `$error`, ...)

Usage:
    python3 scripts/check_yosys_compat.py
    python3 scripts/check_yosys_compat.py --verbose
    python3 scripts/check_yosys_compat.py --strict
    python3 scripts/check_yosys_compat.py --root /path/to/repo
"""

import argparse
import glob
import os
import re
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


# Documented benign exceptions in existing baseline IPs.
# Each entry maps (rel_path, rule) -> justification string.
DOCUMENTED_EXCEPTIONS: Dict[Tuple[str, str], str] = {
    ("ips/18_rmt/rtl/rmt_core.sv", "UNSYNTH_SYSTASK"):
        "Elaboration-time generate-if parameter guard ($error) outside procedural logic",
    ("ips/24_timer_pwm/rtl/timer_pwm_core.sv", "UNSYNTH_SYSTASK"):
        "Elaboration-time generate-if parameter guard ($error) outside procedural logic",
    ("ips/25_plic_clint/rtl/clint_core.sv", "UNSYNTH_SYSTASK"):
        "Elaboration-time generate-if parameter guard ($fatal) outside procedural logic",
    ("ips/25_plic_clint/rtl/plic_core.sv", "UNSYNTH_SYSTASK"):
        "Elaboration-time generate-if parameter guard ($fatal) outside procedural logic",
}

UNSYNTH_TASKS = (
    "display", "write", "strobe", "monitor",
    "finish", "stop", "fatal", "error", "warning", "info",
    "random", "urandom", "urandom_range",
    "readmemh", "readmemb", "writememh", "writememb",
    "fopen", "fclose", "fwrite", "fdisplay", "fscanf", "fgets",
    "time", "stime", "realtime",
    "dumpfile", "dumpvars", "dumpon", "dumpoff",
    "value$plusargs", "test$plusargs",
    "isunknown",
)

UNSYNTH_TASK_RE = re.compile(
    r"\$(?:" + "|".join(re.escape(t) for t in UNSYNTH_TASKS) + r")\b"
)
DELAY_NUM_RE = re.compile(r"#\s*\d")
DELAY_PAREN_RE = re.compile(r"#\s*\(")
PROC_START_RE = re.compile(r"\b(?:always_ff|always_comb|always_latch|always|task|function)\b")
REAL_RE = re.compile(r"\b(?:shortreal|realtime|real)\b")
INITIAL_RE = re.compile(r"\binitial\b")
TRISTATE_RE = re.compile(r"(?:\b\d*'[bBhHoOdD][0-9a-fA-FxX_]*[zZ][0-9a-fA-FxX_]*\b|'[zZ]\b)")

MEM_NAME_RE = re.compile(
    r"(?:mem|sram|bram|dram|spram|dpram|ram|rom|fifo|lut|imem|dmem|cache)",
    re.IGNORECASE,
)


@dataclass
class Finding:
    rel_path: str
    line: int
    rule: str
    detail: str
    exempt: bool = False
    exempt_reason: str = ""


def strip_comments_and_strings_preserve_lines(text: str) -> Tuple[str, List[str]]:
    """Return (sanitized_text, raw_lines) preserving line numbers.

    Replaces string literals with `""` and comments with spaces (preserving
    newlines) while recording inline `// yosys-compat: allow <RULE>` pragmas.
    """
    raw_lines = text.splitlines()
    out: List[str] = []
    i = 0
    n = len(text)
    in_block_comment = False
    in_line_comment = False
    in_string = False

    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""

        if in_block_comment:
            if ch == "*" and nxt == "/":
                out.append("  ")
                i += 2
                in_block_comment = False
            elif ch == "\n":
                out.append("\n")
                i += 1
            else:
                out.append(" ")
                i += 1
        elif in_line_comment:
            if ch == "\n":
                out.append("\n")
                in_line_comment = False
            else:
                out.append(" ")
            i += 1
        elif in_string:
            if ch == "\\" and i + 1 < n:
                out.append("  ")
                i += 2
            elif ch == '"':
                out.append('"')
                in_string = False
                i += 1
            elif ch == "\n":
                out.append("\n")
                in_string = False
                i += 1
            else:
                out.append(" ")
                i += 1
        else:
            if ch == "/" and nxt == "*":
                out.append("  ")
                i += 2
                in_block_comment = True
            elif ch == "/" and nxt == "/":
                out.append("  ")
                i += 2
                in_line_comment = True
            elif ch == '"':
                out.append('"')
                in_string = True
                i += 1
            else:
                out.append(ch)
                i += 1

    return "".join(out), raw_lines


def compute_synth_active_lines(lines: List[str]) -> List[bool]:
    """Track `ifndef SYNTHESIS / `ifdef SIMULATION blocks per line."""
    active = []
    stack: List[bool] = []  # True = suppressed (simulation-only)

    for line in lines:
        s = line.strip()
        if s.startswith("`ifndef"):
            macro = s.split()[1] if len(s.split()) > 1 else ""
            stack.append(macro == "SYNTHESIS")
        elif s.startswith("`ifdef"):
            macro = s.split()[1] if len(s.split()) > 1 else ""
            stack.append(macro in ("SIMULATION", "VERILATOR_SIM", "FORMAL_SIM"))
        elif s.startswith("`else"):
            if stack:
                stack[-1] = not stack[-1]
        elif s.startswith("`elsif"):
            if stack:
                macro = s.split()[1] if len(s.split()) > 1 else ""
                stack[-1] = macro in ("SIMULATION", "VERILATOR_SIM", "FORMAL_SIM")
        elif s.startswith("`endif"):
            if stack:
                stack.pop()
        active.append(not any(stack))

    return active


def check_async_reset_mem_loops(
    rel_path: str, clean_text: str, active_lines: List[bool]
) -> List[Finding]:
    """Detect async-reset `for` loops clearing memory-like unpacked arrays."""
    findings: List[Finding] = []

    # Collect unpacked array declarations: logic [W-1:0] name [D];
    mem_arrays: Dict[str, int] = {}
    decl_re = re.compile(
        r"\b(?:logic|reg|bit)\s*(?:\[[^\]]+\]\s*)+([a-zA-Z_]\w*)\s*\[([^\]]+)\]\s*;"
    )
    for m in decl_re.finditer(clean_text):
        arr_name = m.group(1)
        dim_expr = m.group(2).strip()
        is_mem_name = bool(MEM_NAME_RE.search(arr_name))
        depth_val = 0
        if re.fullmatch(r"\d+", dim_expr):
            depth_val = int(dim_expr)
        elif re.fullmatch(r"0\s*:\s*\d+", dim_expr):
            hi = int(dim_expr.split(":")[1].strip())
            depth_val = hi + 1
        if is_mem_name or depth_val >= 32:
            mem_arrays[arr_name] = depth_val

    if not mem_arrays:
        return findings

    # Scan each async-reset always_ff block up to its matching `end`
    aff_re = re.compile(
        r"\balways_ff\s*@\s*\([^)]*(?:\bor\b|,)[^)]*(?:negedge|posedge)[^)]*\)\s*begin\b"
    )
    tok_re = re.compile(r"\b(begin|end)\b")

    for m in aff_re.finditer(clean_text):
        start_pos = m.end()
        depth = 1
        end_pos = len(clean_text)
        for tm in tok_re.finditer(clean_text, start_pos):
            if tm.group(1) == "begin":
                depth += 1
            else:
                depth -= 1
                if depth == 0:
                    end_pos = tm.start()
                    break

        block = clean_text[start_pos:end_pos]
        line_no = clean_text[: m.start()].count("\n") + 1
        if line_no - 1 < len(active_lines) and not active_lines[line_no - 1]:
            continue

        # Look for `if (!rst...)` containing a `for (...)` loop writing `arr[...] <=`
        rst_if_re = re.compile(r"\bif\s*\(\s*!\s*[a-zA-Z_]*rst[a-zA-Z_0-9]*\s*\)\s*begin\b")
        for rm in rst_if_re.finditer(block):
            r_start = rm.end()
            r_depth = 1
            r_end = len(block)
            for tm in tok_re.finditer(block, r_start):
                if tm.group(1) == "begin":
                    r_depth += 1
                else:
                    r_depth -= 1
                    if r_depth == 0:
                        r_end = tm.start()
                        break
            rst_body = block[r_start:r_end]
            for arr_name in mem_arrays:
                loop_pat = re.compile(
                    rf"\bfor\s*\([^)]+\)[\s\S]*?\b{re.escape(arr_name)}\s*\[[^\]]+\]\s*<="
                )
                lm = loop_pat.search(rst_body)
                if lm:
                    abs_off = start_pos + r_start + lm.start()
                    f_line = clean_text[:abs_off].count("\n") + 1
                    findings.append(
                        Finding(
                            rel_path=rel_path,
                            line=f_line,
                            rule="ASYNC_RST_MEM",
                            detail=(
                                f"Async-reset for-loop writes memory array `{arr_name}` "
                                "(blocks FPGA BRAM / ASIC SRAM inference)"
                            ),
                        )
                    )
    return findings


def scan_file(root: str, abs_path: str) -> List[Finding]:
    rel_path = os.path.relpath(abs_path, root).replace(os.sep, "/")
    with open(abs_path, "r", encoding="utf-8") as fh:
        raw_text = fh.read()

    clean_text, raw_lines = strip_comments_and_strings_preserve_lines(raw_text)
    clean_lines = clean_text.splitlines()
    active_lines = compute_synth_active_lines(clean_lines)

    findings: List[Finding] = []
    in_proc = False
    proc_depth = 0
    proc_tok_re = re.compile(r"\b(begin|end|endtask|endfunction)\b")

    for idx, line in enumerate(clean_lines):
        line_no = idx + 1
        if not active_lines[idx]:
            continue

        if PROC_START_RE.search(line):
            in_proc = True
            proc_depth = 0
        if in_proc:
            for tm in proc_tok_re.finditer(line):
                tok = tm.group(1)
                if tok == "begin":
                    proc_depth += 1
                elif tok in ("endtask", "endfunction"):
                    in_proc = False
                    proc_depth = 0
                elif tok == "end":
                    proc_depth = max(0, proc_depth - 1)
                    if proc_depth == 0:
                        in_proc = False
            if proc_depth == 0 and ";" in line and not PROC_START_RE.search(line):
                in_proc = False

        raw_line = raw_lines[idx] if idx < len(raw_lines) else ""
        pragma_allow: Optional[str] = None
        pm = re.search(r"yosys-compat:\s*allow\s+([A-Z0-9_]+)", raw_line)
        if pm:
            pragma_allow = pm.group(1)

        delay_match = DELAY_NUM_RE.search(line) or (DELAY_PAREN_RE.search(line) if in_proc else None)

        checks = [
            ("DELAY_IN_RTL", delay_match,
             "Timing delay `#` found in synthesizable RTL"),
            ("REAL_TYPE", REAL_RE.search(line),
             "`real`/`shortreal`/`realtime` type found in synthesizable RTL"),
            ("INITIAL_BLOCK", INITIAL_RE.search(line),
             "`initial` block found in synthesizable RTL"),
            ("TRISTATE_LITERAL", TRISTATE_RE.search(line),
             "High-Z tri-state literal (`'bz`/`'hz`/`'z`) inside digital RTL"),
            ("UNSYNTH_SYSTASK", UNSYNTH_TASK_RE.search(line),
             "Unsynthesizable system task in RTL"),
        ]

        for rule, match, msg in checks:
            if not match:
                continue
            token = match.group(0).strip()
            detail = f"{msg}: `{token}`"
            if pragma_allow == rule:
                findings.append(
                    Finding(rel_path, line_no, rule, detail, exempt=True,
                            exempt_reason="Inline yosys-compat pragma")
                )
            elif (rel_path, rule) in DOCUMENTED_EXCEPTIONS:
                findings.append(
                    Finding(rel_path, line_no, rule, detail, exempt=True,
                            exempt_reason=DOCUMENTED_EXCEPTIONS[(rel_path, rule)])
                )
            else:
                findings.append(Finding(rel_path, line_no, rule, detail))

    for f in check_async_reset_mem_loops(rel_path, clean_text, active_lines):
        if (rel_path, f.rule) in DOCUMENTED_EXCEPTIONS:
            f.exempt = True
            f.exempt_reason = DOCUMENTED_EXCEPTIONS[(rel_path, f.rule)]
        findings.append(f)

    return findings


def collect_rtl_files(root: str) -> List[str]:
    patterns = [
        os.path.join(root, "common", "**", "*.sv"),
        os.path.join(root, "common", "**", "*.svh"),
        os.path.join(root, "interconnect", "**", "*.sv"),
        os.path.join(root, "ips", "*", "rtl", "*.sv"),
    ]
    files = set()
    for pat in patterns:
        for path in glob.glob(pat, recursive=True):
            if os.path.isfile(path):
                files.add(os.path.abspath(path))
    return sorted(files)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Static Yosys / OpenLane / ASIC synthesis compatibility checker."
    )
    parser.add_argument(
        "--root",
        default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        help="Repository root directory",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Print every scanned file and documented exception details",
    )
    parser.add_argument(
        "--strict", action="store_true",
        help="Treat documented benign exceptions as errors",
    )
    args = parser.parse_args()

    root = os.path.abspath(args.root)
    files = collect_rtl_files(root)
    if not files:
        print(f"[ERROR] No RTL files found under {root}", file=sys.stderr)
        return 2

    all_findings: List[Finding] = []
    for path in files:
        if args.verbose:
            print(f"[SCAN] {os.path.relpath(path, root)}")
        all_findings.extend(scan_file(root, path))

    errors = [f for f in all_findings if not f.exempt]
    exempts = [f for f in all_findings if f.exempt]

    rule_counts: Dict[str, int] = {}
    for f in errors:
        rule_counts[f.rule] = rule_counts.get(f.rule, 0) + 1

    if exempts:
        print(f"Documented benign exceptions ({len(exempts)}):")
        for f in exempts:
            print(f"  [EXEMPT] {f.rel_path}:{f.line} [{f.rule}] {f.detail} -- {f.exempt_reason}")

    if errors:
        print(f"\nYosys/ASIC compatibility violations ({len(errors)}):", file=sys.stderr)
        for f in errors:
            print(f"  [ERROR]  {f.rel_path}:{f.line} [{f.rule}] {f.detail}", file=sys.stderr)
        if args.strict and exempts:
            for f in exempts:
                print(f"  [STRICT] {f.rel_path}:{f.line} [{f.rule}] {f.detail}", file=sys.stderr)
        return 1

    if args.strict and exempts:
        print(f"[FAIL] --strict enabled and {len(exempts)} exception(s) present", file=sys.stderr)
        return 1

    print(
        f"[YOSYS-COMPAT OK] Scanned {len(files)} RTL files across common/, interconnect/, "
        f"ips/*/rtl/: 0 errors, {len(exempts)} documented benign exception(s)."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
