#!/usr/bin/env python3
"""apply_step4.py -- dead-zone fix plumbing (DUAL_SNAP). Aborts before writing if anything is off."""
import re, sys

def patch(path, edits):
    s = open(path, encoding='utf-8').read()
    if 'DUAL_SNAP' in s:
        sys.exit(f"{path}: already contains DUAL_SNAP -- looks ALREADY APPLIED, nothing written")
    for i, (pat, rep, want) in enumerate(edits, 1):
        s, n = re.subn(pat, rep, s)
        if n != want:
            sys.exit(f"{path}: edit {i} matched {n}x (expected {want}) -- aborting, nothing written")
    return s

ch = patch('tdc_channel.v', [
    (r"(parameter\s+integer\s+SYNC_TAP\s*=\s*30)(\s*\)\s*\()",
     r"\1,\n    // DUAL_SNAP -- dead-zone fix (step 4): if the chosen edge is already full,\n"
     r"    // take the previous edge's taps AND coarse. 0 = old behaviour.\n"
     r"    parameter integer DUAL_SNAP    = 1\2", 1),
    (r"(wire \[TDL_WIDTH-1:0\]\s+sampled_taps;)",
     r"\1\n    wire                   tap_full;\n"
     r"    wire                   use_prev = (DUAL_SNAP != 0) && tap_full;", 1),
    (r"\.captured \(sampled_taps\)",
     r".captured (sampled_taps),\n        .use_prev (use_prev), .top_now (tap_full)", 1),
    (r"\.captured \(coarse_out\)",
     r".captured (coarse_out),\n        .use_prev (use_prev), .top_now ()", 1),
])
top = patch('tdc_dual_top.v', [
    (r"(parameter\s+integer\s+SYNC_TAP\s*=\s*30)([ \t]*//[^\n]*)(\s*\)\s*\()",
     r"\1,\2\n    parameter integer DUAL_SNAP      = 1     // step 4 dead-zone fix\3", 1),
    (r"\.SYNC_TAP\(SYNC_TAP\),", r".SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP),", 2),
])
brd = patch('tdc_dual_board.v', [
    (r"(parameter\s+integer\s+RO_DIV_BITS\s*=\s*10)([ \t]*//[^\n]*)",
     r"\1,\2\n    parameter integer DUAL_SNAP        = 1    // step 4: dead-zone fix, 0 = old capture", 1),
    (r"\.SYNC_TAP\(SYNC_TAP\), \.PHASE_BITS\(PHASE_BITS\)",
     r".SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP), .PHASE_BITS(PHASE_BITS)", 1),
])
for p, s in (('tdc_channel.v', ch), ('tdc_dual_top.v', top), ('tdc_dual_board.v', brd)):
    open(p, 'w', encoding='utf-8', newline='\n').write(s)
    print(f"{p}: patched")