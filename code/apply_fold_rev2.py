#!/usr/bin/env python3
"""apply_fold_rev2.py -- run from code/ AFTER apply_fold.py: raw snapshot width 159 -> 175
(fold 136 taps, counting 168+30j). Each edit must match exactly once."""
import sys
def patch(path, edits):
    s = open(path, encoding='utf-8', newline='').read()
    for i, (old, new) in enumerate(edits, 1):
        n = s.count(old)
        if n != 1:
            sys.exit(f"{path}: edit {i} matched {n} times" + (" (looks ALREADY APPLIED)" if s.count(new) else "") + " -- aborting")
        s = s.replace(old, new)
    open(path, 'w', encoding='utf-8', newline='').write(s)
    print(f"{path}: {len(edits)} edits applied")
patch('tdc_channel.v', [("    parameter integer RAW_W        = 159", "    parameter integer RAW_W        = 175")])
patch('tdc_dual_top.v', [("    parameter integer RAW_W          = 159   // raw snapshot width (FOLD: 32+120+7)",
                          "    parameter integer RAW_W          = 175   // raw snapshot width (FOLD rev 2: 32+136+7)")])
patch('tdc_dual_board.v', [("    localparam integer RAW_W       = 159;   // raw snapshot width carried to the DUMP path",
                            "    localparam integer RAW_W       = 175;   // raw snapshot width carried to the DUMP path")])
