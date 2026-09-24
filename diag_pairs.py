#!/usr/bin/env python3
"""diag_pairs.py capture.csv -- timeout signature and d_coarse pattern of a tied-channel capture."""
import csv, sys, collections
rows = list(csv.DictReader(open(sys.argv[1], newline='')))
I = lambda r, k: int(r[k])
n = len(rows)
to = [r for r in rows if I(r, 'fine_b') == 0 and I(r, 'valid_b') == 0 and I(r, 'd_coarse') == 0]
print(f"{n} frames | timeout signature (fb=0, vb=0, dc=0): {len(to)} = {100 * len(to) / n:.2f} %")
dc = collections.Counter()
for r in rows:
    if I(r, 'valid_a') and I(r, 'valid_b'):
        d = I(r, 'd_coarse'); d = d - 16384 if d >= 8192 else d
        dc[d] += 1
print("d_coarse over valid pairs:", dict(sorted(dc.items())))
fa = collections.Counter(I(r, 'fine_a') for r in to)
if fa:
    ks = sorted(fa)
    print(f"fine_a during timeouts: {ks[0]}..{ks[-1]} | most common: {fa.most_common(8)}")