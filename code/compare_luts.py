#!/usr/bin/env python3
"""compare_luts.py A_prefix B_prefix [--range LO HI] -- bin-width agreement of two LUTs, per chain."""
import csv, math, argparse

def load(p):
    return {int(r['code']): float(r['bin_width_ps']) for r in csv.DictReader(open(p, newline=''))}

def cmp(pa, pb, rng):
    a, b = load(pa), load(pb)
    common = [c for c in sorted(set(a) & set(b)) if rng[0] <= c <= rng[1]]
    x = [a[c] for c in common]; y = [b[c] for c in common]
    mx, my = sum(x) / len(x), sum(y) / len(y)
    cor = (sum((p - mx) * (q - my) for p, q in zip(x, y)) /
           math.sqrt(sum((p - mx) ** 2 for p in x) * sum((q - my) ** 2 for q in y)))
    k = sum(x) / sum(y)
    d = [p - k * q for p, q in zip(x, y)]
    rms = math.sqrt(sum(v * v for v in d) / len(d))
    acc, cum = 0.0, []
    for v in d:
        acc += v; cum.append(acc)
    print(f"{pa} vs {pb}: {len(common)} codes | corr {cor:.3f} | per-bin rms {rms:.2f} ps"
          f" | scale {100 * (k - 1):+.2f} % | shape {min(cum):+.0f}/{max(cum):+.0f} ps")
    top = sorted(zip(common, d, x, y), key=lambda t: -abs(t[1]))[:8]
    print("      largest per-bin changes (code: this / ref ps):  " +
          "  ".join(f"{c}:{p:.0f}/{q:.0f}" for c, _, p, q in top))
    share = sum(v * v for _, v, _, _ in top) / sum(v * v for v in d)
    print(f"      top-8 bins carry {100 * share:.0f} % of the total squared difference")

ap = argparse.ArgumentParser()
ap.add_argument('a'); ap.add_argument('b')
ap.add_argument('--range', nargs=2, type=int, default=[110, 340])
g = ap.parse_args()
for ch in 'ab':
    cmp(f"{g.a}_{ch}.csv", f"{g.b}_{ch}.csv", g.range)