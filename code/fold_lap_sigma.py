#!/usr/bin/env python3
"""fold_lap_sigma.py -- where does the folded TDC's extra noise come from?

Uses a LUT pair from code_density.py and an evaluation capture (read_tdc.py
CSV). Pairs A/B like calib_report (|d_coarse| <= 1), subtracts the median
A-B offset, then breaks the residual down:
  1. by (lap_a, lap_b): sigma and outlier rate per lap pair. Growth with lap
     number = jitter added per pass through the return loop.
  2. by chain, by lap: sigma of the pair residual restricted to pairs where
     the OTHER chain sits in lap 0 (its cleanest laps) so each chain's laps
     can be compared on their own.
  3. the widest bins per chain with their code, lap and position.
Run from the repo root:
    python code\\fold_lap_sigma.py --lut-a data\\lut_fold_ro7_a.csv --lut-b data\\lut_fold_ro7_b.csv --eval data\\raw\\fold_x_ro11.csv
"""
import argparse, csv, math
from collections import defaultdict
import fold_model as fm

P, LAUNCH_W, FOLD_W = 5000.0, fm.LAUNCH_W, fm.FOLD_W


def load_lut(path):
    lut, w = {}, {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            lut[int(r["code"])] = float(r["t_ps"]); w[int(r["code"])] = float(r["bin_width_ps"])
    return lut, w


def lap_of(code):
    return -1 if code < LAUNCH_W else (code - LAUNCH_W) // FOLD_W


def stats(v):
    n = len(v)
    if n < 2:
        return n, float("nan"), float("nan"), 0.0
    m = sum(v) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / (n - 1))
    core = [x for x in v if abs(x) <= 200]
    mc = sum(core) / len(core) if core else float("nan")
    sdc = math.sqrt(sum((x - mc) ** 2 for x in core) / (len(core) - 1)) if len(core) > 1 else float("nan")
    return n, sdc, mc, 100.0 * (n - len(core)) / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lut-a", required=True); ap.add_argument("--lut-b", required=True)
    ap.add_argument("--eval", required=True)
    a = ap.parse_args()
    la, wa = load_lut(a.lut_a); lb, wb = load_lut(a.lut_b)
    res = []
    with open(a.eval, newline="") as fh:
        for r in csv.DictReader(fh):
            if not (int(r["valid_a"]) and int(r["valid_b"])):
                continue
            dc = int(r["d_coarse"]); dc = dc - 16384 if dc >= 8192 else dc
            if abs(dc) > 1:
                continue
            fa, fb = int(r["fine_a"]), int(r["fine_b"])
            if fa not in la or fb not in lb:
                continue
            res.append((fa, fb, dc * P + lb[fb] - la[fa]))
    if not res:
        raise SystemExit("no usable pairs")
    off = sorted(x[2] for x in res)[len(res) // 2]
    res = [(fa, fb, t - off) for fa, fb, t in res]
    n, sd, m, out = stats([t for _, _, t in res])
    print(f"{n} pairs, A-B offset {off:+.1f} ps, sigma_pair {sd:.2f} ps (|res|<=200), outliers >200 ps {out:.3f}%")

    print("\n1. by (lap_a, lap_b)   [-1 = front still in launch section]")
    g = defaultdict(list)
    for fa, fb, t in res:
        g[(lap_of(fa), lap_of(fb))].append(t)
    print(f"  {'lap_a':>5} {'lap_b':>5} {'n':>8} {'sigma ps':>9} {'mean ps':>8} {'outl %':>7}")
    for k in sorted(g):
        n, sd, m, out = stats(g[k])
        if n >= 200:
            print(f"  {k[0]:5d} {k[1]:5d} {n:8d} {sd:9.2f} {m:8.2f} {out:7.3f}")

    print("\n2. each chain's laps, other chain held in lap 0")
    for name, idx, other in (("A", 0, 1), ("B", 1, 0)):
        g = defaultdict(list)
        for row in res:
            if lap_of(row[other]) == 0:
                g[lap_of(row[idx])].append(row[2])
        for lap in sorted(g):
            n, sd, m, out = stats(g[lap])
            if n >= 200:
                print(f"  chain {name} lap {lap:2d}: n {n:7d}  sigma {sd:6.2f} ps  outliers {out:.3f}%")

    print("\n3. outliers (|res| > 200 ps) by (d_coarse, lap_a, lap_b): count, typical residual")
    g = defaultdict(list)
    for fa, fb, t in res:
        if abs(t) > 200:
            g[(0, lap_of(fa), lap_of(fb))].append(t)      # d_coarse already folded into t; key by laps
    for k in sorted(g, key=lambda k: -len(g[k]))[:8]:
        v = sorted(g[k]); med = v[len(v) // 2]
        print(f"  laps ({k[1]},{k[2]}): {len(v):6d}  median residual {med:+8.0f} ps")

    print("\n4. widest bins (code, lap, pos, width)")
    for name, w in (("A", wa), ("B", wb)):
        top = sorted(w.items(), key=lambda kv: -kv[1])[:6]
        print("  chain " + name + ": " + ", ".join(
            f"{c} (lap {lap_of(c)} pos {(c - LAUNCH_W) % FOLD_W if c >= LAUNCH_W else c}) {x:.0f} ps" for c, x in top))


if __name__ == "__main__":
    main()
