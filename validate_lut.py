#!/usr/bin/env python3
"""
validate_lut.py -- does the calibration LUT actually buy anything?

    python validate_lut.py sweep_full.csv --lut lut

Building a LUT is easy. Proving it improves the measurement is the part that
belongs in the paper, and it is the part that is easy to fake by grading the
LUT on the data it was built from. Two tests below; the second is the honest
one.

TEST 1 -- RESIDUAL AGAINST KNOWN PHASE
    Every sample has a KNOWN arrival time: phase x 17.857 ps. So the error of
    any code->time map can be measured directly.
      linear   t = (C0 - code) x tau        one number for the whole chain
      LUT      t = lut[code]                per-bin
    Report RMS residual for each. If the LUT does not beat the linear fit, it
    is not doing anything and you should say so rather than shipping it.

    CAVEAT, AND IT MATTERS: run this on the SAME file the LUT was built from
    and the LUT is being graded on its training data. It will look better than
    it is. Use --lut built from a DIFFERENT capture, or --split to hold out
    half the data. --split is on by default for exactly this reason.

TEST 2 -- DIFFERENTIAL A MINUS B
    Both chains see the SAME cal edge, so the true interval is a constant.
    Any spread in (t_a - t_b) is instrument noise, and it needs no reference
    and no assumption about the phase axis -- it cannot be inflated by a good
    LUT fitting its own training set, because a constant is a constant.

        sigma_pair = std(t_a - t_b)
        sigma_single = sigma_pair / sqrt(2)     (chains independent, similar)

    This is the number to quote as single-shot precision.
"""

import csv
import math
import argparse
from collections import defaultdict

PHASE_STEP_PS = 1000.0 / 56.0
SWEEP_STEPS = 280
PERIOD_PS = SWEEP_STEPS * PHASE_STEP_PS
TDL_TAPS = 352


def wrap(d):
    """Wrap a time difference into (-period/2, +period/2]."""
    d = math.fmod(d, PERIOD_PS)
    if d > PERIOD_PS / 2:
        d -= PERIOD_PS
    elif d <= -PERIOD_PS / 2:
        d += PERIOD_PS
    return d


def load_lut(path, min_conc):
    """code -> t_ps, dropping codes whose phase distribution is not concentrated."""
    lut, dropped = {}, []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            c = int(r['code'])
            k = float(r.get('concentration', 1.0))
            if k < min_conc:
                dropped.append(c)
                continue
            lut[c] = float(r['t_ps'])
    return lut, dropped


def fill_gaps(lut):
    """
    Zero-width codes have no LUT entry. A sample can still legitimately carry
    such a code (jitter), so interpolate rather than discarding the sample --
    discarding would quietly bias the residual toward the well-populated bins
    and flatter the result.
    """
    if not lut:
        return lut
    ks = sorted(lut)
    out = dict(lut)
    for c in range(ks[0], ks[-1] + 1):
        if c in out:
            continue
        lo = max(k for k in ks if k < c)
        hi = min(k for k in ks if k > c)
        f = (c - lo) / (hi - lo)
        a, b = lut[lo], lut[hi]
        d = wrap(b - a)                      # interpolate the short way round
        out[c] = (a + f * d) % PERIOD_PS
    return out


def fit_linear(samples):
    """
    Least-squares t = C0 + s*code on circular data, done by unwrapping against
    a first-pass estimate. samples: list of (code, t_true).
    """
    if len(samples) < 2:
        return 0.0, 0.0
    n = len(samples)
    mc = sum(c for c, _ in samples) / n
    mt = sum(t for _, t in samples) / n
    num = sum((c - mc) * wrap(t - mt) for c, t in samples)
    den = sum((c - mc) ** 2 for c, _ in samples)
    s = num / den if den else 0.0
    c0 = mt - s * mc
    return c0, s


def rms(xs):
    if not xs:
        return float('nan')
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / len(xs))


def main():
    ap = argparse.ArgumentParser(description="Validate a TDC calibration LUT")
    ap.add_argument("csv")
    ap.add_argument("--lut", default="lut", help="prefix -> <p>_a.csv, <p>_b.csv")
    ap.add_argument("--min-conc", type=float, default=0.5,
                    help="drop LUT codes below this circular concentration")
    ap.add_argument("--split", action="store_true", default=True,
                    help="hold out odd-numbered samples for scoring (default)")
    ap.add_argument("--no-split", dest="split", action="store_false")
    a = ap.parse_args()

    luts = {}
    for ch, nm in ((0, 'a'), (1, 'b')):
        raw, dropped = load_lut(f"{a.lut}_{nm}.csv", a.min_conc)
        if dropped:
            print(f"chain {nm.upper()}: dropped {len(dropped)} low-concentration "
                  f"codes: {dropped[:12]}{' ...' if len(dropped) > 12 else ''}")
        luts[ch] = fill_gaps(raw)
        print(f"chain {nm.upper()}: LUT covers {len(luts[ch])} codes "
              f"({len(raw)} measured, {len(luts[ch]) - len(raw)} interpolated)")

    rows = []
    with open(a.csv, newline='') as fh:
        for i, r in enumerate(csv.DictReader(fh)):
            fa, fb = int(r['fine_a']), int(r['fine_b'])
            if fa in (0, TDL_TAPS) or fb in (0, TDL_TAPS):
                continue
            if not (int(r['valid_a']) and int(r['valid_b'])):
                continue
            rows.append((i, int(r['phase']), fa, fb))
    print(f"\nloaded {len(rows)} usable samples from {a.csv}")

    score = [r for r in rows if (r[0] % 2 == 1)] if a.split else rows
    train = [r for r in rows if (r[0] % 2 == 0)] if a.split else rows
    if a.split:
        print(f"held out {len(score)} samples for scoring "
              f"(LUT graded on data it did not see)\n")

    # ---------------- TEST 1 ----------------
    print("=" * 70)
    print("  TEST 1 -- residual against known phase")
    print("=" * 70)
    for ch, nm in ((0, 'A'), (1, 'B')):
        fi = 2 + ch
        tr = [(r[fi], r[1] * PHASE_STEP_PS) for r in train]
        c0, s = fit_linear(tr)
        print(f"  chain {nm}: linear fit  t = {c0:.1f} {s:+.3f} x code  "
              f"-> tau = {abs(s):.2f} ps/tap")

        res_lin, res_lut, missing = [], [], 0
        for r in score:
            code, t_true = r[fi], r[1] * PHASE_STEP_PS
            res_lin.append(wrap((c0 + s * code) - t_true))
            if code in luts[ch]:
                res_lut.append(wrap(luts[ch][code] - t_true))
            else:
                missing += 1
        print(f"      RMS residual  linear : {rms(res_lin):7.2f} ps")
        print(f"      RMS residual  LUT    : {rms(res_lut):7.2f} ps"
              f"   ({100 * (1 - rms(res_lut) / rms(res_lin)):+.1f} %)")
        if missing:
            print(f"      samples with no LUT entry: {missing}")

    # ---------------- TEST 2 ----------------
    print()
    print("=" * 70)
    print("  TEST 2 -- differential A-B  (no reference needed)")
    print("=" * 70)
    print("  Both chains see the same edge, so the true interval is CONSTANT.")
    print("  All spread here is instrument noise. A LUT cannot inflate this by")
    print("  fitting its training data, because a constant stays constant.")

    tra = [(r[2], r[1] * PHASE_STEP_PS) for r in train]
    trb = [(r[3], r[1] * PHASE_STEP_PS) for r in train]
    c0a, sa = fit_linear(tra)
    c0b, sb = fit_linear(trb)

    d_raw, d_lut = [], []
    for r in score:
        _, _, fa, fb = r
        d_raw.append(wrap((c0a + sa * fa) - (c0b + sb * fb)))
        if fa in luts[0] and fb in luts[1]:
            d_lut.append(wrap(luts[0][fa] - luts[1][fb]))

    for label, d in (("linear (uncalibrated)", d_raw), ("LUT (calibrated)", d_lut)):
        if not d:
            continue
        sp = rms(d)
        print(f"  {label:24s} sigma_pair = {sp:7.2f} ps"
              f"   sigma_single = {sp / math.sqrt(2):6.2f} ps")
    if d_raw and d_lut:
        print(f"  improvement: {100 * (1 - rms(d_lut) / rms(d_raw)):+.1f} %")

    print()
    print("  For reference: pure quantisation at tau = 17.0 ps is "
          f"tau/sqrt(12) = {17.0 / math.sqrt(12):.1f} ps per chain.")
    print("  Anything above that is jitter, and the LUT cannot remove jitter --")
    print("  it removes the bin-width non-uniformity sitting on top of it.")


if __name__ == "__main__":
    main()
