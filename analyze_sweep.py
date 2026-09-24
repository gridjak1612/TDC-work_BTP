#!/usr/bin/env python3
"""
analyze_sweep.py -- turn a DPS phase-sweep log into a calibration LUT.

    python analyze_sweep.py probe.csv
    python analyze_sweep.py sweep_a.csv --out-lut lut          # writes lut_a.csv, lut_b.csv

INPUT  the CSV written by read_tdc.py:
       seq, phase, d_coarse, fine_a, fine_b, valid_a, valid_b

THE MEASUREMENT
---------------
clk_cal (the event) and clk200 (the sampler) come from the same MMCM, so their
relative phase is DETERMINISTIC, not random. Stepping CLKOUT1 by one fine step
moves the event 17.857 ps (T_VCO/56) against a stationary sampler. 280 steps is
5.000 ns -- exactly one sampler period -- so the phase axis is CIRCULAR and
closes on itself. Everything below respects that; treating it as a line is the
easy way to get a plausible, wrong answer at the wrap.

TWO INDEPENDENT ROUTES TO THE SAME ANSWER, DELIBERATELY
-------------------------------------------------------
1. TRANSFER CURVE     mean code as a function of phase. Jitter is roughly
                      symmetric, so the mean is a good estimator even when a
                      single shot spans two or three codes. This gives
                      monotonicity, the wrap location, and the effective slope.

2. CODE DENSITY       280 equally-spaced phases tile one period uniformly, so
                      the aggregate code histogram IS a code-density histogram.
                      Bin width(c) = N(c)/N_total x 5000 ps. Free, and the
                      route to DNL/INL.

They measure different things and can disagree. Route 2 is broadened by jitter:
a bin narrower than the jitter still collects hits from neighbouring phases, so
very narrow bins read too wide and very wide bins too narrow. Route 1 is not.
Where they disagree, believe route 1 for the transfer function and route 2 for
the DNL shape -- and say so in the paper rather than quoting whichever is
prettier.

THE LUT ITSELF
--------------
What you actually want is E[t | code] -- given an observed code, the best
estimate of when the event arrived. That is computed directly: for each code,
take the CIRCULAR mean of the phase times of every sample that produced it.
This handles jitter correctly without assuming a bin shape, and it is defined
even for codes whose bin is narrower than one phase step.

HYSTERESIS
----------
The sweep is a triangle (0 -> 279 -> 0). Up and down traversals are separated
and compared. A systematic difference is MMCM fine-phase-shift hysteresis --
real, and invisible to a sawtooth-plus-reset sweep.
"""

import sys
import csv
import math
import argparse
from collections import defaultdict

PHASE_STEP_PS = 1000.0 / 56.0     # T_VCO/56, VCO = 1000 MHz
SWEEP_STEPS = 280                 # 280 x 17.857 ps = 5000 ps = one clk200 period
PERIOD_PS = SWEEP_STEPS * PHASE_STEP_PS
TDL_TAPS = 352


# --------------------------------------------------------------------------
# circular helpers -- the phase axis wraps at SWEEP_STEPS
# --------------------------------------------------------------------------

def circ_mean_phase(phases):
    """Circular mean of phase indices, returned in [0, SWEEP_STEPS)."""
    if not phases:
        return float('nan')
    sx = sy = 0.0
    for p in phases:
        a = 2.0 * math.pi * (p % SWEEP_STEPS) / SWEEP_STEPS
        sx += math.cos(a)
        sy += math.sin(a)
    if abs(sx) < 1e-12 and abs(sy) < 1e-12:
        return float('nan')                      # uniformly spread: no mean
    a = math.atan2(sy, sx)
    return (a / (2.0 * math.pi) * SWEEP_STEPS) % SWEEP_STEPS


def circ_conc(phases):
    """Resultant length in [0,1]. Near 1 = tight, near 0 = spread all round."""
    if not phases:
        return 0.0
    sx = sy = 0.0
    for p in phases:
        a = 2.0 * math.pi * (p % SWEEP_STEPS) / SWEEP_STEPS
        sx += math.cos(a)
        sy += math.sin(a)
    return math.hypot(sx, sy) / len(phases)


def circ_mean_code(codes):
    """Plain mean -- codes do NOT wrap, only phase does."""
    return sum(codes) / len(codes) if codes else float('nan')


# --------------------------------------------------------------------------

def load(path):
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            rows.append((int(r['seq']), int(r['phase']),
                         int(r['fine_a']), int(r['fine_b']),
                         int(r['valid_a']), int(r['valid_b'])))
    return rows


def split_direction(rows):
    """
    Tag each row up/down by watching the phase move over time.
    Rows are in arrival order, so consecutive phase changes give direction.
    """
    out = []
    direction = +1
    last = None
    for r in rows:
        p = r[1]
        if last is not None and p != last:
            d = p - last
            if d > SWEEP_STEPS // 2:
                d -= SWEEP_STEPS
            elif d < -SWEEP_STEPS // 2:
                d += SWEEP_STEPS
            if d != 0:
                direction = 1 if d > 0 else -1
        last = p
        out.append(r + (direction,))
    return out


def analyse_channel(rows, ch, args):
    """ch: 0 = A, 1 = B."""
    fi = 2 + ch
    vi = 4 + ch
    name = 'A' if ch == 0 else 'B'

    by_phase = defaultdict(list)
    by_phase_dir = {+1: defaultdict(list), -1: defaultdict(list)}
    by_code = defaultdict(list)
    railed_by_phase = defaultdict(int)
    total_by_phase = defaultdict(int)
    n_used = 0

    for r in rows:
        phase, code, valid, direction = r[1], r[fi], r[vi], r[6]
        total_by_phase[phase] += 1
        if code in (0, TDL_TAPS):
            railed_by_phase[phase] += 1
            continue
        if not valid:
            continue
        by_phase[phase].append(code)
        by_phase_dir[direction][phase].append(code)
        by_code[code].append(phase)
        n_used += 1

    print()
    print("=" * 74)
    print(f"  CHAIN {name}")
    print("=" * 74)
    print(f"  usable samples : {n_used}")

    # ---- railing zone -----------------------------------------------------
    rail_phases = sorted(p for p in railed_by_phase
                         if railed_by_phase[p] > 0.5 * total_by_phase[p])
    if rail_phases:
        runs = []
        start = prev = rail_phases[0]
        for p in rail_phases[1:]:
            if p == prev + 1:
                prev = p
            else:
                runs.append((start, prev)); start = prev = p
        runs.append((start, prev))
        print(f"  railing phases : {len(rail_phases)} steps "
              f"= {len(rail_phases) * PHASE_STEP_PS:.0f} ps dead zone")
        for a, b in runs:
            print(f"      phases {a}..{b}  ({(b - a + 1) * PHASE_STEP_PS:.0f} ps)")
        print("      This is the launch-skew ambiguity: events landing inside the")
        print("      capture synchroniser's setup window resolve either way, so the")
        print("      snapshot is taken one clock late and the chain reads full.")
    else:
        print("  railing phases : none")

    # ---- route 1: transfer curve -----------------------------------------
    phases = sorted(by_phase)
    if not phases:
        print("  NO USABLE DATA")
        return None
    means = {p: circ_mean_code(by_phase[p]) for p in phases}

    # ---- slope -------------------------------------------------------------
    # SIGN MATTERS AND IT IS NEGATIVE. Incrementing the DPS index DELAYS
    # clk_cal against a stationary sampler, so the carry has LESS time to
    # propagate before the sampling edge and the code goes DOWN. An earlier
    # version of this function assumed a rising transfer curve, kept only the
    # positive deltas, and therefore measured nothing but the wrap jump --
    # reporting ~287 codes/step and tau = 0.06 ps/tap on data that was in fact
    # near-perfectly linear. Take the median of ALL steps, then judge
    # monotonicity against the dominant sign rather than against zero.
    deltas = []
    for i in range(1, len(phases)):
        if phases[i] != phases[i - 1] + 1:
            continue                       # gap (railing): not a single step
        d = means[phases[i]] - means[phases[i - 1]]
        if abs(d) < TDL_TAPS / 2:          # ignore the code-axis rollover
            deltas.append(d)
    deltas.sort()
    slope = deltas[len(deltas) // 2] if deltas else float('nan')
    tau = abs(PHASE_STEP_PS / slope) if slope and slope == slope else float('nan')

    print()
    print("  --- transfer curve (mean code vs phase) ---")
    print(f"      phases with data : {len(phases)} / {SWEEP_STEPS}")
    print(f"      code range       : {min(means.values()):.1f} .. {max(means.values()):.1f}")
    print(f"      median slope     : {slope:+.3f} codes / step   -> tau = {tau:.2f} ps/tap")
    sign = 1 if slope > 0 else -1
    against = sum(1 for d in deltas if d * sign < 0)
    print(f"      steps against the dominant direction : {against} / {len(deltas)}"
          f"  ({100.0 * against / max(1, len(deltas)):.1f} %)")

    if args.curve:
        print()
        print("      phase  mean_code   n")
        for p in phases[::args.curve]:
            print(f"      {p:5d}  {means[p]:9.2f}  {len(by_phase[p]):4d}")

    # ---- route 2: code density -------------------------------------------
    print()
    print("  --- code density (aggregate histogram) ---")
    counts = {c: len(v) for c, v in by_code.items()}
    tot = sum(counts.values())
    seen = sorted(counts)

    # REACHABLE RANGE vs CHAIN LENGTH -- these are not the same thing and
    # conflating them badly overstates the zero-width fraction. The chain is
    # 352 taps, but the sampling edge arrives once the carry has climbed only
    # ~288 of them; the rest is deliberate headroom so the chain always spans a
    # full period. Codes above the maximum reachable one are NOT zero-width
    # bins -- they are simply never in play, and counting them as defects is
    # how a 7 % zero-width chain gets reported as 53 %.
    lo, hi = seen[0], seen[-1]
    n_reach = hi - lo + 1
    missing = n_reach - len(seen)
    print(f"      reachable codes  : {lo} .. {hi}  ({n_reach} codes)")
    print(f"      headroom unused  : {TDL_TAPS - hi} taps above the reachable range")
    print(f"      zero-width bins  : {missing} / {n_reach} "
          f"({100.0 * missing / n_reach:.1f} % WITHIN the reachable range)")

    # LSB is the period divided by the reachable code count, not the mean over
    # the codes that happened to appear -- otherwise every missing bin inflates
    # the LSB and flatters the DNL.
    # ---- PER-PHASE NORMALISED BIN WIDTHS ----------------------------------
    # The naive estimator, width(c) = N(c)/N_total x period, assumes every
    # phase was sampled equally often. A triangle sweep that does not complete
    # a whole number of traversals violates that badly: phases visited twice
    # carry 512 samples and phases visited once carry 256, so a bin sitting in
    # the singly-visited region reads HALF its true width. On a 5 s run that
    # showed up as ~400 ps of entirely fictitious INL.
    #
    # Normalising each phase to unit weight first removes it:
    #     width(c) = SUM_p  P(c | p) x phase_step
    # which is the correct estimator regardless of how the visits fell.
    widths = defaultdict(float)
    for ph in phases:
        codes_here = by_phase[ph]
        n_here = len(codes_here)
        sub = defaultdict(int)
        for c in codes_here:
            sub[c] += 1
        for c, k in sub.items():
            widths[c] += (k / n_here) * PHASE_STEP_PS
    widths = dict(widths)
    covered = len(phases) * PHASE_STEP_PS      # period minus the railing gap
    wmean = covered / n_reach
    print(f"      phase coverage : {len(phases)}/{SWEEP_STEPS} steps = {covered:.0f} ps")
    print(f"      LSB (covered/{n_reach}) : {wmean:.2f} ps")
    print(f"      widest bin     : {max(widths.values()):.1f} ps "
          f"(code {max(widths, key=widths.get)})")
    print(f"      narrowest bin  : {min(widths.values()):.2f} ps "
          f"(code {min(widths, key=widths.get)})")
    print("      NOTE bin widths here are jitter-broadened; use the transfer")
    print("      curve for the transfer function and these for DNL shape only.")

    # DNL over the reachable range. A missing code is a genuine zero-width bin
    # here and correctly scores DNL = -1.
    contiguous = list(range(lo, hi + 1))
    dnl = {c: (widths.get(c, 0.0) / wmean - 1.0) for c in contiguous}

    # INL as the deviation of each bin EDGE from the ideal ramp. The previous
    # version summed DNL without removing the mean, so any small bias in the
    # LSB accumulated linearly across ~290 codes and produced a huge fake tilt
    # (-33 LSB on a chain whose real INL is a few LSB). Referencing the edges
    # to a best-fit line is the standard definition and does not do that.
    edges = []
    acc = 0.0
    for c in contiguous:
        edges.append(acc)
        acc += widths.get(c, 0.0)
    span = acc
    ideal = [span * i / len(contiguous) for i in range(len(contiguous))]
    del ideal[len(contiguous):]
    inl_ps = [e - t for e, t in zip(edges, ideal)]
    inl = {c: v / wmean for c, v in zip(contiguous, inl_ps)}
    print(f"      DNL : {min(dnl.values()):+.2f} / {max(dnl.values()):+.2f} LSB")
    print(f"      INL : {min(inl.values()):+.2f} / {max(inl.values()):+.2f} LSB "
          f"({min(inl_ps):+.0f} / {max(inl_ps):+.0f} ps)")

    # the ten widest bins -- this is where the physical structure shows up
    wide = sorted(widths.items(), key=lambda kv: -kv[1])[:10]
    print("      widest bins (code: ps, xLSB):")
    print("         " + "  ".join(f"{c}:{w:.0f}({w/wmean:.1f}x)" for c, w in wide))

    # ---- EVEN / ODD TAP ALTERNATION ---------------------------------------
    # The dominant DNL mechanism is a PERIOD-2 alternation: even codes are
    # wide, odd codes narrow, by roughly 5x. It is NOT a period-4 CARRY4
    # block-boundary effect -- multiples of 4 are a subset of the evens, so a
    # top-N widest-bin list drawn from a wide-even chain comes out about half
    # divisible by 4 purely by coincidence. Reading that as a period-4 signal
    # is an easy and wrong conclusion; the mod-4 breakdown below shows the
    # period-4 component is weak (a few percent) once the evens and odds are
    # separated.
    #
    # Verified in simulation that the 5-tap majority filter does NOT create
    # this: fed a thermometer with uniform bins it returns 50.0 % even codes.
    # The alternation is in the carry chain itself.
    full = {c: widths.get(c, 0.0) for c in range(lo, hi + 1)}
    ev = [w for c, w in full.items() if c % 2 == 0]
    od = [w for c, w in full.items() if c % 2 == 1]
    if ev and od:
        me, mo = sum(ev) / len(ev), sum(od) / len(od)
        print(f"      even bins : {me:6.2f} ps (n={len(ev)})   "
              f"odd bins : {mo:6.2f} ps (n={len(od)})")
        print(f"      EVEN/ODD ratio : {me / mo:.2f}x    "
              f"{100 * sum(ev) / (sum(ev) + sum(od)):.1f} % of the period is in even bins")
        for m in range(4):
            g = [w for c, w in full.items() if c % 4 == m]
            if g:
                print(f"         code %% 4 == {m} : {sum(g) / len(g):6.2f} ps")

    # ---- QUANTISATION FLOOR -----------------------------------------------
    # For non-uniform bins the RMS quantisation error is
    #     sqrt( SUM w^3 / (12 SUM w) )
    # dominated by the widest bins. Compare against what the same tap count
    # would give if the bins were uniform: the gap is what tap interleaving
    # (using the CARRY4 O outputs as well as CO) could recover. If the
    # measured LUT residual sits at the actual-bin figure, the instrument is
    # QUANTISATION limited and reducing jitter will not help it at all.
    Wl = [w for w in full.values() if w > 0]
    if Wl:
        q = math.sqrt(sum(w ** 3 for w in Wl) / (12 * sum(Wl)))
        qu = (sum(Wl) / len(Wl)) / math.sqrt(12)
        print(f"      RMS quantisation, actual bins  : {q:.2f} ps")
        print(f"      RMS quantisation, if uniform   : {qu:.2f} ps"
              f"   <- headroom from tap interleaving")

    # clock-region boundary check
    bnds = (80, 280) if ch == 0 else (76, 276)
    print(f"      clock-region boundary bins (predicted at taps {bnds}):")
    for b in bnds:
        near = {c: widths.get(c, 0.0) for c in range(b - 2, b + 3) if c in widths}
        if near:
            wc = max(near, key=near.get)
            print(f"          near tap {b}: widest is code {wc} at {near[wc]:.1f} ps "
                  f"({near[wc] / wmean:.1f}x mean)")

    # ---- hysteresis -------------------------------------------------------
    common = [p for p in phases
              if by_phase_dir[+1].get(p) and by_phase_dir[-1].get(p)]
    if common:
        diffs = [circ_mean_code(by_phase_dir[+1][p]) - circ_mean_code(by_phase_dir[-1][p])
                 for p in common]
        md = sum(diffs) / len(diffs)
        print()
        print("  --- up vs down traversal (MMCM DPS hysteresis) ---")
        print(f"      phases compared : {len(common)}")
        print(f"      mean(up - down) : {md:+.3f} codes = {md * PHASE_STEP_PS / (slope or 1):+.1f} ps")
        print(f"      worst |up-down| : {max(abs(d) for d in diffs):.2f} codes")
    else:
        print()
        print("  --- up vs down: not enough coverage (run longer than one traversal)")

    # ---- the LUT ----------------------------------------------------------
    lut = {}
    for c in seen:
        mp = circ_mean_phase(by_code[c])
        lut[c] = (mp * PHASE_STEP_PS, len(by_code[c]), circ_conc(by_code[c]))

    if args.out_lut:
        fn = f"{args.out_lut}_{name.lower()}.csv"
        with open(fn, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(["code", "t_ps", "n", "concentration", "bin_width_ps", "dnl_lsb"])
            for c in seen:
                t, n, k = lut[c]
                w.writerow([c, f"{t:.2f}", n, f"{k:.3f}",
                            f"{widths[c]:.2f}", f"{dnl.get(c, 0):.3f}"])
        print()
        print(f"  LUT written: {fn}  ({len(seen)} codes)")
        print("      t_ps is E[arrival time | code], circular-mean over the period.")
        print("      concentration < ~0.5 means that code appears at phases spread")
        print("      right around the period -- its t_ps is meaningless, drop it.")

    return dict(slope=slope, widths=widths, lut=lut)


def main():
    ap = argparse.ArgumentParser(description="DPS sweep -> calibration LUT")
    ap.add_argument("csv")
    ap.add_argument("--out-lut", default=None,
                    help="prefix; writes <prefix>_a.csv and <prefix>_b.csv")
    ap.add_argument("--curve", type=int, default=0,
                    help="print the transfer curve every Nth phase (e.g. 10)")
    a = ap.parse_args()

    rows = load(a.csv)
    print(f"loaded {len(rows)} rows from {a.csv}")
    if not rows:
        sys.exit("empty")

    seqs = [r[0] for r in rows]
    gaps = sum((seqs[i] - seqs[i - 1] - 1) & 0xFF for i in range(1, len(seqs)))
    print(f"sequence gaps (dropped measurements): {gaps}")

    rows = split_direction(rows)
    nup = sum(1 for r in rows if r[6] > 0)
    print(f"up-traversal rows: {nup}   down-traversal rows: {len(rows) - nup}")
    print(f"phase step {PHASE_STEP_PS:.3f} ps, {SWEEP_STEPS} steps, "
          f"period {PERIOD_PS:.1f} ps")

    for ch in (0, 1):
        analyse_channel(rows, ch, a)


if __name__ == "__main__":
    main()
