#!/usr/bin/env python3
"""
code_density.py -- bin widths, DNL/INL and a calibration LUT from RANDOM hits
(EVENT_SRC = 3, on-chip ring oscillator). No generator, no phase reference.

    python code_density.py ro7.csv --out-lut data/lut_cd7
    python code_density.py ro7.csv --compare data/lut_v2        # vs DPS widths
    python code_density.py ro7.csv --compare data/lut_cd11      # vs other RO length
    python code_density.py --from-bytes uart_bytes.txt          # simulation output
    python code_density.py --selftest

PRINCIPLE
    Hits uniform in phase over one 5 ns period => a bin collects hits in
    proportion to its width:   w(c) = N(c) / N_all x 5000 ps
    N_all counts EVERY frame, including railed (dead-zone) and invalid ones:
    they occupy real time too. Dropping them would stretch the other bins.

    The 5000 ps comes from the crystal-referenced clk200 period, not from the
    MMCM phase interpolator -- so these widths are independent of the DPS
    reference. Comparing them with DPS-sweep widths (--compare) separates
    chain INL from reference nonlinearity.

UNIFORMITY IS AN ASSUMPTION -- IT IS TESTED, NOT TRUSTED
    chunk test : the run is cut into 8 sequential chunks; their histograms must
                 agree within Poisson (chi2/dof ~ 1). A ring oscillator near a
                 low-order resonance with clk200 fails this (model: 13.5).
    two RO lengths : build RO_STAGES 7 and 11, --compare the results.

LUT
    t_ps(c) = 5000 - (E(c) + w(c)/2),  E(c) = sum of widths below c.
    Same sign convention as analyze_sweep.py, so read_tdc.py --lut and
    validate_lut.py work unchanged. Chain B is shifted so tied-channel
    intervals average to zero; the shift IS the A-B launch skew.
"""

import sys
import csv
import math
import random
import argparse
from collections import Counter

PERIOD_PS = 5000.0
TDL_TAPS = 352


def load_csv(path):
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            rows.append(dict(d_coarse=int(r['d_coarse']),
                             fine_a=int(r['fine_a']), fine_b=int(r['fine_b']),
                             valid_a=int(r['valid_a']), valid_b=int(r['valid_b']),
                             seq=int(r['seq'])))
    return rows


def load_bytes(path):
    """Hex-per-line UART dump from tb_ro_cd.v, decoded with the real host framer."""
    import read_tdc
    data = bytes(int(l, 16) for l in open(path) if l.strip() and 'x' not in l)
    fr = read_tdc.Framer()
    rows = fr.feed(data)
    print(f"decoded {len(rows)} frames from {len(data)} bytes "
          f"(crc_errors={fr.crc_errors}, resyncs={fr.resyncs})")
    return rows


def widths_from_codes(codes, n_all):
    cnt = Counter(codes)
    lo, hi = min(cnt), max(cnt)
    w = {c: cnt.get(c, 0) / n_all * PERIOD_PS for c in range(lo, hi + 1)}
    return w, cnt, lo, hi


def chunk_chi2(codes, lo, hi, chunks=8):
    n = len(codes)
    if n < chunks * 50:
        return float('nan')
    parts = [codes[i * n // chunks:(i + 1) * n // chunks] for i in range(chunks)]
    H = [Counter(p) for p in parts]
    tot = Counter(codes)
    chi, dof = 0.0, 0
    for c in range(lo, hi + 1):
        if tot[c] < 5 * chunks:
            continue
        for k in range(chunks):
            e = tot[c] * len(parts[k]) / n
            chi += (H[k][c] - e) ** 2 / e
        dof += chunks - 1
    return chi / dof if dof else float('nan')


def analyse(rows, ch, args):
    name = 'A' if ch == 0 else 'B'
    fk, vk = ('fine_a', 'valid_a') if ch == 0 else ('fine_b', 'valid_b')
    n_all = len(rows)
    railed = sum(1 for r in rows if r[fk] in (0, TDL_TAPS))
    invalid = sum(1 for r in rows if not r[vk] and r[fk] not in (0, TDL_TAPS))
    codes = [r[fk] for r in rows if r[vk] and r[fk] not in (0, TDL_TAPS)]

    w, cnt, lo, hi = widths_from_codes(codes, n_all)
    nreach = hi - lo + 1
    lsb = sum(w.values()) / nreach
    zero = sum(1 for c in w if w[c] == 0)

    print()
    print("=" * 74)
    print(f"  CHAIN {name}   ({n_all} hits)")
    print("=" * 74)
    print(f"  railed (dead zone) : {railed} hits = {railed / n_all * PERIOD_PS:6.1f} ps "
          f"of the period")
    print(f"  invalid (bubbles)  : {invalid} hits = {invalid / n_all * 100:.3f} %  "
          f"(time location unknown; widths below exclude it)")
    print(f"  reachable codes    : {lo}..{hi} ({nreach}),  zero-width {zero} "
          f"({100 * zero / nreach:.1f} %)")
    print(f"  LSB                : {lsb:.2f} ps")
    hits_per_bin = len(codes) / nreach
    print(f"  hits per bin (avg) : {hits_per_bin:.0f}  -> per-bin width "
          f"uncertainty ~{100 / math.sqrt(max(hits_per_bin, 1)):.1f} % (Poisson)")
    cc = chunk_chi2(codes, lo, hi)
    verdict = ("OK" if cc < 1.3 else "SUSPECT -- hits not uniform/independent"
               if cc == cc else "too few hits to test")
    print(f"  chunk chi2/dof     : {cc:.2f}   {verdict}")

    dnl = {c: w[c] / lsb - 1 for c in w}
    edges, acc = [], 0.0
    for c in range(lo, hi + 1):
        edges.append(acc); acc += w[c]
    xs = list(range(len(edges)))
    mx, my = sum(xs) / len(xs), sum(edges) / len(edges)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, edges)) / sum((x - mx) ** 2 for x in xs)
    inl_ps = [y - (my + b * (x - mx)) for x, y in zip(xs, edges)]
    print(f"  DNL                : {min(dnl.values()):+.2f} / {max(dnl.values()):+.2f} LSB")
    print(f"  INL (best-fit line): {min(inl_ps) / lsb:+.2f} / {max(inl_ps) / lsb:+.2f} LSB  "
          f"({min(inl_ps):+.0f} / {max(inl_ps):+.0f} ps)")

    ev = [w[c] for c in w if c % 2 == 0]
    od = [w[c] for c in w if c % 2 == 1]
    me, mo = sum(ev) / len(ev), sum(od) / len(od)
    print(f"  even / odd bins    : {me:.2f} / {mo:.2f} ps   ratio {me / mo:.2f}")
    W = [x for x in w.values() if x > 0]
    q = math.sqrt(sum(x ** 3 for x in W) / (12 * sum(W)))
    print(f"  RMS quantisation   : {q:.2f} ps  (uniform-bin equivalent "
          f"{lsb / math.sqrt(12):.2f} ps)")

    lut, e = {}, 0.0
    for c in range(lo, hi + 1):
        lut[c] = PERIOD_PS - (e + w[c] / 2)
        e += w[c]
    return dict(name=name, w=w, cnt=cnt, lut=lut, dnl=dnl, lo=lo, hi=hi,
                lsb=lsb, chunk=cc, even_odd=(me, mo))


def tie_offset(rows, lut_a, lut_b):
    r = []
    for x in rows:
        ta, tb = lut_a.get(x['fine_a']), lut_b.get(x['fine_b'])
        if ta is None or tb is None or not (x['valid_a'] and x['valid_b']):
            continue
        dc = x['d_coarse'] - (1 << 14) if x['d_coarse'] >= (1 << 13) else x['d_coarse']
        r.append(dc * PERIOD_PS + ta - tb)
    if not r:
        return 0.0, float('nan'), 0
    r.sort()
    off = r[len(r) // 2]
    res = [v - off for v in r]
    m = sum(res) / len(res)
    return off, math.sqrt(sum((v - m) ** 2 for v in res) / len(res)), len(r)


def write_lut(path, res, shift=0.0):
    with open(path, 'w', newline='') as fh:
        wr = csv.writer(fh)
        wr.writerow(["code", "t_ps", "n", "concentration", "bin_width_ps", "dnl_lsb"])
        for c in range(res['lo'], res['hi'] + 1):
            wr.writerow([c, f"{res['lut'][c] - shift:.2f}", res['cnt'].get(c, 0), "1.000",
                         f"{res['w'][c]:.3f}", f"{res['dnl'][c]:.3f}"])
    print(f"  LUT written: {path}")


def compare(res, path):
    ref = {}
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            ref[int(r['code'])] = float(r['bin_width_ps'])
    common = [c for c in range(res['lo'], res['hi'] + 1) if c in ref]
    if len(common) < 10:
        print(f"  compare {path}: only {len(common)} common codes -- different build?")
        return
    d = [res['w'][c] - ref[c] for c in common]
    rms = math.sqrt(sum(x * x for x in d) / len(d))
    a = [res['w'][c] for c in common]; b = [ref[c] for c in common]
    ma, mb = sum(a) / len(a), sum(b) / len(b)
    cor = (sum((x - ma) * (y - mb) for x, y in zip(a, b)) /
           math.sqrt(sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b)))
    cum, acc = [], 0.0
    for x in d:
        acc += x; cum.append(acc)
    print(f"  vs {path}: {len(common)} codes, width diff rms {rms:.2f} ps, "
          f"correlation {cor:.3f}")
    print(f"      cumulative (INL) difference {min(cum):+.0f} / {max(cum):+.0f} ps"
          f"  <- smooth drift here = reference nonlinearity, not chain INL")


def selftest():
    """Uniform hits through a chain with KNOWN widths must give them back."""
    rnd = random.Random(1)
    true = [29.0 if i % 2 == 0 else 5.0 for i in range(1, 300)]
    edges, acc = [], 0.0
    for x in true:
        acc += x; edges.append(acc)
    import bisect
    rows = []
    for _ in range(400000):
        t = rnd.uniform(0, PERIOD_PS)
        c = bisect.bisect_right(edges, t) + 1
        c = c if c <= len(true) else TDL_TAPS
        rows.append(dict(d_coarse=0, fine_a=c, fine_b=c, valid_a=1, valid_b=1, seq=0))
    res = analyse(rows, 0, None)
    me, mo = res['even_odd']
    ok = abs(me - 29.0) < 0.5 and abs(mo - 5.0) < 0.3 and res['chunk'] < 1.3
    print(f"\nSELFTEST {'PASS' if ok else 'FAIL'}: recovered even/odd {me:.2f}/{mo:.2f} "
          f"(true 29/5), chunk chi2 {res['chunk']:.2f}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="Code-density calibration from RO hits")
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--from-bytes", default=None, help="hex-per-line UART dump (sim)")
    ap.add_argument("--out-lut", default=None, help="prefix -> <p>_a.csv, <p>_b.csv")
    ap.add_argument("--compare", default=None,
                    help="LUT prefix with bin_width_ps (DPS or other RO build)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        sys.exit(selftest())

    if a.from_bytes:
        rows = load_bytes(a.from_bytes)
    elif a.csv:
        rows = load_csv(a.csv)
    else:
        ap.error("csv, --from-bytes or --selftest required")

    seqs = [r['seq'] for r in rows]
    gaps = sum((seqs[i] - seqs[i - 1] - 1) & 0xFF for i in range(1, len(seqs)))
    print(f"{len(rows)} frames, sequence gaps (dropped): {gaps}")

    ra = analyse(rows, 0, a)
    rb = analyse(rows, 1, a)

    off, sp, n = tie_offset(rows, ra['lut'], rb['lut'])
    print()
    print(f"  tied channels: A-B launch skew {off:+.1f} ps (absorbed into LUT B)")
    print(f"  sigma_pair at d~0 : {sp:.2f} ps  (n={n}). NOT a single-shot figure:")
    print("      both chains share the period-2 structure, so their quantisation")
    print("      errors are correlated at d~0. Quote per-chain figures instead.")

    if a.compare:
        print()
        compare(ra, f"{a.compare}_a.csv")
        compare(rb, f"{a.compare}_b.csv")
    if a.out_lut:
        print()
        write_lut(f"{a.out_lut}_a.csv", ra)
        write_lut(f"{a.out_lut}_b.csv", rb, shift=-off)


if __name__ == "__main__":
    main()