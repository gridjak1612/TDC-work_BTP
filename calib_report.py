#!/usr/bin/env python3
"""
calib_report.py -- calibration metrics + figures for ONE bitstream.

    python calib_report.py --cal RUN1.csv --eval RUN2.csv --out figs

RUN1 builds the calibration (code-density bin widths -> LUT of bin centres).
RUN2 is an INDEPENDENT capture of the SAME bitstream, used only to evaluate.

Before calibration : raw DNL / INL of the code (RUN1).
After calibration  : the LUT gives each code its measured bin centre, so what
                     remains is the LUT's own error, measured by re-measuring
                     every bin in RUN2:
                       residual DNL(c) = (w2(c) - w1(c)) / LSB
                       residual INL(c) = sum_{k<=c} (w2(k) - w1(k)) / LSB
                     and compared with the Poisson expectation.
Precision          : RUN1 LUT applied to RUN2 pairs (out-of-sample).
NOT covered        : accuracy against an independent time reference.
"""
import csv, math, argparse, os
from collections import Counter

P = 5000.0
RAIL = (0, 352)


def load(path):
    rows = []
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            rows.append((int(r['d_coarse']), int(r['fine_a']), int(r['fine_b']),
                         int(r['valid_a']), int(r['valid_b'])))
    return rows


def signed(dc):
    return dc - 16384 if dc >= 8192 else dc


def chain(rows1, rows2, ch):
    k = 1 + ch
    c1 = Counter(r[k] for r in rows1 if r[k] not in RAIL); n1 = len(rows1)
    c2 = Counter(r[k] for r in rows2 if r[k] not in RAIL); n2 = len(rows2)
    lo, hi = min(c1), max(c1)
    codes = list(range(lo, hi + 1))
    w1 = [c1.get(c, 0) / n1 * P for c in codes]
    w2 = [c2.get(c, 0) / n2 * P for c in codes]
    lsb = sum(w1) / len(codes)

    dnl = [w / lsb - 1 for w in w1]
    edges, acc = [], 0.0
    for w in w1:
        edges.append(acc); acc += w
    xs = list(range(len(edges)))
    mx, my = sum(xs) / len(xs), sum(edges) / len(edges)
    b = sum((x - mx) * (y - my) for x, y in zip(xs, edges)) / sum((x - mx) ** 2 for x in xs)
    inl = [(y - (my + b * (x - mx))) / lsb for x, y in zip(xs, edges)]

    d = [b2 - b1 for b1, b2 in zip(w1, w2)]
    rdnl = [x / lsb for x in d]
    rinl, acc = [], 0.0
    for x in d:
        acc += x; rinl.append(acc / lsb)
    pois = math.sqrt(sum((a * a / max(c1.get(c, 0), 1) + q * q / max(c2.get(c, 0), 1))
                         for c, a, q in zip(codes, w1, w2)) / len(codes)) / lsb

    W = [w for w in w1 if w > 0]
    qfloor = math.sqrt(sum(w ** 3 for w in W) / (12 * sum(W)))
    lut, e = {}, 0.0
    for c, w in zip(codes, w1):
        lut[c] = P - (e + w / 2); e += w
    return dict(codes=codes, w1=w1, lsb=lsb, dnl=dnl, inl=inl, rdnl=rdnl, rinl=rinl,
                pois=pois, qfloor=qfloor, lut=lut, zero=sum(1 for w in w1 if w == 0))


def pairs(rows, la, lb, off=None):
    r = []
    for dc, fa, fb, va, vb in rows:
        if not (va and vb):
            continue
        ta, tb = la.get(fa), lb.get(fb)
        if ta is None or tb is None:
            continue
        s = signed(dc)
        if abs(s) > 1:
            continue
        r.append(s * P + tb - ta)
    if off is None:
        off = sorted(r)[len(r) // 2]
    res = [x - off for x in r]
    m = sum(res) / len(res)
    sd = math.sqrt(sum((x - m) ** 2 for x in res) / len(res))
    return off, sd, res


def rng(v):
    return min(v), max(v)


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v))


def figures(A, B, res, sd, out):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -> pip install matplotlib (figures skipped)")
        return
    os.makedirs(out, exist_ok=True)

    def two(key, ylabel, fname, bar):
        fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        for a, R, n in ((ax[0], A, 'A'), (ax[1], B, 'B')):
            (a.bar(R['codes'], R[key], width=1.0) if bar else a.plot(R['codes'], R[key]))
            a.set_ylabel(f'chain {n}\n{ylabel}'); a.grid(alpha=.3)
        ax[1].set_xlabel('code'); fig.tight_layout()
        fig.savefig(os.path.join(out, fname), dpi=150); plt.close(fig)

    two('dnl', 'DNL (LSB)', 'raw_dnl.png', True)
    two('inl', 'INL (LSB)', 'raw_inl.png', False)
    two('rdnl', 'residual DNL (LSB)', 'cal_dnl.png', True)
    two('rinl', 'residual INL (LSB)', 'cal_inl.png', False)

    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for a, R, n in ((ax[0], A, 'A'), (ax[1], B, 'B')):
        a.plot(R['codes'], R['inl'], label='before calibration (raw code)')
        a.plot(R['codes'], R['rinl'], label='after calibration (residual)')
        a.set_ylabel(f'chain {n}\nINL (LSB)'); a.grid(alpha=.3); a.legend(loc='upper right')
    ax[1].set_xlabel('code'); fig.tight_layout()
    fig.savefig(os.path.join(out, 'inl_before_after.png'), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    for R, n in ((A, 'A'), (B, 'B')):
        ax.plot(R['codes'], R['w1'], '.', ms=3, label=f'chain {n}')
    ax.set_xlabel('code'); ax.set_ylabel('bin width (ps)'); ax.grid(alpha=.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(out, 'bin_widths.png'), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist([x for x in res if abs(x) < 100], bins=120)
    ax.set_xlabel('calibrated A-B residual (ps)'); ax.set_ylabel('count')
    ax.set_title(f'tied channels, out-of-sample: sigma_pair = {sd:.2f} ps')
    ax.grid(alpha=.3); fig.tight_layout()
    fig.savefig(os.path.join(out, 'precision_hist.png'), dpi=150); plt.close(fig)
    print(f"figures written to {out}\\")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cal', required=True)
    ap.add_argument('--eval', required=True)
    ap.add_argument('--out', default='figs')
    ap.add_argument('--rail-max', type=int, default=352)
    a = ap.parse_args()
    global RAIL
    RAIL = (0, a.rail_max)
    r1, r2 = load(a.cal), load(a.eval)
    A, B = chain(r1, r2, 0), chain(r1, r2, 1)
    lb_shift = {}
    off, _, _ = pairs(r1, A['lut'], B['lut'])
    _, sd, res = pairs(r2, A['lut'], B['lut'], off)
    outl = sum(1 for x in res if abs(x) > 200) / len(res)

    print(f"cal : {a.cal}  ({len(r1)} frames)")
    print(f"eval: {a.eval}  ({len(r2)} frames)\n")
    print(f"{'':44s}{'chain A':>18s}{'chain B':>18s}")
    def row(label, fa, fb):
        print(f"{label:44s}{fa:>18s}{fb:>18s}")
    row("codes / zero-width", f"{len(A['codes'])} / {A['zero']}", f"{len(B['codes'])} / {B['zero']}")
    row("LSB (ps)", f"{A['lsb']:.2f}", f"{B['lsb']:.2f}")
    row("BEFORE cal: DNL (LSB)", "%+.2f / %+.2f" % rng(A['dnl']), "%+.2f / %+.2f" % rng(B['dnl']))
    row("BEFORE cal: INL (LSB)", "%+.2f / %+.2f" % rng(A['inl']), "%+.2f / %+.2f" % rng(B['inl']))
    row("BEFORE cal: INL (ps)", "%+.0f / %+.0f" % tuple(x * A['lsb'] for x in rng(A['inl'])),
        "%+.0f / %+.0f" % tuple(x * B['lsb'] for x in rng(B['inl'])))
    row("AFTER cal: residual DNL (LSB) min/max", "%+.3f / %+.3f" % rng(A['rdnl']), "%+.3f / %+.3f" % rng(B['rdnl']))
    row("AFTER cal: residual DNL rms (LSB)", f"{rms(A['rdnl']):.3f}", f"{rms(B['rdnl']):.3f}")
    row("   Poisson expectation rms (LSB)", f"{A['pois']:.3f}", f"{B['pois']:.3f}")
    row("AFTER cal: residual INL (LSB)", "%+.3f / %+.3f" % rng(A['rinl']), "%+.3f / %+.3f" % rng(B['rinl']))
    row("AFTER cal: residual INL (ps)", "%+.1f / %+.1f" % tuple(x * A['lsb'] for x in rng(A['rinl'])),
        "%+.1f / %+.1f" % tuple(x * B['lsb'] for x in rng(B['rinl'])))
    row("quantisation floor (ps rms)", f"{A['qfloor']:.2f}", f"{B['qfloor']:.2f}")
    print(f"\nprecision (RUN1 LUT on RUN2 pairs, |d_coarse|<=1): sigma_pair = {sd:.2f} ps "
          f"-> {sd / math.sqrt(2):.2f} ps per channel (tied channels, random phase); "
          f"outliers >200 ps: {100 * outl:.3f} %")
    figures(A, B, res, sd, a.out)


if __name__ == "__main__":
    main()