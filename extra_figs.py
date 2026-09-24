#!/usr/bin/env python3
"""
extra_figs.py -- every figure we can draw from existing captures (no hardware).

  python extra_figs.py --cal RUN1.csv --eval RUN2.csv --out figs_extra
        [--dps DPS_SWEEP.csv] [--old PRE_FIX_RO.csv] [--build LUT_PREFIX_1 LUT_PREFIX_2]

  --cal/--eval : two ring-oscillator captures of the SAME bitstream
  --dps        : a DPS phase sweep (any build)             -> transfer curve, hysteresis, spread
  --old        : an RO capture from BEFORE the dead-zone fix -> dead zone before/after
  --build      : two LUT prefixes from DIFFERENT builds     -> build-to-build bin change
"""
import csv, math, argparse, os
from array import array
from collections import Counter, defaultdict

P = 5000.0
RAIL = (0, 352)
STEP = 1000.0 / 56.0
K = ('phase', 'd_coarse', 'fine_a', 'fine_b', 'valid_a', 'valid_b')
FK, VK = ('fine_a', 'fine_b'), ('valid_a', 'valid_b')
plt = None


def load(path):
    D = {k: array('i') for k in K}
    with open(path, newline='') as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        idx = [(k, hdr.index(k)) for k in K]
        for r in rd:
            for k, i in idx:
                D[k].append(int(r[i]))
    print(f"loaded {len(D['phase'])} rows  {path}")
    return D


def nrows(D): return len(D['phase'])
def signed(d): return d - 16384 if d >= 8192 else d


def widths(D, ch, codes=None):
    cnt = Counter(x for x in D[FK[ch]] if x not in RAIL)
    if codes is None:
        codes = list(range(min(cnt), max(cnt) + 1))
    n = nrows(D)
    return codes, [cnt.get(c, 0) for c in codes], [cnt.get(c, 0) / n * P for c in codes]


def edges(w):
    e, acc = [], 0.0
    for x in w:
        e.append(acc); acc += x
    return e


def lut_from(codes, w):
    return {c: P - (e + x / 2) for c, x, e in zip(codes, w, edges(w))}


def fitline(y):
    n = len(y); mx = (n - 1) / 2; my = sum(y) / n
    b = sum((x - mx) * (v - my) for x, v in enumerate(y)) / sum((x - mx) ** 2 for x in range(n))
    return [my + b * (x - mx) for x in range(n)]


def corr(a, b):
    ma, mb = sum(a) / len(a), sum(b) / len(b)
    return (sum((x - ma) * (y - mb) for x, y in zip(a, b)) /
            math.sqrt(sum((x - ma) ** 2 for x in a) * sum((y - mb) ** 2 for y in b)))


def std(v):
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def residuals(Dc, De, LA, LB):
    def raw(D):
        out = []
        for dc, fa, fb, va, vb in zip(D['d_coarse'], D['fine_a'], D['fine_b'], D['valid_a'], D['valid_b']):
            if not (va and vb):
                continue
            ta, tb = LA.get(fa), LB.get(fb)
            if ta is None or tb is None:
                continue
            s = signed(dc)
            if abs(s) > 1:
                continue
            out.append((s * P + tb - ta, s, fa))
        return out
        c = sorted(x[0] for x in raw(De)); off = c[len(c) // 2]   # offset of the EVAL source
    return [(x - off, s, fa) for x, s, fa in raw(De)]


def save(fig, out, name):
    fig.tight_layout(); fig.savefig(os.path.join(out, name), dpi=150); plt.close(fig)
    print("  wrote", name)


def main():
    global plt
    ap = argparse.ArgumentParser()
    ap.add_argument('--cal', required=True); ap.add_argument('--eval', required=True)
    ap.add_argument('--out', default='figs_extra'); ap.add_argument('--dps'); ap.add_argument('--old')
    ap.add_argument('--build', nargs=2)
    ap.add_argument('--rail-max', type=int, default=352)
    a = ap.parse_args()
    global RAIL
    RAIL = (0, a.rail_max)
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as _p; plt = _p
    os.makedirs(a.out, exist_ok=True)

    Dc, De = load(a.cal), load(a.eval)
    N = nrows(Dc)
    W = [widths(Dc, ch) for ch in (0, 1)]
    WE = [widths(De, ch, W[ch][0]) for ch in (0, 1)]
    names = ('A', 'B')

    # F01 hits per code (code-density histogram)
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for ch in (0, 1):
        codes, cnt, _ = W[ch]; m = sum(cnt) / len(cnt)
        ax[ch].bar(codes, cnt, width=1.0)
        ax[ch].axhline(m, color='r', lw=1, label=f'mean {m:.0f} hits/code')
        ax[ch].set_ylabel(f'chain {names[ch]}\nhits per code'); ax[ch].grid(alpha=.3); ax[ch].legend()
        print(f"  chain {names[ch]}: {len(codes)} codes, hits/code mean {m:.0f}, "
              f"min {min(cnt)}, max {max(cnt)}, zero-hit codes {sum(1 for x in cnt if x == 0)}")
    ax[1].set_xlabel('code (number of taps set)')
    fig.suptitle(f'Calibration histogram: hits per code (ring oscillator, {N:,} hits)')
    save(fig, a.out, 'F01_hits_per_code.png')

    # F02 bin-width distribution
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist([W[0][2], W[1][2]], bins=60, label=['chain A', 'chain B'])
    ax.set_xlabel('bin width (ps)'); ax.set_ylabel('number of bins'); ax.grid(alpha=.3); ax.legend()
    ax.set_title('Distribution of bin widths')
    save(fig, a.out, 'F02_binwidth_distribution.png')

    # F03 width by position inside CARRY4
    fig, ax = plt.subplots(figsize=(8, 4))
    for ch in (0, 1):
        codes, _, w = W[ch]
        g = [[x for c, x in zip(codes, w) if c % 4 == m] for m in range(4)]
        ax.bar([m + (ch - 0.5) * 0.38 for m in range(4)], [sum(x) / len(x) for x in g], width=0.38,
               yerr=[std(x) for x in g], capsize=3, label=f'chain {names[ch]}')
    ax.set_xticks(range(4))
    ax.set_xticklabels(['code%4=0\n(slice boundary)', 'code%4=1', 'code%4=2', 'code%4=3'])
    ax.set_ylabel('mean bin width (ps)'); ax.grid(alpha=.3); ax.legend()
    ax.set_title('Bin width by position inside the CARRY4 block')
    save(fig, a.out, 'F03_width_by_carry4_position.png')

    # F04 calibrated transfer function + INL in ps
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ch in (0, 1):
        codes, _, w = W[ch]; e = edges(w); f = fitline(e)
        ax[0].plot(codes, e, label=f'chain {names[ch]}')
        ax[1].plot(codes, [x - y for x, y in zip(e, f)], label=f'chain {names[ch]}')
    ax[0].set_ylabel('calibrated time of\nbin edge (ps)'); ax[1].set_ylabel('deviation from line\n= INL (ps)')
    ax[1].set_xlabel('code'); [x.grid(alpha=.3) for x in ax]; ax[0].legend()
    ax[0].set_title('Calibrated transfer function (code -> time)')
    save(fig, a.out, 'F04_transfer_function_and_inl.png')

    # F05 repeatability: run1 vs run2 widths
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.5))
    for ch in (0, 1):
        w1, w2 = W[ch][2], WE[ch][2]
        ax[ch].plot(w1, w2, '.', ms=3); mx = max(w1 + w2)
        ax[ch].plot([0, mx], [0, mx], 'r-', lw=0.8)
        ax[ch].set_title(f'chain {names[ch]}: corr {corr(w1, w2):.4f}')
        ax[ch].set_xlabel('bin width, run 1 (ps)'); ax[ch].set_ylabel('bin width, run 2 (ps)'); ax[ch].grid(alpha=.3)
    fig.suptitle('Calibration repeatability (same bitstream, independent captures)')
    save(fig, a.out, 'F05_repeatability.png')

    # F06 chain A vs chain B
    dA = dict(zip(W[0][0], W[0][2])); dB = dict(zip(W[1][0], W[1][2]))
    com = sorted(set(dA) & set(dB)); xa = [dA[c] for c in com]; xb = [dB[c] for c in com]
    fig, ax = plt.subplots(figsize=(5.5, 5))
    ax.plot(xa, xb, '.', ms=3); mx = max(xa + xb); ax.plot([0, mx], [0, mx], 'r-', lw=0.8)
    ax.set_xlabel('chain A bin width (ps)'); ax.set_ylabel('chain B bin width (ps)'); ax.grid(alpha=.3)
    ax.set_title(f'Same code, two chains: corr {corr(xa, xb):.3f}')
    save(fig, a.out, 'F06_chainA_vs_chainB.png')

    # F07 invalid (bubble) samples per code
    fig, ax = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
    for ch in (0, 1):
        inv = Counter(x for x, v in zip(Dc[FK[ch]], Dc[VK[ch]]) if not v and x not in RAIL)
        codes = W[ch][0]
        ax[ch].bar(codes, [inv.get(c, 0) for c in codes], width=1.0, color='tab:orange')
        ax[ch].set_ylabel(f'chain {names[ch]}\nbubble samples'); ax[ch].grid(alpha=.3)
    ax[1].set_xlabel('code'); fig.suptitle('Where thermometer bubbles occur')
    save(fig, a.out, 'F07_bubbles_per_code.png')

    # F08 fine_a vs fine_b (tied channels), coloured by d_coarse
    step = max(1, N // 60000); g = {0: ([], []), 1: ([], []), -1: ([], [])}
    for i in range(0, N, step):
        fa, fb, s = Dc['fine_a'][i], Dc['fine_b'][i], signed(Dc['d_coarse'][i])
        if fa in RAIL or fb in RAIL or abs(s) > 1:
            continue
        g[s][0].append(fa); g[s][1].append(fb)
    fig, ax = plt.subplots(figsize=(6, 6))
    for s, col in ((0, 'tab:blue'), (1, 'tab:orange'), (-1, 'tab:green')):
        if g[s][0]:
            ax.plot(g[s][0], g[s][1], '.', ms=1, color=col, label=f'd_coarse={s:+d}')
    ax.set_xlabel('fine_a'); ax.set_ylabel('fine_b'); ax.grid(alpha=.3); ax.legend(markerscale=8)
    ax.set_title('Both chains, same event: code pairs')
    save(fig, a.out, 'F08_codeA_vs_codeB.png')

    # residuals (LUT from run 1, evaluated on run 2)
    LA = lut_from(W[0][0], W[0][2]); LB = lut_from(W[1][0], W[1][2])
    R = residuals(Dc, De, LA, LB)

    # F09 precision vs code
    grp = defaultdict(list)
    for r, s, fa in R:
        grp[fa // 10 * 10].append(r)
    ks = sorted(k for k in grp if len(grp[k]) > 200)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot([k + 5 for k in ks], [std(grp[k]) for k in ks], 'o-')
    ax.axhline(std([r for r, _, _ in R]), color='r', lw=1, label='overall')
    ax.set_xlabel('fine_a code (groups of 10)'); ax.set_ylabel('sigma_pair (ps)'); ax.grid(alpha=.3); ax.legend()
    ax.set_title('Precision across the clock period')
    save(fig, a.out, 'F09_precision_vs_code.png')

    # F10 residual histograms by d_coarse (continuity across the join)
    fig, ax = plt.subplots(figsize=(8, 4))
    for s in (0, 1, -1):
        v = [r for r, ss, _ in R if ss == s]
        if len(v) > 100:
            ax.hist(v, bins=120, range=(-60, 60), density=True, histtype='step', lw=1.5,
                    label=f'd_coarse={s:+d}: n={len(v):,}, sigma={std(v):.2f} ps')
    ax.set_xlabel('calibrated A-B residual (ps)'); ax.set_ylabel('density'); ax.grid(alpha=.3); ax.legend()
    ax.set_title('Same-edge vs cross-edge pairs (continuity of the dead-zone fix)')
    save(fig, a.out, 'F10_residual_by_dcoarse.png')

    # F11 quantisation error per bin
    fig, ax = plt.subplots(figsize=(10, 4))
    for ch in (0, 1):
        codes, _, w = W[ch]; Wp = [x for x in w if x > 0]
        fl = math.sqrt(sum(x ** 3 for x in Wp) / (12 * sum(Wp)))
        ax.plot(codes, [x / math.sqrt(12) for x in w], '.', ms=3, label=f'chain {names[ch]} (floor {fl:.2f} ps)')
    ax.set_xlabel('code'); ax.set_ylabel('bin width / sqrt(12)  (ps)'); ax.grid(alpha=.3); ax.legend()
    ax.set_title('Quantisation error of each bin')
    save(fig, a.out, 'F11_quantisation_per_bin.png')

    # F12 stability during the capture (10 chunks)
    fig, ax = plt.subplots(figsize=(8, 4))
    for ch in (0, 1):
        ev, od = [], []
        for k in range(10):
            lo, hi = k * N // 10, (k + 1) * N // 10
            cnt = Counter(x for x in Dc[FK[ch]][lo:hi] if x not in RAIL); n = hi - lo
            codes = W[ch][0]; w = [cnt.get(c, 0) / n * P for c in codes]
            ev.append(sum(x for c, x in zip(codes, w) if c % 2 == 0) / sum(1 for c in codes if c % 2 == 0))
            od.append(sum(x for c, x in zip(codes, w) if c % 2 == 1) / sum(1 for c in codes if c % 2 == 1))
        ax.plot(range(1, 11), ev, 'o-', label=f'{names[ch]} even bins')
        ax.plot(range(1, 11), od, 's--', label=f'{names[ch]} odd bins')
    ax.set_xlabel('chunk (~1/10 of the capture)'); ax.set_ylabel('mean bin width (ps)'); ax.grid(alpha=.3)
    ax.legend(ncol=2); ax.set_title('Stability of bin widths during the capture')
    save(fig, a.out, 'F12_stability_over_time.png')

    # DPS sweep figures
    if a.dps:
        Dd = load(a.dps)
        acc = {t: defaultdict(lambda: [[0, 0.0, 0.0], [0, 0.0, 0.0]]) for t in ('all', 'up', 'dn')}
        rail = defaultdict(lambda: [0, 0]); tot = Counter(); direction, last = 1, None
        for ph, fa, fb, va, vb in zip(Dd['phase'], Dd['fine_a'], Dd['fine_b'], Dd['valid_a'], Dd['valid_b']):
            if last is not None and ph != last:
                d = ph - last; d = d - 280 if d > 140 else (d + 280 if d < -140 else d)
                if d:
                    direction = 1 if d > 0 else -1
            last = ph; tot[ph] += 1
            for ch, (f, v) in enumerate(((fa, va), (fb, vb))):
                if f in RAIL:
                    rail[ph][ch] += 1; continue
                if not v:
                    continue
                for t in ('all', 'up' if direction > 0 else 'dn'):
                    s = acc[t][ph][ch]; s[0] += 1; s[1] += f; s[2] += f * f
        phs = sorted(tot)
        mean = lambda s: s[1] / s[0] if s[0] else float('nan')
        sd = lambda s: math.sqrt(max(s[2] / s[0] - (s[1] / s[0]) ** 2, 0)) if s[0] else float('nan')
        fig, ax = plt.subplots(figsize=(10, 4))
        for ch in (0, 1):
            ax.plot([p * STEP for p in phs], [mean(acc['all'][p][ch]) for p in phs], '.', ms=4, label=f'chain {names[ch]}')
        for p in phs:
            if rail[p][0] > 0.5 * tot[p]:
                ax.axvspan(p * STEP, (p + 1) * STEP, color='red', alpha=0.15, lw=0)
        ax.set_xlabel('event phase (ps)'); ax.set_ylabel('mean code'); ax.grid(alpha=.3); ax.legend()
        ax.set_title('DPS sweep: transfer curve (red = dead zone, pre-fix build)')
        save(fig, a.out, 'F13_dps_transfer_curve.png')

        fig, ax = plt.subplots(figsize=(10, 3.5))
        for ch in (0, 1):
            ax.plot([p * STEP for p in phs], [mean(acc['up'][p][ch]) - mean(acc['dn'][p][ch]) for p in phs],
                    '.', ms=4, label=f'chain {names[ch]}')
        ax.set_xlabel('event phase (ps)'); ax.set_ylabel('up - down (codes)'); ax.grid(alpha=.3); ax.legend()
        ax.set_title('DPS hysteresis: up-sweep minus down-sweep')
        save(fig, a.out, 'F14_dps_hysteresis.png')

        fig, ax = plt.subplots(figsize=(10, 3.5))
        for ch in (0, 1):
            ax.plot([p * STEP for p in phs], [sd(acc['all'][p][ch]) for p in phs], '.', ms=4, label=f'chain {names[ch]}')
        ax.set_xlabel('event phase (ps)'); ax.set_ylabel('std of code (codes)'); ax.grid(alpha=.3); ax.legend()
        ax.set_title('Code spread at a fixed phase (noise + bin-edge effects)')
        save(fig, a.out, 'F15_dps_code_spread.png')

    # dead zone before / after
    if a.old:
        Do = load(a.old)
        dz = lambda D, ch: sum(1 for x in D[FK[ch]] if x in RAIL) / nrows(D) * P
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.bar([0 - 0.2, 1 - 0.2], [dz(Do, 0), dz(Do, 1)], width=0.4, label='before fix')
        ax.bar([0 + 0.2, 1 + 0.2], [dz(Dc, 0), dz(Dc, 1)], width=0.4, label='after fix')
        ax.set_xticks([0, 1]); ax.set_xticklabels(['chain A', 'chain B'])
        ax.set_ylabel('dead zone (ps of 5000)'); ax.grid(alpha=.3); ax.legend()
        ax.set_title('Dead zone before / after the dual-snapshot fix')
        save(fig, a.out, 'F16_dead_zone_before_after.png')

    # build-to-build change
    if a.build:
        def lw(p):
            return {int(r['code']): float(r['bin_width_ps']) for r in csv.DictReader(open(p, newline=''))}
        fig, ax = plt.subplots(2, 1, figsize=(11, 5), sharex=True)
        for ch, s in ((0, 'a'), (1, 'b')):
            w1, w2 = lw(f"{a.build[0]}_{s}.csv"), lw(f"{a.build[1]}_{s}.csv")
            com = sorted(set(w1) & set(w2))
            for m, col, lab in ((0, 'tab:red', 'code%4=0 (slice boundary)'), (1, 'tab:blue', 'other codes')):
                cs = [c for c in com if (c % 4 == 0) == (m == 0)]
                ax[ch].bar(cs, [w2[c] - w1[c] for c in cs], width=1.0, color=col, label=lab)
            ax[ch].set_ylabel(f'chain {names[ch]}\nwidth change (ps)'); ax[ch].grid(alpha=.3); ax[ch].legend()
        ax[1].set_xlabel('code'); fig.suptitle('Bin widths change between two implementations of the same design')
        save(fig, a.out, 'F17_build_to_build_change.png')

    print(f"done -> {a.out}\\")


if __name__ == "__main__":
    main()