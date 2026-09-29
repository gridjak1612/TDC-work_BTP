#!/usr/bin/env python3
"""fold_model.py -- bit-exact reference for fold_decode.v, plus a synthetic
snapshot generator with a physical model of the folding chain.

    python fold_model.py --gen fold_vectors.txt [--n 3000] [--seed 1]
        writes one line per snapshot:  <hex snapshot> <fine> <valid>
        and self-checks that the decoded code tracks elapsed time.
    python fold_model.py --check fold_vectors.txt fold_tb_out.txt
        compares the testbench's outputs with the model (used by tb_fold_decode).

Geometry (rev 2, fold 136): LAUNCH_W 32, FOLD_W 136 (B = tap 32 .. E = tap 167),
NCNT 7 counting taps at 168 + 30 j, chain 352 taps. Laps are ~116-131 taps
depending on chain and polarity, always shorter than the fold, so two edges can
briefly share the fold (overlap) but the fold is never empty (no gap).

DECODE RULE (mirrors fold_decode.v). Every edge still inside the fold is a
valid time reference, so on a lap boundary the decoder reports the OLDER
edge's position (its lap is what the counting taps say); the newer edge, if
already at B, is only used to find where the older one is.
  n     = transitions along {fold[E], cnt[0..6]}   (edges that left the fold)
  ref   = 1 if n even else 0                      (level under edge n)
  x     = fold XNOR ref                           (1s under edge n)
          single edge : x = 1^p 0^rest            pos = popcount(x) = p
          front at B  : x = all 0                 pos = 0
          fold empty  : x = all 1                 pos = FOLD_W (= next lap, pos 0)
          overlap     : x = 0^a 1^(q-a) 0^rest    (newer edge at a, older at q >= FOLD_W-OVL_TOP)
                        pos = q = popcount(x) + FOLD_W - popcount(x | TOP)  (TOP = top OVL_TOP bits)
  ovl   = x[0] == 0 and x not all 0
  code  = launch popcount if front still in launch, else LAUNCH_W + n*FOLD_W + pos
  valid = launch clean step & (in_launch | (x has <= 1 falling transition
          & (x[top] == 0 | x all 1) & (!ovl | x[FOLD_W-OVL_TOP-1] == 1) & n <= N_MAX))
"""
import argparse, random, sys

LAUNCH_W, FOLD_W, NCNT, CNT_STEP, CHAIN = 32, 136, 7, 30, 352
OVL_TOP, N_MAX, FINE_BITS = 28, 4, 10
SW = LAUNCH_W + FOLD_W + NCNT
E_TAP = LAUNCH_W + FOLD_W - 1


def majority5(bits):
    w = len(bits)
    out = [0] * w
    out[0] = 1 if sum(bits[0:3]) >= 1 else 0
    out[1] = 1 if sum(bits[0:4]) >= 2 else 0
    for i in range(2, w - 2):
        out[i] = 1 if sum(bits[i-2:i+3]) >= 3 else 0
    out[w-2] = 1 if sum(bits[w-4:w]) >= 3 else 0
    out[w-1] = 1 if sum(bits[w-3:w]) >= 3 else 0
    return out


def therm_valid(bits):
    """thermometer_validator_piped: at most one 1->0 transition."""
    return sum(1 for i in range(len(bits) - 1) if bits[i] and not bits[i+1]) <= 1


def decode(bits):
    """bits: list of SW ints, LSB first. Returns (fine, valid) exactly as fold_decode."""
    launch = majority5(bits[:LAUNCH_W])
    fold = majority5(bits[LAUNCH_W:LAUNCH_W+FOLD_W])
    cnt = bits[LAUNCH_W+FOLD_W:]
    seq = [fold[-1]] + cnt
    n = sum(seq[j] ^ seq[j+1] for j in range(NCNT))
    ref = 0 if (n & 1) else 1
    x = [1 - (b ^ ref) for b in fold]               # XNOR: 1s under edge n
    ones = sum(x)
    xt = x[:FOLD_W - OVL_TOP] + [1] * OVL_TOP        # x | TOP
    ones_t = sum(xt)
    all_zero, all_ones = (ones == 0), (ones == FOLD_W)
    ovl = (x[0] == 0) and not all_zero
    pos = (ones + FOLD_W - ones_t) if ovl else ones
    launch_cnt = sum(launch)
    in_launch = launch_cnt < LAUNCH_W
    if in_launch:
        fine = launch_cnt
    else:
        fine = min(LAUNCH_W + n * FOLD_W + pos, (1 << FINE_BITS) - 1)
    x_ok = therm_valid(x) and (x[-1] == 0 or all_ones) and (not ovl or x[FOLD_W - OVL_TOP - 1] == 1)
    valid = therm_valid(launch) and (launch[0] == 1 or launch_cnt == 0) and \
        (in_launch or (x_ok and n <= N_MAX))
    return fine, int(valid)


def code_to_taps(code, laps):
    """Elapsed time (taps) implied by a code, for the tracking self-check."""
    if code < LAUNCH_W:
        return code
    n, pos = divmod(code - LAUNCH_W, FOLD_W)
    return LAUNCH_W + sum(laps[m % len(laps)] for m in range(n)) + pos


# ----------------------------------------------------------- physical model
def snapshot(t_taps, rng, laps=(120, 117, 131, 124), bubble_p=0.0):
    """Chain state t_taps tap-delays after the hit. Edge m enters B at
    LAUNCH_W + sum(laps[:m]); each edge then moves one tap per tap-delay.
    Level at a position = number of edges past it, mod 2 (pre-hit level 0).
    Returns SW bits: launch, fold, counting."""
    bits = [1 if t_taps > i else 0 for i in range(LAUNCH_W)]
    starts = []
    s = LAUNCH_W
    for m in range(8):
        starts.append(s)
        s += laps[m % len(laps)]

    def level(tap):
        passed = 0
        for st in starts:
            p = t_taps - st                 # edge position from B
            if p <= 0:
                break
            if LAUNCH_W + p > tap:
                passed += 1
        return passed & 1
    for i in range(FOLD_W):
        bits.append(level(LAUNCH_W + i))
    for j in range(NCNT):
        bits.append(level(LAUNCH_W + FOLD_W + CNT_STEP * j))
    if bubble_p > 0:
        for st in starts:
            p = t_taps - st
            if p <= 0:
                break
            tap = LAUNCH_W + p               # absolute tap of this edge
            if rng.random() < bubble_p:
                i = int(round(tap + rng.uniform(-1.5, 1.5)))
                lo = 1 if st == LAUNCH_W else LAUNCH_W
                if lo <= i < LAUNCH_W + FOLD_W - 1:
                    bits[i] ^= 1
    return bits


def to_hex(bits):
    v = 0
    for i, b in enumerate(bits):
        v |= (b & 1) << i
    return f"{v:0{(SW + 3) // 4}x}"


def from_hex(s):
    v = int(s, 16)
    return [(v >> i) & 1 for i in range(SW)]


def gen(path, n, seed):
    rng = random.Random(seed)
    lines, bad_track, n_valid = [], 0, 0
    t_max = CHAIN - 8
    laps_sets = [(120, 117, 131, 124), (120, 120, 120, 120), (117, 116, 118, 117), (131, 128, 131, 130)]
    for i in range(n):
        laps = laps_sets[i % len(laps_sets)]
        kind = rng.random()
        if kind < 0.05:
            t = rng.uniform(-20, 0)
        elif kind < 0.15:
            t = rng.uniform(0, LAUNCH_W)
        elif kind < 0.35:                            # lap boundaries incl. overlaps
            m = rng.randrange(0, 3)
            t = LAUNCH_W + sum(laps[:m]) + rng.uniform(-3, 25)
        else:
            t = rng.uniform(0, t_max)
        bubble_p = 0.0 if i % 2 == 0 else 0.3
        bits = snapshot(t, rng, laps=laps, bubble_p=bubble_p)
        fine, valid = decode(bits)
        if valid and t > 0:
            t_est = code_to_taps(fine, laps)
            if abs(t_est - t) > 4:
                bad_track += 1
                if bad_track <= 5:
                    print(f"  track: t={t:.2f} laps={laps} code={fine} -> t_est={t_est}")
        n_valid += valid
        lines.append(f"{to_hex(bits)} {fine} {valid}")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"{n} vectors -> {path}: {n_valid} valid, {bad_track} valid codes off by >4 taps in time")
    return bad_track == 0


def check(vec_path, out_path):
    exp = [l.split() for l in open(vec_path) if l.strip()]
    got = [l.split() for l in open(out_path) if l.strip()]
    if len(got) != len(exp):
        print(f"MODEL CHECK FAIL: {len(got)} outputs for {len(exp)} vectors"); return False
    bad = 0
    for i, (e, g) in enumerate(zip(exp, got)):
        if int(e[1]) != int(g[0]) or int(e[2]) != int(g[1]):
            bad += 1
            if bad <= 10:
                print(f"  vec {i}: model fine={e[1]} valid={e[2]}  rtl fine={g[0]} valid={g[1]}")
    print("MODEL CHECK PASS" if bad == 0 else f"MODEL CHECK FAIL: {bad} mismatches")
    return bad == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gen"); ap.add_argument("--n", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=1); ap.add_argument("--check", nargs=2)
    a = ap.parse_args()
    if a.gen:
        sys.exit(0 if gen(a.gen, a.n, a.seed) else 1)
    if a.check:
        sys.exit(0 if check(*a.check) else 1)
    ap.print_help()


if __name__ == "__main__":
    main()
