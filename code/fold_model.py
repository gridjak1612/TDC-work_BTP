#!/usr/bin/env python3
"""fold_model.py -- bit-exact reference for fold_decode.v, plus a synthetic
snapshot generator with a physical model of the folding chain.

    python fold_model.py --gen fold_vectors.txt [--n 3000] [--seed 1]
        writes one line per snapshot:  <hex snapshot> <fine> <valid>
        and self-checks that the decoded code tracks elapsed time.
    python fold_model.py --check fold_vectors.txt fold_tb_out.txt
        compares the testbench's outputs with the model (used by tb_fold_decode).

Geometry defaults match tdc_channel_fold: LAUNCH_W 32, FOLD_W 120, NCNT 7,
CNT_STEP 32, chain 352 taps, K 64 (lap = FOLD_W taps by construction).
"""
import argparse, random, sys

LAUNCH_W, FOLD_W, NCNT, CNT_STEP, CHAIN = 32, 120, 7, 32, 352
N_MAX, FINE_BITS = 4, 10
SW = LAUNCH_W + FOLD_W + NCNT


# ----------------------------------------------------------- RTL mirror
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
    t = sum(1 for i in range(len(bits) - 1) if bits[i] and not bits[i+1])
    return t <= 1


def decode(bits):
    """bits: list of SW ints, LSB first. Returns (fine, valid) exactly as fold_decode."""
    launch = majority5(bits[:LAUNCH_W])
    fold = majority5(bits[LAUNCH_W:LAUNCH_W+FOLD_W])
    cnt = bits[LAUNCH_W+FOLD_W:]
    seq = [fold[-1]] + cnt
    n = sum(seq[j] ^ seq[j+1] for j in range(NCNT))
    ref = 0 if (n & 1) else 1
    x = [b ^ ref for b in fold]
    launch_cnt = sum(launch)
    ones = sum(x)
    in_launch = launch_cnt < LAUNCH_W
    if in_launch:
        fine = launch_cnt
    else:
        full = LAUNCH_W + n * FOLD_W + (FOLD_W - ones)
        fine = min(full, (1 << FINE_BITS) - 1)
    valid = therm_valid(launch) and (launch[0] == 1 or launch_cnt == 0) and \
        (in_launch or (therm_valid(x) and (x[-1] == 1 or ones == 0) and n <= N_MAX))
    return fine, int(valid)


# ----------------------------------------------------------- physical model
def snapshot(t_taps, rng, bubble_p=0.0, jitter=0.0):
    """Chain state t_taps tap-delays after the hit (float). Returns SW bits.
    Edges: front enters the fold at LAUNCH_W; edge m enters at LAUNCH_W + m*FOLD_W
    (lap = FOLD_W taps). Level at a position = number of edges past it, mod 2."""
    bits = []
    for i in range(LAUNCH_W):                       # launch: plain step
        bits.append(1 if t_taps > i else 0)
    def level(pos_from_B):
        passed = 0
        m = 0
        while True:
            p = t_taps - LAUNCH_W - m * FOLD_W      # edge m position from B
            if p <= 0:
                break
            if p > pos_from_B + rng.uniform(-jitter, jitter):
                passed += 1
            m += 1
        return passed & 1
    for i in range(FOLD_W):
        bits.append(level(i))
    for j in range(NCNT):
        bits.append(level(FOLD_W + CNT_STEP * j))
    if bubble_p > 0:
        # Real bubbles: taps within one position of a PROPAGATING edge sample
        # metastably. Flip at most one tap per edge, never elsewhere.
        m = 0
        while True:
            p = t_taps - m * FOLD_W                 # absolute tap index of edge m
            if p <= 0 or (m > 0 and p <= LAUNCH_W):
                break
            if rng.random() < bubble_p:
                i = int(round(p + rng.uniform(-1.0, 1.0)))
                lo = 1 if m == 0 else LAUNCH_W      # return edges start AT B
                if lo <= i < LAUNCH_W + FOLD_W - 1:
                    bits[i] ^= 1
            m += 1
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
    t_max = CHAIN - 8                                # keep the front on the chain
    for i in range(n):
        kind = rng.random()
        if kind < 0.05:
            t = rng.uniform(-20, 0)                  # before the hit
        elif kind < 0.15:
            t = rng.uniform(0, LAUNCH_W)             # front in launch
        elif kind < 0.25:                            # lap boundaries
            m = rng.randrange(0, 3)
            t = LAUNCH_W + m * FOLD_W + rng.uniform(-1.5, 1.5)
        else:
            t = rng.uniform(0, t_max)
        bubble_p = 0.0 if i % 2 == 0 else 0.3
        bits = snapshot(t, rng, bubble_p=bubble_p, jitter=0.0)
        fine, valid = decode(bits)
        # Concept check: a valid code must track elapsed time (in taps) within
        # the bubble filter's reach. Boundary and pre-hit cases are exempt.
        # (> 4, not > 3: the majority filter's top-boundary rule can pull an edge
        #  in the last three fold taps up to E when one of them is a bubble.)
        if valid and t > 0 and abs(fine - t) > 4:
            bad_track += 1
        n_valid += valid
        lines.append(f"{to_hex(bits)} {fine} {valid}")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"{n} vectors -> {path}: {n_valid} valid, {bad_track} valid codes off by >4 taps")
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
    ap.add_argument("--gen")
    ap.add_argument("--n", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--check", nargs=2)
    a = ap.parse_args()
    if a.gen:
        sys.exit(0 if gen(a.gen, a.n, a.seed) else 1)
    if a.check:
        sys.exit(0 if check(*a.check) else 1)
    ap.print_help()


if __name__ == "__main__":
    main()
