#!/usr/bin/env python3
"""read_rp.py -- return-path probe (rp_board): capture, decode, analyse.

Frame (12 bytes): 0xB5 ring K_hi K_lo GATE_BITS DIV_BITS count[4] seq CRC8
    f_ring = count * 2^DIV / (2^GATE * 10 ns)      T_lap = 1 / (2 f_ring)

T_lap(K) is the folding lap time for a fold whose B->D span is K taps:
    T_lap = t(B->D) + t_return        folding factor = 5000 ps / T_lap
With a reference tap delay tau (from a TDC LUT, or --tau):
    t_return(K) = T_lap(K) - K*tau
    K_DE >= t_return / tau    (edge must still be on the fold when its copy
                               re-enters at B -> no big bin at the fold)
    fold length L = K + K_DE

Usage (PowerShell, one per line):
    python read_rp.py --port COM19 --seconds 120 --out data\\raw\\rp_x.csv
    python read_rp.py --analyse data\\raw\\rp_x.csv --lut data\\lut_x_a.csv
    python read_rp.py --from-bytes rp_uart.txt --sim-check        (xsim output)
"""
import argparse, csv, math, sys, time

HEADER, FLEN, CLK_NS, PERIOD_PS = 0xB5, 12, 10.0, 5000.0
FIELDS = ["t_host", "seq", "ring", "k", "gate_bits", "div_bits", "count", "tlap_ps"]


def crc8(data):
    c = 0
    for byte in data:
        c ^= byte
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def tlap_ps(count, gate_bits, div_bits):
    if count == 0:
        return float("nan")
    f_hz = count * (1 << div_bits) / ((1 << gate_bits) * CLK_NS * 1e-9)
    return 1e12 / (2.0 * f_hz)


class Parser:
    def __init__(self):
        self.buf, self.bad_crc, self.skipped = bytearray(), 0, 0

    def feed(self, data):
        self.buf += data
        out = []
        while len(self.buf) >= FLEN:
            if self.buf[0] != HEADER:
                self.buf.pop(0); self.skipped += 1; continue
            f = bytes(self.buf[:FLEN])
            if crc8(f[:11]) != f[11]:
                self.bad_crc += 1; self.buf.pop(0); continue
            del self.buf[:FLEN]
            ring, k, gb, db = f[1], (f[2] << 8) | f[3], f[4], f[5]
            cnt = int.from_bytes(f[6:10], "big")
            out.append(dict(seq=f[10], ring=ring, k=k, gate_bits=gb, div_bits=db,
                            count=cnt, tlap_ps=tlap_ps(cnt, gb, db)))
        return out


def tau_from_lut(path, lo, hi):
    t = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            t[int(r["code"])] = float(r["t_ps"])
    if lo not in t or hi not in t:
        sys.exit(f"LUT {path} has no code {lo} or {hi}")
    return (t[lo] - t[hi]) / (hi - lo)       # t_ps falls as the code rises


def mean_sd(v):
    m = sum(v) / len(v)
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1)) if len(v) > 1 else 0.0
    return m, sd


def analyse(rows, tau=None, drift_frac=0.1):
    by = {}
    for r in rows:
        if not math.isnan(r["tlap_ps"]):
            by.setdefault(r["k"], []).append(r["tlap_ps"])
    if not by:
        sys.exit("no usable frames")
    ks = sorted(by)
    print(f"\n  {'K':>4} {'n':>5} {'T_lap ps':>10} {'sd ps':>7} {'drift ps':>9} "
          f"{'laps/5ns':>9}" + ("" if tau is None else
          f" {'t_ret ps':>9} {'K_DE':>6} {'L taps':>7}"))
    means = []
    for k in ks:
        v = by[k]
        m, sd = mean_sd(v)
        nd = max(1, int(len(v) * drift_frac))
        drift = sum(v[-nd:]) / nd - sum(v[:nd]) / nd
        means.append(m)
        line = f"  {k:4d} {len(v):5d} {m:10.1f} {sd:7.2f} {drift:+9.2f} {PERIOD_PS / m:9.2f}"
        if tau is not None:
            tr = m - k * tau
            kde = math.ceil(tr / tau)
            line += f" {tr:9.1f} {kde:6d} {k + kde:7d}"
        print(line)

    if len(ks) >= 2:
        n = len(ks)
        mx, my = sum(ks) / n, sum(means) / n
        sxx = sum((x - mx) ** 2 for x in ks)
        b = sum((x - mx) * (y - my) for x, y in zip(ks, means)) / sxx
        a = my - b * mx
        res = [y - (a + b * x) for x, y in zip(ks, means)]
        print(f"\n  fit T_lap = a + b*K :  a = {a:.1f} ps (intercept),  b = {b:.3f} ps/tap")
        print(f"  fit residuals ps    :  " + "  ".join(f"{x:+.1f}" for x in res))
        if tau is not None:
            print(f"  tau_ref             :  {tau:.3f} ps/tap   (b - tau = {b - tau:+.3f} ps/tap"
                  f" = return-route growth per tap of span, if tau_ref applies here)")
        print("\n  K needed for folding factor k (from the fit):")
        for fk in (2, 3, 4, 5, 6, 8):
            kk = (PERIOD_PS / fk - a) / b
            flag = "" if ks[0] <= kk <= ks[-1] else "  (extrapolated)"
            if kk <= 0:
                print(f"    k = {fk}: not reachable, lap time floor a = {a:.0f} ps > {PERIOD_PS / fk:.0f} ps")
                continue
            s = f"    k = {fk}: K = {kk:6.1f}{flag}"
            if tau is not None:
                tr = a + b * kk - kk * tau
                s += f",  t_ret = {tr:6.1f} ps,  L = {kk + tr / tau:6.1f} taps"
            print(s)
    return by


def sim_check(rows):
    """Behavioural ring model in rp_board: T_lap = 0.55 ns + 17 ps * K."""
    seen, bad = set(), 0
    for r in rows:
        exp = 550.0 + 17.0 * r["k"]
        err = (r["tlap_ps"] - exp) / exp
        seen.add(r["ring"])
        ok = abs(err) < 0.01
        bad += not ok
        print(f"  seq {r['seq']:3d} ring {r['ring']} K {r['k']:3d} count {r['count']:6d} "
              f"T_lap {r['tlap_ps']:8.1f} ps  model {exp:8.1f}  err {100 * err:+.3f} %"
              f"{'' if ok else '  <-- FAIL'}")
    if seen != set(range(6)):
        print(f"  rings seen {sorted(seen)}, expected 0..5"); bad += 1
    print("SIM CHECK PASS" if bad == 0 else f"SIM CHECK FAIL ({bad})")
    return bad == 0


def load_csv(path):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            rows.append({k: (float(r[k]) if k in ("t_host", "tlap_ps") else int(r[k]))
                         for k in FIELDS})
    return rows


def main():
    ap = argparse.ArgumentParser(description="Return-path probe reader")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--out", help="CSV to write (capture mode)")
    ap.add_argument("--analyse", help="CSV from an earlier capture")
    ap.add_argument("--from-bytes", help="hex-per-line dump from tb_rp")
    ap.add_argument("--sim-check", action="store_true")
    ap.add_argument("--lut", help="TDC LUT csv (code,t_ps,...) for tau_ref")
    ap.add_argument("--lut-range", nargs=2, type=int, default=[80, 276],
                    help="codes spanning the Y50-Y99 clock region (chain A: 80 276)")
    ap.add_argument("--tau", type=float, help="tau_ref in ps/tap (instead of --lut)")
    a = ap.parse_args()

    tau = a.tau
    if a.lut:
        tau = tau_from_lut(a.lut, *a.lut_range)
        print(f"tau_ref from {a.lut} codes {a.lut_range[0]}..{a.lut_range[1]}: {tau:.3f} ps/tap")

    if a.from_bytes:
        p = Parser()
        data = bytes(int(x, 16) for x in open(a.from_bytes).read().split())
        rows = p.feed(data)
        for r in rows:
            r["t_host"] = 0.0
        print(f"{len(rows)} frames, {p.bad_crc} CRC errors, {p.skipped} bytes skipped")
        if a.sim_check:
            ok = sim_check(rows) and p.bad_crc == 0
            sys.exit(0 if ok else 1)
        analyse(rows, tau)
        return

    if a.analyse:
        analyse(load_csv(a.analyse), tau)
        return

    if not (a.port and a.out):
        sys.exit("capture mode needs --port and --out")
    import serial
    ser = serial.Serial(a.port, a.baud, timeout=0.1)
    ser.reset_input_buffer()
    p, rows, t0, last_seq, gaps = Parser(), [], time.time(), None, 0
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        while time.time() - t0 < a.seconds:
            for r in p.feed(ser.read(4096)):
                r["t_host"] = round(time.time() - t0, 3)
                if last_seq is not None and r["seq"] != (last_seq + 1) & 0xFF:
                    gaps += 1
                last_seq = r["seq"]
                rows.append(r)
                w.writerow({k: r[k] for k in FIELDS})
    ser.close()
    print(f"{len(rows)} frames in {a.seconds:.0f} s -> {a.out}")
    print(f"CRC errors {p.bad_crc}, seq gaps {gaps}, bytes skipped {p.skipped}")
    analyse(rows, tau)


if __name__ == "__main__":
    main()
