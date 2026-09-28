#!/usr/bin/env python3
"""dump_taps.py -- reads frame v5 (DUMP=1 builds): raw channel-A folding snapshots.

Frame (26 bytes): 0xC3 | raw[159:0] MSB first | fine[15:8] fine[7:0] | valid | seq | CRC-8
Checks the board's fine/valid against fold_model.decode on every snapshot and
prints what the fold really looks like. Run from code/ (needs fold_model.py).

    python dump_taps.py --port COM19 --seconds 30 --out ..\\data\\raw\\dump_x.csv
    python dump_taps.py --analyse ..\\data\\raw\\dump_x.csv [--show 20]
"""
import argparse, csv, sys, time
from collections import Counter
import fold_model as fm

HEADER, FLEN = 0xC3, 26


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def parse(buf):
    out, skipped, bad = [], 0, 0
    while len(buf) >= FLEN:
        if buf[0] != HEADER:
            buf.pop(0); skipped += 1; continue
        f = bytes(buf[:FLEN])
        if crc8(f[:25]) != f[25]:
            bad += 1; buf.pop(0); continue
        del buf[:FLEN]
        raw = int.from_bytes(f[1:21], "big")
        fine = ((f[21] << 8) | f[22]) & 0x3FF
        out.append(dict(raw=f"{raw & ((1 << fm.SW) - 1):040x}", fine=fine, valid=f[23] & 1, seq=f[24]))
    return out, skipped, bad


def as_string(bits):
    l = "".join(str(b) for b in bits[:fm.LAUNCH_W])
    f = "".join(str(b) for b in bits[fm.LAUNCH_W:fm.LAUNCH_W + fm.FOLD_W])
    c = "".join(str(b) for b in bits[fm.LAUNCH_W + fm.FOLD_W:])
    return f"L {l} | F {f} | C {c}"


def transitions(bits):
    return sum(1 for i in range(len(bits) - 1) if bits[i] != bits[i + 1])


def analyse(rows, show):
    if not rows:
        sys.exit("no frames")
    mism, n_valid, laps, codes, raw_edges, bubbles = 0, 0, Counter(), Counter(), Counter(), 0
    for i, r in enumerate(rows):
        bits = fm.from_hex(r["raw"])
        fine, valid = fm.decode(bits)
        if fine != r["fine"] or valid != r["valid"]:
            mism += 1
            if mism <= 10:
                print(f"  MISMATCH seq {r['seq']}: board fine={r['fine']} valid={r['valid']} "
                      f"model fine={fine} valid={valid}")
                print("   ", as_string(bits))
        n_valid += r["valid"]
        fold = bits[fm.LAUNCH_W:fm.LAUNCH_W + fm.FOLD_W]
        t_raw = transitions(fold)
        raw_edges[t_raw] += 1
        t_clean = transitions(fm.majority5(fold))
        bubbles += max(0, t_raw - t_clean)
        if r["valid"] and r["fine"] >= fm.LAUNCH_W:
            laps[(r["fine"] - fm.LAUNCH_W) // fm.FOLD_W] += 1
        codes[r["fine"]] += 1
        if i < show:
            print(f"seq {r['seq']:3d} fine {r['fine']:4d} valid {r['valid']}  {as_string(bits)}")
    n = len(rows)
    print(f"\n{n} snapshots: valid {n_valid} ({100 * n_valid / n:.1f}%), board/model mismatches {mism}")
    print("raw transitions in the fold (before bubble filter):",
          ", ".join(f"{k}: {v}" for k, v in sorted(raw_edges.items())))
    print(f"bubbles removed by the filter: {bubbles} ({bubbles / n:.2f} per snapshot)")
    print("laps (n) among valid fold codes:", ", ".join(f"{k}: {v}" for k, v in sorted(laps.items())))
    lo, hi = min(codes), max(codes)
    print(f"code range {lo}..{hi}; codes never seen in range: "
          f"{sum(1 for c in range(lo, hi + 1) if c not in codes)}")
    print("MODEL MATCH PASS" if mism == 0 else "MODEL MATCH FAIL")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port"); ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--seconds", type=float, default=30); ap.add_argument("--out")
    ap.add_argument("--analyse"); ap.add_argument("--show", type=int, default=12)
    a = ap.parse_args()
    if a.analyse:
        rows = [dict(raw=r["raw"], fine=int(r["fine"]), valid=int(r["valid"]), seq=int(r["seq"]))
                for r in csv.DictReader(open(a.analyse, newline=""))]
        analyse(rows, a.show); return
    if not (a.port and a.out):
        sys.exit("need --port and --out, or --analyse")
    import serial
    ser = serial.Serial(a.port, a.baud, timeout=0.1); ser.reset_input_buffer()
    buf, rows, t0, skipped, bad = bytearray(), [], time.time(), 0, 0
    while time.time() - t0 < a.seconds:
        buf += ser.read(4096)
        got, s, b = parse(buf); rows += got; skipped += s; bad += b
    ser.close()
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["seq", "raw", "fine", "valid"]); w.writeheader()
        for r in rows: w.writerow(r)
    print(f"{len(rows)} frames in {a.seconds:.0f} s -> {a.out}  (CRC errors {bad}, bytes skipped {skipped})")
    analyse(rows, a.show)


if __name__ == "__main__":
    main()
