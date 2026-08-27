#!/usr/bin/env python3
"""
read_tdc_dual.py -- host reader for the TWO-CHANNEL interval TDC.

FRAME (5 bytes per measurement, 8N1 @ 115200):

    byte0 : 0xAA                                 sync header
    byte1 : fine_a[7:0]                          START fine  (raw taps, chain A)
    byte2 : fine_b[7:0]                          STOP  fine  (raw taps, chain B)
    byte3 : d_coarse[7:0]
    byte4 : {valid_b, valid_a, d_coarse[13:8]}
              ^bit7    ^bit6   ^bits5:0

RECONSTRUCTION

    interval = d_coarse * 5.000 ns  -  ( fine_b * tau_b  -  fine_a * tau_a )

*** THE FINE DIFFERENCE IS SIGNED. ***
fine_b - fine_a is negative about half the time, and that is CORRECT -- it just
means the STOP landed later inside its clock window than the START did inside
its. If you compute it in 8-bit unsigned arithmetic it wraps, and every affected
interval comes out exactly one chain-span (~4 ns) wrong, silently. Python ints
are arbitrary precision so this is safe here, but it WILL bite you if you ever
port this to C or to the FPGA.

*** tau_a != tau_b. ***
Chain A and chain B are different physical carry chains. They have different
per-tap delays AND different per-bin widths. Until you run code-density
calibration, both default to 16 ps and every number below is a FIRST-ORDER
ESTIMATE. Do not quote a resolution from it.

Usage:
    python read_tdc_dual.py COM19
    python read_tdc_dual.py COM19 --tau-a 16.0 --tau-b 16.4
    python read_tdc_dual.py COM19 --expect 21.008      # known cable/target delay
"""

import sys
import csv
import argparse
import statistics
from collections import Counter

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed.  Run:  pip install pyserial")


T_CLK_NS    = 5.000          # 200 MHz
COARSE_BITS = 14
FINE_MAX    = 255            # saturated
WRAP_NS     = (1 << COARSE_BITS) * T_CLK_NS      # 81920.0 ns


def parse_args():
    p = argparse.ArgumentParser(description="Two-channel interval TDC reader")
    p.add_argument("port")
    p.add_argument("baud", nargs="?", type=int, default=115200)
    p.add_argument("--tau-a", type=float, default=16.0, help="chain A per-tap delay, ps")
    p.add_argument("--tau-b", type=float, default=16.0, help="chain B per-tap delay, ps")
    p.add_argument("--expect", type=float, default=None,
                   help="known true interval in ns (cable/target). Prints the residual.")
    p.add_argument("--out", default="tdc_dual_log.csv")
    return p.parse_args()


def main():
    a = parse_args()
    tau_a = a.tau_a / 1000.0     # ps -> ns
    tau_b = a.tau_b / 1000.0

    ser = serial.Serial(a.port, a.baud, timeout=1)
    print(f"Listening on {a.port} @ {a.baud} 8N1.  Ctrl-C to stop.")
    print(f"T_clk = {T_CLK_NS:.3f} ns   tau_a = {a.tau_a:.2f} ps   tau_b = {a.tau_b:.2f} ps")
    print(f"interval = d_coarse x 5.000 ns - (fine_b x tau_b - fine_a x tau_a)")
    print("-" * 88)

    n = 0
    hist_a, hist_b = Counter(), Counter()
    good_intervals = []
    n_bad = 0

    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "d_coarse", "fine_a", "fine_b",
                    "valid_a", "valid_b", "usable", "interval_ns"])

        buf = bytearray()
        try:
            while True:
                b = ser.read(1)
                if not b:
                    continue
                buf.append(b[0])

                if buf[0] != 0xAA:        # resync on the header
                    buf.clear()
                    continue
                if len(buf) < 5:
                    continue

                _, fa, fb, dc_lo, hi = buf
                buf.clear()

                fine_a   = fa
                fine_b   = fb
                d_coarse = ((hi & 0x3F) << 8) | dc_lo
                valid_a  = (hi >> 6) & 1
                valid_b  = (hi >> 7) & 1

                # ---- SIGNED difference. Python ints; never do this in uint8. ----
                interval = d_coarse * T_CLK_NS - (int(fine_b) * tau_b - int(fine_a) * tau_a)

                # A sample is only usable if BOTH codes are legal thermometers and
                # NEITHER chain railed. A railed chain (0 or 255) carries no phase
                # information -- see the dead-zone analysis.
                sat_a   = fine_a in (0, FINE_MAX)
                sat_b   = fine_b in (0, FINE_MAX)
                usable  = valid_a and valid_b and not sat_a and not sat_b

                n += 1
                flags = ""
                if not valid_b and d_coarse == 0 and fine_b == 0:
                    flags += "  [NO STOP - timeout]"
                if sat_a:
                    flags += "  [A railed]"
                if sat_b:
                    flags += "  [B railed]"

                if usable:
                    good_intervals.append(interval)
                    hist_a[fine_a] += 1
                    hist_b[fine_b] += 1
                else:
                    n_bad += 1

                print(f"[{n:6d}] dc={d_coarse:5d}  fine_a={fine_a:3d}  fine_b={fine_b:3d}  "
                      f"va={valid_a} vb={valid_b}  interval={interval:10.3f} ns{flags}")

                w.writerow([n, d_coarse, fine_a, fine_b, valid_a, valid_b,
                            int(usable), f"{interval:.3f}"])
                f.flush()

        except KeyboardInterrupt:
            pass

    # ------------------------------------------------------------------------
    print()
    print("=" * 88)
    print(f"  {n} frames -> {a.out}")
    if n == 0:
        return
    print(f"  usable : {len(good_intervals):6d}  ({100*len(good_intervals)/n:5.1f} %)")
    print(f"  reject : {n_bad:6d}  ({100*n_bad/n:5.1f} %)   (invalid code, railed chain, or no STOP)")

    for name, h in (("A (START)", hist_a), ("B (STOP)", hist_b)):
        if not h:
            continue
        tot = sum(h.values())
        ev  = sum(c for k, c in h.items() if k % 2 == 0)
        print()
        print(f"  --- chain {name} fine histogram ({tot} samples) ---")
        print(f"      distinct codes : {len(h)} / 254")
        print(f"      EVEN codes     : {100*ev/tot:5.1f} %   (50 % if bins were uniform)")
        print(f"      zero-width bins: {254 - len(h)} codes never seen "
              f"-> taps flipping simultaneously")

    if len(good_intervals) >= 2:
        mean = statistics.mean(good_intervals)
        sd   = statistics.pstdev(good_intervals)
        print()
        print("  --- interval statistics ---")
        print(f"      mean  = {mean:10.3f} ns")
        print(f"      sigma = {sd*1000:10.1f} ps      <-- single-shot precision of the PAIR")
        print(f"      (if both chains share one source, sigma_single = sigma/sqrt(2) "
              f"= {sd*1000/1.4142:.1f} ps)")
        if a.expect is not None:
            print(f"      expected  = {a.expect:10.3f} ns")
            print(f"      residual  = {(mean - a.expect)*1000:+10.1f} ps   "
                  f"<-- this is the fixed offset K; it cancels if you take a DIFFERENCE "
                  f"of two cable lengths")
    print("=" * 88)
    print("  Both taus are UNCALIBRATED defaults. Run code-density calibration")
    print("  (>= 1e5 samples, asynchronous source) before quoting any resolution.")
    print("=" * 88)


if __name__ == "__main__":
    main()