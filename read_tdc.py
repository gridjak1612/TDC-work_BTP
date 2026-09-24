#!/usr/bin/env python3
"""
read_tdc.py -- host reader for the two-channel interval TDC, 8-byte frame.

FRAME (8 bytes, 8N1 @ 2 Mbaud)

    byte0 : 0xA5                                       sync header
    byte1 : fine_a[7:0]
    byte2 : fine_b[7:0]
    byte3 : d_coarse[7:0]
    byte4 : {d_coarse[13:8], fine_a[8], fine_b[8]}
    byte5 : phase_idx[7:0]
    byte6 : {2'b00, valid_b, valid_a, phase_idx[11:8]}
    byte7 : seq[7:0]

WHAT CHANGED, AND WHY IT MATTERS
--------------------------------
The previous host parsed a FIVE byte frame while the RTL sent SIX. It stayed in
sync by luck -- byte 5 was never 0xAA, so the resync path silently discarded it
-- and therefore looked like it worked. It did not:

  * fine codes above 255 aliased down by 256. The chain is 352 taps, so the top
    27 % of the range was being folded on top of the bottom.
  * the reported valid_a / valid_b were actually fine_a[8] / fine_b[8]. The real
    thermometer-validity flags were thrown away.

Every histogram taken with that script is unusable. The header was changed from
0xAA to 0xA5 specifically so that an old copy of this file fails loudly instead
of repeating the trick.

phase_idx is SIGNED 12-bit two's complement and is sign-extended below. Reading
it unsigned puts every negative step near +4000 instead of just below zero,
which folds the low half of a sweep onto the high half -- producing a
calibration curve that looks entirely plausible and is wrong.

seq increments on every measurement the FPGA completes, framed or not. A gap
means a record was produced and dropped. That is a different thing from "no
measurement happened", and for code-density work it is the difference between a
genuinely empty bin and a lost sample.

Usage
    python read_tdc.py COM19
    python read_tdc.py COM19 --samples 50000 --out sweep.csv
    python read_tdc.py /dev/ttyUSB1 --baud 2000000 --seconds 60
    python read_tdc.py --selftest          # no hardware needed
"""

import sys
import csv
import math
import time
import argparse
from collections import Counter, defaultdict

FRAME_LEN = 8
HEADER = 0xA5
TDL_TAPS = 352          # 88 CARRY4 x 4. fine ranges 0..352, NOT 0..255.
T_CLK_NS = 5.000        # 200 MHz sampler
PHASE_STEP_PS = 1000.0 / 56.0    # MMCM fine step = T_VCO/56, VCO = 1000 MHz


# ---------------------------------------------------------------------------
# Frame decode
# ---------------------------------------------------------------------------

def load_lut(path):
    """
    code -> arrival time in ps, from analyze_sweep.py.

    Missing codes (zero-width bins) are interpolated: a sample can still carry
    such a code because of jitter, and dropping those samples biases the result
    toward the well-populated bins. Codes whose phase distribution is not
    concentrated are dropped -- their t_ps is an average over the whole period
    and therefore meaningless.
    """
    raw = {}
    with open(path, newline='') as fh:
        for r in csv.DictReader(fh):
            if float(r.get('concentration', 1.0)) < 0.5:
                continue
            raw[int(r['code'])] = float(r['t_ps'])
    if not raw:
        return {}
    ks = sorted(raw)
    out = dict(raw)
    for c in range(ks[0], ks[-1] + 1):
        if c in out:
            continue
        lo = max(k for k in ks if k < c)
        hi = min(k for k in ks if k > c)
        a, b = raw[lo], raw[hi]
        d = b - a
        if d > 2500.0:
            d -= 5000.0
        elif d < -2500.0:
            d += 5000.0
        out[c] = (a + d * (c - lo) / (hi - lo)) % 5000.0
    return out


def interval_ps(r, lut_a, lut_b, t_clk_ns=T_CLK_NS):
    """
    Calibrated STOP - START interval in ps.

        interval = d_coarse * T_clk  +  (t_a - t_b)

    t_x comes from the LUT, which already absorbs the non-uniform bin widths.
    Returns None when either code is outside the calibrated range -- better a
    gap in the output than a number that looks fine and is not.
    """
    ta = lut_a.get(r['fine_a'])
    tb = lut_b.get(r['fine_b'])
    if ta is None or tb is None:
        return None
    return r['d_coarse'] * t_clk_ns * 1000.0 + (ta - tb)


def unpack(f):
    """Decode one 8-byte frame. Mirrors frame_w in tdc_dual_board.v."""
    if len(f) != FRAME_LEN or f[0] != HEADER:
        raise ValueError("not a frame")
    fine_a = (((f[4] >> 1) & 1) << 8) | f[1]
    fine_b = ((f[4] & 1) << 8) | f[2]
    d_coarse = ((f[4] >> 2) << 8) | f[3]
    phase_u = ((f[6] & 0x0F) << 8) | f[5]
    phase = phase_u - 4096 if (phase_u & 0x800) else phase_u   # sign-extend 12b
    valid_a = (f[6] >> 4) & 1
    valid_b = (f[6] >> 5) & 1
    seq = f[7]
    return dict(fine_a=fine_a, fine_b=fine_b, d_coarse=d_coarse,
                phase=phase, valid_a=valid_a, valid_b=valid_b, seq=seq)


class Framer:
    """
    Byte-stream -> frames, with stride-verified sync.

    A payload byte can legitimately equal 0xA5, so "first byte is 0xA5" is not
    proof of alignment. Sync is only declared once headers land at the correct
    8-byte stride LOCK_FRAMES times running, and is dropped the moment the
    stride breaks. Accepting on a single header match is how a misaligned
    stream gets parsed as valid data.
    """
    LOCK_FRAMES = 3

    def __init__(self):
        self.buf = bytearray()
        self.locked = False
        self.streak = 0
        self.resyncs = 0

    def feed(self, chunk):
        self.buf.extend(chunk)
        out = []
        while len(self.buf) >= FRAME_LEN:
            if self.buf[0] != HEADER:
                del self.buf[0]
                if self.locked:
                    self.locked = False
                    self.streak = 0
                    self.resyncs += 1
                continue
            frame = bytes(self.buf[:FRAME_LEN])
            del self.buf[:FRAME_LEN]
            if self.locked:
                out.append(unpack(frame))
            else:
                self.streak += 1
                if self.streak >= self.LOCK_FRAMES:
                    self.locked = True
                    out.append(unpack(frame))
        return out


# ---------------------------------------------------------------------------
# Accounting
# ---------------------------------------------------------------------------

class Stats:
    def __init__(self):
        self.n = 0
        self.dropped = 0
        self.prev_seq = None
        self.hist = {'a': defaultdict(Counter), 'b': defaultdict(Counter)}
        self.invalid_a = 0
        self.invalid_b = 0
        self.railed = 0

    def add(self, r):
        self.n += 1
        if self.prev_seq is not None:
            gap = (r['seq'] - self.prev_seq - 1) & 0xFF
            self.dropped += gap
        self.prev_seq = r['seq']

        if not r['valid_a']:
            self.invalid_a += 1
        if not r['valid_b']:
            self.invalid_b += 1
        if r['fine_a'] in (0, TDL_TAPS) or r['fine_b'] in (0, TDL_TAPS):
            self.railed += 1

        # Only thermometer-legal codes go into the calibration histograms.
        if r['valid_a']:
            self.hist['a'][r['phase']][r['fine_a']] += 1
        if r['valid_b']:
            self.hist['b'][r['phase']][r['fine_b']] += 1

    def report(self):
        print()
        print("=" * 78)
        print(f"  frames received : {self.n}")
        produced = self.n + self.dropped
        if produced:
            print(f"  frames dropped  : {self.dropped}  "
                  f"({100.0 * self.dropped / produced:.2f} % of measurements produced)")
        print(f"  invalid code A  : {self.invalid_a}   invalid code B : {self.invalid_b}")
        print(f"  railed (0 or {TDL_TAPS}) : {self.railed}")

        for ch in ('a', 'b'):
            phases = sorted(self.hist[ch])
            if not phases:
                continue
            allcodes = Counter()
            for p in phases:
                allcodes.update(self.hist[ch][p])
            print()
            print(f"  --- chain {ch.upper()} ---")
            print(f"      phase steps seen : {len(phases)}  "
                  f"[{phases[0]} .. {phases[-1]}]")
            print(f"      distinct codes   : {len(allcodes)} / {TDL_TAPS + 1}")
            print(f"      never seen       : {TDL_TAPS + 1 - len(allcodes)} codes")
            # codes 1 and 2 were structurally impossible before the
            # bubble_correction low-boundary fix -- their presence is the
            # hardware proof that the fix is in this bitstream.
            print(f"      code 1 count     : {allcodes.get(1, 0)}"
                  f"      code 2 count : {allcodes.get(2, 0)}")
            if len(phases) > 1:
                print(f"      mean code vs phase (first/last step): "
                      f"{_mean(self.hist[ch][phases[0]]):.1f} -> "
                      f"{_mean(self.hist[ch][phases[-1]]):.1f}")
        print("=" * 78)
        print("  Codes are RAW tap counts. No LUT applied. Do not quote a")
        print("  resolution from these numbers.")
        print("=" * 78)


def _mean(counter):
    tot = sum(counter.values())
    return sum(k * v for k, v in counter.items()) / tot if tot else float('nan')


# ---------------------------------------------------------------------------
# Self-test -- runs without hardware, checks against RTL-generated vectors
# ---------------------------------------------------------------------------

VECTORS = [
    # fine_a fine_b d_coarse va vb phase seq   bytes
    ((0,     0,     0,       0, 0, 0,     0),   "a5 00 00 00 00 00 00 00"),
    ((1,     2,     3,       1, 1, 1,     1),   "a5 01 02 03 00 01 30 01"),
    ((351,   352,   16383,   1, 0, 279,   200), "a5 5f 60 ff ff 17 11 c8"),
    ((300,   260,   1234,    0, 1, -1,    255), "a5 2c 04 d2 13 ff 2f ff"),
    ((256,   255,   8192,    1, 1, -280,  77),  "a5 00 ff 00 82 e8 3e 4d"),
    ((128,   64,    0,       1, 1, -2048, 1),   "a5 80 40 00 00 00 38 01"),
    ((511,   511,   16383,   1, 1, 2047,  254), "a5 ff ff ff ff ff 37 fe"),
]


def selftest():
    """Decode byte vectors produced by the actual RTL frame_w expression."""
    bad = 0
    for (fa, fb, dc, va, vb, ph, sq), hexs in VECTORS:
        f = bytes(int(x, 16) for x in hexs.split())
        r = unpack(f)
        want = dict(fine_a=fa, fine_b=fb, d_coarse=dc,
                    valid_a=va, valid_b=vb, phase=ph, seq=sq)
        if r != want:
            bad += 1
            print(f"  MISMATCH {hexs}")
            for k in want:
                if r[k] != want[k]:
                    print(f"      {k}: got {r[k]}  want {want[k]}")
    print(f"unpack(): {len(VECTORS) - bad}/{len(VECTORS)} RTL vectors decoded correctly")

    # framer must reject a stream offset by one byte rather than locking to it
    good = b"".join(bytes(int(x, 16) for x in h.split()) for _, h in VECTORS) * 3
    fr = Framer()
    n_ok = len(fr.feed(good))
    fr2 = Framer()
    n_shift = len(fr2.feed(b"\x00" + good[:-1]))
    print(f"framer  : aligned stream -> {n_ok} frames "
          f"(expect {len(VECTORS) * 3 - Framer.LOCK_FRAMES + 1})")
    print(f"framer  : resyncs on offset stream = {fr2.resyncs > 0 or n_shift < n_ok}")

    # sequence-gap accounting must survive the 8-bit wrap
    st = Stats()
    for s in (253, 254, 255, 0, 1):
        st.add(dict(fine_a=10, fine_b=10, d_coarse=0, phase=0,
                    valid_a=1, valid_b=1, seq=s))
    print(f"seq wrap: dropped = {st.dropped} (expect 0)")
    st2 = Stats()
    for s in (10, 13):
        st2.add(dict(fine_a=10, fine_b=10, d_coarse=0, phase=0,
                     valid_a=1, valid_b=1, seq=s))
    print(f"seq gap : dropped = {st2.dropped} (expect 2)")
    return 0 if bad == 0 else 1


# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="Two-channel TDC reader, 8-byte frame")
    p.add_argument("port", nargs="?")
    p.add_argument("--baud", type=int, default=2000000)
    p.add_argument("--out", default="tdc_log.csv")
    p.add_argument("--samples", type=int, default=None, help="stop after N frames")
    p.add_argument("--seconds", type=float, default=None, help="stop after N seconds")
    p.add_argument("--quiet", action="store_true", help="do not print every frame")
    p.add_argument("--selftest", action="store_true")
    p.add_argument("--lut", default=None,
                   help="LUT prefix from analyze_sweep.py; loads <p>_a.csv and "
                        "<p>_b.csv and adds a calibrated interval_ps column")
    a = p.parse_args()

    if a.selftest:
        sys.exit(selftest())
    if not a.port:
        p.error("port is required (or use --selftest)")

    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed.  pip install pyserial")

    lut_a = lut_b = None
    if a.lut:
        lut_a = load_lut(f"{a.lut}_a.csv")
        lut_b = load_lut(f"{a.lut}_b.csv")
        print(f"LUT loaded: chain A {len(lut_a)} codes, chain B {len(lut_b)} codes")
        print("Output carries a calibrated interval_ps column. Raw codes are kept")
        print("alongside it -- never discard them, the LUT is only valid for the")
        print("bitstream it was measured on and you will want to recheck.")

    ser = serial.Serial(a.port, a.baud, timeout=0.1)
    print(f"Listening on {a.port} @ {a.baud} 8N1, 8-byte frames. Ctrl-C to stop.")
    print(f"phase step = {PHASE_STEP_PS:.3f} ps, "
          f"{round(T_CLK_NS * 1000 / PHASE_STEP_PS)} steps per {T_CLK_NS} ns period")
    print("-" * 78)

    fr = Framer()
    st = Stats()
    t0 = time.time()

    with open(a.out, "w", newline="") as fh:
        w = csv.writer(fh)
        cols = ["seq", "phase", "d_coarse", "fine_a", "fine_b",
                "valid_a", "valid_b"]
        if lut_a:
            cols.append("interval_ps")
        w.writerow(cols)
        ivals = []
        try:
            while True:
                # Batched read. One byte at a time cannot keep up at 2 Mbaud
                # and the OS buffer overflows -- which shows up as dropped
                # frames that look like an FPGA problem but are not.
                chunk = ser.read(max(1, ser.in_waiting))
                for r in fr.feed(chunk):
                    st.add(r)
                    row = [r['seq'], r['phase'], r['d_coarse'],
                           r['fine_a'], r['fine_b'],
                           r['valid_a'], r['valid_b']]
                    if lut_a:
                        iv = interval_ps(r, lut_a, lut_b)
                        row.append("" if iv is None else f"{iv:.2f}")
                        if iv is not None and r['valid_a'] and r['valid_b']:
                            ivals.append(iv)
                    w.writerow(row)
                    if not a.quiet and st.n % 200 == 1:
                        print(f"[{st.n:7d}] ph={r['phase']:+5d}  "
                              f"fa={r['fine_a']:4d} fb={r['fine_b']:4d}  "
                              f"dc={r['d_coarse']:5d}  "
                              f"va={r['valid_a']} vb={r['valid_b']}  "
                              f"drop={st.dropped}")
                if a.samples and st.n >= a.samples:
                    break
                if a.seconds and (time.time() - t0) >= a.seconds:
                    break
        except KeyboardInterrupt:
            pass

    if lut_a and ivals:
        m = sum(ivals) / len(ivals)
        sd = math.sqrt(sum((x - m) ** 2 for x in ivals) / len(ivals))
        print()
        print(f"  calibrated interval : mean {m:.2f} ps   sigma {sd:.2f} ps"
              f"   (n={len(ivals)})")
        print(f"  implied single-shot : {sd / math.sqrt(2):.2f} ps"
              f"   (two similar independent chains)")

    print(f"\n  {st.n} frames -> {a.out}   ({time.time() - t0:.1f} s)")
    if fr.resyncs:
        print(f"  WARNING: {fr.resyncs} resync events -- check baud and cabling")
    st.report()


if __name__ == "__main__":
    main()
