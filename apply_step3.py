#!/usr/bin/env python3
"""apply_step3.py -- edits tdc_dual_board.v and read_tdc.py for step 3.
Each edit must match exactly once; aborts before writing if anything is off."""
import sys

def patch(path, edits, post=None):
    s = open(path, encoding='utf-8').read()
    for i, (old, new) in enumerate(edits, 1):
        n = s.count(old)
        if n != 1:
            already = new and s.count(new) >= 1
            sys.exit(f"{path}: edit {i} matched {n} times"
                     + (" (looks ALREADY APPLIED)" if already else "") + " -- aborting, nothing written")
        s = s.replace(old, new)
    if post:
        s = post(s)
    open(path, 'w', encoding='utf-8', newline='\n').write(s)
    print(f"{path}: {len(edits)} edits applied")

# ============================ tdc_dual_board.v ===============================
B = []
B.append(("""//   UART FRAME - 8 bytes per measurement, 8N1 @ 2 Mbaud
//     byte0 : 0xA5                                       sync header""",
"""//   UART FRAME - 9 bytes per measurement, 8N1 @ 2 Mbaud
//     byte0 : 0xA6                                       sync header"""))
B.append(("""//     byte7 : seq[7:0]                                   per-measurement counter
//
//   phase_idx""","""//     byte7 : seq[7:0]                                   per-measurement counter
//     byte8 : CRC-8 (poly 0x07, init 0x00) over bytes 0..7
//
//   CRC: a lost UART byte used to be caught only by the host's stride check,
//   and one corrupt frame (phase = -859) still reached sweep_v2.csv. For code
//   density a corrupt frame lands in a random bin, so every frame is now
//   checked. Header moved 0xA5 -> 0xA6 so an 8-byte host fails loudly.
//
//   phase_idx"""))
B.append(("""//   Header is 0xA5, not the old 0xAA, so a stale host fails loudly rather than
//   silently mis-decoding (which is what the 5-vs-6 byte mismatch did).
//
//   Frame time = 8 bytes x 10 bits / 2e6 = 40 us -> 25 kframe/s.""",
"""//   Frame time = 9 bytes x 10 bits / 2e6 = 45 us -> 22.2 kframe/s."""))
B.append(("""//   Frame time = 6 bytes x 10 bits / 115200 = 521 us -> max ~1.9 kHz.
//   For faster calibration runs raise the baud (CLKS_PER_BIT = 200e6 / baud;
//   921600 -> 217).
//
""", ""))
B.append(("""//     EVENT_SRC = 1 : external pins    (function generator / laser / cables)
""","""//     EVENT_SRC = 1 : external pins    (function generator / laser / cables)
//     EVENT_SRC = 2 : clk_cal + DPS sweep (phase-referenced calibration)
//     EVENT_SRC = 3 : on-chip ring oscillator, both chains (code density).
//                     Needs ro.xdc. phase field is 0 in these frames.
"""))
B.append(("""    parameter integer EVENT_SRC    = 2,      // 0 = buttons, 1 = external pins""",
          """    parameter integer EVENT_SRC    = 2,      // 0 btn, 1 ext, 2 DPS cal, 3 ring osc"""))
B.append(("""    parameter integer SYNC_TAP         = 30   // 0 = old raw-event sync""",
"""    parameter integer SYNC_TAP         = 30,  // 0 = old raw-event sync
    // ---- ring-oscillator hit source (EVENT_SRC = 3 only) --------------------
    parameter integer RO_STAGES        = 7,   // odd. Build 7 AND 11 to cross-check
    parameter integer RO_DIV_BITS      = 10   // event period ~ 2^10 RO periods"""))
B.append(("""    end else begin : g_cal            // EVENT_SRC == 2: calibration
        assign event_a_src = 1'b0;    // unused: core drives both chains from clk_cal
        assign event_b_src = 1'b0;
    end""","""    end else if (EVENT_SRC == 2) begin : g_cal   // DPS calibration
        assign event_a_src = 1'b0;    // unused: core drives both chains from clk_cal
        assign event_b_src = 1'b0;
    end else begin : g_ro                        // EVENT_SRC == 3: code density
        // One RO edge drives BOTH chains: two independent histograms per run,
        // plus the A-B spread at d ~ 0 for free. Constant-folded like the
        // other sources -- the RO only exists in this build.
        wire ro_event;
        ring_osc_event #(.STAGES(RO_STAGES), .DIV_BITS(RO_DIV_BITS)) ro_inst (
            .enable (~rst), .event_out (ro_event));
        assign event_a_src = ro_event;
        assign event_b_src = ro_event;
    end"""))
B.append(("""    // UART framing -- 8 bytes, shifted out of a snapshot register.
    //
    //   byte0 : 0xA5                                       sync header""",
"""    // UART framing -- 9 bytes, shifted out of a snapshot register.
    //
    //   byte0 : 0xA6                                       sync header"""))
B.append(("""    //   byte7 : seq[7:0]
    //
    // Header is 0xA5, NOT the old 0xAA. The previous 6-byte frame was parsed by
    // a host that expected 5 bytes; it stayed in sync by luck and silently
    // mis-decoded every field. Changing the header makes an out-of-date host
    // fail loudly instead of quietly reporting wrong numbers.""",
"""    //   byte7 : seq[7:0]
    //   byte8 : CRC-8/0x07 over bytes 0..7 (computed at load, travels with sr)"""))
B.append(("""    reg [63:0] sr;
    reg [3:0]  nleft;

    wire [63:0] frame_w = { 8'hA5,""","""    reg [71:0] sr;
    reg [3:0]  nleft;

    // CRC-8, poly x^8+x^2+x+1 (0x07), init 0, MSB-first over bytes 0..7.
    function [7:0] crc8_64;
        input [63:0] d;
        integer i;
        reg [7:0] c;
        begin
            c = 8'h00;
            for (i = 63; i >= 0; i = i - 1)
                c = {c[6:0], 1'b0} ^ ((c[7] ^ d[i]) ? 8'h07 : 8'h00);
            crc8_64 = c;
        end
    endfunction

    wire [63:0] frame_w = { 8'hA6,"""))
B.append(("""                            seq_l };
""","""                            seq_l };
    wire [71:0] frame_c = { frame_w, crc8_64(frame_w) };
"""))
B.append(("""            frame_done <= 1'b0; sr <= 64'd0; nleft <= 4'd0;""",
          """            frame_done <= 1'b0; sr <= 72'd0; nleft <= 4'd0;"""))
B.append(("""                    uart_byte <= frame_w[63:56];   // header out now
                    sr        <= {frame_w[55:0], 8'h00};
                    uart_send <= 1'b1;
                    nleft     <= 4'd7;             // 7 payload bytes still to go""",
"""                    uart_byte <= frame_c[71:64];   // header out now
                    sr        <= {frame_c[63:0], 8'h00};
                    uart_send <= 1'b1;
                    nleft     <= 4'd8;             // 7 payload + CRC still to go"""))
B.append(("""                uart_byte <= sr[63:56];
                sr        <= {sr[55:0], 8'h00};""","""                uart_byte <= sr[71:64];
                sr        <= {sr[63:0], 8'h00};"""))
B.append(("""    wire sweep_on   = (AUTO_SWEEP != 0) && sw_autorearm;""",
"""    // Only a DPS build has a phase to sweep. Previously the FSM also ran in
    // button/external builds: it stalled re-arm for every SETTLE window and set
    // sw_mismatch (led[7]) because ps_phase_idx is tied to 0 there.
    wire sweep_on   = (AUTO_SWEEP != 0) && (CAL_EVENT_MODE != 0) && sw_autorearm;"""))
B.append(("""    reg [2:0] rearm_cnt;
    always @(posedge clk200) begin
        if (rst200)                          rearm_cnt <= 3'd0;
        else if (sw_autorearm && frame_done && !sweep_hold) rearm_cnt <= 3'd4;
        else if (rearm_cnt != 3'd0)          rearm_cnt <= rearm_cnt - 1'b1;
    end""","""    //
    // DEADLOCK FIX: a frame_done arriving while sweep_hold is high used to be
    // dropped. The channels then sat 'done' forever: no measurement, no frame,
    // no re-arm. It only worked because SETTLE (10 us) < frame time; a faster
    // baud or a longer settle stalled the sweep after the first step.
    // Now the request is remembered and fired when the hold lifts.
    reg [2:0] rearm_cnt;
    reg       rearm_pend;
    wire      rearm_req = sw_autorearm && (frame_done || rearm_pend);
    always @(posedge clk200) begin
        if (rst200) begin
            rearm_cnt  <= 3'd0;
            rearm_pend <= 1'b0;
        end else if (rearm_req && !sweep_hold) begin
            rearm_cnt  <= 3'd4;
            rearm_pend <= 1'b0;
        end else begin
            if (rearm_req)               rearm_pend <= 1'b1;   // held off: remember
            if (!sw_autorearm)           rearm_pend <= 1'b0;
            if (rearm_cnt != 3'd0)       rearm_cnt  <= rearm_cnt - 1'b1;
        end
    end"""))

# ============================== read_tdc.py ==================================
R = []
R.append(('''read_tdc.py -- host reader for the two-channel interval TDC, 8-byte frame.

FRAME (8 bytes, 8N1 @ 2 Mbaud)

    byte0 : 0xA5                                       sync header''','''read_tdc.py -- host reader for the two-channel interval TDC, 9-byte frame.

FRAME (9 bytes, 8N1 @ 2 Mbaud)

    byte0 : 0xA6                                       sync header'''))
R.append(('''    byte7 : seq[7:0]

WHAT CHANGED''','''    byte7 : seq[7:0]
    byte8 : CRC-8, poly 0x07, init 0x00, over bytes 0..7

CRC (0xA5 -> 0xA6)
------------------
The stride check alone let one corrupt frame (phase = -859) into sweep_v2.csv.
Every frame is now CRC-checked; a failure drops the frame, is counted, and
forces a resync. Header changed so an 8-byte host cannot mis-parse silently.

WHAT CHANGED (history)'''))
R.append(('''FRAME_LEN = 8
HEADER = 0xA5''','''FRAME_LEN = 9
HEADER = 0xA6'''))
R.append(('''def unpack(f):
    """Decode one 8-byte frame. Mirrors frame_w in tdc_dual_board.v."""
    if len(f) != FRAME_LEN or f[0] != HEADER:
        raise ValueError("not a frame")''','''def crc8(data):
    """CRC-8, poly 0x07, init 0, MSB-first. Mirrors crc8_64() in tdc_dual_board.v."""
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def pack(fine_a, fine_b, d_coarse, valid_a, valid_b, phase, seq):
    """Inverse of unpack(); mirrors frame_w + CRC. Used by the self-test."""
    ph = phase & 0xFFF
    b = bytes([HEADER, fine_a & 0xFF, fine_b & 0xFF, d_coarse & 0xFF,
               ((d_coarse >> 8) << 2) | ((fine_a >> 8) << 1) | (fine_b >> 8),
               ph & 0xFF, (valid_b << 5) | (valid_a << 4) | (ph >> 8), seq & 0xFF])
    return b + bytes([crc8(b)])


def unpack(f):
    """Decode one 9-byte frame. Mirrors frame_w in tdc_dual_board.v."""
    if len(f) != FRAME_LEN or f[0] != HEADER:
        raise ValueError("not a frame")
    if crc8(f[:8]) != f[8]:
        raise ValueError("crc")'''))
R.append(('''class Framer:
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
        return out''','''class Framer:
    """
    Byte-stream -> frames. A frame is accepted only if header AND CRC match.

    Lock needs LOCK_FRAMES consecutive CRC-good frames at the 9-byte stride.
    Any header or CRC failure drops ONE byte and restarts the search, so a
    misaligned stream cannot be parsed as data. crc_errors counts CRC failures
    seen while locked -- i.e. real corruption, not sync hunting.
    """
    LOCK_FRAMES = 3

    def __init__(self):
        self.buf = bytearray()
        self.locked = False
        self.streak = 0
        self.resyncs = 0
        self.crc_errors = 0

    def _lose_lock(self):
        if self.locked:
            self.resyncs += 1
        self.locked = False
        self.streak = 0

    def feed(self, chunk):
        self.buf.extend(chunk)
        out = []
        while len(self.buf) >= FRAME_LEN:
            if self.buf[0] != HEADER:
                del self.buf[0]
                self._lose_lock()
                continue
            frame = bytes(self.buf[:FRAME_LEN])
            if crc8(frame[:8]) != frame[8]:
                if self.locked:
                    self.crc_errors += 1
                del self.buf[0]
                self._lose_lock()
                continue
            del self.buf[:FRAME_LEN]
            if self.locked:
                out.append(unpack(frame))
            else:
                self.streak += 1
                if self.streak >= self.LOCK_FRAMES:
                    self.locked = True
                    out.append(unpack(frame))
        return out'''))
R.append(('''    return 0 if bad == 0 else 1''','''    print("SELFTEST", "PASS" if bad == 0 else f"FAIL ({bad})")
    return 0 if bad == 0 else 1'''))
R.append(('''p = argparse.ArgumentParser(description="Two-channel TDC reader, 8-byte frame")''',
          '''p = argparse.ArgumentParser(description="Two-channel TDC reader, 9-byte CRC frame")'''))
R.append(('''print(f"Listening on {a.port} @ {a.baud} 8N1, 8-byte frames. Ctrl-C to stop.")''',
          '''print(f"Listening on {a.port} @ {a.baud} 8N1, 9-byte CRC frames. Ctrl-C to stop.")'''))
R.append(('''    if fr.resyncs:
        print(f"  WARNING: {fr.resyncs} resync events -- check baud and cabling")''',
'''    if fr.resyncs or fr.crc_errors:
        print(f"  WARNING: {fr.resyncs} resyncs, {fr.crc_errors} CRC errors "
              f"-- check baud and cabling. Corrupt frames were dropped, not parsed.")'''))

NEW_SELFTEST = '''# Payload vectors (bytes 1..7) were produced by the RTL frame_w expression; the
# payload layout is unchanged, so they still pin the field packing.
VECTORS = [
    # fine_a fine_b d_coarse va vb phase seq   payload bytes 1..7
    ((0,     0,     0,       0, 0, 0,     0),   "00 00 00 00 00 00 00"),
    ((1,     2,     3,       1, 1, 1,     1),   "01 02 03 00 01 30 01"),
    ((351,   352,   16383,   1, 0, 279,   200), "5f 60 ff ff 17 11 c8"),
    ((300,   260,   1234,    0, 1, -1,    255), "2c 04 d2 13 ff 2f ff"),
    ((256,   255,   8192,    1, 1, -280,  77),  "00 ff 00 82 e8 3e 4d"),
    ((128,   64,    0,       1, 1, -2048, 1),   "80 40 00 00 00 38 01"),
    ((511,   511,   16383,   1, 1, 2047,  254), "ff ff ff ff ff 37 fe"),
]

# Whole frames captured from the RTL in simulation (tb_ro_cd.v): Python crc8()
# must agree with the Verilog crc8_64() byte for byte.
RTL_FRAMES = [
    "a6 c2 c2 00 00 00 30 00 fc",
    "a6 0c 0c 00 03 00 30 01 0b",
    "a6 e8 e8 00 00 00 30 02 26",
]


def selftest():
    bad = 0
    for (fa, fb, dc, va, vb, ph, sq), hexs in VECTORS:
        payload = bytes(int(x, 16) for x in hexs.split())
        f = bytes([HEADER]) + payload
        f = f + bytes([crc8(f)])
        r = unpack(f)
        want = dict(fine_a=fa, fine_b=fb, d_coarse=dc,
                    valid_a=va, valid_b=vb, phase=ph, seq=sq)
        if r != want or pack(fa, fb, dc, va, vb, ph, sq) != f:
            bad += 1
            print(f"  MISMATCH {hexs}: got {r}")
    print(f"pack/unpack : {len(VECTORS) - bad}/{len(VECTORS)} RTL payload vectors")

    nbad = 0
    for h in RTL_FRAMES:
        f = bytes(int(x, 16) for x in h.split())
        if crc8(f[:8]) != f[8]:
            nbad += 1
            print(f"  CRC MISMATCH vs RTL: {h}")
    bad += nbad
    print(f"crc8 vs RTL : {len(RTL_FRAMES) - nbad}/{len(RTL_FRAMES)} frames agree")

    good = b"".join(pack(*v) for v, _ in VECTORS) * 3
    n_frames = len(VECTORS) * 3
    fr = Framer()
    n_ok = len(fr.feed(good))
    exp = n_frames - Framer.LOCK_FRAMES + 1
    print(f"framer      : aligned stream -> {n_ok} frames (expect {exp})")
    bad += n_ok != exp

    corrupt = bytearray(good)
    corrupt[9 * 10 + 4] ^= 0x10
    fr3 = Framer()
    got = fr3.feed(bytes(corrupt))
    print(f"framer      : 1 corrupt frame -> crc_errors={fr3.crc_errors} (expect 1), "
          f"frames {len(got)}")
    bad += fr3.crc_errors != 1

    dropped = good[:9 * 10 + 3] + good[9 * 10 + 4:]
    fr4 = Framer()
    got4 = fr4.feed(dropped)
    ok4 = all(r in [unpack(pack(*v)) for v, _ in VECTORS] for r in got4)
    print(f"framer      : 1 dropped byte -> only legal frames emitted = {ok4}")
    bad += not ok4

'''

def replace_selftest(s):
    a = s.index('VECTORS = [')
    b = s.index('    # sequence-gap accounting must survive the 8-bit wrap')
    return s[:a] + NEW_SELFTEST + s[b:]

# Check both before writing either
for path in ('tdc_dual_board.v', 'read_tdc.py'):
    open(path, encoding='utf-8').close()
patch('tdc_dual_board.v', B)
patch('read_tdc.py', R, post=replace_selftest)