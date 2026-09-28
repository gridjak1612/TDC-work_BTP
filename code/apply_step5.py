#!/usr/bin/env python3
"""apply_step5.py -- host side of the foundation step: frame v4, --rail-max, tb_ro_cd 10-byte count."""
import re, sys

def patch(path, marker, edits, post=None):
    s = open(path, encoding='utf-8').read()
    if marker in s:
        print(f"{path}: already applied -- skipped"); return
    for i, (pat, rep, want) in enumerate(edits, 1):
        s, n = re.subn(pat, lambda m, r=rep: r, s, flags=re.S)
        if n != want:
            sys.exit(f"{path}: edit {i} matched {n}x (expected {want}) -- aborting, nothing written")
    if post:
        s = post(s)
    open(path, 'w', encoding='utf-8', newline='\n').write(s)
    print(f"{path}: {len(edits)} edits applied")

NEW_PACK = r'''def pack(fine_a, fine_b, d_coarse, valid_a, valid_b, phase, seq, cfg=0):
    """Inverse of unpack(); mirrors the v4 payload in tdc_dual_board.v."""
    p = (((fine_a & 0x7FF) << 53) | ((fine_b & 0x7FF) << 42) |
         ((d_coarse & 0x3FFF) << 28) | ((phase & 0xFFF) << 16) |
         ((valid_a & 1) << 15) | ((valid_b & 1) << 14) |
         ((seq & 0xFF) << 6) | (cfg & 0x3F))
    b = bytes([HEADER]) + p.to_bytes(8, 'big')
    return b + bytes([crc8(b)])
'''

NEW_UNPACK = r'''SRC_NAMES = {0: 'ext', 1: 'ro7', 2: 'ro11', 3: 'dps'}


def unpack(f):
    """Decode one 10-byte v4 frame. Mirrors the payload in tdc_dual_board.v."""
    if len(f) != FRAME_LEN or f[0] != HEADER:
        raise ValueError("not a frame")
    if crc8(f[:FRAME_LEN - 1]) != f[FRAME_LEN - 1]:
        raise ValueError("crc")
    p = int.from_bytes(f[1:9], 'big')
    phase_u = (p >> 16) & 0xFFF
    cfg = p & 0x3F
    return dict(fine_a=(p >> 53) & 0x7FF, fine_b=(p >> 42) & 0x7FF,
                d_coarse=(p >> 28) & 0x3FFF,
                phase=phase_u - 4096 if phase_u & 0x800 else phase_u,
                valid_a=(p >> 15) & 1, valid_b=(p >> 14) & 1,
                seq=(p >> 6) & 0xFF, cfg=cfg, src=cfg & 3)
'''

NEW_SELFTEST = r'''VECTORS = [
    # fine_a fine_b d_coarse va vb phase  seq  cfg
    (0,    0,    0,     0, 0, 0,     0,   0),
    (1,    2,    3,     1, 1, 1,     1,   1),
    (351,  352,  16383, 1, 0, 279,   200, 2),
    (300,  260,  1234,  0, 1, -1,    255, 3),
    (2047, 1024, 8192,  1, 1, -280,  77,  5),
    (598,  601,  1,     1, 1, -2048, 1,   63),
    (2047, 2047, 16383, 1, 1, 2047,  254, 42),
]


def selftest():
    bad = 0
    for v in VECTORS:
        fa, fb, dc, va, vb, ph, sq, cfg = v
        r = unpack(pack(*v))
        want = dict(fine_a=fa, fine_b=fb, d_coarse=dc, phase=ph, valid_a=va,
                    valid_b=vb, seq=sq, cfg=cfg, src=cfg & 3)
        if r != want:
            bad += 1
            print(f"  MISMATCH {v}: got {r}")
    print(f"pack/unpack : {len(VECTORS) - bad}/{len(VECTORS)} v4 round-trips "
          "(RTL agreement: check_uart_dump.py on simulation output)")

    good = b"".join(pack(*v) for v in VECTORS) * 3
    n_frames = len(VECTORS) * 3
    fr = Framer()
    n_ok = len(fr.feed(good))
    exp = n_frames - Framer.LOCK_FRAMES + 1
    print(f"framer      : aligned stream -> {n_ok} frames (expect {exp})")
    bad += n_ok != exp

    corrupt = bytearray(good)
    corrupt[FRAME_LEN * 10 + 4] ^= 0x10
    fr3 = Framer()
    got = fr3.feed(bytes(corrupt))
    print(f"framer      : 1 corrupt frame -> crc_errors={fr3.crc_errors} (expect 1), "
          f"frames {len(got)}")
    bad += fr3.crc_errors != 1

    dropped = good[:FRAME_LEN * 10 + 3] + good[FRAME_LEN * 10 + 4:]
    fr4 = Framer()
    got4 = fr4.feed(dropped)
    legal = [unpack(pack(*v)) for v in VECTORS]
    ok4 = all(r in legal for r in got4)
    print(f"framer      : 1 dropped byte -> only legal frames emitted = {ok4}")
    bad += not ok4

'''

DOC_V4 = '''FRAME v4 (10 bytes, 8N1 @ 2 Mbaud)

    byte0    : 0xA7 header
    byte1..8 : 64-bit payload, MSB first:
               fine_a[10:0] fine_b[10:0] d_coarse[13:0] phase[11:0]
               valid_a valid_b seq[7:0] cfg[5:0]
               cfg = {encoder_id[2:0], dual_snap, source[1:0]}
               source: 0=ext 1=RO7 2=RO11 3=DPS
    byte9    : CRC-8 (poly 0x07, init 0) over bytes 0..8

PREVIOUS v3 LAYOUT (9 bytes, header 0xA6), for reference'''

def replace_selftest(s):
    a = s.index('# Payload vectors (bytes 1..7)')
    b = s.index('    # sequence-gap accounting must survive the 8-bit wrap')
    return s[:a] + NEW_SELFTEST + s[b:]

patch('read_tdc.py', 'HEADER = 0xA7', [
    (r"FRAME_LEN = 9\nHEADER = 0xA6", "FRAME_LEN = 10\nHEADER = 0xA7", 1),
    (r"def pack\(.*?return b \+ bytes\(\[crc8\(b\)\]\)\n", NEW_PACK, 1),
    (r"def unpack\(f\):.*?seq=seq\)\n", NEW_UNPACK, 1),
    (r"if crc8\(frame\[:8\]\) != frame\[8\]:",
     "if crc8(frame[:FRAME_LEN - 1]) != frame[FRAME_LEN - 1]:", 1),
    (r'cols = \["seq", "phase", "d_coarse", "fine_a", "fine_b",\s*"valid_a", "valid_b"\]',
     'cols = ["seq", "phase", "d_coarse", "fine_a", "fine_b",\n'
     '                "valid_a", "valid_b", "cfg", "src"]', 1),
    (r"row = \[r\['seq'\], r\['phase'\], r\['d_coarse'\],\s*r\['fine_a'\], r\['fine_b'\],\s*"
     r"r\['valid_a'\], r\['valid_b'\]\]",
     "row = [r['seq'], r['phase'], r['d_coarse'],\n"
     "                           r['fine_a'], r['fine_b'],\n"
     "                           r['valid_a'], r['valid_b'], r['cfg'], r['src']]", 1),
    (r"FRAME \(9 bytes, 8N1 @ 2 Mbaud\)", DOC_V4, 1),
    (r"9-byte CRC frame", "v4 10-byte CRC frame", 2),
], post=replace_selftest)

patch('code_density.py', 'rail_max', [
    (r"seq=int\(r\['seq'\]\)\)\)",
     "seq=int(r['seq']),\n"
     "                             src=int(r['src']) if r.get('src') not in (None, '') else -1))", 1),
    (r'ap\.add_argument\("--selftest", action="store_true"\)',
     'ap.add_argument("--selftest", action="store_true")\n'
     '    ap.add_argument("--rail-max", type=int, default=352,\n'
     '                    help="rail code meaning chain full (352 for the 88-CARRY4 chain)")', 1),
    (r"a = ap\.parse_args\(\)",
     "a = ap.parse_args()\n    global TDL_TAPS\n    TDL_TAPS = a.rail_max", 1),
    (r'print\(f"\{len\(rows\)\} frames, sequence gaps \(dropped\): \{gaps\}"\)',
     'print(f"{len(rows)} frames, sequence gaps (dropped): {gaps}")\n'
     '    srcs = Counter(r.get("src", -1) for r in rows)\n'
     '    print("event source(s):", dict(srcs), "(0=ext 1=RO7 2=RO11 3=DPS, -1=pre-v4 capture)")\n'
     '    if len(srcs) > 1:\n'
     '        print("  WARNING: this capture mixes event sources -- split it before calibrating")', 1),
])

patch('calib_report.py', 'rail_max', [
    (r"ap\.add_argument\('--out', default='figs'\)",
     "ap.add_argument('--out', default='figs')\n    ap.add_argument('--rail-max', type=int, default=352)", 1),
    (r"a = ap\.parse_args\(\)",
     "a = ap.parse_args()\n    global RAIL\n    RAIL = (0, a.rail_max)", 1),
])

patch('extra_figs.py', 'rail_max', [
    (r"ap\.add_argument\('--build', nargs=2\)",
     "ap.add_argument('--build', nargs=2)\n    ap.add_argument('--rail-max', type=int, default=352)", 1),
    (r"a = ap\.parse_args\(\)",
     "a = ap.parse_args()\n    global RAIL\n    RAIL = (0, a.rail_max)", 1),
])

patch('tb_ro_cd.v', '10 * FRAMES', [
    (r"9 \* FRAMES", "10 * FRAMES", 1),
    (r"nbytes / 9,", "nbytes / 10,", 1),
])