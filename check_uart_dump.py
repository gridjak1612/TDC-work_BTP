#!/usr/bin/env python3
"""check_uart_dump.py dump.txt -- decode a simulation UART dump (v4 frames) with read_tdc.Framer."""
import sys
import read_tdc

path = sys.argv[1] if len(sys.argv) > 1 else "uart_bytes.txt"
data = bytes(int(l, 16) for l in open(path)
             if l.strip() and 'x' not in l.lower() and 'z' not in l.lower())
fr = read_tdc.Framer()
rows = fr.feed(data)
seqs = [r['seq'] for r in rows]
gaps = sum((seqs[i] - seqs[i - 1] - 1) & 0xFF for i in range(1, len(seqs)))
print(f"{len(rows)} frames from {len(data)} bytes | crc_errors={fr.crc_errors} "
      f"resyncs={fr.resyncs} seq_gaps={gaps}")
runs = []
for r in rows:
    if not runs or runs[-1][0] != r['src']:
        runs.append([r['src'], 0])
    runs[-1][1] += 1
print("source runs in order:",
      ", ".join(f"{read_tdc.SRC_NAMES.get(s, s)} x{n}" for s, n in runs))
print("cfg values:", sorted(set(r['cfg'] for r in rows)))
print("fine_a codes:", sorted(set(r['fine_a'] for r in rows))[:12],
      " fine_b:", sorted(set(r['fine_b'] for r in rows))[:12])
print("valid_a all 1:", all(r['valid_a'] for r in rows),
      " valid_b all 1:", all(r['valid_b'] for r in rows))