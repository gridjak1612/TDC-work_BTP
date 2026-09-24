#!/usr/bin/env python3
"""check_uart_dump.py uart_bytes.txt -- decode an xsim UART dump with read_tdc.Framer."""
import sys
import read_tdc

data = bytes(int(l, 16) for l in open(sys.argv[1])
             if l.strip() and 'x' not in l.lower() and 'z' not in l.lower())
fr = read_tdc.Framer()
rows = fr.feed(data)
seqs = [r['seq'] for r in rows]
gaps = sum((seqs[i] - seqs[i - 1] - 1) & 0xFF for i in range(1, len(seqs)))
print(f"{len(rows)} frames from {len(data)} bytes | crc_errors={fr.crc_errors} "
      f"resyncs={fr.resyncs} seq_gaps={gaps}")
print("fine_a codes seen:", sorted(set(r['fine_a'] for r in rows)),
      " fine_b:", sorted(set(r['fine_b'] for r in rows)))
print("valid_a all 1:", all(r['valid_a'] for r in rows),
      " valid_b all 1:", all(r['valid_b'] for r in rows))