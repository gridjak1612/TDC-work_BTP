# Capture manifest

Build parameters below were INFERRED from the data (code range, dead-zone
location), not recorded at capture time. From now on, record them when capturing.

| File | Build (inferred) | Codes (A) | Dead zone | Notes |
|---|---|---|---|---|
| sweep_full.csv | SYNC_TAP=0, TAP_SRC=0 | 1..288 | ph 268..275 (A), 268..275 (B) | Source of the 10.1 ps result and slide-2 even/odd. 185 seq gaps |
| probe.csv, probe2.csv | SYNC_TAP=0, TAP_SRC=0 | 1..288 | same as sweep_full | probe.csv -> lut_a/lut_b |
| sweep_v2.csv | SYNC_TAP=30, TAP_SRC=0 | 84..351 | ph 189..213 (A), 182..214 (B) | 474 seq gaps; 1 corrupt frame (phase = -859) |
| probe_ilv.csv, ilv_check.csv | SYNC_TAP=30, TAP_SRC=0 on both chains (NOT interleaved) | 84..351 | as sweep_v2 | Identical to sweep_v2 at 279/280 phases. Source of slide 4/7 DNL/INL |
| probe3.csv, live.csv | SYNC_TAP=30 | 84..351 | - | live.csv has host-computed interval_ps |
| tdc_dual_log.csv, tdc_log.csv | pre-DPS builds | - | - | historical |
| lut_full_*.csv | from sweep_full | | | |
| lut_v2_*.csv | from sweep_v2 | | | |

## Rule from here on
Name every capture `<date>_<githash>_<desc>.csv`, e.g. `0925_a1b2c3d_sweep_sync30.csv`,
and add one line here: build params, bitstream hash, temperature if known.
A LUT is only valid for the bitstream (placement) it was built from.
