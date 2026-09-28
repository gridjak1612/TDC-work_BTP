// =============================================================================
// Module Name:  bubble_correction
// Description:  5-tap sliding-window majority-vote filter to correct bubble
//               errors in a captured thermometer code from the TDL.
//
//               In real hardware, the CARRY4 chain does not produce a perfect
//               thermometer pattern. Due to routing skew, metastability, and
//               process variation, isolated 0s can appear inside the block of
//               1s, and isolated 1s can appear inside the block of 0s.
//
//               Example (ideal):    11111111110000000000
//               Example (bubbles):  11110111110100000000
//                                        ^     ^
//                                      bubble  bubble
//
//               This module applies a 5-tap sliding-window majority filter:
//               For each tap i, the corrected output is the majority vote of
//               taps [i-2], [i-1], [i], [i+1], [i+2].
//
//               Majority of 5 bits: output 1 if 3 or more inputs are 1.
//
//               A 5-tap window corrects up to 2 adjacent bubble errors,
//               which is stronger than the 3-tap filter commonly seen in
//               introductory TDC implementations.
//
//               Reference: Wu & Shi 2008, Song et al. 2006.
//
// Parameters:   WIDTH - The number of taps (must match TDL width, default: 256)
//
// Inputs:       raw_therm  - Captured thermometer code (with potential bubbles)
//
// Outputs:      corrected  - Clean thermometer code after majority filtering
// =============================================================================

`timescale 1ns/1ps

module bubble_correction #(
    parameter WIDTH = 256  // Must match TDL tap count (64 CARRY4 x 4)
)(
    input  wire [WIDTH-1:0] raw_therm,  // Raw captured thermometer code
    output wire [WIDTH-1:0] corrected   // Bubble-corrected thermometer code
);

    // -------------------------------------------------------------------------
    // 5-Tap Majority Vote Logic
    //
    // For interior bits (2 <= i <= WIDTH-3):
    //   corrected[i] = majority(raw[i-2], raw[i-1], raw[i], raw[i+1], raw[i+2])
    //
    // Majority of 5 bits = 1 when 3 or more inputs are 1.
    // Implemented as: sum of 5 bits >= 3.
    //
    // Boundary handling -- THE PADDING IS ASYMMETRIC. READ THIS BEFORE EDITING.
    //
    // The TDL fills from tap 0 UPWARD: taps[0 .. fine-1] = 1, taps[fine ..] = 0.
    // So the virtual taps BELOW index 0 are logically ONE, and the virtual taps
    // ABOVE index WIDTH-1 are logically ZERO.
    //
    //   - Bit 0:       window = {1,        1,        raw[0],   raw[1],   raw[2]}
    //   - Bit 1:       window = {1,        raw[0],   raw[1],   raw[2],   raw[3]}
    //   - Bit WIDTH-2: window = {raw[W-4], raw[W-3], raw[W-2], raw[W-1], 0}
    //   - Bit WIDTH-1: window = {raw[W-3], raw[W-2], raw[W-1], 0,        0}
    //
    // Padding the LOW end with ZEROS (as an earlier revision did) gives
    //   corrected[0] = raw[0] & raw[1] & raw[2]
    //   corrected[1] = (raw[0]+raw[1]+raw[2]+raw[3] >= 3)
    // and fine codes 1 and 2 BOTH collapse to 0 on every single sample. Bin 0
    // then absorbs three taps (~52 ps at 17.4 ps/tap) -- a systematic DNL
    // defect LARGER than the single-shot precision of the instrument, and one
    // that looks exactly like a silicon artefact in a code-density histogram.
    // It is not silicon. It is this comment having been written upside down.
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // Bit 0: window = {1, 1, raw[0], raw[1], raw[2]}
    // Two padded ones, so majority (>=3 of 5) reduces to sum(raw[0:2]) >= 1.
    // -------------------------------------------------------------------------
    // window = {1, 1, raw[0], raw[1], raw[2]} -> majority when sum >= 1
    wire [2:0] sum_0;
    assign sum_0 = raw_therm[0] + raw_therm[1] + raw_therm[2];
    assign corrected[0] = (sum_0 >= 3'd1);

    // -------------------------------------------------------------------------
    // Bit 1: window = {1, raw[0], raw[1], raw[2], raw[3]}
    // One padded one, so majority (>=3 of 5) reduces to sum(raw[0:3]) >= 2.
    // -------------------------------------------------------------------------
    // window = {1, raw[0], raw[1], raw[2], raw[3]} -> majority when sum >= 2
    wire [2:0] sum_1;
    assign sum_1 = raw_therm[0] + raw_therm[1] + raw_therm[2] + raw_therm[3];
    assign corrected[1] = (sum_1 >= 3'd2);

    // -------------------------------------------------------------------------
    // Interior Bits [2 .. WIDTH-3]: full 5-tap window
    // -------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 2; i < WIDTH - 2; i = i + 1) begin : majority_vote
            wire [2:0] sum_i;
            assign sum_i = raw_therm[i-2] + raw_therm[i-1] + raw_therm[i]
                         + raw_therm[i+1] + raw_therm[i+2];
            assign corrected[i] = (sum_i >= 3'd3);
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Bit WIDTH-2: window = {raw[W-4], raw[W-3], raw[W-2], raw[W-1], 0}
    // -------------------------------------------------------------------------
    wire [2:0] sum_wm2;
    assign sum_wm2 = raw_therm[WIDTH-4] + raw_therm[WIDTH-3]
                   + raw_therm[WIDTH-2] + raw_therm[WIDTH-1];
    assign corrected[WIDTH-2] = (sum_wm2 >= 3'd3);

    // -------------------------------------------------------------------------
    // Bit WIDTH-1: window = {raw[W-3], raw[W-2], raw[W-1], 0, 0}
    // -------------------------------------------------------------------------
    wire [2:0] sum_wm1;
    assign sum_wm1 = raw_therm[WIDTH-3] + raw_therm[WIDTH-2] + raw_therm[WIDTH-1];
    assign corrected[WIDTH-1] = (sum_wm1 >= 3'd3);

endmodule