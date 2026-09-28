// =============================================================================
// Module Name:  thermometer_validator_piped
// Description:  WIDTH-GENERIC pipelined thermometer-code validator.
//               valid = the code has AT MOST ONE 1->0 transition.
//
//   The previous version was HARDCODED for 256 bits (a7[127:0], a6[63:0], ...),
//   so it does not survive a change of chain length. Rebuilt generically.
//
//   ALGORITHM (unchanged)
//     transition vector  t[i] = therm[i] & ~therm[i+1]
//     Each tree node carries TWO bits:
//       any = at least one transition somewhere in this subrange
//       ge2 = at least two transitions in this subrange
//     combine(L,R):  any = aL | aR
//                    ge2 = gL | gR | (aL & aR)
//     valid = ~ge2(root)
//
//   This is the fix for the original 255-iteration sequential accumulator,
//   which synthesised into a 255-element-deep ripple chain and was the single
//   worst timing path in the whole design (-32.8 ns).
//
//   STRUCTURE
//     Pad the transition vector to NPAD = 512 with zeros (a zero transition
//     bit is the identity for both any and ge2, so padding is safe and the
//     constants are folded away by synthesis).
//     Three registered stages, each collapsing 8:1 (three combinational levels):
//       stage 1 : 512 -> 64
//       stage 2 :  64 ->  8
//       stage 3 :   8 ->  1   -> valid
//
//   LATENCY = 3 clk cycles.
//
//   WHAT THIS DOES *NOT* CATCH
//     An all-zeros code and an all-ones code both have ZERO transitions, so
//     both are "valid". A drained chain (fine=0) and a saturated chain
//     (fine=INPUT_WIDTH) are legal thermometer codes. This flag detects
//     BUBBLES, not railing. The host must reject rails separately.
// =============================================================================
`timescale 1ns/1ps

module thermometer_validator_piped #(
    parameter integer WIDTH = 352
)(
    input  wire             clk,
    input  wire             rst,
    input  wire [WIDTH-1:0] thermometer_in,
    output reg              valid
);

    localparam integer NPAD = 512;

    // ---- transition vector, zero-padded up to NPAD --------------------------
    wire [WIDTH-1:0] t;
    genvar gi;
    generate
        for (gi = 0; gi < WIDTH-1; gi = gi + 1) begin : trans
            assign t[gi] = thermometer_in[gi] & ~thermometer_in[gi+1];
        end
    endgenerate
    assign t[WIDTH-1] = 1'b0;

    wire [NPAD-1:0] tp = {{(NPAD-WIDTH){1'b0}}, t};

    integer j;

    // ---- Stage 1 (comb): 512 -> 256 -> 128 -> 64, then register --------------
    reg [255:0] a8, g8;
    reg [127:0] a7, g7;
    reg [63:0]  a6c, g6c;
    reg [63:0]  a6, g6;          // registered

    always @(*) begin
        for (j = 0; j < 256; j = j + 1) begin
            a8[j] = tp[2*j] | tp[2*j+1];
            g8[j] = tp[2*j] & tp[2*j+1];
        end
        for (j = 0; j < 128; j = j + 1) begin
            a7[j] = a8[2*j] | a8[2*j+1];
            g7[j] = g8[2*j] | g8[2*j+1] | (a8[2*j] & a8[2*j+1]);
        end
        for (j = 0; j < 64; j = j + 1) begin
            a6c[j] = a7[2*j] | a7[2*j+1];
            g6c[j] = g7[2*j] | g7[2*j+1] | (a7[2*j] & a7[2*j+1]);
        end
    end

    always @(posedge clk) begin
        if (rst) begin a6 <= 64'd0; g6 <= 64'd0; end
        else     begin a6 <= a6c;   g6 <= g6c;   end
    end

    // ---- Stage 2 (comb): 64 -> 32 -> 16 -> 8, then register ------------------
    reg [31:0] a5, g5;
    reg [15:0] a4, g4;
    reg [7:0]  a3c, g3c;
    reg [7:0]  a3, g3;           // registered

    always @(*) begin
        for (j = 0; j < 32; j = j + 1) begin
            a5[j] = a6[2*j] | a6[2*j+1];
            g5[j] = g6[2*j] | g6[2*j+1] | (a6[2*j] & a6[2*j+1]);
        end
        for (j = 0; j < 16; j = j + 1) begin
            a4[j] = a5[2*j] | a5[2*j+1];
            g4[j] = g5[2*j] | g5[2*j+1] | (a5[2*j] & a5[2*j+1]);
        end
        for (j = 0; j < 8; j = j + 1) begin
            a3c[j] = a4[2*j] | a4[2*j+1];
            g3c[j] = g4[2*j] | g4[2*j+1] | (a4[2*j] & a4[2*j+1]);
        end
    end

    always @(posedge clk) begin
        if (rst) begin a3 <= 8'd0; g3 <= 8'd0; end
        else     begin a3 <= a3c;  g3 <= g3c;  end
    end

    // ---- Stage 3 (comb): 8 -> 4 -> 2 -> 1, register valid --------------------
    reg [3:0] a2, g2;
    reg [1:0] a1, g1;
    reg       a0, g0;

    always @(*) begin
        for (j = 0; j < 4; j = j + 1) begin
            a2[j] = a3[2*j] | a3[2*j+1];
            g2[j] = g3[2*j] | g3[2*j+1] | (a3[2*j] & a3[2*j+1]);
        end
        for (j = 0; j < 2; j = j + 1) begin
            a1[j] = a2[2*j] | a2[2*j+1];
            g1[j] = g2[2*j] | g2[2*j+1] | (a2[2*j] & a2[2*j+1]);
        end
        a0 = a1[0] | a1[1];
        g0 = g1[0] | g1[1] | (a1[0] & a1[1]);
    end

    always @(posedge clk) begin
        if (rst) valid <= 1'b0;
        else     valid <= ~g0;      // at most one transition
    end

endmodule