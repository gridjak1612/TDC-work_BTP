// =============================================================================
// Module Name:  ones_counter_encoder_piped
// Description:  WIDTH-GENERIC pipelined population count.
//               Thermometer code (INPUT_WIDTH bits) -> binary fine value.
//
//   The previous version was HARDCODED for 256 bits (16 chunks x 16, tree
//   16->8->4->2->1, saturate via "test bit 8"). None of that survives a change
//   of chain length, so this is rebuilt generically.
//
//   STRUCTURE
//     Pad the input up to NPAD = 512 bits with zeros.
//       -> 32 chunks of 16 bits
//       -> balanced adder tree 32->16->8->4->2->1  (5 levels)
//     The padding bits are CONSTANT ZERO, so synthesis folds and removes the
//     unused chunks and the tree nodes above them. A 256-tap build costs
//     exactly what the old hand-written 256-bit version did; a 352-tap build
//     just keeps more of the tree.
//
//   ONE ADDITION PER PIPELINE STAGE. That is what won timing closure the first
//   time (the original single-cloud popcount was 64 logic levels, -32.9 ns WNS)
//   and the property is preserved here.
//
//     S0  : 32 chunk popcounts (16 bits each -> 0..16)
//     S1  : 16 pairwise sums
//     S2  :  8 pairwise sums
//     S3  :  4 pairwise sums
//     S4  :  2 pairwise sums
//     S5  :  1 final sum        (0 .. INPUT_WIDTH)
//     S6  : register the (saturated) output
//
//   LATENCY = 7 clk cycles.
//
//   NOTE ON SATURATION
//   With INPUT_WIDTH = 352 and OUTPUT_WIDTH = 9, the count maxes at 352 and
//   9 bits hold 0..511 -- so the clamp NEVER fires and the raw count is the
//   output. "Chain full" is therefore fine == INPUT_WIDTH (352), NOT 511.
//   The host must compare against INPUT_WIDTH, not against the all-ones code.
//   (For the old 256/8-bit build the clamp did fire, which is why 256 counts
//   appeared as 255 -- the ambiguity that made `fine=255` mean two things.)
// =============================================================================
`timescale 1ns/1ps

module ones_counter_encoder_piped #(
    parameter integer INPUT_WIDTH  = 352,   // 4 * NUM_CARRY4
    parameter integer OUTPUT_WIDTH = 9      // ceil(log2(INPUT_WIDTH+1))
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire [INPUT_WIDTH-1:0]  thermometer_in,
    output reg  [OUTPUT_WIDTH-1:0] binary_out
);

    localparam integer CW     = 16;    // bits per chunk
    localparam integer NCHUNK = 32;    // fixed -> NPAD = 512
    localparam integer NPAD   = NCHUNK * CW;

    // Zero-pad. These bits are constants; synthesis prunes everything they feed.
    wire [NPAD-1:0] padded = {{(NPAD-INPUT_WIDTH){1'b0}}, thermometer_in};

    integer c, b, k;
    reg [4:0] cnt;

    // ---- S0: per-chunk popcount, 16 bits -> 0..16 (5 bits) ------------------
    reg [4:0] s0 [0:31];
    always @(posedge clk) begin
        for (c = 0; c < 32; c = c + 1) begin
            cnt = 5'd0;
            for (b = 0; b < CW; b = b + 1)
                cnt = cnt + padded[c*CW + b];
            s0[c] <= rst ? 5'd0 : cnt;
        end
    end

    // ---- S1: 16 pairwise sums (0..32) ---------------------------------------
    reg [5:0] s1 [0:15];
    always @(posedge clk)
        for (k = 0; k < 16; k = k + 1)
            s1[k] <= rst ? 6'd0 : (s0[2*k] + s0[2*k+1]);

    // ---- S2: 8 pairwise sums (0..64) ----------------------------------------
    reg [6:0] s2 [0:7];
    always @(posedge clk)
        for (k = 0; k < 8; k = k + 1)
            s2[k] <= rst ? 7'd0 : (s1[2*k] + s1[2*k+1]);

    // ---- S3: 4 pairwise sums (0..128) ---------------------------------------
    reg [7:0] s3 [0:3];
    always @(posedge clk)
        for (k = 0; k < 4; k = k + 1)
            s3[k] <= rst ? 8'd0 : (s2[2*k] + s2[2*k+1]);

    // ---- S4: 2 pairwise sums (0..256) ---------------------------------------
    reg [8:0] s4 [0:1];
    always @(posedge clk)
        for (k = 0; k < 2; k = k + 1)
            s4[k] <= rst ? 9'd0 : (s3[2*k] + s3[2*k+1]);

    // ---- S5: final sum (0..512) ---------------------------------------------
    reg [9:0] total;
    always @(posedge clk)
        total <= rst ? 10'd0 : (s4[0] + s4[1]);

    // ---- S6: register the output, clamped to OUTPUT_WIDTH --------------------
    localparam [9:0] MAXOUT = (10'd1 << OUTPUT_WIDTH) - 10'd1;

    always @(posedge clk) begin
        if (rst)
            binary_out <= {OUTPUT_WIDTH{1'b0}};
        else
            binary_out <= (total > MAXOUT) ? MAXOUT[OUTPUT_WIDTH-1:0]
                                           : total[OUTPUT_WIDTH-1:0];
    end

endmodule