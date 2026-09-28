`timescale 1ns/1ps
// =============================================================================
// snapshot_fold -- snapshot_pipeline with the dead-zone test moved.
//
// Identical free-running history of the sampled bits (reach back a late
// capture_enable to the right clock edge), but the DUAL_SNAP decision bit is
// no longer "top tap of the chosen edge" (meaningless on a folding chain,
// whose top tap toggles as laps pass). It is
//     prev_hit = pipe[DEPTH-1][HIT_BIT]
// = "had the hit already reached launch tap HIT_BIT at the PREVIOUS edge?"
// The launch section never sees return edges, so that bit is monotonic.
// If yes, the previous edge is the golden edge and the channel uses it.
// For a plain chain this is equivalent to the old tap-351 test.
// =============================================================================
module snapshot_fold #(
    parameter integer WIDTH   = 159,
    parameter integer DEPTH   = 4,
    parameter integer HIT_BIT = 8
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             capture_enable,
    input  wire             use_prev,
    input  wire [WIDTH-1:0] din,
    output reg  [WIDTH-1:0] captured,
    output wire             prev_hit
);
    (* SHREG_EXTRACT = "NO", DONT_TOUCH = "TRUE" *)
    reg [WIDTH-1:0] tap_reg;
    always @(posedge clk) tap_reg <= din;

    reg [WIDTH-1:0] pipe [0:DEPTH-1];
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DEPTH; i = i + 1) pipe[i] <= {WIDTH{1'b0}};
            captured <= {WIDTH{1'b0}};
        end else begin
            pipe[0] <= tap_reg;
            for (i = 1; i < DEPTH; i = i + 1) pipe[i] <= pipe[i-1];
            if (capture_enable) captured <= use_prev ? pipe[DEPTH-1] : pipe[DEPTH-2];
        end
    end

    assign prev_hit = pipe[DEPTH-1][HIT_BIT];
endmodule
