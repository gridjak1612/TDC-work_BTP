`timescale 1ns/1ps
// =============================================================================
// snapshot_pipeline -- free-running history of taps (or coarse count); a late
// capture_enable reaches back to the right clock edge.
//
//   captured <= use_prev ? pipe[DEPTH-1] : pipe[DEPTH-2]     on capture_enable
//   top_now   = pipe[DEPTH-2][WIDTH-1]   (top bit of the default edge)
//
// DEAD-ZONE FIX (step 4, dual snapshot). When the synchroniser picks an edge
// one clock late, the chain is already full at that edge (code would rail at
// WIDTH). The previous edge holds the same event one period (~293 taps)
// earlier, well inside the chain. The tap instance exports top_now; the
// channel feeds it back as use_prev to BOTH the tap and the coarse instance,
// so taps and coarse always describe the same edge.
// =============================================================================
module snapshot_pipeline #(
    parameter WIDTH = 256,
    parameter DEPTH = 4
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             capture_enable,
    input  wire             use_prev,
    input  wire [WIDTH-1:0] din,
    output reg  [WIDTH-1:0] captured,
    output wire             top_now
);
    (* SHREG_EXTRACT = "NO", DONT_TOUCH = "TRUE" *)
    reg [WIDTH-1:0] tap_reg;
    always @(posedge clk) tap_reg <= din;

    // One stage deeper than before: pipe[DEPTH-1] is the edge before pipe[DEPTH-2].
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

    assign top_now = pipe[DEPTH-2][WIDTH-1];
endmodule