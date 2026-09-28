`timescale 1ns/1ps
// =============================================================================
// tdc_channel_fold -- ONE TDC channel on a FOLDING delay line (folding step 2).
//
// Drop-in for tdc_channel (same ports, FINE_BITS = 10). Differences:
//   single_tdl        -> fold_tdl        (return path D -> B, loop_en gate)
//   352 sampled taps  -> 159: launch 0..31, fold 32..151, counting 152+32j
//   snapshot_pipeline -> snapshot_fold   (DUAL_SNAP decides on launch tap 8)
//   bubble+popcount   -> fold_decode     (lap count + position, latency 10)
//   loop_en flop      : closed only while armed AND the event line has been
//                       seen low (synchronised tap 30); opened at capture.
//   raw_out           : the captured 159-bit snapshot (for the DUMP build)
//
// Geometry (from the return-path probe, tau 17.26 ps/tap):
//   B = tap 32, D = tap 96 (K = 64), E = tap 151 (fold 120 taps), lap 2.06 ns.
// The coarse counter, capture controller and pairing are untouched.
// =============================================================================
module tdc_channel_fold #(
    parameter integer NUM_CARRY4   = 88,
    parameter integer TDL_WIDTH    = 352,
    parameter integer FINE_BITS    = 10,
    parameter integer COARSE_BITS  = 14,
    parameter integer CAPTURE_LAG  = 4,
    parameter integer FINE_LATENCY = 12,   // fold_decode is 10 deep
    parameter integer SYNC_TAP     = 30,   // must stay inside the launch section
    parameter integer DUAL_SNAP    = 1,
    parameter integer B_C4         = 8,    // B = tap 32
    parameter integer K            = 64,   // B -> D
    parameter integer FOLD_W       = 120,  // B .. E
    parameter integer NCNT         = 7,
    parameter integer CNT_STEP     = 32,
    parameter integer HIT_TAP      = 8
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    event_in,
    input  wire                    clear_status,
    input  wire [COARSE_BITS-1:0]  coarse_count,
    output wire [COARSE_BITS-1:0]  coarse_out,
    output wire [FINE_BITS-1:0]    fine_out,
    output wire                    valid_out,
    output wire                    ready,
    output wire                    done,
    output wire [4*B_C4+FOLD_W+NCNT-1:0] raw_out
);
    localparam integer LAUNCH_W = 4 * B_C4;
    localparam integer SW       = LAUNCH_W + FOLD_W + NCNT;

    wire event_cond;
    tdc_frontend frontend_inst (.event_in(event_in), .event_out(event_cond));

    // ---------------------------------------------------------------- chain
    wire [TDL_WIDTH-1:0] tdl_taps;
    reg                  loop_en;
    fold_tdl #(.NUM_CARRY4(NUM_CARRY4), .B_C4(B_C4), .K(K)) tdl_inst (
        .trigger(event_cond), .loop_en(loop_en), .taps(tdl_taps));

    wire sync_src = tdl_taps[SYNC_TAP];

    // ---------------------------------------------------------------- capture
    wire capture_enable;
    capture_controller cap_ctrl_inst (
        .capture_clk(clk), .rst(rst), .stop_pulse(sync_src),
        .clear_status(clear_status), .capture_enable(capture_enable), .done(done));

    // Event level, synchronised, for the loop gate only.
    (* ASYNC_REG = "TRUE" *) reg hs0 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg hs1 = 1'b0;
    always @(posedge clk) begin hs0 <= sync_src; hs1 <= hs0; end

    // Close the loop once the channel is armed and the line has been seen
    // low; open it as soon as a capture is taken (done). While it is open the
    // fold is a plain chain and cannot oscillate.
    always @(posedge clk) begin
        if (rst)        loop_en <= 1'b0;
        else if (done)  loop_en <= 1'b0;
        else if (!hs1)  loop_en <= 1'b1;
    end

    // ---------------------------------------------------------------- sampling
    wire [SW-1:0] sampled_in;
    assign sampled_in[LAUNCH_W+FOLD_W-1:0] = tdl_taps[LAUNCH_W+FOLD_W-1:0];
    genvar j;
    generate
        for (j = 0; j < NCNT; j = j + 1) begin : g_cnt
            assign sampled_in[LAUNCH_W+FOLD_W+j] = tdl_taps[LAUNCH_W+FOLD_W+CNT_STEP*j];
        end
    endgenerate

    wire [SW-1:0] sampled;
    wire          prev_hit;
    wire          use_prev = (DUAL_SNAP != 0) && prev_hit;

    snapshot_fold #(.WIDTH(SW), .DEPTH(CAPTURE_LAG), .HIT_BIT(HIT_TAP)) tap_snap_inst (
        .clk(clk), .rst(rst), .capture_enable(capture_enable), .use_prev(use_prev),
        .din(sampled_in), .captured(sampled), .prev_hit(prev_hit));

    snapshot_pipeline #(.WIDTH(COARSE_BITS), .DEPTH(CAPTURE_LAG)) coarse_snap_inst (
        .clk(clk), .rst(rst), .capture_enable(capture_enable),
        .din(coarse_count), .captured(coarse_out),
        .use_prev(use_prev), .top_now());

    assign raw_out = sampled;

    // ---------------------------------------------------------------- decode
    fold_decode #(.LAUNCH_W(LAUNCH_W), .FOLD_W(FOLD_W), .NCNT(NCNT), .FINE_BITS(FINE_BITS)) dec_inst (
        .clk(clk), .rst(rst), .sampled(sampled), .fine(fine_out), .valid(valid_out));

    reg [FINE_LATENCY-1:0] cap_pipe;
    always @(posedge clk) begin
        if (rst) cap_pipe <= {FINE_LATENCY{1'b0}};
        else     cap_pipe <= {cap_pipe[FINE_LATENCY-2:0], capture_enable};
    end
    assign ready = cap_pipe[FINE_LATENCY-1];

    generate if (SYNC_TAP >= LAUNCH_W || HIT_TAP >= LAUNCH_W ||
                 (LAUNCH_W + FOLD_W + CNT_STEP*(NCNT-1)) >= TDL_WIDTH ||
                 FINE_LATENCY < 11) begin : g_bad
        TDC_CHANNEL_FOLD_BAD_PARAMS u_err ();
    end endgenerate
endmodule
