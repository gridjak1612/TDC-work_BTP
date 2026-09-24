// =============================================================================
// Module Name:  tdc_channel
// Description:  ONE complete TDC measurement channel.
//
//   This is the old `tdc_top` with two things REMOVED:
//     - the MMCM  (there is only one clock for the whole chip)
//     - the coarse counter (it is now SHARED between both channels)
//
//   Sharing the coarse counter is the entire point. Both channels read their
//   coarse value off the SAME free-running counter, so when the interval
//   calculator subtracts the two timestamps, the counter's arbitrary origin
//   cancels EXACTLY. That is what makes an interval measurement possible
//   without ever resetting anything.
//
//   Chain:
//     event_in (async) --> tdc_frontend (async conditioning, pass-through)
//                      --> single_tdl (88x CARRY4 = 352 taps)
//                      --> snapshot_pipeline (free-running, reach back 4 edges)
//                      --> bubble_correction (5-tap majority)
//                      --> ones_counter_encoder_piped  (352 -> 9, LATENCY 7)
//                      --> thermometer_validator_piped (LATENCY 3)
//
//     event_in (async) --> capture_controller (2-FF CDC sync + arm/lockout)
//                      --> capture_enable
//
//   `ready` is capture_enable delayed by FINE_LATENCY, i.e. a single-cycle
//   strobe meaning "coarse_out / fine_out / valid_out are settled and coherent".
//
// LATENCY BUDGET (why FINE_LATENCY = 12):
//     bubble reg (1) + popcount (7) = 8 cycles for fine
//     bubble reg (1) + validator (3) = 4 cycles for valid
//   Both pipelines free-run off `sampled_taps`, which STOPS changing after the
//   capture. So they settle and then HOLD. Waiting longer than necessary costs
//   nothing; waiting too little silently reports a stale value. 12 > 8 > 4.
// =============================================================================
`timescale 1ns/1ps

module tdc_channel #(
    parameter integer NUM_CARRY4   = 88,   // 88 x 4 = 352 taps
    parameter integer TDL_WIDTH    = 352,  // 4 * NUM_CARRY4
    parameter integer FINE_BITS    = 9,    // 0..352 needs 9 bits
    parameter integer COARSE_BITS  = 14,
    parameter integer CAPTURE_LAG  = 4,     // capture_enable lags the golden edge
    parameter integer FINE_LATENCY = 12,    // popcount is now 7 deep: 1(bubble)+7 = 8 < 12
    parameter integer TAP_SRC      = 0,     // 0 = CO taps (normal), 1 = O taps (XORCY probe)

    // -------------------------------------------------------------------------
    // SYNC_TAP -- LAUNCH-SKEW FIX.
    //
    // The capture controller used to take stop_pulse from the RAW event. Its
    // synchroniser therefore decides "which clock edge is the golden one" at
    // the exact instant the carry chain is at tap 0. An event landing inside
    // that flop's setup window resolves either way:
    //     resolves 1 -> capture at edge N,   fine ~ 0
    //     resolves 0 -> capture at edge N+1, fine ~ 293 (a full period later)
    // Both readings rail against the ends of the chain, and a railed code
    // carries no information -- which is the 125-143 ps dead zone measured at
    // sweep phases 268..275.
    //
    // Taking stop_pulse from a tap PART-WAY DOWN the chain moves the
    // synchroniser's decision point away from tap 0. Now the two possible
    // resolutions are fine ~ SYNC_TAP and fine ~ SYNC_TAP + 293 -- and here is
    // the point: BOTH ARE INSIDE THE CHAIN. When the sync resolves late the
    // coarse counter also increments, so
    //     coarse x 5000 ps  -  fine x tau
    // is CONTINUOUS across the boundary: +1 coarse (+5000 ps) exactly cancels
    // +293 fine (-5000 ps). The ambiguity stops being a dead zone and becomes
    // a whole-period ambiguity that the coarse counter already resolves. It
    // only works while fine does not rail, which is why SYNC_TAP must leave
    // room at BOTH ends.
    //
    // Budget: one period spans 5000/17.05 = 293 taps of the 352 available, so
    // there are 59 taps of slack. SYNC_TAP = 30 splits it about evenly:
    // valid fine codes run 30..323, leaving ~30 taps of margin each side for
    // jitter and PVT. Codes outside that window are now DETECTABLY bad rather
    // than silently wrong.
    //
    // The constant SYNC_TAP offset cancels in the A-B difference. It must be
    // subtracted for an absolute timestamp.
    // -------------------------------------------------------------------------
    parameter integer SYNC_TAP     = 30,
    // DUAL_SNAP -- dead-zone fix (step 4): if the chosen edge is already full,
    // take the previous edge's taps AND coarse. 0 = old behaviour.
    parameter integer DUAL_SNAP    = 1
)(
    input  wire                    clk,           // clk200
    input  wire                    rst,           // active high

    input  wire                    event_in,      // ASYNC. Launches the chain.
    input  wire                    clear_status,  // re-arm

    input  wire [COARSE_BITS-1:0]  coarse_count,  // from the SHARED counter

    output wire [COARSE_BITS-1:0]  coarse_out,    // counter value at the golden edge
    output wire [FINE_BITS-1:0]    fine_out,      // taps climbed before that edge
    output wire                    valid_out,     // thermometer code was legal
    output wire                    ready,         // 1-cycle: outputs are settled
    output wire                    done           // captured, locked out
);

    // -------------------------------------------------------------------------
    // Frontend: asynchronous conditioning, applied BEFORE the fanout so that
    // the carry chain and the capture controller both see the SAME conditioned
    // edge. Any delay added here is common to both paths, so it does not change
    // the launch skew -- it is a constant offset that cancels in an interval.
    //
    // It MUST stay combinational. A flip-flop here would quantise the event to
    // clk200 and destroy the sub-nanosecond phase that IS the measurement.
    // -------------------------------------------------------------------------
    wire event_cond;

    tdc_frontend frontend_inst (
        .event_in  (event_in),
        .event_out (event_cond)
    );

    // -------------------------------------------------------------------------
    // Delay line. event_cond goes in RAW and UNSYNCHRONISED - that is
    // deliberate. Synchronising it would destroy the sub-nanosecond phase
    // information that is the whole measurement.
    // -------------------------------------------------------------------------
    wire [TDL_WIDTH-1:0] tdl_taps;

    // Drive the capture synchroniser from a mid-chain tap, not the raw event.
    // SYNC_TAP = 0 restores the old behaviour for A/B comparison.
    wire sync_src = (SYNC_TAP == 0) ? event_cond : tdl_taps[SYNC_TAP];

    single_tdl #(.NUM_CARRY4(NUM_CARRY4), .TAP_SRC(TAP_SRC)) tdl_inst (
        .trigger (event_cond),
        .taps    (tdl_taps)
    );

    // -------------------------------------------------------------------------
    // Capture control. THIS path IS synchronised (2-FF CDC) - it only decides
    // *when* to freeze, it does not carry timing information.
    // -------------------------------------------------------------------------
    wire capture_enable;

    capture_controller cap_ctrl_inst (
        .capture_clk    (clk),
        .rst            (rst),
        .stop_pulse     (sync_src),
        .clear_status   (clear_status),
        .capture_enable (capture_enable),
        .done           (done)
    );

    // -------------------------------------------------------------------------
    // Snapshot pipelines. capture_enable arrives CAPTURE_LAG edges after the
    // golden edge, so we sample every edge into a free-running pipeline and let
    // the late capture reach back and grab the right snapshot.
    // Taps and coarse use the SAME depth -> they describe the SAME clock edge.
    // -------------------------------------------------------------------------
    wire [TDL_WIDTH-1:0]   sampled_taps;
    wire                   tap_full;
    wire                   use_prev = (DUAL_SNAP != 0) && tap_full;

    snapshot_pipeline #(.WIDTH(TDL_WIDTH), .DEPTH(CAPTURE_LAG)) tap_snap_inst (
        .clk (clk), .rst (rst), .capture_enable (capture_enable),
        .din (tdl_taps), .captured (sampled_taps),
        .use_prev (use_prev), .top_now (tap_full)
    );

    snapshot_pipeline #(.WIDTH(COARSE_BITS), .DEPTH(CAPTURE_LAG)) coarse_snap_inst (
        .clk (clk), .rst (rst), .capture_enable (capture_enable),
        .din (coarse_count), .captured (coarse_out),
        .use_prev (use_prev), .top_now ()
    );

    // -------------------------------------------------------------------------
    // Fine encode path (pipelined - this is what timing closure was won on).
    // -------------------------------------------------------------------------
    wire [TDL_WIDTH-1:0] corrected;

    bubble_correction #(.WIDTH(TDL_WIDTH)) bubble_inst (
        .raw_therm (sampled_taps),
        .corrected (corrected)
    );

    // Register the bubble output: first pipeline stage, feeds both consumers.
    reg [TDL_WIDTH-1:0] corrected_r;
    always @(posedge clk) begin
        if (rst) corrected_r <= {TDL_WIDTH{1'b0}};
        else     corrected_r <= corrected;
    end

    ones_counter_encoder_piped #(
        .INPUT_WIDTH (TDL_WIDTH), .OUTPUT_WIDTH (FINE_BITS)
    ) encoder_inst (
        .clk (clk), .rst (rst),
        .thermometer_in (corrected_r),
        .binary_out     (fine_out)
    );

    thermometer_validator_piped #(.WIDTH(TDL_WIDTH)) validator_inst (
        .clk (clk), .rst (rst),
        .thermometer_in (corrected_r),
        .valid          (valid_out)
    );

    // -------------------------------------------------------------------------
    // Ready strobe: capture_enable delayed by the pipeline latency.
    // Correct by construction, not by a hand-tuned settle counter.
    // -------------------------------------------------------------------------
    reg [FINE_LATENCY-1:0] cap_pipe;
    always @(posedge clk) begin
        if (rst) cap_pipe <= {FINE_LATENCY{1'b0}};
        else     cap_pipe <= {cap_pipe[FINE_LATENCY-2:0], capture_enable};
    end

    assign ready = cap_pipe[FINE_LATENCY-1];

endmodule