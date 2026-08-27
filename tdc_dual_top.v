// =============================================================================
// Module Name:  tdc_dual_top
// Description:  TWO-CHANNEL time-interval TDC core.
//
//                              +-------------------+
//   clk100 ------------------->| clk_wiz_0 (MMCM)  |---> clk200 (5 ns)
//                              +-------------------+
//
//                              +-------------------+
//                              | coarse_counter    |  <-- ONE, SHARED
//                              |   14 bit, free-run|
//                              +---------+---------+
//                                        | coarse_count
//                     +------------------+------------------+
//                     |                                     |
//              +------v-------+                      +------v-------+
//   event_a -->| tdc_channel A|                      | tdc_channel B|<-- event_b
//   (START)    |  352-tap TDL |                      |  352-tap TDL |    (STOP)
//              +------+-------+                      +------+-------+
//                     | {coarse_a, fine_a, valid_a}         | {coarse_b, ...}
//                     +------------------+------------------+
//                                        |
//                              +---------v---------+
//                              |interval_calculator|
//                              | d_coarse = b - a  |
//                              +---------+---------+
//                                        | d_coarse, fine_a, fine_b, valids
//                                        v
//
//   THE WHOLE IDEA: one counter, two chains. Both timestamps are read off the
//   SAME free-running counter, so subtracting them cancels the counter's
//   arbitrary origin exactly. No reset, no START-triggered counter, no lost
//   phase. Both events keep their picosecond fine information.
//
//   The host computes:
//       interval = d_coarse * 5.000 ns  -  (tau_b[fine_b] - tau_a[fine_a])
//   using a PER-CHANNEL calibration LUT, because tau_a != tau_b.
// =============================================================================
`timescale 1ns/1ps

module tdc_dual_top #(
    parameter integer NUM_CARRY4   = 88,
    parameter integer TDL_WIDTH    = 352,
    parameter integer FINE_BITS    = 9,
    parameter integer COARSE_BITS  = 14,
    parameter integer CAPTURE_LAG  = 4,
    parameter integer FINE_LATENCY = 12,
    // How long the pairing FSM waits for the STOP before declaring "no echo".
    // Default = one full coarse rollover (16384 cyc = 81.92 us): beyond that a
    // d_coarse cannot be disambiguated anyway.
    // OVERRIDE THIS TO A SMALL VALUE IN SIMULATION -- at the default, the
    // timeout test alone is 82 us of simulated time and dominates the runtime.
    parameter integer TIMEOUT_CYCLES = (1 << COARSE_BITS)
)(
    input  wire                   clk100,
    input  wire                   rst,

    input  wire                   event_a,       // START (async)
    input  wire                   event_b,       // STOP  (async)
    input  wire                   clear_status,  // re-arm both channels

    output wire [COARSE_BITS-1:0] d_coarse,
    output wire [FINE_BITS-1:0]   fine_a,
    output wire [FINE_BITS-1:0]   fine_b,
    output wire                   valid_a,
    output wire                   valid_b,
    output wire                   timeout,
    output wire                   meas_ready,

    output wire                   done_a,
    output wire                   done_b,
    output wire                   clk200,
    output wire                   mmcm_locked
);

    wire clk200_i, mmcm_locked_i;

    clk_wiz_0 clk_gen (
        .clk_in1 (clk100),
        .clk_out1(clk200_i),
        .reset   (rst),
        .locked  (mmcm_locked_i)
    );

    wire rst_i = rst | ~mmcm_locked_i;

    assign clk200      = clk200_i;
    assign mmcm_locked = mmcm_locked_i;

    // -------------------------------------------------------------------------
    // THE SHARED COARSE COUNTER. One instance. Never reset between measurements.
    // -------------------------------------------------------------------------
    wire [COARSE_BITS-1:0] coarse_count;

    coarse_counter #(.WIDTH(COARSE_BITS)) coarse_cnt_inst (
        .clk   (clk200_i),
        .rst   (rst_i),
        .count (coarse_count)
    );

    // -------------------------------------------------------------------------
    // Channel A -- the START event
    // -------------------------------------------------------------------------
    wire [COARSE_BITS-1:0] coarse_a_w;
    wire [FINE_BITS-1:0]   fine_a_w;
    wire                   valid_a_w, ready_a_w;

    tdc_channel #(
        .NUM_CARRY4(NUM_CARRY4), .TDL_WIDTH(TDL_WIDTH), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG),
        .FINE_LATENCY(FINE_LATENCY)
    ) chan_a (
        .clk          (clk200_i),
        .rst          (rst_i),
        .event_in     (event_a),
        .clear_status (clear_status),
        .coarse_count (coarse_count),
        .coarse_out   (coarse_a_w),
        .fine_out     (fine_a_w),
        .valid_out    (valid_a_w),
        .ready        (ready_a_w),
        .done         (done_a)
    );

    // -------------------------------------------------------------------------
    // Channel B -- the STOP event
    // -------------------------------------------------------------------------
    wire [COARSE_BITS-1:0] coarse_b_w;
    wire [FINE_BITS-1:0]   fine_b_w;
    wire                   valid_b_w, ready_b_w;

    tdc_channel #(
        .NUM_CARRY4(NUM_CARRY4), .TDL_WIDTH(TDL_WIDTH), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG),
        .FINE_LATENCY(FINE_LATENCY)
    ) chan_b (
        .clk          (clk200_i),
        .rst          (rst_i),
        .event_in     (event_b),
        .clear_status (clear_status),
        .coarse_count (coarse_count),
        .coarse_out   (coarse_b_w),
        .fine_out     (fine_b_w),
        .valid_out    (valid_b_w),
        .ready        (ready_b_w),
        .done         (done_b)
    );

    // -------------------------------------------------------------------------
    // Pair them and subtract the coarse fields.
    // -------------------------------------------------------------------------
    interval_calculator #(
        .COARSE_BITS(COARSE_BITS), .FINE_BITS(FINE_BITS),
        .TIMEOUT_CYCLES(TIMEOUT_CYCLES)
    ) interval_inst (
        .clk         (clk200_i),
        .rst         (rst_i),
        .ready_a     (ready_a_w),
        .coarse_a    (coarse_a_w),
        .fine_a      (fine_a_w),
        .valid_a     (valid_a_w),
        .ready_b     (ready_b_w),
        .coarse_b    (coarse_b_w),
        .fine_b      (fine_b_w),
        .valid_b     (valid_b_w),
        .d_coarse    (d_coarse),
        .fine_a_out  (fine_a),
        .fine_b_out  (fine_b),
        .valid_a_out (valid_a),
        .valid_b_out (valid_b),
        .timeout     (timeout),
        .meas_ready  (meas_ready)
    );

endmodule