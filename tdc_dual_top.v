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
    parameter integer CAL_EVENT  = 0,     // 1 = drive BOTH chains from clk_cal (calibration)
    parameter integer PHASE_BITS = 12,
    // How long the pairing FSM waits for the STOP before declaring "no echo".
    // Default = one full coarse rollover (16384 cyc = 81.92 us): beyond that a
    // d_coarse cannot be disambiguated anyway.
    // OVERRIDE THIS TO A SMALL VALUE IN SIMULATION -- at the default, the
    // timeout test alone is 82 us of simulated time and dominates the runtime.
    parameter integer TIMEOUT_CYCLES = (1 << COARSE_BITS),
    parameter integer TAP_SRC        = 0,    // 1 = XORCY probe build
    parameter integer SYNC_TAP       = 30,    // 0 = old raw-event sync
    parameter integer DUAL_SNAP      = 1     // step 4 dead-zone fix
)(
    input  wire                   clk100,
    input  wire                   rst,

    input  wire                   event_a,       // START (async)
    input  wire                   event_b,       // STOP  (async)
    input  wire                   clear_status,  // re-arm both channels
    input  wire                   ps_step_btn,   // async, calibration build only
    input  wire                    ps_dir_btn,    // async, calibration build only
    input  wire                    ps_step_req,   // SYNC 1-cycle, from sweep FSM
    input  wire                    ps_dir_req,    // SYNC level, 1 = decrement
    output wire [PHASE_BITS-1:0]   ps_phase_idx,  // signed running phase index
    output wire                    ps_busy,
    output wire                    ps_error,      // sticky: a PSDONE was missed
    output wire                    clk_cal ,       // 25 MHz cal clock (for ILA/observation)

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

    wire clk200_i, clk_cal_i, mmcm_locked_i;
    wire psen_i, psincdec_i, psdone_i;

    clk_wiz_0 clk_gen (
        .clk_in1  (clk100),
        .clk_out1 (clk200_i),
        .clk_out2 (clk_cal_i),
        .psclk    (clk200_i),     // PS interface clocked by the 200 MHz domain
        .psen     (psen_i),
        .psincdec (psincdec_i),
        .psdone   (psdone_i),
        .reset    (rst),
        .locked   (mmcm_locked_i)
    );
    assign clk_cal = clk_cal_i;

    wire rst_i = rst | ~mmcm_locked_i;
        // Calibration event injection: constant-folds, so nothing enters the launch
    // net at runtime -- same discipline as EVENT_SRC. Both chains tied to the
    // SAME cal edge, so one sweep yields independent START and STOP histograms.
    wire event_a_i, event_b_i;
    generate
        if (CAL_EVENT != 0) begin : g_cal
            assign event_a_i = clk_cal_i;
            assign event_b_i = clk_cal_i;
        end else begin : g_norm
            assign event_a_i = event_a;
            assign event_b_i = event_b;
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Dynamic phase shift -- CALIBRATION BUILDS ONLY.
    //
    // This used to be instantiated unconditionally, with ps_step_btn wired to
    // btn_a at the board level. In an EVENT_SRC=0 build btn_a is the START
    // event, so every manual START also stepped the MMCM phase: the sampler
    // walked out from under the measurement, silently, with nothing in the
    // data to show it. Guarding on CAL_EVENT makes the two roles mutually
    // exclusive by construction instead of by remembering.
    // -------------------------------------------------------------------------
    generate
    if (CAL_EVENT != 0) begin : g_dps
        dps_phase_ctrl #(.PHASE_BITS(PHASE_BITS)) dps_ctrl_inst (
            .psclk     (clk200_i),
            .rst       (rst_i),
            .step_btn  (ps_step_btn),
            .dir_btn   (ps_dir_btn),
            .step_req  (ps_step_req),
            .dir_req   (ps_dir_req),
            .psdone    (psdone_i),
            .psen      (psen_i),
            .psincdec  (psincdec_i),
            .phase_idx (ps_phase_idx),
            .busy      (ps_busy),
            .ps_error  (ps_error)
        );
    end else begin : g_no_dps
        assign psen_i       = 1'b0;
        assign psincdec_i   = 1'b0;
        assign ps_phase_idx = {PHASE_BITS{1'b0}};
        assign ps_busy      = 1'b0;
        assign ps_error     = 1'b0;
    end
    endgenerate

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
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG), .TAP_SRC(TAP_SRC), .SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP),
        .FINE_LATENCY(FINE_LATENCY)
    ) chan_a (
        .clk          (clk200_i),
        .rst          (rst_i),
        
        .clear_status (clear_status),
        .coarse_count (coarse_count),
        .coarse_out   (coarse_a_w),
        .fine_out     (fine_a_w),
        .valid_out    (valid_a_w),
        .ready        (ready_a_w),
        .done         (done_a),
        .event_in     (event_a_i)
    );

    // -------------------------------------------------------------------------
    // Channel B -- the STOP event
    // -------------------------------------------------------------------------
    wire [COARSE_BITS-1:0] coarse_b_w;
    wire [FINE_BITS-1:0]   fine_b_w;
    wire                   valid_b_w, ready_b_w;

    tdc_channel #(
        .NUM_CARRY4(NUM_CARRY4), .TDL_WIDTH(TDL_WIDTH), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG), .TAP_SRC(TAP_SRC), .SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP),
        .FINE_LATENCY(FINE_LATENCY)
    ) chan_b (
        .clk          (clk200_i),
        .rst          (rst_i),
        
        .clear_status (clear_status),
        .coarse_count (coarse_count),
        .coarse_out   (coarse_b_w),
        .fine_out     (fine_b_w),
        .valid_out    (valid_b_w),
        .ready        (ready_b_w),
        .event_in     (event_b_i),
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