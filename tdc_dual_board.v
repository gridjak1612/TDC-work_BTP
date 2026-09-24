// =============================================================================
// Module Name:  tdc_dual_board
// Description:  Board top for the TWO-CHANNEL interval TDC.
//
//   UART FRAME - 8 bytes per measurement, 8N1 @ 2 Mbaud
//     byte0 : 0xA5                                       sync header
//     byte1 : fine_a[7:0]                                START fine, low 8
//     byte2 : fine_b[7:0]                                STOP  fine, low 8
//     byte3 : d_coarse[7:0]
//     byte4 : {d_coarse[13:8], fine_a[8], fine_b[8]}     coarse high + fine MSBs
//     byte5 : phase_idx[7:0]                             MMCM phase, low 8
//     byte6 : {2'b00, valid_b, valid_a, phase_idx[11:8]} flags + phase high
//     byte7 : seq[7:0]                                   per-measurement counter
//
//   phase_idx is SIGNED 12-bit two's complement. The host must sign-extend it;
//   reading it as unsigned puts every negative phase step at ~4000 instead of
//   just below zero, which folds the low half of the sweep on top of the high
//   half and produces a calibration curve that looks plausible and is wrong.
//
//   seq increments on EVERY meas_ready, framed or not. A gap in the received
//   sequence means a measurement was produced and DROPPED -- distinct from no
//   measurement at all. Code-density work needs that distinction: otherwise a
//   lost sample is indistinguishable from a genuinely empty bin.
//
//   Header is 0xA5, not the old 0xAA, so a stale host fails loudly rather than
//   silently mis-decoding (which is what the 5-vs-6 byte mismatch did).
//
//   Frame time = 8 bytes x 10 bits / 2e6 = 40 us -> 25 kframe/s.
//   CLKS_PER_BIT = 200e6/2e6 = 100 exactly; no divisor error.
//
//   RAW fields are shipped, NOT a computed time. tau_a != tau_b and neither is
//   a single number (bins are non-uniform), so the conversion is a per-channel
//   lookup table that lives on the host and can be re-derived without touching
//   the bitstream.
//
//   Frame time = 6 bytes x 10 bits / 115200 = 521 us -> max ~1.9 kHz.
//   For faster calibration runs raise the baud (CLKS_PER_BIT = 200e6 / baud;
//   921600 -> 217).
//
//   EVENT_SRC is a PARAMETER, not a switch, on purpose.
//   A runtime mux would put a LUT in the launch path of both carry chains,
//   adding delay and skew to the one net whose delay we are trying to measure.
//   As a compile-time parameter the mux constant-folds away and the event goes
//   straight from the IBUF into CYINIT. Build two bitstreams instead.
//
//     EVENT_SRC = 0 : buttons          (bring-up)
//     EVENT_SRC = 1 : external pins    (function generator / laser / cables)
//
//   TIE_CHANNELS = 1 drives BOTH chains from event A. Feed one generator in and
//   you get two independent fine histograms in a single run, AND the spread of
//   the reported "interval" is a direct measurement of the differential
//   precision (sigma_pair = sqrt(2) x sigma_single) with no external reference.
// =============================================================================
`timescale 1ns/1ps

module tdc_dual_board #(
    // 200 MHz / 2 000 000 = 100 EXACTLY. Zero divisor error, unlike
    // 3 Mbaud (66.67 -> 66 -> +1.0 % per bit, 10 % of a bit by the stop).
    // 8 bytes x 10 bits / 2 Mbaud = 40 us/frame -> 25 kframe/s.
    parameter integer CLKS_PER_BIT = 100,    // 200 MHz / 2 Mbaud
    parameter integer EVENT_SRC    = 2,      // 0 = buttons, 1 = external pins
    parameter integer TIE_CHANNELS = 0,      // 1 = drive both chains from A
    parameter integer HB_BIT       = 26,
    parameter integer VIS_BITS     = 24,
    // ---- autonomous DPS sweep ------------------------------------------------
    parameter integer AUTO_SWEEP       = 1,     // 0 = manual buttons only
    parameter integer SWEEP_STEPS      = 280,   // 280 x 17.857 ps = 5.000 ns
    parameter integer SAMPLES_PER_STEP = 256,
    // MMCM phase shift settles in a few VCO cycles; 2000 clk200 (10 us) is
    // ~250 clk_cal periods and is deliberately generous. 280 steps x 10 us
    // = 2.8 ms per traversal, negligible against 280 x 256 x 40 us = 2.9 s of
    // measurement. Do not trim this to save time you will not notice.
    parameter integer SETTLE_CYCLES    = 2000,
    parameter integer TAP_SRC          = 0,   // 1 = XORCY probe build
    parameter integer SYNC_TAP         = 30   // 0 = old raw-event sync
)(
    input  wire        clk100,        // F14
    input  wire        rst,           // J2  btn0
    input  wire        btn_a,         // J5  btn1  manual START
    input  wire        btn_b,         // H2  btn2  manual STOP
    input  wire        btn_clear,     // J1  btn3  manual re-arm
    input  wire        ev_a_ext,      // M14 servo0  external START
    input  wire        ev_b_ext,      // M16 servo1  external STOP
    input  wire        sw_autorearm,  // V2  sw0
    output wire [15:0] led,
    output wire        uart_txd       // U11
);

    localparam integer COARSE_BITS = 14;
    localparam integer FINE_BITS   = 9;

    // -------------------------------------------------------------------------
    // Event source select -- resolved at ELABORATION, so no LUT in the launch
    // path. See the header note.
    // -------------------------------------------------------------------------
    wire event_a_src;
    wire event_b_src;

    generate
    if (EVENT_SRC == 0) begin : g_btn
        assign event_a_src = btn_a;
        assign event_b_src = (TIE_CHANNELS != 0) ? btn_a : btn_b;
    end else if (EVENT_SRC == 1) begin : g_ext
        assign event_a_src = ev_a_ext;
        assign event_b_src = (TIE_CHANNELS != 0) ? ev_a_ext : ev_b_ext;
    end else begin : g_cal            // EVENT_SRC == 2: calibration
        assign event_a_src = 1'b0;    // unused: core drives both chains from clk_cal
        assign event_b_src = 1'b0;
    end
    endgenerate

    // -------------------------------------------------------------------------
    // Core
    // -------------------------------------------------------------------------
    localparam integer CAL_EVENT_MODE = (EVENT_SRC == 2) ? 1 : 0;
    localparam integer PHASE_BITS     = 12;

    (* MARK_DEBUG = "true" *) wire [PHASE_BITS-1:0] ps_phase_idx;
    (* MARK_DEBUG = "true" *) wire                  ps_busy;
    (* MARK_DEBUG = "true" *) wire                  ps_error;
    reg                                             sweep_step_req;
    reg                                             sweep_dir;   // dir for the NEXT step
    reg                                             sweep_dir_q; // dir presented WITH the request
        wire                                            clk_cal;
    (* MARK_DEBUG = "true" *) wire [COARSE_BITS-1:0] d_coarse;
    (* MARK_DEBUG = "true" *) wire [FINE_BITS-1:0]   fine_a, fine_b;
    (* MARK_DEBUG = "true" *) wire                   valid_a, valid_b;
    (* MARK_DEBUG = "true" *) wire                   meas_ready;
    (* MARK_DEBUG = "true" *) wire                   mmcm_locked;
    wire                                            timeout;
    wire                                            done_a, done_b, clk200;
    wire rearm;


    tdc_dual_top #(
        .NUM_CARRY4(88), .TDL_WIDTH(352), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(4), .FINE_LATENCY(12),
        .CAL_EVENT(CAL_EVENT_MODE), .TAP_SRC(TAP_SRC), .SYNC_TAP(SYNC_TAP), .PHASE_BITS(PHASE_BITS)
    ) core (
        .clk100       (clk100),
        .rst          (rst),
        .event_a      (event_a_src),
        .event_b      (event_b_src),
        .clear_status (rearm),
        .d_coarse     (d_coarse),
        .fine_a       (fine_a),
        .fine_b       (fine_b),
        .valid_a      (valid_a),
        .valid_b      (valid_b),
        .timeout      (timeout),
        .meas_ready   (meas_ready),
        .done_a       (done_a),
        .done_b       (done_b),
        .clk200       (clk200),
        .mmcm_locked  (mmcm_locked),
        .ps_step_btn  (btn_a),   // reuse btn_a as PHASE STEP in the calibration build
        .ps_dir_btn   (btn_b),   // reuse btn_b as DIRECTION (held = decrement)
        .ps_step_req  (sweep_step_req),
        .ps_dir_req   (sweep_dir_q),
        .ps_phase_idx (ps_phase_idx),
        .ps_busy      (ps_busy),
        .ps_error     (ps_error),
        .clk_cal      (clk_cal)
    );

    wire rst200 = rst | ~mmcm_locked;

    // -------------------------------------------------------------------------
    // Latch the record on meas_ready.
    //
    // ph_l  : the MMCM phase index that produced this sample. Without it the
    //         host receives fine codes with no idea which phase step they
    //         belong to, and a DPS sweep is not reconstructable -- the samples
    //         are just an undifferentiated pile.
    //
    // seq_l : free-running per-measurement counter. It increments on EVERY
    //         meas_ready, whether or not the record actually gets framed. A
    //         gap in the received sequence therefore means "a measurement
    //         happened and was dropped", which is a completely different thing
    //         from "no measurement happened". For code-density work that is
    //         the difference between a real empty bin and a lost sample, and
    //         previously there was no way to tell them apart.
    // -------------------------------------------------------------------------
    reg [COARSE_BITS-1:0] dc_l;
    reg [FINE_BITS-1:0]   fa_l, fb_l;
    reg                   va_l, vb_l;
    reg [PHASE_BITS-1:0]  ph_l;
    reg [7:0]             seq_l, seq_cnt;

    always @(posedge clk200) begin
        if (rst200) begin
            dc_l <= {COARSE_BITS{1'b0}};
            fa_l <= {FINE_BITS{1'b0}};  fb_l <= {FINE_BITS{1'b0}};
            va_l <= 1'b0;               vb_l <= 1'b0;
            ph_l <= {PHASE_BITS{1'b0}};
            seq_l <= 8'd0;              seq_cnt <= 8'd0;
        end else if (meas_ready) begin
            dc_l    <= d_coarse;
            fa_l    <= fine_a;
            fb_l    <= fine_b;
            va_l    <= valid_a;
            vb_l    <= valid_b;
            ph_l    <= ps_phase_idx;
            seq_l   <= seq_cnt;
            seq_cnt <= seq_cnt + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // UART framing -- 8 bytes, shifted out of a snapshot register.
    //
    //   byte0 : 0xA5                                       sync header
    //   byte1 : fine_a[7:0]
    //   byte2 : fine_b[7:0]
    //   byte3 : d_coarse[7:0]
    //   byte4 : {d_coarse[13:8], fine_a[8], fine_b[8]}
    //   byte5 : phase_idx[7:0]
    //   byte6 : {2'b00, valid_b, valid_a, phase_idx[11:8]}
    //   byte7 : seq[7:0]
    //
    // Header is 0xA5, NOT the old 0xAA. The previous 6-byte frame was parsed by
    // a host that expected 5 bytes; it stayed in sync by luck and silently
    // mis-decoded every field. Changing the header makes an out-of-date host
    // fail loudly instead of quietly reporting wrong numbers.
    //
    // WHY A SHIFT REGISTER AND NOT A BYTE-PER-STATE FSM
    // The whole frame is snapshotted into `sr` at load time. A measurement that
    // completes mid-transmission overwrites the *_l latches but CANNOT corrupt
    // the frame already in flight. The old per-state FSM read the latches live
    // on each byte, so a record landing mid-frame produced a torn frame
    // stitched from two measurements -- invisible on the wire, and indisting-
    // uishable from a real sample on the host.
    // -------------------------------------------------------------------------
    wire       uart_busy;
    reg        uart_send;
    reg [7:0]  uart_byte;
    reg        pending;
    reg        frame_done;
    reg [63:0] sr;
    reg [3:0]  nleft;

    wire [63:0] frame_w = { 8'hA5,
                            fa_l[7:0],
                            fb_l[7:0],
                            dc_l[7:0],
                            {dc_l[13:8], fa_l[8], fb_l[8]},
                            ph_l[7:0],
                            {2'b00, vb_l, va_l, ph_l[11:8]},
                            seq_l };

    always @(posedge clk200) begin
        if (rst200) begin
            uart_send <= 1'b0; uart_byte <= 8'h00; pending <= 1'b0;
            frame_done <= 1'b0; sr <= 64'd0; nleft <= 4'd0;
        end else begin
            uart_send  <= 1'b0;
            frame_done <= 1'b0;

            if (meas_ready) pending <= 1'b1;

            if (nleft == 4'd0) begin
                if (pending && !uart_busy && !uart_send) begin
                    pending   <= 1'b0;
                    uart_byte <= frame_w[63:56];   // header out now
                    sr        <= {frame_w[55:0], 8'h00};
                    uart_send <= 1'b1;
                    nleft     <= 4'd7;             // 7 payload bytes still to go
                end
            end else if (!uart_busy && !uart_send) begin
                uart_byte <= sr[63:56];
                sr        <= {sr[55:0], 8'h00};
                uart_send <= 1'b1;
                nleft     <= nleft - 1'b1;
                if (nleft == 4'd1) frame_done <= 1'b1;
            end
        end
    end

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst (
        .clk (clk200), .rst (rst200), .send (uart_send), .data (uart_byte),
        .tx  (uart_txd), .busy (uart_busy)
    );

    // -------------------------------------------------------------------------
    // AUTONOMOUS DPS SWEEP
    //
    //   COLLECT : let SAMPLES_PER_STEP measurements complete at the current
    //             phase, then request a step.
    //   STEP    : one-cycle step request into dps_phase_ctrl.
    //   SETTLE  : wait for the PS handshake to finish (ps_busy low), then a
    //             fixed settle window, then collect again.
    //
    // Re-arm is BLOCKED outside COLLECT. A measurement taken while the shift is
    // in flight would be tagged with the new phase_idx (which updates the
    // instant the step is requested) but physically sampled at the old phase.
    // Those samples are not merely noisy -- they are mislabelled, and they land
    // in the wrong histogram bin where nothing distinguishes them from good
    // data. Blocking re-arm means they are never taken at all.
    //
    // TRIANGLE, NOT SAWTOOTH: the phase walks 0 -> SWEEP_STEPS-1 -> 0 -> ...
    // Returning by stepping back down costs nothing and buys a free
    // consistency check: the up-sweep and the down-sweep must agree. If they
    // do not, that is MMCM fine-phase-shift hysteresis, which is a real effect
    // and one that a sawtooth-plus-reset sweep would hide completely.
    // Interior phases are visited twice per traversal and the two endpoints
    // once; the host normalises by the actual sample count per phase, so this
    // asymmetry does not bias anything.
    // -------------------------------------------------------------------------
    localparam integer SW_CNT_W  = $clog2(SAMPLES_PER_STEP + 1);
    localparam integer SW_PH_W   = $clog2(SWEEP_STEPS + 1);
    localparam integer SW_SET_W  = $clog2(SETTLE_CYCLES + 1);

    localparam SW_COLLECT = 2'd0, SW_STEP = 2'd1, SW_SETTLE = 2'd2;

    reg [1:0]           sw_state;
    reg [SW_CNT_W-1:0]  sw_count;
    reg [SW_PH_W-1:0]   sw_phase;    // local mirror, for wrap control only
    reg [SW_SET_W-1:0]  sw_settle;
    reg                 sw_mismatch; // sticky: local mirror != ps_phase_idx

    wire sweep_on   = (AUTO_SWEEP != 0) && sw_autorearm;
    wire sweep_hold = sweep_on && (sw_state != SW_COLLECT);

    always @(posedge clk200) begin
        if (rst200) begin
            sw_state       <= SW_COLLECT;
            sw_count       <= 0;
            sw_phase       <= 0;
            sw_settle      <= 0;
            sweep_step_req <= 1'b0;
            sweep_dir      <= 1'b0;      // 0 = increment
            sweep_dir_q    <= 1'b0;
            sw_mismatch    <= 1'b0;
        end else begin
            sweep_step_req <= 1'b0;      // single-cycle request

            if (!sweep_on) begin
                sw_state <= SW_COLLECT;
                sw_count <= 0;
            end else begin
                case (sw_state)

                SW_COLLECT: begin
                    if (meas_ready) begin
                        if (sw_count >= SAMPLES_PER_STEP[SW_CNT_W-1:0] - 1'b1)
                            sw_state <= SW_STEP;
                        else
                            sw_count <= sw_count + 1'b1;
                    end
                end

                SW_STEP: begin
                    sweep_step_req <= 1'b1;
                    // Present the direction that is current NOW, not the one
                    // sweep_dir is about to become. Both are non-blocking, so
                    // handing dps_phase_ctrl `sweep_dir` directly makes the
                    // turnaround step arrive with the direction ALREADY
                    // flipped: the step that should climb to the top of the
                    // range instead descends, and the phase index silently
                    // diverges from the sweep position by two counts at every
                    // reversal.
                    sweep_dir_q    <= sweep_dir;
                    sw_count       <= 0;
                    sw_settle      <= 0;
                    if (!sweep_dir) begin                 // walking up
                        sw_phase <= sw_phase + 1'b1;
                        if (sw_phase + 1'b1 >= SWEEP_STEPS[SW_PH_W-1:0] - 1'b1)
                            sweep_dir <= 1'b1;            // turn around at the top
                    end else begin                        // walking down
                        sw_phase <= sw_phase - 1'b1;
                        if (sw_phase <= 1)
                            sweep_dir <= 1'b0;            // turn around at zero
                    end
                    sw_state <= SW_SETTLE;
                end

                SW_SETTLE: begin
                    if (!ps_busy) begin
                        if (sw_settle >= SETTLE_CYCLES[SW_SET_W-1:0] - 1'b1) begin
                            // The frame carries ps_phase_idx, not sw_phase. They
                            // must agree; if they ever do not, a step was lost
                            // or spuriously applied and every subsequent phase
                            // label in the run is shifted. Latch it so the run
                            // can be thrown away rather than quietly believed.
                            if (ps_phase_idx != {{(PHASE_BITS-SW_PH_W){1'b0}}, sw_phase})
                                sw_mismatch <= 1'b1;
                            sw_state <= SW_COLLECT;
                        end else begin
                            sw_settle <= sw_settle + 1'b1;
                        end
                    end
                end

                default: sw_state <= SW_COLLECT;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // Re-arm. Manual (btn_clear) OR automatic once the frame is out.
    // Gating auto-rearm on frame_done (not on meas_ready) means the re-arm rate
    // is set by the UART, which is incommensurate with any sensible generator
    // frequency -- so the sampled phases stay uncorrelated with the clock.
    // Stretched to 4 cycles so the channel's 2-FF clear synchroniser sees it.
    // -------------------------------------------------------------------------
    reg [2:0] rearm_cnt;
    always @(posedge clk200) begin
        if (rst200)                          rearm_cnt <= 3'd0;
        else if (sw_autorearm && frame_done && !sweep_hold) rearm_cnt <= 3'd4;
        else if (rearm_cnt != 3'd0)          rearm_cnt <= rearm_cnt - 1'b1;
    end

    assign rearm = btn_clear | (rearm_cnt != 3'd0);

    // -------------------------------------------------------------------------
    // Status LEDs
    // -------------------------------------------------------------------------
    reg [31:0] hb_cnt;
    always @(posedge clk200) hb_cnt <= rst200 ? 32'd0 : hb_cnt + 1'b1;

    reg [7:0] meas_cnt;
    always @(posedge clk200)
        if (rst200)           meas_cnt <= 8'd0;
        else if (meas_ready)  meas_cnt <= meas_cnt + 1'b1;

    // stretch the per-measurement flags so a human can see them
    reg [VIS_BITS-1:0] vis;
    reg                va_led, vb_led, tmo_led;
    always @(posedge clk200) begin
        if (rst200) begin
            vis <= 0; va_led <= 1'b0; vb_led <= 1'b0; tmo_led <= 1'b0;
        end else if (meas_ready) begin
            va_led  <= valid_a;
            vb_led  <= valid_b;
            tmo_led <= timeout;
            vis     <= {VIS_BITS{1'b1}};
        end else if (vis != 0) begin
            vis <= vis - 1'b1;
        end else begin
            va_led <= 1'b0; vb_led <= 1'b0; tmo_led <= 1'b0;
        end
    end

    assign led[0]    = hb_cnt[HB_BIT];   // alive
    assign led[1]    = mmcm_locked;      // 200 MHz healthy
    assign led[2]    = ~done_a;          // channel A armed
    assign led[3]    = ~done_b;          // channel B armed
    assign led[4]    = va_led;           // last START code was a legal thermometer
    assign led[5]    = vb_led;           // last STOP  code was a legal thermometer
    assign led[6]    = tmo_led;          // STOP never arrived
    // led[7] : sweep integrity. Solid = a step was lost (sw_mismatch) or a
    // PSDONE was missed (ps_error). Either way the phase labels in this run
    // cannot be trusted -- do not build a LUT from it.
    assign led[7]    = sw_mismatch | ps_error;
    assign led[15:8] = meas_cnt;         // measurement counter

endmodule