// =============================================================================
// Module Name:  tdc_dual_board
// Description:  Board top for the TWO-CHANNEL interval TDC.
//
//   UART FRAME v4 - 10 bytes per measurement, 8N1 @ 2 Mbaud
//     byte0    : 0xA7                                   sync header
//     byte1..8 : 64-bit payload, MSB first:
//                  fine_a[10:0]  fine_b[10:0]  d_coarse[13:0]  phase[11:0]
//                  valid_a  valid_b  seq[7:0]  cfg[5:0]
//                cfg = {encoder_id[2:0], dual_snap, source[1:0]}
//                source: 0 = external/buttons, 1 = RO 7-stage, 2 = RO 11-stage,
//                        3 = DPS clk_cal
//     byte9    : CRC-8 (poly 0x07, init 0x00) over bytes 0..8
//
//   WHY v4: multi-edge / folding encoders produce more than 511 codes, so the
//   fine fields are 11 bits (up to 2047). cfg makes every capture describe
//   itself: which event source and which encoder produced it. Header moved
//   0xA6 -> 0xA7 so a v3 host fails loudly instead of mis-parsing.
//
//   phase_idx is SIGNED 12-bit two's complement; the host sign-extends it.
//
//   seq increments on EVERY meas_ready, framed or not. A gap in the received
//   sequence means a measurement was produced and DROPPED -- distinct from no
//   measurement at all.
//
//   Frame time = 10 bytes x 10 bits / 2e6 = 50 us -> 20 kframe/s.
//   CLKS_PER_BIT = 200e6/2e6 = 100 exactly; no divisor error.
//
//   RAW fields are shipped, NOT a computed time; the per-channel LUT lives on
//   the host.
//
//   EVENT SOURCE
//     EVENT_SRC = 0 : buttons          (bring-up)
//     EVENT_SRC = 1 : external pins    (function generator / laser / cables)
//     EVENT_SRC = 2 : clk_cal + DPS sweep (phase-referenced calibration)
//     EVENT_SRC = 3 : on-chip ring oscillator, both chains (code density).
//                     Needs ro.xdc.
//     EVENT_SRC = 4 : RUN-TIME SELECT by switches (sw2 sw1):
//                       00 external pins, 01 RO 7-stage, 10 RO 11-stage,
//                       11 DPS clk_cal + sweep.
//                     Needs ro_multi.xdc (NOT ro.xdc). Set the switches, then
//                     press BTN0.
//   Builds 0..3 constant-fold the source: the event goes straight into CYINIT.
//   Build 4 puts ONE LUT in front of CYINIT for every source. That LUT adds a
//   constant delay per source, which calibration removes; it does not change
//   the bins, which belong to the carry chain. What it buys: calibrate (RO),
//   cross-check (DPS) and measure (external) on the SAME bitstream -- and a
//   LUT is only valid for the bitstream it was measured on.
//
//   TIE_CHANNELS = 1 drives BOTH chains from event A (builds 0/1 only).
// =============================================================================
`timescale 1ns/1ps

module tdc_dual_board #(
    // 200 MHz / 2 000 000 = 100 EXACTLY. 10 bytes x 10 bits / 2 Mbaud = 50 us/frame.
    parameter integer CLKS_PER_BIT = 100,    // 200 MHz / 2 Mbaud
    parameter integer EVENT_SRC    = 2,      // 0 btn, 1 ext, 2 DPS cal, 3 ring osc, 4 run-time select
    parameter integer TIE_CHANNELS = 0,      // 1 = drive both chains from A
    parameter integer HB_BIT       = 26,
    parameter integer VIS_BITS     = 24,
    // ---- autonomous DPS sweep ------------------------------------------------
    parameter integer AUTO_SWEEP       = 1,     // 0 = manual buttons only
    parameter integer SWEEP_STEPS      = 280,   // 280 x 17.857 ps = 5.000 ns
    parameter integer SAMPLES_PER_STEP = 256,
    // MMCM phase shift settles in a few VCO cycles; 2000 clk200 (10 us) is
    // deliberately generous. Do not trim this to save time you will not notice.
    parameter integer SETTLE_CYCLES    = 2000,
    parameter integer TAP_SRC          = 0,   // 1 = XORCY probe build
    parameter integer SYNC_TAP         = 30,  // 0 = old raw-event sync
    // ---- ring-oscillator hit source (EVENT_SRC = 3 only) --------------------
    parameter integer RO_STAGES        = 7,   // odd. Build 7 AND 11 to cross-check
    parameter integer RO_DIV_BITS      = 10,  // event period ~ 2^10 RO periods (EVENT_SRC 3 and 4)
    parameter integer DUAL_SNAP        = 1,   // step 4: dead-zone fix, 0 = old capture
    parameter integer ENCODER_ID       = 0    // reported in cfg: 0 = ones-counter, single edge
)(
    input  wire        clk100,        // F14
    input  wire        rst,           // J2  btn0
    input  wire        btn_a,         // J5  btn1  manual START (DPS builds: phase step)
    input  wire        btn_b,         // H2  btn2  manual STOP  (DPS builds: direction)
    input  wire        btn_clear,     // J1  btn3  manual re-arm
    input  wire        ev_a_ext,      // M14 servo0  external START
    input  wire        ev_b_ext,      // M16 servo1  external STOP
    input  wire        sw_autorearm,  // V2  sw0
    input  wire [1:0]  sw_src,        // U2 sw1 = bit0, U1 sw2 = bit1  (EVENT_SRC = 4 only)
    output wire [15:0] led,
    output wire        uart_txd       // U11
);

    localparam integer COARSE_BITS = 14;
    localparam integer FINE_BITS   = 9;     // encoder output width (0..352)
    localparam integer FRAME_FINE  = 11;    // frame field width
    localparam integer PHASE_BITS  = 12;

    localparam [1:0] SRC_EXT = 2'd0, SRC_RO7 = 2'd1, SRC_RO11 = 2'd2, SRC_DPS = 2'd3;
    localparam [1:0] FIXED_SRC = (EVENT_SRC == 2) ? SRC_DPS :
                                 (EVENT_SRC == 3) ? ((RO_STAGES == 11) ? SRC_RO11 : SRC_RO7) :
                                                    SRC_EXT;
    localparam integer CAL_EVENT_MODE = (EVENT_SRC == 2 || EVENT_SRC == 4) ? 1 : 0;
    localparam integer EVENT_MUX_MODE = (EVENT_SRC == 4) ? 1 : 0;
    localparam [2:0]   ENC3 = ENCODER_ID;
    localparam [0:0]   DS1  = (DUAL_SNAP != 0) ? 1'b1 : 1'b0;

    // -------------------------------------------------------------------------
    // Core-side wires
    // -------------------------------------------------------------------------
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
    wire                                            rst_sync_w;
    wire                                            rearm;

    // -------------------------------------------------------------------------
    // Source-select switches -> clk200 (2-FF). Only used when EVENT_SRC = 4;
    // otherwise the source is a constant and these flops are optimised away.
    // -------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] sw_s1 = 2'b00;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sw_s2 = 2'b00;
    always @(posedge clk200) begin
        sw_s1 <= sw_src;
        sw_s2 <= sw_s1;
    end

    wire [1:0] src_sel    = (EVENT_SRC == 4) ? sw_s2 : FIXED_SRC;
    wire       use_cal    = (EVENT_SRC == 4) && (src_sel == SRC_DPS);
    wire       dps_active = (EVENT_SRC == 2) || use_cal;
    wire [5:0] cfg_now    = {ENC3, DS1, src_sel};

    // -------------------------------------------------------------------------
    // Event source select
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
    end else if (EVENT_SRC == 2) begin : g_cal   // DPS calibration
        assign event_a_src = 1'b0;    // unused: core drives both chains from clk_cal
        assign event_b_src = 1'b0;
    end else if (EVENT_SRC == 3) begin : g_ro    // code density, one fixed ring
        wire ro_event;
        ring_osc_event #(.STAGES(RO_STAGES), .DIV_BITS(RO_DIV_BITS)) ro_inst (
            .enable (~rst), .event_out (ro_event));
        assign event_a_src = ro_event;
        assign event_b_src = ro_event;
    end else begin : g_multi                     // EVENT_SRC == 4: run-time select
        // Both rings are built, but only the selected one is enabled, so they
        // cannot pull on each other. DPS is selected inside the core (use_cal).
        wire ro7_ev, ro11_ev;
        ring_osc_event #(.STAGES(7),  .DIV_BITS(RO_DIV_BITS)) ro7_inst (
            .enable (~rst & (src_sel == SRC_RO7)),  .event_out (ro7_ev));
        ring_osc_event #(.STAGES(11), .DIV_BITS(RO_DIV_BITS)) ro11_inst (
            .enable (~rst & (src_sel == SRC_RO11)), .event_out (ro11_ev));
        assign event_a_src = (src_sel == SRC_RO7)  ? ro7_ev  :
                             (src_sel == SRC_RO11) ? ro11_ev : ev_a_ext;
        assign event_b_src = (src_sel == SRC_RO7)  ? ro7_ev  :
                             (src_sel == SRC_RO11) ? ro11_ev : ev_b_ext;
    end
    endgenerate

    // -------------------------------------------------------------------------
    // Core
    // -------------------------------------------------------------------------
    tdc_dual_top #(
        .NUM_CARRY4(88), .TDL_WIDTH(352), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(4), .FINE_LATENCY(12),
        .CAL_EVENT(CAL_EVENT_MODE), .EVENT_MUX(EVENT_MUX_MODE),
        .TAP_SRC(TAP_SRC), .SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP), .PHASE_BITS(PHASE_BITS)
    ) core (
        .clk100       (clk100),
        .rst          (rst),
        .event_a      (event_a_src),
        .event_b      (event_b_src),
        .use_cal      (use_cal),
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
        .rst_sync     (rst_sync_w),
        .ps_step_btn  (btn_a),   // DPS builds: btn_a = PHASE STEP
        .ps_dir_btn   (btn_b),   // DPS builds: btn_b held = decrement
        .ps_step_req  (sweep_step_req),
        .ps_dir_req   (sweep_dir_q),
        .ps_phase_idx (ps_phase_idx),
        .ps_busy      (ps_busy),
        .ps_error     (ps_error),
        .clk_cal      (clk_cal)
    );

    // Synchronised reset from the core: asserts at once, releases on clk200.
    wire rst200 = rst_sync_w;

    // -------------------------------------------------------------------------
    // Latch the record on meas_ready.
    //   ph_l  : the MMCM phase index that produced this sample.
    //   seq_l : free-running per-measurement counter (gaps = dropped frames).
    //   cfg_l : source + encoder that produced THIS sample.
    // -------------------------------------------------------------------------
    reg [COARSE_BITS-1:0] dc_l;
    reg [FINE_BITS-1:0]   fa_l, fb_l;
    reg                   va_l, vb_l;
    reg [PHASE_BITS-1:0]  ph_l;
    reg [7:0]             seq_l, seq_cnt;
    reg [5:0]             cfg_l;

    always @(posedge clk200) begin
        if (rst200) begin
            dc_l <= {COARSE_BITS{1'b0}};
            fa_l <= {FINE_BITS{1'b0}};  fb_l <= {FINE_BITS{1'b0}};
            va_l <= 1'b0;               vb_l <= 1'b0;
            ph_l <= {PHASE_BITS{1'b0}};
            seq_l <= 8'd0;              seq_cnt <= 8'd0;
            cfg_l <= 6'd0;
        end else if (meas_ready) begin
            dc_l    <= d_coarse;
            fa_l    <= fine_a;
            fb_l    <= fine_b;
            va_l    <= valid_a;
            vb_l    <= valid_b;
            ph_l    <= ps_phase_idx;
            seq_l   <= seq_cnt;
            seq_cnt <= seq_cnt + 1'b1;
            cfg_l   <= cfg_now;
        end
    end

    // -------------------------------------------------------------------------
    // UART framing -- 10 bytes, shifted out of a snapshot register.
    // The whole frame is snapshotted into `sr` at load time, so a measurement
    // completing mid-transmission cannot tear the frame already in flight.
    // -------------------------------------------------------------------------
    wire       uart_busy;
    reg        uart_send;
    reg [7:0]  uart_byte;
    reg        pending;
    reg        frame_done;
    reg [79:0] sr;
    reg [3:0]  nleft;

    // CRC-8, poly x^8+x^2+x+1 (0x07), init 0, MSB-first over bytes 0..8.
    function [7:0] crc8_72;
        input [71:0] d;
        integer i;
        reg [7:0] c;
        begin
            c = 8'h00;
            for (i = 71; i >= 0; i = i - 1)
                c = {c[6:0], 1'b0} ^ ((c[7] ^ d[i]) ? 8'h07 : 8'h00);
            crc8_72 = c;
        end
    endfunction

    wire [FRAME_FINE-1:0] fa_w = fa_l;          // zero-extended to 11 bits
    wire [FRAME_FINE-1:0] fb_w = fb_l;
    // 11 + 11 + 14 + 12 + 1 + 1 + 8 + 6 = 64 bits
    wire [63:0] payload = { fa_w, fb_w, dc_l, ph_l, va_l, vb_l, seq_l, cfg_l };
    wire [71:0] frame_w = { 8'hA7, payload };
    wire [79:0] frame_c = { frame_w, crc8_72(frame_w) };

    always @(posedge clk200) begin
        if (rst200) begin
            uart_send <= 1'b0; uart_byte <= 8'h00; pending <= 1'b0;
            frame_done <= 1'b0; sr <= 80'd0; nleft <= 4'd0;
        end else begin
            uart_send  <= 1'b0;
            frame_done <= 1'b0;

            if (meas_ready) pending <= 1'b1;

            if (nleft == 4'd0) begin
                if (pending && !uart_busy && !uart_send) begin
                    pending   <= 1'b0;
                    uart_byte <= frame_c[79:72];   // header out now
                    sr        <= {frame_c[71:0], 8'h00};
                    uart_send <= 1'b1;
                    nleft     <= 4'd9;             // 8 payload + CRC still to go
                end
            end else if (!uart_busy && !uart_send) begin
                uart_byte <= sr[79:72];
                sr        <= {sr[71:0], 8'h00};
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
    // AUTONOMOUS DPS SWEEP  (triangle 0 -> SWEEP_STEPS-1 -> 0 ...)
    //   COLLECT : SAMPLES_PER_STEP measurements at the current phase
    //   STEP    : one-cycle step request into dps_phase_ctrl
    //   SETTLE  : wait for ps_busy low, then a fixed settle window
    // Re-arm is BLOCKED outside COLLECT so no sample is taken mid-shift.
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

    // The sweep runs only while DPS is the ACTIVE source: always in an
    // EVENT_SRC = 2 build, and only with sw2 sw1 = 11 in an EVENT_SRC = 4 build.
    wire sweep_on   = (AUTO_SWEEP != 0) && dps_active && sw_autorearm;
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
                    // Present the direction that is current NOW (see history:
                    // handing sweep_dir directly made the turnaround step
                    // arrive with the direction already flipped).
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
    // A frame_done arriving while sweep_hold is high is remembered and fired
    // when the hold lifts (deadlock fix).
    // -------------------------------------------------------------------------
    reg [2:0] rearm_cnt;
    reg       rearm_pend;
    wire      rearm_req = sw_autorearm && (frame_done || rearm_pend);
    always @(posedge clk200) begin
        if (rst200) begin
            rearm_cnt  <= 3'd0;
            rearm_pend <= 1'b0;
        end else if (rearm_req && !sweep_hold) begin
            rearm_cnt  <= 3'd4;
            rearm_pend <= 1'b0;
        end else begin
            if (rearm_req)               rearm_pend <= 1'b1;   // held off: remember
            if (!sw_autorearm)           rearm_pend <= 1'b0;
            if (rearm_cnt != 3'd0)       rearm_cnt  <= rearm_cnt - 1'b1;
        end
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
    assign led[6]    = tmo_led;          // a channel timed out
    // led[7] : sweep integrity. Solid = a phase step was lost or a PSDONE was
    // missed -- do not build a LUT from this run.
    assign led[7]    = sw_mismatch | ps_error;
    assign led[15:8] = meas_cnt;         // measurement counter

endmodule