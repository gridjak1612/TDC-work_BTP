// =============================================================================
// Module Name:  tdc_dual_board
// Description:  Board top for the TWO-CHANNEL interval TDC.
//
//   UART FRAME - 6 bytes per measurement, 8N1 @ 115200
//     byte0 : 0xAA                                     sync header
//     byte1 : fine_a[7:0]                              START fine, low  8 bits
//     byte2 : fine_b[7:0]                              STOP  fine, low  8 bits
//     byte3 : d_coarse[7:0]                            interval coarse, low
//     byte4 : {fine_b[8], fine_a[8], d_coarse[13:8]}   fine MSBs + coarse high
//     byte5 : {6'b000000, valid_b, valid_a}            both valid flags
//
//   Six bytes, not five: `fine` is now 9 bits (0..352), so the two MSBs need
//   somewhere to live. 9+9+14+2 = 34 bits -> 5 payload bytes + 1 header.
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
    parameter integer CLKS_PER_BIT = 1736,   // 200 MHz / 115200
    parameter integer EVENT_SRC    = 0,      // 0 = buttons, 1 = external pins
    parameter integer TIE_CHANNELS = 0,      // 1 = drive both chains from A
    parameter integer HB_BIT       = 26,
    parameter integer VIS_BITS     = 24
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
        end else begin : g_ext
            assign event_a_src = ev_a_ext;
            assign event_b_src = (TIE_CHANNELS != 0) ? ev_a_ext : ev_b_ext;
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Core
    // -------------------------------------------------------------------------
    wire [COARSE_BITS-1:0] d_coarse;
    wire [FINE_BITS-1:0]   fine_a, fine_b;
    wire                   valid_a, valid_b, timeout, meas_ready;
    wire                   done_a, done_b, clk200, mmcm_locked;

    wire rearm;

    tdc_dual_top #(
        .NUM_CARRY4(88), .TDL_WIDTH(352), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(4), .FINE_LATENCY(12)
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
        .mmcm_locked  (mmcm_locked)
    );

    wire rst200 = rst | ~mmcm_locked;

    // -------------------------------------------------------------------------
    // Latch the record on meas_ready
    // -------------------------------------------------------------------------
    reg [COARSE_BITS-1:0] dc_l;
    reg [FINE_BITS-1:0]   fa_l, fb_l;
    reg                   va_l, vb_l;

    always @(posedge clk200) begin
        if (meas_ready) begin
            dc_l <= d_coarse;
            fa_l <= fine_a;
            fb_l <= fine_b;
            va_l <= valid_a;
            vb_l <= valid_b;
        end
    end

    // -------------------------------------------------------------------------
    // UART framing FSM: 0xAA, fine_a, fine_b, d_coarse[7:0], {vb,va,d[13:8]}
    // -------------------------------------------------------------------------
    wire       uart_busy;
    reg        uart_send;
    reg [7:0]  uart_byte;
    reg [2:0]  fstate;
    reg        pending;
    reg        frame_done;      // pulses when the last byte has been handed off

    localparam F_IDLE=3'd0, F_B0=3'd1, F_B1=3'd2, F_B2=3'd3,
               F_B3=3'd4,   F_B4=3'd5, F_B5=3'd6;

    always @(posedge clk200) begin
        if (rst200) begin
            fstate <= F_IDLE; uart_send <= 1'b0; uart_byte <= 8'h00;
            pending <= 1'b0;  frame_done <= 1'b0;
        end else begin
            uart_send  <= 1'b0;
            frame_done <= 1'b0;

            if (meas_ready) pending <= 1'b1;

            case (fstate)
                F_IDLE: if (pending && !uart_busy) begin
                            pending   <= 1'b0;
                            uart_byte <= 8'hAA;
                            uart_send <= 1'b1;
                            fstate    <= F_B0;
                        end
                F_B0: if (!uart_busy && !uart_send) begin
                            uart_byte <= fa_l[7:0];            uart_send <= 1'b1; fstate <= F_B1; end
                F_B1: if (!uart_busy && !uart_send) begin
                            uart_byte <= fb_l[7:0];            uart_send <= 1'b1; fstate <= F_B2; end
                F_B2: if (!uart_busy && !uart_send) begin
                            uart_byte <= dc_l[7:0];            uart_send <= 1'b1; fstate <= F_B3; end
                F_B3: if (!uart_busy && !uart_send) begin
                            uart_byte <= {fb_l[8], fa_l[8], dc_l[13:8]};
                                                               uart_send <= 1'b1; fstate <= F_B4; end
                F_B4: if (!uart_busy && !uart_send) begin
                            uart_byte <= {6'b000000, vb_l, va_l};
                                                               uart_send <= 1'b1; fstate <= F_B5; end
                F_B5: if (!uart_busy && !uart_send) begin
                            frame_done <= 1'b1;                                   fstate <= F_IDLE; end
                default: fstate <= F_IDLE;
            endcase
        end
    end

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst (
        .clk (clk200), .rst (rst200), .send (uart_send), .data (uart_byte),
        .tx  (uart_txd), .busy (uart_busy)
    );

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
        else if (sw_autorearm && frame_done) rearm_cnt <= 3'd4;
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
    assign led[7]    = 1'b0;
    assign led[15:8] = meas_cnt;         // measurement counter

endmodule