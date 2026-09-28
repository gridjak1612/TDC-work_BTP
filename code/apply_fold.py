#!/usr/bin/env python3
"""apply_fold.py -- folding step 2 plumbing in the SHARED files (run from code/).

Edits, each must match exactly once (aborts before writing otherwise):
  tdc_channel.v     : + output raw_out (lowest RAW_W sampled taps; new port, no
                      behaviour change) so both channel flavours have the same ports
  tdc_dual_top.v    : + RAW_W param, + raw_a output; channel instances chosen by
                      `ifdef FOLD (tdc_channel_fold) / `else (tdc_channel). No
                      generate block, so hierarchical names (core/chan_a/...) are
                      unchanged and tdl_loc*.xdc keep working.
  tdc_dual_board.v  : FINE_BITS 10 and ENCODER_ID 1 under `ifdef FOLD; + DUMP
                      parameter: DUMP=1 sends raw channel-A snapshots (frame v5,
                      header 0xC3) on the UART instead of the normal frame.

Select the fold build with the Verilog define, NOT a parameter:
    set_property verilog_define {FOLD=1} [get_filesets sources_1]
    set_property verilog_define {FOLD=1} [get_filesets sim_1]
(clear with:  set_property verilog_define {} [get_filesets sources_1])
"""
import sys

def patch(path, edits):
    s = open(path, encoding='utf-8', newline='').read()
    for i, (old, new) in enumerate(edits, 1):
        n = s.count(old)
        if n != 1:
            already = s.count(new) >= 1
            sys.exit(f"{path}: edit {i} matched {n} times"
                     + (" (looks ALREADY APPLIED)" if already else "") + " -- aborting, nothing written")
        s = s.replace(old, new)
    open(path, 'w', encoding='utf-8', newline='').write(s)
    print(f"{path}: {len(edits)} edits applied")

# ============================ tdc_channel.v ==================================
patch('tdc_channel.v', [
("""    parameter integer DUAL_SNAP    = 1
)(""",
"""    parameter integer DUAL_SNAP    = 1,
    // Width of raw_out (lowest sampled taps). Same port on tdc_channel_fold.
    parameter integer RAW_W        = 159
)("""),
("""    output wire                    done           // captured, locked out
);""",
"""    output wire                    done,          // captured, locked out
    output wire [RAW_W-1:0]        raw_out        // lowest RAW_W captured taps (DUMP builds)
);"""),
("""    wire [TDL_WIDTH-1:0]   sampled_taps;
""",
"""    wire [TDL_WIDTH-1:0]   sampled_taps;
    assign raw_out = sampled_taps[RAW_W-1:0];
"""),
])

# ============================ tdc_dual_top.v =================================
patch('tdc_dual_top.v', [
("""    parameter integer DUAL_SNAP      = 1     // step 4 dead-zone fix
)(""",
"""    parameter integer DUAL_SNAP      = 1,    // step 4 dead-zone fix
    parameter integer RAW_W          = 159   // raw snapshot width (FOLD: 32+120+7)
)("""),
("""    output wire                   rst_sync       // synchronised reset, clk200 domain
);""",
"""    output wire                   rst_sync,      // synchronised reset, clk200 domain
    output wire [RAW_W-1:0]       raw_a          // channel A captured snapshot (DUMP builds)
);"""),
("""    tdc_channel #(
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
    );""",
"""`ifdef FOLD
    // FOLDING channel (folding step 2). Same instance name so the placement
    // constraints keep their paths; module chosen by the FOLD define.
    tdc_channel_fold #(
        .NUM_CARRY4(NUM_CARRY4), .TDL_WIDTH(TDL_WIDTH), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG), .SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP),
        .FINE_LATENCY(FINE_LATENCY)
    ) chan_a (
`else
    tdc_channel #(
        .NUM_CARRY4(NUM_CARRY4), .TDL_WIDTH(TDL_WIDTH), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG), .TAP_SRC(TAP_SRC), .SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP),
        .FINE_LATENCY(FINE_LATENCY), .RAW_W(RAW_W)
    ) chan_a (
`endif
        .clk          (clk200_i),
        .rst          (rst_i),
        .clear_status (clear_status),
        .coarse_count (coarse_count),
        .coarse_out   (coarse_a_w),
        .fine_out     (fine_a_w),
        .valid_out    (valid_a_w),
        .ready        (ready_a_w),
        .done         (done_a),
        .event_in     (event_a_i),
        .raw_out      (raw_a)
    );"""),
("""    tdc_channel #(
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
    );""",
"""`ifdef FOLD
    tdc_channel_fold #(
        .NUM_CARRY4(NUM_CARRY4), .TDL_WIDTH(TDL_WIDTH), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG), .SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP),
        .FINE_LATENCY(FINE_LATENCY)
    ) chan_b (
`else
    tdc_channel #(
        .NUM_CARRY4(NUM_CARRY4), .TDL_WIDTH(TDL_WIDTH), .FINE_BITS(FINE_BITS),
        .COARSE_BITS(COARSE_BITS), .CAPTURE_LAG(CAPTURE_LAG), .TAP_SRC(TAP_SRC), .SYNC_TAP(SYNC_TAP), .DUAL_SNAP(DUAL_SNAP),
        .FINE_LATENCY(FINE_LATENCY), .RAW_W(RAW_W)
    ) chan_b (
`endif
        .clk          (clk200_i),
        .rst          (rst_i),
        .clear_status (clear_status),
        .coarse_count (coarse_count),
        .coarse_out   (coarse_b_w),
        .fine_out     (fine_b_w),
        .valid_out    (valid_b_w),
        .ready        (ready_b_w),
        .event_in     (event_b_i),
        .done         (done_b),
        .raw_out      ()
    );"""),
])

# ============================ tdc_dual_board.v ===============================
patch('tdc_dual_board.v', [
("""    parameter integer ENCODER_ID       = 0    // reported in cfg: 0 = ones-counter, single edge
)(""",
"""`ifdef FOLD
    parameter integer ENCODER_ID       = 1,   // 1 = folding, single edge (fold_decode)
`else
    parameter integer ENCODER_ID       = 0,   // reported in cfg: 0 = ones-counter, single edge
`endif
    parameter integer DUMP             = 0    // 1 = send raw channel-A snapshots (frame v5, 0xC3)
)("""),
("""    localparam integer FINE_BITS   = 9;     // encoder output width (0..352)
""",
"""`ifdef FOLD
    localparam integer FINE_BITS   = 10;    // fold code 0..~512 (32 + n*120 + pos)
`else
    localparam integer FINE_BITS   = 9;     // encoder output width (0..352)
`endif
    localparam integer RAW_W       = 159;   // raw snapshot width carried to the DUMP path
    wire [RAW_W-1:0] raw_a;
"""),
("""        .rst_sync     (rst_sync_w),
""",
"""        .rst_sync     (rst_sync_w),
        .raw_a        (raw_a),
"""),
("""    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst (
        .clk (clk200), .rst (rst200), .send (uart_send), .data (uart_byte),
        .tx  (uart_txd), .busy (uart_busy)
    );""",
"""    wire uart_txd_norm, dump_txd;
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst (
        .clk (clk200), .rst (rst200), .send (uart_send), .data (uart_byte),
        .tx  (uart_txd_norm), .busy (uart_busy)
    );
    // DUMP builds: the pin carries raw channel-A snapshots (frame v5) instead of
    // the normal frame. The normal path still runs internally so re-arm
    // (frame_done) keeps its timing.
    generate if (DUMP != 0) begin : g_dump
        dump_tx #(.CLKS_PER_BIT(CLKS_PER_BIT), .RAW_W(RAW_W), .FINE_BITS(FINE_BITS)) u_dump (
            .clk (clk200), .rst (rst200), .meas_ready (meas_ready),
            .raw (raw_a), .fine (fine_a), .valid (valid_a), .txd (dump_txd));
    end else begin : g_nodump
        assign dump_txd = 1'b1;
    end endgenerate
    assign uart_txd = (DUMP != 0) ? dump_txd : uart_txd_norm;"""),
])
