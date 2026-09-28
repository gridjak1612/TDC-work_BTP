// =============================================================================
// ring_osc_event -- on-chip asynchronous hit source for code-density calibration
//
//   RO (STAGES inverting LUTs, one NAND with enable) --> u_tap buffer
//   --> DIV_BITS free-running counter clocked by the RO --> event = MSB
//
// WHY DIVIDE: the TDC takes the FIRST event edge after each clk200-synchronous
// re-arm. The hit phase is uniform only if the event period is >> 5 ns: the
// selection ripple is ~5 ns / T_event. DIV_BITS=10 on a ~150 MHz RO gives
// T_event ~7 us (ripple < 0.1 %).
//
// RESIDUAL RISK: T_event near a low-order rational multiple of 5 ns combined
// with very low RO jitter. Detected from data by code_density.py (chunk
// consistency); cross-checked by building two RO lengths (STAGES 7 vs 11).
//
// xsim defines XILINX_SIMULATOR -> behavioural oscillator (a zero-delay LUT
// loop would hang the simulator). Synthesis builds the real LUT loop.
// Needs ro.xdc. Only instantiated when EVENT_SRC == 3.
// =============================================================================
`timescale 1ns/1ps

module ring_osc_event #(
    parameter integer STAGES   = 7,        // total inverting stages, MUST be odd
    parameter integer DIV_BITS = 10,
    parameter real    SIM_T_RO_NS = 7.1234 // behavioural RO period (sim only)
)(
    input  wire enable,
    output wire event_out
);
    wire ro_clk;

`ifdef XILINX_SIMULATOR
    reg ro_sim = 1'b0;
    always #(SIM_T_RO_NS/2.0) ro_sim = enable ? ~ro_sim : 1'b0;
    assign ro_clk = ro_sim;
`else
    (* ALLOW_COMBINATORIAL_LOOPS = "TRUE", DONT_TOUCH = "TRUE" *) wire [STAGES-1:0] n;

    (* DONT_TOUCH = "TRUE" *)
    LUT2 #(.INIT(4'b0111)) u_nand (.O(n[0]), .I0(n[STAGES-1]), .I1(enable));

    genvar s;
    generate for (s = 1; s < STAGES; s = s + 1) begin : g_inv
        (* DONT_TOUCH = "TRUE" *)
        LUT1 #(.INIT(2'b01)) u_inv (.O(n[s]), .I0(n[s-1]));
    end endgenerate

    (* DONT_TOUCH = "TRUE" *)
    LUT1 #(.INIT(2'b10)) u_tap (.O(ro_clk), .I0(n[STAGES-1]));
`endif

    (* DONT_TOUCH = "TRUE" *) reg [DIV_BITS-1:0] div = {DIV_BITS{1'b0}};
    always @(posedge ro_clk) div <= div + 1'b1;

    assign event_out = div[DIV_BITS-1];

    generate if (STAGES % 2 == 0) begin : g_bad
        STAGES_MUST_BE_ODD u_err ();
    end endgenerate
endmodule