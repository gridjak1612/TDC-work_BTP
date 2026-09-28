`timescale 1ns/1ps
// =============================================================================
// fold_tdl -- FOLDING tap delay line (folding step 2, single edge).
//
// Same 88-CARRY4 chain as single_tdl, plus ONE return path:
//
//   launch   taps 0 .. 4*B_C4-1     plain chain, monotonic step (hit flag, SYNC_TAP)
//   B        tap 4*B_C4             CO[0] of CARRY4 #B_C4; its S input comes from
//                                   u_ret instead of being tied to 1
//   D        tap 4*B_C4 + K         return point
//   fold     B .. B+FOLD_W-1        every tap sampled (FOLD_W set by the channel)
//   counting beyond the fold        sparse taps, lap counting
//
//   u_ret:  S0 = loop_en ? ~D : 1
//
//   Why the inversion (measured geometry: K=64, t_return 957 ps, lap 2062 ps):
//     S = D  would give S = 0 before the hit (D = 0), so B would output DI = 0
//            and BLOCK the incoming edge. Dead on arrival.
//     S = ~D gives S = 1 before the hit: B passes CI, the front enters the fold.
//            Front reaches D -> S = 0 -> B outputs DI = 0 while CI = 1: a FALLING
//            edge is launched. That edge reaches D -> S = 1 -> B outputs CI = 1:
//            a RISING edge. Polarity alternates every lap; the decoder knows.
//   loop_en = 0 opens the loop (S = 1, plain chain): the fold flushes to the
//   hit level and stays quiet. The channel closes the loop only while it is
//   armed and the event line has been seen low, and opens it at capture.
//
// SIMULATION: xsim's CARRY4 has zero delay; a zero-delay loop hangs the
// simulator. Under XILINX_SIMULATOR the chain is a plain zero-delay step with
// NO return path, exactly like single_tdl behaves there. Control and framing
// testbenches still run; the fold itself is only tested on hardware and, for
// the decoder, with synthetic snapshots (tb_fold_decode).
// =============================================================================
(* KEEP_HIERARCHY = "TRUE" *)
module fold_tdl #(
    parameter integer NUM_CARRY4 = 88,
    parameter integer B_C4       = 8,     // B = CO[0] of this CARRY4 = tap 32
    parameter integer K          = 64     // taps B -> D
)(
    input  wire                     trigger,   // async hit, launches the step
    input  wire                     loop_en,   // 1 = fold closed
    output wire [(4*NUM_CARRY4)-1:0] taps
);
    localparam integer TB = 4 * B_C4;         // tap index of B
    localparam integer TD = TB + K;           // tap index of D

    (* KEEP = "TRUE" *) wire launch_trigger = trigger;

`ifdef XILINX_SIMULATOR
    // Behavioural: plain step, no loop (see header).
    assign taps = {(4*NUM_CARRY4){launch_trigger}};
`else
    (* KEEP = "TRUE", ALLOW_COMBINATORIAL_LOOPS = "TRUE" *) wire [(4*NUM_CARRY4)-1:0] co;
    (* KEEP = "TRUE", ALLOW_COMBINATORIAL_LOOPS = "TRUE" *) wire                      s0;
    wire [(4*NUM_CARRY4)-1:0] unused_o;
    wire [4*NUM_CARRY4:0]     cc = {co, 1'b0};       // cc[4n] = carry into CARRY4 #n

    // O = INIT[{I1,I0}] = {loop_en, D}: 00->1 01->1 10->1 11->0
    (* DONT_TOUCH = "TRUE" *)
    LUT2 #(.INIT(4'b0111)) u_ret (.O(s0), .I0(co[TD]), .I1(loop_en));

    genvar n;
    generate
        for (n = 0; n < NUM_CARRY4; n = n + 1) begin : g_c4
            (* DONT_TOUCH = "TRUE" *)
            CARRY4 u_c4 (
                .CO     (co[4*n+3 : 4*n]),
                .O      (unused_o[4*n+3 : 4*n]),
                .CI     ((n == 0) ? 1'b0 : cc[4*n]),
                .CYINIT ((n == 0) ? launch_trigger : 1'b0),
                .DI     (4'b0000),
                .S      ((n == B_C4) ? {3'b111, s0} : 4'b1111)
            );
        end
    endgenerate
    assign taps = co;
`endif

    generate if ((K % 4) != 0 || B_C4 < 1 || (TD + 4) > 4*NUM_CARRY4) begin : g_bad
        FOLD_TDL_BAD_GEOMETRY u_err ();
    end endgenerate
endmodule
