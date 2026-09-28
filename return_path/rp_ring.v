`timescale 1ns/1ps
// =============================================================================
// rp_ring -- RETURN-PATH PROBE (folding step 1). One gated ring oscillator
// built exactly like a folding lap:
//
//   node B = CO[0] of CARRY4 #0.  CYINIT = 1, DI0 = 0, so CO[0] = S0.
//            S0 comes from u_ret (LUT2, forced into B's own slice because a
//            CARRY4 S input can only be driven by its slice's LUT O6).
//   B -> D   K carry stages up the chain (S = 1, DI = 0), D = co[K].
//   D -> B   general routing -> u_ret.I0 -> O6 -> S0.    <- the return path
//
//   u_ret: S0 = en & ~D.   One inversion in the loop -> it oscillates while
//   en = 1 and parks at 0 when en = 0.
//
//   Ring period T = 2 * T_lap,   T_lap = t(B->D, K taps) + t_return.
//   T_lap is the folding lap time for a fold of K taps; 5000 / T_lap is the
//   folding factor that K would give.
//
// Observation: co[K/2] -> u_obs (LUT1 buffer) -> clocks a DIV_BITS counter.
// Tapping mid-chain keeps the extra fanout off node D, so D is loaded only by
// the return LUT (plus its next MUXCY), as it will be in the real fold.
//
// SIMULATION: xsim's CARRY4 has zero delay, so the real loop would hang the
// simulator. Under XILINX_SIMULATOR a behavioural oscillator with lap time
// SIM_TLAP_NS stands in. The carry loop itself is only tested on hardware.
// =============================================================================
(* KEEP_HIERARCHY = "TRUE" *)
module rp_ring #(
    parameter integer K           = 64,    // taps B -> D, multiple of 4, >= 8
    parameter integer DIV_BITS    = 5,     // counter width; MSB = f / 2^DIV_BITS
    parameter real    SIM_TLAP_NS = 1.6    // behavioural lap time (sim only)
)(
    input  wire en,
    output wire div_msb
);
    localparam integer NC4 = K / 4 + 1;    // D = co[K] = CO[0] of CARRY4 #(K/4)

    wire ring_obs;

`ifdef XILINX_SIMULATOR
    reg r = 1'b0;
    always begin
        if (!en) begin r = 1'b0; wait (en); end
        #(SIM_TLAP_NS) r = en ? ~r : 1'b0;
    end
    assign ring_obs = r;
`else
    (* KEEP = "TRUE", ALLOW_COMBINATORIAL_LOOPS = "TRUE" *) wire [4*NC4-1:0] co;
    (* KEEP = "TRUE", ALLOW_COMBINATORIAL_LOOPS = "TRUE" *) wire             s0;
    wire [4*NC4-1:0] unused_o;
    wire [4*NC4:0]   cc = {co, 1'b0};      // cc[4n] = carry into CARRY4 #n

    // Return LUT: O = INIT[{I1,I0}] = 1 only for en=1, D=0.
    (* DONT_TOUCH = "TRUE" *)
    LUT2 #(.INIT(4'b0100)) u_ret (.O(s0), .I0(co[K]), .I1(en));

    genvar n;
    generate
        for (n = 0; n < NC4; n = n + 1) begin : g_c4
            (* DONT_TOUCH = "TRUE" *)
            CARRY4 u_c4 (
                .CO     (co[4*n+3 : 4*n]),
                .O      (unused_o[4*n+3 : 4*n]),
                .CI     (cc[4*n]),
                .CYINIT ((n == 0) ? 1'b1 : 1'b0),
                .DI     (4'b0000),
                .S      ((n == 0) ? {3'b111, s0} : 4'b1111)
            );
        end
    endgenerate

    (* DONT_TOUCH = "TRUE" *)
    LUT1 #(.INIT(2'b10)) u_obs (.O(ring_obs), .I0(co[K/2]));
`endif

    (* DONT_TOUCH = "TRUE" *) reg [DIV_BITS-1:0] div = {DIV_BITS{1'b0}};
    always @(posedge ring_obs) div <= div + 1'b1;
    assign div_msb = div[DIV_BITS-1];

    generate if ((K % 4) != 0 || K < 8) begin : g_bad
        K_MUST_BE_MULTIPLE_OF_4_AND_AT_LEAST_8 u_err ();
    end endgenerate
endmodule
