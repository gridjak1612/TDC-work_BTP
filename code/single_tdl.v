// =============================================================================
// Module Name:  single_tdl
// Description:  Parameterized Single Tap Delay Line (TDL) implemented using
//               cascaded Xilinx CARRY4 primitives.
//
// Architecture:
//   • STOP-launch interpolation.
//   • Asynchronous STOP event launches the carry propagation.
//   • 64 CARRY4 blocks → 256 delay taps.
//   • CO outputs form the raw thermometer code.
//
// Synthesis Notes:
//   • KEEP_HIERARCHY prevents hierarchy flattening.
//   • KEEP preserves the observation taps.
//   • DONT_TOUCH prevents optimization of CARRY4 primitives.
// =============================================================================

`timescale 1ns/1ps

(* KEEP_HIERARCHY = "TRUE" *)
module single_tdl #(
    // TAP_SRC 0 = CO outputs (normal). 1 = O outputs (XORCY probe build).
    parameter TAP_SRC = 0,
    parameter NUM_CARRY4 = 64
)(
    // -------------------------------------------------------------------------
    // Asynchronous STOP event.
    // This signal intentionally bypasses any synchronizer because it launches
    // the carry propagation whose delay is being measured.
    // -------------------------------------------------------------------------
    input  wire trigger,

    // Raw thermometer code from the carry chain.
    output wire [(4*NUM_CARRY4)-1:0] taps
);

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------

    // Preserve the trigger net so Vivado does not optimize or duplicate it.
    (* KEEP = "TRUE" *)
    wire launch_trigger;

    assign launch_trigger = trigger;

    // Carry cascade between CARRY4 blocks.
    wire [NUM_CARRY4-1:0] carry_chain;

    // Raw carry outputs (thermometer code).
    (* KEEP = "TRUE" *)
    wire [(4*NUM_CARRY4)-1:0] tdl_taps_raw;

    // SUM outputs are unused.
    wire [(4*NUM_CARRY4)-1:0] unused_sum;

    // Export thermometer code.
    // -------------------------------------------------------------------------
    // TAP_SRC = 1 : XORCY PROBE BUILD. Diagnostic only, do not ship.
    //
    // With S = 4'b1111 the CARRY4 sum output is O[j] = 1 XOR ci_j = ~ci_j, so
    // ~O[k] is the carry-chain node ONE POSITION EARLIER than CO[k], seen
    // through one XORCY:
    //
    //     T( ~O[k] )  =  T( CO[k-1] )  +  t_XORCY
    //
    // t_XORCY is the ONLY thing that decides whether tapping O as well as CO
    // is worth anything. If it lands mid-bin, 704 interleaved taps roughly
    // halve the quantisation error. If it lands near a whole tap, the O taps
    // sit on top of existing CO taps and the entire 352->704 rebuild buys
    // nothing. Nobody has measured it, so measure it before committing.
    //
    // Build this, run the SAME sweep and analyser, then difference the LUTs:
    //     t_XORCY = lut_probe[k] - lut_normal[k-1]
    // Everything else -- width, encoder, frame, host, placement -- is
    // unchanged, so the two runs are directly comparable.
    // -------------------------------------------------------------------------
    generate
    if (TAP_SRC == 0) begin : g_co
        assign taps = tdl_taps_raw;
    end
    else if (TAP_SRC == 1) begin : g_o_only
        // Diagnostic only. Measures the O chain in isolation, which CANNOT
        // determine t_XORCY: comparing it against a separate CO build leaves an
        // unknown per-bitstream offset inseparable from t_XORCY itself.
        assign taps = ~unused_sum;
    end
    else begin : g_interleave
        // ---------------------------------------------------------------------
        // TAP_SRC = 2 : INTERLEAVED PROBE -- the build that answers the question.
        //
        // With S = 4'b1111 the sum output is O[k] = 1 XOR ci_k = ~ci_k, so
        //     T( ~O[k+1] ) = T( CO[k] ) + t_XORCY
        // The O tap sits INSIDE the bin between CO[k] and CO[k+1], provided
        // t_XORCY is smaller than that bin. Interleaving puts both tap families
        // into ONE measurement, so their relative spacing appears directly as
        // the bin-width distribution -- no cross-bitstream offset to cancel,
        // which is precisely what an O-only build cannot escape.
        //
        //     taps[2k]   = CO[k]        k = 0 .. 175
        //     taps[2k+1] = ~O[k+1]      lands t_XORCY later
        //
        // Still 352 taps, so TDL_WIDTH, the encoder, the frame, the host and
        // tdl_loc.xdc are ALL unchanged. Only CARRY4 blocks 0..44 are read; the
        // rest stay placed and constrained but unused.
        //
        // COST: the chain spans ~176 carry stages ~= 3.0 ns, not 5.0 ns, so any
        // phase needing more than 3 ns of propagation rails at 352 -- expect
        // ~40 % of phases railed. That is EXPECTED. The remaining ~170 phase
        // steps still give full statistics on all 352 interleaved taps, which
        // is all this measurement needs.
        //
        // READING THE RESULT (analyze_sweep.py, usable region):
        //   even/odd ratio near 1, mean bin ~8.5 ps -> t_XORCY splits the wide
        //       bins. Interleaving is worth the 704-tap rebuild (~2x better).
        //   even/odd still 5-6x, half the bins zero-width -> the O taps land on
        //       top of the CO taps. Interleaving buys NOTHING. Drop it for good.
        // ---------------------------------------------------------------------
        genvar t;
        for (t = 0; t < (4*NUM_CARRY4)/2; t = t + 1) begin : tap_mux
            assign taps[2*t]     =  tdl_taps_raw[t];
            assign taps[2*t + 1] = ~unused_sum[t + 1];
        end
    end
    endgenerate

    // -------------------------------------------------------------------------
    // CARRY4 Chain
    // -------------------------------------------------------------------------

    genvar i;

    generate

        for(i=0;i<NUM_CARRY4;i=i+1) begin : carry_loop

            //------------------------------------------------------------------
            // First CARRY4
            //------------------------------------------------------------------
            if(i==0) begin

                (* DONT_TOUCH = "TRUE" *)
                CARRY4 carry4_inst (

                    .CO(tdl_taps_raw[3:0]),
                    .O (unused_sum[3:0]),

                    .CI(1'b0),
                    .CYINIT(launch_trigger),

                    .DI(4'b0000),
                    .S (4'b1111)

                );

                assign carry_chain[0] = tdl_taps_raw[3];

            end

            //------------------------------------------------------------------
            // Remaining CARRY4 blocks
            //------------------------------------------------------------------
            else begin

                (* DONT_TOUCH = "TRUE" *)
                CARRY4 carry4_inst (

                    .CO(tdl_taps_raw[4*i+3 : 4*i]),
                    .O (unused_sum[4*i+3 : 4*i]),

                    .CI(carry_chain[i-1]),
                    .CYINIT(1'b0),

                    .DI(4'b0000),
                    .S (4'b1111)

                );

                assign carry_chain[i] = tdl_taps_raw[4*i+3];

            end

        end

    endgenerate

endmodule