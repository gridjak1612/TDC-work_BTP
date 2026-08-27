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
    assign taps = tdl_taps_raw;

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