// =============================================================================
// Module Name:  tdc_frontend
// Description:  Input conditioning for the event signal, applied BEFORE the
//               signal fans out to (a) the carry chain and (b) the capture
//               controller's CDC synchroniser.
//
//               Currently a pure pass-through. It exists as the single, correct
//               insertion point for any future conditioning, so that such logic
//               never has to be bolted on in two places.
//
// WHY IT SITS HERE (before the fanout, not after)
// -----------------------------------------------
//   event_in --> [tdc_frontend] --> event_cond --+--> single_tdl  (CYINIT)
//                                                |
//                                                +--> capture_controller (CDC)
//
//   Because the conditioning happens BEFORE the fanout, any delay it adds is
//   common to both paths. It therefore does NOT change the launch skew
//   (the difference between when the chain sees the event and when the
//   synchroniser sees it), which is the thing that costs us measurement range.
//   It only adds a constant offset to the absolute timestamp -- and a constant
//   offset cancels completely when you SUBTRACT two timestamps to get an
//   interval, or when you calibrate against a known reference.
//
// =============================================================================
// *** THE RULE THAT MATTERS ***
//
//   THIS MODULE MUST REMAIN PURELY COMBINATIONAL AND ASYNCHRONOUS.
//   IT MUST NEVER CONTAIN A FLIP-FLOP, A CLOCK, OR A SYNCHRONISER.
//
//   The instant you clock this signal, you quantise the event edge to the
//   200 MHz clock -- and the sub-nanosecond phase information IS the entire
//   measurement. Every fine value would collapse to a constant. The TDC would
//   still run, still produce timestamps, still pass every functional test, and
//   be silently worthless.
//
//   This is a real trap: "input conditioning" naturally suggests "debouncing",
//   and debouncing naturally suggests a clocked filter. A clocked debouncer
//   here destroys the instrument. If you need to reject bounce, do it in the
//   capture controller's arm/lockout logic (which already does exactly that),
//   or on the host by discarding bad samples -- NEVER in the launch path.
//
// SAFE things to put here (all asynchronous):
//   - polarity inversion, gating with an enable, a comparator interface,
//     an analogue-domain discriminator, glitch rejection built from
//     asynchronous delay elements
//
// UNSAFE (destroys the measurement):
//   - any always @(posedge clk), any register, any 2-flop synchroniser,
//     any clocked counter or debouncer
// =============================================================================

`timescale 1ns/1ps

module tdc_frontend (
    input  wire event_in,     // raw asynchronous event straight from the pin
    output wire event_out     // conditioned event -> carry chain AND capture ctrl
);

    // -------------------------------------------------------------------------
    // Version 1.0: direct pass-through.
    //
    // Synthesis collapses this to a wire, so it currently costs zero logic and
    // zero delay. Insert asynchronous conditioning here when needed.
    // -------------------------------------------------------------------------
    assign event_out = event_in;

endmodule