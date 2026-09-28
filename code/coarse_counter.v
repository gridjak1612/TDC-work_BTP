// =============================================================================
// Module Name:  coarse_counter
// Description:  A free-running, parameterized binary counter used to track
//               coarse time steps (in units of clock periods) for a Time-to-
//               Digital Converter (TDC).
//
// Parameters:   WIDTH - The bit width of the counter, which determines the
//                       maximum time interval before rollover.
//                       Default: 14 bits → 0 to 16383 → 81.92 us at 200 MHz.
//
// Inputs:       clk   - System clock (200 MHz, 5 ns period)
//               rst   - Active-high synchronous reset
//
// Outputs:      count - Current coarse cycle count
// =============================================================================

`timescale 1ns/1ps

module coarse_counter #(
    parameter WIDTH = 14  // 14-bit coarse count (0 to 16383, ~81.92 us at 200 MHz)
)(
    input  wire                 clk,   // Rising-edge triggered system clock
    input  wire                 rst,   // Synchronous reset, active-high

    output reg  [WIDTH-1:0]     count  // Coarse clock cycle count output
);

    // -------------------------------------------------------------------------
    // Sequential Logic: Increments count on every clock cycle.
    // Uses non-blocking assignment (<=) to prevent race conditions.
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            // Synchronous reset clears the counter to 0
            count <= {WIDTH{1'b0}};
        end else begin
            // Increment the counter value by 1 on each rising clock edge
            count <= count + 1'b1;
        end
    end

endmodule
