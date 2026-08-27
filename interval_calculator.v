// =============================================================================
// Module Name:  interval_calculator
// Description:  Pairs a channel-A (START) result with a channel-B (STOP) result
//               and emits one measurement record.
//
//   THE COARSE SUBTRACTION
//   ----------------------
//   d_coarse = coarse_b - coarse_a, computed in COARSE_BITS-wide arithmetic.
//
//   That is ALL that is needed. Two's-complement subtraction at COARSE_BITS
//   width IS modulo-2^COARSE_BITS arithmetic, so a counter rollover between the
//   two events is handled automatically and exactly:
//
//       coarse_a = 16380, coarse_b = 4   (counter wrapped)
//       4 - 16380 = -16376
//       -16376 mod 16384 = 8             <-- correct, 8 clock periods
//
//   This is valid as long as the true interval is < 2^COARSE_BITS clock
//   periods = 16384 * 5 ns = 81.92 us (a 12.3 km round trip). Any real
//   START/STOP pair is orders of magnitude inside that.
//
//   WHY THE FINE VALUES ARE **NOT** COMBINED HERE
//   ---------------------------------------------
//   The final interval is
//       t = d_coarse * 5 ns  -  (fine_b * tau_b  -  fine_a * tau_a)
//   and tau_a != tau_b: chain A and chain B are different physical carry chains
//   with different per-tap delays AND different per-BIN widths. The correct
//   conversion is a per-bin lookup table from code-density calibration, not a
//   multiply. That LUT lives on the host, where it can be updated without
//   re-synthesising. So we ship the RAW fields and let the host do the maths.
//
//   PAIRING / TIMEOUT
//   -----------------
//   S_IDLE  : wait for ready_a. If ready_b lands on the SAME cycle (which is
//             what happens when both channels are fed the same edge, i.e.
//             calibration mode), complete immediately.
//   S_WAIT_B: wait for ready_b, with a timeout of one full coarse rollover.
//             Beyond that a d_coarse would be ambiguous anyway, so a missed
//             STOP ("no echo") is reported with valid_b forced to 0 rather than
//             silently hanging the channel.
// =============================================================================
`timescale 1ns/1ps

module interval_calculator #(
    parameter integer COARSE_BITS     = 14,
    parameter integer FINE_BITS       = 8,
    // One full coarse rollover. Past this, d_coarse cannot be disambiguated.
    parameter integer TIMEOUT_CYCLES  = 16384
)(
    input  wire                   clk,
    input  wire                   rst,

    // ---- channel A (START) ----
    input  wire                   ready_a,
    input  wire [COARSE_BITS-1:0] coarse_a,
    input  wire [FINE_BITS-1:0]   fine_a,
    input  wire                   valid_a,

    // ---- channel B (STOP) ----
    input  wire                   ready_b,
    input  wire [COARSE_BITS-1:0] coarse_b,
    input  wire [FINE_BITS-1:0]   fine_b,
    input  wire                   valid_b,

    // ---- paired measurement record ----
    output wire [COARSE_BITS-1:0] d_coarse,     // coarse_b - coarse_a (mod 2^N)
    output reg  [FINE_BITS-1:0]   fine_a_out,
    output reg  [FINE_BITS-1:0]   fine_b_out,
    output reg                    valid_a_out,
    output reg                    valid_b_out,  // 0 also means "STOP never came"
    output reg                    timeout,      // sticky-ish flag for the LED
    output reg                    meas_ready    // 1-cycle: record is settled
);

    localparam S_IDLE = 1'b0, S_WAIT_B = 1'b1;

    reg                    state;
    reg [COARSE_BITS-1:0]  coarse_a_r, coarse_b_r;
    reg [31:0]             tmo;

    // Modular subtraction. Truncation to COARSE_BITS does the wrap for us.
    // COMBINATIONAL, not registered: coarse_a_r/coarse_b_r are both written on
    // the cycle BEFORE meas_ready pulses, so on the meas_ready cycle this wire
    // already carries the correct difference. Registering it here would sample
    // coarse_b_r one cycle too early and emit a stale value.
    assign d_coarse = coarse_b_r - coarse_a_r;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            meas_ready  <= 1'b0;
            timeout     <= 1'b0;
            fine_a_out  <= {FINE_BITS{1'b0}};
            fine_b_out  <= {FINE_BITS{1'b0}};
            valid_a_out <= 1'b0;
            valid_b_out <= 1'b0;
            coarse_a_r  <= {COARSE_BITS{1'b0}};
            coarse_b_r  <= {COARSE_BITS{1'b0}};
            tmo         <= 32'd0;
        end else begin
            meas_ready <= 1'b0;      // default: single-cycle strobe

            case (state)

            // ---------------------------------------------------------------
            S_IDLE: begin
                if (ready_a) begin
                    coarse_a_r  <= coarse_a;
                    fine_a_out  <= fine_a;
                    valid_a_out <= valid_a;

                    if (ready_b) begin
                        // Both channels fired on the same edge. This is the
                        // normal case when a single source drives both chains
                        // (calibration / precision measurement): d_coarse ~ 0.
                        coarse_b_r  <= coarse_b;
                        fine_b_out  <= fine_b;
                        valid_b_out <= valid_b;
                        timeout     <= 1'b0;
                        meas_ready  <= 1'b1;
                        state       <= S_IDLE;
                    end else begin
                        tmo   <= TIMEOUT_CYCLES;
                        state <= S_WAIT_B;
                    end
                end
            end

            // ---------------------------------------------------------------
            S_WAIT_B: begin
                if (ready_b) begin
                    coarse_b_r  <= coarse_b;
                    fine_b_out  <= fine_b;
                    valid_b_out <= valid_b;
                    timeout     <= 1'b0;
                    meas_ready  <= 1'b1;
                    state       <= S_IDLE;
                end else if (tmo == 32'd0) begin
                    // No STOP inside one coarse rollover. Report the miss.
                    coarse_b_r  <= coarse_a_r;   // -> d_coarse = 0
                    fine_b_out  <= {FINE_BITS{1'b0}};
                    valid_b_out <= 1'b0;         // host filters on this
                    timeout     <= 1'b1;
                    meas_ready  <= 1'b1;
                    state       <= S_IDLE;
                end else begin
                    tmo <= tmo - 1'b1;
                end
            end

            endcase
        end
    end

endmodule