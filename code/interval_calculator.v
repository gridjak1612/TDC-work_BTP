// =============================================================================
// Module Name:  interval_calculator
// Description:  Pairs a channel-A result with a channel-B result into one record.
//
//   EITHER-ORDER PAIRING (fix after step 4). The old FSM looked only at ready_a
//   while idle, so a ready_b arriving one clock EARLIER was dropped; A then
//   waited for a B result that had already been captured and timed out 82 us
//   later. In the tied RO build this hit 2.65 % of all events, all from one
//   ~140 ps arrival-phase window (fine_a 100..108). Now whichever result comes
//   first is held, and the other is awaited.
//
//   d_coarse = coarse_b - coarse_a (mod 2^COARSE_BITS). A B-first pair gives
//   -1 (= 16383 unsigned); the host reads d_coarse as signed (+/-40.96 us).
//   With external START/STOP, a negative interval means a STOP-channel event
//   before START (noise); the host rejects it rather than the record vanishing.
//
//   Timeout: if the second result never comes, the record is emitted with that
//   channel's valid = 0, fine = 0, d_coarse = 0, timeout = 1.
// =============================================================================
`timescale 1ns/1ps

module interval_calculator #(
    parameter integer COARSE_BITS     = 14,
    parameter integer FINE_BITS       = 8,
    parameter integer TIMEOUT_CYCLES  = 16384
)(
    input  wire                   clk,
    input  wire                   rst,

    input  wire                   ready_a,
    input  wire [COARSE_BITS-1:0] coarse_a,
    input  wire [FINE_BITS-1:0]   fine_a,
    input  wire                   valid_a,

    input  wire                   ready_b,
    input  wire [COARSE_BITS-1:0] coarse_b,
    input  wire [FINE_BITS-1:0]   fine_b,
    input  wire                   valid_b,

    output wire [COARSE_BITS-1:0] d_coarse,
    output reg  [FINE_BITS-1:0]   fine_a_out,
    output reg  [FINE_BITS-1:0]   fine_b_out,
    output reg                    valid_a_out,
    output reg                    valid_b_out,
    output reg                    timeout,
    output reg                    meas_ready
);

    localparam [1:0] S_IDLE = 2'd0, S_WAIT_B = 2'd1, S_WAIT_A = 2'd2;

    reg [1:0]              state;
    reg [COARSE_BITS-1:0]  coarse_a_r, coarse_b_r;
    reg [31:0]             tmo;

    // Combinational on purpose: both halves are written the cycle BEFORE
    // meas_ready pulses, so this is correct on the meas_ready cycle.
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
            meas_ready <= 1'b0;

            case (state)
            S_IDLE: begin
                if (ready_a) begin
                    coarse_a_r <= coarse_a; fine_a_out <= fine_a; valid_a_out <= valid_a;
                end
                if (ready_b) begin
                    coarse_b_r <= coarse_b; fine_b_out <= fine_b; valid_b_out <= valid_b;
                end
                if (ready_a && ready_b) begin
                    timeout    <= 1'b0;
                    meas_ready <= 1'b1;
                end else if (ready_a) begin
                    tmo   <= TIMEOUT_CYCLES;
                    state <= S_WAIT_B;
                end else if (ready_b) begin
                    tmo   <= TIMEOUT_CYCLES;
                    state <= S_WAIT_A;
                end
            end

            S_WAIT_B: begin
                if (ready_b) begin
                    coarse_b_r <= coarse_b; fine_b_out <= fine_b; valid_b_out <= valid_b;
                    timeout    <= 1'b0;
                    meas_ready <= 1'b1;
                    state      <= S_IDLE;
                end else if (tmo == 32'd0) begin
                    coarse_b_r  <= coarse_a_r;          // d_coarse = 0
                    fine_b_out  <= {FINE_BITS{1'b0}};
                    valid_b_out <= 1'b0;
                    timeout     <= 1'b1;
                    meas_ready  <= 1'b1;
                    state       <= S_IDLE;
                end else begin
                    tmo <= tmo - 1'b1;
                end
            end

            S_WAIT_A: begin
                if (ready_a) begin
                    coarse_a_r <= coarse_a; fine_a_out <= fine_a; valid_a_out <= valid_a;
                    timeout    <= 1'b0;
                    meas_ready <= 1'b1;
                    state      <= S_IDLE;
                end else if (tmo == 32'd0) begin
                    coarse_a_r  <= coarse_b_r;          // d_coarse = 0
                    fine_a_out  <= {FINE_BITS{1'b0}};
                    valid_a_out <= 1'b0;
                    timeout     <= 1'b1;
                    meas_ready  <= 1'b1;
                    state       <= S_IDLE;
                end else begin
                    tmo <= tmo - 1'b1;
                end
            end

            default: state <= S_IDLE;
            endcase
        end
    end

endmodule