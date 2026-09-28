// =============================================================================
// Module Name:  capture_controller
// Description:  Controls the measurement capture cycle. When armed, a STOP
//               event schedules a capture. On the next rising edge of
//               capture_clk after the STOP is recognised, capture_enable is
//               asserted for exactly ONE clock cycle to latch the TDL and
//               coarse-counter data. Subsequent triggers are ignored until
//               clear_status re-arms the controller.
//
// Clock-Domain-Crossing (CDC) handling — IMPORTANT:
//   `stop_pulse` and `clear_status` are ASYNCHRONOUS to capture_clk. Sampling
//   them directly in the synchronous FSM (as the previous version did) causes
//   intermittent metastability in real timing: a STOP whose edge lands in the
//   setup/hold window of the FSM flip-flops can be dropped, so `done` never
//   asserts and the measurement is lost. This showed up as missed captures in
//   post-synthesis timing simulation while behavioural simulation looked clean.
//
//   Fix: both asynchronous inputs are passed through a 2-flop synchroniser
//   into the capture_clk domain before use. `stop_pulse` is then rising-edge
//   detected to produce a clean single-cycle request. This makes capture
//   recognition deterministic and metastability-safe.
//
//   NOTE (real hardware): a 2-flop level synchroniser reliably captures any
//   STOP held high for >= ~2 capture_clk periods. If, on silicon, STOP can be
//   SHORTER than one capture_clk period, precede this block with an
//   async-set / sync-clear "pulse catcher" so the short event is stretched
//   before synchronisation. (The launch path into the TDL stays asynchronous
//   and is unaffected — only the capture-scheduling path is synchronised.)
//
// Inputs:       capture_clk  - System clock used for capturing TDL/counter
//               rst          - Active-high asynchronous reset
//               stop_pulse   - Incoming (async) STOP pulse from the frontend
//               clear_status - Re-arm the controller (async)
//
// Outputs:      capture_enable - Single-cycle gate for sampling registers
//               done           - Status flag indicating capture complete
// =============================================================================

`timescale 1ns/1ps

module capture_controller (
    input  wire capture_clk,
    input  wire rst,
    input  wire stop_pulse,
    input  wire clear_status,
    output reg  capture_enable,
    output reg  done
);

    reg armed;
    reg pending_measurement;

    // -------------------------------------------------------------------------
    // CDC: 2-flop synchronisers for the asynchronous inputs.
    //   stop_sync[2:0] : {edge-detect, sync2, sync1} for stop_pulse
    //   clr_sync[1:0]  : 2-flop synchroniser for clear_status
    // -------------------------------------------------------------------------
    reg [2:0] stop_sync;
    reg [1:0] clr_sync;

    always @(posedge capture_clk or posedge rst) begin
        if (rst) begin
            stop_sync <= 3'b000;
            clr_sync  <= 2'b00;
        end else begin
            stop_sync <= {stop_sync[1:0], stop_pulse};
            clr_sync  <= {clr_sync[0],   clear_status};
        end
    end

    // Clean, single-cycle STOP request in the capture_clk domain.
    wire stop_request = stop_sync[1] & ~stop_sync[2];
    // Synchronised re-arm level.
    wire clear_synced = clr_sync[1];

    // -------------------------------------------------------------------------
    // Capture FSM — now driven only by synchronised, glitch-free signals.
    // -------------------------------------------------------------------------
    always @(posedge capture_clk or posedge rst) begin
        if (rst) begin
            capture_enable      <= 1'b0;
            done                <= 1'b0;
            armed               <= 1'b1;
            pending_measurement <= 1'b0;
        end else if (clear_synced) begin
            capture_enable      <= 1'b0;
            done                <= 1'b0;
            armed               <= 1'b1;
            pending_measurement <= 1'b0;
        end else begin
            // Default: clear the single-cycle pulse
            capture_enable <= 1'b0;

            // Latch the synchronised STOP request
            if (stop_request && armed && !done) begin
                pending_measurement <= 1'b1;
            end

            // Issue one-cycle capture enable on a pending measurement
            if (pending_measurement && !done) begin
                capture_enable      <= 1'b1;
                done                <= 1'b1;
                armed               <= 1'b0;
                pending_measurement <= 1'b0;
            end
        end
    end

endmodule