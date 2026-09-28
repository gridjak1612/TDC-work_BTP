// =============================================================================
// Module Name:  dps_phase_ctrl
// Description:  Controller for the MMCM fine dynamic phase shift on CLKOUT1
//               (the 25 MHz calibration clock).
//
//   ONE step = T_VCO/56 = 17.857 ps of clk_cal relative to the FIXED 200 MHz
//   sampler. 280 steps = 5.000 ns = exactly one sampler period.
//
//   TWO step sources, OR-ed:
//     step_btn  - raw, async, bouncy board button. Synchronised, debounced,
//                 rising-edge detected. One press = one step. Bring-up only.
//     step_req  - SYNCHRONOUS single-cycle request from the autonomous sweep
//                 FSM. Bypasses the debouncer entirely: a 328 us debounce on a
//                 clean internal pulse would cap the sweep at ~3 kstep/s for no
//                 reason, and the signal has no bounce to remove.
//
//   PS PROTOCOL (7-series, UG472):
//     PSEN high for exactly ONE psclk cycle starts a step; PSINCDEC (1=inc,
//     0=dec) is sampled with it; PSDONE returns high once, a deterministic
//     12 psclk cycles later. No new step may start until PSDONE returns.
//
//   PSDONE WATCHDOG -- WHY IT EXISTS
//     Without it, a missed PSDONE leaves the FSM parked in S_WAIT with busy
//     stuck high, and the controller never accepts another step. Under manual
//     button presses you would notice within seconds. Under an unattended
//     280-step sweep you would get a truncated dataset, no error, and no way
//     to tell it apart from a short run -- the samples that DID arrive look
//     perfectly valid. The watchdog recovers the FSM and raises a sticky
//     ps_error so the failure is visible on an LED and, more importantly, so
//     the run is known to be suspect.
//
//   Control path only. Never near the launch net.
// =============================================================================
`timescale 1ns/1ps

module dps_phase_ctrl #(
    parameter integer DEBOUNCE_BITS = 16,   // ~0.33 ms @200 MHz: kills contact bounce
    parameter integer PHASE_BITS    = 12,   // signed running index, +/-2048 steps
    // PSDONE arrives 12 psclk cycles after PSEN. 64 is generous; anything
    // beyond it is a real failure, not slow silicon.
    parameter integer PSDONE_TIMEOUT = 64
)(
    input  wire                         psclk,     // = clk200
    input  wire                         rst,       // active high

    input  wire                         step_btn,  // async: one press = one step
    input  wire                         dir_btn,   // async: held = decrement

    input  wire                         step_req,  // SYNC 1-cycle: sweep FSM
    input  wire                         dir_req,   // SYNC level: 1 = decrement

    input  wire                         psdone,    // from MMCM

    output reg                          psen,      // to MMCM
    output reg                          psincdec,  // to MMCM

    output reg signed [PHASE_BITS-1:0]  phase_idx, // running net phase (frame/ILA)
    output reg                          busy,      // high while a step is in flight
    output reg                          ps_error   // sticky: a PSDONE was missed
);

    // ---- 2-FF synchronisers for the async buttons -----------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] step_sync, dir_sync;
    always @(posedge psclk) begin
        if (rst) begin step_sync <= 2'b00; dir_sync <= 2'b00; end
        else     begin step_sync <= {step_sync[0], step_btn};
                       dir_sync  <= {dir_sync[0],  dir_btn }; end
    end
    wire step_level = step_sync[1];
    wire dir_level  = dir_sync[1];

    // ---- Debounce step_level, then rising-edge detect -------------------------
    reg [DEBOUNCE_BITS-1:0] db_cnt;
    reg                     step_stable, step_stable_d;
    always @(posedge psclk) begin
        if (rst) begin
            db_cnt <= 0; step_stable <= 1'b0; step_stable_d <= 1'b0;
        end else begin
            if (step_level == step_stable) db_cnt <= 0;            // steady: hold
            else begin
                db_cnt <= db_cnt + 1'b1;                           // changing: count
                if (&db_cnt) step_stable <= step_level;            // stable long enough
            end
            step_stable_d <= step_stable;
        end
    end
    wire btn_pulse = step_stable & ~step_stable_d;                 // 1 cycle / press

    // Either source may request a step. step_req wins the direction when both
    // land on the same cycle, because the sweep must not be steered by a button
    // someone happens to be leaning on.
    wire step_go  = btn_pulse | step_req;
    wire step_dec = step_req ? dir_req : dir_level;                // 1 = decrement

    // ---- PS handshake FSM -----------------------------------------------------
    localparam S_IDLE = 2'd0, S_PULSE = 2'd1, S_WAIT = 2'd2;
    reg [1:0] state;
    reg [$clog2(PSDONE_TIMEOUT+1)-1:0] wd;

    always @(posedge psclk) begin
        if (rst) begin
            state <= S_IDLE; psen <= 1'b0; psincdec <= 1'b0;
            busy  <= 1'b0;   phase_idx <= {PHASE_BITS{1'b0}};
            wd    <= 0;      ps_error  <= 1'b0;
        end else begin
            psen <= 1'b0;                                          // PSEN is 1 cycle
            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (step_go) begin
                        psincdec  <= ~step_dec;                    // 1 = increment
                        psen      <= 1'b1;
                        busy      <= 1'b1;
                        phase_idx <= step_dec ? (phase_idx - 1'b1)
                                              : (phase_idx + 1'b1);
                        wd        <= 0;
                        state     <= S_PULSE;
                    end
                end
                S_PULSE: state <= S_WAIT;                          // PSEN de-asserted
                S_WAIT: begin
                    if (psdone) begin
                        state <= S_IDLE;
                    end else if (wd == PSDONE_TIMEOUT[$clog2(PSDONE_TIMEOUT+1)-1:0]) begin
                        // Recover rather than hang. Flag the run as suspect.
                        ps_error <= 1'b1;
                        state    <= S_IDLE;
                    end else begin
                        wd <= wd + 1'b1;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
