`timescale 1ns/1ps
//=============================================================================
// tb_tdc_dual_timing.v
//
//   >>> FOR POST-IMPLEMENTATION TIMING SIMULATION ONLY. <<<
//   Vivado: Simulation -> Run Post-Implementation Timing Simulation
//
// WHY THIS TESTBENCH EXISTS
// -------------------------
// In BEHAVIOURAL sim, CARRY4 is the zero-delay UNISIM model: the fine path
// cannot be tested at all. In TIMING sim, the SDF back-annotates REAL delays
// onto the actual placed-and-routed chain. So this is the ONLY Vivado flow in
// which the interpolator is testable -- and the only simulation of ANY kind
// that models METASTABILITY.
//
// It therefore does NOT assume a value for tau. It MEASURES it.
//
// WHAT IT CHECKS (none of it reachable any other way in Vivado):
//
//   1. CDC / METASTABILITY.  Fire N events at random phases and count how many
//      produce a measurement. A 2-flop synchroniser whose flops got scattered
//      by the placer will DROP captures -- intermittently, non-deterministically,
//      and invisibly in behavioural sim. That is exactly the bug that hit the
//      original capture_controller. There are TWO synchronisers now.
//      >>> ANY missed capture here is a real bug. Zero tolerance. <<<
//
//   2. TAU, from YOUR routed chain.  Sweep the event phase across one clock
//      period and record `fine`. The slope gives the per-tap delay. This is a
//      prediction of what the hardware will do, obtained before you program the
//      board.
//
//   3. MONOTONICITY of the transfer function on the real netlist.
//
//   4. DEAD ZONE.  How much of the 5 ns window rails? This tells you whether
//      88 CARRY4 was the right call BEFORE you burn a bitstream.
//
// TWO GOTCHAS
// -----------
//  * GSR. Post-implementation sim pulls in glbl, which holds every flop in
//    reset for the first 100 ns. Nothing works before then. This TB waits.
//  * SPEED. An SDF-annotated netlist with 176 CARRY4s is SLOW -- expect minutes
//    to tens of minutes. The sweeps here are deliberately short. Do NOT put an
//    800-vector random test in a timing sim; run those behaviourally first.
//    Debugging a logic bug inside an SDF simulation is miserable.
//=============================================================================
module tb_tdc_dual_timing;

    localparam integer TAPS   = 352;
    localparam real    TCLK   = 5.000;
    localparam integer NPHASE = 21;     // phase sweep points, 0.00 .. 5.00 ns
    localparam integer NCDC   = 60;     // random-phase events for the CDC hunt

    reg clk100 = 0;
    reg rst    = 1;
    reg ev_a   = 0;
    reg ev_b   = 0;
    reg clr    = 0;

    wire [13:0] d_coarse;
    wire [8:0]  fine_a, fine_b;
    wire        valid_a, valid_b, timeout, meas_ready;
    wire        done_a, done_b, clk200, mmcm_locked;

    // NOTE: no parameter override. In a post-implementation netlist the
    // parameters are already baked in -- TIMEOUT_CYCLES is whatever you
    // synthesised (16384). The TB must not rely on overriding it.
    // NO parameter override: a post-implementation netlist is FLAT, the
    // parameters are already baked in. That is fine here -- `shot()` fires
    // START and STOP together, so the pairing FSM completes immediately and
    // the 16384-cycle timeout is NEVER entered.
    tdc_dual_top uut (
        .clk100(clk100), .rst(rst),
        .event_a(ev_a), .event_b(ev_b), .clear_status(clr),
        .d_coarse(d_coarse), .fine_a(fine_a), .fine_b(fine_b),
        .valid_a(valid_a), .valid_b(valid_b),
        .timeout(timeout), .meas_ready(meas_ready),
        .done_a(done_a), .done_b(done_b),
        .clk200(clk200), .mmcm_locked(mmcm_locked)
    );

    always #5 clk100 = ~clk100;

    integer pass = 0, fail = 0;

    reg [8:0]  r_fa, r_fb;
    reg [13:0] r_dc;
    reg        r_va, r_vb, r_tmo, r_ok;

    task wait_meas(input integer max_cyc);
        integer g;
        begin
            r_ok = 1'b0; g = 0;
            while (g < max_cyc && r_ok == 1'b0) begin
                @(posedge clk200); #0.1;
                if (meas_ready === 1'b1) begin
                    r_dc = d_coarse; r_fa = fine_a;  r_fb  = fine_b;
                    r_va = valid_a;  r_vb = valid_b; r_tmo = timeout;
                    r_ok = 1'b1;
                end
                g = g + 1;
            end
        end
    endtask

    task rearm;
        integer g;
        begin
            clr = 1'b1; repeat (8) @(posedge clk200);
            clr = 1'b0; repeat (8) @(posedge clk200);
            g = 0;
            while (g < 200 && (done_a === 1'b1 || done_b === 1'b1)) begin
                @(posedge clk200); g = g + 1;
            end
        end
    endtask

    // Fire START and STOP together, at a chosen phase.
    // Both chains see the same edge -> d_coarse ~ 0, and we get BOTH fine
    // values from one shot. That is also exactly how the calibration bitstream
    // (TIE_CHANNELS=1) will be used on hardware.
    task shot(input real phase);
        begin
            rearm;
            @(posedge clk200);
            #(phase);
            ev_a = 1'b1; ev_b = 1'b1;
            #60;
            ev_a = 1'b0; ev_b = 1'b0;
            wait_meas(400);
        end
    endtask

    integer i, k, n_miss, n_sat, n_drain, n_mid, n_invalid;
    integer f_lo, f_hi, monotonic;
    real    ph, tau_est, span;
    integer prev;

    initial begin
        $display("");
        $display("======================================================================");
        $display("  POST-IMPLEMENTATION TIMING SIMULATION");
        $display("  Real SDF delays on the placed-and-routed chain.");
        $display("  This is the ONLY simulation that models METASTABILITY, and the");
        $display("  only Vivado flow in which the fine interpolator works at all.");
        $display("======================================================================");

        // ---- GSR: glbl holds every flop in reset for 100 ns ----
        #200;
        repeat (10) @(posedge clk100);
        rst = 0;
        wait (mmcm_locked);
        repeat (40) @(posedge clk200);

        //------------------------------------------------------------------
        $display("");
        $display("--- 1. TRANSFER FUNCTION: sweep the event phase across one clock ---");
        f_lo = -1; f_hi = -1;
        prev = 9999;
        monotonic = 1;
        n_sat = 0; n_drain = 0; n_mid = 0;

        for (i = 0; i < NPHASE; i = i + 1) begin
            ph = (TCLK * i) / (NPHASE - 1);       // 0.00 .. 5.00 ns
            shot(ph);
            if (!r_ok) begin
                $display("    phase=%0.2f ns -> *** NO MEASUREMENT (capture lost) ***", ph);
                fail = fail + 1;
            end else begin
                $display("    phase=%0.2f ns -> fine_a=%3d  fine_b=%3d  valid=%b%b",
                         ph, r_fa, r_fb, r_va, r_vb);
                if (r_fa == TAPS)      n_sat   = n_sat + 1;
                else if (r_fa == 0)    n_drain = n_drain + 1;
                else begin
                    n_mid = n_mid + 1;
                    if (f_hi == -1) begin f_hi = r_fa; end   // first mid-range = earliest phase
                    f_lo = r_fa;                              // last  mid-range = latest  phase
                    if (r_fa > prev) monotonic = 0;
                    prev = r_fa;
                end
            end
        end

        $display("");
        if (monotonic == 1) begin
            pass = pass + 1;
            $display("  [PASS] transfer function is MONOTONIC on the real netlist");
        end else begin
            fail = fail + 1;
            $display("  [FAIL] transfer function FOLDS -- the chain is not monotone");
        end

        //------------------------------------------------------------------
        $display("");
        $display("--- 2. TAU, measured from YOUR routed chain ---");
        if (f_hi > f_lo && n_mid >= 2) begin
            // fine(phase) ~ (T - phase)/tau  ->  slope = -1/tau
            // Across the mid-range span we covered roughly (n_mid-1) steps.
            tau_est = ((TCLK / (NPHASE-1)) * (n_mid - 1)) / (f_hi - f_lo);
            span    = TAPS * tau_est;
            $display("    fine ran %0d -> %0d over the mid-range sweep", f_hi, f_lo);
            $display("    tau ~ %0.2f ps / tap", tau_est * 1000.0);
            $display("    chain span = %0d taps x %0.2f ps = %0.2f ns  (clock is %0.2f ns)",
                     TAPS, tau_est*1000.0, span, TCLK);
            if (span > TCLK) begin
                pass = pass + 1;
                $display("  [PASS] chain SPANS the clock period -> no saturation dead zone");
            end else begin
                fail = fail + 1;
                $display("  [FAIL] chain is SHORTER than the clock period -> %0.0f%% dead zone",
                         100.0*(TCLK-span)/TCLK);
                $display("         -> increase NUM_CARRY4 before building a bitstream");
            end
        end else begin
            fail = fail + 1;
            $display("  [FAIL] could not extract tau -- not enough mid-range points");
        end

        $display("");
        $display("    saturated (fine=%0d) : %0d / %0d", TAPS, n_sat,   NPHASE);
        $display("    drained   (fine=0)  : %0d / %0d", n_drain, NPHASE);
        $display("    mid-range           : %0d / %0d", n_mid,   NPHASE);

        //------------------------------------------------------------------
        $display("");
        $display("--- 3. CDC / METASTABILITY HUNT  (the whole reason to run this) ---");
        $display("    %0d events at random phases. A scattered synchroniser DROPS", NCDC);
        $display("    captures intermittently. Behavioural sim CANNOT see this.");
        n_miss = 0; n_invalid = 0;
        for (i = 0; i < NCDC; i = i + 1) begin
            ph = ($urandom % 5000) / 1000.0;      // 0.000 .. 4.999 ns
            shot(ph);
            if (!r_ok) begin
                n_miss = n_miss + 1;
                $display("    *** MISSED CAPTURE at phase %0.3f ns ***", ph);
            end else if (!r_va || !r_vb) begin
                n_invalid = n_invalid + 1;
            end
        end
        $display("");
        $display("    missed captures    : %0d / %0d", n_miss, NCDC);
        $display("    validator rejects  : %0d / %0d   (bubbles -- expected, not a fault)",
                 n_invalid, NCDC);
        if (n_miss == 0) begin
            pass = pass + 1;
            $display("  [PASS] ZERO missed captures -> CDC synchronisers are sound");
        end else begin
            fail = fail + 1;
            $display("  [FAIL] %0d CAPTURES LOST. This is the metastability bug.", n_miss);
            $display("         -> check ASYNC_REG is applied to BOTH channels'");
            $display("            cap_ctrl_inst/stop_sync_reg* and clr_sync_reg*");
        end

        //------------------------------------------------------------------
        $display("");
        $display("======================================================================");
        $display("  PASS = %0d   FAIL = %0d", pass, fail);
        $display("");
        $display("  Setup/hold warnings on tap_reg_reg[*] are EXPECTED and BENIGN.");
        $display("  That register exists specifically to sample an asynchronous,");
        $display("  actively-propagating delay line. Its inputs are IN MOTION at the");
        $display("  clock edge BY DESIGN. Those warnings are the signature of the TDC");
        $display("  working, not evidence of a fault.");
        $display("======================================================================");
        $display("");
        $finish;
    end

endmodule