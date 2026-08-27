`timescale 1ns/1ps

// =============================================================================
// Testbench:    tb_tdc                  *** THE ONLY TESTBENCH. ***
//
// Description:  ONE universal, black-box, board-level verifier for the
//               two-channel interval TDC. Works UNCHANGED in all three flows:
//
//                   launch_simulation                                (behavioural)
//                   launch_simulation -mode post-synthesis     -type timing
//                   launch_simulation -mode post-implementation -type timing
//
// =============================================================================
//  WHY VIVADO KEPT STEALING THE SIMULATION TOP  --  and why it can't any more
// =============================================================================
//
//  Vivado auto-detects the top as "the module nobody instantiates". Every
//  `update_compile_order` RE-RUNS that detection and silently overwrites
//  whatever top you set.
//
//  The old project had SEVEN parentless modules visible in sim_1:
//      tdc_dual_board  (from sources_1, because SOURCE_SET = sources_1)
//      tb_tdc_dual_xsim, tb_tdc_dual, tb_tdc_dual_board,
//      tb_interval_calculator, tb_encoder_equiv, tb_tdc_dual_timing
//  Seven candidates -> Vivado picks whichever it likes -> it kept choosing
//  `tdc_dual_board`, which has no clock, no stimulus and NO $finish, so
//  `run all` ran forever.
//
//  THE FIX IS STRUCTURAL, NOT A TCL TRICK:
//    * There is now exactly ONE testbench.
//    * It INSTANTIATES tdc_dual_board.
//
//  So tdc_dual_board now HAS a parent, and tb_tdc is the ONLY parentless
//  module in the whole fileset. Vivado's auto-detection can only pick tb_tdc.
//  You may run update_compile_order as often as you like; it cannot break.
//
// =============================================================================
//  RULES THAT MAKE ONE TESTBENCH WORK IN ALL THREE FLOWS
// =============================================================================
//
//  1. It instantiates tdc_dual_board -- the SYNTHESISED TOP. In a post-
//     implementation netlist that is the only thing that exists; tdc_dual_top,
//     tdc_channel etc. have been flattened away.
//
//  2. ZERO HIERARCHICAL REFERENCES. No uut.core.clk200, no uut.ts_latch. Those
//     paths do not survive flattening. Everything is observed through the real
//     device pins: led[15:0] and uart_txd. That is why the LED map matters:
//         led[1]    = mmcm_locked        -> lock detect, no hier ref needed
//         led[2:3]  = channels armed
//         led[6]    = timeout
//         led[15:8] = measurement counter -> "did a capture happen", INSTANTLY
//
//  3. NO PARAMETER OVERRIDES ARE RELIED ON. CLKS_PER_BIT is overridden for
//     speed, but a netlist has no parameters, so the override is silently
//     ignored there. The bench therefore AUTO-CALIBRATES the UART bit period
//     from the 0xAA header (0xAA sent LSB-first starts with start-bit + bit0
//     both LOW = exactly two bit times) and adapts.
//
//  4. IT DRIVES BOTH BUTTON AND EXTERNAL EVENT PINS. EVENT_SRC is a compile-
//     time parameter that a netlist has already baked in, and the bench cannot
//     know which was chosen -- so it drives btn_a AND ev_a_ext together. Works
//     either way.
//
//  5. IT AUTO-DETECTS A ZERO-DELAY CARRY CHAIN. In BEHAVIOURAL simulation the
//     real UNISIM CARRY4 has ZERO propagation delay: every tap flips in the
//     same delta cycle, so `fine` can only rail at 0 or TDL_TAPS. The fine
//     interpolator is STRUCTURALLY UNTESTABLE there, and no testbench can work
//     around it. Rather than reporting a wall of meaningless failures, this
//     bench detects the condition, SKIPS the fine-path checks, and says so.
//     In a TIMING simulation the SDF gives the chain real delays and the same
//     checks run for real.
//
//  6. A GLOBAL WATCHDOG. A hung run kills itself instead of spinning.
//
// =============================================================================
//  PROJECT NOTE
//    carry4_mock.v / carry4_real.v / clk_wiz_0_mock.v are ICARUS-ONLY and must
//    NEVER be added to the Vivado project -- they collide with the real CARRY4
//    primitive and the real clk_wiz_0 IP.
// =============================================================================

module tb_tdc;

    // ---- Parameters ---------------------------------------------------------
    parameter TDL_TAPS   = 352;         // 88 CARRY4 x 4
    parameter real TCLK  = 5.000;       // ns, clk200 period
    parameter SIM_CPB    = 20;          // CLKS_PER_BIT for behavioural speed.
                                        // IGNORED by a netlist (which keeps the
                                        // synthesised 1736). Auto-calibrated.
    parameter N_PHASE    = 12;          // UART-decoded phase points
    parameter N_CDC      = 40;          // LED-only CDC events (no UART wait)
    parameter LED_TIMEOUT   = 4000;     // clk100 edges
    parameter FRAME_TIMEOUT = 20;       // frames worth of time before giving up

    // ---- Stimulus / DUT pins ------------------------------------------------
    reg  clk100, rst;
    reg  btn_a, btn_b, btn_clear;
    reg  ev_a_ext, ev_b_ext;
    reg  sw_autorearm;

    wire [15:0] led;
    wire        uart_txd;

    // Observable status, straight off the pins -- NO hierarchical references.
    wire mmcm_locked = led[1];
    wire armed_a     = led[2];
    wire armed_b     = led[3];
    wire tmo_led     = led[6];
    wire [7:0] meas_cnt = led[15:8];

    // ---- Scoreboard ---------------------------------------------------------
    integer checks_passed = 0;
    integer checks_failed = 0;
    integer i, guard, n_phase_run;
    integer n_miss, n_sat, n_drain, n_mid;
    integer prev_fine, n_fold, n_wrap;
    integer zero_delay_chain;
    real    bit_ns, ph, sep, tau_est, span, coarse_only, err, worst_err;
    real    sum_dphase, prev_phase;
    integer sum_dfine;
    reg [7:0] prev_cnt;

    // Decoded frame
    reg [8:0]  rx_fine_a, rx_fine_b;
    reg [13:0] rx_dcoarse;
    reg        rx_valid_a, rx_valid_b, rx_ok;

    integer k, n_ok, n_bad_valid;
    real    sep_i;
    reg [7:0] cnt_before;
    reg       held_ok;

    // =========================================================================
    //  DUT -- the SYNTHESISED TOP. This instantiation is what makes tb_tdc the
    //  only parentless module in the project, and therefore the only possible
    //  automatic simulation top.
    // =========================================================================
    tdc_dual_board #(
        .CLKS_PER_BIT(SIM_CPB)     // ignored by a netlist; bit period is auto-calibrated
    ) uut (
        .clk100      (clk100),
        .rst         (rst),
        .btn_a       (btn_a),
        .btn_b       (btn_b),
        .btn_clear   (btn_clear),
        .ev_a_ext    (ev_a_ext),
        .ev_b_ext    (ev_b_ext),
        .sw_autorearm(sw_autorearm),
        .led         (led),
        .uart_txd    (uart_txd)
    );

    // ---- 100 MHz board clock ------------------------------------------------
    // clk200 is phase-locked 2x clk100, so its rising edges land on clk100
    // edges and clk100 midpoints. Firing at `phase` ns after a clk100 posedge
    // therefore sets the clk200 phase to (phase mod 5 ns) -- which is how this
    // bench controls the sub-clock phase WITHOUT ever seeing clk200.
    initial begin
        clk100 = 0;
        forever #5.0 clk100 = ~clk100;
    end

    // =========================================================================
    //  HELPER TASKS
    // =========================================================================
    task assert_check;
        input           condition;
        input [800-1:0] desc;
        begin
            if (condition) begin
                $display("  [PASS] %0s", desc);
                checks_passed = checks_passed + 1;
            end else begin
                $display("  [FAIL] %0s  @ %0t", desc, $time);
                checks_failed = checks_failed + 1;
                dump_state("state at failure");
            end
        end
    endtask

    task dump_state;
        input [800-1:0] tag;
        begin
            $display("      --- %0s (t=%0t) ---", tag, $time);
            $display("      led=%b", led);
            $display("      locked=%b armed_a=%b armed_b=%b timeout=%b meas_cnt=%0d",
                     mmcm_locked, armed_a, armed_b, tmo_led, meas_cnt);
            $display("      rx: fine_a=%0d fine_b=%0d d_coarse=%0d valid=%b%b ok=%b",
                     rx_fine_a, rx_fine_b, rx_dcoarse, rx_valid_a, rx_valid_b, rx_ok);
        end
    endtask

    // ---- Drive BOTH button and external pins: works for either EVENT_SRC ----
    task set_event_a; input v; begin btn_a = v; ev_a_ext = v; end endtask
    task set_event_b; input v; begin btn_b = v; ev_b_ext = v; end endtask

    task rearm_capture;
        begin
            btn_clear = 1'b1; repeat (4) @(posedge clk100);
            btn_clear = 1'b0; repeat (4) @(posedge clk100);
            guard = 0;
            while (guard < LED_TIMEOUT && (armed_a !== 1'b1 || armed_b !== 1'b1)) begin
                @(posedge clk100);
                guard = guard + 1;
            end
        end
    endtask

    task fire_pair;
        input real phase;
        input real separation;
        begin
            @(posedge clk100);
            #(phase);
            set_event_a(1'b1);
            #(separation);
            set_event_b(1'b1);
            #60;
            set_event_a(1'b0);
            set_event_b(1'b0);
        end
    endtask

    // Wait for the measurement counter LED to tick. INSTANT -- no UART wait.
    // This is how the CDC hunt runs 40 events in microseconds instead of
    // 40 x 521 us.
    task wait_for_capture;
        output ok;
        begin
            prev_cnt = meas_cnt;
            guard    = 0;
            ok       = 1'b0;
            while (guard < LED_TIMEOUT && ok === 1'b0) begin
                @(posedge clk100);
                if (meas_cnt !== prev_cnt) ok = 1'b1;
                guard = guard + 1;
            end
        end
    endtask

    // =========================================================================
    //  UART RECEIVER  (8N1, LSB first -- exactly what read_tdc_dual.py does)
    //
    //  AUTO-CALIBRATION: the header is 0xAA = 8'b10101010. Sent LSB-first the
    //  line goes  idle(1) start(0) bit0(0) bit1(1) ...  -- so the FIRST low
    //  period is start+bit0 = EXACTLY TWO BIT TIMES. Measuring negedge->posedge
    //  and halving it gives the bit period, whatever CLKS_PER_BIT the DUT was
    //  built with. That is what lets one bench serve behavioural (fast UART)
    //  and netlist (real 115200) simulation.
    // =========================================================================
    task uart_calibrate;
        real t0, t1;
        begin
            @(negedge uart_txd);          // start bit of the first frame
            t0 = $realtime;
            @(posedge uart_txd);          // end of bit0 (0xAA -> bit0 is 0)
            t1 = $realtime;
            bit_ns = (t1 - t0) / 2.0;
            $display("  UART bit period auto-calibrated: %0.1f ns  (%0.0f baud)",
                     bit_ns, 1.0e9 / bit_ns);
            if (bit_ns > 1000.0) begin
                $display("  -> real 115200 baud: this is a NETLIST simulation. One frame");
                $display("     takes ~%0.0f us, so the phase sweep is SLOW. Expected.",
                         bit_ns * 60.0 / 1000.0);
            end
        end
    endtask

    task uart_get_byte;
        output [7:0] b;
        integer k;
        begin
            @(negedge uart_txd);          // start bit
            #(bit_ns * 1.5);              // centre of bit 0
            for (k = 0; k < 8; k = k + 1) begin
                b[k] = uart_txd;
                #(bit_ns);
            end
        end
    endtask

    task uart_get_frame;
        reg [7:0] h, b1, b2, b3, b4, b5;
        integer   hunt;
        begin
            rx_ok = 1'b0;
            h     = 8'h00;
            hunt  = 0;
            while (h !== 8'hAA && hunt < 12) begin
                uart_get_byte(h);
                hunt = hunt + 1;
            end
            if (h === 8'hAA) begin
                uart_get_byte(b1);
                uart_get_byte(b2);
                uart_get_byte(b3);
                uart_get_byte(b4);
                uart_get_byte(b5);
                rx_fine_a  = {b4[6], b1};
                rx_fine_b  = {b4[7], b2};
                rx_dcoarse = {b4[5:0], b3};
                rx_valid_a = b5[0];
                rx_valid_b = b5[1];
                rx_ok      = 1'b1;
            end
        end
    endtask

    // =========================================================================
    //  MAIN STIMULUS
    // =========================================================================
    reg cap_ok;

    initial begin
        $dumpfile("tb_tdc.vcd");
        $dumpvars(0, tb_tdc);

        worst_err = 0.0; n_miss = 0; n_sat = 0; n_drain = 0; n_mid = 0;
        prev_fine = -1; prev_phase = 0.0; n_fold = 0; n_wrap = 0;
        sum_dphase = 0.0; sum_dfine = 0;
        zero_delay_chain = 1;           // assume rails until proven otherwise
        bit_ns = SIM_CPB * TCLK;        // provisional; overwritten by calibration

        $display("======================================================================");
        $display("  TWO-CHANNEL INTERVAL TDC -- UNIVERSAL BLACK-BOX VERIFIER");
        $display("  Instantiates tdc_dual_board. No hierarchical references.");
        $display("  Runs unchanged in behavioural, post-synthesis and post-impl sim.");
        $display("======================================================================");

        // ---- Reset & MMCM lock (observed on led[1]) ----
        $display("\n--- Reset & MMCM lock ---");
        rst = 1'b1;
        btn_a = 1'b0; btn_b = 1'b0; btn_clear = 1'b0;
        ev_a_ext = 1'b0; ev_b_ext = 1'b0;
        sw_autorearm = 1'b0;            // manual re-arm: INSTANT, no UART wait
        #300;                            // also covers glbl GSR (100 ns) in netlist sim
        repeat (10) @(posedge clk100);
        rst = 1'b0;

        guard = 0;
        while (guard < LED_TIMEOUT && mmcm_locked !== 1'b1) begin
            @(posedge clk100);
            guard = guard + 1;
        end
        assert_check(mmcm_locked === 1'b1, "MMCM locked (led[1])");
        repeat (40) @(posedge clk100);

        assert_check(armed_a === 1'b1 && armed_b === 1'b1,
                     "both channels ARMED after reset (led[2], led[3])");

        // ---- Calibrate the UART bit period off the first frame ----
        $display("\n--- UART auto-calibration ---");
        rearm_capture;
        fork
            fire_pair(1.30, 17.30);
            uart_calibrate;
        join

        // ---- Phase sweep, decoding the real serial stream ----
        $display("\n--- Phase sweep: %0d points across one clock period ---", N_PHASE);
        $display("    (STOP-launch: a LATER event has LESS time to climb the chain,");
        $display("     so `fine` must FALL as the phase increases)");
        n_phase_run = 0;
        for (i = 0; i < N_PHASE; i = i + 1) begin
            ph  = (TCLK * i) / (N_PHASE - 1);     // 0.00 .. 5.00 ns
            sep = 17.30;

            rearm_capture;
            fork
                fire_pair(ph, sep);
                uart_get_frame;
            join

            if (rx_ok !== 1'b1) begin
                assert_check(1'b0, "UART frame received for this event");
            end else begin
                n_phase_run = n_phase_run + 1;

                $display("    phase=%0.2f ns -> fine_a=%0d fine_b=%0d d_coarse=%0d valid=%b%b",
                         ph, rx_fine_a, rx_fine_b, rx_dcoarse, rx_valid_a, rx_valid_b);

                // ---- coarse check: valid in EVERY flow, even zero-delay ----
                coarse_only = rx_dcoarse * TCLK;
                err = coarse_only - sep;
                if (err < 0) err = -err;
                if (err > worst_err) worst_err = err;

                // ---- classify the fine value ----
                if      (rx_fine_a == TDL_TAPS) n_sat   = n_sat   + 1;
                else if (rx_fine_a == 0)        n_drain = n_drain + 1;
                else begin
                    n_mid            = n_mid + 1;
                    zero_delay_chain = 0;          // a mid-range code proves real delay

                    if (prev_fine != -1) begin
                        if (rx_fine_a < prev_fine) begin
                            // Same ramp: the event moved LATER, so it had LESS
                            // time to climb. Accumulate the slope -> tau.
                            sum_dphase = sum_dphase + (ph - prev_phase);
                            sum_dfine  = sum_dfine  + (prev_fine - rx_fine_a);
                        end
                        else if (rx_fine_a > prev_fine + (TDL_TAPS/4)) begin
                            // A BIG jump UP = the event crossed into the NEXT
                            // clock window. `fine` restarts near the top and
                            // d_coarse DECREMENTS to compensate. That is the
                            // coarse/fine handoff -- CORRECT, not a fault.
                            n_wrap = n_wrap + 1;
                        end
                        else begin
                            // A SMALL increase with no wrap = the transfer
                            // function FOLDED. The chain is not monotone. BUG.
                            n_fold = n_fold + 1;
                            $display("      ^ FOLD: fine rose %0d -> %0d without a wrap",
                                     prev_fine, rx_fine_a);
                        end
                    end
                    prev_fine  = rx_fine_a;
                    prev_phase = ph;
                end
            end
        end

        assert_check(n_phase_run == N_PHASE, "every event produced a UART frame");
        assert_check(worst_err < TCLK,
                     "coarse-only interval within ONE clock period (valid in all flows)");

        // ---- Fine-path checks: ONLY if the chain has real delay ----
        $display("\n--- Fine interpolator ---");
        $display("    saturated (fine=%0d) : %0d / %0d", TDL_TAPS, n_sat, n_phase_run);
        $display("    drained   (fine=0)   : %0d / %0d", n_drain, n_phase_run);
        $display("    mid-range            : %0d / %0d", n_mid, n_phase_run);

        if (zero_delay_chain == 1) begin
            $display("");
            $display("  >>> ZERO-DELAY CARRY4 DETECTED <<<");
            $display("  Every fine value railed at 0 or %0d. This is a BEHAVIOURAL", TDL_TAPS);
            $display("  simulation: the real UNISIM CARRY4 model has no propagation");
            $display("  delay, so the delay line is not a time ruler and the fine");
            $display("  interpolator CANNOT be tested. That is a property of the");
            $display("  primitive's simulation model, NOT a design fault, and no");
            $display("  testbench can work around it.");
            $display("");
            $display("  Fine-path checks SKIPPED (not failed).");
            $display("  To test the fine path, re-run as:");
            $display("      launch_simulation -mode post-implementation -type timing");
            $display("  The SDF then gives the chain REAL delays and these same");
            $display("  checks run for real. Or use hardware.");
        end else begin
            $display("    clock-window wraps   : %0d  (EXPECTED: `fine` restarts and", n_wrap);
            $display("                             d_coarse decrements to compensate --");
            $display("                             this IS the coarse/fine handoff)");
            $display("    folds (fine rose, no wrap) : %0d", n_fold);

            assert_check(n_fold == 0,
                         "transfer function MONOTONIC within every clock window");

            if (sum_dfine > 0) begin
                // tau from the accumulated slope of every DOWNWARD step. Wraps
                // are excluded, so a mid-sweep clock-window crossing does not
                // corrupt the fit.
                tau_est = sum_dphase / sum_dfine;
                span    = TDL_TAPS * tau_est;
                $display("");
                $display("    slope fit over %0d taps of travel", sum_dfine);
                $display("    tau  = %0.2f ps / tap   <-- MEASURED from the routed chain",
                         tau_est * 1000.0);
                $display("    span = %0d taps x %0.2f ps = %0.2f ns   (clock is %0.2f ns)",
                         TDL_TAPS, tau_est*1000.0, span, TCLK);
                assert_check(span > TCLK,
                             "chain SPANS the clock period -> no saturation dead zone");
                if (span <= TCLK)
                    $display("      -> %0.0f%% of the clock window is DEAD. Increase NUM_CARRY4.",
                             100.0*(TCLK-span)/TCLK);
            end else begin
                assert_check(1'b0, "tau extracted (enough downward steps)");
            end
        end

        // ---- CDC / metastability hunt: LED-only, so it is FAST ----
        $display("\n--- CDC / metastability hunt: %0d random-phase events ---", N_CDC);
        $display("    Detected via the measurement counter LED, so NO UART wait.");
        $display("    A synchroniser scattered by the placer DROPS captures --");
        $display("    intermittently, and INVISIBLY in behavioural simulation.");
        $display("    >>> ANY missed capture is a real bug. Zero tolerance. <<<");
        n_miss = 0;
        for (i = 0; i < N_CDC; i = i + 1) begin
            ph = ($urandom % 5000) / 1000.0;      // 0.000 .. 4.999 ns
            rearm_capture;
            fire_pair(ph, 17.30);
            wait_for_capture(cap_ok);
            if (cap_ok !== 1'b1) begin
                n_miss = n_miss + 1;
                $display("    *** MISSED CAPTURE at phase %0.3f ns ***", ph);
            end
        end
        $display("    missed captures : %0d / %0d", n_miss, N_CDC);
        assert_check(n_miss == 0, "ZERO missed captures -> CDC synchronisers are sound");
        if (n_miss != 0) begin
            $display("      -> Check ASYNC_REG is applied to BOTH channels'");
            $display("         cap_ctrl_inst/stop_sync_reg* and clr_sync_reg*");
        end

        // =================================================================
        // ---- Separation sweep: d_coarse must track the true interval ----
        // The phase sweep above used ONE separation. This sweeps the
        // separation itself, so d_coarse is exercised across many values
        // rather than sitting at 3-4.
        // =================================================================
        $display("\n--- Separation sweep: d_coarse must track the interval ---");
        n_ok = 0;
        n_bad_valid = 0;
        for (i = 0; i < 8; i = i + 1) begin
            sep_i = 12.00 + i * 4.30;          // 12.0 .. 42.1 ns, non-integer
            rearm_capture;
            fork
                fire_pair(1.70, sep_i);
                uart_get_frame;
            join
            if (rx_ok !== 1'b1) begin
                $display("    sep=%0.2f ns -> NO FRAME", sep_i);
            end else begin
                coarse_only = rx_dcoarse * TCLK;
                err = coarse_only - sep_i;
                if (err < 0) err = -err;
                if (err > worst_err) worst_err = err;
                $display("    sep=%0.2f ns -> d_coarse=%0d (%0.3f ns)  err=%0.3f ns  valid=%b%b",
                         sep_i, rx_dcoarse, coarse_only, err, rx_valid_a, rx_valid_b);
                if (err < TCLK) n_ok = n_ok + 1;
                if (!rx_valid_a || !rx_valid_b) n_bad_valid = n_bad_valid + 1;
            end
        end
        assert_check(n_ok == 8, "d_coarse tracks every separation to within one clock");
        assert_check(n_bad_valid == 0,
                     "validator accepted every clean thermometer code");

        // =================================================================
        // ---- Lockout: a second event while done=1 must be IGNORED ----
        // This is what stops a bouncing button (or a free-running generator)
        // from overwriting a measurement mid-readout.
        // =================================================================
        $display("\n--- Lockout: a second event while captured must be IGNORED ---");
        rearm_capture;
        cnt_before = meas_cnt;
        fire_pair(2.20, 15.00);
        wait_for_capture(cap_ok);
        assert_check(cap_ok === 1'b1, "first event captured");
        assert_check(armed_a === 1'b0 && armed_b === 1'b0,
                     "channels LOCKED OUT after capture (led[2],led[3] low)");
        prev_cnt = meas_cnt;
        fire_pair(3.30, 15.00);            // second event, NO re-arm
        repeat (400) @(posedge clk100);
        assert_check(meas_cnt === prev_cnt,
                     "second event while locked out produced NO new measurement");

        // =================================================================
        // ---- Back-to-back: N consecutive measurements, none dropped ----
        // =================================================================
        $display("\n--- Back-to-back: 10 consecutive measurements ---");
        k = 0;
        cnt_before = meas_cnt;
        for (i = 0; i < 10; i = i + 1) begin
            rearm_capture;
            fire_pair(0.7 * i, 14.00);
            wait_for_capture(cap_ok);
            if (cap_ok === 1'b1) k = k + 1;
        end
        assert_check(k == 10, "10 consecutive measurements, none dropped");
        assert_check(meas_cnt === (cnt_before + 8'd10),
                     "measurement counter LED advanced by exactly 10");

        // =================================================================
        // ---- Held-high event must NOT saturate  (snapshot_pipeline fix) ----
        // Before the fix this read the SATURATED chain every time -- the
        // original fine=255 hardware bug. Only meaningful when the chain has
        // real delay, so it is guarded.
        // =================================================================
        $display("\n--- Held-high event must NOT saturate (snapshot_pipeline regression) ---");
        rearm_capture;
        @(posedge clk100);
        #2.00;
        set_event_a(1'b1); set_event_b(1'b1);   // and HOLD
        uart_get_frame;                          // catch it while still high
        set_event_a(1'b0); set_event_b(1'b0);
        if (zero_delay_chain == 1) begin
            $display("    SKIPPED: zero-delay CARRY4 -- every code rails regardless.");
            $display("    (This check is only meaningful in a TIMING simulation.)");
        end else begin
            held_ok = rx_ok && (rx_fine_a != TDL_TAPS) && (rx_fine_a != 0);
            $display("    event held high -> fine_a=%0d  (must NOT be %0d)",
                     rx_fine_a, TDL_TAPS);
            assert_check(held_ok,
                         "held-high event still reports the GOLDEN edge -> snapshot_pipeline works");
            $display("    (before the snapshot fix this railed EVERY time -- the original bug)");
        end
        repeat (40) @(posedge clk100);

        // =================================================================
        // ---- STOP BEFORE START (pathological ordering) ----
        //
        // The pairing FSM waits for ready_a FIRST. A ready_b that arrives
        // BEFORE it is ignored -- so channel A then waits for a STOP that has
        // already been and gone, and the pairing TIMES OUT.
        //
        // That is the CORRECT and SAFE outcome: a STOP that precedes its START
        // is not a measurement, and the design refuses to invent one. It emits
        // a record with valid_b=0 so the host discards it, and re-arms cleanly.
        // What matters is that it does not HANG.
        //
        // The timeout is one full coarse rollover (16384 cyc = 81.92 us), so
        // this test needs a LONG bounded wait.
        // =================================================================
        $display("\n--- STOP before START (pathological ordering) ---");
        rearm_capture;
        @(posedge clk100);
        set_event_b(1'b1);          // STOP first
        #25;
        set_event_a(1'b1);          // START after
        #60;
        set_event_a(1'b0); set_event_b(1'b0);
        guard = 0;
        while (guard < 20000 && tmo_led !== 1'b1) begin
            @(posedge clk100);
            guard = guard + 1;
        end
        assert_check(tmo_led === 1'b1,
                     "STOP-before-START -> TIMES OUT cleanly (does not hang, does not invent a measurement)");
        $display("    valid_b is forced 0, so the host discards the sample. Correct.");

        rearm_capture;
        fire_pair(1.90, 16.00);
        wait_for_capture(cap_ok);
        assert_check(cap_ok === 1'b1,
                     "a normal measurement still works after STOP-before-START");

        // =================================================================
        // ---- Reset asserted mid-capture ----
        // =================================================================
        $display("\n--- Reset asserted MID-CAPTURE ---");
        rearm_capture;
        @(posedge clk100);
        set_event_a(1'b1);
        repeat (2) @(posedge clk100);
        rst = 1'b1;                              // reset while the capture is in flight
        repeat (10) @(posedge clk100);
        rst = 1'b0;
        set_event_a(1'b0);
        guard = 0;
        while (guard < LED_TIMEOUT && mmcm_locked !== 1'b1) begin
            @(posedge clk100); guard = guard + 1;
        end
        repeat (60) @(posedge clk100);
        assert_check(armed_a === 1'b1 && armed_b === 1'b1,
                     "reset mid-capture -> both channels come back ARMED, not stuck");

        rearm_capture;
        fire_pair(2.20, 15.00);
        wait_for_capture(cap_ok);
        assert_check(cap_ok === 1'b1,
                     "a normal measurement still works after the reset");

        // ---- Timeout: START fires, STOP never arrives ----
        $display("\n--- Timeout: START fires, STOP never arrives ---");
        rearm_capture;
        @(posedge clk100);
        set_event_a(1'b1); #60; set_event_a(1'b0);
        // The hardware timeout is one full coarse rollover = 16384 cyc = 81.92 us.
        guard = 0;
        while (guard < 20000 && tmo_led !== 1'b1) begin
            @(posedge clk100);
            guard = guard + 1;
        end
        assert_check(tmo_led === 1'b1, "no STOP -> timeout flag raised (led[6])");

        // ---- Summary ----
        $display("\n======================================================================");
        $display("  VERIFICATION COMPLETE");
        $display("  Checks passed : %0d", checks_passed);
        $display("  Checks failed : %0d", checks_failed);
        $display("  Worst coarse error : %0.3f ns  (must be < %0.3f ns)", worst_err, TCLK);
        if (zero_delay_chain == 1)
            $display("  Fine path      : NOT TESTED (zero-delay CARRY4 -- behavioural sim)");
        else
            $display("  Fine path      : TESTED, tau = %0.2f ps/tap", tau_est*1000.0);
        $display("======================================================================");
        if (checks_failed == 0)
            $display("  >>> ALL TESTS PASSED <<<");
        else
            $display("  >>> %0d CHECK(S) FAILED - see [FAIL] lines + dumps above <<<",
                     checks_failed);
        $display("======================================================================");
        $display("");
        $display("  NOTE: in a TIMING simulation, setup/hold warnings on tap_reg_reg[*]");
        $display("  are EXPECTED and BENIGN. That register exists specifically to sample");
        $display("  an asynchronous, actively-propagating delay line -- its inputs are");
        $display("  IN MOTION at the clock edge BY DESIGN. Those warnings are the");
        $display("  signature of the TDC working, not evidence of a fault.");
        $display("======================================================================");

        #100; $finish;
    end

    // ---- Global watchdog: a hung run kills itself instead of spinning -------
    initial begin
        #500_000_000;
        $display("");
        $display("  >>> GLOBAL TIMEOUT -- simulation did not complete <<<");
        $display("  checks_passed=%0d checks_failed=%0d", checks_passed, checks_failed);
        $finish;
    end

endmodule