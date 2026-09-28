`timescale 1ns/1ps

// =============================================================================
// Testbench:    tb_tdc_unit          *** RUNS IN ITS OWN SIMSET: sim_2 ***
//
// Description:  DEEP RTL UNIT verifier. Everything that CANNOT be reached from
//               the board pins, and therefore cannot live in tb_tdc.
//
//               1. ones_counter_encoder_piped + thermometer_validator_piped
//                  proven BIT-IDENTICAL to a behavioural reference, at BOTH the
//                  old width (256) and the new one (352), against random
//                  vectors. Both modules were HARDCODED for 256 taps and had to
//                  be rebuilt width-generic for the 88-CARRY4 chain -- which
//                  INVALIDATED the original equivalence proof. This restores it.
//
//               2. interval_calculator's COARSE ROLLOVER:
//                       d_coarse = coarse_b - coarse_a   (14-bit modular)
//                  A load-bearing claim for EVERY interval measurement, and one
//                  that no board-level test can reach -- the counter wraps every
//                  81.92 us on hardware but never in a short simulation. A bug
//                  here shows up in the lab as rare, random, wildly-wrong
//                  intervals: the worst bug class there is.
//
// =============================================================================
//  WHY A SEPARATE SIMSET, AND NOT A SECOND FILE IN sim_1
// =============================================================================
//  Vivado auto-detects the simulation top as "the module nobody instantiates".
//  Two testbenches in one simset = two parentless modules = Vivado picks one at
//  random and every update_compile_order re-rolls the dice. That is exactly what
//  kept stealing the top before.
//
//  A SIMSET has its own top. sim_1 holds tb_tdc (the only orphan there, because
//  it instantiates tdc_dual_board). sim_2 holds tb_tdc_unit (the only orphan
//  there). Neither can steal the other's top. Vivado only ever elaborates the
//  ACTIVE simset.
//
//  Create it once:
//      create_fileset -simset sim_2
//      set_property SOURCE_SET {} [get_filesets sim_2]        ;# <-- IMPORTANT
//      add_files -fileset sim_2 {tb_tdc_unit.v
//                                ones_counter_pipelined.v
//                                thermometer_validator_pipelined.v
//                                interval_calculator.v}
//
//  Run it:
//      current_fileset -simset [get_filesets sim_2]
//      launch_simulation
//
//  Go back to the board tests:
//      current_fileset -simset [get_filesets sim_1]
//
//  SOURCE_SET must be EMPTY on sim_2. If it points at sources_1 then
//  tdc_dual_board becomes visible and parentless in sim_2, and the top ambiguity
//  comes straight back.
//
//  This bench has NO CARRY4 and NO MMCM, so it runs in XSim unchanged and in
//  seconds.
// =============================================================================

module tb_tdc_unit;

    // ---- Parameters ---------------------------------------------------------
    parameter W352 = 352;      // new chain: 88 CARRY4
    parameter W256 = 256;      // old chain: 64 CARRY4 (regression)
    parameter OW   = 9;        // 9 bits hold 0..511 -> no clamp at either width
    parameter N_RANDOM_352 = 300;
    parameter N_RANDOM_256 = 200;

    parameter COARSE_BITS = 14;
    parameter FINE_BITS   = 9;
    parameter TMO_CYCLES  = 64;   // small, so the timeout test is quick

    // ---- Clock --------------------------------------------------------------
    reg clk, rst;
    initial begin
        clk = 0;
        forever #2.5 clk = ~clk;   // 200 MHz
    end

    // ---- Scoreboard ---------------------------------------------------------
    integer checks_passed = 0;
    integer checks_failed = 0;
    integer i, k, guard, exp_fine;
    reg     exp_valid;
    reg [W352-1:0] tv;

    // =========================================================================
    //  DUT 1 / 2 : encoder + validator at BOTH widths
    // =========================================================================
    reg  [W352-1:0] vec352;
    wire [OW-1:0]   fine352;
    wire            valid352;

    reg  [W256-1:0] vec256;
    wire [OW-1:0]   fine256;
    wire            valid256;

    ones_counter_encoder_piped #(.INPUT_WIDTH(W352), .OUTPUT_WIDTH(OW))
        enc352 (.clk(clk), .rst(rst), .thermometer_in(vec352), .binary_out(fine352));
    thermometer_validator_piped #(.WIDTH(W352))
        val352 (.clk(clk), .rst(rst), .thermometer_in(vec352), .valid(valid352));

    ones_counter_encoder_piped #(.INPUT_WIDTH(W256), .OUTPUT_WIDTH(OW))
        enc256 (.clk(clk), .rst(rst), .thermometer_in(vec256), .binary_out(fine256));
    thermometer_validator_piped #(.WIDTH(W256))
        val256 (.clk(clk), .rst(rst), .thermometer_in(vec256), .valid(valid256));

    // =========================================================================
    //  DUT 3 : interval_calculator
    // =========================================================================
    reg                    ready_a, ready_b;
    reg [COARSE_BITS-1:0]  coarse_a, coarse_b;
    reg [FINE_BITS-1:0]    fine_a, fine_b;
    reg                    valid_a, valid_b;

    wire [COARSE_BITS-1:0] d_coarse;
    wire [FINE_BITS-1:0]   fine_a_out, fine_b_out;
    wire                   valid_a_out, valid_b_out, timeout, meas_ready;

    interval_calculator #(
        .COARSE_BITS(COARSE_BITS), .FINE_BITS(FINE_BITS), .TIMEOUT_CYCLES(TMO_CYCLES)
    ) ic (
        .clk(clk), .rst(rst),
        .ready_a(ready_a), .coarse_a(coarse_a), .fine_a(fine_a), .valid_a(valid_a),
        .ready_b(ready_b), .coarse_b(coarse_b), .fine_b(fine_b), .valid_b(valid_b),
        .d_coarse(d_coarse), .fine_a_out(fine_a_out), .fine_b_out(fine_b_out),
        .valid_a_out(valid_a_out), .valid_b_out(valid_b_out),
        .timeout(timeout), .meas_ready(meas_ready)
    );

    // =========================================================================
    //  REFERENCE MODELS
    // =========================================================================
    function integer ref_popcount;
        input [511:0] v;
        input integer w;
        integer j, n;
        begin
            n = 0;
            for (j = 0; j < w; j = j + 1) n = n + v[j];
            ref_popcount = n;
        end
    endfunction

    function ref_valid;
        input [511:0] v;
        input integer w;
        integer j, tr;
        begin
            tr = 0;
            for (j = 0; j < w-1; j = j + 1)
                if (v[j] === 1'b1 && v[j+1] === 1'b0) tr = tr + 1;
            ref_valid = (tr <= 1);
        end
    endfunction

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
            end
        end
    endtask

    task silent_check;                 // for the bulk random loops
        input condition;
        begin
            if (condition) checks_passed = checks_passed + 1;
            else           checks_failed = checks_failed + 1;
        end
    endtask

    task check352;
        input [W352-1:0] v;
        begin
            vec352 = v;
            repeat (12) @(posedge clk);   // popcount 7, validator 3 -- both settle then HOLD
            #1;
            exp_fine  = ref_popcount({{(512-W352){1'b0}}, v}, W352);
            exp_valid = ref_valid   ({{(512-W352){1'b0}}, v}, W352);
            if (fine352 !== exp_fine[OW-1:0] || valid352 !== exp_valid)
                $display("      [FAIL 352] fine=%0d (exp %0d)  valid=%b (exp %b)",
                         fine352, exp_fine, valid352, exp_valid);
            silent_check(fine352 === exp_fine[OW-1:0] && valid352 === exp_valid);
        end
    endtask

    task check256;
        input [W256-1:0] v;
        begin
            vec256 = v;
            repeat (12) @(posedge clk);
            #1;
            exp_fine  = ref_popcount({{(512-W256){1'b0}}, v}, W256);
            exp_valid = ref_valid   ({{(512-W256){1'b0}}, v}, W256);
            if (fine256 !== exp_fine[OW-1:0] || valid256 !== exp_valid)
                $display("      [FAIL 256] fine=%0d (exp %0d)  valid=%b (exp %b)",
                         fine256, exp_fine, valid256, exp_valid);
            silent_check(fine256 === exp_fine[OW-1:0] && valid256 === exp_valid);
        end
    endtask

    task check_dcoarse;
        input [COARSE_BITS-1:0] expected;
        input [800-1:0]         desc;
        begin
            if (d_coarse === expected) begin
                $display("  [PASS] %0s : d_coarse = %0d", desc, d_coarse);
                checks_passed = checks_passed + 1;
            end else begin
                $display("  [FAIL] %0s : d_coarse = %0d, expected %0d  @ %0t",
                         desc, d_coarse, expected, $time);
                checks_failed = checks_failed + 1;
            end
        end
    endtask

    task wait_meas;
        begin
            guard = 0;
            while (guard < 1000 && meas_ready !== 1'b1) begin
                @(posedge clk);
                guard = guard + 1;
            end
            @(posedge clk); #1;
        end
    endtask

    task send_pair;
        input [COARSE_BITS-1:0] ca;
        input [COARSE_BITS-1:0] cb;
        input integer           gap;
        integer g;
        begin
            @(posedge clk); #1;
            coarse_a = ca; fine_a = 9'd100; valid_a = 1'b1; ready_a = 1'b1;
            @(posedge clk); #1;
            ready_a = 1'b0;
            for (g = 0; g < gap; g = g + 1) @(posedge clk);
            #1;
            coarse_b = cb; fine_b = 9'd40; valid_b = 1'b1; ready_b = 1'b1;
            @(posedge clk); #1;
            ready_b = 1'b0;
            wait_meas;
        end
    endtask

    // =========================================================================
    //  MAIN STIMULUS
    // =========================================================================
    initial begin
        $dumpfile("tb_tdc_unit.vcd");
        $dumpvars(0, tb_tdc_unit);

        $display("======================================================================");
        $display("  DEEP RTL UNIT TESTS");
        $display("  Everything unreachable from the board pins.");
        $display("======================================================================");

        // ---- Reset ----
        rst = 1'b1;
        vec352 = 0; vec256 = 0;
        ready_a = 1'b0; ready_b = 1'b0;
        coarse_a = 0; coarse_b = 0; fine_a = 0; fine_b = 0;
        valid_a = 1'b1; valid_b = 1'b1;
        repeat (4) @(posedge clk); #1;
        rst = 1'b0;
        repeat (4) @(posedge clk);

        // =================================================================
        $display("\n--- 1. ENCODER + VALIDATOR, %0d taps: corner cases ---", W352);
        check352({W352{1'b0}});
        $display("  [INFO] all zeros              -> fine=%0d valid=%b", fine352, valid352);
        check352({W352{1'b1}});
        $display("  [INFO] all ones (chain full)  -> fine=%0d valid=%b  (NOT clamped)",
                 fine352, valid352);
        assert_check(fine352 === W352[OW-1:0],
                     "a FULL 352-tap chain reports 352, not a clamped code");
        $display("      (the old 8-bit build clamped 256 -> 255, the SAME code a 255-tap");
        $display("       chain produced. `fine=255` meant two different things. Gone now.)");

        $display("\n--- 2. Clean thermometer codes: fine must equal k ---");
        for (k = 1; k < W352; k = k + 40) begin
            tv = {W352{1'b0}};
            for (i = 0; i < k; i = i + 1) tv[i] = 1'b1;
            check352(tv);
        end
        assert_check(checks_failed == 0, "k = 1, 41, 81 ... 321 all decode exactly");

        $display("\n--- 3. Multi-transition codes must be flagged INVALID ---");
        tv = {W352{1'b0}};
        for (i = 0; i < 100; i = i + 1) tv[i] = 1'b1;
        tv[150] = 1'b1;                                  // isolated 1 far away
        check352(tv);
        assert_check(valid352 === 1'b0, "two separate runs -> 2 transitions -> REJECTED");
        tv[151] = 1'b1; tv[152] = 1'b1;
        check352(tv);
        assert_check(valid352 === 1'b0, "three separate blocks -> REJECTED");

        $display("\n--- 4. %0d random vectors at %0d taps ---", N_RANDOM_352, W352);
        for (i = 0; i < N_RANDOM_352; i = i + 1) begin
            tv = {$random, $random, $random, $random, $random,
                  $random, $random, $random, $random, $random, $random};
            check352(tv);
        end
        $display("  [INFO] %0d random vectors compared against the reference", N_RANDOM_352);

        $display("\n--- 5. %0d random vectors at %0d taps (old width, regression) ---",
                 N_RANDOM_256, W256);
        check256({W256{1'b0}});
        check256({W256{1'b1}});
        assert_check(fine256 === W256[OW-1:0], "a FULL 256-tap chain reports 256");
        for (i = 0; i < N_RANDOM_256; i = i + 1) begin
            check256({$random, $random, $random, $random,
                      $random, $random, $random, $random});
        end
        $display("  [INFO] %0d random vectors compared against the reference", N_RANDOM_256);

        // =================================================================
        $display("\n--- 6. INTERVAL CALCULATOR: normal case, no wrap ---");
        send_pair(14'd5000, 14'd5004, 3);  check_dcoarse(14'd4, "5000 -> 5004");
        send_pair(14'd0,    14'd1,    2);  check_dcoarse(14'd1, "   0 ->    1");
        send_pair(14'd100,  14'd100,  2);  check_dcoarse(14'd0, " 100 ->  100 (sub-clock interval)");

        $display("\n--- 7. COARSE ROLLOVER  (unreachable from the board pins) ---");
        $display("    On hardware the counter wraps every 81.92 us. If this arithmetic");
        $display("    is wrong, the lab sees RARE, RANDOM, WILDLY-WRONG intervals.");
        send_pair(14'd16380, 14'd4,     3); check_dcoarse(14'd8,     "16380 -> 4      (wrapped)");
        send_pair(14'd16383, 14'd0,     3); check_dcoarse(14'd1,     "16383 -> 0      (wrap by 1)");
        send_pair(14'd16383, 14'd16383, 2); check_dcoarse(14'd0,     "16383 -> 16383");
        send_pair(14'd16000, 14'd100,   3); check_dcoarse(14'd484,   "16000 -> 100    (wrapped)");
        send_pair(14'd8192,  14'd8191,  3); check_dcoarse(14'd16383, " 8192 -> 8191   (max representable)");

        $display("\n--- 8. Simultaneous ready (calibration build: one source, both chains) ---");
        @(posedge clk); #1;
        coarse_a = 14'd777; fine_a = 9'd60; valid_a = 1'b1; ready_a = 1'b1;
        coarse_b = 14'd777; fine_b = 9'd58; valid_b = 1'b1; ready_b = 1'b1;
        @(posedge clk); #1;
        ready_a = 1'b0; ready_b = 1'b0;
        wait_meas;
        check_dcoarse(14'd0, "both ready on the SAME cycle");
        assert_check(fine_a_out === 9'd60 && fine_b_out === 9'd58,
                     "both fine values latched from the simultaneous pair");

        $display("\n--- 9. Timeout and recovery ---");
        @(posedge clk); #1;
        coarse_a = 14'd1234; fine_a = 9'd77; valid_a = 1'b1; ready_a = 1'b1;
        @(posedge clk); #1;
        ready_a = 1'b0;
        wait_meas;
        assert_check(timeout === 1'b1,     "no STOP -> timeout flag asserted");
        assert_check(valid_b_out === 1'b0, "valid_b forced 0 -> host filters the sample");
        send_pair(14'd200, 14'd207, 3);
        check_dcoarse(14'd7, "a normal pair right after a timeout");
        assert_check(timeout === 1'b0, "timeout flag cleared");

        // ---- Summary ----
        $display("\n======================================================================");
        $display("  VERIFICATION COMPLETE");
        $display("  Checks passed : %0d", checks_passed);
        $display("  Checks failed : %0d", checks_failed);
        $display("======================================================================");
        if (checks_failed == 0)
            $display("  >>> ENCODER + VALIDATOR PROVEN AT 256 AND 352 TAPS <<<");
        else
            $display("  >>> %0d CHECK(S) FAILED <<<", checks_failed);
        $display("======================================================================");

        #50; $finish;
    end

    // ---- Global watchdog ----------------------------------------------------
    initial begin
        #100_000_000;
        $display("  >>> GLOBAL TIMEOUT -- simulation did not complete <<<");
        $finish;
    end

endmodule