`timescale 1ns/1ps
//=============================================================================
// tb_interval_calculator.v
//
// UNIT test of the pairing FSM and -- critically -- the COARSE ROLLOVER claim.
//
// tdc_dual_top asserts that
//        d_coarse = coarse_b - coarse_a     (14-bit truncating subtraction)
// handles a counter rollover between START and STOP automatically, because
// N-bit two's-complement subtraction IS modulo-2^N arithmetic.
//
// That claim is load-bearing for EVERY interval measurement, and the system
// testbench never exercises it (the counter does not wrap in 4 us of sim).
// So prove it here, directly, including the exact boundary cases.
//
// Also covers: simultaneous ready (tie mode), the no-STOP timeout, and
// back-to-back measurements.
//=============================================================================
module tb_interval_calculator;

    localparam integer CB = 14;
    localparam integer FB = 8;
    localparam integer TMO = 64;          // small timeout so the test is quick

    reg clk = 0, rst = 1;
    reg          ready_a = 0, ready_b = 0;
    reg [CB-1:0] coarse_a = 0, coarse_b = 0;
    reg [FB-1:0] fine_a = 0,   fine_b = 0;
    reg          valid_a = 1,  valid_b = 1;

    wire [CB-1:0] d_coarse;
    wire [FB-1:0] fine_a_o, fine_b_o;
    wire          valid_a_o, valid_b_o, timeout, meas_ready;

    interval_calculator #(
        .COARSE_BITS(CB), .FINE_BITS(FB), .TIMEOUT_CYCLES(TMO)
    ) uut (
        .clk(clk), .rst(rst),
        .ready_a(ready_a), .coarse_a(coarse_a), .fine_a(fine_a), .valid_a(valid_a),
        .ready_b(ready_b), .coarse_b(coarse_b), .fine_b(fine_b), .valid_b(valid_b),
        .d_coarse(d_coarse), .fine_a_out(fine_a_o), .fine_b_out(fine_b_o),
        .valid_a_out(valid_a_o), .valid_b_out(valid_b_o),
        .timeout(timeout), .meas_ready(meas_ready)
    );

    always #2.5 clk = ~clk;   // 200 MHz

    integer pass = 0, fail = 0;

    task chk(input [CB-1:0] got, input [CB-1:0] exp, input [200*8-1:0] what);
        begin
            if (got === exp) begin
                pass = pass + 1;
                $display("  [PASS] %0s : d_coarse = %0d", what, got);
            end else begin
                fail = fail + 1;
                $display("  [FAIL] %0s : d_coarse = %0d, expected %0d", what, got, exp);
            end
        end
    endtask

    // Fire A at ca, then B at cb, `gap` cycles later. Returns when meas_ready.
    task pair(input [CB-1:0] ca, input [CB-1:0] cb, input integer gap);
        integer g;
        begin
            @(posedge clk); #1;
            coarse_a = ca; fine_a = 8'd100; valid_a = 1'b1; ready_a = 1'b1;
            @(posedge clk); #1;
            ready_a = 1'b0;
            for (g = 0; g < gap; g = g + 1) @(posedge clk);
            #1;
            coarse_b = cb; fine_b = 8'd40; valid_b = 1'b1; ready_b = 1'b1;
            @(posedge clk); #1;
            ready_b = 1'b0;
            wait (meas_ready === 1'b1);
            @(posedge clk); #1;   // settle
        end
    endtask

    initial begin
        $display("");
        $display("======================================================================");
        $display("  UNIT TEST: interval_calculator -- COARSE ROLLOVER + PAIRING");
        $display("======================================================================");
        repeat (4) @(posedge clk); #1; rst = 0; repeat (2) @(posedge clk);

        $display("\n--- Normal case, no wrap ---");
        pair(14'd5000, 14'd5004, 3);   chk(d_coarse, 14'd4,     "5000 -> 5004");
        pair(14'd0,    14'd1,    2);   chk(d_coarse, 14'd1,     "   0 ->    1");
        pair(14'd100,  14'd100,  2);   chk(d_coarse, 14'd0,     " 100 ->  100 (sub-clock interval)");

        $display("\n--- COARSE ROLLOVER (this is the untested claim) ---");
        // counter wraps 16383 -> 0 between START and STOP
        pair(14'd16380, 14'd4,    3);  chk(d_coarse, 14'd8,     "16380 -> 4   (wrapped)");
        pair(14'd16383, 14'd0,    3);  chk(d_coarse, 14'd1,     "16383 -> 0   (wrap by 1)");
        pair(14'd16383, 14'd16383,2);  chk(d_coarse, 14'd0,     "16383 -> 16383");
        pair(14'd16000, 14'd100,  3);  chk(d_coarse, 14'd484,   "16000 -> 100 (wrapped)");
        pair(14'd8192,  14'd8191, 3);  chk(d_coarse, 14'd16383, " 8192 -> 8191 (max representable)");

        $display("\n--- Simultaneous ready (TIE mode: one source, both chains) ---");
        @(posedge clk); #1;
        coarse_a = 14'd777; fine_a = 8'd60; valid_a = 1'b1; ready_a = 1'b1;
        coarse_b = 14'd777; fine_b = 8'd58; valid_b = 1'b1; ready_b = 1'b1;
        @(posedge clk); #1;
        ready_a = 1'b0; ready_b = 1'b0;
        wait (meas_ready === 1'b1); @(posedge clk); #1;
        chk(d_coarse, 14'd0, "both ready same cycle");
        if (fine_a_o == 8'd60 && fine_b_o == 8'd58) begin
            pass = pass + 1; $display("  [PASS] both fine values latched (a=%0d b=%0d)", fine_a_o, fine_b_o);
        end else begin
            fail = fail + 1; $display("  [FAIL] fine latch: a=%0d b=%0d", fine_a_o, fine_b_o);
        end

        $display("\n--- TIMEOUT: START fires, STOP never comes ---");
        @(posedge clk); #1;
        coarse_a = 14'd1234; fine_a = 8'd77; valid_a = 1'b1; ready_a = 1'b1;
        @(posedge clk); #1; ready_a = 1'b0;
        wait (meas_ready === 1'b1); @(posedge clk); #1;
        if (timeout === 1'b1 && valid_b_o === 1'b0) begin
            pass = pass + 1;
            $display("  [PASS] timeout asserted, valid_b forced 0 -> host filters the sample");
        end else begin
            fail = fail + 1;
            $display("  [FAIL] timeout=%b valid_b=%b (expected 1, 0)", timeout, valid_b_o);
        end

        $display("\n--- Recovery: a normal pair right after a timeout ---");
        pair(14'd200, 14'd207, 3);
        chk(d_coarse, 14'd7, "200 -> 207 after timeout");
        if (timeout === 1'b0) begin
            pass = pass + 1; $display("  [PASS] timeout flag cleared");
        end else begin
            fail = fail + 1; $display("  [FAIL] timeout flag stuck high");
        end

        $display("\n======================================================================");
        $display("  PASS = %0d   FAIL = %0d", pass, fail);
        if (fail == 0) $display("  >>> coarse rollover arithmetic PROVEN <<<");
        else           $display("  >>> %0d FAILURE(S) <<<", fail);
        $display("======================================================================\n");
        $finish;
    end

endmodule