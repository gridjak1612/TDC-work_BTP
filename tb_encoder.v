`timescale 1ns/1ps
//=============================================================================
// tb_encoder_equiv.v
//
// Proves the WIDTH-GENERIC ones_counter_encoder_piped and
// thermometer_validator_piped are BIT-IDENTICAL to a behavioural reference,
// against random vectors, at BOTH the old width (256) and the new one (352).
//
// This is the same discipline used when the modules were first pipelined:
// "every pipelined module was proven bit-identical to its combinational
//  predecessor against thousands of random vectors." Rewriting them for a new
// chain length invalidates that proof, so it has to be re-established.
//
// Reference popcount : count the 1s.
// Reference validator: count 1->0 transitions; valid = (count <= 1).
//=============================================================================
module tb_encoder_equiv;

    localparam integer W352 = 352;
    localparam integer O352 = 9;
    localparam integer W256 = 256;
    localparam integer O256 = 9;      // 9 bits so 256 is representable (no clamp)

    reg clk = 0;
    reg rst = 1;
    always #2.5 clk = ~clk;

    // ---------------- 352-tap instances ----------------
    reg  [W352-1:0] vec352;
    wire [O352-1:0] fine352;
    wire            valid352;

    ones_counter_encoder_piped #(.INPUT_WIDTH(W352), .OUTPUT_WIDTH(O352))
        enc352 (.clk(clk), .rst(rst), .thermometer_in(vec352), .binary_out(fine352));

    thermometer_validator_piped #(.WIDTH(W352))
        val352 (.clk(clk), .rst(rst), .thermometer_in(vec352), .valid(valid352));

    // ---------------- 256-tap instances ----------------
    reg  [W256-1:0] vec256;
    wire [O256-1:0] fine256;
    wire            valid256;

    ones_counter_encoder_piped #(.INPUT_WIDTH(W256), .OUTPUT_WIDTH(O256))
        enc256 (.clk(clk), .rst(rst), .thermometer_in(vec256), .binary_out(fine256));

    thermometer_validator_piped #(.WIDTH(W256))
        val256 (.clk(clk), .rst(rst), .thermometer_in(vec256), .valid(valid256));

    //-------------------------------------------------------------------------
    // Reference models
    //-------------------------------------------------------------------------
    function integer ref_popcount(input [511:0] v, input integer w);
        integer i, n;
        begin
            n = 0;
            for (i = 0; i < w; i = i + 1) n = n + v[i];
            ref_popcount = n;
        end
    endfunction

    function ref_valid(input [511:0] v, input integer w);
        integer i, tr;
        begin
            tr = 0;
            for (i = 0; i < w-1; i = i + 1)
                if (v[i] === 1'b1 && v[i+1] === 1'b0) tr = tr + 1;
            ref_valid = (tr <= 1);
        end
    endfunction

    integer pass = 0, fail = 0;
    integer i, k, exp_f;
    reg     exp_v;

    // Apply a vector, wait out the pipeline (7 for popcount, 3 for validator),
    // then compare. Both settle and HOLD, so an over-long wait is harmless.
    task check352(input [W352-1:0] v, input [200*8-1:0] name);
        begin
            vec352 = v;
            repeat (12) @(posedge clk);
            #1;
            exp_f = ref_popcount({{(512-W352){1'b0}}, v}, W352);
            exp_v = ref_valid   ({{(512-W352){1'b0}}, v}, W352);
            if (fine352 === exp_f[O352-1:0] && valid352 === exp_v) pass = pass + 1;
            else begin
                fail = fail + 1;
                $display("  [FAIL 352] %0s : fine=%0d (exp %0d)  valid=%b (exp %b)",
                         name, fine352, exp_f, valid352, exp_v);
            end
        end
    endtask

    task check256(input [W256-1:0] v, input [200*8-1:0] name);
        begin
            vec256 = v;
            repeat (12) @(posedge clk);
            #1;
            exp_f = ref_popcount({{(512-W256){1'b0}}, v}, W256);
            exp_v = ref_valid   ({{(512-W256){1'b0}}, v}, W256);
            if (fine256 === exp_f[O256-1:0] && valid256 === exp_v) pass = pass + 1;
            else begin
                fail = fail + 1;
                $display("  [FAIL 256] %0s : fine=%0d (exp %0d)  valid=%b (exp %b)",
                         name, fine256, exp_f, valid256, exp_v);
            end
        end
    endtask

    reg [W352-1:0] tv;

    initial begin
        $display("");
        $display("=====================================================================");
        $display("  EQUIVALENCE: generic encoder + validator vs behavioural reference");
        $display("=====================================================================");
        vec352 = 0; vec256 = 0;
        repeat (4) @(posedge clk); rst = 0; repeat (4) @(posedge clk);

        // ---- 352: corner cases ----
        $display("\n--- 352 taps: corners ---");
        check352({W352{1'b0}}, "all zeros            -> fine 0,   valid 1");
        check352({W352{1'b1}}, "all ones (chain full)-> fine 352, valid 1");
        for (k = 1; k < W352; k = k + 40) begin
            tv = {W352{1'b0}};
            for (i = 0; i < k; i = i + 1) tv[i] = 1'b1;
            check352(tv, "clean thermometer");
        end
        $display("  clean thermometer codes at k = 1,41,...,321  -> fine must equal k");

        // ---- 352: bubbles (must be flagged INVALID) ----
        $display("\n--- 352 taps: multi-transition codes must be flagged INVALID ---");
        tv = {W352{1'b0}};
        for (i = 0; i < 100; i = i + 1) tv[i] = 1'b1;
        tv[150] = 1'b1;                                  // isolated 1 far away
        check352(tv, "two separate runs -> 2 transitions -> invalid");
        tv[151] = 1'b1; tv[152] = 1'b1;
        check352(tv, "three separate blocks");

        // ---- 352: random ----
        $display("\n--- 352 taps: 500 random vectors ---");
        for (i = 0; i < 500; i = i + 1) begin
            tv = {$random, $random, $random, $random, $random,
                  $random, $random, $random, $random, $random, $random};
            check352(tv, "random");
        end

        // ---- 256: regression (the old width must still work) ----
        $display("\n--- 256 taps: corners + 300 random vectors (regression) ---");
        check256({W256{1'b0}}, "all zeros");
        check256({W256{1'b1}}, "all ones -> fine 256");
        for (i = 0; i < 300; i = i + 1) begin
            check256({$random, $random, $random, $random,
                      $random, $random, $random, $random}, "random");
        end

        $display("");
        $display("=====================================================================");
        $display("  PASS = %0d   FAIL = %0d", pass, fail);
        if (fail == 0)
            $display("  >>> generic encoder + validator PROVEN at 256 AND 352 taps <<<");
        $display("=====================================================================");
        $display("");
        $finish;
    end

endmodule