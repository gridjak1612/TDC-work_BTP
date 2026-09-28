`timescale 1ns/1ps
// tb_dual_snap -- unit test of the step-4 selection rule.
module tb_dual_snap;
    reg clk = 0, rst = 1, cap = 0;
    always #2.5 clk = ~clk;

    reg  [15:0] t = 0;                       // edge id
    wire        F = t[3];                    // fake "chain full", toggles every 8 edges
    wire [16:0] din = {F, t};                // top bit = last tap
    always @(posedge clk) t <= t + 1'b1;

    wire [16:0] cap_taps;  wire [15:0] cap_coarse;  wire full;
    snapshot_pipeline #(.WIDTH(17), .DEPTH(4)) taps (
        .clk(clk), .rst(rst), .capture_enable(cap), .use_prev(full),
        .din(din), .captured(cap_taps), .top_now(full));
    snapshot_pipeline #(.WIDTH(16), .DEPTH(4)) coarse (
        .clk(clk), .rst(rst), .capture_enable(cap), .use_prev(full),
        .din(t), .captured(cap_coarse), .top_now());

    reg [16:0] exp_def, exp_prev, want;
    integer k, errs = 0, n_prev = 0, n_def = 0;
    initial begin
        repeat (3) @(posedge clk); rst = 0;
        repeat (10) @(posedge clk);
        for (k = 0; k < 200; k = k + 1) begin
            repeat (3 + (k % 5)) @(posedge clk);
            @(negedge clk);
            cap = 1; exp_def = taps.pipe[2]; exp_prev = taps.pipe[3];
            @(posedge clk); #1 cap = 0;
            want = exp_def[16] ? exp_prev : exp_def;
            if (exp_def[16]) n_prev = n_prev + 1; else n_def = n_def + 1;
            if (cap_taps !== want || cap_coarse !== want[15:0]) begin
                errs = errs + 1;
                if (errs <= 5) $display("MISMATCH k=%0d def=%h prev=%h got taps=%h coarse=%h",
                                        k, exp_def, exp_prev, cap_taps, cap_coarse);
            end
            if (exp_def[15:0] - exp_prev[15:0] != 16'd1) begin
                errs = errs + 1; $display("history not consecutive at k=%0d", k);
            end
        end
        if (errs == 0 && n_prev > 0 && n_def > 0)
            $display("PASS: 200 captures, %0d took previous edge, %0d default; taps and coarse always the same edge",
                     n_prev, n_def);
        else
            $display("FAIL: errs=%0d n_prev=%0d n_def=%0d", errs, n_prev, n_def);
        $finish;
    end
endmodule