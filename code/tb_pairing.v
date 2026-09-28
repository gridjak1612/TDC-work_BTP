`timescale 1ns/1ps
// tb_pairing -- interval_calculator must pair A and B in either order.
module tb_pairing;
    reg clk = 0, rst = 1;
    always #2.5 clk = ~clk;

    reg        ra = 0, rb = 0, va = 1, vb = 1;
    reg [13:0] ca = 0, cb = 0;
    reg [8:0]  fa = 0, fb = 0;
    wire [13:0] dc;  wire [8:0] foa, fob;  wire voa, vob, to, mr;

    interval_calculator #(.COARSE_BITS(14), .FINE_BITS(9), .TIMEOUT_CYCLES(50)) dut (
        .clk(clk), .rst(rst),
        .ready_a(ra), .coarse_a(ca), .fine_a(fa), .valid_a(va),
        .ready_b(rb), .coarse_b(cb), .fine_b(fb), .valid_b(vb),
        .d_coarse(dc), .fine_a_out(foa), .fine_b_out(fob),
        .valid_a_out(voa), .valid_b_out(vob), .timeout(to), .meas_ready(mr));

    integer errs = 0, got = 0, ncases = 0;
    reg [13:0] exp_dc;  reg exp_to;  reg [8:0] exp_fa, exp_fb;

    always @(posedge clk) if (mr) begin
        got = got + 1;
        if (dc !== exp_dc || to !== exp_to || foa !== exp_fa || fob !== exp_fb) begin
            errs = errs + 1;
            $display("MISMATCH: dc=%0d (exp %0d) to=%b (exp %b) fa=%0d (exp %0d) fb=%0d (exp %0d)",
                     dc, exp_dc, to, exp_to, foa, exp_fa, fob, exp_fb);
        end
    end

    task run_case(input integer lag, input integer ha, input integer hb);
        integer k, ta, tb, g0, last;
        begin
            ta = (lag >= 0) ? 0 : -lag;
            tb = (lag >= 0) ? lag : 0;
            last = (ta > tb) ? ta : tb;
            exp_fa = ha ? 9'd111 : 9'd0;
            exp_fb = hb ? 9'd222 : 9'd0;
            exp_to = !(ha && hb);
            exp_dc = (ha && hb) ? lag : 0;       // coarse_b - coarse_a, mod 2^14
            g0 = got;  ncases = ncases + 1;
            for (k = 0; k <= last; k = k + 1) begin
                @(negedge clk);
                ra = ha && (k == ta); ca = 14'd100;       fa = 9'd111;
                rb = hb && (k == tb); cb = 14'd100 + lag; fb = 9'd222;
            end
            @(negedge clk); ra = 0; rb = 0;
            repeat (80) @(negedge clk);          // longer than the 50-cycle timeout
            if (got - g0 != 1) begin
                errs = errs + 1;
                $display("RECORD COUNT lag=%0d ha=%0d hb=%0d: %0d records", lag, ha, hb, got - g0);
            end
        end
    endtask

    initial begin
        repeat (3) @(negedge clk); rst = 0;
        run_case( 0, 1, 1);  run_case( 1, 1, 1);  run_case( 2, 1, 1);  run_case( 3, 1, 1);
        run_case(-1, 1, 1);  run_case(-2, 1, 1);  run_case(-3, 1, 1);
        run_case( 0, 1, 0);  run_case( 0, 0, 1);
        if (errs == 0) $display("PASS: %0d cases, A-first, B-first, A-only, B-only all paired correctly", ncases);
        else           $display("FAIL: %0d errors in %0d cases", errs, ncases);
        $finish;
    end
endmodule