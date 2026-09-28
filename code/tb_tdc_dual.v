`timescale 1ns/1ps
//=============================================================================
// tb_tdc_dual.v
//
// Drives channel A (START) and channel B (STOP) with a KNOWN separation and
// checks that the host-side reconstruction
//
//     interval = d_coarse * 5.000 ns  -  (fine_b - fine_a) * tau
//
// recovers it. The separation and the absolute phase are both swept, so the
// coarse rollover, the sub-clock interpolation, and the pairing FSM all get
// exercised.
//
// carry4_mock = 20 ps/tap, so tau = 0.020 ns and 256 taps span 5.12 ns.
//=============================================================================
module tb_tdc_dual;

    localparam real TCLK = 5.000;    // ns, clk200 period
    localparam real TAU  = 0.020;    // ns per tap (matches carry4_mock)

    reg clk100 = 0;
    reg rst    = 1;
    reg ev_a   = 0;
    reg ev_b   = 0;
    reg clr    = 0;

    wire [13:0] d_coarse;
    wire [8:0]  fine_a, fine_b;
    wire        valid_a, valid_b, timeout, meas_ready;
    wire        done_a, done_b, clk200, mmcm_locked;

    tdc_dual_top uut (
        .clk100(clk100), .rst(rst),
        .event_a(ev_a), .event_b(ev_b), .clear_status(clr),
        .d_coarse(d_coarse), .fine_a(fine_a), .fine_b(fine_b),
        .valid_a(valid_a), .valid_b(valid_b),
        .timeout(timeout), .meas_ready(meas_ready),
        .done_a(done_a), .done_b(done_b),
        .clk200(clk200), .mmcm_locked(mmcm_locked)
    );

    always #5 clk100 = ~clk100;   // 100 MHz

    integer pass = 0, fail = 0;
    real    err_max = 0.0;

    // ---- capture the record when meas_ready pulses ----
    reg        got;
    reg [13:0] r_dc;
    reg [8:0]  r_fa, r_fb;
    reg        r_va, r_vb, r_tmo;

    always @(posedge clk200) begin
        if (meas_ready) begin
            r_dc  <= d_coarse; r_fa <= fine_a; r_fb <= fine_b;
            r_va  <= valid_a;  r_vb <= valid_b; r_tmo <= timeout;
            got   <= 1'b1;
        end
    end

    task rearm;
        begin
            clr = 1'b1; repeat (6) @(posedge clk200);
            clr = 1'b0; repeat (6) @(posedge clk200);
        end
    endtask

    // Fire START, wait `sep` ns, fire STOP. `phase` shifts the whole pair
    // relative to the clock so every sub-clock alignment gets tested.
    task do_pair(input real phase, input real sep);
        real    meas, err;
        integer ifa, ifb;          // *** fine difference is SIGNED ***
        begin
            rearm;
            got = 1'b0;

            @(posedge clk200);
            #(phase);
            ev_a = 1'b1;              // START launches chain A
            #(sep);
            ev_b = 1'b1;              // STOP  launches chain B
            #40;
            ev_a = 1'b0; ev_b = 1'b0; // release both

            // wait for the pairing FSM + both fine pipelines
            wait (got == 1'b1);
            repeat (2) @(posedge clk200);

            // ---- host-side reconstruction ----
            // CRITICAL: widen fine_a/fine_b to signed integers BEFORE
            // subtracting. In 8-bit unsigned, (85 - 185) wraps to 156 and the
            // interval comes out exactly one chain-span wrong.
            ifa  = r_fa;
            ifb  = r_fb;
            meas = r_dc * TCLK - (ifb - ifa) * TAU;
            err  = meas - sep;
            if (err < 0) err = -err;
            if (err > err_max) err_max = err;

            if (err < 0.030 && r_va && r_vb && !r_tmo) begin
                pass = pass + 1;
                $display("  phase=%4.2f sep=%6.2f | d_coarse=%3d fine_a=%3d fine_b=%3d | measured=%7.3f ns  err=%+6.3f ns  PASS",
                         phase, sep, r_dc, r_fa, r_fb, meas, meas - sep);
            end else begin
                fail = fail + 1;
                $display("  phase=%4.2f sep=%6.2f | d_coarse=%3d fine_a=%3d fine_b=%3d | measured=%7.3f ns  err=%+6.3f ns  *** FAIL (va=%b vb=%b tmo=%b)",
                         phase, sep, r_dc, r_fa, r_fb, meas, meas - sep, r_va, r_vb, r_tmo);
            end
        end
    endtask

    integer i;

    initial begin
        $display("");
        $display("=========================================================================");
        $display("  TWO-CHANNEL INTERVAL TDC   (tau = %0.0f ps, T_clk = %0.3f ns)", TAU*1000, TCLK);
        $display("  Reconstruction: interval = d_coarse x 5.000 ns - (fine_b - fine_a) x tau");
        $display("=========================================================================");

        repeat (10) @(posedge clk100);
        rst = 0;
        wait (mmcm_locked);
        repeat (20) @(posedge clk200);

        $display("\n--- Sweep the STOP separation at a fixed phase ---");
        for (i = 0; i < 8; i = i + 1)
            do_pair(1.30, 12.00 + i * 3.70);      // 12.0 .. 37.9 ns, non-integer

        $display("\n--- Sweep the phase at a fixed separation (21.00 ns, a LiDAR-ish 3.15 m) ---");
        for (i = 0; i < 10; i = i + 1)
            do_pair(0.50 * i, 21.00);

        $display("\n--- Sub-clock separations (interval < one clock period) ---");
        for (i = 0; i < 5; i = i + 1)
            do_pair(2.10, 1.00 + i * 0.90);

        $display("\n=========================================================================");
        $display("  PASS = %0d    FAIL = %0d    worst error = %.3f ns (%.0f ps)",
                 pass, fail, err_max, err_max*1000);
        $display("=========================================================================\n");
        $finish;
    end

endmodule