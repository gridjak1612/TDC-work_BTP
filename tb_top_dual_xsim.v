`timescale 1ns/1ps
//=============================================================================
// tb_tdc_dual_xsim.v   --   RUN THIS ONE IN VIVADO / XSim
//=============================================================================
//
// 1. WHY A SEPARATE TESTBENCH FOR XSim
// ------------------------------------
// In Vivado behavioural simulation the delay line is built from the REAL Xilinx
// UNISIM CARRY4 primitive, and that model has ZERO PROPAGATION DELAY. When the
// event fires, all 256 taps flip in the same delta cycle. There is no time
// ruler, so `fine` can ONLY ever read 255 (or 0).
//
// That is a property of the primitive's simulation model, not a design fault,
// and NO testbench can work around it. The fine interpolator is STRUCTURALLY
// UNTESTABLE in Vivado behavioural simulation.
//
//   fine path -> Icarus + carry4_mock.v (20 ps/tap)   [tb_tdc_dual.v]
//             -> post-implementation TIMING sim (SDF back-annotation)
//             -> hardware
//
// What XSim CAN prove, on the real primitives: both channels capture off the
// SHARED coarse counter, the pairing FSM matches a START to its STOP, d_coarse
// is correct, and the timeout / re-arm logic works. With the fine term dead the
// reconstruction degenerates to  interval ~= d_coarse * 5.000 ns,  which must
// land within ONE CLOCK PERIOD of the truth. That is the pass criterion.
//
//
// 2. WHY THIS USED TO TAKE MINUTES (and no longer does)
// -----------------------------------------------------
// interval_calculator's default TIMEOUT_CYCLES is 2^14 = 16384 cycles = 81.92 us
// -- correct for hardware (one full coarse rollover), catastrophic for sim. The
// single no-STOP test was ~95% of the entire runtime.
//
// Here TIMEOUT_CYCLES is overridden to 256 (1.28 us). Same logic, same coverage,
// ~20x faster. Simulated time drops from 85.6 us to roughly 4 us.
//
//
// 3. NO UNBOUNDED WAITS
// ---------------------
// Every wait is a bounded polling loop, so a bug FAILS the test instead of
// hanging the simulator. Also removes the blocking/non-blocking race on the
// shared "got" flag that the previous version had.
//
// 4. No "%+" printf flags anywhere. XSim does not support them (Icarus does),
//    and one silently shifts every argument after it.
//=============================================================================
module tb_tdc_dual_xsim;

    localparam real    TCLK     = 5.000;
    localparam integer SIM_TMO  = 256;      // pairing timeout, clk200 cycles
    localparam integer MAX_WAIT = 5000;     // guard on every polling loop

    reg clk100 = 0;
    reg rst    = 1;
    reg ev_a   = 0;
    reg ev_b   = 0;
    reg clr    = 0;

    wire [13:0] d_coarse;
    wire [8:0]  fine_a, fine_b;
    wire        valid_a, valid_b, timeout, meas_ready;
    wire        done_a, done_b, clk200, mmcm_locked;

    tdc_dual_top #(
        .TIMEOUT_CYCLES(SIM_TMO)         // <-- the speed fix
    ) uut (
        .clk100(clk100), .rst(rst),
        .event_a(ev_a), .event_b(ev_b), .clear_status(clr),
        .d_coarse(d_coarse), .fine_a(fine_a), .fine_b(fine_b),
        .valid_a(valid_a), .valid_b(valid_b),
        .timeout(timeout), .meas_ready(meas_ready),
        .done_a(done_a), .done_b(done_b),
        .clk200(clk200), .mmcm_locked(mmcm_locked)
    );

    always #5 clk100 = ~clk100;   // 100 MHz

    integer pass      = 0;
    integer fail      = 0;
    integer n_railed  = 0;
    integer n_meas    = 0;
    real    worst_err = 0.0;

    reg [13:0] r_dc;
    reg [8:0]  r_fa, r_fb;
    reg        r_va, r_vb, r_tmo, r_ok;

    // Bounded wait for the measurement strobe. Never hangs.
    task wait_meas;
        integer guard;
        begin
            r_ok  = 1'b0;
            guard = 0;
            while (guard < MAX_WAIT && r_ok == 1'b0) begin
                @(posedge clk200);
                #1;
                if (meas_ready === 1'b1) begin
                    r_dc = d_coarse; r_fa = fine_a;  r_fb  = fine_b;
                    r_va = valid_a;  r_vb = valid_b; r_tmo = timeout;
                    r_ok = 1'b1;
                end
                guard = guard + 1;
            end
            if (r_ok == 1'b0)
                $display("  *** wait_meas gave up after %0d cycles ***", MAX_WAIT);
        end
    endtask

    task rearm;
        integer guard;
        begin
            clr = 1'b1; repeat (6) @(posedge clk200);
            clr = 1'b0; repeat (6) @(posedge clk200);
            guard = 0;
            while (guard < MAX_WAIT && (done_a === 1'b1 || done_b === 1'b1)) begin
                @(posedge clk200);
                guard = guard + 1;
            end
        end
    endtask

    task do_pair(input real phase, input real sep);
        real coarse_only, err;
        begin
            rearm;

            @(posedge clk200);
            #(phase);
            ev_a = 1'b1;
            #(sep);
            ev_b = 1'b1;
            #40;
            ev_a = 1'b0;
            ev_b = 1'b0;

            wait_meas;

            if (r_ok == 1'b0) begin
                fail = fail + 1;
                $display("  [FAIL] sep=%0.2f ns : no measurement produced", sep);
            end else begin
                n_meas = n_meas + 1;

                // A chain carries NO phase information at either rail:
                //   255 = the carry wave ran off the end of the chain
                //     0 = it never started
                // Under a zero-delay CARRY4, every sample lands on a rail.
                if ((r_fa == 9'd352 || r_fa == 9'd0) &&
                    (r_fb == 9'd352 || r_fb == 9'd0))
                    n_railed = n_railed + 1;

                coarse_only = r_dc * TCLK;
                err = coarse_only - sep;
                if (err < 0) err = -err;
                if (err > worst_err) worst_err = err;

                if (err < TCLK && r_va === 1'b1 && r_vb === 1'b1 && r_tmo === 1'b0) begin
                    pass = pass + 1;
                    $display("  [PASS] sep=%0.2f ns | d_coarse=%0d -> %0.3f ns | err=%0.3f ns (< 5 ns) | fine_a=%0d fine_b=%0d",
                             sep, r_dc, coarse_only, err, r_fa, r_fb);
                end else begin
                    fail = fail + 1;
                    $display("  [FAIL] sep=%0.2f ns | d_coarse=%0d -> %0.3f ns | err=%0.3f ns | fine_a=%0d fine_b=%0d va=%b vb=%b tmo=%b",
                             sep, r_dc, coarse_only, err, r_fa, r_fb, r_va, r_vb, r_tmo);
                end
            end
        end
    endtask

    integer i;

    initial begin
        $display("");
        $display("=====================================================================");
        $display("  TWO-CHANNEL INTERVAL TDC  --  XSim / behavioural");
        $display("");
        $display("  XSim uses the real UNISIM CARRY4, which has ZERO delay, so the");
        $display("  fine interpolator CANNOT be tested here and will read 352 or 0.");
        $display("  That is expected. This run checks the COARSE path: both channels");
        $display("  capturing off the shared counter, the pairing FSM, and d_coarse.");
        $display("");
        $display("  PASS: |d_coarse x 5.000 ns  -  true separation|  <  5 ns");
        $display("  Pairing timeout overridden to %0d cycles for speed.", SIM_TMO);
        $display("=====================================================================");

        repeat (10) @(posedge clk100);
        rst = 0;
        wait (mmcm_locked);
        repeat (20) @(posedge clk200);

        $display("");
        $display("--- Sweep the STOP separation ---");
        for (i = 0; i < 8; i = i + 1)
            do_pair(1.30, 12.00 + i * 3.70);

        $display("");
        $display("--- Sweep the phase at a fixed separation (21.00 ns) ---");
        for (i = 0; i < 6; i = i + 1)
            do_pair(0.80 * i, 21.00);

        $display("");
        $display("--- Sub-clock separations (interval < one clock period) ---");
        for (i = 0; i < 5; i = i + 1)
            do_pair(2.10, 1.00 + i * 0.90);

        $display("");
        $display("--- TIMEOUT: START fires, STOP never arrives ---");
        rearm;
        @(posedge clk200);
        ev_a = 1'b1; #40; ev_a = 1'b0;
        wait_meas;
        if (r_ok === 1'b1 && r_tmo === 1'b1 && r_vb === 1'b0) begin
            pass = pass + 1;
            $display("  [PASS] timeout asserted, valid_b forced 0 -> host filters the sample");
        end else begin
            fail = fail + 1;
            $display("  [FAIL] ok=%b timeout=%b valid_b=%b (expected 1, 1, 0)", r_ok, r_tmo, r_vb);
        end

        $display("");
        $display("=====================================================================");
        $display("  PASS = %0d    FAIL = %0d    worst coarse error = %0.3f ns",
                 pass, fail, worst_err);
        $display("");
        if (n_meas > 0 && n_railed == n_meas) begin
            $display("  fine railed (0 or 352) on %0d/%0d measurements.", n_railed, n_meas);
            $display("  -> ZERO-DELAY CARRY4 CONFIRMED. This is XSim, not a design fault.");
            $display("  -> Validate the fine path with Icarus + carry4_mock.v (tb_tdc_dual.v),");
            $display("     post-implementation TIMING sim (SDF), or hardware.");
        end else begin
            $display("  fine did NOT rail -> the carry chain has real delay in this run.");
            $display("  -> Run tb_tdc_dual.v; it checks the full picosecond path.");
        end
        $display("=====================================================================");
        $display("");
        $finish;
    end

endmodule