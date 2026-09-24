// =============================================================================
// tb_deadlock -- regression for the sweep re-arm deadlock (xsim).
// SETTLE (400 cyc = 2 us) > frame time (< 1 us at CLKS_PER_BIT=2), so every
// step's last frame_done lands inside sweep_hold.
// Old RTL: sweep stops after the first step. Fixed RTL: keeps walking.
// =============================================================================
`timescale 1ns/1ps
module tb_deadlock;
    reg clk100 = 0, rst = 1;
    wire [15:0] led; wire txd;
    always #5 clk100 = ~clk100;
    tdc_dual_board #(
        .CLKS_PER_BIT(2), .EVENT_SRC(2), .AUTO_SWEEP(1),
        .SWEEP_STEPS(5), .SAMPLES_PER_STEP(2), .SETTLE_CYCLES(400)
    ) dut (
        .clk100(clk100), .rst(rst), .btn_a(1'b0), .btn_b(1'b0), .btn_clear(1'b0),
        .ev_a_ext(1'b0), .ev_b_ext(1'b0), .sw_autorearm(1'b1),
        .led(led), .uart_txd(txd));

    integer nmeas = 0, nsteps = 0;
    reg signed [11:0] last = 0;
    always @(posedge dut.clk200) begin
        if (dut.meas_ready) nmeas = nmeas + 1;
                if (dut.mmcm_locked === 1'b1 && dut.ps_phase_idx !== last) begin nsteps = nsteps + 1; last = dut.ps_phase_idx; end end
    
    initial begin
        repeat (40) @(posedge clk100); rst = 0;
        wait (dut.mmcm_locked === 1'b1);
        $display("MMCM locked at %0t", $time);
        #30000;
        $display("measurements=%0d  phase steps=%0d  final phase=%0d  led[7]=%0d",
                 nmeas, nsteps, dut.ps_phase_idx, led[7]);
        if (nsteps >= 3 && nmeas >= 8 && led[7] == 0) $display("PASS: sweep keeps running");
        else                                           $display("FAIL: sweep stalled");
        $finish;
    end
endmodule