`timescale 1ns/1ps
//=============================================================================
// tb_tdc_dual_board.v
//
// BOARD-LEVEL test. Does not peek at internal buses to check the result --
// it DECODES THE ACTUAL SERIAL BIT STREAM coming out of uart_txd, exactly as
// read_tdc_dual.py will, and checks the five bytes reconstruct the record.
//
// This is the only thing that tests:
//   - the new 5-byte frame packing  {valid_b, valid_a, d_coarse[13:8]}
//   - the framing FSM sequencing
//   - auto-rearm (sw_autorearm) producing back-to-back measurements
//   - the host-side reconstruction, including the SIGNED fine difference
//
// CLKS_PER_BIT is overridden to 20 so the simulation runs in a sane time.
// On hardware it is 1736 (200 MHz / 115200); the logic is identical.
//=============================================================================
module tb_tdc_dual_board;

    localparam integer CPB    = 20;             // sim baud divider
    localparam real    BIT_NS = CPB * 5.0;      // one UART bit at 200 MHz
    localparam real    TCLK   = 5.000;
    localparam real    TAU    = 0.020;          // carry4_mock: 20 ps/tap

    reg clk100 = 0;
    reg rst = 1, btn_a = 0, btn_b = 0, btn_clear = 0;
    reg ev_a = 0, ev_b = 0;
    reg sw_autorearm = 1;                        // auto re-arm ON

    wire [15:0] led;
    wire        uart_txd;

    tdc_dual_board #(
        .CLKS_PER_BIT(CPB),
        .EVENT_SRC(1),          // external pins
        .TIE_CHANNELS(0)        // independent START / STOP
    ) uut (
        .clk100(clk100), .rst(rst),
        .btn_a(btn_a), .btn_b(btn_b), .btn_clear(btn_clear),
        .ev_a_ext(ev_a), .ev_b_ext(ev_b),
        .sw_autorearm(sw_autorearm),
        .led(led), .uart_txd(uart_txd)
    );

    always #5 clk100 = ~clk100;

    integer pass = 0, fail = 0;

    //-------------------------------------------------------------------------
    // UART receiver -- 8N1, LSB first. Exactly what the host does.
    //-------------------------------------------------------------------------
    task uart_get(output [7:0] b);
        integer i;
        begin
            @(negedge uart_txd);          // start bit
            #(BIT_NS * 1.5);              // centre of bit 0
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = uart_txd;
                #(BIT_NS);
            end
            // now inside the stop bit
        end
    endtask

    // Receive one complete frame, resynchronising on 0xAA.
    // 6-byte frame: AA, fa[7:0], fb[7:0], dc[7:0], {fb[8],fa[8],dc[13:8]}, {6'b0,vb,va}
    task get_frame(output [8:0] fa, output [8:0] fb,
                   output [13:0] dc, output va, output vb);
        reg [7:0] h, b1, b2, b3, b4, b5;
        begin
            h = 8'h00;
            while (h !== 8'hAA) uart_get(h);   // hunt for the header
            uart_get(b1);
            uart_get(b2);
            uart_get(b3);
            uart_get(b4);
            uart_get(b5);
            fa = {b4[6], b1};
            fb = {b4[7], b2};
            dc = {b4[5:0], b3};
            va = b5[0];
            vb = b5[1];
        end
    endtask

    //-------------------------------------------------------------------------
    reg [8:0]  fa, fb;
    reg [13:0] dc;
    reg        va, vb;
    real       meas, err;
    integer    ifa, ifb;
    integer    k;
    real       sep;

    // Fire a START/STOP pair with a known separation.
    // Must WAIT for both channels to be re-armed first: after a capture the
    // controller locks out (done=1) and ignores events until clear_status.
    // Auto-rearm fires on frame_done, i.e. only once the last UART byte is out.
    // (On hardware with a free-running generator this is a non-issue: events
    //  during the lockout are simply ignored, and the next rising edge after
    //  re-arm is caught.)
    task fire(input real phase, input real separation);
        begin
            wait (uut.core.done_a === 1'b0 && uut.core.done_b === 1'b0);
            repeat (4) @(posedge uut.core.clk200);
            @(posedge uut.core.clk200);
            #(phase);
            ev_a = 1'b1;
            #(separation);
            ev_b = 1'b1;
            #40;
            ev_a = 1'b0;
            ev_b = 1'b0;
        end
    endtask

    initial begin
        $display("");
        $display("======================================================================");
        $display("  BOARD-LEVEL: decoding the REAL uart_txd bit stream");
        $display("  (6-byte frame: AA, fa[7:0], fb[7:0], dc[7:0], {fb8,fa8,dc[13:8]}, {vb,va})");
        $display("======================================================================");

        repeat (10) @(posedge clk100);
        rst = 0;
        wait (uut.core.mmcm_locked);
        repeat (20) @(posedge clk100);

        for (k = 0; k < 5; k = k + 1) begin
            sep = 13.40 + k * 4.30;      // 13.4 .. 30.6 ns, non-integer

            fork
                fire(1.70, sep);
                get_frame(fa, fb, dc, va, vb);
            join

            // ---- host reconstruction. SIGNED fine difference. ----
            ifa  = fa;
            ifb  = fb;
            meas = dc * TCLK - (ifb - ifa) * TAU;
            err  = meas - sep;
            if (err < 0) err = -err;

            if (err < 0.030 && va && vb) begin
                pass = pass + 1;
                $display("  [PASS] true=%6.2f ns | frame: fa=%3d fb=%3d dc=%3d va=%b vb=%b | decoded=%7.3f ns",
                         sep, fa, fb, dc, va, vb, meas);
            end else begin
                fail = fail + 1;
                $display("  [FAIL] true=%6.2f ns | frame: fa=%3d fb=%3d dc=%3d va=%b vb=%b | decoded=%7.3f ns",
                         sep, fa, fb, dc, va, vb, meas);
            end
        end

        $display("");
        $display("  auto-rearm produced %0d consecutive frames with no button press", pass+fail);
        $display("  measurement counter (led[15:8]) = %0d", led[15:8]);
        if (led[15:8] == (pass+fail)) begin
            pass = pass + 1;
            $display("  [PASS] LED counter matches frame count");
        end else begin
            fail = fail + 1;
            $display("  [FAIL] LED counter = %0d, expected %0d", led[15:8], pass+fail);
        end

        $display("");
        $display("======================================================================");
        $display("  PASS = %0d   FAIL = %0d", pass, fail);
        $display("======================================================================");
        $display("");
        $finish;
    end

    initial begin
        #200000;
        $display("  *** TIMEOUT -- no frames decoded ***");
        $finish;
    end

endmodule