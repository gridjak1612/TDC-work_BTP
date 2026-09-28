`timescale 1ns/1ps
// tb_multi -- EVENT_SRC = 4 run-time source select.
module tb_multi;
    reg clk100 = 0, rst = 1;
    reg [1:0] sw = 2'd1;
    reg eva = 0, evb = 0;
    wire [15:0] led; wire txd;
    always #5 clk100 = ~clk100;
    always #487 eva = ~eva;              // external "generator", ~1 MHz
    always @(eva) evb <= #3 eva;         // STOP 3 ns after START

    localparam CPB = 4;
    tdc_dual_board #(
        .CLKS_PER_BIT(CPB), .EVENT_SRC(4), .RO_DIV_BITS(7),
        .SWEEP_STEPS(5), .SAMPLES_PER_STEP(3), .SETTLE_CYCLES(50)
    ) dut (
        .clk100(clk100), .rst(rst), .btn_a(1'b0), .btn_b(1'b0), .btn_clear(1'b0),
        .ev_a_ext(eva), .ev_b_ext(evb), .sw_autorearm(1'b1), .sw_src(sw),
        .led(led), .uart_txd(txd));
    defparam dut.g_multi.ro11_inst.SIM_T_RO_NS = 11.3719;

    localparam real BIT_NS = CPB * 5.0;
    integer fh, nbytes = 0;
    initial fh = $fopen("uart_multi.txt", "w");
    initial begin : rx
        integer i; reg [7:0] d;
        forever begin
            @(negedge txd);
            #(BIT_NS * 1.5);
            for (i = 0; i < 8; i = i + 1) begin d[i] = txd; #(BIT_NS); end
            $fwrite(fh, "%02x\n", d); nbytes = nbytes + 1;
        end
    end

    task run_source(input [1:0] s, input integer nframes);
        integer start;
        begin
            sw = s;
            #200;                                              // switch synchroniser
            rst = 1; repeat (40) @(posedge clk100); rst = 0;   // BTN0
            start = nbytes;
            wait (nbytes >= start + 10 * nframes);
            $display("source %0d: %0d frames, led[7]=%0d", s, (nbytes - start) / 10, led[7]);
        end
    endtask

    initial begin : watchdog
        #5000000;
        $display("TIMEOUT: stuck with sw=%0d after %0d bytes", sw, nbytes);
        $fclose(fh); $finish;
    end

    initial begin
        repeat (40) @(posedge clk100); rst = 0;
        run_source(2'd1, 40);
        run_source(2'd2, 40);
        run_source(2'd3, 40);
        run_source(2'd0, 40);
        $fclose(fh);
        $display("DONE -- now run: python check_uart_dump.py uart_multi.txt");
        $finish;
    end
endmodule