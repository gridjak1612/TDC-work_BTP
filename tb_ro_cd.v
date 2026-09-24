// =============================================================================
// tb_ro_cd -- RO build plumbing check (xsim): RO -> chains -> 9-byte CRC frame
// -> UART -> uart_bytes.txt. xsim's CARRY4 is zero-delay, so every code reads
// 352 (railed). That is expected: this checks frames/CRC/seq/re-arm only.
// =============================================================================
`timescale 1ns/1ps
module tb_ro_cd;
    reg clk100 = 0, rst = 1;
    wire [15:0] led; wire txd;
    always #5 clk100 = ~clk100;

    localparam CPB = 4;
    tdc_dual_board #(
        .CLKS_PER_BIT(CPB), .EVENT_SRC(3), .RO_DIV_BITS(7), .SYNC_TAP(30)
    ) dut (
        .clk100(clk100), .rst(rst), .btn_a(1'b0), .btn_b(1'b0), .btn_clear(1'b0),
        .ev_a_ext(1'b0), .ev_b_ext(1'b0), .sw_autorearm(1'b1),
        .led(led), .uart_txd(txd));

    localparam real BIT_NS = CPB * 5.0;
    integer fh, nbytes = 0;
    initial fh = $fopen("uart_bytes.txt", "w");
    initial begin : rx
        integer i; reg [7:0] d;
        forever begin
            @(negedge txd);
            #(BIT_NS * 1.5);
            for (i = 0; i < 8; i = i + 1) begin d[i] = txd; #(BIT_NS); end
            $fwrite(fh, "%02x\n", d); nbytes = nbytes + 1;
        end
    end

    localparam integer FRAMES = 300;
    initial begin
        repeat (40) @(posedge clk100); rst = 0;
        wait (nbytes >= 9 * FRAMES);
        $fclose(fh);
        $display("frames=%0d  led[7]=%0d (must be 0)", nbytes / 9, led[7]);
        $finish;
    end
endmodule