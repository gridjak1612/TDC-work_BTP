`timescale 1ns/1ps
// =============================================================================
// tb_rp -- control/framing test for rp_board (xsim).
//   Behavioural rings (XILINX_SIMULATOR branch of rp_ring), short gate.
//   Decodes the UART, writes rp_uart.txt (one hex byte per line), and checks:
//   12 frames, header 0xB5, ring ids 0..5 twice, seq 0..11, K per ring, stop bits.
//   Frequency / CRC are checked on the host:
//       python read_rp.py --from-bytes rp_uart.txt --sim-check
// =============================================================================
module tb_rp;
    localparam integer CPB    = 4;          // clocks per UART bit in this TB
    localparam real    BIT_NS = CPB * 10.0;
    localparam integer NFR    = 12;
    localparam integer FLEN   = 12;

    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst = 1'b1;

    wire        tx;
    wire [15:0] led;

    rp_board #(.CLKS_PER_BIT(CPB), .GATE_BITS(14), .SETTLE_BITS(8), .DIV_BITS(5)) dut (
        .clk100(clk), .rst(rst), .led(led), .uart_txd(tx));

    reg [7:0] mem [0:NFR*FLEN-1];
    integer nbytes = 0, errors = 0, fh, f, i;
    reg [7:0] b;

    initial begin
        fh = $fopen("rp_uart.txt", "w");
        #200 rst = 1'b0;
    end

    // UART receiver
    always begin
        @(negedge tx);
        #(BIT_NS * 1.5);
        for (i = 0; i < 8; i = i + 1) begin b[i] = tx; #(BIT_NS); end
        if (tx !== 1'b1) begin
            $display("TB: framing error at byte %0d", nbytes);
            errors = errors + 1;
        end
        if (nbytes < NFR*FLEN) begin
            mem[nbytes] = b;
            $fdisplay(fh, "%02x", b);
        end
        nbytes = nbytes + 1;
    end

    function integer k_of;
        input integer ring;
        begin
            case (ring)
                0: k_of = 32;  1: k_of = 48;  2: k_of = 64;
                3: k_of = 96;  4: k_of = 128; default: k_of = 192;
            endcase
        end
    endfunction

    initial begin
        wait (nbytes >= NFR*FLEN);
        #100;
        $fclose(fh);
        for (f = 0; f < NFR; f = f + 1) begin
            if (mem[f*FLEN] !== 8'hB5) begin
                $display("TB: frame %0d bad header %02x", f, mem[f*FLEN]); errors = errors + 1;
            end
            if (mem[f*FLEN+1] !== (f % 6)) begin
                $display("TB: frame %0d ring %0d, expected %0d", f, mem[f*FLEN+1], f % 6); errors = errors + 1;
            end
            if ({mem[f*FLEN+2], mem[f*FLEN+3]} !== k_of(f % 6)) begin
                $display("TB: frame %0d K %0d, expected %0d", f,
                         {mem[f*FLEN+2], mem[f*FLEN+3]}, k_of(f % 6)); errors = errors + 1;
            end
            if (mem[f*FLEN+4] !== 8'd14 || mem[f*FLEN+5] !== 8'd5) begin
                $display("TB: frame %0d gate/div bits %0d/%0d", f, mem[f*FLEN+4], mem[f*FLEN+5]); errors = errors + 1;
            end
            if (mem[f*FLEN+10] !== f) begin
                $display("TB: frame %0d seq %0d", f, mem[f*FLEN+10]); errors = errors + 1;
            end
            $display("TB: frame %0d ring %0d K %0d count %0d", f, mem[f*FLEN+1],
                     {mem[f*FLEN+2], mem[f*FLEN+3]},
                     {mem[f*FLEN+6], mem[f*FLEN+7], mem[f*FLEN+8], mem[f*FLEN+9]});
        end
        if (errors == 0) $display("TB PASS: %0d frames, rp_uart.txt written", NFR);
        else             $display("TB FAIL: %0d errors", errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("TB FAIL: timeout, %0d bytes received", nbytes);
        $finish;
    end
endmodule
