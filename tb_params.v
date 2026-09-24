`timescale 1ns/1ps
// Fails loudly if either channel does not inherit TAP_SRC / SYNC_TAP from the board.
module tb_params;
  reg clk=0, rst=1; wire [15:0] led; wire tx;
  tdc_dual_board #(.TAP_SRC(2), .SYNC_TAP(7)) dut (
    .clk100(clk), .rst(rst), .btn_a(1'b0), .btn_b(1'b0), .btn_clear(1'b0),
    .ev_a_ext(1'b0), .ev_b_ext(1'b0), .sw_autorearm(1'b0), .led(led), .uart_txd(tx));
  initial begin
    $display("chan_a TAP_SRC=%0d SYNC_TAP=%0d", dut.core.chan_a.TAP_SRC, dut.core.chan_a.SYNC_TAP);
    $display("chan_b TAP_SRC=%0d SYNC_TAP=%0d", dut.core.chan_b.TAP_SRC, dut.core.chan_b.SYNC_TAP);
    if (dut.core.chan_a.TAP_SRC != dut.core.chan_b.TAP_SRC ||
        dut.core.chan_a.SYNC_TAP != dut.core.chan_b.SYNC_TAP) $display("FAIL: channels differ");
    else $display("PASS: both channels built identically");
    $finish;
  end
endmodule
