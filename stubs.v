`timescale 1ns/1ps
// Icarus-only stubs for tb_params.v. Do NOT add to the Vivado project.
module CARRY4(output [3:0] CO, output [3:0] O, input CI, input CYINIT, input [3:0] DI, input [3:0] S);
  wire c0 = CI | CYINIT;
  assign CO = {4{c0}}; assign O = ~CO;
endmodule
module clk_wiz_0(input clk_in1, output clk_out1, output clk_out2, input psclk, input psen,
                 input psincdec, output psdone, input reset, output locked);
  assign clk_out1 = clk_in1; assign clk_out2 = clk_in1; assign psdone = psen; assign locked = ~reset;
endmodule
