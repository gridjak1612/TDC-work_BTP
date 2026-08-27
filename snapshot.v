`timescale 1ns/1ps
module snapshot_pipeline #(
    parameter WIDTH = 256,
    parameter DEPTH = 4
)(
    input  wire             clk,
    input  wire             rst,
    input  wire             capture_enable,
    input  wire [WIDTH-1:0] din,
    output reg  [WIDTH-1:0] captured
);
    (* SHREG_EXTRACT = "NO", DONT_TOUCH = "TRUE" *)
    reg [WIDTH-1:0] tap_reg;
    always @(posedge clk) tap_reg <= din;

    reg [WIDTH-1:0] pipe [0:DEPTH-2];
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DEPTH-1; i = i + 1) pipe[i] <= {WIDTH{1'b0}};
            captured <= {WIDTH{1'b0}};
        end else begin
            pipe[0] <= tap_reg;
            for (i = 1; i < DEPTH-1; i = i + 1) pipe[i] <= pipe[i-1];
            if (capture_enable) captured <= pipe[DEPTH-2];
        end
    end
endmodule