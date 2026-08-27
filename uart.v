// =============================================================================
// Module Name:  uart_tx
// Description:  Simple 8N1 UART transmitter.
//               CLKS_PER_BIT = f_clk / baud  (e.g. 200e6 / 115200 = 1736).
//               Assert `send` for one clock with `data` valid; the byte is
//               transmitted LSB-first with 1 start (0) and 1 stop (1) bit.
//               `busy` is high while transmitting; ignore `send` when busy.
// =============================================================================
`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLKS_PER_BIT = 1736
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       send,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);
    localparam IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    always @(posedge clk) begin
        if (rst) begin
            state   <= IDLE;
            tx      <= 1'b1;   // UART idles high
            busy    <= 1'b0;
            clk_cnt <= 16'd0;
            bit_idx <= 3'd0;
            shreg   <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    tx   <= 1'b1;
                    busy <= 1'b0;
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (send) begin
                        shreg <= data;
                        busy  <= 1'b1;
                        state <= START;
                    end
                end
                START: begin
                    tx <= 1'b0;                       // start bit
                    if (clk_cnt < CLKS_PER_BIT-1) clk_cnt <= clk_cnt + 1'b1;
                    else begin clk_cnt <= 0; state <= DATA; end
                end
                DATA: begin
                    tx <= shreg[bit_idx];
                    if (clk_cnt < CLKS_PER_BIT-1) clk_cnt <= clk_cnt + 1'b1;
                    else begin
                        clk_cnt <= 0;
                        if (bit_idx < 3'd7) bit_idx <= bit_idx + 1'b1;
                        else begin bit_idx <= 0; state <= STOP; end
                    end
                end
                STOP: begin
                    tx <= 1'b1;                        // stop bit
                    if (clk_cnt < CLKS_PER_BIT-1) clk_cnt <= clk_cnt + 1'b1;
                    else begin clk_cnt <= 0; busy <= 1'b0; state <= IDLE; end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule