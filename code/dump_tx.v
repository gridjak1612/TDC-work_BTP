`timescale 1ns/1ps
// =============================================================================
// dump_tx -- frame v5: raw channel-A snapshot for folding bring-up.
//   0xC3 | raw[159:0] MSB first (20 bytes, bit 159 = 0) | fine[15:8] | fine[7:0]
//        | {7'b0, valid} | seq | CRC-8 (poly 0x07, init 0, over the 25 bytes before it)
//   26 bytes = 130 us at 2 Mbaud. A meas_ready that arrives while a frame is
//   still going out is dropped (seq shows the gaps).
// =============================================================================
module dump_tx #(
    parameter integer CLKS_PER_BIT = 100,
    parameter integer RAW_W        = 159,
    parameter integer FINE_BITS    = 10
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 meas_ready,
    input  wire [RAW_W-1:0]     raw,
    input  wire [FINE_BITS-1:0] fine,
    input  wire                 valid,
    output wire                 txd
);
    localparam integer NB = 26;
    function [7:0] crc8_200;
        input [199:0] d;
        integer i;
        reg [7:0] c;
        begin
            c = 8'h00;
            for (i = 199; i >= 0; i = i - 1)
                c = {c[6:0], 1'b0} ^ ((c[7] ^ d[i]) ? 8'h07 : 8'h00);
            crc8_200 = c;
        end
    endfunction

    reg  [7:0]   seq;
    wire [159:0] raw160 = {{(160-RAW_W){1'b0}}, raw};
    wire [15:0]  fine16 = fine;
    wire [199:0] body   = {8'hC3, raw160, fine16, 7'd0, valid, seq};
    wire [207:0] frame  = {body, crc8_200(body)};

    reg        send, sending;
    reg [7:0]  byte_r;
    reg [207:0] sr;
    reg [4:0]  nleft;
    wire       busy;

    always @(posedge clk) begin
        if (rst) begin
            send <= 1'b0; sending <= 1'b0; byte_r <= 8'h00; sr <= 0; nleft <= 0; seq <= 0;
        end else begin
            send <= 1'b0;
            if (meas_ready && !sending) begin
                sr <= frame; nleft <= NB; sending <= 1'b1; seq <= seq + 1'b1;
            end else if (sending && !busy && !send) begin
                if (nleft == 0) sending <= 1'b0;
                else begin
                    byte_r <= sr[207:200]; sr <= {sr[199:0], 8'h00};
                    send <= 1'b1; nleft <= nleft - 1'b1;
                end
            end
        end
    end

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart (
        .clk(clk), .rst(rst), .send(send), .data(byte_r), .tx(txd), .busy(busy));
endmodule
