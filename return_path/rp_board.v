`timescale 1ns/1ps
// =============================================================================
// rp_board -- RETURN-PATH PROBE top (folding step 1). Standalone bitstream:
// no TDC, no MMCM. Six rp_ring instances with K = 32 48 64 96 128 192 taps.
//
// Loop, forever:  enable ring `cur` alone
//                 SETTLE  2^SETTLE_BITS clk100 cycles (ring starts, mux settles)
//                 GATE    2^GATE_BITS   clk100 cycles, count rising edges of
//                         that ring's div_msb (f_ring / 2^DIV_BITS)
//                 SEND    one 12-byte frame, then next ring
//
//   f_ring = count * 2^DIV_BITS / (2^GATE_BITS * 10 ns)
//   T_lap  = 1 / (2 * f_ring)
//
// FRAME (12 bytes, 8N1, 2 Mbaud), MSB first:
//   0xB5 | ring | K[15:8] | K[7:0] | GATE_BITS | DIV_BITS | count[31:0] | seq | CRC-8
//   CRC-8 poly 0x07, init 0, over the first 11 bytes (same CRC as TDC frame v4).
//   Header 0xB5 differs from the TDC's 0xA7 so the two can't be confused.
//
// LEDs: [0] heartbeat, [6:1] ring enabled (one-hot), [15:8] seq.
// =============================================================================
module rp_board #(
    parameter integer CLKS_PER_BIT = 50,   // 100 MHz / 2 Mbaud
    parameter integer GATE_BITS    = 22,   // 2^22 x 10 ns = 41.9 ms per reading
    parameter integer SETTLE_BITS  = 16,   // 655 us
    parameter integer DIV_BITS     = 5     // ring / 32 before the synchroniser
)(
    input  wire        clk100,     // F14
    input  wire        rst,        // J2 btn0
    output wire [15:0] led,
    output wire        uart_txd    // U11
);
    localparam integer NRING = 6;
    localparam integer TW    = (GATE_BITS > SETTLE_BITS) ? GATE_BITS : SETTLE_BITS;

    // ---------------------------------------------------------------- reset
    (* ASYNC_REG = "TRUE" *) reg rs0 = 1'b1;
    (* ASYNC_REG = "TRUE" *) reg rs1 = 1'b1;
    always @(posedge clk100 or posedge rst)
        if (rst) begin rs0 <= 1'b1; rs1 <= 1'b1; end
        else     begin rs0 <= 1'b0; rs1 <= rs0;  end
    reg r = 1'b1;
    always @(posedge clk100) r <= rs1;

    // ---------------------------------------------------------------- rings
    reg  [2:0]       cur;
    wire [NRING-1:0] en = r ? {NRING{1'b0}} : (6'b000001 << cur);
    wire [NRING-1:0] dm;

    // Sim lap-time model 0.55 ns + 17 ps/tap (behavioural only).
    rp_ring #(.K(32),  .DIV_BITS(DIV_BITS), .SIM_TLAP_NS(0.55 + 0.017*32))  u_ring0 (.en(en[0]), .div_msb(dm[0]));
    rp_ring #(.K(48),  .DIV_BITS(DIV_BITS), .SIM_TLAP_NS(0.55 + 0.017*48))  u_ring1 (.en(en[1]), .div_msb(dm[1]));
    rp_ring #(.K(64),  .DIV_BITS(DIV_BITS), .SIM_TLAP_NS(0.55 + 0.017*64))  u_ring2 (.en(en[2]), .div_msb(dm[2]));
    rp_ring #(.K(96),  .DIV_BITS(DIV_BITS), .SIM_TLAP_NS(0.55 + 0.017*96))  u_ring3 (.en(en[3]), .div_msb(dm[3]));
    rp_ring #(.K(128), .DIV_BITS(DIV_BITS), .SIM_TLAP_NS(0.55 + 0.017*128)) u_ring4 (.en(en[4]), .div_msb(dm[4]));
    rp_ring #(.K(192), .DIV_BITS(DIV_BITS), .SIM_TLAP_NS(0.55 + 0.017*192)) u_ring5 (.en(en[5]), .div_msb(dm[5]));

    reg [15:0] k_cur;
    always @(*) begin
        case (cur)
            3'd0: k_cur = 16'd32;
            3'd1: k_cur = 16'd48;
            3'd2: k_cur = 16'd64;
            3'd3: k_cur = 16'd96;
            3'd4: k_cur = 16'd128;
            default: k_cur = 16'd192;
        endcase
    end

    // Selected divider MSB -> 2-FF synchroniser -> rising-edge detect.
    // The select only changes at the start of SETTLE, so any mux glitch is over
    // long before GATE opens.
    wire dsel = dm[cur];
    (* ASYNC_REG = "TRUE" *) reg sy0 = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg sy1 = 1'b0;
    reg sy2 = 1'b0;
    always @(posedge clk100) begin sy0 <= dsel; sy1 <= sy0; sy2 <= sy1; end
    wire rise = sy1 & ~sy2;

    // ---------------------------------------------------------------- FSM
    localparam [1:0] S_SETTLE = 2'd0, S_GATE = 2'd1, S_SEND = 2'd2;
    reg [1:0]    st;
    reg [TW-1:0] tmr;
    reg [31:0]   edges, cnt_l;
    reg [7:0]    seq;
    reg [2:0]    ring_l;
    reg [15:0]   k_l;
    reg          start_tx;
    wire         tx_done;

    // Plain arithmetic: a {0{...}} replication (when TW == GATE_BITS) is
    // illegal Verilog-2001 and xsim rejects it at elaboration.
    localparam [TW-1:0] SETTLE_END = (1 << SETTLE_BITS) - 1;
    localparam [TW-1:0] GATE_END   = (1 << GATE_BITS) - 1;

    always @(posedge clk100) begin
        if (r) begin
            st <= S_SETTLE; tmr <= 0; edges <= 0; cnt_l <= 0; seq <= 0;
            cur <= 3'd0; ring_l <= 3'd0; k_l <= 16'd0; start_tx <= 1'b0;
        end else begin
            start_tx <= 1'b0;
            case (st)
            S_SETTLE: begin
                if (tmr == SETTLE_END) begin tmr <= 0; edges <= 0; st <= S_GATE; end
                else tmr <= tmr + 1'b1;
            end
            S_GATE: begin
                if (tmr == GATE_END) begin
                    cnt_l    <= edges + rise;     // exactly 2^GATE_BITS samples
                    ring_l   <= cur;
                    k_l      <= k_cur;
                    start_tx <= 1'b1;
                    st       <= S_SEND;
                end else begin
                    edges <= edges + rise;
                    tmr   <= tmr + 1'b1;
                end
            end
            S_SEND: begin
                if (tx_done) begin
                    seq <= seq + 1'b1;
                    cur <= (cur == NRING-1) ? 3'd0 : cur + 1'b1;
                    tmr <= 0;
                    st  <= S_SETTLE;
                end
            end
            default: st <= S_SETTLE;
            endcase
        end
    end

    // ---------------------------------------------------------------- frame
    function [7:0] crc8_88;
        input [87:0] d;
        integer i;
        reg [7:0] c;
        begin
            c = 8'h00;
            for (i = 87; i >= 0; i = i - 1)
                c = {c[6:0], 1'b0} ^ ((c[7] ^ d[i]) ? 8'h07 : 8'h00);
            crc8_88 = c;
        end
    endfunction

    wire [7:0]  gb8 = GATE_BITS;
    wire [7:0]  db8 = DIV_BITS;
    wire [87:0] body  = {8'hB5, 5'd0, ring_l, k_l, gb8, db8, cnt_l, seq};
    wire [95:0] frame = {body, crc8_88(body)};

    wire       uart_busy;
    reg        uart_send;
    reg [7:0]  uart_byte;
    reg [95:0] sr;
    reg [3:0]  nleft;
    reg        sending;
    reg        done_q;
    assign tx_done = done_q;

    always @(posedge clk100) begin
        if (r) begin
            uart_send <= 1'b0; uart_byte <= 8'h00; sr <= 96'd0;
            nleft <= 4'd0; sending <= 1'b0; done_q <= 1'b0;
        end else begin
            uart_send <= 1'b0;
            done_q    <= 1'b0;
            if (start_tx) begin
                sr      <= frame;
                nleft   <= 4'd12;
                sending <= 1'b1;
            end else if (sending && !uart_busy && !uart_send) begin
                if (nleft == 4'd0) begin
                    sending <= 1'b0;
                    done_q  <= 1'b1;
                end else begin
                    uart_byte <= sr[95:88];
                    sr        <= {sr[87:0], 8'h00};
                    uart_send <= 1'b1;
                    nleft     <= nleft - 1'b1;
                end
            end
        end
    end

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) uart_tx_inst (
        .clk (clk100), .rst (r), .send (uart_send), .data (uart_byte),
        .tx  (uart_txd), .busy (uart_busy)
    );

    // ---------------------------------------------------------------- LEDs
    reg [25:0] hb = 26'd0;
    always @(posedge clk100) hb <= hb + 1'b1;
    assign led[0]    = hb[25];
    assign led[6:1]  = en;
    assign led[7]    = 1'b0;
    assign led[15:8] = seq;
endmodule
