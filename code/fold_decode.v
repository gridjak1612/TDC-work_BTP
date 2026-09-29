`timescale 1ns/1ps
// =============================================================================
// fold_decode (rev 2) -- turns one sampled folding snapshot into a fine code.
//
// Snapshot layout (SAMPLED_W = LAUNCH_W + FOLD_W + NCNT bits), LSB first:
//   [LAUNCH_W-1:0]                 launch taps 0..LAUNCH_W-1   (monotonic step)
//   [LAUNCH_W+FOLD_W-1:LAUNCH_W]   fold taps B..E              (alternating polarity per lap)
//   [top NCNT bits]                counting taps, sparse, E+CNT_STEP*j
//
// Laps (116..131 taps on this board) are SHORTER than the fold (136), so the
// fold is never empty but two edges can share it at a lap boundary. Every edge
// still inside the fold is a valid time reference, so the decoder reports the
// OLDER edge (lap n from the counting taps); the newer edge, if already at B,
// is only used to locate the older one. (Rev 1 summed the two into a bogus
// position, and the 120-tap fold left gaps on chain B.)
//
//   n   = transitions along {fold[E], cnt[0..]}      edges that left the fold
//   ref = 1 for even n, else 0                       level under edge n
//   x   = fold XNOR ref                              1s under edge n
//       single edge  x = 1^p 0^...                   pos = popcount(x)
//       front at B   x = all 0                       pos = 0
//       fold empty   x = all 1                       pos = FOLD_W  (= next lap pos 0)
//       overlap      x = 0^a 1^(q-a) 0^...           newer edge at a, older at q >= FOLD_W-OVL_TOP
//                    pos = q = popcount(x) + FOLD_W - popcount(x | TOP)
//   ovl = ~x[0] & ~(x all 0)
//   code = launch popcount (front still in launch)  else  LAUNCH_W + n*FOLD_W + pos
//   valid = launch clean step & (in_launch |
//           (x has <= 1 falling transition & (x[top]==0 | x all 1)
//            & (~ovl | x[FOLD_W-OVL_TOP-1]) & n <= N_MAX))
//
// Latency: 1 (bubble) + 1 (xor) + 7 (popcount) + 1 (combine) = 10 < FINE_LATENCY (12).
// Reference model: fold_model.py (bit-exact, same majority filter).
// =============================================================================
module fold_decode #(
    parameter integer LAUNCH_W  = 32,
    parameter integer FOLD_W    = 136,
    parameter integer NCNT      = 7,
    parameter integer OVL_TOP   = 28,
    parameter integer N_MAX     = 4,
    parameter integer FINE_BITS = 10
)(
    input  wire                            clk,
    input  wire                            rst,
    input  wire [LAUNCH_W+FOLD_W+NCNT-1:0] sampled,
    output reg  [FINE_BITS-1:0]            fine,
    output reg                             valid
);
    localparam integer SW = LAUNCH_W + FOLD_W + NCNT;
    localparam integer PC = 7;                     // ones_counter_encoder_piped latency
    localparam integer VL = 3;                     // thermometer_validator_piped latency
    localparam integer LW = 6;                     // launch count 0..32
    localparam integer FW = 8;                     // fold count 0..136
    localparam [FOLD_W-1:0] TOP = {{OVL_TOP{1'b1}}, {(FOLD_W-OVL_TOP){1'b0}}};

    // ------------------------------------------------------------ stage A
    wire [LAUNCH_W-1:0] launch_raw = sampled[LAUNCH_W-1:0];
    wire [FOLD_W-1:0]   fold_raw   = sampled[LAUNCH_W+FOLD_W-1:LAUNCH_W];
    wire [NCNT-1:0]     cnt_raw    = sampled[SW-1:LAUNCH_W+FOLD_W];
    wire [LAUNCH_W-1:0] launch_c;
    wire [FOLD_W-1:0]   fold_c;

    bubble_correction #(.WIDTH(LAUNCH_W)) bub_l (.raw_therm(launch_raw), .corrected(launch_c));
    bubble_correction #(.WIDTH(FOLD_W))   bub_f (.raw_therm(fold_raw),   .corrected(fold_c));

    reg [LAUNCH_W-1:0] launch_a;
    reg [FOLD_W-1:0]   fold_a;
    reg [NCNT:0]       seq_a;
    always @(posedge clk) begin
        if (rst) begin launch_a <= 0; fold_a <= 0; seq_a <= 0; end
        else begin
            launch_a <= launch_c;
            fold_a   <= fold_c;
            seq_a    <= {cnt_raw, fold_c[FOLD_W-1]};
        end
    end

    // ------------------------------------------------------------ stage B
    integer j;
    reg [3:0] n_c;
    always @(*) begin
        n_c = 4'd0;
        for (j = 0; j < NCNT; j = j + 1)
            n_c = n_c + (seq_a[j] ^ seq_a[j+1]);
    end
    wire ref_c = ~n_c[0];

    reg [LAUNCH_W-1:0] launch_b;
    reg [FOLD_W-1:0]   x_b, xt_b;
    reg [3:0]          n_b;
    always @(posedge clk) begin
        if (rst) begin launch_b <= 0; x_b <= 0; xt_b <= 0; n_b <= 0; end
        else begin
            launch_b <= launch_a;
            x_b      <= fold_a ~^ {FOLD_W{ref_c}};
            xt_b     <= (fold_a ~^ {FOLD_W{ref_c}}) | TOP;
            n_b      <= n_c;
        end
    end

    // ------------------------------------------------------------ stage C
    wire [LW-1:0] launch_cnt;
    wire [FW-1:0] ones, ones_t;
    wire          launch_ok, x_ok;

    ones_counter_encoder_piped #(.INPUT_WIDTH(LAUNCH_W), .OUTPUT_WIDTH(LW)) pc_l (
        .clk(clk), .rst(rst), .thermometer_in(launch_b), .binary_out(launch_cnt));
    ones_counter_encoder_piped #(.INPUT_WIDTH(FOLD_W), .OUTPUT_WIDTH(FW)) pc_f (
        .clk(clk), .rst(rst), .thermometer_in(x_b), .binary_out(ones));
    ones_counter_encoder_piped #(.INPUT_WIDTH(FOLD_W), .OUTPUT_WIDTH(FW)) pc_t (
        .clk(clk), .rst(rst), .thermometer_in(xt_b), .binary_out(ones_t));
    thermometer_validator_piped #(.WIDTH(LAUNCH_W)) va_l (
        .clk(clk), .rst(rst), .thermometer_in(launch_b), .valid(launch_ok));
    thermometer_validator_piped #(.WIDTH(FOLD_W)) va_f (
        .clk(clk), .rst(rst), .thermometer_in(x_b), .valid(x_ok));

    // Align the 3-cycle validators and the side bits with the 7-cycle popcounts.
    reg [PC-VL-1:0] lok_d, xok_d;
    reg [3:0]       n_d   [0:PC-1];
    reg             x0_d  [0:PC-1];                // x[0]
    reg             xtop_d[0:PC-1];                // x[FOLD_W-1]
    reg             xov_d [0:PC-1];                // x[FOLD_W-OVL_TOP-1]
    reg             l0_d  [0:PC-1];                // launch[0]
    integer k;
    always @(posedge clk) begin
        if (rst) begin
            lok_d <= 0; xok_d <= 0;
            for (k = 0; k < PC; k = k + 1) begin
                n_d[k] <= 0; x0_d[k] <= 0; xtop_d[k] <= 0; xov_d[k] <= 0; l0_d[k] <= 0;
            end
        end else begin
            lok_d <= {lok_d[PC-VL-2:0], launch_ok};
            xok_d <= {xok_d[PC-VL-2:0], x_ok};
            n_d[0] <= n_b; x0_d[0] <= x_b[0]; xtop_d[0] <= x_b[FOLD_W-1];
            xov_d[0] <= x_b[FOLD_W-OVL_TOP-1]; l0_d[0] <= launch_b[0];
            for (k = 1; k < PC; k = k + 1) begin
                n_d[k] <= n_d[k-1]; x0_d[k] <= x0_d[k-1]; xtop_d[k] <= xtop_d[k-1];
                xov_d[k] <= xov_d[k-1]; l0_d[k] <= l0_d[k-1];
            end
        end
    end

    // ------------------------------------------------------------ stage D
    wire        in_launch = (launch_cnt < LAUNCH_W);
    wire        all_zero  = (ones == 0);
    wire        all_ones  = (ones == FOLD_W);
    wire        ovl       = ~x0_d[PC-1] & ~all_zero;
    wire [3:0]  n_f       = n_d[PC-1];
    wire [FW:0] pos       = ovl ? ({1'b0, ones} + FOLD_W - {1'b0, ones_t}) : {1'b0, ones};
    wire [FINE_BITS+2:0] full = LAUNCH_W + n_f * FOLD_W + pos;
    localparam [FINE_BITS-1:0] FMAX = {FINE_BITS{1'b1}};

    always @(posedge clk) begin
        if (rst) begin fine <= 0; valid <= 1'b0; end
        else begin
            if (in_launch)
                fine <= launch_cnt;
            else
                fine <= (full > FMAX) ? FMAX : full[FINE_BITS-1:0];
            valid <= lok_d[PC-VL-1] & (l0_d[PC-1] | (launch_cnt == 0))
                   & (in_launch | (xok_d[PC-VL-1] & (~xtop_d[PC-1] | all_ones)
                                   & (~ovl | xov_d[PC-1]) & (n_f <= N_MAX)));
        end
    end
endmodule
