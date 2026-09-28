`timescale 1ns/1ps
// =============================================================================
// fold_decode -- turns one sampled folding snapshot into a fine code.
//
// Snapshot layout (SAMPLED_W = LAUNCH_W + FOLD_W + NCNT bits), LSB first:
//   [LAUNCH_W-1:0]                 launch taps 0..LAUNCH_W-1   (monotonic step)
//   [LAUNCH_W+FOLD_W-1:LAUNCH_W]   fold taps B..E              (one edge, alternating polarity)
//   [top NCNT bits]                counting taps, sparse, E+CNT_STEP*j
//
// Lap count n     = transitions along {fold[E], cnt[0], cnt[1], ...}
// Reference level = level right under the latest edge = 1 for even n, 0 for odd n
// x = fold ^ ref  -> 0s from B up to the edge, then 1s   (a thermometer code)
// pos             = FOLD_W - popcount(x)
//
// code = launch_popcount                        if the front is still in the launch section
//      = LAUNCH_W + n*FOLD_W + pos              otherwise
// Adjacent across lap boundaries: an off-by-one in n at the boundary lands on
// a neighbouring code, never in another lap.
//
// valid = launch is a clean step
//       & (front in launch | (x is 0..01..1  &  n <= N_MAX))
//   x all-0 (edge exactly at E) is accepted: pos = FOLD_W = next lap's pos 0.
//
// Latency (clk cycles, input -> fine/valid): 1 (bubble) + 1 (xor) + 7 (popcount)
//   + 1 (combine) = 10.  Must be < FINE_LATENCY of the channel (12).
// Reference model: fold_model.py (bit-exact, same majority filter).
// =============================================================================
module fold_decode #(
    parameter integer LAUNCH_W  = 32,
    parameter integer FOLD_W    = 120,
    parameter integer NCNT      = 7,
    parameter integer N_MAX     = 4,
    parameter integer FINE_BITS = 10
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire [LAUNCH_W+FOLD_W+NCNT-1:0] sampled,
    output reg  [FINE_BITS-1:0]          fine,
    output reg                           valid
);
    localparam integer SW = LAUNCH_W + FOLD_W + NCNT;
    localparam integer PC = 7;                     // ones_counter_encoder_piped latency
    localparam integer VL = 3;                     // thermometer_validator_piped latency

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
    reg [NCNT:0]       seq_a;                      // {cnt, fold[E]}
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
    reg [FOLD_W-1:0]   x_b;
    reg [3:0]          n_b;
    always @(posedge clk) begin
        if (rst) begin launch_b <= 0; x_b <= 0; n_b <= 0; end
        else begin
            launch_b <= launch_a;
            x_b      <= fold_a ^ {FOLD_W{ref_c}};
            n_b      <= n_c;
        end
    end

    // ------------------------------------------------------------ stage C
    localparam integer LW = 6;                     // 0..32
    localparam integer FW = 7;                     // 0..120
    wire [LW-1:0] launch_cnt;
    wire [FW-1:0] fold_ones;
    wire          launch_ok, fold_ok;

    ones_counter_encoder_piped #(.INPUT_WIDTH(LAUNCH_W), .OUTPUT_WIDTH(LW)) pc_l (
        .clk(clk), .rst(rst), .thermometer_in(launch_b), .binary_out(launch_cnt));
    ones_counter_encoder_piped #(.INPUT_WIDTH(FOLD_W), .OUTPUT_WIDTH(FW)) pc_f (
        .clk(clk), .rst(rst), .thermometer_in(x_b), .binary_out(fold_ones));
    thermometer_validator_piped #(.WIDTH(LAUNCH_W)) va_l (
        .clk(clk), .rst(rst), .thermometer_in(launch_b), .valid(launch_ok));
    thermometer_validator_piped #(.WIDTH(FOLD_W)) va_f (
        .clk(clk), .rst(rst), .thermometer_in(x_b), .valid(fold_ok));

    // Align the 3-cycle validators and the side bits with the 7-cycle popcounts.
    reg [PC-VL-1:0] lok_d, fok_d;
    reg [3:0]       n_d   [0:PC-1];
    reg             xt_d  [0:PC-1];                // x[FOLD_W-1]
    reg             l0_d  [0:PC-1];                // launch[0]
    integer k;
    always @(posedge clk) begin
        if (rst) begin
            lok_d <= 0; fok_d <= 0;
            for (k = 0; k < PC; k = k + 1) begin n_d[k] <= 0; xt_d[k] <= 0; l0_d[k] <= 0; end
        end else begin
            lok_d <= {lok_d[PC-VL-2:0], launch_ok};
            fok_d <= {fok_d[PC-VL-2:0], fold_ok};
            n_d[0]  <= n_b;  xt_d[0] <= x_b[FOLD_W-1];  l0_d[0] <= launch_b[0];
            for (k = 1; k < PC; k = k + 1) begin
                n_d[k] <= n_d[k-1]; xt_d[k] <= xt_d[k-1]; l0_d[k] <= l0_d[k-1];
            end
        end
    end

    // ------------------------------------------------------------ stage D
    wire        in_launch = (launch_cnt < LAUNCH_W);
    wire        all_zero  = (fold_ones == 0);
    wire [3:0]  n_f       = n_d[PC-1];
    wire [FINE_BITS-1:0] pos  = FOLD_W - fold_ones;
    wire [FINE_BITS+2:0] full = LAUNCH_W + n_f * FOLD_W + pos;
    localparam [FINE_BITS-1:0] FMAX = {FINE_BITS{1'b1}};

    always @(posedge clk) begin
        if (rst) begin fine <= 0; valid <= 1'b0; end
        else begin
            if (in_launch)
                fine <= launch_cnt;
            else
                fine <= (full > FMAX) ? FMAX : full[FINE_BITS-1:0];
            // launch must be a clean step whose bottom tap carries the hit
            // (or be empty: code 0, which the host treats as railed).
            valid <= lok_d[PC-VL-1] & (l0_d[PC-1] | (launch_cnt == 0))
                   & (in_launch | (fok_d[PC-VL-1] & (xt_d[PC-1] | all_zero) & (n_f <= N_MAX)));
        end
    end
endmodule
