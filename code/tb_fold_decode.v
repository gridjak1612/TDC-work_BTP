`timescale 1ns/1ps
// =============================================================================
// tb_fold_decode -- bit-exact check of fold_decode against fold_model.py.
//   Reads fold_vectors.txt (hex snapshot, model fine, model valid), one vector
//   per clock into the pipeline, compares each output LATENCY cycles later,
//   and writes fold_tb_out.txt (rtl fine, rtl valid) for
//       python fold_model.py --check fold_vectors.txt fold_tb_out.txt
//   Add fold_vectors.txt to the sim_1 fileset so Vivado copies it next to xsim.
// =============================================================================
module tb_fold_decode;
    localparam integer LAUNCH_W = 32, FOLD_W = 136, NCNT = 7, FINE_BITS = 10;
    localparam integer SW = LAUNCH_W + FOLD_W + NCNT;
    localparam integer LATENCY = 10;
    localparam integer MAXV = 8192;

    reg clk = 1'b0;
    always #2.5 clk = ~clk;
    reg rst = 1'b1;

    reg  [SW-1:0]        sampled = 0;
    wire [FINE_BITS-1:0] fine;
    wire                 valid;

    fold_decode #(.LAUNCH_W(LAUNCH_W), .FOLD_W(FOLD_W), .NCNT(NCNT), .FINE_BITS(FINE_BITS)) dut (
        .clk(clk), .rst(rst), .sampled(sampled), .fine(fine), .valid(valid));

    reg [SW-1:0] vec  [0:MAXV-1];
    integer      efin [0:MAXV-1];
    integer      eval [0:MAXV-1];
    integer nvec = 0, fh, fo, r, i, errors = 0;
    reg [SW-1:0] hx; integer f, v;

    initial begin
        fh = $fopen("fold_vectors.txt", "r");
        if (fh == 0) begin $display("TB FAIL: cannot open fold_vectors.txt"); $finish; end
        while (!$feof(fh) && nvec < MAXV) begin
            r = $fscanf(fh, "%h %d %d\n", hx, f, v);
            if (r == 3) begin vec[nvec] = hx; efin[nvec] = f; eval[nvec] = v; nvec = nvec + 1; end
        end
        $fclose(fh);
        $display("TB: %0d vectors", nvec);
        fo = $fopen("fold_tb_out.txt", "w");
        repeat (3) @(posedge clk);
        rst = 1'b0;
        // drive vector i on cycle i; check on cycle i + LATENCY
        for (i = 0; i < nvec + LATENCY; i = i + 1) begin
            @(negedge clk);
            sampled = (i < nvec) ? vec[i] : {SW{1'b0}};
            if (i >= LATENCY) begin
                $fdisplay(fo, "%0d %0d", fine, valid);
                if (fine !== efin[i-LATENCY] || valid !== eval[i-LATENCY]) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("TB: vec %0d model fine=%0d valid=%0d  rtl fine=%0d valid=%0d",
                                 i-LATENCY, efin[i-LATENCY], eval[i-LATENCY], fine, valid);
                end
            end
        end
        $fclose(fo);
        if (errors == 0) $display("TB PASS: %0d vectors bit-exact, fold_tb_out.txt written", nvec);
        else             $display("TB FAIL: %0d mismatches", errors);
        $finish;
    end
endmodule
