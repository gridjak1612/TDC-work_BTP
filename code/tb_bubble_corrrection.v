// =============================================================================
// tb_bubble_correction -- boundary + bubble-rejection check for the 5-tap
// majority filter. Runs the OLD (zero-padded) and NEW (ones-padded) low
// boundary side by side.
//
//   The TDL fills from tap 0 upward: taps[0..fine-1] = 1, taps[fine..] = 0.
//   So the virtual taps BELOW index 0 are logically ONE, not zero. Padding the
//   low boundary with zeros makes corrected[0..1] unreachable and silently
//   deletes fine codes 1 and 2 on every single sample.
// =============================================================================
`timescale 1ns/1ps
module tb_bubble_correction;
  localparam integer W = 352;

  reg  [W-1:0] raw;
  wire [W-1:0] cor_old, cor_new;

  bubble_old #(.WIDTH(W)) u_old (.raw_therm(raw), .corrected(cor_old));
  bubble_new #(.WIDTH(W)) u_new (.raw_therm(raw), .corrected(cor_new));

  integer f, k, b, errs_old, errs_new, promo_old, promo_new;

  function integer pop; input [W-1:0] v; integer i;
    begin pop = 0; for (i = 0; i < W; i = i + 1) pop = pop + v[i]; end
  endfunction

  task mk_ideal; input integer n; integer i;
    begin raw = {W{1'b0}}; for (i = 0; i < n; i = i + 1) raw[i] = 1'b1; end
  endtask

  initial begin
    // ---- TEST 1: every ideal code 0..W must map to itself -------------------
    errs_old = 0; errs_new = 0;
    for (f = 0; f <= W; f = f + 1) begin
      mk_ideal(f); #1;
      if (pop(cor_old) !== f) begin
        errs_old = errs_old + 1;
        if (errs_old <= 8) $display("  OLD: ideal %0d -> %0d", f, pop(cor_old));
      end
      if (pop(cor_new) !== f) begin
        errs_new = errs_new + 1;
        $display("  NEW: ideal %0d -> %0d", f, pop(cor_new));
      end
    end
    $display("TEST 1  ideal codes 0..%0d   OLD errors = %0d   NEW errors = %0d",
             W, errs_old, errs_new);

    // ---- TEST 2: single bubble INSIDE the ones region must be repaired ------
    errs_old = 0; errs_new = 0;
    for (f = 6; f <= W-6; f = f + 1)
      for (b = 2; b <= 4; b = b + 1) begin
        mk_ideal(f); raw[f-b] = 1'b0; #1;      // drop a one, away from the edge
        if (pop(cor_old) !== f) errs_old = errs_old + 1;
        if (pop(cor_new) !== f) errs_new = errs_new + 1;
      end
    $display("TEST 2  1 dropped bit inside ones  OLD errors = %0d   NEW errors = %0d",
             errs_old, errs_new);

    // ---- TEST 3: single bubble INSIDE the zeros region must be rejected -----
    errs_old = 0; errs_new = 0;
    for (f = 6; f <= W-6; f = f + 1)
      for (b = 2; b <= 4; b = b + 1) begin
        mk_ideal(f); raw[f+b] = 1'b1; #1;      // stray one, away from the edge
        if (pop(cor_old) !== f) errs_old = errs_old + 1;
        if (pop(cor_new) !== f) errs_new = errs_new + 1;
      end
    $display("TEST 3  1 stray bit inside zeros   OLD errors = %0d   NEW errors = %0d",
             errs_old, errs_new);

    // ---- TEST 4: cost of ones-padding -- stray bit at index 0..2 with fine=0
    promo_old = 0; promo_new = 0;
    for (k = 0; k <= 2; k = k + 1) begin
      raw = {W{1'b0}}; raw[k] = 1'b1; #1;
      if (pop(cor_old) != 0) promo_old = promo_old + 1;
      if (pop(cor_new) != 0) promo_new = promo_new + 1;
    end
    $display("TEST 4  isolated bubble at tap 0..2, true fine=0:");
    $display("        OLD promotes it %0d/3 times, NEW promotes it %0d/3 times",
             promo_old, promo_new);
    $finish;
  end
endmodule