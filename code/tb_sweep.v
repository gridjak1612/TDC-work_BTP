`timescale 1ns/1ps
module tb_sweep;
  reg clk100=0, rst=1, btn_a=0, btn_b=0, btn_clear=0;
  reg ev_a=0, ev_b=0, sw_autorearm=1;
  wire [15:0] led; wire txd;
  always #5 clk100 = ~clk100;

  localparam CPB = 4;                 // tiny baud for sim speed

  tdc_dual_board #(
      .CLKS_PER_BIT(CPB), .EVENT_SRC(2), .TIE_CHANNELS(0),
      .AUTO_SWEEP(1), .SWEEP_STEPS(5), .SAMPLES_PER_STEP(3), .SETTLE_CYCLES(4)
  ) dut (
      .clk100(clk100), .rst(rst), .btn_a(btn_a), .btn_b(btn_b),
      .btn_clear(btn_clear), .ev_a_ext(ev_a), .ev_b_ext(ev_b),
      .sw_autorearm(sw_autorearm), .led(led), .uart_txd(txd));

  // ---- UART receiver: decode txd back into bytes ----------------------------
  localparam real BIT_NS = CPB * 5.0;
  integer nbytes = 0;
  reg [7:0] rxb [0:2047];
  initial begin : rxproc
    integer i; reg [7:0] d;
    forever begin
      @(negedge txd);
      #(BIT_NS*1.5);
      for (i=0;i<8;i=i+1) begin d[i] = txd; #(BIT_NS); end
      rxb[nbytes] = d; nbytes = nbytes + 1;
    end
  end

  // ---- phase-trajectory logger ----------------------------------------------
  integer nph = 0; reg signed [11:0] phlog [0:255];
  reg signed [11:0] ph_prev;
  initial ph_prev = 0;
  always @(posedge dut.clk200) begin
    if (dut.ps_phase_idx !== ph_prev) begin
      phlog[nph] = dut.ps_phase_idx; nph = nph + 1;
      ph_prev = dut.ps_phase_idx;
    end
  end

  // ---- count measurements per phase ------------------------------------------
  integer meas_at_phase [0:15]; integer i;
  initial for (i=0;i<16;i=i+1) meas_at_phase[i] = 0;
  always @(posedge dut.clk200)
    if (dut.meas_ready && dut.ps_phase_idx < 16)
      meas_at_phase[dut.ps_phase_idx] = meas_at_phase[dut.ps_phase_idx] + 1;

  integer k, f0;
  initial begin
    repeat (40) @(posedge clk100); rst = 0;
    #62000;

    $display("phase trajectory (%0d transitions):", nph);
    $write("   0");
    for (k=0;k<nph && k<40;k=k+1) $write(" %0d", phlog[k]);
    $write("\n");
    $display("   expect triangle 0 1 2 3 4 3 2 1 0 1 2 ...");

    $display("");
    $display("measurements per phase (expect 3 each, endpoints may differ):");
    for (k=0;k<6;k=k+1) $display("   phase %0d : %0d", k, meas_at_phase[k]);

    $display("");
    $display("uart bytes captured : %0d  (%0d whole frames)", nbytes, nbytes/8);
    // find first header and dump two frames
    f0 = -1;
    for (k=0;k+15<nbytes;k=k+1)
      if (rxb[k]==8'hA5 && rxb[k+8]==8'hA5 && f0<0) f0=k;
    if (f0 >= 0) begin
      for (k=f0;k<f0+24 && k<nbytes;k=k+8)
        $display("   frame: %02h %02h %02h %02h %02h %02h %02h %02h  -> fa=%0d fb=%0d ph=%0d seq=%0d va=%0d vb=%0d",
          rxb[k],rxb[k+1],rxb[k+2],rxb[k+3],rxb[k+4],rxb[k+5],rxb[k+6],rxb[k+7],
          {rxb[k+4][1],rxb[k+1]}, {rxb[k+4][0],rxb[k+2]},
          {rxb[k+6][3:0],rxb[k+5]}, rxb[k+7],
          rxb[k+6][4], rxb[k+6][5]);
    end else $display("   NO ALIGNED HEADER FOUND");

    $display("");
    $display("led[7] (sweep integrity fault) = %0d   expect 0", led[7]);
    $display("sw_mismatch=%0d  ps_error=%0d", dut.sw_mismatch, dut.ps_error);
    $finish;
  end
endmodule
