# =============================================================================
# rp.xdc -- return-path probe (rp_board). Standalone build, own Vivado project.
# =============================================================================
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports clk100]
create_clock -period 10.000 -name clk100 [get_ports clk100]

set_property -dict {PACKAGE_PIN J2  IOSTANDARD LVCMOS33} [get_ports rst]
set_property -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports uart_txd]
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN G2 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN F1 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN F2 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN E1 IOSTANDARD LVCMOS33} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN E2 IOSTANDARD LVCMOS33} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN E5 IOSTANDARD LVCMOS33} [get_ports {led[7]}]
set_property -dict {PACKAGE_PIN E6 IOSTANDARD LVCMOS33} [get_ports {led[8]}]
set_property -dict {PACKAGE_PIN C3 IOSTANDARD LVCMOS33} [get_ports {led[9]}]
set_property -dict {PACKAGE_PIN B2 IOSTANDARD LVCMOS33} [get_ports {led[10]}]
set_property -dict {PACKAGE_PIN A2 IOSTANDARD LVCMOS33} [get_ports {led[11]}]
set_property -dict {PACKAGE_PIN B3 IOSTANDARD LVCMOS33} [get_ports {led[12]}]
set_property -dict {PACKAGE_PIN A3 IOSTANDARD LVCMOS33} [get_ports {led[13]}]
set_property -dict {PACKAGE_PIN B4 IOSTANDARD LVCMOS33} [get_ports {led[14]}]
set_property -dict {PACKAGE_PIN A4 IOSTANDARD LVCMOS33} [get_ports {led[15]}]
set_false_path -from [get_ports rst]
set_false_path -to   [get_ports {led[*]}]
set_false_path -to   [get_ports uart_txd]

# The six rings are deliberate combinational loops (clears LUTLP-1).
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hier -filter {NAME =~ u_ring*/co*}]
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hier -filter {NAME =~ u_ring*/s0}]

# Divider clocks. 2.0 ns is tighter than any ring should run (fastest, K = 32,
# is expected around 400-500 MHz): it only makes Vivado prove the 5-bit
# divider keeps up. It does NOT set the ring frequency.
create_clock -period 2.000 -name rp_clk0 [get_pins u_ring0/u_obs/O]
create_clock -period 2.000 -name rp_clk1 [get_pins u_ring1/u_obs/O]
create_clock -period 2.000 -name rp_clk2 [get_pins u_ring2/u_obs/O]
create_clock -period 2.000 -name rp_clk3 [get_pins u_ring3/u_obs/O]
create_clock -period 2.000 -name rp_clk4 [get_pins u_ring4/u_obs/O]
create_clock -period 2.000 -name rp_clk5 [get_pins u_ring5/u_obs/O]
set_clock_groups -asynchronous \
    -group [get_clocks clk100] \
    -group [get_clocks rp_clk0] -group [get_clocks rp_clk1] -group [get_clocks rp_clk2] \
    -group [get_clocks rp_clk3] -group [get_clocks rp_clk4] -group [get_clocks rp_clk5]

# Expected, harmless in this build:
#   methodology warnings: LUT output drives flip-flop clock pins (u_obs -> div)
