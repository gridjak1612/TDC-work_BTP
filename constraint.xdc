# =============================================================================
# constraint_dual.xdc -- TWO-CHANNEL interval TDC, Digilent Boolean (xc7s50csga324-1)
# =============================================================================

set_property CFGBVS VCCO       [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

# ---- Clock: 100 MHz board oscillator -> MMCM -> 200 MHz ----------------------
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports clk100]
create_clock -period 10.000 -name clk100 [get_ports clk100]

# ---- Buttons ----------------------------------------------------------------
set_property -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS33} [get_ports rst]        ;# btn0
set_property -dict {PACKAGE_PIN J5 IOSTANDARD LVCMOS33} [get_ports btn_a]      ;# btn1  manual START
set_property -dict {PACKAGE_PIN H2 IOSTANDARD LVCMOS33} [get_ports btn_b]      ;# btn2  manual STOP
set_property -dict {PACKAGE_PIN J1 IOSTANDARD LVCMOS33} [get_ports btn_clear]  ;# btn3  manual re-arm

# ---- External event inputs (servo header) -----------------------------------
# !! 3.3 V MAXIMUM. A 5 V TTL or bipolar generator output will destroy the FPGA.
# !! Verify the generator amplitude BEFORE connecting. Use a divider if unsure.
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports ev_a_ext]  ;# servo0  START
set_property -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33} [get_ports ev_b_ext]  ;# servo1  STOP

# ---- Switch -----------------------------------------------------------------
set_property -dict {PACKAGE_PIN V2 IOSTANDARD LVCMOS33} [get_ports sw_autorearm]  ;# sw0

# ---- UART -------------------------------------------------------------------
set_property -dict {PACKAGE_PIN U11 IOSTANDARD LVCMOS33} [get_ports uart_txd]

# ---- LEDs -------------------------------------------------------------------
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

# =============================================================================
# TIMING EXCEPTIONS
# =============================================================================
# The event inputs feed the CARRY4 chains directly (256 carry levels). That path
# is the MEASUREMENT, not a logic path -- it must not be timed. It is also the
# async input to the CDC synchronisers, which is handled by ASYNC_REG below.
set_false_path -from [get_ports ev_a_ext]
set_false_path -from [get_ports ev_b_ext]
set_false_path -from [get_ports btn_a]
set_false_path -from [get_ports btn_b]
set_false_path -from [get_ports btn_clear]
set_false_path -from [get_ports rst]
set_false_path -from [get_ports sw_autorearm]
set_false_path -to   [get_ports {led[*]}]
set_false_path -to   [get_ports uart_txd]

# CDC synchroniser flops -- place adjacent, do not retime. BOTH channels.
set_property ASYNC_REG TRUE [get_cells -hierarchical -filter {NAME =~ *cap_ctrl_inst/stop_sync_reg*}]
set_property ASYNC_REG TRUE [get_cells -hierarchical -filter {NAME =~ *cap_ctrl_inst/clr_sync_reg*}]

# =============================================================================
# FLOORPLAN -- STILL TO DO (see notes)
# =============================================================================
# Each carry chain must occupy a single contiguous vertical carry column, with
# its tap flip-flops in the adjacent slices. Without this the placer scatters
# the chain and the bin widths become non-uniform AND non-reproducible between
# builds. Two chains -> two separate pblocks, in DIFFERENT columns.
#
# create_pblock pblock_tdl_a
# add_cells_to_pblock [get_pblocks pblock_tdl_a] [get_cells -hier -filter {NAME =~ *chan_a/tdl_inst/*}]
# resize_pblock [get_pblocks pblock_tdl_a] -add {SLICE_X10Y0:SLICE_X10Y63}
#
# create_pblock pblock_tdl_b
# add_cells_to_pblock [get_pblocks pblock_tdl_b] [get_cells -hier -filter {NAME =~ *chan_b/tdl_inst/*}]
# resize_pblock [get_pblocks pblock_tdl_b] -add {SLICE_X14Y0:SLICE_X14Y63}
#
# Pick the actual SLICE ranges from the device view -- they must be columns that
# contain 64 vertically-adjacent CARRY4 sites.
