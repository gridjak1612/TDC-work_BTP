# =============================================================================
# constraint_dual.xdc -- TWO-CHANNEL interval TDC, Digilent Boolean (xc7s50csga324-1)
# =============================================================================

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

# ---- Clock: 100 MHz board oscillator -> MMCM -> 200 MHz + 25 MHz cal ---------
set_property -dict {PACKAGE_PIN F14 IOSTANDARD LVCMOS33} [get_ports clk100]
create_clock -period 10.000 -name clk100 [get_ports clk100]

# ---- Calibration clock is ASYNCHRONOUS to the sampler for timing -------------
# clk_out1 (200 MHz sampler) and clk_out2 (25 MHz cal) share the VCO, so Vivado
# treats them as related and WILL try to time paths between them. But every path
# between the two domains is intentional and must NOT be timed:
#   - the cal edge launches the CARRY4 chain (the measurement, a ~5.6 ns line)
#   - the cal edge is the async input to the capture-controller CDC synchronisers
# Grouping them asynchronous false-paths both. Nothing is clocked BY clk_out2, so
# there is no real synchronous register path to protect -- this is safe.
set_clock_groups -asynchronous -group [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *mmcm_adv_inst/CLKOUT0}]] -group [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *mmcm_adv_inst/CLKOUT1}]]
# If the pin filter returns nothing on your build, run report_clocks and use the
# generated-clock names directly, e.g.:
#   set_clock_groups -asynchronous -group [get_clocks clk_out1] -group [get_clocks clk_out2]

# ---- Buttons ----------------------------------------------------------------
# In the calibration build (EVENT_SRC=2) btn_a/btn_b no longer carry events:
#   btn_a -> ps_step_btn (one press = one 17.857 ps phase step)
#   btn_b -> ps_dir_btn  (held = decrement)
set_property -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS33} [get_ports rst]
set_property -dict {PACKAGE_PIN J5 IOSTANDARD LVCMOS33} [get_ports btn_a]
set_property -dict {PACKAGE_PIN H2 IOSTANDARD LVCMOS33} [get_ports btn_b]
set_property -dict {PACKAGE_PIN J1 IOSTANDARD LVCMOS33} [get_ports btn_clear]

# ---- External event inputs (servo header) -----------------------------------
# !! 3.3 V MAXIMUM. A 5 V TTL or bipolar generator output will destroy the FPGA.
# !! Verify the generator amplitude BEFORE connecting. Use a divider if unsure.
# Unused in the calibration build, but keep them constrained/false-pathed.
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33} [get_ports ev_a_ext]
set_property -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33} [get_ports ev_b_ext]

# ---- Switch -----------------------------------------------------------------
set_property -dict {PACKAGE_PIN V2 IOSTANDARD LVCMOS33} [get_ports sw_autorearm]

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
# The event inputs feed the CARRY4 chains directly (352 carry levels). That path
# is the MEASUREMENT, not a logic path -- it must not be timed. It is also the
# async input to the CDC synchronisers, which is handled by ASYNC_REG below.
set_false_path -from [get_ports ev_a_ext]
set_false_path -from [get_ports ev_b_ext]
set_false_path -from [get_ports btn_a]
set_false_path -from [get_ports btn_b]
set_false_path -from [get_ports btn_clear]
set_false_path -from [get_ports rst]
set_false_path -from [get_ports sw_autorearm]
set_false_path -to [get_ports {led[*]}]
set_false_path -to [get_ports uart_txd]

# CDC synchroniser flops -- place adjacent, do not retime. BOTH channels.
set_property ASYNC_REG true [get_cells -hierarchical -filter {NAME =~ *cap_ctrl_inst/stop_sync_reg*}]
set_property ASYNC_REG true [get_cells -hierarchical -filter {NAME =~ *cap_ctrl_inst/clr_sync_reg*}]

# =============================================================================
# FLOORPLAN -- STILL TO DO (do this AFTER the phase-sweep proof passes)
# =============================================================================
# Each carry chain must occupy a single contiguous vertical carry column, with
# its tap flip-flops in the adjacent slices. Without this the placer scatters
# the chain and the bin widths become non-uniform AND non-reproducible between
# builds. Two chains -> two separate pblocks, in DIFFERENT columns.
# Each column must hold 88 vertically-adjacent CARRY4 sites.
#
# create_pblock pblock_tdl_a
# add_cells_to_pblock [get_pblocks pblock_tdl_a] [get_cells -hier -filter {NAME =~ *chan_a/tdl_inst/*}]
# resize_pblock [get_pblocks pblock_tdl_a] -add {SLICE_X10Y0:SLICE_X10Y87}
#
# create_pblock pblock_tdl_b
# add_cells_to_pblock [get_pblocks pblock_tdl_b] [get_cells -hier -filter {NAME =~ *chan_b/tdl_inst/*}]
# resize_pblock [get_pblocks pblock_tdl_b] -add {SLICE_X14Y0:SLICE_X14Y87}
#
# Pick the actual SLICE ranges from the device view.
# =============================================================================
# FLOORPLAN -- lock each TDL into its own contiguous CARRY4 column.
# Ranges taken from the achieved placement (impl_1): chan_a=X36Y6..Y93,
# chan_b=X46Y7..Y94. Locking makes bin widths reproducible across builds so a
# calibration LUT stays valid. CONTAIN_ROUTING keeps the chain's routing inside
# the column too. Different columns (X36 vs X46) -> no interaction.
# =============================================================================

