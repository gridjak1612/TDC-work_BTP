# =============================================================================
# ro.xdc -- ONLY for EVENT_SRC = 3 builds.
# For every other build: right-click this file in Vivado > Disable File.
# =============================================================================

# The ring is a deliberate combinational loop (clears DRC LUTLP-1).
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hier -filter {NAME =~ *ro_inst/n[*]}]

# Clock the divider is timed against. 3.0 ns is deliberately faster than the
# expected ~150 MHz ring. It does NOT set the ring frequency.
create_clock -name ro_clk -period 3.000 [get_pins -hier -filter {NAME =~ *ro_inst/u_tap/O}]
set_clock_groups -asynchronous \
    -group [get_clocks ro_clk] \
    -group [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *mmcm_adv_inst/CLKOUT0}]]

# Keep the ring far from both chains (chan_a X37 Y30-117, chan_b X48 Y31-118).
# If Vivado rejects this range, pick any ~12x25 SLICE block far from the
# chains in the Device view.
create_pblock pb_ro
add_cells_to_pblock pb_ro [get_cells -hier -filter {NAME =~ *ro_inst/* && IS_PRIMITIVE}]
resize_pblock pb_ro -add {SLICE_X0Y125:SLICE_X11Y149}

# Expected, harmless in this build:
#   methodology warning: LUT output drives flip-flop clock pins (u_tap -> div)