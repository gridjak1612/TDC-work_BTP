# =============================================================================
# ro_multi.xdc -- ONLY for EVENT_SRC = 4 (run-time select) builds.
# In these builds DISABLE ro.xdc; in every other build DISABLE this file.
# =============================================================================

# Two deliberate combinational loops.
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hier -filter {NAME =~ *ro7_inst/n[*]}]
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hier -filter {NAME =~ *ro11_inst/n[*]}]

# Divider clocks (the rings' own frequency is NOT set by these).
create_clock -name ro7_clk  -period 3.000 [get_pins -hier -filter {NAME =~ *ro7_inst/u_tap/O}]
create_clock -name ro11_clk -period 3.000 [get_pins -hier -filter {NAME =~ *ro11_inst/u_tap/O}]
set_clock_groups -asynchronous \
    -group [get_clocks ro7_clk] \
    -group [get_clocks ro11_clk] \
    -group [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *mmcm_adv_inst/CLKOUT0}]] \
    -group [get_clocks -of_objects [get_pins -hier -filter {NAME =~ *mmcm_adv_inst/CLKOUT1}]]

# The source select is static during a run. Its path runs select flop -> mux
# LUT -> CYINIT -> 352 carry stages -> tap flops (~6 ns): it must not be timed.
set_false_path -from [get_cells -hier -filter {NAME =~ *sw_s2_reg*}]
set_property ASYNC_REG true [get_cells -hier -filter {NAME =~ *sw_s1_reg*}]
set_property ASYNC_REG true [get_cells -hier -filter {NAME =~ *sw_s2_reg*}]

# Keep both rings in the left corner, far from the chains; one pblock each.
create_pblock pb_ro7
add_cells_to_pblock pb_ro7 [get_cells -hier -filter {NAME =~ *ro7_inst/* && IS_PRIMITIVE}]
resize_pblock pb_ro7 -add {SLICE_X0Y125:SLICE_X11Y149}
create_pblock pb_ro11
add_cells_to_pblock pb_ro11 [get_cells -hier -filter {NAME =~ *ro11_inst/* && IS_PRIMITIVE}]
resize_pblock pb_ro11 -add {SLICE_X0Y100:SLICE_X11Y124}