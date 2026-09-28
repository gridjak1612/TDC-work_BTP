# =============================================================================
# fold.xdc -- FOLD builds only (enable together with tdl_loc_fold.xdc; disable
# tdl_loc.xdc). The two folding loops are deliberate combinational loops.
# =============================================================================
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hier -filter {NAME =~ *tdl_inst/co[*]}]
set_property ALLOW_COMBINATORIAL_LOOPS TRUE [get_nets -hier -filter {NAME =~ *tdl_inst/s0}]
# loop_en (clk200 flop) -> u_ret -> chain: never a timed path.
set_false_path -from [get_cells -hier -filter {NAME =~ *chan_*/loop_en_reg}]
