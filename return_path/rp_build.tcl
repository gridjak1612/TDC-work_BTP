# =============================================================================
# rp_build.tcl -- creates the SEPARATE Vivado project for the return-path probe.
# The TDC project is not touched. Run ONCE from the Vivado Tcl console:
#   source {D:/vivado_work/TDC_code/single tdl/design_code_with_dsp_calibration/return_path/rp_build.tcl}
# =============================================================================
set here     [file normalize [file dirname [info script]]]
set proj_dir D:/vivado_work/rp_probe

if {[file exists [file join $proj_dir rp_probe.xpr]]} {
    error "rp_probe.xpr already exists -- open it with open_project instead of re-running this"
}
create_project rp_probe $proj_dir -part xc7s50csga324-1

add_files -norecurse [list \
    [file join $here rp_ring.v] \
    [file join $here rp_board.v] \
    [file normalize [file join $here .. code uart.v]]]
add_files -fileset constrs_1 -norecurse [list \
    [file join $here rp.xdc] \
    [file join $here rp_loc.xdc]]
set_property top rp_board [get_filesets sources_1]

add_files -fileset sim_1 -norecurse [file join $here tb_rp.v]
set_property top tb_rp [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
puts "rp_probe project created in $proj_dir"
