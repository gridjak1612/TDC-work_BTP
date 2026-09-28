# =============================================================================
# check_placement_fold.tcl -- FOLD builds. After open_run impl_1:
#   source {D:/vivado_work/TDC_code/single tdl/design_code_with_dsp_calibration/code/check_placement_fold.tcl}
# Verifies every LOC/BEL in tdl_loc_fold.xdc, that each return LUT sits in its
# chain's B slice (CARRY4 #8), and prints the WNS.
# =============================================================================
set xdc_path [file join [file dirname [info script]] tdl_loc_fold.xdc]
set fh [open $xdc_path r]
set n_lines 0
set n_ok 0
set bad {}
while {[gets $fh line] >= 0} {
    if {![regexp {^\s*set_property\s+(LOC|BEL)\s+(\S+)\s+\[get_cells\s+\{([^\}]+)\}\]} \
            $line -> prop want name]} { continue }
    incr n_lines
    set c [get_cells -quiet $name]
    if {[llength $c] != 1} { lappend bad "MISSING  $name"; continue }
    set got [get_property $prop $c]
    if {$got eq $want} { incr n_ok } else { lappend bad "$prop  $name  want=$want  got=$got" }
}
close $fh
puts "tdl_loc_fold.xdc: $n_lines LOC/BEL constraints, matched $n_ok"
foreach ch {chan_a chan_b} {
    set b   [get_cells -quiet "core/$ch/tdl_inst/g_c4\[8\].u_c4"]
    set ret [get_cells -quiet core/$ch/tdl_inst/u_ret]
    if {[llength $b] != 1 || [llength $ret] != 1} { lappend bad "$ch: B CARRY4 or u_ret not found"; continue }
    set bl [get_property LOC $b]
    set rl [get_property LOC $ret]
    puts "$ch: B at $bl, u_ret at $rl ([get_property BEL $ret])"
    if {$bl ne $rl} { lappend bad "$ch: u_ret not in the B slice" }
}
puts "impl_1 WNS: [get_property STATS.WNS [get_runs impl_1]] ns"
if {[llength $bad] == 0} {
    puts "PASS: fold placement is exactly as constrained"
} else {
    foreach b $bad { puts "  $b" }
    puts "FAIL: [llength $bad] problems"
}
