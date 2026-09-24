# =============================================================================
# check_placement.tcl -- verify every LOC/BEL in tdl_loc.xdc against the
# currently open IMPLEMENTED design. Kept out of the XDC because XDC files
# reject foreach/puts.
#
# Usage (Vivado Tcl console, after open_run impl_1):
#   source {D:/vivado_work/TDC_code/single tdl/design_code_with_dsp_calibration/check_placement.tcl}
# =============================================================================
set xdc_path [file join [file dirname [info script]] tdl_loc.xdc]
set fh [open $xdc_path r]
set n_lines 0
set n_ok 0
set bad {}
array unset cells
array set cells {}

while {[gets $fh line] >= 0} {
    if {![regexp {^\s*set_property\s+(LOC|BEL)\s+(\S+)\s+\[get_cells\s+\{([^\}]+)\}\]} \
            $line -> prop want name]} { continue }
    incr n_lines
    set cells($name) 1
    set c [get_cells -quiet $name]
    if {[llength $c] != 1} { lappend bad "MISSING  $name"; continue }
    set got [get_property $prop $c]
    if {$got eq $want} { incr n_ok } else { lappend bad "$prop  $name  want=$want  got=$got" }
}
close $fh

set nfix 0
foreach name [array names cells] {
    set c [get_cells -quiet $name]
    if {[llength $c] == 1 && [get_property IS_LOC_FIXED $c]} { incr nfix }
}

puts "tdl_loc.xdc     : [array size cells] cells, $n_lines LOC/BEL constraints"
puts "matched         : $n_ok / $n_lines"
puts "LOC-fixed cells : $nfix / [array size cells]"
if {[llength $bad] == 0} {
    puts "PASS: every chain cell is exactly where tdl_loc.xdc put it"
} else {
    puts "FAIL: [llength $bad] mismatches (first 15):"
    foreach b [lrange $bad 0 14] { puts "  $b" }
}