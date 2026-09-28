# =============================================================================
# rp_check.tcl -- after open_run impl_1 in the rp_probe project:
#   source {D:/vivado_work/TDC_code/single tdl/design_code_with_dsp_calibration/return_path/rp_check.tcl}
# Checks every ring: CARRY4 count, one column, contiguous Y50.., one clock
# region, u_ret in the slice of CARRY4 #0. Then prints the static-timing
# delay of the D -> u_ret.I0 route (model estimate, for comparison only).
# =============================================================================
set ks   {32 48 64 96 128 192}
set fail 0
for {set i 0} {$i < 6} {incr i} {
    set k   [lindex $ks $i]
    set nc4 [expr {$k / 4 + 1}]
    set c4  [get_cells -quiet -hier -filter "REF_NAME == CARRY4 && NAME =~ u_ring${i}/*"]
    set ys {}
    set xs {}
    set regions {}
    foreach c $c4 {
        set loc [get_property LOC $c]
        regexp {SLICE_X(\d+)Y(\d+)} $loc -> x y
        lappend xs $x
        lappend ys $y
        lappend regions [get_property CLOCK_REGION [get_sites $loc]]
    }
    set ys [lsort -integer $ys]
    set xs [lsort -unique $xs]
    set regions [lsort -unique $regions]
    set ymin [lindex $ys 0]
    set ymax [lindex $ys end]
    set ret  [get_cells -quiet u_ring${i}/u_ret]
    set rloc [get_property LOC $ret]
    set ok 1
    if {[llength $c4] != $nc4}                 { set ok 0; puts "  ring $i: [llength $c4] CARRY4, expected $nc4" }
    if {[llength $xs] != 1 || $xs != [expr {37 + $i}]} { set ok 0; puts "  ring $i: columns $xs, expected [expr {37 + $i}]" }
    if {$ymin != 50 || $ymax != [expr {50 + $nc4 - 1}]} { set ok 0; puts "  ring $i: Y $ymin..$ymax" }
    if {[llength $regions] != 1}               { set ok 0; puts "  ring $i: spans clock regions $regions" }
    if {$rloc ne "SLICE_X[lindex $xs 0]Y50"}   { set ok 0; puts "  ring $i: u_ret at $rloc, not in CARRY4 #0 slice" }
    puts [format "ring %d  K=%3d  X%s Y%s..%s  region %s  u_ret %s/%s  %s" \
          $i $k $xs $ymin $ymax $regions $rloc [get_property BEL $ret] [expr {$ok ? "OK" : "FAIL"}]]
    if {!$ok} { set fail 1 }

    # Static-timing model of the return route D -> u_ret.I0 (not a measurement).
    set pin [get_pins u_ring${i}/u_ret/I0]
    set net [get_nets -of_objects $pin]
    if {[catch {
        set d [get_net_delays -of_objects $net -to $pin -interconnect_only]
        puts "    route D->u_ret.I0 (model):"
        report_property $d
    } msg]} { puts "    get_net_delays unavailable here: $msg" }
}
puts "impl_1 WNS: [get_property STATS.WNS [get_runs impl_1]] ns"
puts [expr {$fail ? "RP_CHECK FAIL" : "RP_CHECK PASS"}]
