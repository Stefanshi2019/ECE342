cd software/leds_sw/obj/default/runtime/sim/mentor/
file copy -force ../../../../../../../part1_tester.sv ./part1_tester.sv
do load_sim.tcl
dev_com
com
vlog -work work part1_tester.sv
elab
run 1ms
exit
