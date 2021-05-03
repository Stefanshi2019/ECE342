cd software/fpmult_sw/obj/default/runtime/sim/mentor/
file copy -force ../../../../../../../part3_tester.sv ./part3_tester.sv
do load_sim.tcl
dev_com
com
vlog -work work part3_tester.sv
elab
run 3ms
exit
