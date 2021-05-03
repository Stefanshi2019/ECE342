vlib work
vlog -work work part2_tester.sv
vlog -work work part2.sv
vlog -work work fp_mult.v
vsim -L work -L altera_mf_ver -L lpm_ver part2_tester
run 600ns
exit
