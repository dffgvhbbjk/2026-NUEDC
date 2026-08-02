if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work
vlog -work work -novopt rtl/pcm1808_i2s_rx.v
vlog -work work -novopt sim/tb_pcm1808_i2s_rx.v
vsim -t 1ns -L work work.tb_pcm1808_i2s_rx
run -all
quit -f
