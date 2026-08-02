# ============================================================================
# batch_run_trigger.do -- wave_trigger unit test (frozen plan 4.2 noise_ema)
# Usage (from project root): vsim -c -do "do sim/batch_run_trigger.do"
# ============================================================================

if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/wave_trigger.v

puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_wave_trigger.v

puts ">>> Running tb_wave_trigger..."
vsim -t 1ns -L work work.tb_wave_trigger
onfinish stop
run -all

quit -f
