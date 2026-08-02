# ============================================================================
# batch_run_chain.do — 全链路集成仿真 (触发→平滑→采集→RAM→UART)
# 用法 (工程根目录下): vsim -c -do "do sim/batch_run_chain.do"
# ============================================================================

# ---- 1. 创建 work 库 ----
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

# ---- 2. 编译 DUT ----
puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/wave_trigger.v
vlog -work work -novopt rtl/wave_smooth.v
vlog -work work -novopt rtl/wave_capture.v
vlog -work work -novopt rtl/wave_buffer_dp.v
vlog -work work -novopt rtl/uart_tx.v
vlog -work work -novopt rtl/wave_uart_export.v

# ---- 3. 编译 TB ----
puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_wave_chain.v

# ---- 4. 运行 ----
puts ">>> Running tb_wave_chain..."
vsim -t 1ns -L work work.tb_wave_chain
onfinish stop
run -all

quit -f
