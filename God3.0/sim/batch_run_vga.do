# ============================================================================
# batch_run_vga.do — 显示链 (buffer+precision display) 仿真批处理脚本
#   对应审查建议第 5 条: 高精度 VGA 显示模块仿真
# 用法 (工程根目录下): vsim -c -do "do sim/batch_run_vga.do"
# ============================================================================

# ---- 1. 创建 work 库 ----
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

# ---- 2. 编译 DUT ----
puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/wave_buffer_dp.v
vlog -work work -novopt rtl/vga_waveform_precision.v

# ---- 3. 编译 TB ----
puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_vga_waveform_precision.v

# ---- 4. 启动仿真 ----
puts ">>> Starting simulation..."
vsim -t 1ns -L work work.tb_vga_waveform_precision

run -all
quit -f
