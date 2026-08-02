# ============================================================================
# batch_run_capture.do — 采集链 (trigger+capture+buffer) 仿真批处理脚本
#   对应文档 §17.3 采集状态机仿真
# ============================================================================

# ---- 1. 创建 work 库 ----
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

# ---- 2. 编译 DUT ----
puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/wave_trigger.v
vlog -work work -novopt rtl/wave_capture.v
vlog -work work -novopt rtl/wave_buffer_dp.v

# ---- 3. 编译 TB ----
puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_wave_capture.v

# ---- 4. 启动仿真 ----
puts ">>> Starting simulation..."
vsim -t 1ns -L work work.tb_wave_capture

run -all
quit -f
