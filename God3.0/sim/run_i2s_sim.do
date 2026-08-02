# ============================================================================
# run_i2s_sim.do — PCM1808 I2S 接收器一键仿真脚本
#
# 用法:
#   方式 1 (推荐): 在 ModelSim 命令行执行
#       vsim -do sim/run_i2s_sim.do
#
#   方式 2: 在 ModelSim GUI 中
#       File → Change Directory → 切到工程根目录 (God3.0/God3.0)
#       在 Transcript 窗口输入: do sim/run_i2s_sim.do
#
#   方式 3: 双击 sim/run_i2s_sim.bat (Windows 批处理)
#
# 脚本自动完成:
#   1. 创建/清理 work 库
#   2. 编译 rtl/pcm1808_i2s_rx.v 和 sim/tb_pcm1808_i2s_rx.v
#   3. 启动仿真
#   4. 运行到 TB 自动 $finish
#   5. 打印结果 (PASS/FAIL)
# ============================================================================

# ModelSim 的 [info script] 在不同启动方式下可能指向安装目录，
# 因此不自动 cd；要求调用者从工程根目录运行，并在此明确校验。
if {![file exists rtl/pcm1808_i2s_rx.v]} {
    puts "ERROR: 未找到 rtl/pcm1808_i2s_rx.v。"
    puts "请先切换到工程根目录，再执行: do sim/run_i2s_sim.do"
    return
}

puts "=========================================="
puts "PCM1808 I2S Receiver Simulation"
puts "Working dir: [pwd]"
puts "=========================================="

# ---- 1. 创建/重建 work 库 ----
if {[file exists work]} {
    vdel -lib work -all
    puts "Deleted existing work library"
}
vlib work
vmap work work
puts "Created work library"

# ---- 2. 编译源文件 ----
puts "\n>>> Compiling RTL..."
if {[catch {vlog -work work +acc rtl/pcm1808_i2s_rx.v} err]} {
    puts "ERROR: Failed to compile rtl/pcm1808_i2s_rx.v: $err"
    return
}

puts "\n>>> Compiling Testbench..."
if {[catch {vlog -work work +acc sim/tb_pcm1808_i2s_rx.v} err]} {
    puts "ERROR: Failed to compile sim/tb_pcm1808_i2s_rx.v: $err"
    return
}

# ---- 3. 启动仿真 ----
puts "\n>>> Starting simulation..."
vsim -t 1ns -L work work.tb_pcm1808_i2s_rx -voptargs="+acc"

# ---- 4. 添加波形 (可选, 方便观察) ----
if {[catch {add wave -divider {Clocks & Reset}} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/bclk} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/rst_n} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/lrclk} err]} {}

if {[catch {add wave -divider {I2S Input}} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/sdin} err]} {}

if {[catch {add wave -divider {DUT Outputs}} err]} {}
if {[catch {add wave -format Hexadecimal /tb_pcm1808_i2s_rx/left_data} err]} {}
if {[catch {add wave -format Hexadecimal /tb_pcm1808_i2s_rx/right_data} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/left_valid} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/right_valid} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/frame_error} err]} {}

if {[catch {add wave -divider {Internal (DUT)}} err]} {}
if {[catch {add wave -format Logic /tb_pcm1808_i2s_rx/DUT/channel} err]} {}
if {[catch {add wave -format Literal -radix unsigned /tb_pcm1808_i2s_rx/DUT/slot_count} err]} {}
if {[catch {add wave -format Hexadecimal /tb_pcm1808_i2s_rx/DUT/shift_reg} err]} {}

# ---- 5. 运行仿真 ----
puts "\n>>> Running simulation (this may take a while for 1000 random frames)..."
run -all

# ---- 6. 结束 ----
puts "\n=========================================="
puts "Simulation finished."
puts "Check the output above for PASS/FAIL result."
puts "=========================================="

# GUI 入口保留仿真窗口和波形；无 GUI 回归请使用 run_i2s_sim_batch.do。
