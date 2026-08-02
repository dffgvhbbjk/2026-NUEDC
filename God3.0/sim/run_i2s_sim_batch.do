# ============================================================================
# run_i2s_sim_batch.do — PCM1808 I2S 接收器批处理仿真脚本 (无 GUI)
#
# 在命令行模式下运行, 不打开 GUI, 直接输出结果到 stdout
# 用法 (工程根目录下): vsim -c -do "do sim/run_i2s_sim_batch.do"
# ============================================================================
# 注: 不能用 [info script] 推导路径 — vsim -do 方式下会解析到 ModelSim
#     安装目录, 导致找不到 rtl/。改为校验当前目录是否为工程根。
if {![file exists rtl/pcm1808_i2s_rx.v]} {
    puts "ERROR: 未找到 rtl/pcm1808_i2s_rx.v, 请在工程根目录下启动:"
    puts "       vsim -c -do \"do sim/run_i2s_sim_batch.do\""
    quit -f
}

puts "=========================================="
puts "PCM1808 I2S Receiver Simulation (batch)"
puts "Working dir: [pwd]"
puts "=========================================="

# ---- 1. 创建 work 库 ----
if {[file exists work]} {
    vdel -lib work -all
}
vlib work
vmap work work

# ---- 2. 编译源文件 ----
puts "\n>>> Compiling RTL..."
if {[catch {vlog -work work -novopt rtl/pcm1808_i2s_rx.v} err]} {
    puts "ERROR: compile rtl failed: $err"
    quit -f
}

puts ">>> Compiling Testbench..."
if {[catch {vlog -work work -novopt sim/tb_pcm1808_i2s_rx.v} err]} {
    puts "ERROR: compile tb failed: $err"
    quit -f
}

# ---- 3. 启动仿真 ----
# 外层已经由 vsim -c 执行本脚本，此处不能再次嵌套启动 vsim -c。
puts "\n>>> Starting simulation..."
vsim -t 1ns -L work work.tb_pcm1808_i2s_rx

# ---- 4. 运行 ----
puts "\n>>> Running (1000 random frames, may take 1-2 minutes)..."
run -all

# ---- 5. 结束 ----
puts "\n=========================================="
puts "Simulation finished. See output above for PASS/FAIL."
puts "=========================================="
quit -f
