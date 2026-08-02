# ============================================================================
# batch_run_overlay.do — VGA 状态叠加层 (vga_status_overlay) 仿真批处理脚本
#   对应第一版方案 §VGA显示布局: 状态栏文字 + 标记线 + BCD 转换
# 用法 (工程根目录下): vsim -c -do "do sim/batch_run_overlay.do"
# ============================================================================

# ---- 1. 创建 work 库 ----
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

# ---- 2. 编译 DUT ----
puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/vga_status_overlay.v

# ---- 3. 编译 TB ----
puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_vga_status_overlay.v

# ---- 4. 启动仿真 ----
puts ">>> Starting simulation..."
vsim -t 1ps -L work work.tb_vga_status_overlay

run -all
quit -f
