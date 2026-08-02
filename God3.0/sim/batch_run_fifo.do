# ============================================================================
# run_fifo_sim_batch.do — FIFO 读对齐仿真批处理脚本
#
# 依赖: altera_mf 库 (dcfifo 原语, Verilog 版本)
# ============================================================================

# ---- 1. 创建 work 库 ----
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

# ---- 2. 编译 altera_mf 库 (Verilog 版本) ----
# ModelSim-Altera 自带 altera_mf Verilog 源码
set ALTERA_MF_VER "C:/intelFPGA_lite/18.1/modelsim_ase/altera/verilog/src"

puts ">>> Compiling altera_mf (Verilog)..."
vlog -work work -novopt "$ALTERA_MF_VER/altera_mf.v"

# ---- 3. 编译 DUT ----
puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/audio_fifo.v
vlog -work work -novopt rtl/fifo_sample_reader.v

# ---- 4. 编译 TB ----
puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_fifo_sample_reader.v

# ---- 5. 启动仿真 ----
puts ">>> Starting simulation..."
vsim -t 1ns -L work -L altera_mf work.tb_fifo_sample_reader

run -all
quit -f
