# ============================================================================
# batch_run_classifier.do — 新缺陷分类器仿真 (defect_ac_classifier)
# 用法 (工程根目录 God3.0 下): vsim -c -do "do sim/batch_run_classifier.do"
#   权重头文件 rtl/defect_lda_weights.vh 由 tools/fpga_model.py 生成
#   测试向量 sim/lda_vectors/hit_NN.memh 亦由该脚本生成
# ============================================================================

if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

puts ">>> Compiling DUT (with +incdir+rtl for defect_lda_weights.vh)..."
vlog -work work +incdir+rtl rtl/defect_ac_classifier.v

# ROM 初始化文件 readmemh 按仿真工作目录(God3.0)解析, 拷入当前目录
puts ">>> Staging ROM init file (defect_lda_weights.mem)..."
file copy -force rtl/defect_lda_weights.mem defect_lda_weights.mem

puts ">>> Compiling TB..."
vlog -work work sim/tb_defect_ac_classifier.v

puts ">>> Running tb_defect_ac_classifier..."
vsim -t 1ns -voptargs="+acc" -L work work.tb_defect_ac_classifier
onfinish stop
run -all

quit -f
