## =====================================================================
## run.do — 仿真当前 ov5640_config (官方 i2c_ctrl + 257 寄存器表)
## 用法:
##   命令行:  vsim -c -do run.do
##   GUI:     先 vlib/vlog, 再 vsim -do run.do (去掉下方 vsim 前两行)
## =====================================================================

## 创建库 (已存在则重建)
if {[file exists work]} { file delete -force work }
vlib work

## 编译
vlog ../rtl/i2c_ctrl.v
vlog ../rtl/ov5640_config.v
vlog ./tb_ov5640_config.v

## 仿真
vsim -voptargs=+acc work.tb_ov5640_config

## 波形
view wave
add wave -divider {tb_ov5640_config}
add wave tb_ov5640_config/sys_clk
add wave tb_ov5640_config/rst_n
add wave tb_ov5640_config/scl
add wave tb_ov5640_config/sda
add wave tb_ov5640_config/config_done
add wave -divider {SCCB slave (OV5640 模型)}
add wave tb_ov5640_config/in_txn
add wave tb_ov5640_config/byte_idx
add wave tb_ov5640_config/bit_cnt
add wave tb_ov5640_config/byte_buf
add wave tb_ov5640_config/reg_addr_r
add wave tb_ov5640_config/ack_pending
add wave tb_ov5640_config/wr_count

## 运行到 $finish
run -all
