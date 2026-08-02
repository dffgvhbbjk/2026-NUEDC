# ============================================================================
# create_signaltap.tcl — 自动创建 SignalTap II 配置 (P0 验收用)
#
# 用法:
#   在 Quartus 中: View → Tcl Console → 输入: source create_signaltap.tcl
#   或命令行: quartus_sh -t create_signaltap.tcl
#
# 创建完成后:
#   1. Tools → SignalTap II Logic Analyzer 打开
#   2. 选择 stp1.stp 查看
#   3. 重新编译工程 (SignalTap 会嵌入到 .sof)
#   4. 下载 .sof 后即可抓信号
# ============================================================================

package require ::quartus::sto

# ---- 1. 创建新的 SignalTap 文件 ----
create_stp_file stp1.stp

# ---- 2. 设置采样时钟 (用 VGA 40MHz, 能采到 BCLK 域信号) ----
# 注: pll1.clk[0] 是 VGA 40MHz
set sto_inst [get_sto_instance_pointer -instance_index 0]
set_clock_domain -instance $sto_inst -clock "u_clk_gen|altpll_component|auto_generated|pll1|clk[0]"

# ---- 3. 设置采样深度 2048 ----
set_storage_type -instance $sto_inst -type "Continuous" -depth 2048

# ---- 4. 添加要抓取的信号 ----
# 分组1: I2S 时钟与原始数据
add_instance_nodes -instance $sto_inst -node_names {
    "adc_bclk"
    "adc_lrclk"
    "adc_data"
    "adc_mclk"
}

# 分组2: I2S 接收器输出
add_instance_nodes -instance $sto_inst -node_names {
    "adc_l_data[23..0]"
    "adc_r_data[23..0]"
    "adc_l_valid"
    "adc_r_valid"
    "adc_frame_error"
}

# 分组3: FIFO 状态
add_instance_nodes -instance $sto_inst -node_names {
    "wave_fifo_full"
    "wave_fifo_level[7..0]"
}

# 分组4: 跨域后输出
add_instance_nodes -instance $sto_inst -node_names {
    "bypass_data[23..0]"
    "bypass_valid"
}

# 分组5: 触发指示
add_instance_nodes -instance $sto_inst -node_names {
    "hammer_hit"
}

# ---- 5. 设置触发条件: adc_l_valid 上升沿 ----
# 这是最关键的验证 - 每帧应该恰好1个valid脉冲
set_trigger_condition -instance $sto_inst \
    -name "Valid pulse" \
    -condition "adc_l_valid RISING EDGE"

# ---- 6. 保存 ----
save_stp_file stp1.stp

puts "=========================================="
puts "SignalTap 配置创建成功: stp1.stp"
puts ""
puts "信号列表:"
puts "  I2S 原始: adc_bclk, adc_lrclk, adc_data, adc_mclk"
puts "  I2S 输出: adc_l/r_data[23..0], adc_l/r_valid, adc_frame_error"
puts "  FIFO:     wave_fifo_full, wave_fifo_level[7..0]"
puts "  跨域输出: bypass_data[23..0], bypass_valid"
puts "  触发:     adc_l_valid 上升沿"
puts "  深度:     2048"
puts "  时钟:     VGA 40MHz (pll1.clk[0])"
puts ""
puts "下一步:"
puts "  1. 在 Quartus 中重新编译 (Processing → Start Compilation)"
puts "  2. Tools → SignalTap II Logic Analyzer"
puts "  3. 连接 USB-Blaster, 下载 .sof"
puts "  4. 点击 Run Analysis 抓取波形"
puts "=========================================="
