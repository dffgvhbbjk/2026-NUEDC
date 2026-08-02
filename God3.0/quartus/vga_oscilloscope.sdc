##=====================================================================
## vga_oscilloscope.sdc — 时序约束 (EP4CE6E22C8)
##
## 时钟架构:
##   sys_clk   50MHz  板载晶振
##   pll clk0  40MHz  VGA 像素时钟 (800x600@60Hz)
##   pll clk1  24.576MHz ADC MCLK (SCKI)
##   pll clk2  6.144MHz  ADC BCLK (I2S)
##
## 跨时钟域路径 (均已做 CDC 处理, 关闭跨域时序检查):
##   ADC BCLK域 → VGA域 : audio_fifo (DCFIFO)
##   VGA域 ↔ SYS域      : toggle 握手 + 3级FF (FIR 数据通路)
##=====================================================================

## ---- 主时钟 ----
create_clock -name sys_clk -period 20.000 [get_ports sys_clk]

## ---- PLL 输出时钟自动派生 ----
derive_pll_clocks

## ---- BCK 输出引脚生成时钟 (PLL c2 直通引脚, 供 PCM1808) ----
## 作为 adc_lrclk 输出时序检查的外部参考时钟
create_generated_clock -name bck_out \
    -source [get_pins {*|altpll_component|auto_generated|pll1|clk[2]}] \
    [get_ports adc_bclk]

derive_clock_uncertainty

## ---- 异步时钟组: 域间全部为 DCFIFO / 多级同步器 / toggle 握手 ----
## bck_out 与 pll clk[2] 同源同组 (LRCK 输出检查在组内进行)
set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk}] \
    -group [get_clocks {*pll1|clk[0]}] \
    -group [get_clocks {*pll1|clk[1]}] \
    -group [get_clocks {*pll1|clk[2] bck_out}]

## CDC 路径: FIR toggle 握手 (vga_clk ↔ sys_clk), 已做 CDC 处理, 关闭跨域时序检查
set_false_path -from [get_clocks {sys_clk}] -to [get_clocks {*pll1|clk[0]}]
set_false_path -from [get_clocks {*pll1|clk[0]}] -to [get_clocks {sys_clk}]

## ---- 异步输入端口 (复位/按键/UART均有同步消抖, 与时钟无关) ----
set_false_path -from [get_ports {sys_rst_n freeze_btn gap_btn_53 gap_btn_54 btn_55 gap_btn_58 gap_btn_59 uart_rxd}]

## ---- ADC 数据输入 (PCM1808 DOUT, 源同步: BCK 由 FPGA 输出) ----
## PCM1808 在 BCK 下降沿更新 DOUT, FPGA 在 BCK 上升沿采样,
## 可用窗口 = 半个 BCK 周期 ≈ 81.5ns。用 -clock_fall 表达 下降沿发射→上升沿采样。
## PCM1808 数据手册 t(CKDO): BCK 下降沿→DOUT 有效 = -10 ~ +40ns
##   max = tCO(max) 40 + FPGA BCK 输出延迟 ~6 + 走线往返 ~2 = 48ns
##   min = tCO(min) -10 (数据可在时钟沿前变化, 用于 hold 检查)
set_input_delay -clock [get_clocks {*pll1|clk[2]}] -clock_fall -max  48.0 [get_ports adc_data]
set_input_delay -clock [get_clocks {*pll1|clk[2]}] -clock_fall -min -10.0 [get_ports adc_data]

## ---- 输出端口 (逐组显式约束, 不再使用一揽子 false_path -to all_outputs) ----

## LRCK → PCM1808 从机: FPGA 在 BCK 下降沿翻转 LRCK (negedge 寄存),
## PCM1808 在 BCK 上升沿采样, 可用窗口 = 半 BCK 周期 ≈ 81.5ns。
## 方案 §7.2: LRCK 建立 50ns / 保持 10ns (相对 BCK 上升沿)
set_output_delay -clock bck_out -max  50.0 [get_ports adc_lrclk]
set_output_delay -clock bck_out -min -10.0 [get_ports adc_lrclk]

## 时钟直通引脚 (MCLK=PLL c1, BCK=PLL c2): 纯时钟输出, 无寄存器→端口
## 数据路径; 与 DOUT/LRCK 的相位关系已由上面的 IO 约束按同一 PLL 时钟检查
set_false_path -to [get_ports {adc_mclk adc_bclk}]

## VGA 模拟显示链路: 显示器不对沿精确采样, hsync/vsync/rgb 同域同源,
## 相对偏差 << 像素周期 25ns, 无需 IO 检查
set_false_path -to [get_ports {hsync vsync rgb*}]

## UART 异步串行 (自定时, 位周期 500ns) / 蜂鸣器直流电平: 无外部时钟采样关系
set_false_path -to [get_ports {uart_txd buzzer}]
