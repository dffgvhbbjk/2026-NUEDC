##=====================================================================
## vga_oscilloscope.sdc — 时序约束 (EP4CE6E22C8)
##
## 时钟架构:
##   sys_clk   50MHz    板载晶振
##   pll clk0  40MHz    VGA 像素时钟 (800x600@60Hz)
##   pll clk1  24.576MHz ADC MCLK (SCKI)
##   pll clk2  6.144MHz  ADC BCLK (I2S)
##   pll clk3  24MHz    OV5640 XCLK (外部输出, 不驱动内部逻辑)
##   cam_pclk  ~44MHz   OV5640 像素时钟 (外部输入, SVGA 30fps 最坏情况)
##
## 跨时钟域路径 (均已做 CDC 处理, 关闭跨域时序检查):
##   ADC BCLK域 → VGA域 : audio_fifo (DCFIFO)
##   VGA域 ↔ SYS域      : toggle 握手 + 3级FF (FIR 数据通路)
##   CAM PCLK域 ↔ SYS域 : 3级FF同步器 + toggle握手 (指令/测量结果)
##=====================================================================

## ---- 主时钟 ----
create_clock -name sys_clk -period 20.000 [get_ports sys_clk]

## ---- OV5640 像素时钟 (外部输入, ~44MHz 最坏情况) ----
##   SVGA 800x600 @ 30fps: PCLK = HTS×VTS×fps = 1896×780×30 ≈ 44.4MHz
##   0x3824=0x02 可能分频到 15~22MHz, 但按最坏 44MHz 约束
##   若采集逻辑在 44MHz 下满足, 则任何 PCLK ≤ 44MHz 都能正常工作
create_clock -name cam_pclk -period 22.500 [get_ports cam_pclk]

## ---- PLL 输出时钟自动派生 ----
derive_pll_clocks
derive_clock_uncertainty

## ---- 异步时钟组: 域间全部为 DCFIFO / 多级同步器 / toggle 握手 ----
set_clock_groups -asynchronous \
    -group [get_clocks {sys_clk}] \
    -group [get_clocks {*pll1|clk[0]}] \
    -group [get_clocks {*pll1|clk[1]}] \
    -group [get_clocks {*pll1|clk[2]}] \
    -group [get_clocks {cam_pclk}]

## CDC 路径: FIR toggle 握手 (vga_clk ↔ sys_clk), 已做 CDC 处理, 关闭跨域时序检查
set_false_path -from [get_clocks {sys_clk}] -to [get_clocks {*pll1|clk[0]}]
set_false_path -from [get_clocks {*pll1|clk[0]}] -to [get_clocks {sys_clk}]

## CDC 路径: cam_pclk ↔ sys_clk (toggle 握手 + 3级FF同步器), 关闭跨域时序检查
set_false_path -from [get_clocks {cam_pclk}] -to [get_clocks {sys_clk}]
set_false_path -from [get_clocks {sys_clk}] -to [get_clocks {cam_pclk}]

## ---- 异步输入端口 (复位/按键/UART均有同步消抖, 与时钟无关) ----
set_false_path -from [get_ports {sys_rst_n freeze_btn test_btn btn_55 btn_58 btn_59 uart_rxd}]

## ---- ADC 数据输入 (PCM1808 DOUT) ----
## BCK=6.144MHz (周期 163ns), DOUT 在 BCK 下降沿变化, FPGA 上升沿采样,
## 半周期裕量 ~81ns, 板级延迟按宽裕的 100ns 约束
set_input_delay -clock [get_clocks {*pll1|clk[2]}] -max 100.0 [get_ports adc_data]
set_input_delay -clock [get_clocks {*pll1|clk[2]}] -min   0.0 [get_ports adc_data]

## ---- OV5640 DVP 输入 (cam_pclk 域) ----
## PCLK ~44MHz (周期 22.5ns), 数据在 PCLK 下降沿变化, FPGA 上升沿采样
## 半周期裕量 ~11.25ns, 板级延迟 ~3ns, 按 8ns 约束留余量
set_input_delay -clock [get_clocks {cam_pclk}] -max 8.0 [get_ports {cam_data[*] cam_href cam_vsync}]
set_input_delay -clock [get_clocks {cam_pclk}] -min 0.0 [get_ports {cam_data[*] cam_href cam_vsync}]

## ---- 输出端口 ----
## 外部无严格时序要求的输出 (VGA/ADC时钟/数码管/摄像头控制), 关闭检查
set_false_path -to [all_outputs]
