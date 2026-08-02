---
name: quartus-build
description: |
  Quartus II 工程编译和下载指南。当用户涉及 FPGA 编译、引脚分配、时序约束、下载调试等主题时自动触发。
  包括 Quartus 项目配置、编译流程、SignalTap 调试、下载编程等技术。
---

# Quartus Build 技能

本技能提供 Altera Quartus II 的工程配置、编译和下载指南。

---

## 一、工程配置

### 1.1 项目文件结构

```
quartus/
├── vga_oscilloscope.qpf      # 工程文件
├── vga_oscilloscope.qsf      # 设置文件
├── vga_oscilloscope.qws      # 工作空间
├── ip_core/                  # IP 核目录
│   └── clk_gen/              # PLL IP
│       ├── clk_gen.qip
│       └── clk_gen.v
├── db/                       # 数据库目录
└── output_files/             # 编译输出
    ├── vga_oscilloscope.sof  # SRAM 配置文件
    └── vga_oscilloscope.pof  # Flash 配置文件
```

### 1.2 目标器件

| 参数 | 值 |
|------|-----|
| 系列 | Cyclone IV E |
| 型号 | EP4CE6E22C8 |
| 封装 | TQFP-144 |
| 速度等级 | 8 |
| 温度范围 | 0°C ~ 85°C |

### 1.3 关键 QSF 设置

```tcl
# 项目设置
set_global_assignment -name FAMILY "Cyclone IV E"
set_global_assignment -name DEVICE EP4CE6E22C8
set_global_assignment -name TOP_LEVEL_ENTITY vga_dual_mode_top
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files

# 源文件
set_global_assignment -name VERILOG_FILE ../rtl/vga_dual_mode_top.v
set_global_assignment -name VERILOG_FILE ../rtl/vga_ctrl_simple.v
# ... 其他源文件

# 引脚分配 (示例)
set_location_assignment PIN_23 -to sys_clk
set_location_assignment PIN_32 -to hsync
set_location_assignment PIN_34 -to vsync

# I/O 标准
set_global_assignment -name STRATIX_DEVICE_IO_STANDARD "2.5 V"

# 时序约束
set_global_assignment -name SDC_FILE vga_oscilloscope.sdc
```

---

## 二、引脚分配

### 2.1 完整引脚表

#### 时钟与复位

| 信号 | 引脚 | I/O | 说明 |
|------|------|-----|------|
| sys_clk | PIN_23 | 输入 | 50MHz 晶振 |
| sys_rst_n | - | - | 内部复位 (弱上拉) |

#### VGA 输出

| 信号 | 引脚 | I/O | 说明 |
|------|------|-----|------|
| hsync | PIN_32 | 输出 | 行同步 |
| vsync | PIN_34 | 输出 | 场同步 |
| rgb[11] | PIN_30 | 输出 | R[3] |
| rgb[10] | PIN_11 | 输出 | R[2] |
| rgb[9] | PIN_7 | 输出 | R[1] |
| rgb[8] | PIN_2 | 输出 | R[0] |
| rgb[7] | PIN_44 | 输出 | G[3] |
| rgb[6] | PIN_42 | 输出 | G[2] |
| rgb[5] | PIN_38 | 输出 | G[1] |
| rgb[4] | PIN_33 | 输出 | G[0] |
| rgb[3] | PIN_31 | 输出 | B[3] |
| rgb[2] | PIN_28 | 输出 | B[2] |
| rgb[1] | PIN_10 | 输出 | B[1] |
| rgb[0] | PIN_3 | 输出 | B[0] |

#### ADC/I2S 接口

| 信号 | 引脚 | I/O | 说明 |
|------|------|-----|------|
| adc_mclk | PIN_51 | 输出 | 主时钟 (SCKI 24.576MHz) |
| adc_bclk | PIN_43 | 输出 | 位时钟 (6.144MHz) |
| adc_lrclk | PIN_49 | 输出 | 左右时钟 (96kHz) |
| adc_data | PIN_46 | 输入 | 数据输入 |

#### 74HC595 数码管

| 信号 | 引脚 | I/O | 说明 |
|------|------|-----|------|
| seg_sclk | PIN_121 | 输出 | 移位时钟 |
| seg_rclk | PIN_125 | 输出 | 锁存时钟 |
| seg_din | PIN_127 | 输出 | 串行数据 |

#### 用户按键

| 信号 | 引脚 | I/O | 说明 |
|------|------|-----|------|
| freeze_btn | PIN_25 | 输入 | 波形冻结/重新武装 (需外部 10kΩ 上拉) |

> 注: 本项目为固定示波器模式，无 mode_btn。摄像头 (OV5640) 已于 2026-07-23 移除。

### 2.2 QSF 引脚分配脚本

```tcl
# VGA 接口
set_location_assignment PIN_32 -to hsync
set_location_assignment PIN_34 -to vsync
set_location_assignment PIN_30 -to rgb[11]
set_location_assignment PIN_11 -to rgb[10]
set_location_assignment PIN_7 -to rgb[9]
set_location_assignment PIN_2 -to rgb[8]
set_location_assignment PIN_44 -to rgb[7]
set_location_assignment PIN_42 -to rgb[6]
set_location_assignment PIN_38 -to rgb[5]
set_location_assignment PIN_33 -to rgb[4]
set_location_assignment PIN_31 -to rgb[3]
set_location_assignment PIN_28 -to rgb[2]
set_location_assignment PIN_10 -to rgb[1]
set_location_assignment PIN_3 -to rgb[0]

# ADC/I2S 接口
set_location_assignment PIN_51 -to adc_mclk
set_location_assignment PIN_43 -to adc_bclk
set_location_assignment PIN_49 -to adc_lrclk
set_location_assignment PIN_46 -to adc_data

# 数码管
set_location_assignment PIN_121 -to seg_sclk
set_location_assignment PIN_125 -to seg_rclk
set_location_assignment PIN_127 -to seg_din

# 按键
set_location_assignment PIN_25 -to freeze_btn
set_location_assignment PIN_85 -to sys_rst_n
```

---

## 三、编译流程

### 3.1 命令行编译

```bash
# 进入工程目录
cd quartus

# 分析与综合
quartus_map vga_oscilloscope

# 布局布线
quartus_fit vga_oscilloscope

# 时序分析
quartus_sta vga_oscilloscope

# 汇编 (生成 SOF)
quartus_asm vga_oscilloscope

# 一键编译
quartus_sh --flow compile vga_oscilloscope
```

### 3.2 编译状态检查

```bash
# 查看编译报告
quartus_cdb vga_oscilloscope --report

# 查看资源使用
grep -A 20 "Fitter Resource Usage Summary" output_files/vga_oscilloscope.fit.summary

# 查看 Timing
grep -A 10 "Timing Analysis Summary" output_files/vga_oscilloscope.sta.summary
```

### 3.3 常见编译警告处理

| 警告 | 含义 | 处理 |
|------|------|------|
| Inferred latch | 锁存器推断 | 检查组合逻辑完整性 |
| Reduced register | 寄存器优化 | 检查逻辑是否正确 |
| No clock | 无时钟信号 | 检查时钟连接 |
| Timing not met | 时序不满足 | 增加时序约束 |

---

## 四、时序约束

### 4.1 SDC 文件示例

```sdc
# 系统时钟
create_clock -name sys_clk -period 20 [get_ports sys_clk]

# PLL 输出时钟
derive_pll_clocks

# 时钟组 (异步)
set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks vga_clk] \
    -group [get_clocks bclk]

# 输入延迟
set_input_delay -clock sys_clk -max 5 [get_ports adc_data]
set_input_delay -clock sys_clk -min -5 [get_ports adc_data]

# 输出延迟
set_output_delay -clock vga_clk -max 5 [get_ports {rgb[*]}]
set_output_delay -clock vga_clk -min -5 [get_ports {rgb[*]}]

# 多周期路径
set_multicycle_path -setup 2 -from [get_registers state_reg]
```

### 4.2 时序分析方法

```bash
# 生成时序报告
quartus_sta vga_oscilloscope --report_panel "Timing Analyzer||Summary"

# 查看最差路径
quartus_sta vga_oscilloscope --report_panel "Timing Analyzer||Slow 1200mV 85C Model||Fmax Summary"
```

---

## 五、下载调试

### 5.1 编程文件生成

```tcl
# 转换 SOF 为 POF (用于 EPCS)
quartus_cpf -c -d EPCS16 -s 1 output_files/vga_oscilloscope.sof output_files/vga_oscilloscope.pof

# 生成 JIC 文件 (用于 EPCS)
quartus_cpf -c -d EPCS16 -s 1 output_files/vga_oscilloscope.sof output_files/vga_oscilloscope.jic
```

### 5.2 Programmer 使用

```bash
# 命令行下载
quartus_pgm -m JTAG -o "p;output_files/vga_oscilloscope.sof"

# 指定电缆
quartus_pgm -c USB-Blaster -m JTAG -o "p;output_files/vga_oscilloscope.sof"
```

### 5.3 CDF 配置文件

```xml
<?xml version="1.0"?>
<JTAG>
  <Device>
    <Name>EP4CE6</Name>
    <File>output_files/vga_oscilloscope.sof</File>
  </Device>
</JTAG>
```

---

## 六、SignalTap II 调试

### 6.1 创建 SignalTap 文件

1. File → New → SignalTap II Logic Analyzer File
2. 添加信号到监控列表
3. 设置采样深度和时钟
4. 编译并下载

### 6.2 信号添加示例

```
时钟: vga_clk
采样深度: 2048
触发条件: init_done 上升沿

监控信号:
- state[2:0]
- h_cnt[10:0]
- v_cnt[9:0]
- init_done
- pixel_valid
```

### 6.3 通过 STP 文件配置

```tcl
# 在 QSF 中添加
set_global_assignment -name SIGNALTAP_FILE vga_oscilloscope.stp
set_global_assignment -name ENABLE_SIGNALTAP ON
```

---

## 七、常见问题

### 7.1 编译失败

| 错误 | 解决方法 |
|------|----------|
| License error | 检查 license 文件路径 |
| Device not found | 检查 QSF 中的器件设置 |
| Memory overflow | 减少存储器使用或优化设计 |

### 7.2 下载失败

| 错误 | 解决方法 |
|------|----------|
| Device not found | 检查 JTAG 电缆连接 |
| USB-Blaster not found | 安装驱动或检查 USB |
| Programming failed | 检查 FPGA 供电 |

### 7.3 运行异常

| 现象 | 可能原因 |
|------|----------|
| 无输出 | 检查引脚分配 |
| 输出不稳定 | 检查时序约束 |
| 功能错误 | 检查仿真验证 |

---

## 八、资源监控

### 8.1 资源报告解读

```
Fitter Resource Usage Summary (2026-07-21 实测)
  Logic Elements: 1,837 / 6,272 (29%)
  Registers:      ~1,500
  M9K Memory:     ~16 / 30 (53%)
  PLLs:           1 / 2 (50%)
  DSP Blocks:     1 / 30 (3%)
  I/O Pins:       46 / 92 (50%)
```

### 8.2 优化建议

| 资源 | 优化方法 |
|------|----------|
| LE | 使用状态机编码优化 |
| M9K | 调整存储器深度和宽度 |
| DSP | 复用乘法器 |

---

## 九、参考资源

- Quartus II Handbook Volume 1: Design and Synthesis
- Quartus II Handbook Volume 2: Design Implementation
- Cyclone IV Device Handbook
- 项目 AGENTS.md 中的引脚定义