# AGENTS.md — FPGA VGA 波形显示项目规则

本项目是一个基于 Altera Cyclone IV FPGA 的桩基低应变完整性检测系统，包含 VGA 波形显示和桩长计算功能。

---

## ⚠️ 不可变基准 (CRITICAL — 必读)

**VGA 显示链路 (如下文件) 已验证可在硬件上正常显示, 严禁 AI 自动修改:**

- `rtl/vga_ctrl_simple.v` — VGA 时序控制器 (800×600@60Hz, 40MHz, **负极性 HSYNC/VSYNC**, 与本项目的物理显示器/扩展板相配)
- `rtl/vga_dual_mode_top.v` 内的 VGA 路径、按键消抖、示波器路径与 RGB 选择逻辑
- `rtl/vga_waveform.v` — 波形显示算法
- `quartus/vga_oscilloscope.qsf` — 引脚分配、I/O 标准、PLL IP 配置

**AI 在编辑任何上述文件之前必须显式询问用户。** 不要因为"VESA 标准是正极性同步"就擅自改 VGA 时序——本项目的显示器/扩展板实测对应**负极性**, 改成"正确"的 VESA 极性反而会让屏幕锁不上 GSP 出现花屏。

---

## 项目概述

### 核心功能

- **波形显示**: ADC (PCM1808) → I2S 接收 → FIFO → 波形显示
- **桩长计算**: 基于应力波反射原理, 计算缺陷深度和桩长
- **锤击触发**: 按键武装 → 检测锤击 → 采集 + 冻结波形

### 目标硬件

| 部件 | 规格 |
|------|------|
| FPGA | Altera Cyclone IV E, EP4CE6E22C8 (TQFP144) |
| ADC | PCM1808 (24-bit, I2S 接口) |
| 显示 | VGA 800×600 @ 60Hz (40MHz 像素时钟) |

---

## 目录结构

```
项目根/
├── rtl/                           # RTL 源代码
│   ├── vga_dual_mode_top.v        # 顶层模块
│   ├── vga_ctrl_simple.v          # VGA 时序控制 (不可变基准)
│   ├── vga_waveform.v             # 波形绘制 (不可变基准)
│   ├── iis_slave_rx.v             # I2S 从机接收
│   ├── audio_fifo.v               # 音频 FIFO (BCLK→VGA 域 CDC)
│   ├── pile_length_calc.v         # 桩长计算
│   └── seg_display.v              # 数码管显示 (74HC595)
├── quartus/                       # Quartus 工程文件
│   ├── vga_oscilloscope.qpf       # 工程文件
│   ├── vga_oscilloscope.qsf       # 设置文件 (引脚分配, 不可变基准)
│   ├── vga_oscilloscope.sdc       # 时序约束
│   ├── vga_oscilloscope_assignment_defaults.qdf
│   ├── ip_core/clk_gen/           # PLL IP 核
│   └── output_files/              # 编译输出 (.sof/.cdf/.jdi/.sld/.pin)
├── doc/                           # 文档
│   ├── VGA示波器原理详解.md
│   ├── vga.vsdx
│   └── 基于FPGA的桩基低应变完整性检测系统赛题预测与备赛全指南.md
├── .agents/skills/                # AI 辅助技能
│   ├── fpga-verilog/SKILL.md
│   ├── vga-display/SKILL.md
│   └── quartus-build/SKILL.md
├── .claude/settings.json          # Claude 配置
├── .gitignore
└── AGENTS.md                      # 本文件
```

---

## 时钟域架构

| 时钟域 | 频率 | 来源 | 用途 |
|--------|------|------|------|
| sys_clk | 50MHz | 板载晶振 | 系统主时钟 |
| vga_clk | 40MHz | PLL c0 | VGA 800×600 像素时钟 |
| bclk | 6.144MHz | PLL c2 | I2S 位时钟 |
| lrclk | 96kHz | BCLK/64 | I2S 左右声道时钟 |

### 跨时钟域处理

```
ADC (BCLK域) ──→ DCFIFO ──→ VGA (VGA_CLK域)
```

### 时序约束

全项目时序约束见 `quartus/vga_oscilloscope.sdc` (Quartus 自动加载 `<revision>.sdc`):

- sys_clk 50MHz + PLL 输出自动派生
- 各时钟域声明为异步时钟组 (域间均为 DCFIFO / 多级同步器)
- 复位/按键输入 false_path, ADC 输入延迟已约束

---

## 关键模块接口

### VGA 时序控制器 (vga_ctrl_simple)

```verilog
// 800×600 @ 60Hz, 40MHz 像素时钟
module vga_ctrl_simple (
    input  wire        vga_clk,    // 40MHz
    input  wire        sys_rst_n,
    output wire [9:0]  pix_x,      // 0-799
    output wire [9:0]  pix_y,      // 0-599
    output wire        rgb_valid,
    output wire        hsync,      // 负极性
    output wire        vsync       // 负极性
);
```

### I2S 从机接收器 (iis_slave_rx)

```verilog
module iis_slave_rx #(
    parameter DATA_WIDTH = 24,
    parameter BCLK_DIV = 32
) (
    input  wire        rst_n,
    input  wire        bclk,       // 6.144MHz
    input  wire        lrclk,      // 96kHz
    input  wire        sdin,
    output wire [23:0] l_data,     // 左声道
    output wire [23:0] r_data,     // 右声道
    output wire        l_valid,
    output wire        r_valid
);
```

---

## 代码风格规范

### 1. 命名约定

| 类型 | 规则 | 示例 |
|------|------|------|
| 模块名 | 小写下划线 (snake_case) | `vga_ctrl_simple`, `iis_slave_rx` |
| 参数/宏 | 大写下划线 (UPPER_CASE) | `H_SYNC`, `DATA_WIDTH` |
| 局部参数 | 大写下划线 | `IDLE`, `START`, `DONE` |

**信号后缀约定:**

| 后缀 | 含义 | 示例 |
|------|------|------|
| `_n` | 低电平有效 | `rst_n`, `cs_n`, `wr_n` |
| `_vld` | 数据有效脉冲 | `ad_data_vld`, `l_valid` |
| `_en` | 使能信号 | `rd_en`, `wr_en` |
| `_cnt` | 计数器 | `h_cnt`, `v_cnt` |
| `_reg` | 寄存器输出 | `state_reg`, `data_reg` |
| `_next` | 下一状态 | `state_next` |
| `_d1`, `_d2` | 同步器延迟 | `signal_d1`, `signal_d2` |

### 2. 时钟与复位

- 始终使用**异步复位、同步释放**策略
- 复位信号命名为 `rst_n` 或 `xxx_rst_n`, 低电平有效
- 复位逻辑统一放在 always 块开头

```verilog
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 复位逻辑
    end else begin
        // 正常逻辑
    end
end
```

### 3. 参数定义

- 使用 `parameter` 定义模块参数, 放在端口声明之前
- 使用 `localparam` 定义模块内部常量
- 状态机状态使用 `localparam` 定义

```verilog
module my_module #(
    parameter DATA_WIDTH = 24,
    parameter FIFO_DEPTH = 256
) (
    // 端口声明
);
    localparam IDLE  = 3'd0;
    localparam START = 3'd1;
```

### 4. 跨时钟域处理

| 信号类型 | 处理方式 |
|----------|----------|
| 单比特信号 | 两级同步器 (打两拍) |
| 多比特数据 | 异步 FIFO (DCFIFO) |
| 控制信号 | 握手协议 |

```verilog
// 两级同步器示例
always @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) begin
        src_signal_d1 <= 1'b0;
        src_signal_d2 <= 1'b0;
    end else begin
        src_signal_d1 <= src_signal;
        src_signal_d2 <= src_signal_d1;
    end
end

// 边沿检测
wire signal_posedge = src_signal_d2 & ~src_signal_d1;
wire signal_negedge = ~src_signal_d2 & src_signal_d1;
```

### 5. 状态机设计

推荐使用**三段式状态机**, 状态转移逻辑与时序逻辑分离:

```verilog
// 状态寄存器 (时序逻辑)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= IDLE;
    else        state <= next_state;
end

// 状态转移 (组合逻辑)
always @(*) begin
    next_state = state;
    case (state)
        IDLE:  if (start) next_state = RUN;
        RUN:   if (done)  next_state = IDLE;
        default: next_state = IDLE;
    endcase
end

// 输出逻辑 (时序逻辑)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 复位输出
    end else begin
        case (state)
            // 各状态输出
        endcase
    end
end
```

### 6. 资源优化

- 优先使用 **M9K 嵌入式存储器** (见避坑指南#7)
- 使用 **DSP 乘法器**进行乘法运算
- 避免不必要的寄存器复制
- 时序逻辑优先, 组合逻辑最小化

---

## AI 容易犯的错误 (避坑指南 — 从过往事故总结)

### 1. 擅自改 VGA 同步极性

- **事故**: 把 `hsync/vsync` 从负极性改成 VESA 标准"正极性", 导致显示器锁不到同步、整屏花屏
- **正确做法**: VGA 时序模块一旦验证可用即冻结, 极性问题归咎于物理显示器匹配而非标准合规

### 2. 把"不能生成 .sof"误判为代码错误

- **事故**: Quartus 在 Evaluation Mode 会报 `Can't generate programming files because you are currently using the Quartus II software in Evaluation Mode` 但编译状态为 "Successful"。AI 误判为代码错误或漏改
- **正确做法**: 检查 `output_files/*.asm.rpt` 末尾的 warning 行, 如果是 Evaluation Mode 警告则用户需要用正式版 Quartus 或 Web Edition, 与代码无关

### 3. 给不支持弱上拉的引脚加 `WEAK_PULL_UP_RESISTOR`

- **事故**: 给 PIN_25 (freeze_btn) 加内部弱上拉, 编译报 `Error (169057): Can't place I/O pin ... uses weak pullup, which is not supported by this pin location` → 设计 fit 失败
- **正确做法**: 这些引脚在 Cyclone IV E 上不支持内部弱上拉, 必须用外部 10kΩ 上拉电阻到 3.3V。QSF 里**禁止**对 `freeze_btn` 使用 `WEAK_PULL_UP_RESISTOR`

### 4. 跨时钟域同步器使用异步复位的反向逻辑

- **事故**: `always @(posedge dst_clk or negedge src_rst_n)` 用源域的复位信号复位目的域寄存器——这是跨域复位, 容易导致目的域刚复位完因 `src_rst_n` 释放而过早解除
- **正确做法**: CDC 同步器应使用**目的域产生的同步后复位** (如 `vga_rst_n`), 单纯的 `init_done` 这类稳定电平 CDC 用 3 级 FF 即可

### 5. 只看代码不查实际硬件行为就否定用户描述

- **事故**: AI 反复说"代码没问题, 编译失败导致 .sof 是旧的"——其实根因是硬件信号链路问题
- **正确做法**: **用户硬件现象高于代码静态分析**。优先排查 ADC → I2S → FIFO → 波形显示这条链路

### 6. 修改 RTL 后未清理 Quartus 编译缓存导致修改不生效

- **事故**: 修改了 `vga_waveform.v` 的显示参数和字符渲染逻辑, 但 Quartus 增量编译使用了旧的 `db/` 和 `incremental_db/` 缓存, 跳过了已修改的文件, 烧录后现象不变
- **正确做法**: **每次修改 RTL 源码后, 必须先清理 Quartus 编译缓存再重新编译**

```
1. 关闭 Quartus (或确保无编译进程运行)
2. 删除 quartus/db/、quartus/incremental_db/、quartus/output_files/greybox_tmp/
3. 在 Quartus 中执行完全编译 (Processing → Start Compilation)
4. 重新烧录 .sof 文件
```

PowerShell 清理命令:

```powershell
Remove-Item -Recurse -Force quartus/db, quartus/incremental_db, quartus/output_files/greybox_tmp -ErrorAction SilentlyContinue
```

### 7. M9K RAM 读写与异步复位混在同一 always 块导致推断失败

- **事故**: `vga_waveform.v` 的 `wave_buf` 读写分别在两个含 `negedge rst_n` 的 always 块中, Quartus 无法将 `wave_buf` 推断为 M9K RAM (M9K 输出寄存器/写端口不支持异步复位), 退化为 32768 个普通寄存器 → `Error (276003): Cannot convert all sets of registers into RAM megafunctions` → 编译失败
- **正确做法**: M9K RAM 的读和写必须各自放在**独立的 `always @(posedge clk)` 块中, 不含异步复位**。其他控制寄存器 (wr_ptr, snapped_flag 等) 放另一个含复位的 always 块

---

## 资源使用情况

(2026-07-21 编译实测, fit.summary)

| 资源 | 总量 | 已用 | 占比 |
|------|------|------|------|
| LE | 6,272 | 1,837 | 29% |
| 存储 bit | 276,480 (30×M9K) | 145,776 (~16 M9K) | 53% |
| PLL | 2 | 1 | 50% |
| 9-bit 乘法器 | 30 | 1 | 3% |
| 引脚 | 92 | 46 | 50% |

---

## 代码审查清单

提交代码前检查以下项:

- [ ] 所有跨时钟域信号已正确处理
- [ ] 状态机有完整的复位状态
- [ ] 参数有合理的默认值
- [ ] 组合逻辑没有锁存器推断
- [ ] 信号位宽匹配, 无截断警告
- [ ] 时序约束已设置 (SDC 文件)

---

## 常见任务指南

### 修改 VGA 分辨率

1. 更新 `vga_ctrl_simple.v` 中的时序参数
2. 调整 PLL 配置以生成正确的像素时钟

### 添加新的 I2S 设备

1. 在 `iis_slave_rx.v` 中检查时序参数是否匹配
2. 更新 FIFO 深度以适应新的采样率
3. 检查时钟域是否正确配置

---

## 相关技能

本项目配备以下 AI 辅助技能 (位于 `.agents/skills/`):

| 技能 | 用途 |
|------|------|
| `fpga-verilog` | Verilog 编码规范和最佳实践 |
| `vga-display` | VGA 显示系统设计指南 |
| `quartus-build` | Quartus 工程编译和下载 |

使用方式: 在对话中描述相关需求, AI 会自动加载相应技能。

---

## 变更记录

### 2026-07-26 (项目结构整理 + 清理 FIR 滤波通路 + 旧版顶层)

**项目结构整理:**
- 去掉 `vga/` 嵌套层, 所有内容上移到项目根
- 清理 `quartus/db/`、`incremental_db/`、`output_files/greybox_tmp/` 编译缓存
- 清理 `output_files/` 报告类文件 (`.rpt`/`.summary`/`.smsg`/`.done`), 仅保留 `.sof`/`.cdf`/`.jdi`/`.sld`/`.pin`
- 新增 `.gitignore` 防止编译产物再次混入
- 修正 `.claude/settings.json` 路径

**清理 FIR 滤波通路** (FIR 已长期被 `FIR_BYPASS=1` 旁路, 波形显示与桩长计算实际使用原始 ADC `bypass_data`):
- 删除 `rtl/fir_lpf.v`、`rtl/fir_coef.hex` (FIR 滤波器模块及系数)
- 删除 `quartus/ip_core/Filter/` (FIR II IP 核目录, .qip 未被 QSF 引用)
- 删除 `rtl/vga_oscilloscope_top.v` (旧版单模式顶层, 端口与当前 QSF 不匹配, 未被任何模块例化)
- `rtl/vga_dual_mode_top.v`: 移除 FIR generate 通路 (CDC + fir_lpf 例化 + 回传同步, 约 110 行)、`FIR_BYPASS` 参数、`fir_out_data`/`fir_out_valid` 线网; `pile_length_calc` 与 `vga_waveform` 消费端直连 `bypass_data`/`bypass_valid`。**VGA 时序/按键消抖/示波器路径/RGB 选择逻辑等不可变基准部分未改动**, 功能行为与 `FIR_BYPASS=1` 时完全一致
- `quartus/vga_oscilloscope.qsf`: 移除 `fir_lpf.v` 与 `vga_oscilloscope_top.v` 源文件行 (引脚分配/I/O 标准/PLL IP 配置未改动)

**早期清理 (根级 stray 文件):**
- 删除项目根错位的 `vga_oscilloscope.qpf`/`.qsf` (Cyclone V 配置错误版) 和报告
- 删除三层嵌套 `vga/vga/vga/rtl/ad_data_gen.v` (已弃用)
- 删除 `.uploads/` 空目录和过时的 `.claude/settings.local.json`

### 2026-07-23 (清理摄像头和仿真代码)

- 删除全部摄像头 RTL: `ov5640_top.v`, `ov5640_init.v`, `ov5640_capture.v`, `frame_buffer.v`
- 删除摄像头叠加层: `vga_overlay.v`, `vga_overlay_debug.v`
- 删除仿真文件: `sim/` 目录全部测试
- 删除仿真辅助模块: `ad_data_gen.v`, `pll_config_note.v`
- 删除摄像头文档和技能: `OV5640_原理手册.md`, `ov5640-camera/SKILL.md`
- 更新 AGENTS.md, QSF, SDC, `vga_dual_mode_top.v` 清理所有摄像头引用
- 项目现为纯波形显示 + 桩长计算系统

### 2026-07-21 ~ 22 (历史代码修复, 已归档)

> 详细修复内容见 git 历史。主要包括: mode_btn 消抖重写、`vga_waveform.v` 计数器位宽修复、`iis_slave_rx` 复位域修复、`pile_length_calc.v` 乘法位宽 bug 修复 (3 级流水, TB 验证 712cm PASS)、`seg_display.v` 74HC595 移位时序重写、新增 SDC 时序约束、新增 ModelSim 自校验 TB。FIR 通路相关修复已随 FIR 删除归档。
