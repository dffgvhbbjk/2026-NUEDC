# AGENTS.md — 路由文件 (详细规则见 CLAUDE.md)

> **本文件仅作路由用途。所有不可变约束、引脚表、功能合约请见 [CLAUDE.md](CLAUDE.md)。**

---

## ⛔ 核心红线 (摘要, 完整列表见 CLAUDE.md)

1. **引脚映射不可自改** — 22 个引脚的完整映射表在 CLAUDE.md，含 freeze_btn(PIN_25) 不可开内部弱上拉、buzzer 必须用 PIN_1 非 PIN_91 等硬件陷阱
2. **VGA 显示链路不可自改** — `vga_ctrl_simple.v`, `vga_waveform.v`, `vga_dual_mode_top.v` 的 VGA/波形路径
3. **时钟配置不可自改** — PLL 频率 (40MHz/24.576MHz/6.144MHz) 在 `clk_gen.ppf`
4. **模块功能合约** — 锤击检测、蜂鸣器、UART、桩长计算的精确行为描述在 CLAUDE.md

---

## AI 容易犯的错误 (避坑指南)

### 1. 擅自改 VGA 同步极性
- **事故**: 把 `hsync/vsync` 从负极性改成 VESA 标准"正极性" → 显示器花屏
- **正确做法**: 本项目的显示器/扩展板实测对应**负极性**, VGA 时序模块已验证冻结

### 2. 把 Evaluation Mode 警告误判为代码错误
- Quartus 试用模式下编译 Success 但无法生成 .sof, 与代码无关

### 3. 给 PIN_25 加 WEAK_PULL_UP_RESISTOR
- Cyclone IV E 部分引脚不支持内部弱上拉, PIN_25 是其中之一, 编译报 Error 169057

### 4. 跨时钟域复位
- bclk 域模块(iis_slave_rx)必须用独立的 `bclk_rst_n`, 不能用 `vga_rst_n`

### 5. 修改 RTL 后未清理编译缓存
- Quartus 增量编译可能跳过已修改文件, 每次改 RTL 后必须:
```powershell
Remove-Item -Recurse -Force quartus/db, quartus/incremental_db, quartus/output_files/greybox_tmp -ErrorAction SilentlyContinue
```

### 6. M9K RAM 读写与异步复位混在同一 always 块
- `wave_buf` 推断为 M9K 失败 → 退化为 32768 个 LE → 编译失败

### 7. 只看代码不查硬件
- 用户硬件现象高于代码静态分析。优先排查 ADC → I2S → FIFO → 波形显示链路

---

## 工程转移/发给别人时的检查清单

1. **确认 Quartus 版本**: 本项目用 Quartus Prime 18.0 Standard Edition
2. **IP 核完整性**: 确认 `quartus/ip_core/clk_gen/` 和 `quartus/ip_core/uart_fifo/` 目录存在且含 .qip + .v 文件
3. **⚠️ 不要使用 `fifo_uart/`**: 该目录是废弃的重复 IP, QSF 中用的是 `uart_fifo/`
4. **清理编译缓存后完全重编译**: 新机器上 `db/` 不兼容, 必须删除后全编译
5. **检查 .sof 生成**: 确认 `output_files/vga_oscilloscope.sof` 存在
6. **烧录后验证**: 按 PIN_54 蜂鸣器响, VGA 显示坐标轴, 敲击后显示波形

---

## 代码风格规范

### 命名约定
| 类型 | 规则 | 示例 |
|------|------|------|
| 模块名 | snake_case | `vga_ctrl_simple` |
| 参数/宏 | UPPER_CASE | `DATA_WIDTH` |
| 信号后缀 `_n` | 低有效 | `rst_n` |
| 信号后缀 `_vld` | 数据有效 | `ad_data_vld` |
| 信号后缀 `_en` | 使能 | `display_en` |

### 时钟与复位
- 异步复位、同步释放: `always @(posedge clk or negedge rst_n)`
- 复位信号命名为 `xxx_rst_n`, 低电平有效

---

## 目录结构

```
God1.0/
├── CLAUDE.md                    ← 权威约束文件 (必读)
├── AGENTS.md                    ← 本文件 (路由 + 避坑 + 规范)
├── rtl/                         ← 全部 RTL 源码
├── quartus/
│   ├── vga_oscilloscope.qpf/.qsf/.sdc
│   └── ip_core/
│       ├── clk_gen/             ← PLL IP (50M→40M/24.576M/6.144M)
│       ├── uart_fifo/           ← UART FIFO IP (16→8bit×2048)
│       └── fifo_uart/           ← ⚠️ 废弃! 勿用
└── doc/
```

---

## 变更记录

### 2026-07-28 — 花屏修复②: ov5640_capture byte_cnt 饱和
- 第一轮修复 (downsample in_range) 烧录后实测仍花屏, 串口原始流分析显示行周期从 131 变为 108
- 根因链: 实际行长 ~2620 拍 (~1310px) > byte_cnt 11bit 上限 2047 → 行中回卷, pix_x 重新从 0 计数
  → 回卷段 29 个采样点坐标 <800, 穿透 in_range 过滤 → 每行 109 个采样 (实测 108, 仿真 109 ✓)
- 旧数据亦吻合: 无过滤时 132/行 (实测 131 ✓)
- 修复: byte_cnt 计到 2047 后饱和保持 (pix_x 停 1023), 配合 downsample in_range → 每行严格 80 采样
- 数值仿真验证: 饱和+过滤 → 每帧恰好 4800 采样, 不错位不回卷
- 备份 rtl/ov5640_capture.v.bak; 已清理编译缓存
- 根治仍待修正 ov5640_config (DVP 输出尺寸 0x3808-0x380B 未生效)

### 2026-07-28 — 花屏修复: downsample 范围保护
- 诊断: OV5640 实际输出 ~1310×976 (DVP 输出尺寸 0x3808-0x380B 未生效, ISP 缩放未起作用),
  每行 ~131 个降采样点而非 80 → uart_tx_2M 顺序打包错位 (对角线花屏);
  且采样总数 ~12838 > 8192 (13bit 写地址物理深度), 回卷覆盖缓冲区前 4646 字节
- 与串口丢包无关: 460800 波特率下帧结构全部完好 (帧头/帧尾位置正确)
- 修复: downsample.v 新增 in_range (pix_x<800 && pix_y<600), 每帧严格 4800 采样,
  打包不错位、不回卷; 预览为实际场景左上角 800×600 裁切
- 备份 rtl/downsample.v.bak; 已按规程清理编译缓存 (db/incremental_db/greybox_tmp)
- Qt 上位机 (GapDetector) 经逐行审查无 bug, 未改动
- 待办: 根治需修正 ov5640_config 使 DVP 输出真正为 800×600 (可用 SignalTap 数 HREF 内字节数验证, 应为 1600, 现 ~2620)

### 2026-07-27 — OV5640 缝隙检测子系统集成
- 新增 8 个摄像头模块: ov5640_config/capture, downsample, gap_detect/vote/calibrate, uart_tx_2M/rx_2M
- vga_dual_mode_top.v 新增 OV5640 端口 + 摄像头模块链实例化
- 移除 uart_fifo_loopback 实例, UART 改用于摄像头通信
- PLL c3 (24MHz) 启用, 连接 cam_xclk
- 新增 cam_pclk 域复位同步 (pclk_rst_n)
- 指令寄存器 CDC: toggle 握手 (sys_clk → cam_pclk)
- QSF 新增 16 个 OV5640 引脚 + 8 个源文件
- SDC 新增 cam_pclk 时钟约束 + 异步时钟组 + OV5640 输入 delay
- CLAUDE.md 更新引脚映射表/模块合约/已知陷阱

### 2026-07-27 — 冻结逻辑重构 + CLAUDE.md 创建
- 锤击检测改为自动更新模式 (每次敲击自动替换冻结波形)
- 新增静默窗口机制 (连续 100ms 低于阈值才武装)
- 采集期间闭屏防残影
- 创建 CLAUDE.md 作为项目宪章

### 2026-07-26 — UART 回环 + 按键/蜂鸣器最终确定
- 从废弃/vga 工程复制 UART 回环功能
- freeze_btn 最终确定 PIN_25 (途径 PIN_53串扰→PIN_59外接坏)
- buzzer 最终确定 PIN_1 (途径 PIN_91纯输入→PIN_60无声)
- 新增 buzzer_beep.v, uart_rx.v, uart_tx.v, uart_fifo_loopback.v

### 2026-07-23~26 — 项目结构整理
- 清理 FIR 滤波通路、摄像头代码、仿真文件
- 目录扁平化、新增 .gitignore
