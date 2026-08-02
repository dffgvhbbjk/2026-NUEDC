# God1.0 — FPGA 基桩检测示波器

基于 Altera Cyclone IV E EP4CE6E22C8 (TQFP-144)，用于低应变反射波法桩基完整性检测。

## ⛔ 不可修改的约束 (INVIOLABLE)

以下内容除非用户**明确要求**，否则**任何人不得修改**。AI 助手必须遵守：

---

### 1. 引脚映射 (PIN MAP) — 绝对不可自行更改

| 信号名 | 引脚 | 功能 | 备注 |
|---|---|---|---|
| `sys_clk` | PIN_23 | 50MHz 晶振 | |
| `sys_rst_n` | **PIN_52** | 系统复位 (按下→GND, 低有效) | 内部弱上拉 ON |
| `freeze_btn` | **PIN_25** | 复位回待机/清屏 | 板载自带外部上拉，**不可开内部弱上拉** |
| `test_btn` | **PIN_54** | 按下蜂鸣器响 | 内部弱上拉 ON |
| `btn_55` | PIN_55 | 备用 | 内部弱上拉 ON |
| `btn_58` | PIN_58 | 备用 | 内部弱上拉 ON |
| `btn_59` | PIN_59 | 备用 | 内部弱上拉 ON |
| `buzzer` | **PIN_1** | 蜂鸣器输出 (高电平有效) | PIN_91 是纯输入脚，不可用于输出 |
| `hsync` | PIN_32 | VGA 行同步 | |
| `vsync` | PIN_34 | VGA 场同步 | |
| `rgb[11:8]` | 30,11,7,2 | VGA 红 4-bit | |
| `rgb[7:4]` | 44,42,38,33 | VGA 绿 4-bit | |
| `rgb[3:0]` | 31,28,10,3 | VGA 蓝 4-bit | |
| `adc_mclk` | PIN_51 | PCM1808 SCKI (24.576MHz) | |
| `adc_bclk` | PIN_43 | PCM1808 BCK (6.144MHz) | |
| `adc_lrclk` | PIN_49 | PCM1808 LRCK (96kHz) | |
| `adc_data` | PIN_46 | PCM1808 DOUT | |
| `seg_sclk` | PIN_121 | 74HC595 移位时钟 | |
| `seg_rclk` | PIN_125 | 74HC595 锁存时钟 | |
| `seg_din` | PIN_127 | 74HC595 数据输入 | |
| `uart_rxd` | PIN_143 | UART 接收 (PC→FPGA) | 摄像头通信 |
| `uart_txd` | PIN_144 | UART 发送 (FPGA→PC) | 摄像头通信 |
| `cam_xclk` | PIN_112 | OV5640 主时钟 (24MHz, PLL c3) | 2.5V/8mA |
| `cam_rst_n` | PIN_114 | OV5640 复位 (低有效) | 2.5V/8mA |
| `cam_pwdn` | PIN_119 | OV5640 掉电 (高有效) | 2.5V/8mA |
| `cam_sccb_sclk` | PIN_101 | OV5640 SCCB 时钟 | 2.5V/8mA |
| `cam_sccb_sdat` | PIN_99 | OV5640 SCCB 数据 (开漏) | 2.5V/8mA |
| `cam_pclk` | PIN_110 | OV5640 像素时钟 (~24MHz) | 2.5V/8mA |
| `cam_vsync` | PIN_105 | OV5640 帧同步 | 2.5V/8mA |
| `cam_href` | PIN_98 | OV5640 行有效 | 2.5V/8mA |
| `cam_data[0]` | PIN_132 | OV5640 DVP D0 | 2.5V/8mA |
| `cam_data[1]` | PIN_128 | OV5640 DVP D1 | 2.5V/8mA |
| `cam_data[2]` | PIN_126 | OV5640 DVP D2 | 2.5V/8mA |
| `cam_data[3]` | PIN_124 | OV5640 DVP D3 | 2.5V/8mA |
| `cam_data[4]` | PIN_120 | OV5640 DVP D4 | 2.5V/8mA |
| `cam_data[5]` | PIN_111 | OV5640 DVP D5 | 2.5V/8mA |
| `cam_data[6]` | PIN_106 | OV5640 DVP D6 | 2.5V/8mA |
| `cam_data[7]` | PIN_104 | OV5640 DVP D7 | 2.5V/8mA |

**规则**：
- 所有引脚重分配必须经用户明确确认
- 修改 QSF 时必须同步更新此表
- `freeze_btn` 的 PIN_25 **不可开内部弱上拉**（板载已有外部上拉，Quartus 会报错）
- 蜂鸣器**不能**用 PIN_91（纯输入脚）、PIN_60（已验证无声）

---

### 2. 模块功能合约 (BEHAVIORAL CONTRACTS)

#### 锤击检测 + 冻结逻辑 (`vga_dual_mode_top.v` 中段)
- 上电后：VGA 仅显示坐标轴，等待首次敲击
- 敲击检测：ADC 绝对值 > 10% 满幅 (838861) 且 `bypass_valid` 有效
- 触发后：snap 显示原点 → 采集 10ms（期间闭屏）→ 冻结显示波形
- 后续敲击：自动更新为新波形，无需按任何按键
- freeze_btn 按下：复位回待机状态（清屏、解冻）
- 静默窗口：冻结后信号必须连续 100ms 低于阈值才重新武装（防振铃自触发）

#### 蜂鸣器 (`buzzer_beep.v`)
- `test_btn` (PIN_54) 按下 → 蜂鸣器响；松开 → 停
- 20ms 消抖，同步复位设计

#### VGA 显示 (`vga_waveform.v`)
- 800×600@60Hz，2048 点循环缓冲
- 波形显示区域：X=80~720，Y=30~569
- 显示使能 `display_en`：0=仅坐标轴，1=显示波形
- 采集期间 `display_en=0`（闭屏防残影），冻结后 `display_en=1`

#### UART 回环 (`uart_fifo_loopback.v`) — 已停用
- 2Mbps，sys_clk 域
- RX → DCFIFO (16bit→8bit) → TX
- 仅回环测试用，独立于 VGA/ADC 链路
- **已从顶层移除**, UART 现用于 OV5640 摄像头通信 (uart_rx_2M + uart_tx_2M)

#### OV5640 缝隙检测子系统 (cam_pclk 域 + sys_clk 域)
- **ov5640_config**: 控制器用野火官方 52_ov5640_vga_640x480 的 i2c_ctrl.v (250kHz, 标准START/STOP; **2026-08-01 去掉ACK卡死**: 官方版"从机不应答则停住", 在本板SDA应答临界时卡死在cfg_progress=0, 配置完全写不进; 现应答时钟照走不检查ACK, I2C从机照样收到写入), 配置表 257 寄存器 (官方 251 + 手动曝光初值 6; 曾试末尾软复位0x3008=0x82/0x02 会把寄存器清回默认→64fps无图, 已撤)
  → 输出 **640×480 RGB565** (0x4300=0x61), PLL 0x3035=0x41/0x3036=0x72 (野火官方 STM32 15fps 配置, PCLK≈28.5MHz。**关键约束**: 本板采集逻辑按 44MHz 布线, 30fps+ 需 52-70MHz PCLK 接不住 → href 采样不到/图像全零; 15fps 在承受范围内已实测出图), HTS=1896 (0x0768), VTS=984 (0x03D8)
- **采集链**: ov5640_config → ov5640_capture → downsample → gap_detect → gap_vote → gap_calibrate
- **ov5640_capture**: 取 RGB565 每像素第 1 字节 (红通道) 作灰度; 每行 640 像素 (pix_x 0-639)
- **downsample**: 640×480 → 64×48 (因子 10, 乘倒数无除法); 范围保护 pix_x<800 && pix_y<600, 每帧 3072 个采样
- **gap_detect**: 对所有行检测, 行末判定 `pix_x>=639` (640 宽下原 pix_x==799 永不触发); gap_left/right 为 /10 的 64-wide 坐标, 哨兵 0x7F
- **UART TX** (uart_tx_2M): 帧头4B + 图像3072B + 结果8B + 帧尾4B = **3088B/帧, 2Mbps 波特率**
  (2026-08-01 CH343 升级: CH340 在 2M 下丢包 → 换 CH343, 原定 5M 但 PC 端(串口助手/CH343 驱动)跑不通
  → 回退 2M, 50MHz/2M=25 整除 0% 误差; CH343 在 2M 下比 CH340 稳定)
  帧缓冲 = **两块 3072B ping-pong 双缓冲** (pclk 写一块, clk 读另一块), 图像数据经 frame_sync 同步触发发送
- **UART RX** (uart_rx_2M): 2Mbps 波特率, 解析 PC 指令 (0x5A cmd hi lo cksum), 设置阈值/标定/检测行
- **CDC**: 指令寄存器用 toggle 握手从 sys_clk 传到 cam_pclk; frame_sync 用 3级FF同步到 sys_clk;
  降采样图像数据用 samp_tog toggle + 慢变数据锁存从 pclk 过到 clk 域 (采样间隔 ≥417ns)
- **cam_pclk**: OV5640 输出的 ~24MHz 像素时钟, 独立时钟域
- **cam_xclk**: PLL c3 (24MHz) 输出到 OV5640 XCLK 引脚
- **capture byte_cnt 饱和**: 行长 >2047 拍时 byte_cnt 保持不回卷 (pix_x 停 1023),
  防止回卷段穿透 in_range 过滤 (2026-07-28 花屏修复②)
- 摄像头子系统与 VGA/ADC 波形链路并行运行, 互不干扰

#### 桩长计算 (`pile_length_calc.v`)
- 公式：L = c × ΔT / 2，波速 3800 m/s
- 结果送 74HC595 数码管显示（0.1m 分辨率）

#### I2S 接收 (`iis_slave_rx.v`)
- PCM1808 从机模式，24-bit，96kHz 采样
- FPGA 输出 BCLK/LRCLK（主模式）

---

### 3. 文件结构 — 不可删除/重命名

```
God1.0/
├── CLAUDE.md                    ← 本文件
├── rtl/
│   ├── vga_dual_mode_top.v      ← 顶层 (禁止随意改名)
│   ├── vga_ctrl_simple.v        ← VGA 时序
│   ├── vga_waveform.v           ← 波形显示
│   ├── iis_slave_rx.v           ← I2S ADC 接收
│   ├── audio_fifo.v             ← ADC FIFO 封装
│   ├── pile_length_calc.v       ← 桩长计算
│   ├── seg_display.v            ← 数码管驱动
│   ├── buzzer_beep.v            ← 蜂鸣器
│   ├── uart_rx.v                ← UART 接收 (被 uart_rx_2M 复用)
│   ├── uart_tx.v                ← UART 发送 (被 uart_tx_2M 复用)
│   ├── uart_fifo_loopback.v     ← UART 回环 (已停用, 保留备份)
│   ├── i2c_ctrl.v               ← SCCB/I2C 控制器 (野火官方 52_ov5640_vga_640x480, 250kHz, ACK检查+标准STOP)
│   ├── ov5640_config.v          ← OV5640 SCCB 配置 (官方 i2c_ctrl + 257 寄存器表, 640×480 RGB565)
│   ├── ov5640_capture.v         ← OV5640 图像采集 + 单通道提取
│   ├── downsample.v             ← 降采样 64×48 硬边界裁剪 (x_div10<64 && y_div10<48, 摄像头实际~1280×960时取左上区域, 防缓冲溢出花屏; 因子10, 乘倒数无除法)
│   ├── gap_detect.v             ← 缝隙检测状态机 (行末 pix_x>=639)
│   ├── gap_vote.v               ← 多行投票取众数
│   ├── gap_calibrate.v          ← 像素→毫米换算
│   ├── uart_tx_2M.v             ← UART 发送 + 帧打包 (2Mbps, ping-pong 双缓冲)
│   └── uart_rx_2M.v             ← UART 接收 + 指令解析 (2Mbps)
├── quartus/
│   ├── vga_oscilloscope.qsf     ← 引脚+工程设置 (修改需同步 CLAUDE.md)
│   ├── vga_oscilloscope.sdc     ← 时序约束
│   └── ip_core/
│       ├── clk_gen/             ← PLL IP (50M→40M/24.576M/6.144M/24M)
│       ├── audio_fifo/          ← ADC FIFO IP (24bit×256)
│       └── uart_fifo/           ← UART FIFO IP (16→8bit×2048, 回环用)
├── sim/
│   ├── tb_ov5640_config.v       ← SCCB 配置仿真 (标准从机模型, 257 寄存器逐条比对)
│   └── run.do                   ← ModelSim 运行脚本 (vsim -c -do run.do)
└── doc/
```

---

### 4. 修改规则 (MODIFICATION RULES)

- **禁止**自行添加/删除/重命名顶层端口
- **禁止**自行更改任何引脚分配
- **禁止**修改 PLL 时钟频率配置
- **禁止**删除或绕过任何现有模块的功能
- **禁止**将 `freeze_btn` 从 PIN_25 移到其他引脚（PIN_52 邻脚有串扰，PIN_59 外接按键不可靠）
- 新增功能应通过备用引脚（btn_55/58/59）或未使用的 I/O 实现
- 修改 QSF 后必须同步更新本文档的引脚映射表
- 模块实例化名称（`u_*`）尽量保持稳定，方便增量编译

---

### 5. 已知陷阱 (KNOWN PITFALLS)

| 陷阱 | 说明 |
|---|---|
| PIN_52/53 邻脚串扰 | PIN_53 与复位脚 PIN_52 物理相邻，按下 PIN_53 可能误触发复位 |
| PIN_91 纯输入 | TQFP-144 封装中 PIN_91 是 dedicated input，不可做输出 |
| PIN_25 无内部弱上拉 | Cyclone IV 部分引脚不支持 WEAK_PULL_UP，PIN_25 是其中之一 |
| 异步复位 vs 同步复位 | `vga_rst_n` 从 0→1 翻转（无 negedge），vga_clk 域模块须用同步复位或异步复位同步释放 |
| bclk 域复位 | iis_slave_rx 必须用独立的 bclk_rst_n（bclk 域同步释放），跨域用 vga_rst_n 会导致 Recovery 时序违例 |
| pclk 域复位 | OV5640 摄像头模块必须用独立的 pclk_rst_n（cam_pclk 域同步释放），跨域用 sys_rst_n 会导致 Recovery 时序违例 |
| fifo_uart vs uart_fifo | 两个 QIP 指向同一 IP，工程中只保留 `uart_fifo.qip`，**不要**同时添加两个 |
| OV5640 引脚无弱上拉 | OV5640 引脚不加 WEAK_PULL_UP_RESISTOR (Cyclone IV E 部分引脚不支持) |
| SCCB sdat 开漏 | cam_sccb_sdat 必须设 AUTO_OPEN_DRAIN_PINS ON，否则三态实现可能时序异常 |
| cam_pclk 是外部时钟 | cam_pclk 不是 PLL 派生，需在 SDC 中 create_clock 单独约束，并加入异步时钟组 |
| 多 bit CDC | 指令寄存器(threshold/calib_coef/detect_row)从 sys_clk 传到 cam_pclk 必须用 toggle 握手，不能直接采样 |
| cbx_lpm 系列 DLL | 本机 Quartus Lite 的 cbx_lpm_divide.dll/cbx_lpm_mult.dll 加载失败，HDL 里的 `/` 除法与变量×变量 `*` 乘法会报 Error 272000。**新代码一律用移位加法/乘倒数/计数器实现**，不要写可综合的 `/` 运算符（除 localparam 编译期常量） |
