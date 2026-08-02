---
name: vga-display
description: |
  VGA 显示系统设计指南。当用户涉及 VGA 时序、波形显示等主题时自动触发。
  本项目采用 800×600@60Hz 分辨率，包括时序参数、波形绘制算法、字符渲染等技术。
---

# VGA 显示系统技能

本技能提供 FPGA VGA 显示系统的设计指南，包括时序生成、波形显示、字符渲染等。

> ⚠️ **VGA 时序已验证可在硬件上正常显示，严禁擅自修改**。详见项目 `AGENTS.md` 和 `project_memory.md` 的不可变基准规则。

---

## 一、VGA 时序标准

### 1.1 本项目参数 (800×600 @ 60Hz)

| 参数 | 行 (像素) | 场 (行) |
|------|-----------|---------|
| 同步 | 128 | 4 |
| 后沿 | 88 | 23 |
| 有效 | 800 | 600 |
| 前沿 | 40 | 1 |
| **总计** | **1056** | **628** |

- **像素时钟**: 40 MHz (PLL c0)
- **帧率**: 60 Hz
- **同步极性**: 负极性（同步脉冲为低电平）— 硬件匹配要求，不按 VESA 标准

### 1.2 时序控制器实现

详见项目 `rtl/vga_ctrl_simple.v`（不可变基准文件，禁止修改）。

---

## 二、波形显示模块

### 2.1 数据缓存设计

本项目使用 M9K 实现 2048 点波形缓存（8-bit 宽），覆盖 21.3ms @96kHz：

```verilog
(* ramstyle = "M9K" *) reg [7:0] wave_buf [0:2047];
```

> **M9K 推断规则**：读和写必须各自放在独立的 `always @(posedge clk)` 块中，不含异步复位，否则 Quartus 无法推断为 M9K。

### 2.2 波形显示区域与时间轴映射

本项目参数（`rtl/vga_waveform.v`）：

```verilog
parameter X_LEFT  = 10'd80;
parameter X_RIGHT = 10'd720;   // 640px 显示宽度
parameter Y_TOP   = 10'd30;
parameter Y_BTM   = 10'd569;
parameter Y_MID   = 10'd300;   // 零点基线

// 时间轴: 1.5 倍压缩, 640px × 1.5 = 960 点 = 10.0ms @96kHz
wire [9:0]  x_offset = pix_x - X_LEFT;
wire [11:0] x_scaled = x_offset * 3;
wire [10:0] x_buf    = x_scaled >> 1;   // ÷2 = ×1.5
```

- 5 格 × 128px/格，每格 2ms
- X 轴标签: 0/2/4/6/8/10 (ms)

### 2.3 数据压缩 (24-bit 补码 → 8-bit 无符号)

```verilog
// 符号位 bit[23] 取反 + 高 7 位 bit[22:16]
wave_buf[wr_ptr] <= {~ad_data[23], ad_data[22:16]};
```

- 0x000000 (零点) → 128 (中点)
- 0x7FFFFF (满幅正) → 255 (顶部)
- 0x800000 (满幅负) → 0 (底部)

### 2.4 波形绘制 (点+线段连接)

```verilog
// 点绘制: 当前像素 Y 坐标 = 基线 - 样本值
wire [9:0] wave_y = 10'd428 - {2'b0, sample};
wire on_wave = in_wave_d1 && (pix_y_d1 >= wave_y - 1) && (pix_y_d1 <= wave_y + 1);

// 线段绘制: 连接相邻采样点 (x_buf 变化时锁定 wave_y_prev)
wire line_active = x_buf_chg || x_buf_chg_d1;
wire on_line = in_wave_d1 && line_active &&
    ((pix_y_d1 >= wave_y && pix_y_d1 <= wave_y_prev) ||
     (pix_y_d1 >= wave_y_prev && pix_y_d1 <= wave_y));
```

### 2.5 触发与冻结

- 锤击检测阈值: 10% × 2^23 = 838861 (≈0.3Vp-p @3V 满幅)
- snap_pulse: 锤击瞬间锁定 disp_origin = wr_ptr，波形始于原点
- 采集 20ms 后冻结显示 (cap_timer=800000 @40MHz)

---

## 三、字符渲染

### 3.1 8×8 字符 ROM

项目使用 function 实现的 8×8 字符 ROM，仅包含数字和少量字母 (t/m/s/)：

```verilog
function [7:0] char_8x8;
    input [6:0] ascii;
    input [2:0] row;
    begin
        case (ascii)
            "0": case (row) 3'd0: char_8x8=8'b00111100; ...
            // ... 详见 rtl/vga_waveform.v
        endcase
    end
endfunction
```

### 3.2 字符渲染函数

```verilog
function [1:0] draw_char_at;
    input [9:0] px, py, lx, ly;
    input [6:0] ch;
    // 返回 2'b01 = 该像素点亮该字符
endfunction
```

---

## 四、彩条测试

### 4.1 标准彩条生成

```verilog
// 8 色彩条 (每色 100 像素宽)
wire [2:0] color_bar = pix_x[9:7];  // 800/8 = 100
wire [3:0] r = (color_bar[2]) ? 4'hF : 4'h0;
wire [3:0] g = (color_bar[1]) ? 4'hF : 4'h0;
wire [3:0] b = (color_bar[0]) ? 4'hF : 4'h0;
assign rgb = {r, g, b};
```

---

## 五、常见问题排查

### 5.1 显示花屏

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 水平条纹 | 行同步不匹配 | 检查 h_cnt 周期 |
| 垂直滚动 | 场同步不匹配 | 检查 v_cnt 周期 |
| 随机噪点 | 时钟域问题 | 检查 FIFO 同步 |
| 颜色错位 | RGB 位序 | 检查引脚分配 |
| 整屏花屏 | 同步极性错误 | 本项目用负极性，勿改 |

### 5.2 波形不稳定

1. **触发问题**: 检查触发阈值设置 (838861 = 10% 满幅)
2. **FIFO 溢出/下溢**: 检查 rdreq 逻辑 (用 rdempty 下降沿单脉冲读)
3. **采样率不匹配**: 确认 BCLK 配置 (6.144MHz = 64×96kHz)
4. **波形跳变**: 检查数据压缩位选 (必须用 bit[23]+bit[22:16])

### 5.3 波形不显示

排查顺序：
1. FIFO 有无数据 (检查 fifo_empty / wrreq)
2. bypass_valid 与 bypass_data 是否对齐 (DCFIFO rdreq→q 延迟 1 拍)
3. wave_buf 写入是否被 freeze 阻止
4. display_en / wave_freeze 逻辑是否正确

---

## 六、性能指标

| 参数 | 典型值 | 说明 |
|------|--------|------|
| 帧率 | 60 Hz | 标准 VGA |
| 采样率 | 96 kHz | PCM1808 |
| 显示时长 | 10.0 ms | 960 点 @96kHz |
| 缓冲深度 | 2048 点 | 21.3ms @96kHz |
| 触发精度 | ±1 采样点 | ≈10.4µs @96kHz |
| 显示延迟 | <2 帧 | FIFO+处理 |

---

## 七、参考资源

- 项目 `rtl/vga_ctrl_simple.v`: 时序控制器实现 (不可变)
- 项目 `rtl/vga_waveform.v`: 波形显示实现
- 项目 `doc/VGA示波器原理详解.md`: 原理说明
- PCM1808 Datasheet (SLES177B): ADC 数据手册