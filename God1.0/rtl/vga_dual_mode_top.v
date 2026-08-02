//======================================================================
// Module: vga_dual_mode_top
// Function: VGA波形显示顶层 — ADC → I2S → 波形 → 桩长
// Date:    2026-07-16
//
// 按键:
//   sys_rst_n  (PIN_52) 系统复位, 低有效
//   freeze_btn (PIN_25) 复位回待机状态(清屏), 内置20ms消抖
//   test_btn   (PIN_54) 测试键, 按下蜂鸣器响
//   btn_55/58/59 (PIN_55/58/59) 备用按键
//
// 蜂鸣器:
//   buzzer (PIN_1) 高电平触发, 直连蜂鸣器模块I/O口
//
// 时钟架构:
//   50MHz → PLL → 40MHz(VGA) + 24.576MHz(ADC MCLK) + 6.144MHz(BCK)
//======================================================================

module vga_dual_mode_top (
    input  wire         sys_clk,        // 50MHz 系统时钟
    input  wire         sys_rst_n,      // 系统复位, 低有效 (PIN_52)

    // 按键
    input  wire         freeze_btn,     // 波形冻结/解冻 (PIN_25)
    input  wire         test_btn,       // 测试按键, 按下蜂鸣器响 (PIN_54)
    input  wire         btn_55,         // 备用按键 (PIN_55)
    input  wire         btn_58,         // 备用按键 (PIN_58)
    input  wire         btn_59,         // 备用按键 (PIN_59)

    // 蜂鸣器
    output wire         buzzer,         // 蜂鸣器, 高电平触发 (PIN_1)

    // ADC接口
    output wire         adc_mclk,       // SCKI: 24.545MHz
    output wire         adc_bclk,       // BCK:  6.136MHz
    output wire         adc_lrclk,      // LRCK: 95.88kHz
    input  wire         adc_data,       // DOUT: ADC数据输入

    // VGA接口
    output wire         hsync,
    output wire         vsync,
    output wire [11:0]  rgb,

    // 74HC595 数码管 — 显示桩长
    output wire         seg_din,
    output wire         seg_sclk,
    output wire         seg_rclk,

    // UART 串口 (sys_clk 域, 2Mbps, 摄像头通信)
    input  wire         uart_rxd,       // PC → FPGA (PIN_143)
    output wire         uart_txd,       // FPGA → PC (PIN_144)

    // OV5640 摄像头接口
    input  wire [7:0]   cam_data,       // DVP 像素数据 [7:0]
    input  wire         cam_pclk,       // 像素时钟 (~24MHz, OV5640 输出)
    input  wire         cam_href,       // 行有效
    input  wire         cam_vsync,      // 帧同步
    inout  wire         cam_sccb_sclk,  // SCCB 时钟
    inout  wire         cam_sccb_sdat,  // SCCB 数据 (开漏)
    output wire         cam_xclk,       // 24MHz 主时钟 (PLL c3)
    output wire         cam_rst_n,      // 复位 (低有效)
    output wire         cam_pwdn        // 掉电控制 (高有效)
);

//======================================================================
// PLL — 统一时钟生成
//   c0: 40MHz      VGA像素时钟 (800×600@60Hz)
//   c1: 24.545MHz  ADC主时钟 SCKI (256fs @96kHz)
//   c2: 6.136MHz   ADC位时钟 BCK (64fs @96kHz)
//   c3: 24MHz      摄像头 XCLK (OV5640 主时钟)
//======================================================================
wire        vga_clk;
wire        pll_locked;
wire        clk_24m;        // PLL c3: 24MHz → OV5640 XCLK

clk_gen u_clk_gen (
    .areset (~sys_rst_n),
    .inclk0 (sys_clk),
    .c0     (vga_clk),      // 40MHz
    .c1     (adc_mclk),     // 24.576MHz
    .c2     (adc_bclk),     // 6.144MHz
    .c3     (clk_24m),      // 24MHz → OV5640 XCLK
    .locked (pll_locked)
);

// OV5640 XCLK = PLL c3 (24MHz)
assign cam_xclk = clk_24m;

// OV5640 上电时序控制 (严格遵循 datasheet: 6ms PWDN + 2ms RST_N + 21ms SCCB)
wire poweron_done;
wire cam_rst_n_w;
wire cam_pwdn_w;

ov5640_poweron u_ov5640_poweron (
    .clk          (sys_clk),
    .rst_n        (sys_rst_n),
    .pll_locked   (pll_locked),
    .cam_pwdn     (cam_pwdn_w),
    .cam_rst_n    (cam_rst_n_w),
    .poweron_done (poweron_done)
);

assign cam_rst_n = cam_rst_n_w;
assign cam_pwdn  = cam_pwdn_w;

//======================================================================
// VGA域复位同步
//======================================================================
reg [2:0] rst_sync;
wire      vga_rst_n;

always @(posedge vga_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        rst_sync <= 3'b000;
    else
        rst_sync <= {rst_sync[1:0], pll_locked};
end
assign vga_rst_n = rst_sync[2];

//======================================================================
// BCLK域复位同步 (异步复位, 同步释放到 adc_bclk)
//   修复: iis_slave_rx/lrclk 原直接用 vga_clk 域产生的 vga_rst_n,
//         跨域异步释放导致 bclk 域 Recovery 时序违例 (sta 报告 -2.2ns)
//======================================================================
reg [2:0] bclk_rst_sync;
wire      bclk_rst_n;

always @(posedge adc_bclk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        bclk_rst_sync <= 3'b000;
    else
        bclk_rst_sync <= {bclk_rst_sync[1:0], pll_locked};
end
assign bclk_rst_n = bclk_rst_sync[2];

//======================================================================
// PCLK域复位同步 (异步复位, 同步释放到 cam_pclk)
//   OV5640 摄像头像素时钟域, 用于图像采集/缝隙检测/降采样
//======================================================================
reg [2:0] pclk_rst_sync;
wire      pclk_rst_n;

always @(posedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        pclk_rst_sync <= 3'b000;
    else
        pclk_rst_sync <= {pclk_rst_sync[1:0], pll_locked};
end
assign pclk_rst_n = pclk_rst_sync[2];

//======================================================================
// 固定示波器模式 (摄像头已移除)
wire mode_sel = 1'b0;

//======================================================================
// 锤击检测 + 波形提取
//   待机 → 首次敲击 → snap原点+采集10ms(屏闭) → 冻结显示
//   后续敲击 → 闭屏→snap新原点→采集10ms→冻结→显示→静默100ms→待敲...
//   freeze_btn → 复位回待机状态
//
//   静默窗口: 采集完成后, 信号必须连续 100ms 低于阈值才重新武装.
//            期间若有任何超阈值信号 → 计时器重置, 彻底杜绝振铃自触发.
//   采集闭屏: 采集期间 display_en=0, 避免新旧波形混叠产生残影.
//======================================================================
// 注: bypass_data/bypass_valid 在本文件后部 FIFO 读出处理处赋值,
//     此处提前声明供锤击检测使用
//======================================================================
reg  [23:0] bypass_data;
reg         bypass_valid;

reg [1:0]  frz_sync;
reg [19:0] frz_cnt;
reg        frz_stable;
reg        frz_stable_d1;
reg        wave_freeze;        // 1=冻结缓冲写入
reg        display_en;         // 1=显示波形 (采集期间闭屏防残影, 冻结后显示)
reg        hammer_latched;     // 1=已检测到锤击, 采集倒计时中
reg [21:0] cap_timer;          // 采集倒计时 10ms @40MHz
reg        snap_pulse;         // 对齐显示窗口
reg        cooldown;           // 1=静默窗口, 禁止新触发 (防振铃自触发)
reg [21:0] cooldown_timer;     // 静默倒计时 100ms @40MHz, 超阈值则重置

// 锤击检测: 使用 bypass_data (原始ADC, 零延迟)
//   阈值 = 10%×2^23 = 838861, 对应ADC高8位≈13(10%满幅)
wire [23:0] adc_abs;
assign adc_abs = bypass_data[23] ? (~bypass_data + 1'b1) : bypass_data;
wire hammer_hit = (adc_abs > 24'd838861) && bypass_valid;

always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n) begin
        frz_sync       <= 2'b11;
        frz_cnt        <= 20'd0;
        frz_stable     <= 1'b1;
        frz_stable_d1  <= 1'b1;
        wave_freeze    <= 1'b0;
        display_en     <= 1'b0;
        hammer_latched <= 1'b0;
        cap_timer      <= 22'd0;
        snap_pulse     <= 1'b0;
        cooldown       <= 1'b0;
        cooldown_timer <= 22'd0;
    end
    else begin
        snap_pulse <= 1'b0;

        // ---- 按键消抖 (PIN_25 = 复位回待机状态) ----
        frz_sync <= {frz_sync[0], freeze_btn};
        if (frz_sync[1] != frz_stable) begin
            frz_cnt    <= 20'd0;
            frz_stable <= frz_sync[1];
        end
        else if (frz_cnt < 20'd800_000) begin
            frz_cnt <= frz_cnt + 1'b1;
        end
        else begin
            frz_stable_d1 <= frz_stable;
            if (frz_stable_d1 && !frz_stable) begin
                // 按键 → 复位: 清显示, 解冻, 回到待机等待首次敲击
                wave_freeze    <= 1'b0;
                display_en     <= 1'b0;
                hammer_latched <= 1'b0;
                cap_timer      <= 22'd0;
                cooldown       <= 1'b0;
                cooldown_timer <= 22'd0;
            end
        end

        // ---- 静默窗口倒计时 (超阈值即重置, 连续100ms低于阈值才武装) ----
        if (cooldown) begin
            if (hammer_hit) begin
                cooldown_timer <= 22'd0;   // 信号仍超阈值 → 重置计时
            end else begin
                cooldown_timer <= cooldown_timer + 1'b1;
            end
            // 100ms @ 40MHz = 4,000,000 周期
            if (cooldown_timer == 22'd4000000) begin
                cooldown       <= 1'b0;
                cooldown_timer <= 22'd0;
            end
        end

        // ---- 锤击检测 → snap原点 + 开始采集 (闭屏, 防残影) ----
        // 触发条件: 锤击有效 + 未在采集中 + 不在静默窗口
        if (hammer_hit && !hammer_latched && !cooldown) begin
            hammer_latched <= 1'b1;
            cap_timer      <= 22'd0;
            snap_pulse     <= 1'b1;   // 敲击瞬间对齐disp_base=wr_ptr
            wave_freeze    <= 1'b0;   // 解冻以写入新数据
            display_en     <= 1'b0;   // 采集期间闭屏, 避免新旧波形混叠残影
        end

        // ---- 采集10ms后 → 冻结 + 显示 + 进入静默窗口 ----
        if (hammer_latched) begin
            cap_timer <= cap_timer + 1'b1;
            if (cap_timer == 22'd400000) begin  // 10ms @ 40MHz
                wave_freeze    <= 1'b1;
                display_en     <= 1'b1;   // 采集完成, 显示干净新波形
                hammer_latched <= 1'b0;
                cap_timer      <= 22'd0;
                cooldown       <= 1'b1;   // 进入静默窗口
                cooldown_timer <= 22'd0;
            end
        end
    end
end

//======================================================================
// 备用按键哑连接 (防止 Quartus 优化掉未用引脚)
//======================================================================
wire dummy_spare;
assign dummy_spare = btn_55 ^ btn_58 ^ btn_59;

//======================================================================
// 蜂鸣器控制 (buzzer_beep 模块)
//   按下 test_btn → 蜂鸣器响, 松开 → 停
//======================================================================
buzzer_beep u_buzzer_beep (
    .clk   (vga_clk),
    .rst_n (vga_rst_n),
    .btn   (test_btn),
    .buzzer(buzzer)
);

//======================================================================
// VGA时序控制器 (共享)
//======================================================================
wire [9:0]  vga_pix_x;
wire [9:0]  vga_pix_y;
wire        vga_rgb_valid;

vga_ctrl_simple u_vga_ctrl (
    .vga_clk   (vga_clk),
    .sys_rst_n (vga_rst_n),
    .pix_x     (vga_pix_x),
    .pix_y     (vga_pix_y),
    .rgb_valid (vga_rgb_valid),
    .hsync     (hsync),
    .vsync     (vsync)
);


//======================================================================
// 示波器路径
//======================================================================

// --- LRCK生成 (BCK / 64 = 6.144MHz / 64 = 96kHz) ---
reg [5:0] lrclk_div;
reg       lrclk_reg;

always @(posedge adc_bclk or negedge bclk_rst_n) begin
    if (!bclk_rst_n) begin
        lrclk_div <= 6'd0;
        lrclk_reg <= 1'b0;
    end
    else if (lrclk_div == 6'd31) begin
        lrclk_div <= 6'd0;
        lrclk_reg <= ~lrclk_reg;
    end
    else begin
        lrclk_div <= lrclk_div + 1'b1;
    end
end
assign adc_lrclk = lrclk_reg;

// --- I2S从机接收 ---
wire [23:0] adc_l_data;
wire [23:0] adc_r_data;
wire        adc_l_valid;
wire        adc_r_valid;

iis_slave_rx #(
    .DATA_WIDTH (24),
    .BCLK_DIV   (32)
) u_iis_rx (
    .rst_n  (bclk_rst_n),
    .bclk   (adc_bclk),
    .lrclk  (adc_lrclk),
    .sdin   (adc_data),
    .l_data (adc_l_data),
    .r_data (adc_r_data),
    .l_valid(adc_l_valid),
    .r_valid(adc_r_valid)
);

// --- 音频FIFO ---
reg         fifo_rdreq;
wire [23:0] fifo_q;
wire        fifo_empty;
wire        fifo_full;
wire [7:0]  fifo_level;

audio_fifo u_adc_fifo (
    .data    (adc_l_data),
    .wrclk   (adc_bclk),
    .wrreq   (adc_l_valid),
    .rdclk   (vga_clk),
    .rdreq   (fifo_rdreq),
    .q       (fifo_q),
    .rdempty (fifo_empty),
    .wrfull  (fifo_full),
    .rdusedw (fifo_level),
    .wrusedw ()
);

// FIFO读取控制: 电平模式清空积压 + rdempty下降沿标记新样本
//   电平 rdreq=!empty 清空 FIFO 积压, 避免 fifo_full 数据链断裂
//   rdempty 下降沿标记新样本到来, bypass_valid 仅在新样本时拉高
//   DCFIFO normal mode: rdreq@T → q@T+1
reg fifo_empty_d1;
wire fifo_pop = fifo_empty_d1 && !fifo_empty;  // 空→非空: 新样本到来
always @(posedge vga_clk) fifo_empty_d1 <= fifo_empty;

always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n)
        fifo_rdreq <= 1'b0;
    else
        fifo_rdreq <= !fifo_empty;  // 电平模式: 有数据就读, 清空积压
end

//======================================================================
// 数据路径: bypass_valid 与 bypass_data 对齐
//   fifo_pop@T → bypass_valid@T+1,  fifo_q@T → bypass_data@T+1  ✓
//======================================================================

// 原始ADC数据 (I2S FIFO 读出, 直通波形/桩长计算)
always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n) begin
        bypass_data  <= 24'd0;
        bypass_valid <= 1'b0;
    end else begin
        bypass_data  <= fifo_q;
        bypass_valid <= fifo_pop;   // 与 fifo_q 同周期对齐
    end
end

// (FIR 滤波通路已移除 — 波形显示与桩长计算直接使用原始 ADC 数据 bypass_data)

//======================================================================
// 桩长计算 (低应变反射波法)
//   L = c × ΔT / 2
//   波速默认 3800 m/s, 可通过参数修改
//======================================================================
localparam [15:0] WAVE_SPEED = 16'd38000;  // 混凝土纵波波速 ×10 (3800.0 m/s)

wire [15:0] pile_len;
wire        pile_len_valid;

pile_length_calc u_pile_calc (
    .clk          (vga_clk),
    .rst_n        (vga_rst_n),
    .adc_data     (bypass_data),
    .adc_data_vld (bypass_valid),
    .wave_speed   (WAVE_SPEED),
    .pile_length  (pile_len),
    .length_valid (pile_len_valid)
);

//======================================================================
// 数码管显示 (74HC595 × 8位) — 显示桩长
//   pile_length 单位 cm → 四舍五入转 dm 后送显 (0.1m 分辨率)
//   例: 1250cm → 125 → 显示 "12.5" (米)
//   pile_len_valid 电平有效; seg_display 内部检测数值变化才重发,
//   每次新测量完成自动更新显示
//======================================================================
wire [15:0] pile_len_dm;
// 桩长 cm → dm, 四舍五入: (x+5)/10
//   用乘倒数实现: x*419431 >> 22 (419431/2^22 ≈ 0.10000014, 对 x≤65535 误差<0.01, 精确)
//   避开 HDL `/` 除法 (本机 cbx_lpm_divide.dll 加载失败, 见 gap_calibrate 惯例)
//   2 级流水线: 16-bit 常数乘法组合延迟大, 直接 assign 会破坏 40MHz 时序,
//   拆成 "加5" / "乘倒数" 两级寄存器, 并把 value_valid 同步延迟 2 拍对齐.
reg [15:0] pile_len_plus5_r;
reg [15:0] pile_len_dm_r;
reg [1:0]  pile_len_valid_d;
always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n) begin
        pile_len_plus5_r <= 16'd0;
        pile_len_dm_r    <= 16'd0;
        pile_len_valid_d <= 2'b00;
    end else begin
        pile_len_plus5_r <= pile_len + 16'd5;
        pile_len_dm_r    <= (pile_len_plus5_r * 32'd419431) >> 22;
        pile_len_valid_d <= {pile_len_valid_d[0], pile_len_valid};
    end
end
assign pile_len_dm = pile_len_dm_r;

seg_display u_seg (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .value       (pile_len_dm),
    .value_valid (pile_len_valid_d[1]),
    .seg_din     (seg_din),
    .seg_sclk    (seg_sclk),
    .seg_rclk    (seg_rclk)
);

// --- 波形显示 ---
wire [11:0] osc_rgb;

// 触发电平 (固定参数, 当前未使用)
wire [7:0] trigger_level = 8'd5;

vga_waveform u_waveform (
    .vga_clk       (vga_clk),
    .rst_n         (vga_rst_n),
    .ad_data       (bypass_data),
    .ad_data_vld   (bypass_valid),
    .freeze        (wave_freeze),
    .snap          (snap_pulse),
    .display_en    (display_en),
    .pix_x         (vga_pix_x),
    .pix_y         (vga_pix_y),
    .rgb_valid     (vga_rgb_valid),
    .rgb           (osc_rgb)
);

//======================================================================

// 仅示波器模式
assign rgb = osc_rgb;

//======================================================================
// OV5640 缝隙检测子系统 (独立于 VGA/ADC 波形链路并行运行)
//   cam_pclk 域: 采集 → Y提取 → 降采样 → 缝隙检测 → 投票 → 标定
//   sys_clk 域:  UART 收发 (指令解析 + 帧打包发送)
//======================================================================

// --- OV5640 SCCB 配置 (sys_clk 域) ---
//   等待上电时序完成 (29ms) 后才开始 SCCB 配置, 确保 XCLK 稳定
wire config_done;
wire [8:0] cfg_progress;   // 配置进度 (诊断: 已写入寄存器数)

ov5640_config u_ov5640_config (
    .clk        (sys_clk),
    .rst_n      (sys_rst_n & poweron_done),
    .sclk       (cam_sccb_sclk),
    .sdat       (cam_sccb_sdat),
    .config_done(config_done),
    .cfg_progress(cfg_progress)
);

// --- OV5640 图像采集 + Y 提取 (pclk 域) ---
wire [7:0]  y_data;
wire        y_valid;
wire [9:0]  pix_x;
wire [9:0]  pix_y;
wire        frame_sync;

ov5640_capture u_ov5640_capture (
    .pclk       (cam_pclk),
    .rst_n      (pclk_rst_n),
    .cam_data   (cam_data),
    .cam_href   (cam_href),
    .cam_vsync  (cam_vsync),
    .y_data     (y_data),
    .y_valid    (y_valid),
    .pix_x      (pix_x),
    .pix_y      (pix_y),
    .frame_sync (frame_sync)
);

// --- 降采样 640×480 → 64×48 (pclk 域) ---
wire [7:0]  ds_y_data;
wire        ds_y_valid;
wire [6:0]  ds_pix_x;
wire [5:0]  ds_pix_y;

downsample u_downsample (
    .pclk       (cam_pclk),
    .rst_n      (pclk_rst_n),
    .y_data     (y_data),
    .y_valid    (y_valid),
    .pix_x      (pix_x),
    .pix_y      (pix_y),
    .ds_y_data  (ds_y_data),
    .ds_y_valid (ds_y_valid),
    .ds_pix_x   (ds_pix_x),
    .ds_pix_y   (ds_pix_y)
);

// --- 指令寄存器 (sys_clk 域) + CDC 到 pclk 域 ---
//   PC 指令: 0x01=阈值, 0x02=标定系数, 0x03=检测行, 0x04=开始/停止
wire [7:0]  cmd;
wire [15:0] cmd_data;
wire        cmd_valid;

reg [7:0]   threshold_reg;     // 二值化阈值 (默认 80)
reg [15:0]  calib_coef_reg;    // 标定系数 K×10000 (默认 1176 ≈ 0.1176 mm/pix)
reg [9:0]   detect_row_reg;    // 检测行 (默认 300)
reg         capture_en_reg;    // 采集使能 (默认 1=开始, 0x04 命令控制)

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        threshold_reg  <= 8'd80;
        calib_coef_reg <= 16'd1176;
        detect_row_reg <= 10'd300;
        capture_en_reg <= 1'b1;    // 上电默认开始采集
    end else if (cmd_valid) begin
        case (cmd)
            8'h01: threshold_reg  <= cmd_data[7:0];
            8'h02: calib_coef_reg <= cmd_data;
            8'h03: detect_row_reg <= cmd_data[9:0];
            8'h04: capture_en_reg <= cmd_data[0];   // 0x04: 1=开始, 0=停止
            default: ;
        endcase
    end
end

// CDC 握手: 指令更新时翻转标志, pclk 域检测变化后安全采样
reg cmd_update;
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)
        cmd_update <= 1'b0;
    else if (cmd_valid)
        cmd_update <= ~cmd_update;   // 每次指令更新翻转
end

reg        cmd_update_d1, cmd_update_d2, cmd_update_d3;
always @(posedge cam_pclk or negedge pclk_rst_n) begin
    if (!pclk_rst_n) begin
        cmd_update_d1 <= 1'b0;
        cmd_update_d2 <= 1'b0;
        cmd_update_d3 <= 1'b0;
    end else begin
        cmd_update_d1 <= cmd_update;
        cmd_update_d2 <= cmd_update_d1;
        cmd_update_d3 <= cmd_update_d2;
    end
end
wire cmd_update_pulse = cmd_update_d2 ^ cmd_update_d3;

reg [7:0]   threshold_pclk;
reg [15:0]  calib_coef_pclk;
reg [9:0]   detect_row_pclk;
always @(posedge cam_pclk or negedge pclk_rst_n) begin
    if (!pclk_rst_n) begin
        threshold_pclk  <= 8'd80;
        calib_coef_pclk <= 16'd1176;
        detect_row_pclk <= 10'd300;
    end else if (cmd_update_pulse) begin
        threshold_pclk  <= threshold_reg;
        calib_coef_pclk <= calib_coef_reg;
        detect_row_pclk <= detect_row_reg;
    end
end

// --- 缝隙检测 (pclk 域) ---
wire [9:0]  gap_pix;
wire        gap_valid;
wire [6:0]  gap_left;
wire [6:0]  gap_right;

gap_detect u_gap_detect (
    .pclk       (cam_pclk),
    .rst_n      (pclk_rst_n),
    .y_data     (y_data),
    .y_valid    (y_valid),
    .pix_x      (pix_x),
    .pix_y      (pix_y),
    .threshold  (threshold_pclk),
    .detect_row (detect_row_pclk),
    .frame_sync (frame_sync),
    .gap_pix    (gap_pix),
    .gap_valid  (gap_valid),
    .gap_left   (gap_left),
    .gap_right  (gap_right)
);

// --- 帧级 gap_valid: 本帧至少有一个有效行检测 ---
reg frame_gap_valid;
always @(posedge cam_pclk or negedge pclk_rst_n) begin
    if (!pclk_rst_n)
        frame_gap_valid <= 1'b0;
    else if (frame_sync)
        frame_gap_valid <= 1'b0;    // 新帧清零
    else if (gap_valid)
        frame_gap_valid <= 1'b1;    // 本帧检测到有效缝隙
end

// --- 多行投票 (pclk 域) ---
wire [9:0]  gap_pix_stable;
wire        stable_valid;

gap_vote u_gap_vote (
    .pclk          (cam_pclk),
    .rst_n         (pclk_rst_n),
    .gap_pix       (gap_pix),
    .gap_valid     (gap_valid),
    .gap_pix_stable(gap_pix_stable),
    .stable_valid  (stable_valid)
);

// --- 像素→毫米换算 (pclk 域) ---
wire [15:0] gap_mm_x10;
wire        mm_valid;

gap_calibrate u_gap_calibrate (
    .clk        (cam_pclk),
    .rst_n      (pclk_rst_n),
    .gap_pix    (gap_pix_stable),
    .valid      (stable_valid),
    .calib_coef (calib_coef_pclk),
    .gap_mm_x10 (gap_mm_x10),
    .mm_valid   (mm_valid)
);

// --- frame_sync 时锁存 (pclk 域, 供 50MHz 域安全采样) ---
//   gap_left/gap_right 在 frame_sync 拍是上一帧值, 锁存后与发送的图像同帧
reg [6:0]  gap_left_lat;
reg [6:0]  gap_right_lat;
reg        frame_gap_valid_lat;
always @(posedge cam_pclk or negedge pclk_rst_n) begin
    if (!pclk_rst_n) begin
        gap_left_lat       <= 7'h7F;
        gap_right_lat      <= 7'h7F;
        frame_gap_valid_lat<= 1'b0;
    end else if (frame_sync) begin
        gap_left_lat       <= gap_left;
        gap_right_lat      <= gap_right;
        frame_gap_valid_lat<= frame_gap_valid;
    end
end

//======================================================================
// UART 2Mbps 收发 (sys_clk 域, 摄像头通信)
//   RX: 接收 PC 指令 (阈值/标定/检测行)
//   TX: 发送降采样图像 + 测量结果 (3088 字节/帧)
//======================================================================
uart_rx_2M u_uart_rx_2M (
    .clk       (sys_clk),
    .rst_n     (sys_rst_n),
    .rxd       (uart_rxd),
    .cmd       (cmd),
    .cmd_data  (cmd_data),
    .cmd_valid (cmd_valid)
);

//======================================================================
// 采集链诊断计数器 (pclk 域) — 联调用
//   现象: SCCB 配置完成 (cfg_done=1) 但图像全 0。
//   作用: 每帧统计 cam_href 行数 / 最长行 PCLK 数 / 降采样样本数,
//         走 status bit5 诊断通道 (Qt 已支持解析), 定位是
//         "href 没来" 还是 "href 来了但数据线没信号"。
//======================================================================
reg  [15:0] diag_href_lines;      // 本帧 HREF 上升沿数 (进行中)
reg  [15:0] diag_href_lines_lat;  // 上一帧 HREF 行数 (frame_sync 锁存)
reg  [15:0] diag_ds_count;        // 本帧降采样样本数 (进行中)
reg  [15:0] diag_ds_count_lat;    // 上一帧样本数
reg  [15:0] diag_pclk_per_line;   // 上一帧最长行 PCLK 数 (>>6, 单位=64 PCLK)
reg  [19:0] line_pclk_cnt;        // 当前行 PCLK 计数 (20位防饱和)
reg  [19:0] max_line_pclk;        // 本帧最长行
reg         diag_href_d1;

always @(posedge cam_pclk or negedge pclk_rst_n) begin
    if (!pclk_rst_n) begin
        diag_href_lines     <= 16'd0;
        diag_href_lines_lat <= 16'd0;
        diag_ds_count       <= 16'd0;
        diag_ds_count_lat   <= 16'd0;
        diag_pclk_per_line  <= 16'd0;
        line_pclk_cnt       <= 20'd0;
        max_line_pclk       <= 20'd0;
        diag_href_d1        <= 1'b0;
    end else begin
        diag_href_d1 <= cam_href;

        // 帧起始 (VSYNC 上升沿): 锁存上一帧统计, 清零本帧计数
        if (frame_sync) begin
            diag_href_lines_lat <= diag_href_lines;
            diag_ds_count_lat   <= diag_ds_count;
            diag_pclk_per_line  <= max_line_pclk >> 6;   // 单位=64 PCLK, 防饱和
            diag_href_lines     <= 16'd0;
            diag_ds_count       <= 16'd0;
            max_line_pclk       <= 16'd0;
        end else begin
            // HREF 上升沿: 行数 +1, 记录该行长度
            if (cam_href && !diag_href_d1) begin
                diag_href_lines <= diag_href_lines + 16'd1;
                if (line_pclk_cnt > max_line_pclk)
                    max_line_pclk <= line_pclk_cnt;
                line_pclk_cnt <= 16'd0;
            end
            if (cam_href)
                line_pclk_cnt <= line_pclk_cnt + 16'd1;
            // 降采样样本数
            if (ds_y_valid)
                diag_ds_count <= diag_ds_count + 16'd1;
        end
    end
end

uart_tx_2M u_uart_tx_2M (
    .clk          (sys_clk),
    .rst_n        (sys_rst_n),
    .pclk         (cam_pclk),
    .ds_y_data    (ds_y_data),
    .ds_y_valid   (ds_y_valid),
    .frame_sync   (frame_sync),
    .capture_en   (capture_en_reg),            // 0x04 命令控制
    // 测量结果 (pclk 域, frame_sync 锁存后稳定, 50MHz 域 frame_ready 时采样)
    .gap_pix      ({6'b0, gap_pix_stable}),   // 10→16 bit 零扩展
    .gap_valid    (frame_gap_valid_lat),       // 帧级有效标志
    .stable_valid (stable_valid),
    .gap_mm_x10   (gap_mm_x10),
    .detect_row   (detect_row_reg),            // sys_clk 域寄存器
    .gap_left     (gap_left_lat),              // 64-wide
    .gap_right    (gap_right_lat),
    .config_done  (config_done),               // OV5640 SCCB 配置完成 → status bit6
    // 采集链诊断 (DIAG_MODE=1 时结果字节携带, 见 uart_tx_2M.v)
    .diag_href_lines   (diag_href_lines_lat),
    .diag_pclk_per_line(diag_pclk_per_line),
    .diag_ds_count     (diag_ds_count_lat),
    .cfg_progress      (cfg_progress[7:0]),   // 配置进度 → 诊断字节4
    .txd          (uart_txd)
);

endmodule