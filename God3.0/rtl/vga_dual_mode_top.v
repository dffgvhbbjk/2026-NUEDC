//======================================================================
// Module: vga_dual_mode_top
// Function: VGA波形显示顶层 — ADC → I2S → 波形 → 桩长
// Date:    2026-07-16
//
// 按键:
//   sys_rst_n  (PIN_25) 系统复位, 低有效, 开发板板载按键
//   freeze_btn (PIN_52) 强制重新武装+清屏, 外接按键, 低有效, 内置20ms消抖
//   gap_btn_53 (PIN_53) 黑色物块间距预设 12mm
//   gap_btn_54 (PIN_54) 黑色物块间距预设 26mm
//   btn_55     (PIN_55) 采集长度切换: 128 → 256 → 512 → 128
//   gap_btn_58 (PIN_58) 黑色物块间距预设 48mm
//   gap_btn_59 (PIN_59) 黑色物块间距预设 72mm
//
// 蜂鸣器:
//   buzzer (PIN_1) 高电平触发, 直连蜂鸣器模块I/O口
//
// 时钟架构:
//   50MHz → PLL → 40MHz(VGA) + 24.576MHz(ADC MCLK) + 6.144MHz(BCK)
//======================================================================

module vga_dual_mode_top (
    input  wire         sys_clk,        // 50MHz 系统时钟
    input  wire         sys_rst_n,      // 系统复位, 低有效 (PIN_25, 板载)

    // 按键
    input  wire         freeze_btn,     // 波形冻结/解冻 (PIN_52, 外接)
    input  wire         gap_btn_53,     // 黑色物块间距预设 12mm (PIN_53)
    input  wire         gap_btn_54,     // 黑色物块间距预设 26mm (PIN_54)
    input  wire         btn_55,         // 采集长度切换 (PIN_55)
    input  wire         gap_btn_58,     // 黑色物块间距预设 48mm (PIN_58)
    input  wire         gap_btn_59,     // 黑色物块间距预设 72mm (PIN_59)

    // 蜂鸣器
    output wire         buzzer,         // 蜂鸣器, 高电平触发 (PIN_1)

    // ADC接口
    output wire         adc_mclk,       // SCKI: 24.545MHz
    output wire         adc_bclk,       // BCK:  6.136MHz
    output wire         adc_lrclk,      // LRCK: 95.88kHz
    input  wire         adc_data,       // DOUT: ADC数据输入
    output wire         dac_data,       // DIN: PCM5102实时回放输入 (PIN_121)

    // VGA接口
    output wire         hsync,
    output wire         vsync,
    output wire [11:0]  rgb,

    // UART (vga_clk 域, 2Mbps): 采集波形自动导出
    input  wire         uart_rxd,       // PC → FPGA (PIN_143, 预留未用)
    output wire         uart_txd        // FPGA → PC (PIN_144)
);

//======================================================================
// PLL — 统一时钟生成
//   c0: 40MHz      VGA像素时钟 (800×600@60Hz)
//   c1: 24.545MHz  ADC主时钟 SCKI (256fs @96kHz)
//   c2: 6.136MHz   ADC位时钟 BCK (64fs @96kHz)
//   c3: 24MHz      摄像头 XCLK (未使用)
//======================================================================
wire        vga_clk;
wire        clk_24m;        // PLL c3 输出 24MHz (备用, 未使用)
wire        pll_locked;

clk_gen u_clk_gen (
    .areset (~sys_rst_n),
    .inclk0 (sys_clk),
    .c0     (vga_clk),      // 40MHz
    .c1     (adc_mclk),     // 24.576MHz
    .c2     (adc_bclk),     // 6.144MHz
    .c3     (clk_24m),      // 24MHz (未使用)
    .locked (pll_locked)
);

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
// 锤击触发采集链路 (文档 §10~§13 第四~七阶段)
//   bypass_data/bypass_valid (原始ADC, vga_clk 域)
//     → wave_trigger  基线扣除 + 自适应阈值触发
//     → wave_capture  IDLE→ARMED→CAPTURE→HOLD, 按 sample_valid 计数
//                     采集 64/128/256 点 (替代旧版固定 10ms VGA 定时)
//     → wave_buffer_dp 512×16 双口 RAM (乒乓 bank)
//     → vga_waveform_precision 整数像素/点映射显示
//
// 按键分配:
//   freeze_btn (PIN_52): 强制重新武装 + 清屏
//   btn_55     (PIN_55): 采集长度切换 128 → 256 → 512 → 128
//   gap_btn_53/54/58/59: 选择 12/26/48/72mm 摄像头间距预设
//======================================================================
// 注: bypass_data/bypass_valid 由后部 fifo_sample_reader 模块输出驱动
wire [23:0] bypass_data;
wire        bypass_valid;

// ---- 按键消抖 (20ms, 按下产生单周期脉冲) ----
wire freeze_trigger;   // 强制重新武装
wire len_key_pulse;     // 采集长度切换
wire gap_53_pulse;
wire gap_54_pulse;
wire gap_58_pulse;
wire gap_59_pulse;

key_debounce u_key_freeze (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .key_in      (freeze_btn),
    .press_pulse (freeze_trigger)
);

key_debounce u_key_length (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .key_in      (btn_55),
    .press_pulse (len_key_pulse)
);

key_debounce u_key_gap_53 (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .key_in      (gap_btn_53),
    .press_pulse (gap_53_pulse)
);

key_debounce u_key_gap_54 (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .key_in      (gap_btn_54),
    .press_pulse (gap_54_pulse)
);

key_debounce u_key_gap_58 (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .key_in      (gap_btn_58),
    .press_pulse (gap_58_pulse)
);

key_debounce u_key_gap_59 (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .key_in      (gap_btn_59),
    .press_pulse (gap_59_pulse)
);

// ---- 采集长度配置 ----
// 编码沿用现有采集/显示/UART模块: 01=128, 10=256, 11=512。
// 复位默认 128 点, PIN_55 每次按下循环到下一档。
reg [1:0] len_sel;
always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n)
        len_sel <= 2'b01;              // 默认 128 点
    else if (len_key_pulse) begin
        case (len_sel)
            2'b01:   len_sel <= 2'b10;  // 128 → 256
            2'b10:   len_sel <= 2'b11;  // 256 → 512
            default: len_sel <= 2'b01;  // 512/非法值 → 128
        endcase
    end
end

// ---- 摄像头黑色物块间距预设 ----
// 仅用于 VGA 演示显示，不接入 ADC 波形或缺陷计算链路。
reg [11:0] gap_mm_sel;
always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n)
        gap_mm_sel <= 12'd0;
    else if (freeze_trigger)
        gap_mm_sel <= 12'd0;
    else if (gap_53_pulse)
        gap_mm_sel <= 12'd12;
    else if (gap_54_pulse)
        gap_mm_sel <= 12'd26;
    else if (gap_58_pulse)
        gap_mm_sel <= 12'd48;
    else if (gap_59_pulse)
        gap_mm_sel <= 12'd72;
end

// ---- 显示增益固定为 ×1 ----
wire        gain_is_auto = 1'b0;
wire [2:0]  auto_gain;                     // wave_capture 输出（仅保留采集统计）
wire [2:0]  gain_shift = 3'd0;              // 固定 ×1

// Keep the original 512-point capture/calculation frame.
// PIN_55 changes only the number of points rendered by VGA.
wire [1:0] capture_len_sel = 2'b11;

// ---- 平滑固定旁路，保持原始波形和缺陷计算链路 ----
wire smooth_on = 1'b0;

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

// --- LRCK生成 (BCK / 64 = 6.136MHz / 64 = 95.88kHz) ---
// 文档 §7.2: LRCLK 在 negedge bclk 翻转, 满足 PCM1808 从机建立时间 50ns
//   (半周期 ~81.5ns > 50ns)
//   FPGA 仍在 posedge bclk 采样 DOUT
reg [5:0] lrclk_div;
reg       lrclk_reg;

// 按可用旧项目的音频时序，LRCK 在 BCLK 上升沿翻转。
// iis_dac_out 仍在 BCLK 下降沿更新 DIN，PCM5102 在上升沿采样。
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

// --- I2S从机接收 (文档 §8: pcm1808_i2s_rx, 消除重复 valid) ---
wire [23:0] adc_l_data;
wire [23:0] adc_r_data;
wire        adc_l_valid;
wire        adc_r_valid;
wire        adc_frame_error;       // I2S 帧错误标志 (SignalTap 可抓)

pcm1808_i2s_rx #(
    .DATA_WIDTH (24),
    .I2S_MODE   (1)       // FMT=0: 标准 I2S, 1 位延迟
) u_iis_rx (
    .rst_n       (bclk_rst_n),
    .bclk        (adc_bclk),
    .lrclk       (adc_lrclk),
    .sdin        (adc_data),
    .left_data   (adc_l_data),
    .right_data  (adc_r_data),
    .left_valid  (adc_l_valid),
    .right_valid (adc_r_valid),
    .frame_error (adc_frame_error)
);

// --- 实时音频回放: PCM1808右声道传感器数据 → PCM5102 ---
// 与参考工程相同，使用 BCLK 域内部 LRCK，DIN 在 BCK 下降沿更新。
iis_dac_out u_iis_dac_out (
    .ad_bck       (adc_bclk),
    .rst_n        (bclk_rst_n),
    .lrck         (lrclk_reg),
    .sample_in    (adc_r_data),
    .sample_valid (adc_r_valid),
    .dac_data     (dac_data)
);

// --- 波形 FIFO + 读对齐 (文档 §9: BCLK域 → VGA域 CDC) ---
//   fifo_sample_reader 内部封装 DCFIFO + 读端 q/valid 对齐逻辑
//   消除旧版 empty 下降沿丢点问题 (文档 §3.3)
//   右声道 → 波形链路 (传感器通道)
//   [P1] 系统复位时内部 aclr 清空 FIFO 残留数据 (读写两域同步释放)
//   [P2] 写请求内部带 !wrfull 保护, 满时丢弃并由 overflow 计数记录
// Waveform path stays on the PCM1808 left-channel sample stream.
// The right-channel stream is reserved for PCM5102 playback below.
wire        wave_fifo_full;
wire [7:0]  wave_fifo_level;
wire [7:0]  wave_fifo_ovf;      // FIFO 写满丢弃计数, 正常应恒为 0 (SignalTap 可抓)

fifo_sample_reader #(
    .DATA_WIDTH (24),
    .ADDR_WIDTH (8)
) u_wave_fifo (
    .rdclk        (vga_clk),
    .rst_n        (vga_rst_n),
    .wr_rst_n     (bclk_rst_n),      // 写域复位: 任一复位 → FIFO aclr 清空
    .rd_en        (1'b1),            // 常开: 有数据即自动读
    .wr_data      (adc_l_data),      // 左声道 → 波形链路
    .wrreq        (adc_l_valid),     // 内部已做 !wrfull 保护
    .wrclk        (adc_bclk),
    .sample_data  (bypass_data),     // 与 sample_valid 对齐
    .sample_valid (bypass_valid),
    .fifo_full    (wave_fifo_full),
    .fifo_level   (wave_fifo_level),
    .overflow_cnt (wave_fifo_ovf)
);

// (FIR 滤波通路已移除)
// (桩长计算已移除)
// (数码管已移除)

//======================================================================
// 第四阶段: 基线估计 + 自适应阈值触发 (文档 §10)
//======================================================================
wire signed [23:0] corrected_data;
wire signed [23:0] trig_raw_data;    // 原始样点随动 (与 corrected_data 同拍)
wire               corrected_valid;
wire               wave_trig;
wire               wave_rearm_ok;
wire               trig_base_en;
wire               trig_track_en;
wire               trig_arm_en;
wire               capture_rearm_pulse;
wire               trigger_clear = freeze_trigger | capture_rearm_pulse;
wire [23:0]        dbg_threshold;   // SignalTap 观测
wire [23:0]        dbg_noise_ema;   // SignalTap 观测 (冻结版 §4.2 噪声慢均)
wire [23:0]        dbg_magnitude;   // SignalTap 观测
wire [23:0]        dbg_baseline;    // SignalTap 观测

wave_trigger u_wave_trigger (
    .clk             (vga_clk),
    .rst_n           (vga_rst_n),
    .sample_data     (bypass_data),
    .sample_valid    (bypass_valid),
    .base_en         (trig_base_en),
    .track_en        (trig_track_en),
    .arm_en          (trig_arm_en),
    .clear_history   (trigger_clear),
    .corrected_data  (corrected_data),
    .raw_data        (trig_raw_data),
    .corrected_valid (corrected_valid),
    .trigger         (wave_trig),
    .rearm_ok        (wave_rearm_ok),
    .dbg_threshold   (dbg_threshold),
    .dbg_noise_ema   (dbg_noise_ema),
    .dbg_magnitude   (dbg_magnitude),
    .dbg_baseline    (dbg_baseline)
);

//======================================================================
// 第八阶段: 可旁路三点平滑 (文档 §14, P2)
//   y[n] = (x[n-1] + 2x[n] + x[n+1])/4, 两种模式恒定 1 样点延迟,
//   trigger 随样点同步延迟 → 切换模式不影响触发点对齐
//======================================================================
wire signed [23:0] smooth_data;
wire signed [23:0] smooth_raw;       // 原始样点随动 (仅延迟, 永不平滑)
wire               smooth_valid;
wire               smooth_trig;

wave_smooth u_wave_smooth (
    .clk         (vga_clk),
    .rst_n       (vga_rst_n),
    .bypass      (~smooth_on),
    .in_data     (corrected_data),
    .in_raw      (trig_raw_data),
    .in_valid    (corrected_valid),
    .in_trigger  (wave_trig),
    .out_data    (smooth_data),
    .out_raw     (smooth_raw),
    .out_valid   (smooth_valid),
    .out_trigger (smooth_trig)
);

//======================================================================
// 第五阶段: 64/128/256/512 点采集状态机 (文档 §11)
//   采集结束由 sample_valid 计数决定, 不再使用固定 10ms VGA 定时
//======================================================================
wire               buf_wr_en;
wire [9:0]         buf_wr_addr;
wire signed [15:0] buf_wr_data;
wire               wave_display_en;
wire [9:0]         wave_disp_origin;
wire [1:0]         wave_disp_len;
wire               wave_cap_done;   // 采集完成脉冲 (UART 导出)
wire [1:0]         dbg_cap_state;   // SignalTap 观测

wave_capture u_wave_capture (
    .clk             (vga_clk),
    .rst_n           (vga_rst_n),
    .corrected_data  (smooth_data),
    .corrected_valid (smooth_valid),
    .trigger         (smooth_trig),
    .rearm_ok        (wave_rearm_ok),
    .len_sel         (capture_len_sel),
    .force_rearm     (freeze_trigger),
    .base_en         (trig_base_en),
    .track_en        (trig_track_en),
    .arm_en          (trig_arm_en),
    .wr_en           (buf_wr_en),
    .wr_addr         (buf_wr_addr),
    .wr_data         (buf_wr_data),
    .display_en      (wave_display_en),
    .disp_origin     (wave_disp_origin),
    .disp_len_sel    (wave_disp_len),
    .auto_gain       (auto_gain),
    .cap_done        (wave_cap_done),
    .rearm_pulse     (capture_rearm_pulse),
    .dbg_state       (dbg_cap_state)
);

//======================================================================
// 第六阶段: 16 位 × 1024 双口波形 RAM (文档 §12)
//======================================================================
wire [9:0]         buf_rd_addr;
wire signed [15:0] buf_rd_data;

wave_buffer_dp u_wave_buf (
    .clk     (vga_clk),
    .wr_en   (buf_wr_en),
    .wr_addr (buf_wr_addr),
    .wr_data (buf_wr_data),
    .rd_addr (buf_rd_addr),
    .rd_data (buf_rd_data)
);

//======================================================================
// 第七阶段: 高精度波形显示 (文档 §13)
//   整数像素/点映射 (64点=8px, 128点=4px, 256点=2px), 不丢点
//======================================================================
wire [11:0] osc_rgb;

vga_waveform_precision u_waveform (
    .vga_clk      (vga_clk),
    .rst_n        (vga_rst_n),
    .rd_addr      (buf_rd_addr),
    .rd_data      (buf_rd_data),
    .display_en   (wave_display_en),
    .disp_origin  (wave_disp_origin),
    .disp_len_sel (len_sel),
    .len_sel_cfg  (len_sel),
    .gain_shift   (gain_shift),
    .gain_is_auto (gain_is_auto),
    .pix_x        (vga_pix_x),
    .pix_y        (vga_pix_y),
    .rgb_valid    (vga_rgb_valid),
    .rgb          (osc_rgb)
);

//======================================================================

// (rgb 输出见后部 vga_status_overlay: 波形层 + 缺陷检测状态叠加层)

//======================================================================
// P3: 冻结版缺陷检测 (doc/比赛前三天冻结版检测方案.md §5/§7)
//   defect_analyzer 监听平滑前的 corrected_data/corrected_valid/wave_trig，
//   使 PIN_59 只改变显示波形，不再改变分类结果；cap_done 时输出三态判定;
//   rearm 接 capture_rearm_pulse: 强制重武装/重新武装时中止未完成分析
//   (freeze_trigger 经 wave_capture.force_rearm 也会发出该脉冲)。
//
//   赛前固化参数 (§1: 现场只测量、不学习、不更新模板):
//     D_REF     模板正常棒底间隔 (采样点), 赛前离线数据生成后改
//     L_REF_MM  自有棒卷尺实测长度 (模式B退化比例用)
//     L_TEST_MM 现场棒标称长度 (模式A棒底归一化用)
//   运行期间三者只读, 原 1/4 IIR 运行时标定已按方案删除。
//
//   DEFECT 时启动 defect_distance_calc 双模式除法 (22 拍):
//     有棒底: delta×L_TEST_MM/bottom_delta (高置信)
//     无棒底: delta×L_REF_MM/D_REF (低置信标志)
//======================================================================
localparam [8:0]  D_REF     = 9'd105;   // 棒底间隔中位 (触发点基准, 250帧实测)
localparam [11:0] L_REF_MM  = 12'd1000; // 自有棒实测长度 mm (待实测更新)
localparam [11:0] L_TEST_MM = 12'd1000; // 现场棒标称长度 mm

// 三态结果编码 (与 defect_analyzer/overlay/UART 保持一致)
localparam [1:0] RES_INVALID = 2'd0;
localparam [1:0] RES_NORMAL  = 2'd1;
localparam [1:0] RES_DEFECT  = 2'd2;

// [已移除] 旧 defect_analyzer (包络波包法) —— 释放 ~2278 LE 以适配 EP4CE6。
//   其分类职责已被 defect_qiudao_classifier 全面取代; 唯一残余用途是 VGA 波形
//   红(缺陷反射)/紫(棒底)标记线索引, 但新算法基于自相关频率指纹, 无对应
//   波形反射采样点, 故红线索引不再有意义 (overlay 对 defect_index==0 不绘制),
//   紫线退化为 D_REF 固化预测。主判定/距离/黄色主冲击线均来自新分类器。

//----------------------------------------------------------------------
// 缺陷分类器 (defect_qiudao_classifier): 5 点递减 + 方向翻转计数
//   算法来源: jisuan.md / daima.txt (原 ADC_HUANCUN 模块)
//     - 归一化: data/128 + 200
//     - qiudao_panjue = 5 点严格递减
//     - 窗口 [81,412] 内累计翻转次数 win_edge_cnt
//     - win_edge_cnt >= 11 (可参数化) → DEFECT, 否则 → NORMAL
//     - 第 4~6 翻转区域下降段宽度 × 15 → distance_mm
//   资源 ~300 LE + 1 M9K (vs 旧 LDA 分类器 ~2500 LE + 30 M9K)
//   处理延迟 ~38μs @40MHz (vs 旧 LDA ~1.15ms)
//----------------------------------------------------------------------
wire        ac_result_valid;
wire [1:0]  ac_result_state;
wire [1:0]  ac_confidence;
wire [2:0]  ac_defect_class;
wire [11:0] ac_distance_mm;
wire [8:0]  ac_impact_index;
wire        ac_busy;
wire [3:0]  dbg_ac_state;

defect_qiudao_classifier u_defect_ac (
    .clk          (vga_clk),
    .rst_n        (vga_rst_n),
    .wr_en        (buf_wr_en),
    .wr_addr      (buf_wr_addr),
    .wr_raw       (smooth_raw),
    .cap_done     (wave_cap_done),
    .disp_origin  (wave_disp_origin),
    .disp_len_sel (wave_disp_len),
    .rearm        (capture_rearm_pulse),
    .result_valid (ac_result_valid),
    .result_state (ac_result_state),
    .confidence   (ac_confidence),
    .defect_class (ac_defect_class),
    .distance_mm  (ac_distance_mm),
    .impact_index (ac_impact_index),
    .busy         (ac_busy),
    .dbg_state    (dbg_ac_state)
);

//======================================================================
// 蜂鸣器提示音
//   NORMAL: 一声; DEFECT: 两声; INVALID/RETRY: 不响。
//   不再使用手动测试键，四个间距按键只更新 VGA 预设值。
//======================================================================
buzzer_beep u_buzzer_beep (
    .clk          (vga_clk),
    .rst_n        (vga_rst_n),
    .btn          (1'b1),
    .normal_pulse (ac_result_valid && (ac_result_state == RES_NORMAL)),
    .defect_pulse (ac_result_valid && (ac_result_state == RES_DEFECT)),
    .buzzer       (buzzer)
);

// ---- 结果保持寄存器 (§6: 每次敲击独立计分, 主结果始终对应当前这一击) ----
//   三态保持到下一次敲击输出; INVALID 也如实保持 (VGA 显示 RETRY),
//   不清除波形 (波形 RAM 与显示由采集链独立管理, 此处不干预)
reg         defect_result_ready;   // 已有至少一次分析结果
reg  [1:0]  result_state_hold;     // 0=INVALID 1=NORMAL 2=DEFECT
reg  [1:0]  confidence_hold;       // 0=无 1=低(无棒底) 2=高(有棒底)
reg         bottom_found_hold;
reg  [11:0] defect_dist_hold;
reg  [8:0]  defect_delta_hold;
reg  [8:0]  impact_index_hold;     // 主冲击峰索引 (VGA 标记线)
reg  [8:0]  defect_index_hold;     // 缺陷反射峰索引 (VGA 标记线)
reg  [8:0]  bottom_index_hold;     // 棒底反射峰索引 (VGA 紫线)
reg  [8:0]  bottom_delta_hold;     // 本次棒底间隔 (紫线退化用 D_REF)

always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n) begin
        defect_result_ready <= 1'b0;
        result_state_hold   <= RES_INVALID;
        confidence_hold     <= 2'd0;
        bottom_found_hold   <= 1'b0;
        defect_delta_hold   <= 9'd0;
        defect_dist_hold    <= 12'd0;
        impact_index_hold   <= 9'd0;
        defect_index_hold   <= 9'd0;
        bottom_index_hold   <= 9'd0;
        bottom_delta_hold   <= 9'd0;
    end else begin
        // ---- 分类结果: defect_qiudao_classifier (5点递减+翻转计数) ----
        //   defect_analyzer 已移除: 标记线索引 (defect_index/bottom_index 等)
        //   保持复位值 0, 不再更新 (overlay 据此不绘制红线, 紫线用 D_REF)。
        if (ac_result_valid) begin
            defect_result_ready <= 1'b1;
            result_state_hold   <= ac_result_state;
            confidence_hold     <= ac_confidence;
            impact_index_hold   <= ac_impact_index;
            defect_dist_hold    <= ac_distance_mm;  // NORMAL/INVALID 时模块已输出 0
        end
    end
end

// ---- 紫线间隔 (§8): 找到本次棒底显示本次 Tend, 否则显示固化 D_REF 预测 ----
wire [8:0] bottom_mark_delta = bottom_found_hold ? bottom_delta_hold : D_REF;

// ---- 触发阈值锁存 (UART 帧尾字段, 采集完成拍冻结) ----
reg [23:0] trig_thr_hold;
always @(posedge vga_clk or negedge vga_rst_n) begin
    if (!vga_rst_n)
        trig_thr_hold <= 24'd0;
    else if (wave_cap_done)
        trig_thr_hold <= dbg_threshold;
end

//======================================================================
// P3: VGA 状态叠加层 (冻结版方案 §8, vga_status_overlay)
//   叠加在 osc_rgb 之后: 左侧系统状态 / 右侧检测结果 / 底部状态栏 /
//   波形区标记线 (黄=主冲击, 红=缺陷位置, 紫=本次棒底或 D_REF 预测)
//   三态结果: NORMAL/DEFECT/RETRY(INVALID 黄色), 另显 BOTTOM/CONF
//   内部 3 拍流水与 vga_waveform_precision 延迟一致, 标记线像素对齐
//======================================================================
vga_status_overlay u_status_overlay (
    .clk           (vga_clk),
    .rst_n         (vga_rst_n),
    .pix_x         (vga_pix_x),
    .pix_y         (vga_pix_y),
    .rgb_valid     (vga_rgb_valid),
    .wave_rgb      (osc_rgb),
    .cap_state     (dbg_cap_state),
    .display_en    (wave_display_en),
    .disp_len_sel  (len_sel),
    .len_sel_cfg   (len_sel),
    .smooth_on     (smooth_on),
    .gap_mm        (gap_mm_sel),
    .result_ready  (defect_result_ready),
    .result_state  (result_state_hold),
    .confidence    (confidence_hold),
    .bottom_found  (bottom_found_hold),
    .defect_delta  (defect_delta_hold),
    .distance_mm   (defect_dist_hold),
    .impact_index  (impact_index_hold),
    .defect_index  (defect_index_hold),
    .bottom_delta  (bottom_mark_delta),
    .rgb_out       (rgb)
);

//======================================================================
// P2: 采集波形 UART 导出 (115200 baud, 8N1, vga_clk 域)
//   采集完成 (cap_done) 自动导出一帧原始 24 位样点:
//   AA 55 len_sel 样点([23:16][15:8][7:0])×N 校验和
//   + 版本化结果字段帧尾 (冻结版 §12.7): 5A ver result_state confidence
//     impact_index defect_index bottom_index distance trigger_threshold rsum
//   影子 RAM 与显示 RAM 同址同拍写入, 但存的是未经基线扣除/平滑的
//   原始数据 (smooth_raw 随动通道), 供 PC 端模板制作/标定使用
//   (替换原 uart_fifo_loopback 回环测试; uart_rxd 暂未使用, 保留引脚)
//======================================================================
wire uart_export_busy;   // SignalTap 可观测

wave_uart_export u_wave_export (
    .clk               (vga_clk),
    .rst_n             (vga_rst_n),
    .wr_en             (buf_wr_en),
    .wr_addr           (buf_wr_addr),
    .wr_raw            (smooth_raw),
    .cap_done          (wave_cap_done),
    .disp_origin       (wave_disp_origin),
    .disp_len_sel      (wave_disp_len),
    .result_ready      (defect_result_ready),
    .result_state      (result_state_hold),
    .confidence        (confidence_hold),
    .impact_index      (impact_index_hold),
    .defect_index      (defect_index_hold),
    .bottom_index      (bottom_index_hold),
    .distance_mm       (defect_dist_hold),
    .trigger_threshold (trig_thr_hold),
    .busy              (uart_export_busy),
    .uart_txd          (uart_txd)
);

//======================================================================
// SignalTap 调试探针 (noprune): 诊断信号仅供在线抓取, 无下游逻辑,
//   不加保持属性会被综合整体剪除 (frame_error/阈值/基线/采集状态等)。
//   汇聚为一组观测寄存器: SignalTap 直接添加 dbg_probe 即可。
//   注: frame_error/fifo_full/fifo_ovf 为 bclk 域信号, 此处仅打拍观测
//   (趋势/计数用途), 多位值瞬时可能不一致, 不回馈任何控制逻辑。
//======================================================================
(* noprune *) reg [79:0] dbg_probe;

always @(posedge vga_clk) begin
    dbg_probe <= { defect_result_ready, // [79]    已有检测结果
                   result_state_hold,   // [78:77] 三态结果 (保持)
                   confidence_hold,     // [76:75] 置信度 (保持)
                   bottom_found_hold,   // [74]    本次找到棒底 (保持)
                   dbg_ac_state[2:0],   // [73:71] 新分类器状态机 (低3位)
                   adc_frame_error,     // [70]    I2S 帧错误 (bclk 域)
                   wave_fifo_full,      // [69]    FIFO 写满 (bclk 域)
                   wave_fifo_ovf,       // [68:61] FIFO 溢出丢弃计数
                   wave_fifo_level,     // [60:53] FIFO 水位
                   dbg_threshold,       // [52:29] 自适应触发阈值
                   dbg_noise_ema,       // [28:5]  噪声慢速平均
                   dbg_cap_state,       // [4:3]   采集状态机
                   wave_trig,           // [2]     触发脉冲
                   wave_rearm_ok,       // [1]     静默达标
                   wave_cap_done };     // [0]     采集完成脉冲
end

endmodule
