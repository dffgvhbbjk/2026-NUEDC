// ============================================================================
// Module: vga_status_overlay
// Description: 缺陷检测状态/结果叠加层 (冻结版方案 §8)
//
//   叠加在 vga_waveform_precision 输出 (wave_rgb) 之后, 不改动已验证的
//   波形读取/缩放/绘制逻辑:
//   - 左侧状态栏 (x=16~127):  SYSTEM 状态 / TRIG / LEN / FILT
//   - 右侧结果栏 (x=672~783): RESULT NORMAL/DEFECT/RETRY / DIST / PEAK /
//                             BTM (棒底间隔) / CONF (置信度)
//   - 底部状态栏 (y=584~591): AUTO MODE / Fs:95.88k / UART:115200 / 状态
//   - 顶部标题   (y=8~15):    DEFECT CHECK
//   - 波形区标记线: 黄=本次主冲击位置(距离起点，短线),
//     红=检测到的缺陷反射位置(整高), 紫=本次棒底或固化 D_REF 预测(虚线);
//     display_index = 预触发点数 + 检测索引,
//     x = 144 + display_index × (8/4/2/1 px/点), 越界不绘制 (防回绕)
//
//   三态结果 (§5.5): NORMAL 绿 / DEFECT 红 / INVALID 显示黄色 RETRY
//   (提示重新敲击, 不计好坏); INVALID 时不绘制标记线, 波形保留。
//
//   流水线: 3 拍 (选字 → 字模行 → 像素判定), 与 vga_waveform_precision
//   的 3 拍延迟严格对齐, 标记线与波形像素一一对应
//
//   字符网格: 8×8 字模, 8px 间距, 起始 x 为 8 的倍数 → 列号 = pix_x[9:3],
//   行内行号 = pix_y[2:0] (所有文字行 y 均 8 对齐)
//
//   距离/PEAK 数值: 自由运行 double-dabble 二进制转 BCD (12/14 拍一轮,
//   无除法器), 结果变化后约 0.35us 内刷新, 人眼不可见
//
// 时钟域: vga_clk 40MHz
// ============================================================================
module vga_status_overlay (
    input  wire        clk,             // vga_clk 40MHz
    input  wire        rst_n,

    // VGA 时序 (与 vga_waveform_precision 同源)
    input  wire [9:0]  pix_x,
    input  wire [9:0]  pix_y,
    input  wire        rgb_valid,
    input  wire [11:0] wave_rgb,        // 波形层输出 (3 拍延迟)

    // 采集/显示状态
    input  wire [1:0]  cap_state,       // 0=IDLE(预热) 1=ARMED 2=CAPTURE 3=HOLD
    input  wire        display_en,      // 1=已有波形显示
    input  wire [1:0]  disp_len_sel,    // 已采集长度 (锁存)
    input  wire [1:0]  len_sel_cfg,     // 当前长度配置 (待机显示)
    input  wire        smooth_on,       // 三点平滑开关
    input  wire [11:0] gap_mm,          // 摄像头黑色物块间距预设 mm

    // 缺陷检测结果 (顶层保持寄存器, 稳定电平)
    input  wire        result_ready,    // 已有至少一次分析结果
    input  wire [1:0]  result_state,    // 0=INVALID 1=NORMAL 2=DEFECT
    input  wire [1:0]  confidence,      // 0=无 1=低(无棒底) 2=高(有棒底)
    input  wire        bottom_found,    // 1=本次找到棒底
    input  wire [8:0]  defect_delta,    // 主冲击→反射间隔 (点)
    input  wire [11:0] distance_mm,     // 缺陷距离 mm (正常棒为 0)
    input  wire [8:0]  impact_index,    // 主冲击峰索引 (触发点起)
    input  wire [8:0]  defect_index,    // 缺陷反射峰索引 (触发点起)
    input  wire [8:0]  bottom_delta,    // 紫线间隔: 本次棒底或 D_REF (点)

    output wire [11:0] rgb_out
);

// 三态结果编码 (与 defect_analyzer/顶层一致)
localparam [1:0] RES_INVALID = 2'd0;
localparam [1:0] RES_NORMAL  = 2'd1;
localparam [1:0] RES_DEFECT  = 2'd2;

// 波形区边界 (与 vga_waveform_precision 一致)
localparam X_LEFT = 10'd144;
localparam Y_TOP  = 10'd30;
localparam Y_BTM  = 10'd569;
localparam Y_MID  = 10'd300;

// 文字颜色编码
localparam C_WHITE  = 3'd0;
localparam C_GREEN  = 3'd1;
localparam C_RED    = 3'd2;
localparam C_YELLOW = 3'd3;
localparam C_CYAN   = 3'd4;

//==================== 8x8 字符 ROM ====================
function [7:0] char_8x8;
    input [7:0] ascii;
    input [2:0] row;
    begin
        case (ascii)
            "0": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h6E; 3'd3: char_8x8=8'h76; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "1": case (row) 3'd0: char_8x8=8'h18; 3'd1: char_8x8=8'h38; 3'd2: char_8x8=8'h18; 3'd3: char_8x8=8'h18; 3'd4: char_8x8=8'h18; 3'd5: char_8x8=8'h18; 3'd6: char_8x8=8'h7E; default: char_8x8=0; endcase
            "2": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h06; 3'd3: char_8x8=8'h0C; 3'd4: char_8x8=8'h18; 3'd5: char_8x8=8'h30; 3'd6: char_8x8=8'h7E; default: char_8x8=0; endcase
            "3": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h06; 3'd3: char_8x8=8'h1C; 3'd4: char_8x8=8'h06; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "4": case (row) 3'd0: char_8x8=8'h0C; 3'd1: char_8x8=8'h1C; 3'd2: char_8x8=8'h2C; 3'd3: char_8x8=8'h4C; 3'd4: char_8x8=8'h7E; 3'd5: char_8x8=8'h0C; 3'd6: char_8x8=8'h0C; default: char_8x8=0; endcase
            "5": case (row) 3'd0: char_8x8=8'h7E; 3'd1: char_8x8=8'h60; 3'd2: char_8x8=8'h7C; 3'd3: char_8x8=8'h06; 3'd4: char_8x8=8'h06; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "6": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h60; 3'd3: char_8x8=8'h7C; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "7": case (row) 3'd0: char_8x8=8'h7E; 3'd1: char_8x8=8'h06; 3'd2: char_8x8=8'h0C; 3'd3: char_8x8=8'h18; 3'd4: char_8x8=8'h30; 3'd5: char_8x8=8'h30; 3'd6: char_8x8=8'h30; default: char_8x8=0; endcase
            "8": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h3C; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "9": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h3E; 3'd4: char_8x8=8'h06; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "A": case (row) 3'd0: char_8x8=8'h18; 3'd1: char_8x8=8'h3C; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h66; 3'd4: char_8x8=8'h7E; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h66; default: char_8x8=0; endcase
            "B": case (row) 3'd0: char_8x8=8'h7C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h7C; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h7C; default: char_8x8=0; endcase
            "C": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h60; 3'd3: char_8x8=8'h60; 3'd4: char_8x8=8'h60; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "D": case (row) 3'd0: char_8x8=8'h78; 3'd1: char_8x8=8'h6C; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h66; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h6C; 3'd6: char_8x8=8'h78; default: char_8x8=0; endcase
            "E": case (row) 3'd0: char_8x8=8'h7E; 3'd1: char_8x8=8'h60; 3'd2: char_8x8=8'h60; 3'd3: char_8x8=8'h7C; 3'd4: char_8x8=8'h60; 3'd5: char_8x8=8'h60; 3'd6: char_8x8=8'h7E; default: char_8x8=0; endcase
            "F": case (row) 3'd0: char_8x8=8'h7E; 3'd1: char_8x8=8'h60; 3'd2: char_8x8=8'h60; 3'd3: char_8x8=8'h7C; 3'd4: char_8x8=8'h60; 3'd5: char_8x8=8'h60; 3'd6: char_8x8=8'h60; default: char_8x8=0; endcase
            "G": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h60; 3'd3: char_8x8=8'h6E; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "H": case (row) 3'd0: char_8x8=8'h66; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h7E; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h66; default: char_8x8=0; endcase
            "I": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h18; 3'd2: char_8x8=8'h18; 3'd3: char_8x8=8'h18; 3'd4: char_8x8=8'h18; 3'd5: char_8x8=8'h18; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "K": case (row) 3'd0: char_8x8=8'h66; 3'd1: char_8x8=8'h6C; 3'd2: char_8x8=8'h78; 3'd3: char_8x8=8'h70; 3'd4: char_8x8=8'h78; 3'd5: char_8x8=8'h6C; 3'd6: char_8x8=8'h66; default: char_8x8=0; endcase
            "L": case (row) 3'd0: char_8x8=8'h60; 3'd1: char_8x8=8'h60; 3'd2: char_8x8=8'h60; 3'd3: char_8x8=8'h60; 3'd4: char_8x8=8'h60; 3'd5: char_8x8=8'h60; 3'd6: char_8x8=8'h7E; default: char_8x8=0; endcase
            "M": case (row) 3'd0: char_8x8=8'h63; 3'd1: char_8x8=8'h77; 3'd2: char_8x8=8'h7F; 3'd3: char_8x8=8'h6B; 3'd4: char_8x8=8'h63; 3'd5: char_8x8=8'h63; 3'd6: char_8x8=8'h63; default: char_8x8=0; endcase
            "N": case (row) 3'd0: char_8x8=8'h66; 3'd1: char_8x8=8'h76; 3'd2: char_8x8=8'h7E; 3'd3: char_8x8=8'h7E; 3'd4: char_8x8=8'h6E; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h66; default: char_8x8=0; endcase
            "O": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h66; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "P": case (row) 3'd0: char_8x8=8'h7C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h7C; 3'd4: char_8x8=8'h60; 3'd5: char_8x8=8'h60; 3'd6: char_8x8=8'h60; default: char_8x8=0; endcase
            "R": case (row) 3'd0: char_8x8=8'h7C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h7C; 3'd4: char_8x8=8'h78; 3'd5: char_8x8=8'h6C; 3'd6: char_8x8=8'h66; default: char_8x8=0; endcase
            "S": case (row) 3'd0: char_8x8=8'h3C; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h60; 3'd3: char_8x8=8'h3C; 3'd4: char_8x8=8'h06; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "T": case (row) 3'd0: char_8x8=8'h7E; 3'd1: char_8x8=8'h18; 3'd2: char_8x8=8'h18; 3'd3: char_8x8=8'h18; 3'd4: char_8x8=8'h18; 3'd5: char_8x8=8'h18; 3'd6: char_8x8=8'h18; default: char_8x8=0; endcase
            "U": case (row) 3'd0: char_8x8=8'h66; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h66; 3'd4: char_8x8=8'h66; 3'd5: char_8x8=8'h66; 3'd6: char_8x8=8'h3C; default: char_8x8=0; endcase
            "W": case (row) 3'd0: char_8x8=8'h63; 3'd1: char_8x8=8'h63; 3'd2: char_8x8=8'h63; 3'd3: char_8x8=8'h6B; 3'd4: char_8x8=8'h7F; 3'd5: char_8x8=8'h77; 3'd6: char_8x8=8'h63; default: char_8x8=0; endcase
            "Y": case (row) 3'd0: char_8x8=8'h66; 3'd1: char_8x8=8'h66; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h3C; 3'd4: char_8x8=8'h18; 3'd5: char_8x8=8'h18; 3'd6: char_8x8=8'h18; default: char_8x8=0; endcase
            "k": case (row) 3'd0: char_8x8=8'h60; 3'd1: char_8x8=8'h60; 3'd2: char_8x8=8'h66; 3'd3: char_8x8=8'h6C; 3'd4: char_8x8=8'h78; 3'd5: char_8x8=8'h6C; 3'd6: char_8x8=8'h66; default: char_8x8=0; endcase
            "m": case (row) 3'd2: char_8x8=8'h6C; 3'd3: char_8x8=8'h6C; 3'd4: char_8x8=8'h6C; 3'd5: char_8x8=8'h6C; 3'd6: char_8x8=8'h6C; default: char_8x8=0; endcase
            "s": case (row) 3'd2: char_8x8=8'h3C; 3'd3: char_8x8=8'h60; 3'd4: char_8x8=8'h3C; 3'd5: char_8x8=8'h06; 3'd6: char_8x8=8'h7C; default: char_8x8=0; endcase
            ":": case (row) 3'd2: char_8x8=8'h18; 3'd5: char_8x8=8'h18; default: char_8x8=0; endcase
            ".": case (row) 3'd5: char_8x8=8'h18; 3'd6: char_8x8=8'h18; default: char_8x8=0; endcase
            "-": case (row) 3'd3: char_8x8=8'h7E; default: char_8x8=0; endcase
            default: char_8x8 = 0;     // 空格及未定义字符
        endcase
    end
endfunction

//==================== 数值 → BCD (自由运行 double-dabble) ====================
// 距离 12 位 → 4 BCD 位: 加载 1 拍 + 12 轮 adjust-shift, 循环刷新
reg  [3:0]  dd_cnt;
reg  [27:0] dd_sh;
reg  [15:0] dist_bcd;
wire [3:0]  dd_a3 = (dd_sh[27:24] >= 4'd5) ? dd_sh[27:24] + 4'd3 : dd_sh[27:24];
wire [3:0]  dd_a2 = (dd_sh[23:20] >= 4'd5) ? dd_sh[23:20] + 4'd3 : dd_sh[23:20];
wire [3:0]  dd_a1 = (dd_sh[19:16] >= 4'd5) ? dd_sh[19:16] + 4'd3 : dd_sh[19:16];
wire [3:0]  dd_a0 = (dd_sh[15:12] >= 4'd5) ? dd_sh[15:12] + 4'd3 : dd_sh[15:12];
wire [27:0] dd_adj = {dd_a3, dd_a2, dd_a1, dd_a0, dd_sh[11:0]};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dd_cnt   <= 4'd0;
        dd_sh    <= 28'd0;
        dist_bcd <= 16'd0;
    end else if (dd_cnt == 4'd0) begin
        dist_bcd <= dd_sh[27:12];          // 上一轮结果
        dd_sh    <= {16'd0, distance_mm};  // 加载当前值
        dd_cnt   <= 4'd12;
    end else begin
        dd_sh  <= {dd_adj[26:0], 1'b0};    // adjust 后左移
        dd_cnt <= dd_cnt - 1'b1;
    end
end

// PEAK (defect_delta) 9 位 → 3 BCD 位 (借 12 位转换器结构, 高位恒 0)
reg  [3:0]  pd_cnt;
reg  [27:0] pd_sh;
reg  [11:0] delta_bcd;
wire [3:0]  pd_a2 = (pd_sh[23:20] >= 4'd5) ? pd_sh[23:20] + 4'd3 : pd_sh[23:20];
wire [3:0]  pd_a1 = (pd_sh[19:16] >= 4'd5) ? pd_sh[19:16] + 4'd3 : pd_sh[19:16];
wire [3:0]  pd_a0 = (pd_sh[15:12] >= 4'd5) ? pd_sh[15:12] + 4'd3 : pd_sh[15:12];
wire [27:0] pd_adj = {pd_sh[27:24], pd_a2, pd_a1, pd_a0, pd_sh[11:0]};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pd_cnt    <= 4'd0;
        pd_sh     <= 28'd0;
        delta_bcd <= 12'd0;
    end else if (pd_cnt == 4'd0) begin
        delta_bcd <= pd_sh[23:12];
        pd_sh     <= {16'd0, 3'd0, defect_delta};
        pd_cnt    <= 4'd12;
    end else begin
        pd_sh  <= {pd_adj[26:0], 1'b0};
        pd_cnt <= pd_cnt - 1'b1;
    end
end

// BTM 棒底间隔 (本次或 D_REF) 9 位 → 3 BCD 位
reg  [3:0]  ed_cnt;
reg  [27:0] ed_sh;
reg  [11:0] end_bcd;
wire [3:0]  ed_a2 = (ed_sh[23:20] >= 4'd5) ? ed_sh[23:20] + 4'd3 : ed_sh[23:20];
wire [3:0]  ed_a1 = (ed_sh[19:16] >= 4'd5) ? ed_sh[19:16] + 4'd3 : ed_sh[19:16];
wire [3:0]  ed_a0 = (ed_sh[15:12] >= 4'd5) ? ed_sh[15:12] + 4'd3 : ed_sh[15:12];
wire [27:0] ed_adj = {ed_sh[27:24], ed_a2, ed_a1, ed_a0, ed_sh[11:0]};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ed_cnt  <= 4'd0;
        ed_sh   <= 28'd0;
        end_bcd <= 12'd0;
    end else if (ed_cnt == 4'd0) begin
        end_bcd <= ed_sh[23:12];
        ed_sh   <= {16'd0, 3'd0, bottom_delta};
        ed_cnt  <= 4'd12;
    end else begin
        ed_sh  <= {ed_adj[26:0], 1'b0};
        ed_cnt <= ed_cnt - 1'b1;
    end
end

// 摄像头间距 12 位 → 4 BCD 位
reg  [3:0]  gd_cnt;
reg  [27:0] gd_sh;
reg  [15:0] gap_bcd;
wire [3:0]  gd_a3 = (gd_sh[27:24] >= 4'd5) ? gd_sh[27:24] + 4'd3 : gd_sh[27:24];
wire [3:0]  gd_a2 = (gd_sh[23:20] >= 4'd5) ? gd_sh[23:20] + 4'd3 : gd_sh[23:20];
wire [3:0]  gd_a1 = (gd_sh[19:16] >= 4'd5) ? gd_sh[19:16] + 4'd3 : gd_sh[19:16];
wire [3:0]  gd_a0 = (gd_sh[15:12] >= 4'd5) ? gd_sh[15:12] + 4'd3 : gd_sh[15:12];
wire [27:0] gd_adj = {gd_a3, gd_a2, gd_a1, gd_a0, gd_sh[11:0]};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        gd_cnt  <= 4'd0;
        gd_sh   <= 28'd0;
        gap_bcd <= 16'd0;
    end else if (gd_cnt == 4'd0) begin
        gap_bcd <= gd_sh[27:12];
        gd_sh   <= {16'd0, gap_mm};
        gd_cnt  <= 4'd12;
    end else begin
        gd_sh  <= {gd_adj[26:0], 1'b0};
        gd_cnt <= gd_cnt - 1'b1;
    end
end

// 数字字符 (ASCII '0' + BCD)
wire [7:0] dch3 = 8'h30 + {4'd0, dist_bcd[15:12]};
wire [7:0] dch2 = 8'h30 + {4'd0, dist_bcd[11:8]};
wire [7:0] dch1 = 8'h30 + {4'd0, dist_bcd[7:4]};
wire [7:0] dch0 = 8'h30 + {4'd0, dist_bcd[3:0]};
wire [7:0] pch2 = 8'h30 + {4'd0, delta_bcd[11:8]};
wire [7:0] pch1 = 8'h30 + {4'd0, delta_bcd[7:4]};
wire [7:0] pch0 = 8'h30 + {4'd0, delta_bcd[3:0]};
wire [7:0] ech2 = 8'h30 + {4'd0, end_bcd[11:8]};
wire [7:0] ech1 = 8'h30 + {4'd0, end_bcd[7:4]};
wire [7:0] ech0 = 8'h30 + {4'd0, end_bcd[3:0]};
wire [7:0] gch3 = 8'h30 + {4'd0, gap_bcd[15:12]};
wire [7:0] gch2 = 8'h30 + {4'd0, gap_bcd[11:8]};
wire [7:0] gch1 = 8'h30 + {4'd0, gap_bcd[7:4]};
wire [7:0] gch0 = 8'h30 + {4'd0, gap_bcd[3:0]};

//==================== 标记线坐标预计算 (寄存, 慢变) ====================
wire [6:0] pre_pts = (disp_len_sel == 2'b00) ? 7'd8  :
                     (disp_len_sel == 2'b01) ? 7'd16 :
                     (disp_len_sel == 2'b10) ? 7'd32 : 7'd64;
wire [9:0] len_pts = (disp_len_sel == 2'b00) ? 10'd64  :
                     (disp_len_sel == 2'b01) ? 10'd128 :
                     (disp_len_sel == 2'b10) ? 10'd256 : 10'd512;
wire [9:0] imp_disp = {4'd0, pre_pts} + {1'b0, impact_index};
wire [9:0] def_disp = {4'd0, pre_pts} + {1'b0, defect_index};
wire [9:0] end_disp = imp_disp + {1'b0, bottom_delta};

// display_index → 屏幕 x (64点=8px/点, 128点=4px/点, 256点=2px/点, 512点=1px/点)
function [9:0] disp2x;
    input [9:0] d;
    input [1:0] ls;
    reg  [12:0] t;
    begin
        t = (ls == 2'b00) ? {d, 3'b000} :
            (ls == 2'b01) ? {1'b0, d, 2'b00} :
            (ls == 2'b10) ? {2'b00, d, 1'b0} : {3'd0, d};
        disp2x = X_LEFT + t[9:0];      // 越界由 *_ok 过滤, 不会用到回绕值
    end
endfunction

reg        imp_ok, def_ok, end_ok;
reg [9:0]  imp_x,  def_x,  end_x;

// 标记线仅在有效结果 (非 INVALID) 时绘制; INVALID 保留波形但无标记
wire mark_valid = result_ready && (result_state != RES_INVALID);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        imp_ok <= 1'b0;  def_ok <= 1'b0;  end_ok <= 1'b0;
        imp_x  <= 10'd0; def_x  <= 10'd0; end_x  <= 10'd0;
    end else begin
        imp_ok <= display_en && mark_valid && (imp_disp < len_pts);
        imp_x  <= disp2x(imp_disp, disp_len_sel);
        def_ok <= display_en && mark_valid &&
                  (result_state == RES_DEFECT) && (defect_index != 9'd0) &&
                  (def_disp < len_pts);
        def_x  <= disp2x(def_disp, disp_len_sel);
        end_ok <= display_en && mark_valid && (end_disp < len_pts);
        end_x  <= disp2x(end_disp, disp_len_sel);
    end
end

//==================== 字符选择 (组合, 进入流水线前) ====================
// 列号: 所有文字区起始 x 为 8 的倍数 → 直接用 pix_x[9:3]
wire [6:0] xq = pix_x[9:3];            // 0~99
wire [6:0] ly = pix_y[9:3];            // 行组号 (文字行 y 均 8 对齐)
wire       in_left  = (xq >= 7'd2)  && (xq < 7'd16);   // x=16~127
wire       in_right = (xq >= 7'd84) && (xq < 7'd98);   // x=672~783
wire [3:0] lcol = xq[3:0] - 4'd2;
wire [3:0] rcol = xq[3:0] - 4'd4;      // 84 低 4 位 = 4
wire [6:0] bcol = xq - 7'd2;           // 底栏/标题列号 (x=16 起)

// LEN 标签: 显示中=已采集长度, 待机=当前配置 (与波形模块一致)
wire [1:0] label_len = display_en ? disp_len_sel : len_sel_cfg;

reg [7:0] ascii;
reg [2:0] tcol3;

always @(*) begin
    ascii = " ";
    tcol3 = C_WHITE;
    if (ly == 7'd1) begin
        // ---- 顶部标题 y=8~15: "DEFECT CHECK" 居中 (x=352 起) ----
        case (bcol)
            7'd42: ascii = "D"; 7'd43: ascii = "E"; 7'd44: ascii = "F";
            7'd45: ascii = "E"; 7'd46: ascii = "C"; 7'd47: ascii = "T";
            7'd49: ascii = "C"; 7'd50: ascii = "H"; 7'd51: ascii = "E";
            7'd52: ascii = "C"; 7'd53: ascii = "K";
            default: ;
        endcase
    end else if (ly == 7'd73) begin
        // ---- 底部状态栏 y=584~591 ----
        case (bcol)
            // "AUTO MODE"
            7'd0:  ascii = "A"; 7'd1:  ascii = "U"; 7'd2:  ascii = "T";
            7'd3:  ascii = "O"; 7'd5:  ascii = "M"; 7'd6:  ascii = "O";
            7'd7:  ascii = "D"; 7'd8:  ascii = "E";
            // "Fs:95.88k"
            7'd14: ascii = "F"; 7'd15: ascii = "s"; 7'd16: ascii = ":";
            7'd17: ascii = "9"; 7'd18: ascii = "5"; 7'd19: ascii = ".";
            7'd20: ascii = "8"; 7'd21: ascii = "8"; 7'd22: ascii = "k";
            // "UART:115200"
            7'd28: ascii = "U"; 7'd29: ascii = "A"; 7'd30: ascii = "R";
            7'd31: ascii = "T"; 7'd32: ascii = ":"; 7'd33: ascii = "1";
            7'd34: ascii = "1"; 7'd35: ascii = "5"; 7'd36: ascii = "2";
            7'd37: ascii = "0"; 7'd38: ascii = "0";
            // 采集状态 WARMUP/READY/CAPTURE/HOLD
            7'd60, 7'd61, 7'd62, 7'd63, 7'd64, 7'd65, 7'd66: begin
                case (cap_state)
                    2'd0: begin           // WARMUP 黄
                        tcol3 = C_YELLOW;
                        case (bcol)
                            7'd60: ascii = "W"; 7'd61: ascii = "A";
                            7'd62: ascii = "R"; 7'd63: ascii = "M";
                            7'd64: ascii = "U"; 7'd65: ascii = "P";
                            default: ;
                        endcase
                    end
                    2'd1: begin           // READY 青
                        tcol3 = C_CYAN;
                        case (bcol)
                            7'd60: ascii = "R"; 7'd61: ascii = "E";
                            7'd62: ascii = "A"; 7'd63: ascii = "D";
                            7'd64: ascii = "Y";
                            default: ;
                        endcase
                    end
                    2'd2: begin           // CAPTURE 黄
                        tcol3 = C_YELLOW;
                        case (bcol)
                            7'd60: ascii = "C"; 7'd61: ascii = "A";
                            7'd62: ascii = "P"; 7'd63: ascii = "T";
                            7'd64: ascii = "U"; 7'd65: ascii = "R";
                            7'd66: ascii = "E";
                            default: ;
                        endcase
                    end
                    default: begin        // HOLD 白
                        case (bcol)
                            7'd60: ascii = "H"; 7'd61: ascii = "O";
                            7'd62: ascii = "L"; 7'd63: ascii = "D";
                            default: ;
                        endcase
                    end
                endcase
            end
            default: ;
        endcase
    end else if (in_left) begin
        // ---- 左侧状态栏 x=16~127 ----
        case (ly)
            7'd8: case (lcol)                       // y=64: "SYSTEM"
                4'd0: ascii = "S"; 4'd1: ascii = "Y"; 4'd2: ascii = "S";
                4'd3: ascii = "T"; 4'd4: ascii = "E"; 4'd5: ascii = "M";
                default: ;
            endcase
            7'd10: begin                            // y=80: 系统状态
                case (cap_state)
                    2'd0: begin                     // WARMUP 黄
                        tcol3 = C_YELLOW;
                        case (lcol)
                            4'd0: ascii = "W"; 4'd1: ascii = "A";
                            4'd2: ascii = "R"; 4'd3: ascii = "M";
                            4'd4: ascii = "U"; 4'd5: ascii = "P";
                            default: ;
                        endcase
                    end
                    2'd1: begin                     // ARMED 青
                        tcol3 = C_CYAN;
                        case (lcol)
                            4'd0: ascii = "A"; 4'd1: ascii = "R";
                            4'd2: ascii = "M"; 4'd3: ascii = "E";
                            4'd4: ascii = "D";
                            default: ;
                        endcase
                    end
                    2'd2: begin                     // CAPTURE 黄
                        tcol3 = C_YELLOW;
                        case (lcol)
                            4'd0: ascii = "C"; 4'd1: ascii = "A";
                            4'd2: ascii = "P"; 4'd3: ascii = "T";
                            4'd4: ascii = "U"; 4'd5: ascii = "R";
                            4'd6: ascii = "E";
                            default: ;
                        endcase
                    end
                    default: begin                  // HOLD 白
                        case (lcol)
                            4'd0: ascii = "H"; 4'd1: ascii = "O";
                            4'd2: ascii = "L"; 4'd3: ascii = "D";
                            default: ;
                        endcase
                    end
                endcase
            end
            7'd14: case (lcol)                      // y=112: "TRIG:"
                4'd0: ascii = "T"; 4'd1: ascii = "R"; 4'd2: ascii = "I";
                4'd3: ascii = "G"; 4'd4: ascii = ":";
                default: ;
            endcase
            7'd16: begin                            // y=128: 触发状态
                if (cap_state == 2'd2 || cap_state == 2'd3) begin
                    tcol3 = C_YELLOW;               // FIRED 黄
                    case (lcol)
                        4'd0: ascii = "F"; 4'd1: ascii = "I";
                        4'd2: ascii = "R"; 4'd3: ascii = "E";
                        4'd4: ascii = "D";
                        default: ;
                    endcase
                end else begin
                    tcol3 = C_CYAN;                 // WAIT 青
                    case (lcol)
                        4'd0: ascii = "W"; 4'd1: ascii = "A";
                        4'd2: ascii = "I"; 4'd3: ascii = "T";
                        default: ;
                    endcase
                end
            end
            7'd20: case (lcol)                      // y=160: "LEN:"
                4'd0: ascii = "L"; 4'd1: ascii = "E"; 4'd2: ascii = "N";
                4'd3: ascii = ":";
                default: ;
            endcase
            7'd22: case (lcol)                      // y=176: 64/128/256/512
                4'd0: ascii = (label_len == 2'b00) ? "6" :
                              (label_len == 2'b01) ? "1" :
                              (label_len == 2'b10) ? "2" : "5";
                4'd1: ascii = (label_len == 2'b00) ? "4" :
                              (label_len == 2'b01) ? "2" :
                              (label_len == 2'b10) ? "5" : "1";
                4'd2: ascii = (label_len == 2'b00) ? " " :
                              (label_len == 2'b01) ? "8" :
                              (label_len == 2'b10) ? "6" : "2";
                default: ;
            endcase
            7'd26: case (lcol)                      // y=208: "FILT:"
                4'd0: ascii = "F"; 4'd1: ascii = "I"; 4'd2: ascii = "L";
                4'd3: ascii = "T"; 4'd4: ascii = ":";
                default: ;
            endcase
            7'd28: begin                            // y=224: RAW/SMOOTH
                if (smooth_on)
                    case (lcol)
                        4'd0: ascii = "S"; 4'd1: ascii = "M";
                        4'd2: ascii = "O"; 4'd3: ascii = "O";
                        4'd4: ascii = "T"; 4'd5: ascii = "H";
                        default: ;
                    endcase
                else
                    case (lcol)
                        4'd0: ascii = "R"; 4'd1: ascii = "A";
                        4'd2: ascii = "W";
                        default: ;
                    endcase
            end
            default: ;
        endcase
    end else if (in_right) begin
        // ---- 右侧结果栏 x=672~783 ----
        case (ly)
            7'd8: case (rcol)                       // y=64: "RESULT"
                4'd0: ascii = "R"; 4'd1: ascii = "E"; 4'd2: ascii = "S";
                4'd3: ascii = "U"; 4'd4: ascii = "L"; 4'd5: ascii = "T";
                default: ;
            endcase
            7'd10: begin                            // y=80: WAIT/NORMAL/DEFECT/RETRY
                if (!result_ready)
                    case (rcol)
                        4'd0: ascii = "W"; 4'd1: ascii = "A";
                        4'd2: ascii = "I"; 4'd3: ascii = "T";
                        default: ;
                    endcase
                else case (result_state)
                    RES_DEFECT: begin
                        tcol3 = C_RED;
                        case (rcol)
                            4'd0: ascii = "D"; 4'd1: ascii = "E";
                            4'd2: ascii = "F"; 4'd3: ascii = "E";
                            4'd4: ascii = "C"; 4'd5: ascii = "T";
                            default: ;
                        endcase
                    end
                    RES_NORMAL: begin
                        tcol3 = C_GREEN;
                        case (rcol)
                            4'd0: ascii = "N"; 4'd1: ascii = "O";
                            4'd2: ascii = "R"; 4'd3: ascii = "M";
                            4'd4: ascii = "A"; 4'd5: ascii = "L";
                            default: ;
                        endcase
                    end
                    default: begin                  // INVALID → RETRY 黄 (§5.5)
                        tcol3 = C_YELLOW;
                        case (rcol)
                            4'd0: ascii = "R"; 4'd1: ascii = "E";
                            4'd2: ascii = "T"; 4'd3: ascii = "R";
                            4'd4: ascii = "Y";
                            default: ;
                        endcase
                    end
                endcase
            end
            7'd14: case (rcol)                      // y=112: "DIST:"
                4'd0: ascii = "D"; 4'd1: ascii = "I"; 4'd2: ascii = "S";
                4'd3: ascii = "T"; 4'd4: ascii = ":";
                default: ;
            endcase
            7'd16: begin                            // y=128: "0506 mm" / "----"
                if (result_ready && result_state == RES_DEFECT) begin
                    tcol3 = C_RED;
                    case (rcol)
                        4'd0: ascii = dch3; 4'd1: ascii = dch2;
                        4'd2: ascii = dch1; 4'd3: ascii = dch0;
                        4'd5: ascii = "m";  4'd6: ascii = "m";
                        default: ;
                    endcase
                end else
                    case (rcol)
                        4'd0, 4'd1, 4'd2, 4'd3: ascii = "-";
                        default: ;
                    endcase
            end
            7'd20: case (rcol)                      // y=160: "PEAK:"
                4'd0: ascii = "P"; 4'd1: ascii = "E"; 4'd2: ascii = "A";
                4'd3: ascii = "K"; 4'd4: ascii = ":";
                default: ;
            endcase
            7'd22: begin                            // y=176: delta 3 位
                if (result_ready && result_state != RES_INVALID)
                    case (rcol)
                        4'd0: ascii = pch2; 4'd1: ascii = pch1;
                        4'd2: ascii = pch0;
                        default: ;
                    endcase
                else
                    case (rcol)
                        4'd0, 4'd1, 4'd2: ascii = "-";
                        default: ;
                    endcase
            end
            7'd26: case (rcol)                      // y=208: "BTM:" (棒底间隔)
                4'd0: ascii = "B"; 4'd1: ascii = "T"; 4'd2: ascii = "M";
                4'd3: ascii = ":";
                default: ;
            endcase
            7'd28: begin                            // y=224: 本次棒底(绿)/D_REF预测(青)
                tcol3 = bottom_found ? C_GREEN : C_CYAN;
                case (rcol)
                    4'd0: ascii = ech2;
                    4'd1: ascii = ech1;
                    4'd2: ascii = ech0;
                    default: ;
                endcase
            end
            7'd32: case (rcol)                      // y=256: "CONF:"
                4'd0: ascii = "C"; 4'd1: ascii = "O"; 4'd2: ascii = "N";
                4'd3: ascii = "F"; 4'd4: ascii = ":";
                default: ;
            endcase
            7'd34: begin                            // y=272: HIGH/LOW/---
                if (!result_ready || confidence == 2'd0)
                    case (rcol)
                        4'd0, 4'd1, 4'd2: ascii = "-";
                        default: ;
                    endcase
                else if (confidence == 2'd2) begin
                    tcol3 = C_GREEN;                // 高置信 (有棒底归一化)
                    case (rcol)
                        4'd0: ascii = "H"; 4'd1: ascii = "I";
                        4'd2: ascii = "G"; 4'd3: ascii = "H";
                        default: ;
                    endcase
                end else begin
                    tcol3 = C_YELLOW;               // 低置信 (无棒底, §7.2)
                    case (rcol)
                        4'd0: ascii = "L"; 4'd1: ascii = "O";
                        4'd2: ascii = "W";
                        default: ;
                    endcase
                end
            end
            7'd38: case (rcol)                      // y=304: "GAP:"
                4'd0: ascii = "G"; 4'd1: ascii = "A";
                4'd2: ascii = "P"; 4'd3: ascii = ":";
                default: ;
            endcase
            7'd40: begin                            // y=320: camera gap preset
                tcol3 = C_CYAN;
                case (rcol)
                    4'd0: ascii = gch3; 4'd1: ascii = gch2;
                    4'd2: ascii = gch1; 4'd3: ascii = gch0;
                    4'd5: ascii = "m";  4'd6: ascii = "m";
                    default: ;
                endcase
            end
            default: ;
        endcase
    end
end

//==================== 3 拍流水线 (与 vga_waveform_precision 对齐) ====================
// T1: 锁存字符/颜色/坐标  T2: 查字模行  T3: 寄存像素判定
// 波形层 rgb 在第 3 拍寄存输出, 故本层判定也在 T3 寄存, 再与 wave_rgb
// 组合叠加 → 两层像素严格同相位
reg [9:0] px_d1, py_d1, px_d2, py_d2;
reg [7:0] ascii_d1;
reg [2:0] tcol_d1, tcol_d2;
reg [2:0] crow_d1;
reg [2:0] ccol_d1, ccol_d2;
reg [7:0] font_row_d2;
reg       valid_d1, valid_d2;

always @(posedge clk) begin
    // ---- T1 ----
    px_d1    <= pix_x;
    py_d1    <= pix_y;
    ascii_d1 <= ascii;
    tcol_d1  <= tcol3;
    crow_d1  <= pix_y[2:0];
    ccol_d1  <= pix_x[2:0];
    valid_d1 <= rgb_valid;
    // ---- T2 ----
    px_d2       <= px_d1;
    py_d2       <= py_d1;
    tcol_d2     <= tcol_d1;
    ccol_d2     <= ccol_d1;
    font_row_d2 <= char_8x8(ascii_d1, crow_d1);
    valid_d2    <= valid_d1;
end

// T2 组合: 字符像素命中
wire on_text = font_row_d2[3'd7 - ccol_d2];

// T2 组合: 标记线
wire in_wave_y = (py_d2 >= Y_TOP) && (py_d2 <= Y_BTM);
// 主冲击: 黄色短竖线 (中线上下 40px)
wire on_imp_mark = imp_ok && (px_d2 == imp_x) &&
                   (py_d2 >= Y_MID - 10'd40) && (py_d2 <= Y_MID + 10'd40);
// 缺陷位置: 红色整高竖线
wire on_def_mark = def_ok && (px_d2 == def_x) && in_wave_y;
// 棒底参考: 紫色虚线 (4px 亮 4px 灭)
wire on_end_mark = end_ok && (px_d2 == end_x) && in_wave_y && ~py_d2[2];

// 文字颜色解码
reg [11:0] text_rgb;
always @(*) begin
    case (tcol_d2)
        C_GREEN:  text_rgb = 12'h0F0;
        C_RED:    text_rgb = 12'hF00;
        C_YELLOW: text_rgb = 12'hFF0;
        C_CYAN:   text_rgb = 12'h0FF;
        default:  text_rgb = 12'hFFF;
    endcase
end

// ---- T3: 寄存判定结果 (与波形层 rgb 输出同拍) ----
reg        text_d3, imp_d3, def_d3, end_d3;
reg [11:0] text_rgb_d3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        text_d3     <= 1'b0;
        imp_d3      <= 1'b0;
        def_d3      <= 1'b0;
        end_d3      <= 1'b0;
        text_rgb_d3 <= 12'h000;
    end else begin
        text_d3     <= on_text     && valid_d2;
        imp_d3      <= on_imp_mark && valid_d2;
        def_d3      <= on_def_mark && valid_d2;
        end_d3      <= on_end_mark && valid_d2;
        text_rgb_d3 <= text_rgb;
    end
end

//==================== 输出叠加 (方案 §7 优先级) ====================
// 缺陷线/文字 > 主冲击线/棒底参考线 > 波形层 (波形层内部已含轴/刻度优先级)
assign rgb_out = def_d3  ? 12'hF00 :
                 text_d3 ? text_rgb_d3 :
                 imp_d3  ? 12'hFF0 :
                 end_d3  ? 12'hF0F :
                           wave_rgb;

endmodule
