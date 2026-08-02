//======================================================================
// Module: vga_waveform_precision
// Function: 高精度波形显示 (文档 §13 第七阶段)
//
//   - 显示宽度 512px (X_LEFT=144 ~ X_RIGHT=656), 整数像素/点映射 (§13.4):
//       64 点 → 8px/点, 128 点 → 4px/点, 256 点 → 2px/点, 512 点 → 1px/点
//     每个采样点都显示, 不跳点、不丢点 (替代旧版 1.5 倍分数映射)
//   - 手动增益 (§13.1): gain_shift 0~4 → ×1/×2/×4/×8/×16
//   - 饱和限幅 (§13.3): 21 位中间结果, Y 越界钳位, 不绕回屏幕另一端
//   - 数据来源: wave_buffer_dp (16 位有符号, 已基线扣除), 读延迟 1 拍
//   - 屏幕标签: 左上=采集长度(待机显当前配置), 右上=增益档,
//               轴下右侧=本次波形时间跨度 (按实际 fs=95.8807kHz):
//               64点=0.67ms, 128点=1.34ms, 256点=2.67ms, 512点=5.34ms (§7.1)
//
// 时钟域: vga_clk 40MHz
//======================================================================
module vga_waveform_precision (
    input  wire               vga_clk,
    input  wire               rst_n,

    // wave_buffer_dp 读口 (1 拍延迟)
    output wire [9:0]         rd_addr,
    input  wire signed [15:0] rd_data,

    // 来自 wave_capture
    input  wire               display_en,     // 1=显示波形
    input  wire [9:0]         disp_origin,    // 显示起点 {bank, origin}
    input  wire [1:0]         disp_len_sel,   // 已采集波形长度 (锁存)

    // 配置
    input  wire [1:0]         len_sel_cfg,    // 当前长度配置 (待机时标签显示)
    input  wire [2:0]         gain_shift,     // 显示增益档 0~4 (×1~×16)
    input  wire               gain_is_auto,   // 1=自动增益 (标签加 "A" 前缀)

    // VGA
    input  wire [9:0]         pix_x,
    input  wire [9:0]         pix_y,
    input  wire               rgb_valid,
    output reg  [11:0]        rgb
);

// 显示区域: 512px 宽, 居中 ((800-512)/2 = 144)
parameter X_LEFT  = 10'd144;
parameter X_RIGHT = 10'd656;   // X_LEFT + 512
parameter Y_TOP   = 10'd30;
parameter Y_BTM   = 10'd569;
parameter Y_MID   = 10'd300;
// 基础缩放: 16 位样本 >>> 7 → ±256px (满幅), 之后再乘增益
parameter BASE_SHIFT = 7;

//==================== 横坐标映射 (§13.4, 整数像素/点) ====================
wire [9:0] xrel    = pix_x - X_LEFT;
wire       in_span = (pix_x >= X_LEFT) && (pix_x < X_RIGHT);

// 采样点索引: 64点→xrel/8, 128点→xrel/4, 256点→xrel/2, 512点→xrel/1 (纯移位)
wire [8:0] idx = (disp_len_sel == 2'b00) ? {2'd0, xrel[8:3]} :
                 (disp_len_sel == 2'b01) ? {1'd0, xrel[8:2]} :
                 (disp_len_sel == 2'b10) ? xrel[8:1] : xrel[8:0];

// RAM 地址: bank 位 + bank 内回绕 (乒乓 bank 由 wave_capture 管理)
assign rd_addr = {disp_origin[9], disp_origin[8:0] + idx};

//==================== 两级流水线 (与旧版 vga_waveform 相同结构) ====================
// Stage 1 (T1): RAM 内部寄存 rd_data, 同拍锁存像素坐标
// Stage 2 (T2): 增益缩放 + 饱和 → wave_y, 与 pix_y_d2 对齐判断
reg [9:0] pix_x_d1, pix_y_d1, pix_x_d2, pix_y_d2;
reg       in_wave_d1, in_wave_d2;
reg       rgb_valid_d1, rgb_valid_d2;   // 有效区标志跟随两拍流水线
reg [8:0] idx_d1, idx_d2;
reg       idx_chg, idx_chg_d1;
reg [9:0] wave_y;
reg [9:0] wave_y_prev;

//==================== 纵坐标计算 (§13.1/§13.3) ====================
// scaled = sample <<< gain_shift (16+4=20 位, 用 21 位有符号防溢出)
wire signed [20:0] sample_ext = {{5{rd_data[15]}}, rd_data};
wire signed [20:0] scaled     = sample_ext <<< gain_shift;
// 右移前有符号四舍五入 (同旧版: 防低位被优化, 提高小信号显示精度)
wire signed [20:0] round_bias =
    scaled[20] ? ((21'sd1 <<< BASE_SHIFT) - 21'sd1)
               :  (21'sd1 <<< (BASE_SHIFT - 1));
wire signed [20:0] sample_px  = (scaled + round_bias) >>> BASE_SHIFT;
// y = Y_MID - sample_px, 22 位有符号中间结果
wire signed [21:0] wave_y_calc = $signed({12'd0, Y_MID}) - sample_px;

always @(posedge vga_clk) begin
    // ---- Stage 1: 锁存像素 (RAM 同拍锁存 rd_data) ----
    pix_x_d1   <= pix_x;
    pix_y_d1   <= pix_y;
    in_wave_d1 <= in_span && (pix_y >= Y_TOP) && (pix_y <= Y_BTM);
    idx_d1     <= idx;
    rgb_valid_d1 <= rgb_valid;

    // ---- Stage 2: 增益/饱和 → wave_y ----
    pix_x_d2   <= pix_x_d1;
    pix_y_d2   <= pix_y_d1;
    in_wave_d2 <= in_wave_d1;
    idx_d2     <= idx_d1;
    rgb_valid_d2 <= rgb_valid_d1;
    idx_chg    <= (idx_d1 != idx_d2);
    idx_chg_d1 <= idx_chg;

    // 点切换时保存上一点 Y 坐标 (垂直连线)
    if (idx_d1 != idx_d2)
        wave_y_prev <= wave_y;

    // 饱和限幅 (§13.3): 越界钳位, 禁止绕回
    if (wave_y_calc < $signed({12'd0, Y_TOP + 10'd1}))
        wave_y <= Y_TOP + 10'd1;
    else if (wave_y_calc > $signed({12'd0, Y_BTM - 10'd1}))
        wave_y <= Y_BTM - 10'd1;
    else
        wave_y <= wave_y_calc[9:0];
end

//==================== 波形/连线命中 ====================
wire on_wave = display_en && in_wave_d2 &&
               (pix_y_d2 >= wave_y - 1) && (pix_y_d2 <= wave_y + 1);
// 连线仅在点切换后前 2 列绘制; 第 0 点无前一点, 禁止连线
// (消隐期 idx 回绕会污染 wave_y_prev, 若不屏蔽会在左边缘画出伪竖线)
wire line_active = (idx_chg || idx_chg_d1) && (idx_d2 != 9'd0);
wire on_line = display_en && in_wave_d2 && line_active &&
    ((pix_y_d2 >= wave_y && pix_y_d2 <= wave_y_prev) ||
     (pix_y_d2 >= wave_y_prev && pix_y_d2 <= wave_y));

//==================== 坐标轴/刻度 ====================
wire on_y_axis = (pix_x_d2 == X_LEFT) && (pix_y_d2 >= Y_TOP) && (pix_y_d2 <= Y_BTM);
wire on_x_axis = (pix_y_d2 == Y_MID) && (pix_x_d2 >= X_LEFT) && (pix_x_d2 <= X_RIGHT);
wire on_y_ticks = (pix_x_d2 >= X_LEFT - 3) && (pix_x_d2 <= X_LEFT) &&
                  ((pix_y_d2 == Y_MID) || (pix_y_d2 == Y_TOP + 135) || (pix_y_d2 == Y_BTM - 135));
// X 轴刻度: 主刻度每 64px (8 格), 小刻度每 32px
wire [9:0] xrel2 = (pix_x_d2 >= X_LEFT) ? (pix_x_d2 - X_LEFT) : 10'd1000;
wire on_x_major = (pix_y_d2 >= Y_MID) && (pix_y_d2 <= Y_MID + 4) &&
                  (xrel2 <= 10'd512) && (xrel2[5:0] == 6'd0);
wire on_x_minor = (pix_y_d2 >= Y_MID) && (pix_y_d2 <= Y_MID + 2) &&
                  (xrel2 <= 10'd512) && (xrel2[4:0] == 5'd0) && (xrel2[5:0] != 6'd0);
wire on_x_ticks = on_x_major || on_x_minor;

//==================== 8x8 字符 ROM ====================
function [7:0] char_8x8;
    input [7:0] ascii;
    input [2:0] row;
    begin
        case (ascii)
            "0": case (row) 3'd0: char_8x8=8'b00111100; 3'd1: char_8x8=8'b01100110; 3'd2: char_8x8=8'b01101110; 3'd3: char_8x8=8'b01110110; 3'd4: char_8x8=8'b01100110; 3'd5: char_8x8=8'b01100110; 3'd6: char_8x8=8'b00111100; default: char_8x8=0; endcase
            "1": case (row) 3'd0: char_8x8=8'b00011000; 3'd1: char_8x8=8'b00111000; 3'd2: char_8x8=8'b00011000; 3'd3: char_8x8=8'b00011000; 3'd4: char_8x8=8'b00011000; 3'd5: char_8x8=8'b00011000; 3'd6: char_8x8=8'b01111110; default: char_8x8=0; endcase
            "2": case (row) 3'd0: char_8x8=8'b00111100; 3'd1: char_8x8=8'b01100110; 3'd2: char_8x8=8'b00000110; 3'd3: char_8x8=8'b00001100; 3'd4: char_8x8=8'b00011000; 3'd5: char_8x8=8'b00110000; 3'd6: char_8x8=8'b01111110; default: char_8x8=0; endcase
            "3": case (row) 3'd0: char_8x8=8'b00111100; 3'd1: char_8x8=8'b01100110; 3'd2: char_8x8=8'b00000110; 3'd3: char_8x8=8'b00011100; 3'd4: char_8x8=8'b00000110; 3'd5: char_8x8=8'b01100110; 3'd6: char_8x8=8'b00111100; default: char_8x8=0; endcase
            "4": case (row) 3'd0: char_8x8=8'b00001100; 3'd1: char_8x8=8'b00011100; 3'd2: char_8x8=8'b00101100; 3'd3: char_8x8=8'b01001100; 3'd4: char_8x8=8'b01111110; 3'd5: char_8x8=8'b00001100; 3'd6: char_8x8=8'b00001100; default: char_8x8=0; endcase
            "5": case (row) 3'd0: char_8x8=8'b01111110; 3'd1: char_8x8=8'b01100000; 3'd2: char_8x8=8'b01111100; 3'd3: char_8x8=8'b00000110; 3'd4: char_8x8=8'b00000110; 3'd5: char_8x8=8'b01100110; 3'd6: char_8x8=8'b00111100; default: char_8x8=0; endcase
            "6": case (row) 3'd0: char_8x8=8'b00111100; 3'd1: char_8x8=8'b01100110; 3'd2: char_8x8=8'b01100000; 3'd3: char_8x8=8'b01111100; 3'd4: char_8x8=8'b01100110; 3'd5: char_8x8=8'b01100110; 3'd6: char_8x8=8'b00111100; default: char_8x8=0; endcase
            "7": case (row) 3'd0: char_8x8=8'b01111110; 3'd1: char_8x8=8'b00000110; 3'd2: char_8x8=8'b00001100; 3'd3: char_8x8=8'b00011000; 3'd4: char_8x8=8'b00110000; 3'd5: char_8x8=8'b00110000; 3'd6: char_8x8=8'b00110000; default: char_8x8=0; endcase
            "8": case (row) 3'd0: char_8x8=8'b00111100; 3'd1: char_8x8=8'b01100110; 3'd2: char_8x8=8'b01100110; 3'd3: char_8x8=8'b00111100; 3'd4: char_8x8=8'b01100110; 3'd5: char_8x8=8'b01100110; 3'd6: char_8x8=8'b00111100; default: char_8x8=0; endcase
            "9": case (row) 3'd0: char_8x8=8'b00111100; 3'd1: char_8x8=8'b01100110; 3'd2: char_8x8=8'b01100110; 3'd3: char_8x8=8'b00111110; 3'd4: char_8x8=8'b00000110; 3'd5: char_8x8=8'b01100110; 3'd6: char_8x8=8'b00111100; default: char_8x8=0; endcase
            "t": case (row) 3'd1: char_8x8=8'b00010000; 3'd2: char_8x8=8'b01111100; 3'd3: char_8x8=8'b00010000; 3'd4: char_8x8=8'b00010000; 3'd5: char_8x8=8'b00010000; 3'd6: char_8x8=8'b00001100; default: char_8x8=0; endcase
            "m": case (row) 3'd2: char_8x8=8'b01101100; 3'd3: char_8x8=8'b01101100; 3'd4: char_8x8=8'b01101100; 3'd5: char_8x8=8'b01101100; 3'd6: char_8x8=8'b01101100; default: char_8x8=0; endcase
            "s": case (row) 3'd2: char_8x8=8'b00111100; 3'd3: char_8x8=8'b01100000; 3'd4: char_8x8=8'b00111100; 3'd5: char_8x8=8'b00000110; 3'd6: char_8x8=8'b01111100; default: char_8x8=0; endcase
            "x": case (row) 3'd2: char_8x8=8'b01100110; 3'd3: char_8x8=8'b00111100; 3'd4: char_8x8=8'b00011000; 3'd5: char_8x8=8'b00111100; 3'd6: char_8x8=8'b01100110; default: char_8x8=0; endcase
            "p": case (row) 3'd2: char_8x8=8'b01111100; 3'd3: char_8x8=8'b01100110; 3'd4: char_8x8=8'b01111100; 3'd5: char_8x8=8'b01100000; 3'd6: char_8x8=8'b01100000; default: char_8x8=0; endcase
            "A": case (row) 3'd0: char_8x8=8'b00011000; 3'd1: char_8x8=8'b00111100; 3'd2: char_8x8=8'b01100110; 3'd3: char_8x8=8'b01100110; 3'd4: char_8x8=8'b01111110; 3'd5: char_8x8=8'b01100110; 3'd6: char_8x8=8'b01100110; default: char_8x8=0; endcase
            ".": case (row) 3'd5: char_8x8=8'b00011000; 3'd6: char_8x8=8'b00011000; default: char_8x8=0; endcase
            " ": char_8x8 = 0;
            default: char_8x8 = 0;
        endcase
    end
endfunction

function [1:0] draw_char_at;
    input [9:0] px, py, lx, ly;
    input [7:0] ch;
    reg [7:0] rd;
    reg [2:0] ri, ci;
    begin
        draw_char_at = 2'b00;
        rd = 8'd0;
        ri = 3'd0;
        ci = 3'd0;
        if ((px >= lx) && (px < lx + 10'd8) && (py >= ly) && (py < ly + 10'd8)) begin
            ri = py[2:0] - ly[2:0];
            ci = px[2:0] - lx[2:0];
            rd = char_8x8(ch, ri);
            if (rd[3'd7 - ci]) draw_char_at = 2'b01;
        end
    end
endfunction

//==================== 屏幕标签 ====================
// 左上: 采集长度 + "pts" (显示中=已采集长度, 待机=当前配置)
wire [1:0] label_len = display_en ? disp_len_sel : len_sel_cfg;
wire [7:0] lc0 = (label_len == 2'b00) ? " " : (label_len == 2'b01) ? "1" :
                  (label_len == 2'b10) ? "2" : "5";
wire [7:0] lc1 = (label_len == 2'b00) ? "6" : (label_len == 2'b01) ? "2" :
                  (label_len == 2'b10) ? "5" : "1";
wire [7:0] lc2 = (label_len == 2'b00) ? "4" : (label_len == 2'b01) ? "8" :
                  (label_len == 2'b10) ? "6" : "2";
wire [9:0] lly = 10'd14;
wire [1:0] ll0 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT,     lly,lc0);
wire [1:0] ll1 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+9,   lly,lc1);
wire [1:0] ll2 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+18,  lly,lc2);
wire [1:0] llp = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+30,  lly,"p");
wire [1:0] llt = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+39,  lly,"t");
wire [1:0] lls = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+48,  lly,"s");
wire on_len_label = ll0[0]|ll1[0]|ll2[0]|llp[0]|llt[0]|lls[0];

// 右上: 增益档 "x1"~"x16", 自动增益模式加 "A" 前缀 ("Ax4" 等)
wire [7:0] gc1 = (gain_shift == 3'd0) ? "1" : (gain_shift == 3'd1) ? "2" :
                 (gain_shift == 3'd2) ? "4" : (gain_shift == 3'd3) ? "8" : "1";
wire [7:0] gc2 = (gain_shift >= 3'd4) ? "6" : " ";
wire [1:0] gla = draw_char_at(pix_x_d2,pix_y_d2,X_RIGHT-36, lly, gain_is_auto ? "A" : " ");
wire [1:0] gl0 = draw_char_at(pix_x_d2,pix_y_d2,X_RIGHT-27, lly,"x");
wire [1:0] gl1 = draw_char_at(pix_x_d2,pix_y_d2,X_RIGHT-18, lly,gc1);
wire [1:0] gl2 = draw_char_at(pix_x_d2,pix_y_d2,X_RIGHT-9,  lly,gc2);
wire on_gain_label = gla[0]|gl0[0]|gl1[0]|gl2[0];

// 轴下右侧: 时间跨度 "0.67ms"/"1.34ms"/"2.67ms" (fs=95.8807kHz 实际换算)
wire [7:0] tc0 = (label_len == 2'b00) ? "0" : (label_len == 2'b01) ? "1" :
                  (label_len == 2'b10) ? "2" : "5";
wire [7:0] tc2 = (label_len == 2'b00) ? "6" : (label_len == 2'b01) ? "3" :
                  (label_len == 2'b10) ? "6" : "3";
wire [7:0] tc3 = (label_len == 2'b00) ? "7" : (label_len == 2'b01) ? "4" :
                  (label_len == 2'b10) ? "7" : "4";
wire [9:0] tly = Y_MID + 10'd8;
wire [9:0] tlx = X_RIGHT - 10'd54;
wire [1:0] tm0 = draw_char_at(pix_x_d2,pix_y_d2,tlx,     tly,tc0);
wire [1:0] tm1 = draw_char_at(pix_x_d2,pix_y_d2,tlx+9,   tly,".");
wire [1:0] tm2 = draw_char_at(pix_x_d2,pix_y_d2,tlx+18,  tly,tc2);
wire [1:0] tm3 = draw_char_at(pix_x_d2,pix_y_d2,tlx+27,  tly,tc3);
wire [1:0] tm4 = draw_char_at(pix_x_d2,pix_y_d2,tlx+37,  tly,"m");
wire [1:0] tm5 = draw_char_at(pix_x_d2,pix_y_d2,tlx+46,  tly,"s");
wire on_time_label = tm0[0]|tm1[0]|tm2[0]|tm3[0]|tm4[0]|tm5[0];

//==================== RGB 输出 ====================
// rgb_valid 经 d2 两拍对齐, 与 pix_x_d2/pix_y_d2 同相位 (避免有效区边缘错位)
always @(posedge vga_clk or negedge rst_n) begin
    if (!rst_n)
        rgb <= 12'b0;
    else if (rgb_valid_d2) begin
        if (on_wave || on_line)
            rgb <= 12'h0F0;
        else if (on_len_label || on_time_label)
            rgb <= 12'hFF0;
        else if (on_gain_label)
            rgb <= 12'h0FF;
        else if (on_y_axis || on_x_axis)
            rgb <= 12'hFFF;
        else if (on_y_ticks || on_x_ticks)
            rgb <= 12'h888;
        else
            rgb <= 12'h000;
    end else
        rgb <= 12'b0;
end

endmodule
