//======================================================================
// vga_waveform — 锤击触发示波器显示
//   95.88kHz 采样, 16-bit signed × 2048点循环缓冲 (21.3ms), 描点连线
//   display_en=0 待机只显坐标轴, =1 锤击冻结后显波形
//======================================================================
module vga_waveform (
    input  wire         vga_clk,
    input  wire         rst_n,
    input  wire [23:0]  ad_data,
    input  wire         ad_data_vld,
    input  wire         freeze,         // 1=冻结显示
    input  wire         snap,           // 敲击脉冲: 锁定显示原点
    input  wire         display_en,     // 1=显示波形, 0=仅坐标轴(待机)
    input  wire [9:0]   pix_x,
    input  wire [9:0]   pix_y,
    input  wire         rgb_valid,
    output reg  [11:0]  rgb
);

// 显示区域: 640px × 1.5 = 960点 = 10.0ms @96kHz
// 20格 × 32px/格, 每格 500us; 主刻度标签 0/2/4/6/8/10 间距128px(4格=2ms)
parameter X_LEFT  = 10'd80;
parameter X_RIGHT = 10'd720;
parameter Y_TOP   = 10'd30;
parameter Y_BTM   = 10'd569;
parameter Y_MID   = 10'd300;
// 显示增益: 16-bit 样本算术右移后映射为像素。
// 7 表示相较旧 8-bit 显示提高 2 倍灵敏度；减小该值会继续放大。
parameter DISPLAY_SHIFT = 7;

// 16-bit × 2048 点缓冲 = 32768 bit, 使用 4 个 M9K。
// 保存 PCM1808 24-bit 补码的高 16 位，量化精度比旧 8-bit RAM 提高 256 倍。
localparam BUF_SIZE = 2048;
(* ramstyle = "M9K" *) reg signed [15:0] wave_buf [0:BUF_SIZE-1];
integer i;
initial begin
    for (i = 0; i < BUF_SIZE; i = i + 1)
        wave_buf[i] = 16'sd0;
end

// 写入: 每个 ad_data_vld 写一个点, freeze=1 时停止
reg [10:0] wr_ptr;
// snap 时锁定显示原点, 使锤击点对齐左边缘
reg [10:0] disp_origin;
always @(posedge vga_clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_ptr      <= 11'd0;
        disp_origin <= 11'd0;
    end else begin
        if (snap)
            disp_origin <= wr_ptr;
        if (ad_data_vld && !freeze)
            wr_ptr <= wr_ptr + 1'b1;
    end
end

// RAM 写端不能带异步复位，否则 Cyclone IV 可能无法推断 M9K。
// PCM1808 输出为 24-bit 二进制补码；保留高 16 位并保持符号格式。
always @(posedge vga_clk) begin
    if (ad_data_vld && !freeze)
        wave_buf[wr_ptr] <= $signed(ad_data[23:8]);
end

// 显示读出: 640px=10.0ms @95.88kHz (1.5倍压缩, 960点), 锤击点对齐最左(x_buf=0)
wire [9:0]  x_offset = pix_x - X_LEFT;
wire [11:0] x_scaled = x_offset * 3;
wire [10:0] x_buf    = x_scaled >> 1;
wire [10:0] rd_addr  = (disp_origin + x_buf) & 11'd2047;

reg signed [15:0] sample;
reg [9:0]  pix_x_d1, pix_y_d1, pix_x_d2, pix_y_d2;
reg        in_wave_d1, in_wave_d2;
reg [10:0] x_buf_d1, x_buf_d2;
reg        x_buf_chg, x_buf_chg_d1;
reg [9:0]  wave_y;           // 当前 x_buf 对应的波形 Y 坐标
reg [9:0]  wave_y_prev;      // 上一个 x_buf 对应的波形 Y 坐标 (用于连线)
wire signed [16:0] sample_ext = {sample[15], sample};
// 右移前做有符号四舍五入，使被移出的低位仍参与显示结果，
// 同时防止综合器因“低位未使用”而缩减 16-bit RAM 的物理宽度。
wire signed [16:0] round_bias =
    sample[15] ? ((17'sd1 <<< DISPLAY_SHIFT) - 17'sd1)
               :  (17'sd1 <<< (DISPLAY_SHIFT - 1));
wire signed [16:0] sample_rounded = sample_ext + round_bias;
wire signed [16:0] sample_px      = sample_rounded >>> DISPLAY_SHIFT;
wire signed [17:0] wave_y_calc =
    $signed({1'b0, Y_MID}) - sample_px;

// 两级 pipeline, 保证 wave_y 与 pix_y 对齐 (修复 v1.0 波峰跑到下面的 bug):
//   Stage 1 (T1): sample = wave_buf[rd_addr], pix_y_d1 = pix_y, x_buf_d1 = x_buf
//   Stage 2 (T2): 16-bit signed sample缩放、饱和并映射到Y坐标
//   判断 (T2): pix_y_d2 和 wave_y 都对应同一像素, 时序对齐
always @(posedge vga_clk) begin
    // ---- Stage 1: 读 M9K + 锁存像素 ----
    sample     <= wave_buf[rd_addr];
    pix_x_d1   <= pix_x;
    pix_y_d1   <= pix_y;
    in_wave_d1 <= (pix_x >= X_LEFT) && (pix_x <= X_RIGHT) &&
                  (pix_y >= Y_TOP)  && (pix_y <= Y_BTM);
    x_buf_d1   <= x_buf;

    // ---- Stage 2: 计算 wave_y + 延迟像素 ----
    pix_x_d2   <= pix_x_d1;
    pix_y_d2   <= pix_y_d1;
    in_wave_d2 <= in_wave_d1;
    x_buf_d2   <= x_buf_d1;
    x_buf_chg    <= (x_buf_d1 != x_buf_d2);
    x_buf_chg_d1 <= x_buf_chg;

    // wave_y_prev: x_buf 切换时, 保存旧的 wave_y (对应上一个 x_buf)
    if (x_buf_d1 != x_buf_d2)
        wave_y_prev <= wave_y;

    // wave_y: 基于上一拍读出的有符号 sample 计算，并限制在绘图区内。
    if (wave_y_calc < $signed({1'b0, Y_TOP + 10'd1}))
        wave_y <= Y_TOP + 10'd1;
    else if (wave_y_calc > $signed({1'b0, Y_BTM - 10'd1}))
        wave_y <= Y_BTM - 10'd1;
    else
        wave_y <= wave_y_calc[9:0];
end

// 波形命中: 用 pix_y_d2 和 wave_y (都在 Stage 2 对齐)
wire on_wave = display_en && in_wave_d2 && (pix_y_d2 >= wave_y - 1) && (pix_y_d2 <= wave_y + 1);
// 连线: x_buf 变化时, 在 wave_y 和 wave_y_prev 之间画垂直线
wire line_active = x_buf_chg || x_buf_chg_d1;
wire on_line = display_en && in_wave_d2 && line_active &&
    ((pix_y_d2 >= wave_y && pix_y_d2 <= wave_y_prev) ||
     (pix_y_d2 >= wave_y_prev && pix_y_d2 <= wave_y));

// UI (统一用 d2 与波形对齐)
wire on_y_axis = (pix_x_d2 == X_LEFT) && (pix_y_d2 >= Y_TOP) && (pix_y_d2 <= Y_BTM);
wire on_x_axis = (pix_y_d2 == Y_MID) && (pix_x_d2 >= X_LEFT) && (pix_x_d2 <= X_RIGHT);
wire on_y_ticks = (pix_x_d2 >= X_LEFT - 3) && (pix_x_d2 <= X_LEFT) &&
                   ((pix_y_d2 == Y_MID) || (pix_y_d2 == Y_TOP + 135) || (pix_y_d2 == Y_BTM - 135));
// X轴刻度: 主刻度 128px(2ms), 小刻度 32px(500us)
wire [9:0] xrel = (pix_x_d2 >= X_LEFT) ? (pix_x_d2 - X_LEFT) : 10'd1000;
wire on_x_major = (pix_y_d2 >= Y_MID) && (pix_y_d2 <= Y_MID + 4) &&
                  (xrel <= 10'd640) && (xrel[6:0] == 7'd0);   // 每128px主刻度
wire on_x_minor = (pix_y_d2 >= Y_MID) && (pix_y_d2 <= Y_MID + 2) &&
                  (xrel <= 10'd640) && (xrel[4:0] == 5'd0) && (xrel[6:0] != 7'd0);  // 每32px小刻度(不含主刻度位置)
wire on_x_ticks = on_x_major || on_x_minor;

// 8x8 字符 ROM (仅数字)
function [7:0] char_8x8;
    input [6:0] ascii;
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
            "/": case (row) 3'd0: char_8x8=8'b00000110; 3'd1: char_8x8=8'b00001100; 3'd2: char_8x8=8'b00011000; 3'd3: char_8x8=8'b00110000; 3'd4: char_8x8=8'b01100000; 3'd5: char_8x8=8'b01000000; default: char_8x8=0; endcase
            "m": case (row) 3'd2: char_8x8=8'b01101100; 3'd3: char_8x8=8'b01101100; 3'd4: char_8x8=8'b01101100; 3'd5: char_8x8=8'b01101100; 3'd6: char_8x8=8'b01101100; default: char_8x8=0; endcase
            "s": case (row) 3'd2: char_8x8=8'b00111100; 3'd3: char_8x8=8'b01100000; 3'd4: char_8x8=8'b00111100; 3'd5: char_8x8=8'b00000110; 3'd6: char_8x8=8'b01111100; default: char_8x8=0; endcase
            " ": char_8x8 = 0;
            default: char_8x8 = 0;
        endcase
    end
endfunction

function [1:0] draw_char_at;
    input [9:0] px, py, lx, ly;
    input [6:0] ch;
    reg [7:0] rd;
    reg [2:0] ri, ci;
    begin
        draw_char_at = 2'b00;
        if ((px >= lx) && (px < lx + 10'd8) && (py >= ly) && (py < ly + 10'd8)) begin
            ri = py[2:0] - ly[2:0];
            ci = px[2:0] - lx[2:0];
            rd = char_8x8(ch, ri);
            if (rd[3'd7 - ci]) draw_char_at = 2'b01;
        end
    end
endfunction

// 时间标签 "0" "2" "4" "6" "8" "10"
wire [9:0] tly = Y_MID + 10'd4;
wire [1:0] tl0 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+  0-4,tly,"0");
wire [1:0] tl2 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+128-4,tly,"2");
wire [1:0] tl4 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+256-4,tly,"4");
wire [1:0] tl6 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+384-4,tly,"6");
wire [1:0] tl8 = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+512-4,tly,"8");
wire [1:0] tla = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+640-13,tly,"1");
wire [1:0] tlb = draw_char_at(pix_x_d2,pix_y_d2,X_LEFT+640-5,tly,"0");
wire on_tlabel = tl0[0]|tl2[0]|tl4[0]|tl6[0]|tl8[0]|tla[0]|tlb[0];

// "t/ms" 标签
wire [9:0] tlx = X_RIGHT - 10'd36;
wire [1:0] tl_t = draw_char_at(pix_x_d2,pix_y_d2,tlx,    Y_MID+10'd8,"t");
wire [1:0] tl_s = draw_char_at(pix_x_d2,pix_y_d2,tlx+9,  Y_MID+10'd8,"/");
wire [1:0] tl_m = draw_char_at(pix_x_d2,pix_y_d2,tlx+18, Y_MID+10'd8,"m");
wire [1:0] tl_ss= draw_char_at(pix_x_d2,pix_y_d2,tlx+27, Y_MID+10'd8,"s");
wire on_tms = tl_t[0]|tl_s[0]|tl_m[0]|tl_ss[0];

// RGB
always @(posedge vga_clk or negedge rst_n) begin
    if (!rst_n)
        rgb <= 12'b0;
    else if (rgb_valid) begin
        if (on_wave || on_line)
            rgb <= 12'h0F0;
        else if (on_tlabel || on_tms)
            rgb <= 12'hFF0;
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
