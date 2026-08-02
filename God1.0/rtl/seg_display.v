//======================================================================
// Module: seg_display
// Function: 74HC595 驱动 8位数码管, 显示桩长 (XXXX.X 米)
//
// 接口:
//   seg_din  — 串行数据
//   seg_sclk — 移位时钟 (~78kHz)
//   seg_rclk — 锁存时钟 (64位发送完成后给一个锁存脉冲)
//   (OE 板级已接地 = 常显, 不占用 FPGA 引脚)
//
// 显示格式 (8位, 右对齐):
//   "    12.5" → 桩长 12.5米
//   "   100.0" → 桩长 100.0米
//   小数点在从右数第2位
//
// Date:    2026-07-16
//======================================================================

module seg_display (
    input  wire         clk,           // 系统时钟
    input  wire         rst_n,

    // 输入值 (桩长×10 dm, 如 125 = 12.5m)
    input  wire [15:0]  value,
    input  wire         value_valid,

    // 74HC595 接口
    output reg          seg_din,
    output reg          seg_sclk,
    output reg          seg_rclk
    // 注: OE 信号已由板级接地(常显), 不再占用 FPGA 引脚
);

//======================================================================
// 7段译码 LUT (共阴极: 0=灭 1=亮)
//   段顺序: {dp, g, f, e, d, c, b, a}
//======================================================================
function [7:0] seg_lut;
    input [3:0] digit;
    input       dp;
    begin
        case (digit)
            4'd0:  seg_lut = 8'b00111111;  // "0"
            4'd1:  seg_lut = 8'b00000110;  // "1"
            4'd2:  seg_lut = 8'b01011011;  // "2"
            4'd3:  seg_lut = 8'b01001111;  // "3"
            4'd4:  seg_lut = 8'b01100110;  // "4"
            4'd5:  seg_lut = 8'b01101101;  // "5"
            4'd6:  seg_lut = 8'b01111101;  // "6"
            4'd7:  seg_lut = 8'b00000111;  // "7"
            4'd8:  seg_lut = 8'b01111111;  // "8"
            4'd9:  seg_lut = 8'b01101111;  // "9"
            default: seg_lut = 8'b00000000;
        endcase
        if (dp) seg_lut[7] = 1'b1;
    end
endfunction

//======================================================================
// Binary → BCD (16-bit → 5位BCD, 最大65535)
//======================================================================
function [19:0] bin2bcd;
    input [15:0] bin;
    reg [35:0] s;
    integer j;
    begin
        s = {20'b0, bin};
        for (j = 0; j < 16; j = j + 1) begin
            if (s[19:16] >= 4'd5) s[19:16] = s[19:16] + 4'd3;
            if (s[23:20] >= 4'd5) s[23:20] = s[23:20] + 4'd3;
            if (s[27:24] >= 4'd5) s[27:24] = s[27:24] + 4'd3;
            if (s[31:28] >= 4'd5) s[31:28] = s[31:28] + 4'd3;
            if (s[35:32] >= 4'd5) s[35:32] = s[35:32] + 4'd3;
            s = s << 1;
        end
        bin2bcd = s[35:16];  // 5位BCD = 20-bit
    end
endfunction

//======================================================================
// 显示寄存器 (锁存新值, 用变化检测替代多驱信号)
//======================================================================
reg [15:0] disp_value;
reg [15:0] disp_value_d1;
wire       new_data;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        disp_value    <= 16'd0;
        disp_value_d1 <= 16'hFFFF;  // 上电即触发一次发送, 初始显示"0.0"
    end
    else begin
        disp_value_d1 <= disp_value;
        if (value_valid)
            disp_value <= value;
    end
end

assign new_data = (disp_value != disp_value_d1);

// BCD转换 (连续进行, 不耗额外周期)
wire [19:0] bcd_val;
assign bcd_val = bin2bcd(disp_value);

// 8位数码管映射 (seg7=最左, seg0=最右):
//   d7     d6     d5     d4     d3    d2    d1    d0
//   万     千     百    十.dp   个    blank blank blank
//   前导零消隐, 十位和个位始终显示
//
// 例: 125(12.5m) → "     12.5  "
//     99(9.9m)    → "      9.9  "
//     1000(100m)  → "    100.0  "

wire b4 = (bcd_val[19:16] != 4'd0);  // 万
wire b3 = (bcd_val[15:12] != 4'd0);  // 千
wire b2 = (bcd_val[11:8]  != 4'd0);  // 百

wire [7:0] seg7 = b4                         ? seg_lut(bcd_val[19:16], 1'b0) : 8'h00;
wire [7:0] seg6 = b3 || b4                   ? seg_lut(bcd_val[15:12], 1'b0) : 8'h00;
wire [7:0] seg5 = b2 || b3 || b4             ? seg_lut(bcd_val[11:8],  1'b0) : 8'h00;
wire [7:0] seg4 =                                 seg_lut(bcd_val[7:4],   1'b1);   // 十位+dp 始终显示
wire [7:0] seg3 =                                 seg_lut(bcd_val[3:0],   1'b0);   // 个位 始终显示
wire [7:0] seg2 = 8'h00;
wire [7:0] seg1 = 8'h00;
wire [7:0] seg0 = 8'h00;

//======================================================================
// 74HC595 发送状态机 (64-bit: 8位×8段)
//   时序 (修复后):
//     低半周期: 更新 DIN 为下一位, SCLK 保持低
//     高半周期: SCLK 上升沿, 595 采样已稳定的 DIN
//     64 位全部移入后: SCLK 拉低, RCLK 保持一个 sclk_div 周期完成锁存
//   ⚠️ 旧版在上升沿同拍才更新数据且少发 1 个上升沿 (64位只移63位),
//      导致显示错位, 已修复, 勿回退
//======================================================================
reg [6:0]  bit_cnt;      // 0~63
reg [63:0] shift_reg;
reg [9:0]  sclk_div;
reg        busy;
reg        sclk_high;    // 0=低半周期(更新数据), 1=高半周期(上升沿)
reg        latch_req;    // 64位发送完成, 等待 RCLK 锁存
reg        pending;      // 发送期间数值又变化 → 挂起, 空闲后补发

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        seg_din   <= 1'b0;
        seg_sclk  <= 1'b0;
        seg_rclk  <= 1'b0;
        bit_cnt   <= 7'd0;
        shift_reg <= 64'd0;
        sclk_div  <= 10'd0;
        busy      <= 1'b0;
        sclk_high <= 1'b0;
        latch_req <= 1'b0;
        pending   <= 1'b0;
    end
    else begin
        // 发送期间数值变化 → 登记挂起 (变化脉冲仅1拍, 不能丢)
        if (new_data && (busy || latch_req))
            pending <= 1'b1;

        if (latch_req) begin
            // 锁存阶段: SCLK 拉低, RCLK 保持一个 sclk_div 周期 (~6.4µs)
            seg_sclk <= 1'b0;
            if (sclk_div == 10'd256) begin
                sclk_div  <= 10'd0;
                seg_rclk  <= 1'b0;
                latch_req <= 1'b0;
            end
            else begin
                sclk_div <= sclk_div + 1'b1;
                seg_rclk <= 1'b1;
            end
        end
        else if (!busy) begin
            seg_sclk <= 1'b0;
            if (new_data || pending) begin
                // 8位段码 → 64-bit移位寄存器 (MSB先出: seg7→seg0)
                shift_reg <= {seg7, seg6, seg5, seg4, seg3, seg2, seg1, seg0};
                bit_cnt   <= 7'd0;
                sclk_div  <= 10'd0;
                sclk_high <= 1'b0;
                busy      <= 1'b1;
                pending   <= 1'b0;
            end
        end
        else begin
            // SCLK半周期 = 257 clk @40MHz ≈ 6.4µs → SCLK ≈ 78kHz
            sclk_div <= sclk_div + 1'b1;
            if (sclk_div == 10'd256) begin
                sclk_div <= 10'd0;
                if (!sclk_high) begin
                    // 低半周期: 移出下一位到 DIN, SCLK 保持低
                    seg_din   <= shift_reg[63];
                    shift_reg <= {shift_reg[62:0], 1'b0};
                    seg_sclk  <= 1'b0;
                    sclk_high <= 1'b1;
                end
                else begin
                    // 高半周期: SCLK 上升沿, 74HC595 采样稳定的 DIN
                    seg_sclk  <= 1'b1;
                    sclk_high <= 1'b0;
                    if (bit_cnt == 7'd63) begin
                        busy      <= 1'b0;
                        latch_req <= 1'b1;
                    end
                    bit_cnt <= bit_cnt + 1'b1;
                end
            end
        end
    end
end

endmodule
