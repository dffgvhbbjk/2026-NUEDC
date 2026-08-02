`timescale  1ns/1ns    // 时间单位/时间精度：单位1ns，精度1ns，用于仿真延迟定义

//======================================================================
// Module: vga_ctrl_simple
// Function: VGA时序控制模块（800×600@60Hz 标准VESA时序）
//           输出行场同步信号、有效显示标志、像素坐标，供上层模块绘制波形/画面
// Input :  vga_clk - VGA像素时钟，800×600@60Hz 对应40MHz
//          sys_rst_n - 全局复位，低电平有效
// Output:  pix_x/pix_y - 当前像素的屏幕坐标（有效区范围0~799 / 0~599）
//          rgb_valid - 有效显示区域标志，高电平时可输出RGB颜色
//          hsync/vsync - VGA行/场同步信号，负极性标准
// Date:    2026-07-19
//======================================================================

module vga_ctrl_simple (
    input  wire         vga_clk,    // VGA像素时钟输入，40MHz对应800×600@60Hz
    input  wire         sys_rst_n,  // 系统复位，低电平有效

    output wire [9:0]   pix_x,      // 输出当前像素X坐标（水平方向，0~799）
    output wire [9:0]   pix_y,      // 输出当前像素Y坐标（垂直方向，0~599）
    output wire         rgb_valid,  // 有效显示区标志：高电平=处于可视区域，可输出RGB数据
    output wire         hsync,      // VGA行同步信号（水平同步）
    output wire         vsync       // VGA场同步信号（垂直同步）
);

//==================== 时序参数定义（800×600@60Hz 标准VESA, 40MHz时钟） ====================
// 行方向时序参数（单位：像素时钟周期）
parameter H_SYNC    =   11'd128,  // 行同步脉冲宽度
          H_BACK    =   11'd88,   // 行后沿
          H_VALID   =   11'd800,  // 行有效显示宽度（800像素）
          H_FRONT   =   11'd40,   // 行前沿
          H_TOTAL   =   11'd1056; // 一行总像素数（128+88+800+40=1056）

// 场方向时序参数（单位：行）
parameter V_SYNC    =   10'd4,    // 场同步脉冲宽度（4行）
          V_BACK    =   10'd23,   // 场后沿
          V_VALID   =   10'd600,  // 场有效显示高度（600行）
          V_FRONT   =   10'd1,    // 场前沿
          V_TOTAL   =   10'd628;  // 一帧总行数（4+23+600+1=628）

//==================== 内部计数器定义 ====================
reg [10:0] cnt_h;  // 行计数器（11-bit: 0~1055）
reg [9:0]  cnt_v;  // 场计数器（10-bit: 0~627）

//==================== 行计数器逻辑 ====================
always @(posedge vga_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)                // 复位时行计数器清零
        cnt_h <= 11'd0;
    else if (cnt_h == H_TOTAL - 1'd1)  // 计完一行总像素，清零开始下一行
        cnt_h <= 11'd0;
    else                           // 每个像素时钟，行计数器+1
        cnt_h <= cnt_h + 1'b1;
end

//==================== 场计数器逻辑 ====================
always @(posedge vga_clk or negedge sys_rst_n) begin
    if (!sys_rst_n)                // 复位时场计数器清零
        cnt_v <= 10'd0;
    else if ((cnt_v == V_TOTAL - 1'd1) && (cnt_h == H_TOTAL - 1'd1))
        cnt_v <= 10'd0;            // 计完一帧最后一行的最后一个像素，清零开始下一帧
    else if (cnt_h == H_TOTAL - 1'd1)
        cnt_v <= cnt_v + 1'b1;     // 每一行结束（行计数器到最大值），场计数器+1
end

//==================== 行/场同步信号生成（负极性同步） ====================
// 行同步：前H_SYNC个像素输出低电平（同步脉冲），其余时间高电平
assign hsync = (cnt_h <= H_SYNC - 1'd1) ? 1'b0 : 1'b1;
// 场同步：前V_SYNC行输出低电平（同步脉冲），其余时间高电平
assign vsync = (cnt_v <= V_SYNC - 1'd1) ? 1'b0 : 1'b1;

//==================== 有效显示区域标志 ====================
assign rgb_valid =
    ((cnt_h >= H_SYNC + H_BACK) &&
     (cnt_h <  H_SYNC + H_BACK + H_VALID)) &&
    ((cnt_v >= V_SYNC + V_BACK) &&
     (cnt_v <  V_SYNC + V_BACK + V_VALID));

//==================== 像素坐标转换 ====================
assign pix_x = rgb_valid ? (cnt_h - (H_SYNC + H_BACK)) : 10'd0;
assign pix_y = rgb_valid ? (cnt_v - (V_SYNC + V_BACK)) : 10'd0;

endmodule
