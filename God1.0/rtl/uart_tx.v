`timescale  1ns/1ns
////////////////////////////////////////////////////////////////////////
// Author        : EmbedFire (original) / modified for 2Mbps fix
// Create Date   : 2019/06/12
// Module Name   : uart_tx
// Project Name  : uart_sdram
// Target Devices: Altera EP4CE6E22C8
// Tool Versions : Quartus 18.1
// Description   : UART 发送器, 修复 2Mbps 下停止位被截短的 bug
//
// Revision      : V1.1 (2026-07-27 修复停止位时序)
//
// 修复说明:
//   原代码 bit_flag 在 baud_cnt==1 触发, bit_cnt 也在 baud_cnt==1 递增,
//   导致 bit_cnt==9(停止位)时 work_en 立即清零, 停止位仅持续 2 拍.
//   2Mbps 下停止位应为 500ns(25 拍), 原代码仅 40ns, PC 采样点累积偏移.
//
//   修复方案:
//   1. bit_flag 在 baud_cnt==0 (bit 开始) 触发, 用于更新 tx 输出
//   2. bit_cnt 在 baud_cnt==BAUD_CNT_MAX-1 (bit 结束) 递增
//   3. work_en 在停止位完整结束 (baud_cnt==MAX-1 && bit_cnt==9) 时清零
//   这样每个 bit (含起始/数据/停止位) 都完整持续 25 拍 = 500ns
////////////////////////////////////////////////////////////////////////

module  uart_tx
#(
    parameter   UART_BPS    =   'd9600,         //串口波特率
    parameter   CLK_FREQ    =   'd50_000_000    //时钟频率
)
(
     input   wire            sys_clk     ,   //系统时钟50MHz
     input   wire            sys_rst_n   ,   //全局复位
     input   wire    [7:0]   pi_data     ,   //模块输入的8bit数据
     input   wire            pi_flag     ,   //并行数据有效标志信号

     output  wire            ready       ,   //发送模块空闲标志(work_en==0)
     output  reg             tx              //串转并后的1bit数据
);

//********************************************************************//
//****************** Parameter and Internal Signal *******************//
//********************************************************************//
//localparam    define
localparam  BAUD_CNT_MAX    =   CLK_FREQ/UART_BPS   ;

//reg   define
reg [12:0]  baud_cnt;
reg         bit_flag;
reg [3:0]   bit_cnt ;
reg         work_en ;

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
//work_en: 发送工作使能信号
//  pi_flag 触发启动, 停止位完整发送后(baud_cnt计满 && bit_cnt==9)清零
always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            work_en <= 1'b0;
        else    if(pi_flag == 1'b1)
            work_en <= 1'b1;
        else    if((work_en == 1'b1) && (baud_cnt == BAUD_CNT_MAX - 1) && (bit_cnt == 4'd9))
            work_en <= 1'b0;

//ready: 当发送模块空闲(work_en==0)时拉高, 供外部判断是否可以写入新数据
assign  ready = ~work_en;

//baud_cnt: 波特率计数器, 0 到 BAUD_CNT_MAX-1 循环
//  work_en==0 时保持 0 (空闲); work_en==1 时递增; 计满后清零
always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            baud_cnt <= 13'b0;
        else    if((work_en == 1'b0) || (baud_cnt == BAUD_CNT_MAX - 1))
            baud_cnt <= 13'b0;
        else
            baud_cnt <= baud_cnt + 1'b1;

//bit_flag: 在每个 bit 的开始(baud_cnt==0, work_en==1)拉高一拍
//  用于在该拍更新 tx 输出为新 bit 的值
always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            bit_flag <= 1'b0;
        else    if((work_en == 1'b1) && (baud_cnt == 13'd0))
            bit_flag <= 1'b1;
        else
            bit_flag <= 1'b0;

//bit_cnt: 数据位数计数 (0=起始位, 1-8=数据位, 9=停止位)
//  在每个 bit 结束(baud_cnt==MAX-1)时递增, bit_cnt==9 时回绕到 0
//  注意: 4-bit 计数器 9+1=10 不会自动回绕, 必须显式判断
always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        bit_cnt <= 4'b0;
    else    if((work_en == 1'b1) && (baud_cnt == BAUD_CNT_MAX - 1)) begin
        if (bit_cnt == 4'd9)
            bit_cnt <= 4'b0;
        else
            bit_cnt <= bit_cnt + 1'b1;
    end

//tx: 输出数据, 满足 RS232 协议 (起始位0, 停止位1, LSB first)
//  在 bit_flag 拉高时(bit 开始)更新 tx 为当前 bit_cnt 对应的值
always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            tx <= 1'b1; //空闲状态时为高电平
        else    if(bit_flag == 1'b1)
            case(bit_cnt)
                0       : tx <= 1'b0;          //起始位
                1       : tx <= pi_data[0];    //bit0 (LSB)
                2       : tx <= pi_data[1];
                3       : tx <= pi_data[2];
                4       : tx <= pi_data[3];
                5       : tx <= pi_data[4];
                6       : tx <= pi_data[5];
                7       : tx <= pi_data[6];
                8       : tx <= pi_data[7];    //bit7 (MSB)
                9       : tx <= 1'b1;          //停止位
                default : tx <= 1'b1;
            endcase

endmodule
