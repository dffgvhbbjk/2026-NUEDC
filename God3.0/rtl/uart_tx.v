`timescale  1ns/1ns
////////////////////////////////////////////////////////////////////////
// Author        : EmbedFire
// Create Date   : 2019/06/12
// Module Name   : uart_tx
// Project Name  : uart_sdram
// Target Devices: Altera EP4CE10F17C8N
// Tool Versions : Quartus 13.0
// Description   : 
// 
// Revision      : V1.0
// Additional Comments:
// 
// 实验平台: 野火_征途Pro_FPGA开发板
// 公司    : http://www.embedfire.com
// 论坛    : http://www.firebbs.cn
// 淘宝    : https://fire-stm32.taobao.com
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
reg [3:0]   bit_cnt;
reg         work_en;
reg [7:0]   data_latch;

//********************************************************************//
//***************************** Main Code ****************************//
//********************************************************************//
// ready 仅在完整停止位发送结束后拉高。
assign  ready = ~work_en;

// 8N1 发送状态:
//   bit_cnt=0: 起始位
//   bit_cnt=1~8: data[0]~data[7]
//   bit_cnt=9: 停止位
// 接收请求时锁存 pi_data，避免发送期间上游数据变化。
always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        baud_cnt   <= 13'd0;
        bit_cnt    <= 4'd0;
        work_en    <= 1'b0;
        data_latch <= 8'd0;
        tx          <= 1'b1;
    end else if (!work_en) begin
        baud_cnt <= 13'd0;
        bit_cnt  <= 4'd0;
        tx       <= 1'b1;
        if (pi_flag) begin
            data_latch <= pi_data;
            work_en    <= 1'b1;
            tx          <= 1'b0;
        end
    end else if (baud_cnt == BAUD_CNT_MAX - 1) begin
        baud_cnt <= 13'd0;
        case (bit_cnt)
            4'd0: begin bit_cnt <= 4'd1; tx <= data_latch[0]; end
            4'd1: begin bit_cnt <= 4'd2; tx <= data_latch[1]; end
            4'd2: begin bit_cnt <= 4'd3; tx <= data_latch[2]; end
            4'd3: begin bit_cnt <= 4'd4; tx <= data_latch[3]; end
            4'd4: begin bit_cnt <= 4'd5; tx <= data_latch[4]; end
            4'd5: begin bit_cnt <= 4'd6; tx <= data_latch[5]; end
            4'd6: begin bit_cnt <= 4'd7; tx <= data_latch[6]; end
            4'd7: begin bit_cnt <= 4'd8; tx <= data_latch[7]; end
            4'd8: begin bit_cnt <= 4'd9; tx <= 1'b1;          end
            default: begin
                bit_cnt <= 4'd0;
                work_en <= 1'b0;
                tx      <= 1'b1;
            end
        endcase
    end else begin
        baud_cnt <= baud_cnt + 1'b1;
    end
end

endmodule
