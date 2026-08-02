//======================================================================
// Module: uart_rx_2M
// Function: 2Mbps UART 接收器 + PC 指令解析
//   复用现有 uart_rx 模块 (2Mbps, 50MHz, 分频=25, 0% 误差)
//   指令协议: 0x5A <cmd> <data_hi> <data_lo> <checksum>
//   checksum = cmd ^ data_hi ^ data_lo
//   cmd: 0x01=设阈值, 0x02=设标定系数, 0x03=选检测行, 0x04=开始/停止
//
//   v2 (2026-08-01): 波特率 460800 → 2000000 (与 uart_tx_2M 同步, CH343 2Mbps, 原 5M PC 端跑不通回退)
//
// Clock:  50MHz (sys_clk 域)
//======================================================================
module uart_rx_2M (
    input  wire        clk,           // 50MHz
    input  wire        rst_n,
    input  wire        rxd,           // 串口接收
    // 解析后的指令
    output reg  [7:0]  cmd,           // 命令码
    output reg  [15:0] cmd_data,      // 命令数据
    output reg         cmd_valid      // 指令有效脉冲
);

    //==================================================================
    // UART 接收器 (复用现有模块)
    //==================================================================
    wire [7:0] rx_data;
    wire       rx_flag;

    uart_rx #(
        .UART_BPS (2_000_000),   // 与 uart_tx_2M 同步回退 2M (50MHz/2M=25, 0% 误差)
        .CLK_FREQ (50_000_000)
    ) u_rx (
        .sys_clk   (clk),
        .sys_rst_n (rst_n),
        .rx        (rxd),
        .po_data   (rx_data),
        .po_flag   (rx_flag)
    );

    //==================================================================
    // 指令解析状态机
    //==================================================================
    localparam ST_HEAD  = 3'd0;   // 等待帧头 0x5A
    localparam ST_CMD   = 3'd1;   // 接收命令码
    localparam ST_HI    = 3'd2;   // 接收数据高字节
    localparam ST_LO    = 3'd3;   // 接收数据低字节
    localparam ST_CKSUM = 3'd4;   // 接收校验和

    reg [2:0]  state;
    reg [7:0]  cmd_reg;
    reg [7:0]  data_hi;
    reg [7:0]  data_lo;
    reg [7:0]  cksum_calc;        // 计算的校验和

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_HEAD;
            cmd         <= 8'd0;
            cmd_data    <= 16'd0;
            cmd_valid   <= 1'b0;
            cmd_reg     <= 8'd0;
            data_hi     <= 8'd0;
            data_lo     <= 8'd0;
            cksum_calc  <= 8'd0;
        end else begin
            cmd_valid <= 1'b0;   // 默认低

            if (rx_flag) begin
                case (state)
                    //--------------------------------------------------
                    // 等待帧头 0x5A
                    //--------------------------------------------------
                    ST_HEAD: begin
                        if (rx_data == 8'h5A)
                            state <= ST_CMD;
                    end

                    //--------------------------------------------------
                    // 接收命令码
                    //--------------------------------------------------
                    ST_CMD: begin
                        cmd_reg    <= rx_data;
                        cksum_calc <= rx_data;   // cksum = cmd
                        state      <= ST_HI;
                    end

                    //--------------------------------------------------
                    // 接收数据高字节
                    //--------------------------------------------------
                    ST_HI: begin
                        data_hi    <= rx_data;
                        cksum_calc <= cksum_calc ^ rx_data;
                        state      <= ST_LO;
                    end

                    //--------------------------------------------------
                    // 接收数据低字节
                    //--------------------------------------------------
                    ST_LO: begin
                        data_lo    <= rx_data;
                        cksum_calc <= cksum_calc ^ rx_data;
                        state      <= ST_CKSUM;
                    end

                    //--------------------------------------------------
                    // 校验和比对
                    //--------------------------------------------------
                    ST_CKSUM: begin
                        if (rx_data == cksum_calc) begin
                            // 校验通过, 输出指令
                            cmd       <= cmd_reg;
                            cmd_data  <= {data_hi, data_lo};
                            cmd_valid <= 1'b1;
                        end
                        state <= ST_HEAD;
                    end

                    default: state <= ST_HEAD;
                endcase
            end
        end
    end

endmodule
