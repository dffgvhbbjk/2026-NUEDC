//======================================================================
// Module: gap_detect
// Function: 行扫描缝隙检测状态机
//   每行扫描: IDLE → WAIT_BLK1 → IN_RECT1 → IN_GAP → IN_RECT2
//   在 IN_RECT1→IN_GAP 锁存 gap_left, IN_GAP→IN_RECT2 锁存 gap_right
//   行末若到达 IN_RECT2, 输出 gap_pix = right - left + 1
//
//   对所有行检测 (不只 detect_row), 为 gap_vote 提供多行样本.
//   gap_left/gap_right 仅在 detect_row 行更新 (供上位机画红线),
//   其余行保持 0x7F 哨兵 (UART 层打包为 0xFF).
//
//   v3 (2026-08-01): 摄像头改用野火官方 640×480 RGB565 配置,
//   行末判定改为 pix_x>=639 (原 pix_x==799 在 640 宽下永不触发, gap 检测失效);
//   gap_left/gap_right 为 /10 的 64-wide 坐标 (0-63), 哨兵 0x7F.
//
// Clock:  pclk 域
//======================================================================
module gap_detect (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire [7:0]  y_data,        // Y 通道
    input  wire        y_valid,       // Y 有效脉冲
    input  wire [9:0]  pix_x,         // 列坐标 0-799
    input  wire [9:0]  pix_y,         // 行坐标 0-599
    input  wire [7:0]  threshold,     // 二值化阈值 (PC 可调)
    input  wire [9:0]  detect_row,    // 检测行 (PC 可调, 用于红线显示)
    input  wire        frame_sync,    // 帧起始脉冲 (来自 capture)
    output reg  [9:0]  gap_pix,       // 缝隙像素宽度 (全分辨率)
    output reg         gap_valid,     // 检测有效脉冲 (每有效行末拉高 1 拍)
    output reg  [6:0]  gap_left,      // 缝隙左边界 (64-wide, 0-63 或 0x7F=哨兵)
    output reg  [6:0]  gap_right      // 缝隙右边界 (64-wide, 0-63 或 0x7F=哨兵)
);

    //------------------------------------------------------------------
    // 状态定义
    //------------------------------------------------------------------
    localparam IDLE      = 3'd0;
    localparam WAIT_BLK1 = 3'd1;   // 等待第一个黑矩形
    localparam IN_RECT1  = 3'd2;   // 在第一个黑矩形内
    localparam IN_GAP    = 3'd3;   // 在缝隙内 (白)
    localparam IN_RECT2  = 3'd4;   // 在第二个黑矩形内

    reg [2:0] state;
    reg [9:0] gap_left_full;       // 全分辨率左边界 (0-799)
    reg [9:0] gap_right_full;      // 全分辨率右边界 (0-799)

    //------------------------------------------------------------------
    // 辅助信号
    //------------------------------------------------------------------
    wire is_black  = (y_data < threshold);
    wire row_start = y_valid && (pix_x == 10'd0);
    // 640×480 官方配置: 每行 640 像素 (pix_x 0-639), 行末在 pix_x>=639 判定
    // 若摄像头实际输出略宽 (>640), 也只会多触发同值重复, 不影响投票
    wire row_end   = y_valid && (pix_x >= 10'd639);

    // 全分辨率 → 64-wide 降采样坐标 (/10), 用于 detect_row 行的 gap_left/right
    //   用乘倒数实现: x*6554 >> 16, 6554/65536 ≈ 0.1000061.
    //   对 x≤1023 误差 < 0.007, 小于 x/10 的最小余数 0.1, 结果精确.
    //   显式移位加法, 避开 HDL `/` (cbx_lpm_divide.dll) 与 `*` (cbx_lpm_mult.dll)
    wire [23:0] gap_left_p  = (gap_left_full  << 12) + (gap_left_full  << 11)
                            + (gap_left_full  << 8)  + (gap_left_full  << 7)
                            + (gap_left_full  << 4)  + (gap_left_full  << 3)
                            + (gap_left_full  << 1);
    wire [23:0] gap_right_p = (gap_right_full << 12) + (gap_right_full << 11)
                            + (gap_right_full << 8)  + (gap_right_full << 7)
                            + (gap_right_full << 4)  + (gap_right_full << 3)
                            + (gap_right_full << 1);
    wire [9:0] gap_left_ds  = gap_left_p[23:16];
    wire [9:0] gap_right_ds = gap_right_p[23:16];

    //------------------------------------------------------------------
    // 主状态机 + 输出逻辑
    //------------------------------------------------------------------
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            gap_pix        <= 10'd0;
            gap_valid      <= 1'b0;
            gap_left_full  <= 10'd0;
            gap_right_full <= 10'd0;
            gap_left       <= 7'h7F;   // 哨兵
            gap_right      <= 7'h7F;
        end else begin
            gap_valid <= 1'b0;   // 默认低

            //----------------------------------------------------------
            // 帧同步: 复位状态 + 重置哨兵
            //----------------------------------------------------------
            if (frame_sync) begin
                state     <= IDLE;
                gap_left  <= 7'h7F;    // 哨兵 (本帧未检测到 detect_row 有效值)
                gap_right <= 7'h7F;
            end
            //----------------------------------------------------------
            // 行起始: 复位状态机, 准备扫描新行
            //   (pix_x==0 的 Y 不处理, 从 pix_x==1 开始)
            //----------------------------------------------------------
            else if (row_start) begin
                state <= WAIT_BLK1;
            end
            //----------------------------------------------------------
            // 行内处理: 根据 Y 值转移状态
            //----------------------------------------------------------
            else if (y_valid) begin
                case (state)
                    WAIT_BLK1: begin
                        if (is_black)
                            state <= IN_RECT1;
                    end
                    IN_RECT1: begin
                        if (!is_black) begin
                            state         <= IN_GAP;
                            gap_left_full <= pix_x;
                        end
                    end
                    IN_GAP: begin
                        if (is_black) begin
                            state          <= IN_RECT2;
                            gap_right_full <= pix_x;
                        end
                    end
                    IN_RECT2: begin
                        // 等待行末
                    end
                    default: state <= WAIT_BLK1;
                endcase
            end

            //----------------------------------------------------------
            // 行末输出: 若到达 IN_RECT2, 计算缝隙宽度
            //----------------------------------------------------------
            if (row_end && state == IN_RECT2) begin
                gap_pix   <= gap_right_full - gap_left_full + 1'b1;
                gap_valid <= 1'b1;
                // 仅在 detect_row 行更新 gap_left/gap_right (供上位机红线)
                if (pix_y == detect_row) begin
                    gap_left  <= gap_left_ds[6:0];   // /10 → 64-wide
                    gap_right <= gap_right_ds[6:0];
                end
            end
        end
    end

endmodule
