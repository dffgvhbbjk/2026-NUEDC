// ============================================================================
// Module: pcm1808_i2s_rx
// Description: PCM1808 24-bit I2S Slave Receiver (strict, single-valid)
//
// 整改点 (对应文档 §3.1, §8):
//   - 每个声道每个 LRCLK 周期只产生一次 valid (在第 24 位接收完成时)
//   - 不再同时使用 "LRCLK 边沿锁存" 和 "时隙结束锁存" 两套完成逻辑
//   - I2S 格式 (FMT=0): LRCLK 边沿所在 BCLK 周期即为 1 位延迟位,
//     边沿后第 1~24 个 posedge (slot_count=0..23) 采 MSB~LSB
//     (修复 P0 BUG: 旧版从 slot_count=1 开始采, 整体晚一位, 丢失符号位)
//     参考 NXP I2S 规范 UM11732: WS 比 MSB 提前一个完整位时钟改变
//
// 时钟域: bclk
// 时序前提 (文档 §7.2):
//   - LRCLK 在 negedge bclk 翻转 (FPGA 产生)
//   - FPGA 在 posedge bclk 采样 DOUT
//   - PCM1808 DOUT 在 BCK 下降沿后更新
//   - 半周期 ~81.5ns 满足 LRCLK 建立时间 50ns
//
// 接口兼容 iis_slave_rx: 同名端口, 可直接替换
// ============================================================================
module pcm1808_i2s_rx #(
    parameter DATA_WIDTH = 24,
    parameter I2S_MODE   = 1    // 1=I2S (1-bit delay), 0=Left-Justified (0 delay)
) (
    input  wire                    rst_n,      // 异步复位, 低有效 (bclk 域)
    input  wire                    bclk,       // I2S 位时钟 (~6.136MHz)
    input  wire                    lrclk,      // I2S 帧时钟 (~95.88kHz)
    input  wire                    sdin,       // 串行数据输入

    output reg  [DATA_WIDTH-1:0]   left_data,  // 左声道并行数据
    output reg  [DATA_WIDTH-1:0]   right_data, // 右声道并行数据
    output reg                     left_valid, // 左声道数据有效 (1 bclk 脉冲)
    output reg                     right_valid,// 右声道数据有效 (1 bclk 脉冲)

    output reg                     frame_error // 帧错误标志 (slot_count 异常)
);

    // ------------------------------------------------------------------------
    // LRCLK 边沿检测 (1 级延迟)
    //   LRCLK 由 FPGA 在 negedge bclk 产生, 与 posedge bclk 同域, 无亚稳态.
    //   1 级延迟即可检测边沿.
    // ------------------------------------------------------------------------
    reg lrclk_d1;
    wire lrclk_edge;

    always @(posedge bclk or negedge rst_n) begin
        if (!rst_n)
            lrclk_d1 <= 1'b0;
        else
            lrclk_d1 <= lrclk;
    end

    assign lrclk_edge = lrclk ^ lrclk_d1;

    // ------------------------------------------------------------------------
    // 声道内位置计数器
    //   slot_count: 0..31 (BCLK_DIV=32, 每声道 32 BCLK)
    //   LRCLK 边沿所在 posedge 为延迟位 (不计入 slot_count, 边沿时清零)
    //   I2S: slot_count=0..23 为数据位 (MSB..LSB)
    //        slot_count=24..30 为尾随空闲位
    //   时序推导 (LRCLK 在 negedge N0 翻转):
    //     N0: LRCLK 翻转 → P0: FPGA 检测 lrclk_edge, slot_count<=0 (延迟位)
    //     N1: PCM1808 移出 MSB → P1: slot_count=0, 采样 MSB
    // ------------------------------------------------------------------------
    reg [5:0]  slot_count;   // 6 位, 支持 0..63
    reg        channel;      // 0=Left, 1=Right (锁存 LRCLK 边沿时的新值)
    reg [DATA_WIDTH-1:0] shift_reg;

    // ------------------------------------------------------------------------
    // 核心接收逻辑
    //   原则: 一个声道只产生一次 valid, 在第 24 位数据位接收完成时输出.
    //   不再有时隙结束的第二次锁存.
    // ------------------------------------------------------------------------
    always @(posedge bclk or negedge rst_n) begin
        if (!rst_n) begin
            slot_count   <= 6'd0;
            channel      <= 1'b0;
            shift_reg    <= {DATA_WIDTH{1'b0}};
            left_data    <= {DATA_WIDTH{1'b0}};
            right_data   <= {DATA_WIDTH{1'b0}};
            left_valid   <= 1'b0;
            right_valid  <= 1'b0;
            frame_error  <= 1'b0;
        end else begin
            // 默认撤销 valid 脉冲
            left_valid   <= 1'b0;
            right_valid  <= 1'b0;
            frame_error  <= 1'b0;

            if (lrclk_edge) begin
                // LRCLK 边沿: 新声道开始
                // channel 锁存 LRCLK 新值: 0=Left, 1=Right
                channel    <= lrclk;
                slot_count <= 6'd0;
                // Left-Justified 格式: MSB 与 LRCLK 边沿同拍, 在边沿周期采样
                if (I2S_MODE == 0)
                    shift_reg <= {shift_reg[DATA_WIDTH-2:0], sdin};
            end else begin
                // 非边沿周期: 递增 slot_count
                slot_count <= slot_count + 1'b1;

                if (I2S_MODE == 1) begin
                    // I2S 格式: 边沿周期本身即延迟位,
                    //   边沿后第 1 个非边沿 posedge (slot_count=0) 采 MSB
                    //   slot_count=0..23 采 MSB..LSB (修复 P0 晚一位 BUG)
                    if (slot_count <= 6'd23) begin
                        shift_reg <= {shift_reg[DATA_WIDTH-2:0], sdin};

                        if (slot_count == 6'd23) begin
                            // 第 24 位接收完成: 锁存数据并产生 valid
                            // {shift_reg[22:0], sdin} = 完整 24 位 (MSB..LSB)
                            if (channel == 1'b0) begin
                                left_data  <= {shift_reg[DATA_WIDTH-2:0], sdin};
                                left_valid <= 1'b1;
                            end else begin
                                right_data  <= {shift_reg[DATA_WIDTH-2:0], sdin};
                                right_valid <= 1'b1;
                            end
                        end
                    end
                end else begin
                    // Left-Justified: MSB 已在边沿周期采样,
                    //   slot_count=0..22 采 bit22..bit0, 在 22 锁存完整 24 位
                    if (slot_count <= 6'd22) begin
                        shift_reg <= {shift_reg[DATA_WIDTH-2:0], sdin};

                        if (slot_count == 6'd22) begin
                            if (channel == 1'b0) begin
                                left_data  <= {shift_reg[DATA_WIDTH-2:0], sdin};
                                left_valid <= 1'b1;
                            end else begin
                                right_data  <= {shift_reg[DATA_WIDTH-2:0], sdin};
                                right_valid <= 1'b1;
                            end
                        end
                    end
                end

                // 帧错误检测: slot_count 超过 31仍未检测到 LRCLK 边沿
                if (slot_count >= 6'd32)
                    frame_error <= 1'b1;
            end
        end
    end

endmodule
