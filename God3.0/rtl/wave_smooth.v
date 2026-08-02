// ============================================================================
// Module: wave_smooth
// Description: 可旁路三点平滑滤波 (文档 §14 第八阶段, P2)
//
//   y[n] = (x[n-1] + 2*x[n] + x[n+1]) / 4        (§14.2, 纯移位实现)
//
// 插入位置: wave_trigger 与 wave_capture 之间, 作用于基线扣除后数据
//   (§14.1: filter_mode=0 时 output = corrected 原样通过)
//
// 延迟特性 (关键):
//   - 平滑需要 x[n+1], 输出恒定滞后 1 个样点
//   - bypass 模式同样延迟 1 个样点 (输出 x[n] 原值), 保证两种模式下
//     trigger 与其对应样点的对齐关系一致, 切换模式不引起触发点漂移
//   - trigger 脉冲随样点同步延迟 (in_trigger 与 in_valid 同拍进入,
//     out_trigger 与该样点的 out_valid 同拍输出)
//
// 边界处理: 前 2 个样点仅填充移位寄存器, 不产生输出 (上电/复位后
//   丢 2 点, 相对 95.88kHz 采样率可忽略)
//
// 时钟域: vga_clk 40MHz (样点间隔 ~417 周期, 时序余量充足)
// ============================================================================
module wave_smooth (
    input  wire               clk,          // vga_clk 40MHz
    input  wire               rst_n,        // 低有效异步复位

    input  wire               bypass,       // 1=原始模式 (仅延迟), 0=三点平滑

    // 来自 wave_trigger (corrected_data/valid/trigger 同拍)
    input  wire signed [23:0] in_data,
    input  wire signed [23:0] in_raw,       // 原始样点 (与 in_data 同一样点)
    input  wire               in_valid,
    input  wire               in_trigger,

    // 到 wave_capture (data/valid/trigger 同拍, 滞后 1 样点)
    output reg  signed [23:0] out_data,
    output reg  signed [23:0] out_raw,      // 原始样点随动 (永不平滑, 仅延迟)
    output reg                out_valid,
    output reg                out_trigger
);

    // ------------------------------------------------------------------------
    // 样点移位寄存器: s0 = x[n] (最新已存), s1 = x[n-1]
    //   trig_s0/raw_s0 = x[n] 对应的触发脉冲/原始值 (随样点走)
    // ------------------------------------------------------------------------
    reg signed [23:0] s0, s1;
    reg signed [23:0] raw_s0;
    reg               trig_s0;
    reg [1:0]         prime_cnt;    // 已填充样点数 (饱和到 2)

    // 三点加权和: x[n-1] + 2*x[n] + x[n+1], 26 位有符号防溢出
    //   +2 为右移前四舍五入偏置
    wire signed [25:0] sum3 = {{2{s1[23]}}, s1}
                            + {{1{s0[23]}}, s0, 1'b0}
                            + {{2{in_data[23]}}, in_data}
                            + 26'sd2;
    wire signed [23:0] smoothed = sum3[25:2];   // >>> 2 (凸组合, 不溢出)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0          <= 24'sd0;
            s1          <= 24'sd0;
            raw_s0      <= 24'sd0;
            trig_s0     <= 1'b0;
            prime_cnt   <= 2'd0;
            out_data    <= 24'sd0;
            out_raw     <= 24'sd0;
            out_valid   <= 1'b0;
            out_trigger <= 1'b0;
        end else begin
            out_valid   <= 1'b0;
            out_trigger <= 1'b0;
            if (in_valid) begin
                // 新样点 x[n+1] 到来 → 输出 x[n] 的结果 (平滑或原值)
                if (prime_cnt >= 2'd2) begin
                    out_data    <= bypass ? s0 : smoothed;
                    out_raw     <= raw_s0;
                    out_valid   <= 1'b1;
                    out_trigger <= trig_s0;
                end else begin
                    prime_cnt <= prime_cnt + 1'b1;
                end
                // 移位: x[n+1] 成为新的 s0
                s1      <= s0;
                s0      <= in_data;
                raw_s0  <= in_raw;
                trig_s0 <= in_trigger;
            end
        end
    end

endmodule
