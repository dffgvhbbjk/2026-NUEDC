// ============================================================================
// Module: key_debounce
// Description: 按键同步 + 按下边沿脉冲 (低有效按键, 20ms 稳定计数 @40MHz)
//   与已验证硬件案例一致: 两级同步 → 电平立即跟随 → 饱和稳定计数
//   计数器不延迟按键响应, 避免短按被 20ms 确认窗口吞掉。
// ============================================================================
module key_debounce #(
    parameter STABLE_CNT = 20'd800_000   // 20ms @ 40MHz
) (
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,        // 低有效按键 (空闲高电平)
    output wire press_pulse    // 按下瞬间单周期脉冲
);

    reg [1:0]  sync;
    reg [19:0] cnt;
    reg        stable;
    reg        stable_d1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync      <= 2'b11;
            cnt       <= 20'd0;
            stable    <= 1'b1;
            stable_d1 <= 1'b1;
        end else begin
            sync <= {sync[0], key_in};
            if (sync[1] != stable) begin
                // 已验证方案: 同步后的电平变化立即生效。
                // 机械抖动会反复清零计数器, 连续稳定后计数器饱和。
                stable <= sync[1];
                cnt <= 20'd0;
            end else if (cnt < STABLE_CNT) begin
                cnt <= cnt + 1'b1;
            end
            stable_d1 <= stable;
        end
    end

    assign press_pulse = stable_d1 && !stable;   // 下降沿 = 按键按下

endmodule
