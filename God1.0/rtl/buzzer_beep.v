//======================================================================
// Module: buzzer_beep
// Function: 按键消抖 + 蜂鸣器控制
//   按下按键 → 蜂鸣器响, 松开 → 蜂鸣器停
//   内置 20ms 消抖
//======================================================================

module buzzer_beep (
    input  wire clk,           // 40MHz 时钟
    input  wire rst_n,         // 同步复位, 低有效
    input  wire btn,           // 按键输入 (按下→GND, 上拉→高)
    output reg  buzzer         // 蜂鸣器输出 (高电平触发)
);

//======================================================================
// 按键消抖 (20ms @ 40MHz = 800,000 周期)
//======================================================================
reg [1:0]  sync;
reg [19:0] cnt;
reg        stable;

always @(posedge clk) begin
    if (!rst_n) begin
        sync   <= 2'b11;
        cnt    <= 20'd0;
        stable <= 1'b1;
    end else begin
        sync <= {sync[0], btn};
        if (sync[1] != stable) begin
            cnt    <= 20'd0;
            stable <= sync[1];
        end else if (cnt < 20'd800_000) begin
            cnt <= cnt + 1'b1;
        end
    end
end

//======================================================================
// 蜂鸣器输出: 按下(stable=0) → 响, 松开(stable=1) → 停
//======================================================================
always @(posedge clk) begin
    if (!rst_n)
        buzzer <= 1'b0;
    else
        buzzer <= ~stable;
end

endmodule
