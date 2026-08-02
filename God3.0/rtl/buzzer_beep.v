//======================================================================
// Module: buzzer_beep
// Function: 检测结果提示音
//   btn 保持高电平时关闭手动测试;
//   NORMAL 响一声, DEFECT 响两声;
//   每声 100ms, 两声之间间隔 50ms.
//======================================================================

module buzzer_beep (
    input  wire clk,             // 40MHz 时钟
    input  wire rst_n,           // 异步复位, 低有效
    input  wire btn,             // 低有效测试键（当前顶层固定为高电平）
    input  wire normal_pulse,    // NORMAL 结果单周期脉冲
    input  wire defect_pulse,    // DEFECT 结果单周期脉冲
    output reg  buzzer           // 蜂鸣器输出 (高电平触发)
);

localparam [22:0] BEEP_CYCLES = 23'd4_000_000; // 100ms @ 40MHz
localparam [22:0] GAP_CYCLES  = 23'd2_000_000; //  50ms @ 40MHz

localparam [1:0] ST_IDLE = 2'd0;
localparam [1:0] ST_BEEP = 2'd1;
localparam [1:0] ST_GAP  = 2'd2;

wire test_pulse;
key_debounce u_test_key (
    .clk         (clk),
    .rst_n       (rst_n),
    .key_in      (btn),
    .press_pulse (test_pulse)
);

reg [1:0]  state;
reg [22:0] timer;
reg [1:0]  beeps_left;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= ST_IDLE;
        timer      <= 23'd0;
        beeps_left <= 2'd0;
    end else begin
        case (state)
            ST_IDLE: begin
                // DEFECT 优先于 NORMAL；手动测试输入当前由顶层关闭。
                if (defect_pulse) begin
                    state      <= ST_BEEP;
                    timer      <= BEEP_CYCLES - 1'b1;
                    beeps_left <= 2'd2;
                end else if (normal_pulse || test_pulse) begin
                    state      <= ST_BEEP;
                    timer      <= BEEP_CYCLES - 1'b1;
                    beeps_left <= 2'd1;
                end
            end

            ST_BEEP: begin
                if (timer == 23'd0) begin
                    if (beeps_left > 2'd1) begin
                        state      <= ST_GAP;
                        timer      <= GAP_CYCLES - 1'b1;
                        beeps_left <= beeps_left - 1'b1;
                    end else begin
                        state      <= ST_IDLE;
                        beeps_left <= 2'd0;
                    end
                end else begin
                    timer <= timer - 1'b1;
                end
            end

            ST_GAP: begin
                if (timer == 23'd0) begin
                    state <= ST_BEEP;
                    timer <= BEEP_CYCLES - 1'b1;
                end else begin
                    timer <= timer - 1'b1;
                end
            end

            default: begin
                state      <= ST_IDLE;
                timer      <= 23'd0;
                beeps_left <= 2'd0;
            end
        endcase
    end
end

always @(*) begin
    buzzer = (state == ST_BEEP);
end

endmodule
