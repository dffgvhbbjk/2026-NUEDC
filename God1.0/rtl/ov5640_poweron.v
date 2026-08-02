//======================================================================
// Module: ov5640_poweron
// Function: OV5640 严格上电时序控制
//   依据 OV5640 datasheet 要求:
//     t2: DVDD 稳定后保持 PWDN=1, RST_N=0 至少 5ms
//     t3: PWDN 拉低后等至少 1ms 再拉高 RST_N
//     t4: RST_N 拉高后等至少 20ms 才能开始 SCCB 配置
//   本模块用 50MHz 时钟实现 (留余量):
//     6ms (PWDN 保持) + 2ms (RST_N 延迟) + 21ms (SCCB 延迟) = 29ms
//
//   输出 poweron_done 在 29ms 后拉高, 用于门控 SCCB 配置模块
//
// Clock: 50MHz 系统时钟
//======================================================================
module ov5640_poweron (
    input  wire        clk,           // 50MHz
    input  wire        rst_n,         // 系统复位 (异步, 低有效)
    input  wire        pll_locked,    // PLL 锁定信号 (XCLK 稳定标志)
    output reg         cam_pwdn,      // OV5640 PWDN 引脚 (高有效)
    output reg         cam_rst_n,     // OV5640 RST_N 引脚 (低有效)
    output reg         poweron_done   // 上电完成, 允许 SCCB 配置
);

    //------------------------------------------------------------------
    // 计数器 (50MHz = 20ns/周期)
    //   总计 29ms = 1,450,000 周期, 用 21 位计数器
    //   阶段划分 (留余量):
    //     [0, 300000)      = 6ms:  保持 PWDN=1, RST_N=0
    //     [300000, 400000)  = 2ms:  PWDN=0, RST_N=0
    //     [400000, 1450000) = 21ms: PWDN=0, RST_N=1, 等 SCCB
    //     >= 1450000:              poweron_done=1
    //------------------------------------------------------------------
    localparam [20:0] T2_END = 21'd300000;       // 6ms @50MHz
    localparam [20:0] T3_END = 21'd400000;       // 6ms+2ms = 8ms @50MHz
    localparam [20:0] T4_END = 21'd1450000;      // 6ms+2ms+21ms = 29ms @50MHz

    reg [20:0] cnt;

    //------------------------------------------------------------------
    // 计数器: 只在 PLL 锁定后才开始计数
    //   保证 XCLK 稳定后才开始上电时序
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 21'd0;
        end else if (pll_locked) begin
            if (cnt < T4_END)
                cnt <= cnt + 1'b1;
            // 达到 T4_END 后停止计数 (省电)
        end
    end

    //------------------------------------------------------------------
    // PWDN: 上电时=1, 6ms 后拉低
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cam_pwdn <= 1'b1;          // 复位时 PWDN=1 (掉电)
        else if (pll_locked && cnt >= T2_END)
            cam_pwdn <= 1'b0;          // 6ms 后 PWDN=0 (正常)
    end

    //------------------------------------------------------------------
    // RST_N: 上电时=0, 8ms 后拉高
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cam_rst_n <= 1'b0;         // 复位时 RST_N=0
        else if (pll_locked && cnt >= T3_END)
            cam_rst_n <= 1'b1;         // 8ms 后 RST_N=1
    end

    //------------------------------------------------------------------
    // poweron_done: 29ms 后拉高, 允许 SCCB 配置
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            poweron_done <= 1'b0;
        else if (cnt >= T4_END)
            poweron_done <= 1'b1;
    end

endmodule
