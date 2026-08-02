// ============================================================================
// Module: fifo_sample_reader
// Description: DCFIFO 读端 q 与 valid 对齐模块 (文档 §9.3)
//
// 整改点 (对应文档 §3.3):
//   - 禁止使用 empty 下降沿作为样本有效信号
//   - valid 必须由实际成功的 FIFO 读操作产生 (rdreq && !empty)
//   - 根据 DCFIFO normal mode 输出延迟, 将成功读信号延迟后与 q 对齐
//   - [P1] 系统复位清空 DCFIFO: 任一域复位 → aclr 异步置位,
//         释放分别同步到 wrclk/rdclk, 释放前禁止 wrreq/rdreq
//   - [P2] wrreq 带 !wrfull 保护, 满时写入丢弃并由 overflow_cnt 记录
//
// DCFIFO 时序 (normal mode, lpm_showahead=OFF, 无输出寄存器):
//   - rdreq@T, empty=0 → q@T+1 = 数据
//   - empty@T+1 更新 (读最后一个数据时 empty 拉高)
//
// 对齐策略:
//   - rdreq 采用组合逻辑 (assign rdreq = !empty), 避免 registered rdreq 的下溢风险
//   - fire = rdreq & !empty (组合, 表示本周期发生了一次成功读)
//   - fire_d1 延迟 1 拍, 与 q@T+1 对齐
//   - sample_valid = fire_d1, sample_data = q (在 fire_d1 时锁存)
//
// 时钟域: 读侧 (rdclk = vga_clk 40MHz)
// ============================================================================
module fifo_sample_reader #(
    parameter DATA_WIDTH = 24,
    parameter ADDR_WIDTH = 8    // FIFO 深度 256 → 8 位地址
) (
    input  wire                    rdclk,         // 读时钟 (VGA 40MHz)
    input  wire                    rst_n,         // 读域异步复位, 低有效
    input  wire                    wr_rst_n,      // 写域异步复位, 低有效 (bclk 域)
    input  wire                    rd_en,         // 读使能 (1=有数据即自动读, 0=暂停读取)

    // DCFIFO 写侧 (BCLK 域, 透传到内部 audio_fifo)
    input  wire [DATA_WIDTH-1:0]   wr_data,       // 写数据
    input  wire                    wrreq,         // 写请求
    input  wire                    wrclk,         // 写时钟 (BCLK ~6.136MHz)

    // 对齐后的读侧输出
    output reg  [DATA_WIDTH-1:0]   sample_data,   // 读出数据 (与 sample_valid 对齐)
    output reg                     sample_valid,  // 数据有效脉冲 (每成功读一次, 拉高一次)

    // 状态/调试
    output wire                    fifo_full,     // FIFO 满 (写侧)
    output wire [ADDR_WIDTH-1:0]   fifo_level,    // FIFO 读侧余量
    output reg  [7:0]              overflow_cnt /* synthesis noprune */
                                                  // 写满丢弃计数 (wrclk 域, 饱和 255)
);

    // ------------------------------------------------------------------------
    // FIFO 复位策略 (整改 P1: 系统复位必须清空 DCFIFO)
    //   aclr: 任一域复位有效时异步置位, 清空读写指针与残留数据
    //   释放: 分别同步到 wrclk/rdclk (异步置位、同步释放),
    //         wr_rdy/rd_rdy 拉高前禁止 wrreq/rdreq,
    //         满足 DCFIFO 对 aclr 释放后读写恢复的跨域要求
    // ------------------------------------------------------------------------
    wire fifo_aclr = (~rst_n) | (~wr_rst_n);

    reg [1:0] wr_aclr_sync;
    always @(posedge wrclk or posedge fifo_aclr) begin
        if (fifo_aclr)
            wr_aclr_sync <= 2'b00;
        else
            wr_aclr_sync <= {wr_aclr_sync[0], 1'b1};
    end
    wire wr_rdy = wr_aclr_sync[1];

    reg [1:0] rd_aclr_sync;
    always @(posedge rdclk or posedge fifo_aclr) begin
        if (fifo_aclr)
            rd_aclr_sync <= 2'b00;
        else
            rd_aclr_sync <= {rd_aclr_sync[0], 1'b1};
    end
    wire rd_rdy = rd_aclr_sync[1];

    // ------------------------------------------------------------------------
    // 内部 DCFIFO 实例 (复用现有 audio_fifo)
    //   写请求带保护 (整改 P2): 复位释放前禁止写, 满时禁止写
    // ------------------------------------------------------------------------
    wire [DATA_WIDTH-1:0] fifo_q;
    wire                  fifo_empty;
    wire                  fifo_full_w;
    wire [ADDR_WIDTH-1:0] fifo_level_w;
    wire                  fifo_rdreq;     // 读请求 (组合: rd_en & rd_rdy & !empty)
    wire                  fifo_wrreq;     // 写请求 (带 full/复位保护)

    assign fifo_wrreq = wrreq & wr_rdy & ~fifo_full_w;

    audio_fifo u_adc_fifo (
        .aclr    (fifo_aclr),
        .data    (wr_data),
        .wrclk   (wrclk),
        .wrreq   (fifo_wrreq),
        .rdclk   (rdclk),
        .rdreq   (fifo_rdreq),
        .q       (fifo_q),
        .rdempty (fifo_empty),
        .wrfull  (fifo_full_w),
        .rdusedw (fifo_level_w),
        .wrusedw ()
    );

    assign fifo_full  = fifo_full_w;
    assign fifo_level = fifo_level_w;

    // ------------------------------------------------------------------------
    // 溢出计数 (整改 P2): 满时的写请求被丢弃, 不静默 — 计数供 SignalTap 观测
    //   正常工况读端远快于写端, 该计数应恒为 0; 非 0 即异常
    // ------------------------------------------------------------------------
    always @(posedge wrclk or negedge wr_rst_n) begin
        if (!wr_rst_n)
            overflow_cnt <= 8'd0;
        else if (wrreq && fifo_full_w && overflow_cnt != 8'hFF)
            overflow_cnt <= overflow_cnt + 1'b1;
    end

    // ------------------------------------------------------------------------
    // 读请求: 组合逻辑 (level mode)
    //   rdreq = rd_en & rd_rdy & !empty: 使能且 FIFO 有数据就读
    //   组合 rdreq 避免 registered rdreq 的 1 周期延迟导致的下溢问题
    //   rd_en=0 时真正暂停读取 (仿真可验证暂停/积压/恢复)
    // ------------------------------------------------------------------------
    assign fifo_rdreq = rd_en & rd_rdy & ~fifo_empty;

    // ------------------------------------------------------------------------
    // 成功读信号: 本周期确实读出了一个数据
    //   fire = rdreq & !empty (组合)
    //   DCFIFO normal mode: rdreq@T → q@T+1
    //   所以 fire@T 对应 q@T+1, 需延迟 1 拍对齐
    // ------------------------------------------------------------------------
    wire fifo_rd_fire;
    assign fifo_rd_fire = fifo_rdreq & ~fifo_empty;   // 等价于 ~fifo_empty

    reg fifo_rd_fire_d1;
    always @(posedge rdclk or negedge rst_n) begin
        if (!rst_n)
            fifo_rd_fire_d1 <= 1'b0;
        else
            fifo_rd_fire_d1 <= fifo_rd_fire;
    end

    // ------------------------------------------------------------------------
    // 输出对齐: sample_valid 与 sample_data 在同一周期有效
    //   fire_d1@T+1 = fire@T = 1 → q@T+1 = 本次读出的数据
    //   sample_valid@T+2 = fire_d1@T+1 = 1
    //   sample_data@T+2  = q@T+1 (在 fire_d1 时锁存)
    // ------------------------------------------------------------------------
    always @(posedge rdclk or negedge rst_n) begin
        if (!rst_n) begin
            sample_data  <= {DATA_WIDTH{1'b0}};
            sample_valid <= 1'b0;
        end else begin
            sample_valid <= fifo_rd_fire_d1;
            if (fifo_rd_fire_d1)
                sample_data <= fifo_q;
        end
    end

endmodule
