//======================================================================
// Module: gap_vote
// Function: 多行投票取众数, 输出稳定的缝隙像素宽度
//   收集 VOTE_LINES 个有效 gap_pix 样本 → 顺序比较求众数 → 输出
//   stable_valid 拉高条件: 众数计数 >= 60% × VOTE_LINES
//   顺序模式查找: 64×64 = 4096 周期 (170μs @24MHz, 远小于帧周期 33ms)
// Clock:  pclk 域
// 注: 方案原文 gap_detect 只在 detect_row 单行检测, 但为让投票每帧
//     收集到足够样本, gap_detect 改为对所有行检测, gap_vote 收集
//     每帧前 64 个有效样本. 这样每帧可输出一次稳定值.
//======================================================================
module gap_vote #(
    parameter VOTE_LINES = 64
) (
    input  wire        pclk,
    input  wire        rst_n,
    input  wire [9:0]  gap_pix,        // 来自 gap_detect
    input  wire        gap_valid,      // 来自 gap_detect
    output reg  [9:0]  gap_pix_stable, // 稳定的像素宽度
    output reg         stable_valid    // 接 uart_tx status bit1
);

    //------------------------------------------------------------------
    // 状态机
    //------------------------------------------------------------------
    localparam ST_COLLECT = 2'd0;   // 收集样本
    localparam ST_COMPUTE = 2'd1;   // 计算众数
    localparam ST_OUTPUT  = 2'd2;   // 输出结果

    reg [1:0]  state;
    reg [6:0]  wr_ptr;              // 写指针 0-63
    reg [6:0]  cmp_i;               // 外环 0-63
    reg [6:0]  cmp_j;               // 内环 0-63
    reg [6:0]  match_cnt;           // 匹配计数
    reg [6:0]  best_cnt;            // 最佳计数
    reg [9:0]  best_val;            // 最佳值

    //------------------------------------------------------------------
    // 样本缓冲 (64 × 10-bit, LE/MLAB 推断)
    //   双读端口: 同时读 samples[cmp_i] 和 samples[cmp_j]
    //------------------------------------------------------------------
    reg [9:0] samples [0:63];

    wire [9:0] sample_i = samples[cmp_i[5:0]];
    wire [9:0] sample_j = samples[cmp_j[5:0]];
    wire       match    = (sample_i == sample_j);

    //------------------------------------------------------------------
    // 主状态机
    //------------------------------------------------------------------
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_COLLECT;
            wr_ptr     <= 7'd0;
            cmp_i      <= 7'd0;
            cmp_j      <= 7'd0;
            match_cnt  <= 7'd0;
            best_cnt   <= 7'd0;
            best_val   <= 10'd0;
            gap_pix_stable <= 10'd0;
            stable_valid   <= 1'b0;
        end else begin
            stable_valid <= 1'b0;   // 默认低

            case (state)
                //----------------------------------------------------------
                // COLLECT: 收集 VOTE_LINES 个有效样本
                //----------------------------------------------------------
                ST_COLLECT: begin
                    if (gap_valid) begin
                        samples[wr_ptr[5:0]] <= gap_pix;
                        if (wr_ptr == VOTE_LINES - 1) begin
                            state     <= ST_COMPUTE;
                            cmp_i     <= 7'd0;
                            cmp_j     <= 7'd0;
                            match_cnt <= 7'd0;
                            best_cnt  <= 7'd0;
                            best_val  <= 10'd0;
                        end
                        wr_ptr <= wr_ptr + 1'b1;
                    end
                end

                //----------------------------------------------------------
                // COMPUTE: 顺序求众数
                //   对每个 i, 统计 samples[i] 在所有 samples[0..63] 中
                //   出现的次数. 取次数最大的为众数.
                //   match_cnt 在内环开始时清零, 内环结束时判最佳.
                //   注意非阻塞赋值: j=63 的 match 未计入 match_cnt,
                //   需要手动补加 (match ? 1 : 0).
                //----------------------------------------------------------
                ST_COMPUTE: begin
                    // 累加当前匹配
                    if (match)
                        match_cnt <= match_cnt + 1'b1;

                    cmp_j <= cmp_j + 1'b1;
                    if (cmp_j == 7'd63) begin
                        // 内环结束: 判最佳 (补加 j=63 的匹配)
                        cmp_j <= 7'd0;
                        match_cnt <= 7'd0;
                        cmp_i <= cmp_i + 1'b1;
                        if (match_cnt + (match ? 7'd1 : 7'd0) > best_cnt) begin
                            best_cnt <= match_cnt + (match ? 7'd1 : 7'd0);
                            best_val <= sample_i;
                        end
                        if (cmp_i == 7'd63) begin
                            state <= ST_OUTPUT;
                        end
                    end
                end

                //----------------------------------------------------------
                // OUTPUT: 输出众数, 判 stable_valid
                //   60% × 64 = 38.4 → 取 38
                //----------------------------------------------------------
                ST_OUTPUT: begin
                    gap_pix_stable <= best_val;
                    stable_valid   <= (best_cnt >= 7'd38);
                    wr_ptr         <= 7'd0;
                    state          <= ST_COLLECT;
                end

                default: state <= ST_COLLECT;
            endcase
        end
    end

endmodule
