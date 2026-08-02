// ============================================================================
// Module: defect_qiudao_classifier
// Description: 5点连续递减 + 方向翻转计数 缺陷分类器 (替代 defect_ac_classifier)
//
//   算法来源: jisuan.md / pc_tools/daima.txt (ADC_HUANCUN 模块)
//   已验证: Web 端 PC 移植版 (web_app.py analyze_samples) 对好棒/坏棒
//           批量扫描 20500 文件 (平衡准确率优化), 最优参数: 窗口[135,390] 阈值8
//           好棒 ~6.1 翻转, 坏棒 ~9.0 翻转, Fisher=1.55, 好棒85.1% 坏棒80.8% 平衡82.9%
//
//   算法流程:
//     1. 数据预处理: norm = sample / 128 + 200
//     2. 峰值扫描: 仅在 [0, PEAK_SCAN_LEN) 区段内找 |x| max → imp
//        (与 PC 端 argmax(|arr|[:100]) 一致; 棒底/多次回波振幅常大于主冲击,
//         扫全窗口会把 imp 错定位到 ~344 处 → 距离严重偏小)
//     3. 5点流水线: r1..r5 打拍, qiudao = (r1>r2 && r2>r3 && r3>r4 && r4>r5)
//     4. 翻转检测: edge = qiudao ^ qiudao_prev
//     5. 窗口[WIN_START, WIN_END]内累计翻转次数 win_edge_cnt
//     6. win_edge_cnt >= EDGE_THRESHOLD → DEFECT, else → NORMAL
//     7. B2反射位置测距: 冲击峰后第2个 qiudao 边沿 - 冲击峰位置 = reflection_pos
//     8. 距离: result = reflection_pos * DIST_COEFF_NUM >> DIST_COEFF_SHIFT
//        (235/16=14.6875≈14.67 mm/样点, 1m尼龙棒 96kHz 实测标定)
//     9. ★位宽陷阱: 乘积必须用 18 位中间量 (b2_dist_prod), result_dist 是 12 位,
//        直接乘会把 55*235=12925 截断成 637 → 距离严重偏小 (实测 39/142mm)
//
//   资源 (EP4CE6): ~300 LE + 1 M9K (影子 RAM), 处理 ~1500 拍 @40MHz ≈ 37.5μs
//
//   接口: 完全兼容 defect_ac_classifier, 直接替换
// ============================================================================
module defect_qiudao_classifier #(
    parameter [7:0] EDGE_THRESHOLD    = 8'd7,    // 缺陷翻转阈值 (平衡优化, 好棒85%+坏棒81%)
    parameter [9:0] WIN_START         = 10'd135,  // 分析窗口起始 (平衡优化, 原 120)
    parameter [9:0] WIN_END           = 10'd390,  // 分析窗口结束 (平衡优化, 原 512)
    parameter [7:0] DIST_COEFF_NUM    = 8'd235,   // 距离系数分子 (235/16=14.6875≈14.67 mm/样点)
    parameter [3:0] DIST_COEFF_SHIFT  = 4'd4,     // 距离系数右移位数 (/16)
    parameter [9:0] PEAK_SCAN_LEN     = 10'd100,  // 冲击峰搜索区段 (匹配 PC argmax[:100])
    parameter [9:0] EDGE_LATENCY      = 10'd3     // 边沿检测流水线延迟(拍): 补偿后与 PC 距离一致
) (
    input  wire               clk,            // vga_clk 40MHz
    input  wire               rst_n,          // 低有效

    // ── 影子 RAM 镜像写口 (与 wave_uart_export 同址同拍) ──
    input  wire               wr_en,
    input  wire [9:0]         wr_addr,        // {bank, addr[8:0]}
    input  wire signed [23:0] wr_raw,         // 原始 ADC 样点

    // ── 采集控制 ──
    input  wire               cap_done,       // 采集完成脉冲 (1 拍)
    input  wire [9:0]         disp_origin,    // {bank, origin} 帧起始地址
    input  wire [1:0]         disp_len_sel,   // 11 = 512 点帧
    input  wire               rearm,          // 重武装: 中止当前分析

    // ── 结果输出 ──
    output reg                result_valid,   // 1 拍脉冲: 分析完成
    output reg  [1:0]         result_state,   // 0=INVALID 1=NORMAL 2=DEFECT
    output reg  [1:0]         confidence,     // 0=NONE 1=LOW 2=HIGH
    output reg  [2:0]         defect_class,   // 0=good/N/A (本算法不细分缺陷距离)
    output reg  [11:0]        distance_mm,    // 缺陷距离 mm
    output reg  [8:0]         impact_index,   // 主冲击峰索引
    output reg                busy,           // 分析进行中
    output wire [3:0]         dbg_state       // FSM 状态 (SignalTap 可观测)
);

    // ========================================================================
    // 影子 RAM: 1024×24 (双 bank, 从 wave_capture 镜像写入)
    // ========================================================================
    (* ramstyle = "M9K" *) reg signed [23:0] shadow_ram [0:1023];
    reg  [9:0]         sh_rd_addr;
    reg  signed [23:0] sh_rd_q;

    always @(posedge clk) begin
        if (wr_en) shadow_ram[wr_addr] <= wr_raw;
    end
    always @(posedge clk) begin
        sh_rd_q <= shadow_ram[sh_rd_addr];
    end

    // ========================================================================
    // 状态机
    // ========================================================================
    localparam [3:0]
        S_IDLE  = 4'd0,
        S_PSCAN = 4'd1,   // 峰值扫描: 512 点找 |x| max → impact_index
        S_PREP  = 4'd2,   // 准备: 初始化流水线 + 预取第 0 点
        S_CHECK = 4'd3,   // 主处理: 5点流水线 → qiudao → 边沿计数 → 判定
        S_DONE  = 4'd4;   // 输出结果

    reg [3:0] state;
    assign dbg_state = state;

    // ========================================================================
    // 帧参数锁存
    // ========================================================================
    reg        f_bank;
    reg [8:0]  f_org;

    // ========================================================================
    // 峰值扫描
    // ========================================================================
    reg [9:0]  ps_cnt;        // 0..512
    reg [23:0] peak_mag;      // 当前最大绝对值
    reg [9:0]  imp;           // 冲击峰索引

    wire signed [23:0] ps_sample = sh_rd_q;
    wire [23:0] ps_abs = ps_sample[23] ? (~ps_sample + 1'b1) : ps_sample;

    // ========================================================================
    // 5 点流水线 + qiudao_panjue
    // ========================================================================
    reg [9:0]  chk_cnt;       // 当前样点索引 0..512
    reg signed [16:0] norm_data;  // 归一化: sample/128 + 200
    reg signed [16:0] r1, r2, r3, r4, r5;
    reg        qiudao_panjue;
    reg        qiudao_prev;   // 上一拍的 qiudao
    reg        qiudao_valid;  // 流水线已填满 (chk_cnt >= 4)
    wire       qiudao_edge;   // 边沿检测

    // qiudao 组合逻辑: 5 点严格递减
    wire qiudao_next = (r1 > r2) && (r2 > r3) && (r3 > r4) && (r4 > r5);
    assign qiudao_edge = qiudao_valid && (qiudao_panjue ^ qiudao_prev);

    // ========================================================================
    // 边沿计数 + 窗口内判定
    // ========================================================================
    reg [7:0]  win_edge_cnt;      // 窗口内累计翻转次数
    reg        win_edge_cnt_done; // 窗口扫描完成标志
    reg [7:0]  global_edge_cnt;   // 全局翻转次数 (跨全帧)

    // ========================================================================
    // B2 反射位置测距 (冲击峰后第2个 qiudao 边沿)
    // ========================================================================
    reg [1:0]  edge_after_cnt;    // 冲击峰后的边沿计数 (0,1,2)
    reg [9:0]  e1_after_pos;      // 冲击峰后第1个边沿位置
    reg [9:0]  e2_after_pos;      // 冲击峰后第2个边沿位置 (B2反射)

    // 距离乘积: 必须用 18 位宽中间量! result_dist 是 12 位,
    // 直接写 ((chk-imp-3)*DIST_COEFF_NUM)>>4 会让乘法在 12 位上下文
    // 中计算, 55*235=12925 被截断成 637 → 距离严重偏小 (实测 39/142mm)
    wire [17:0] b2_dist_prod = (chk_cnt - imp - EDGE_LATENCY) * DIST_COEFF_NUM;

    // ========================================================================
    // 距离结果锁存
    // ========================================================================
    reg [11:0] result_dist;
    reg        result_valid_i;

    // ========================================================================
    // 主 FSM
    // ========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            busy              <= 1'b0;
            result_valid      <= 1'b0;
            result_state      <= 2'd0;
            confidence        <= 2'd0;
            defect_class      <= 3'd0;
            distance_mm       <= 12'd0;
            impact_index      <= 9'd0;
            ps_cnt            <= 10'd0;
            peak_mag          <= 24'd0;
            imp               <= 10'd0;
            chk_cnt           <= 10'd0;
            norm_data         <= 17'd0;
            r1 <= 17'd0; r2 <= 17'd0; r3 <= 17'd0; r4 <= 17'd0; r5 <= 17'd0;
            qiudao_panjue     <= 1'b0;
            qiudao_prev       <= 1'b0;
            qiudao_valid      <= 1'b0;
            win_edge_cnt      <= 8'd0;
            win_edge_cnt_done <= 1'b0;
            global_edge_cnt   <= 8'd0;
            edge_after_cnt    <= 2'd0;
            e1_after_pos      <= 10'd0;
            e2_after_pos      <= 10'd0;
            result_dist       <= 12'd0;
            result_valid_i    <= 1'b0;
        end else begin
            result_valid <= 1'b0;

            // 分析进行中遇重武装 → 中止
            if (rearm && state != S_IDLE) begin
                state <= S_IDLE;
                busy  <= 1'b0;
            end else begin
            case (state)

                // ---- 空闲: 等 512 点采集完成 ----
                S_IDLE: begin
                    busy <= 1'b0;
                    if (cap_done && (disp_len_sel == 2'b11)) begin
                        f_bank     <= disp_origin[9];
                        f_org      <= disp_origin[8:0];
                        ps_cnt     <= 10'd0;
                        peak_mag   <= 24'd0;
                        imp        <= 10'd0;
                        sh_rd_addr <= {disp_origin[9], disp_origin[8:0]}; // pt 0
                        busy       <= 1'b1;
                        state      <= S_PSCAN;
                    end
                end

                // ---- 峰值扫描: 仅 [0, PEAK_SCAN_LEN) 区段内找 |x| 最大值位置 ----
                //   注意: 全窗口扫描会把棒底/多次回波 (振幅常大于主冲击) 当成峰值,
                //   imp 错定位到 ~344 → chk_cnt-imp 偏小 → 距离严重偏小 (实测 155mm)
                S_PSCAN: begin
                    if (ps_cnt < 10'd512)
                        sh_rd_addr <= {f_bank, f_org + ps_cnt[8:0]};
                    if (ps_cnt >= 10'd1) begin
                        // ps_abs = |sample[ps_cnt-1]|, 仅扫描 PEAK_SCAN_LEN 个样点
                        if ((ps_cnt - 10'd1) < PEAK_SCAN_LEN) begin
                            if (ps_abs > peak_mag) begin
                                peak_mag <= ps_abs;
                                imp      <= ps_cnt - 10'd1;
                            end
                        end
                    end
                    if (ps_cnt == 10'd512) begin
                        // 扫描完成, 进入主处理阶段
                        // 预取第 0 点
                        sh_rd_addr <= {f_bank, f_org};
                        chk_cnt    <= 10'd0;
                        // 重置流水线
                        r1 <= 17'd0; r2 <= 17'd0; r3 <= 17'd0; r4 <= 17'd0; r5 <= 17'd0;
                        qiudao_panjue <= 1'b0;
                        qiudao_prev   <= 1'b0;
                        qiudao_valid  <= 1'b0;
                        win_edge_cnt    <= 8'd0;
                        win_edge_cnt_done <= 1'b0;
                        global_edge_cnt  <= 8'd0;
                        edge_after_cnt    <= 2'd0;
                        e1_after_pos      <= 10'd0;
                        e2_after_pos      <= 10'd0;
                        result_dist   <= 12'd0;
                        result_valid_i <= 1'b0;
                        state <= S_PREP;
                    end else begin
                        ps_cnt <= ps_cnt + 10'd1;
                    end
                end

                // ---- 准备: 等 RAM 读数据稳定 + 预取 ----
                S_PREP: begin
                    // sh_rd_q 现在有第 0 点
                    // 归一化: sample / 128 + 200
                    norm_data <= (sh_rd_q / 24'sd128) + 17'sd200;
                    // 预取第 1 点
                    sh_rd_addr <= {f_bank, f_org + 9'd1};
                    chk_cnt    <= 10'd1;  // 下一个要处理的 index
                    state      <= S_CHECK;
                end

                // ---- 主处理: 每拍一个样点, 推进流水线 ----
                S_CHECK: begin
                    // sh_rd_q = 样点 chk_cnt-1 (上一拍预取的)
                    // norm_data = 样点 chk_cnt-2 的归一化值

                    // ── 推进 5 点流水线 ──
                    r5 <= r4;
                    r4 <= r3;
                    r3 <= r2;
                    r2 <= r1;
                    r1 <= norm_data;
                    qiudao_prev   <= (chk_cnt == 10'd5) ? qiudao_next : qiudao_panjue;
                    qiudao_panjue <= qiudao_next;
                    if (chk_cnt >= 10'd5)
                        qiudao_valid <= 1'b1;

                    // ── 边沿检测 + 计数 ──
                    if (qiudao_valid && qiudao_edge) begin
                        // 全局翻转计数
                        global_edge_cnt <= global_edge_cnt + 8'd1;

                        // 窗口内累积
                        if (!win_edge_cnt_done && (chk_cnt >= WIN_START) && (chk_cnt <= WIN_END)) begin
                            win_edge_cnt <= win_edge_cnt + 8'd1;
                        end

                        // B2反射位置: 追踪冲击峰后的边沿
                        if (chk_cnt > imp) begin
                            if (edge_after_cnt == 2'd0) begin
                                e1_after_pos <= chk_cnt;
                            end else if (edge_after_cnt == 2'd1 && !result_valid_i) begin
                                e2_after_pos <= chk_cnt;
                                // 反射位置 × 系数 → 距离 (235/16≈14.67 mm/样点)
                                // 减 EDGE_LATENCY: 边沿检测有 +3 拍流水线延迟,
                                // 不补偿则 FPGA 距离恒比 PC 高 ~44mm
                                if (chk_cnt >= imp + EDGE_LATENCY) begin
                                    result_dist <= b2_dist_prod >> DIST_COEFF_SHIFT;
                                end else begin
                                    result_dist <= 12'd0;
                                end
                                result_valid_i <= 1'b1;
                            end
                            edge_after_cnt <= edge_after_cnt + 2'd1;
                        end
                    end

                    // 距离已在边沿检测块中计算 (B2反射位置法, 冲击峰后第2边沿)

                    // ── 当前样点归一化 (用于下一拍流水线) ──
                    norm_data <= (sh_rd_q / 24'sd128) + 17'sd200;

                    // 推进或结束
                    if (chk_cnt == 10'd512) begin
                        // 窗口边界: WIN_END = 412
                        // 已经扫描完 512 点, 进入判定
                        state <= S_DONE;
                    end else begin
                        // 预取下一个样点
                        if (chk_cnt < 10'd511)
                            sh_rd_addr <= {f_bank, f_org + chk_cnt[8:0] + 9'd1};
                        chk_cnt <= chk_cnt + 10'd1;
                    end
                end

                // ---- 输出结果 ----
                S_DONE: begin
                    impact_index <= imp[8:0];
                    defect_class <= 3'd0;  // 本算法不细分缺陷类别

                    if (peak_mag < 24'd300000) begin
                        // 敲击太弱 → INVALID
                        result_state  <= 2'd0;
                        confidence    <= 2'd0;
                        distance_mm   <= 12'd0;
                    end else if (win_edge_cnt >= EDGE_THRESHOLD) begin
                        // 缺陷判定
                        result_state  <= 2'd2;   // DEFECT
                        confidence    <= (win_edge_cnt >= EDGE_THRESHOLD + 2) ? 2'd2 : 2'd1;
                        distance_mm   <= result_dist;
                    end else begin
                        // 好桩
                        result_state  <= 2'd1;   // NORMAL
                        confidence    <= (win_edge_cnt <= EDGE_THRESHOLD - 3) ? 2'd2 : 2'd1;
                        distance_mm   <= 12'd0;
                    end

                    result_valid <= 1'b1;
                    busy         <= 1'b0;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
            end
        end
    end

endmodule
