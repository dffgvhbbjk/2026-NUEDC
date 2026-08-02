// ============================================================================
// Module: wave_trigger
// Description: 基线估计 + noise_ema 自适应阈值触发判断 (冻结版方案 §4.2/§12.1)
//
// 功能:
//   - 一阶慢速基线: baseline += (sample - baseline) >>> 6 (单步钳位
//     ±BASE_STEP_MAX 防单点毛刺拉偏), 采集期间冻结
//     (base_en=0 仅在 CAPTURE: 触发确认样点会把基线抬高 ~幅值/32, 若 HOLD
//      继续冻结, 大冲击后残差 > rearm_thr 将永远无法满足静默判据 → 死锁;
//      HOLD 恢复跟踪让基线在静默期重新收敛, ~4ms << 85ms 静默期)
//   - 基线扣除: corrected = signed(sample) - baseline (25位, 防溢出)
//   - 绝对值扩大一位: magnitude = |corrected| (25位无符号)
//   - 噪声估计 (冻结版 §4.2): 不再用窗口最大值 (单个异常大点会把阈值抬高
//     约一个窗口, 造成轻敲无法触发), 改为绝对值慢速平均:
//       noise_ema += (magnitude - noise_ema) >>> EMA_SHIFT
//     保护规则:
//       * magnitude > threshold/2 (疑似冲击) 时暂停更新, 防锤击污染背景;
//       * track_en=0 (CAPTURE/HOLD) 时冻结噪声估计
//   - 阈值: threshold = clamp(noise_ema × K, TH_MIN, TH_MAX)
//       K = 6 (移位加法 ×4+×2); TH_MAX 保证阈值不会超过轻敲主冲击能力
//   - 触发条件: 连续 TRIG_CONFIRM 个样点 magnitude > threshold, 且重武装后
//     已观察到 SETTLE_POINTS 个连续静默点 (settled) 才允许触发
//   - 重新武装 (迟滞): magnitude < rearm_thr (=threshold/2, 独立信号,
//     便于单独验证/观测) 持续 REARM_POINTS 点 → rearm_ok 电平拉高
//
// 控制接口 (由 wave_capture 状态机驱动):
//   base_en:  1=更新基线 (IDLE/ARMED/HOLD), 0=冻结 (仅 CAPTURE)
//   track_en: 1=更新 noise_ema (IDLE/ARMED), 0=冻结 (CAPTURE/HOLD,
//             振铃尾不计入噪声统计, 避免抬高阈值)
//   arm_en:   1=允许输出 trigger (仅 ARMED 且预触发缓冲已填满)
//
// 流水线 (40MHz vga_clk, 样点间隔 ~417 周期, 时序余量充足):
//   Stage A (sample_valid@T):   corrected 寄存 + 基线更新
//   Stage B (T+1):              绝对值/阈值比较/触发/静默计数,
//                               corrected_data + corrected_valid + trigger 同拍输出
//
// SignalTap 观测: dbg_noise_ema / dbg_threshold / dbg_magnitude / dbg_baseline
//
// 时钟域: vga_clk 40MHz (输入来自 fifo_sample_reader 的对齐输出)
// ============================================================================
module wave_trigger #(
    // 350 帧实测回放: 500k 连续两点门限排除踩地面 50/50 帧，同时保留
    // 真实中等敲击 298/300 帧；上限 1M 仍远低于有效敲击主体幅值。
    parameter [23:0]  TH_MIN        = 24'd500000,
    parameter [23:0]  TH_MAX        = 24'd1000000,
    parameter integer EMA_SHIFT     = 6,          // noise_ema 时间常数 (÷64)
    parameter integer TRIG_CONFIRM  = 2,          // 连续超阈值点数
    parameter integer REARM_POINTS  = 8192,       // 静默重新武装点数 (~85ms)
    parameter integer SETTLE_POINTS = 16,         // 重武装后静默观察点数
    parameter signed [24:0] BASE_STEP_MAX = 25'sd4096  // 基线单步钳位 (毛刺防护)
) (
    input  wire               clk,             // vga_clk 40MHz
    input  wire               rst_n,           // 低有效异步复位

    input  wire [23:0]        sample_data,     // 原始采样 (24位补码)
    input  wire               sample_valid,    // 采样有效脉冲

    input  wire               base_en,         // 1=基线跟踪 (IDLE/ARMED/HOLD)
    input  wire               track_en,        // 1=噪声估计更新 (IDLE/ARMED)
    input  wire               arm_en,          // 1=允许触发输出 (ARMED)
    input  wire               clear_history,   // 强制/超时重武装时清触发与静默历史

    output reg  signed [23:0] corrected_data,  // 基线扣除后数据 (饱和到24位)
    output reg  signed [23:0] raw_data,        // 原始样点随动输出 (UART 导出用,
                                               //   与 corrected_data 同一样点同拍)
    output reg                corrected_valid, // 与 corrected_data 对齐
    output reg                trigger,         // 触发脉冲 (与 corrected_valid 同拍)
    output reg                rearm_ok,        // 静默持续达标, 可重新武装 (电平)

    // 调试观测 (SignalTap)
    output wire [23:0]        dbg_threshold,   // 当前自适应阈值
    output wire [23:0]        dbg_noise_ema,   // 噪声慢速平均
    output wire [23:0]        dbg_magnitude,   // 当前样点幅值
    output wire [23:0]        dbg_baseline     // 当前基线
);

    // ------------------------------------------------------------------------
    // 基线估计: baseline += clamp((sample - baseline) >>> 6, ±BASE_STEP_MAX)
    //   base_en=0 (仅 CAPTURE) 冻结, 避免主冲击拉偏记录中的波形零点;
    //   HOLD 期间恢复跟踪, 消除触发确认样点带来的基线残差 (否则死锁)
    //   单步钳位 (§12.1 单点毛刺不触发): 1.5M 毛刺不钳位时基线跳升
    //   1.5M/64≈23437 > TH_MIN, 后续静默样点残差连续超阈值 → 绕过
    //   TRIG_CONFIRM 假触发; 钳位后残差 ≤4096 << TH_MIN。真实漂移单步
    //   远小于 4096, 大冲击后收敛仍远快于 85ms 静默重武装, 无死锁风险
    // ------------------------------------------------------------------------
    reg  signed [23:0] baseline;
    wire signed [24:0] diff = {sample_data[23], sample_data} - {baseline[23], baseline};
    wire signed [24:0] base_step_raw = diff >>> 6;
    wire signed [24:0] base_step =
        (base_step_raw >  BASE_STEP_MAX) ?  BASE_STEP_MAX :
        (base_step_raw < -BASE_STEP_MAX) ? -BASE_STEP_MAX : base_step_raw;
    wire signed [24:0] baseline_next_w =
        $signed({baseline[23], baseline}) + base_step;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            baseline <= 24'sd0;
        else if (sample_valid && base_en)
            // 一阶跟踪为凸组合，结果处于输入/原基线之间；显式取低24位
            // 以表达寄存器位宽，避免工具产生隐式25→24位截断警告。
            baseline <= baseline_next_w[23:0];
    end

    assign dbg_baseline = baseline;

    // ------------------------------------------------------------------------
    // Stage A: 基线扣除结果寄存 (25位, 无溢出); 原始样点随动寄存
    // ------------------------------------------------------------------------
    reg signed [24:0] corr_r;
    reg signed [23:0] raw_r;
    reg               valid_a;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            corr_r  <= 25'sd0;
            raw_r   <= 24'sd0;
            valid_a <= 1'b0;
        end else begin
            valid_a <= sample_valid;
            if (sample_valid) begin
                corr_r <= diff;
                raw_r  <= sample_data;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 绝对值: 25位扩展避免最小负数溢出
    // ------------------------------------------------------------------------
    wire [24:0] magnitude = corr_r[24] ? (~corr_r + 1'b1) : corr_r[24:0];

    assign dbg_magnitude = (magnitude > 25'h0FFFFFF) ? 24'hFFFFFF
                                                     : magnitude[23:0];

    // ------------------------------------------------------------------------
    // 噪声慢速平均 (冻结版 §4.2): noise_ema += (magnitude - noise_ema) >>> 6
    //   疑似冲击 (magnitude > threshold/2) 期间暂停更新 → 锤击/碰线/单点
    //   尖峰不污染背景噪声估计, 阈值不会被一次冲击抬高一个窗口
    // ------------------------------------------------------------------------
    reg  [24:0] noise_ema;
    wire signed [26:0] ema_delta =
        ($signed({2'b00, magnitude}) - $signed({2'b00, noise_ema})) >>> EMA_SHIFT;
    wire signed [26:0] ema_next = $signed({2'b00, noise_ema}) + ema_delta;

    // threshold = clamp(noise_ema × 6, TH_MIN, TH_MAX): ×6 = ×4 + ×2 移位加法
    wire [27:0] thr_raw = {1'b0, noise_ema, 2'b00} + {2'b00, noise_ema, 1'b0};
    wire [23:0] threshold =
        (thr_raw >= {4'd0, TH_MAX}) ? TH_MAX :
        (thr_raw[23:0] < TH_MIN)    ? TH_MIN : thr_raw[23:0];

    // 疑似冲击: magnitude > threshold/2 (与静默判据同一比较, 独立命名观测)
    wire [23:0] rearm_thr     = {1'b0, threshold[23:1]};   // 静默/冻结门限
    wire        over_half     = (magnitude > {1'b0, rearm_thr});
    wire        quiet         = !over_half;                // mag ≤ threshold/2

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            noise_ema <= 25'd0;
        else if (valid_a && track_en && !over_half)
            noise_ema <= ema_next[24:0];
    end

    assign dbg_threshold = threshold;
    assign dbg_noise_ema = (noise_ema > 25'h0FFFFFF) ? 24'hFFFFFF
                                                     : noise_ema[23:0];

    // ------------------------------------------------------------------------
    // 触发判断: 连续 TRIG_CONFIRM 点 magnitude > threshold
    //   over_cnt 饱和计数, 仅在恰好达到 TRIG_CONFIRM 时输出单脉冲
    //   (持续超阈值不会重复触发, 直到 magnitude 回落清零计数)
    //   settled: 重武装后须先观察 SETTLE_POINTS 个连续静默点才允许触发
    //   (冻结版 §4.2 保护规则: 重新武装后先观察一小段静默)
    // ------------------------------------------------------------------------
    wire over = (magnitude > {1'b0, threshold});

    reg [3:0]  over_cnt;
    reg [13:0] quiet_cnt;
    reg [7:0]  settle_cnt;
    wire       settled = (settle_cnt >= SETTLE_POINTS[7:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            over_cnt   <= 4'd0;
            quiet_cnt  <= 14'd0;
            settle_cnt <= 8'd0;
            trigger    <= 1'b0;
            rearm_ok   <= 1'b0;
        end else if (clear_history) begin
            over_cnt   <= 4'd0;
            quiet_cnt  <= 14'd0;
            settle_cnt <= 8'd0;
            trigger    <= 1'b0;
            rearm_ok   <= 1'b0;
        end else begin
            trigger <= 1'b0;
            if (valid_a) begin
                // 重武装后静默观察 (连续 quiet 点计数, 达标后保持)
                if (!settled)
                    settle_cnt <= quiet ? settle_cnt + 1'b1 : 8'd0;

                // 超阈值确认计数
                if (over) begin
                    if (over_cnt != 4'hF)
                        over_cnt <= over_cnt + 1'b1;
                    if (arm_en && settled && (over_cnt == TRIG_CONFIRM - 1))
                        trigger <= 1'b1;
                end else begin
                    over_cnt <= 4'd0;
                end

                // 静默迟滞计数 (重新武装条件, 判据: mag ≤ rearm_thr)
                if (quiet) begin
                    if (quiet_cnt == REARM_POINTS - 1) begin
                        rearm_ok <= 1'b1;
                    end else begin
                        quiet_cnt <= quiet_cnt + 1'b1;
                        rearm_ok  <= 1'b0;
                    end
                end else begin
                    quiet_cnt <= 14'd0;
                    rearm_ok  <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Stage B 输出: corrected 饱和到 24 位, valid/trigger 同拍
    // ------------------------------------------------------------------------
    wire signed [23:0] corr_sat =
        (corr_r[24] != corr_r[23]) ? (corr_r[24] ? 24'sh800000 : 24'sh7FFFFF)
                                   : corr_r[23:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            corrected_data  <= 24'sd0;
            raw_data        <= 24'sd0;
            corrected_valid <= 1'b0;
        end else begin
            corrected_valid <= valid_a;
            if (valid_a) begin
                corrected_data <= corr_sat;
                raw_data       <= raw_r;
            end
        end
    end

endmodule
