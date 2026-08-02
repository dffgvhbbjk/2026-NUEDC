// ============================================================================
// Testbench: tb_wave_trigger
// 对象: wave_trigger (冻结版方案 §4.2/§12.1 noise_ema 自适应阈值)
//
// 测试项:
// 本 TB 用 20k/120k 参数覆盖专测 EMA 算法细节；硬件默认门限已按
// 350 帧实测数据改为 500k/1M，并由离线真实波形回放验证。
//   T1 零输入: noise_ema=0, threshold=TH_MIN(20000) 下限钳位,
//      64 静默点后 rearm_ok=1 (REARM_POINTS=64 加速)
//   T2 ±8000 交替噪声 192 点 (≈3τ): ema≈7600 → threshold≈45.6k ∈ [40k,48k]
//   T3 track_en=0 冻结: ±16000 噪声 32 点, ema/threshold 不变
//   T4 over_half 冻结: ±30000 (mag×2 > threshold) 20 点, ema 不变,
//      rearm_ok 掉 0 (非静默复位迟滞计数)
//   T5 静默衰减: 64 零点 → ema 衰减 ≈×0.37, threshold 回落钳位 TH_MIN,
//      rearm_ok 重新拉高
//   T6 单点尖峰不触发 (TRIG_CONFIRM=2): 1×250000 + 静默 → 无触发
//   T7 连续 5 点超阈值 → 恰好 1 个触发脉冲 (over_cnt 饱和不重复触发)
//   T8 TH_MAX 钳位: ±10000 抬 ema 后 ±25000 持续 → threshold=120000
//   T9 settled 保护: clear_history 后立即冲击不触发, 16 静默点后恢复触发
//   全程: trigger 与 corrected_valid 同拍; raw_data 随动 = 原始样点
//
// 冲击段 base_en=0 (复刻 wave_capture CAPTURE 冻结), 防基线被冲击拉偏
// ============================================================================
`timescale 1ns/1ps

module tb_wave_trigger;

    reg clk, rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;        // 40MHz

    localparam SAMPLE_GAP = 20;     // 样点间隔 (clk), 流水线仅 2 拍, 余量充足

    reg  signed [23:0] sample_data;
    reg                sample_valid;
    reg                base_en, track_en, arm_en, clear_history;
    wire signed [23:0] corrected_data, raw_data;
    wire               corrected_valid, trigger, rearm_ok;
    wire [23:0]        dbg_threshold, dbg_noise_ema, dbg_magnitude, dbg_baseline;

    wave_trigger #(
        .TH_MIN        (24'd20000),
        .TH_MAX        (24'd120000),
        .REARM_POINTS (64)          // 加速: 硬件 8192 (~85ms)
    ) u_dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .sample_data     (sample_data),
        .sample_valid    (sample_valid),
        .base_en         (base_en),
        .track_en        (track_en),
        .arm_en          (arm_en),
        .clear_history   (clear_history),
        .corrected_data  (corrected_data),
        .raw_data        (raw_data),
        .corrected_valid (corrected_valid),
        .trigger         (trigger),
        .rearm_ok        (rearm_ok),
        .dbg_threshold   (dbg_threshold),
        .dbg_noise_ema   (dbg_noise_ema),
        .dbg_magnitude   (dbg_magnitude),
        .dbg_baseline    (dbg_baseline)
    );

    // ------------------------------------------------------------------------
    // 触发计数 + 对齐/随动监视
    // ------------------------------------------------------------------------
    integer errors, trig_count, align_err, raw_err;
    reg signed [23:0] exp_raw;      // 最近发送样点 (间隔 >> 流水线深度)

    always @(posedge clk) begin
        if (rst_n && trigger) begin
            trig_count = trig_count + 1;
            if (!corrected_valid) begin
                align_err = align_err + 1;
                $display("[%0t] FAIL: trigger 未与 corrected_valid 同拍", $time);
            end
        end
        if (rst_n && corrected_valid && raw_data !== exp_raw) begin
            raw_err = raw_err + 1;
            if (raw_err < 5)
                $display("[%0t] FAIL: raw_data=%0d, 期望 %0d",
                         $time, raw_data, exp_raw);
        end
    end

    // ------------------------------------------------------------------------
    // 激励任务
    // ------------------------------------------------------------------------
    task send_sample(input signed [23:0] val);
        begin
            @(negedge clk);
            sample_data  = val;
            sample_valid = 1'b1;
            exp_raw      = val;
            @(negedge clk);
            sample_valid = 1'b0;
            repeat (SAMPLE_GAP) @(posedge clk);
        end
    endtask

    // 交替 ±amp 噪声 n 点
    integer bi;
    task send_alt(input integer n, input signed [23:0] amp);
        begin
            for (bi = 0; bi < n; bi = bi + 1)
                send_sample(bi[0] ? -amp : amp);
        end
    endtask

    task send_zeros(input integer n);
        begin
            for (bi = 0; bi < n; bi = bi + 1)
                send_sample(24'sd0);
        end
    endtask

    // ------------------------------------------------------------------------
    // 校验任务
    // ------------------------------------------------------------------------
    task check_eq24(input [8*24-1:0] name, input [23:0] got, input [23:0] exp);
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("  [FAIL] %0s: got %0d, expect %0d", name, got, exp);
            end else
                $display("  [PASS] %0s = %0d", name, got);
        end
    endtask

    task check_range(input [8*24-1:0] name, input [23:0] got,
                     input [23:0] lo, input [23:0] hi);
        begin
            if (got < lo || got > hi) begin
                errors = errors + 1;
                $display("  [FAIL] %0s: got %0d, expect [%0d,%0d]",
                         name, got, lo, hi);
            end else
                $display("  [PASS] %0s = %0d (in [%0d,%0d])", name, got, lo, hi);
        end
    endtask

    task check_bit(input [8*24-1:0] name, input got, input exp);
        begin
            if (got !== exp) begin
                errors = errors + 1;
                $display("  [FAIL] %0s: got %b, expect %b", name, got, exp);
            end else
                $display("  [PASS] %0s = %b", name, got);
        end
    endtask

    task check_trig(input [8*24-1:0] name, input integer exp);
        begin
            if (trig_count !== exp) begin
                errors = errors + 1;
                $display("  [FAIL] %0s: trig_count=%0d, expect %0d",
                         name, trig_count, exp);
            end else
                $display("  [PASS] %0s: trig_count = %0d", name, trig_count);
        end
    endtask

    // ------------------------------------------------------------------------
    // 主流程
    // ------------------------------------------------------------------------
    reg [23:0] ema_hold;            // 冻结检查基准

    initial begin
        rst_n         = 1'b0;
        sample_data   = 24'sd0;
        sample_valid  = 1'b0;
        base_en       = 1'b1;
        track_en      = 1'b1;
        arm_en        = 1'b0;
        clear_history = 1'b0;
        exp_raw       = 24'sd0;
        errors        = 0;
        trig_count    = 0;
        align_err     = 0;
        raw_err       = 0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // ============ T1: 零输入 → TH_MIN 钳位 + rearm_ok ============
        $display("=== T1: 零输入 (TH_MIN 钳位 / rearm_ok) ===");
        send_zeros(80);                          // > REARM_POINTS=64
        check_eq24("T1 noise_ema",  dbg_noise_ema, 24'd0);
        check_eq24("T1 threshold",  dbg_threshold, 24'd20000);
        check_bit ("T1 rearm_ok",   rearm_ok, 1'b1);

        // ============ T2: ±8000 噪声收敛 (192 点 ≈ 3τ) ============
        // ema ≈ 8000×(1-e^-3) ≈ 7600 → threshold ≈ 45613
        $display("=== T2: +/-8000 噪声 192 点 (EMA 收敛) ===");
        send_alt(192, 24'sd8000);
        check_range("T2 threshold", dbg_threshold, 24'd40000, 24'd48000);
        check_bit  ("T2 rearm_ok",  rearm_ok, 1'b1);  // 16000 < thr/2, 持续静默

        // ============ T3: track_en=0 → ema 冻结 ============
        $display("=== T3: track_en=0 冻结 (+/-16000 x32) ===");
        ema_hold = dbg_noise_ema;
        track_en = 1'b0;
        send_alt(32, 24'sd16000);                // 32000 < thr, 本应更新但被门控
        check_eq24("T3 ema frozen", dbg_noise_ema, ema_hold);
        track_en = 1'b1;

        // ============ T4: over_half → ema 冻结 (防冲击污染) ============
        $display("=== T4: over_half 冻结 (+/-30000 x20) ===");
        ema_hold = dbg_noise_ema;
        send_alt(20, 24'sd30000);                // 60000 > thr≈45.6k → 暂停更新
        check_eq24("T4 ema frozen", dbg_noise_ema, ema_hold);
        check_bit ("T4 rearm_ok",   rearm_ok, 1'b0);  // 非静默复位迟滞

        // ============ T5: 静默衰减 → 阈值回落 TH_MIN + 重新武装 ============
        // 64 零点 = 1τ: ema ≈ 7600×0.37 ≈ 2800 → 6×ema < 20000 → 钳位
        $display("=== T5: 64 零点衰减 (阈值回 TH_MIN / rearm 恢复) ===");
        send_zeros(64);
        check_eq24("T5 threshold",  dbg_threshold, 24'd20000);
        check_bit ("T5 rearm_ok",   rearm_ok, 1'b1);

        // ============ T6: 单点尖峰不触发 (TRIG_CONFIRM=2) ============
        $display("=== T6: 单点尖峰 250000 (无触发) ===");
        arm_en  = 1'b1;
        base_en = 1'b0;                          // 复刻 CAPTURE 基线冻结
        send_sample(24'sd250000);
        send_zeros(4);
        check_trig("T6 no trigger", 0);

        // ============ T7: 连续 5 点超阈值 → 恰 1 个脉冲 ============
        $display("=== T7: 连续 5 点 250000 (单脉冲) ===");
        repeat (5) send_sample(24'sd250000);
        send_zeros(4);
        check_trig("T7 one trigger", 1);

        // ============ T8: TH_MAX 钳位 ============
        // ±9000 抬 ema (18000<TH_MIN 有余量, ±10000 会被基线纹波推过
        // over_half 边界导致冻结) → thr≈52k, 再 ±25000 (50000<thr 持续
        // 更新) → 6×ema > 120k 钳位
        $display("=== T8: TH_MAX 钳位 (threshold=120000) ===");
        arm_en  = 1'b0;
        base_en = 1'b1;
        send_alt(192, 24'sd9000);
        check_range("T8 mid thr",   dbg_threshold, 24'd46000, 24'd56000);
        send_alt(256, 24'sd25000);
        check_eq24("T8 threshold",  dbg_threshold, 24'd120000);

        // ============ T9: clear_history → settled 保护 ============
        $display("=== T9: clear_history 后 settled 保护 ===");
        @(negedge clk); clear_history = 1'b1;
        @(negedge clk); clear_history = 1'b0;
        check_bit("T9 rearm clear", rearm_ok, 1'b0);
        arm_en  = 1'b1;
        base_en = 1'b0;
        repeat (5) send_sample(24'sd500000);     // 未静默观察 → 不许触发
        check_trig("T9 blocked", 1);
        send_zeros(16);                          // SETTLE_POINTS=16 静默观察
        repeat (2) send_sample(24'sd500000);     // 恢复触发能力
        send_zeros(2);
        check_trig("T9 re-trigger", 2);

        // ============ 汇总 ============
        if (align_err != 0) errors = errors + align_err;
        if (raw_err   != 0) errors = errors + raw_err;
        if (errors == 0)
            $display("=== tb_wave_trigger: ALL PASS ===");
        else
            $display("=== tb_wave_trigger: %0d FAIL ===", errors);
        $finish;
    end

endmodule
