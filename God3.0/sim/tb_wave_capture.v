`timescale 1ns / 1ps
// ============================================================================
// Testbench: tb_wave_capture
// 对象: wave_trigger + wave_capture + wave_buffer_dp 采集链 (文档 §17.3)
//
// 测试项 (文档 §17.3 采集状态机仿真):
//   1. 64 点:  未触发时不进入 HOLD; 触发后采集准确点数; 预触发 8 点正确;
//              RAM 内容与影子模型逐点比对; HOLD 后 RAM 不再被覆盖
//   2. 128 点: 负向冲击 (验证绝对值触发), RAM 逐点比对, bank 翻转
//   3. 256 点: RAM 逐点比对, bank 再翻转 (乒乓回 bank0)
//   4. 自适应阈值 (冻结版 noise_ema): 噪声 ±8000 → noise_ema≈8000 →
//              threshold≈×6≈45000; 30000 冲击 (>TH_MIN 但 <阈值) 不触发,
//              300000 冲击正常触发
//   5. force_rearm: HOLD 中按键 → display_en=0, 状态回 ARMED
//   6. 预触发窗口孤立尖峰计入自动增益 (完整 N 点窗口峰值, 修复回归)
//   7. HOLD 持续非静默时超时退出，并可再次响应敲击 (卡死修复回归)
//
// 影子模型: TB 复刻基线一阶迭代 baseline += clamp(corr >>> 6, ±4096), 精确预测
//   corrected_data 与 RAM[origin+k] == hist[trig_idx-pre+k][23:8]
//
// 仿真加速参数: REARM_POINTS=64, WARMUP_POINTS=64,
//   采样间隔压缩为 100 clk (真实 ~417 clk, 逻辑与间隔无关)
//   wave_trigger 显式覆盖旧门限 20k/120k，以继续验证采集状态机的小幅
//   合成激励；硬件默认 500k/1M 由触发/全链路测试覆盖。
// ============================================================================
module tb_wave_capture;

    // ---- 时钟/复位 ----
    reg clk;
    reg rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;    // 40MHz

    // ---- TB 参数 ----
    localparam SAMPLE_GAP    = 100;   // 样点间隔 (clk)
    localparam TB_NOISE_PTS  = 192;   // 噪声注入点数 (~3×EMA 时间常数)
    localparam TB_REARM_PTS  = 64;
    localparam TB_WARMUP     = 64;

    // ---- 链路互连 ----
    reg  signed [23:0] sample_data;
    reg                sample_valid;
    wire               base_en, track_en, arm_en;
    wire signed [23:0] corrected_data;
    wire               corrected_valid;
    wire               trigger;
    wire               rearm_ok;
    wire [23:0]        dbg_threshold, dbg_noise_ema, dbg_magnitude, dbg_baseline;

    reg  [1:0]         len_sel;
    reg                force_rearm;
    wire               wr_en;
    wire [8:0]         wr_addr;
    wire signed [15:0] wr_data;
    wire               display_en;
    wire [8:0]         disp_origin;
    wire [1:0]         disp_len_sel;
    wire [2:0]         auto_gain;
    wire               cap_done;
    wire               rearm_pulse;
    wire [1:0]         cap_state;
    wire               clear_history = force_rearm | rearm_pulse;

    wire [8:0]         rd_addr = 9'd0;   // 显示读口本 TB 不用 (直接层次引用 ram)
    wire signed [15:0] rd_data;

    wave_trigger #(
        .TH_MIN        (24'd20000),
        .TH_MAX        (24'd120000),
        .REARM_POINTS (TB_REARM_PTS)
    ) u_trig (
        .clk             (clk),
        .rst_n           (rst_n),
        .sample_data     (sample_data),
        .sample_valid    (sample_valid),
        .base_en         (base_en),
        .track_en        (track_en),
        .arm_en          (arm_en),
        .clear_history   (clear_history),
        .corrected_data  (corrected_data),
        .raw_data        (),               // 原始随动通道本 TB 不用 (chain TB 覆盖)
        .corrected_valid (corrected_valid),
        .trigger         (trigger),
        .rearm_ok        (rearm_ok),
        .dbg_threshold   (dbg_threshold),
        .dbg_noise_ema   (dbg_noise_ema),
        .dbg_magnitude   (dbg_magnitude),
        .dbg_baseline    (dbg_baseline)
    );

    wave_capture #(
        .WARMUP_POINTS       (TB_WARMUP),
        .HOLD_TIMEOUT_POINTS (256)
    ) u_cap (
        .clk             (clk),
        .rst_n           (rst_n),
        .corrected_data  (corrected_data),
        .corrected_valid (corrected_valid),
        .trigger         (trigger),
        .rearm_ok        (rearm_ok),
        .len_sel         (len_sel),
        .force_rearm     (force_rearm),
        .base_en         (base_en),
        .track_en        (track_en),
        .arm_en          (arm_en),
        .wr_en           (wr_en),
        .wr_addr         (wr_addr),
        .wr_data         (wr_data),
        .display_en      (display_en),
        .disp_origin     (disp_origin),
        .disp_len_sel    (disp_len_sel),
        .auto_gain       (auto_gain),
        .cap_done        (cap_done),
        .rearm_pulse     (rearm_pulse),
        .dbg_state       (cap_state)
    );

    wave_buffer_dp u_buf (
        .clk     (clk),
        .wr_en   (wr_en),
        .wr_addr (wr_addr),
        .wr_data (wr_data),
        .rd_addr (rd_addr),
        .rd_data (rd_data)
    );

    localparam ST_IDLE    = 2'd0;
    localparam ST_ARMED   = 2'd1;
    localparam ST_CAPTURE = 2'd2;
    localparam ST_HOLD    = 2'd3;

    // ------------------------------------------------------------------------
    // 影子模型: 基线复刻 + corrected 期望值历史
    // ------------------------------------------------------------------------
    reg  signed [23:0] sh_base;                // 影子基线
    reg  signed [23:0] hist [0:16383];         // 期望 corrected 历史
    integer            sent_cnt;               // 已发送样点数
    integer            trig_idx;               // 触发样点的 hist 索引
    integer            chk_cnt;                // corrected_valid 校验计数
    integer            errors;

    reg  signed [24:0] sh_diff;
    reg  signed [24:0] sh_corr;
    reg  signed [24:0] sh_step;

    // ---- 发送一个样点 (negedge 驱动, posedge 被 DUT 采样) ----
    task send_sample(input signed [23:0] val);
        reg trk;
        begin
            @(negedge clk);
            trk          = base_en;            // 采样期状态稳定 (间隔>>流水深度)
            sample_data  = val;
            sample_valid = 1'b1;
            // 影子模型: corrected 用更新前基线, base_en=1 时基线迭代
            sh_diff = {val[23], val} - {sh_base[23], sh_base};
            if (sh_diff[24] != sh_diff[23])
                sh_corr = sh_diff[24] ? 25'sh1800000 : 25'sh07FFFFF;  // 饱和(不会发生)
            else
                sh_corr = sh_diff;
            hist[sent_cnt] = sh_corr[23:0];
            // 基线单步钳位 ±4096 (与 wave_trigger BASE_STEP_MAX 一致)
            sh_step = sh_diff >>> 6;
            if (sh_step > 25'sd4096)       sh_step = 25'sd4096;
            else if (sh_step < -25'sd4096) sh_step = -25'sd4096;
            if (trk)
                sh_base = sh_base + sh_step;
            sent_cnt = sent_cnt + 1;
            @(negedge clk);
            sample_valid = 1'b0;
            repeat (SAMPLE_GAP) @(posedge clk);
        end
    endtask

    // ---- corrected_data 输出逐点比对 (验证 wave_trigger 基线数学) ----
    always @(posedge clk) begin
        if (rst_n && corrected_valid) begin
            if (corrected_data !== hist[chk_cnt]) begin
                errors = errors + 1;
                if (errors < 20)
                    $display("[%0t] FAIL: corrected[%0d] = %h, 期望 %h",
                             $time, chk_cnt, corrected_data, hist[chk_cnt]);
            end
            chk_cnt = chk_cnt + 1;
        end
    end

    // ---- 触发样点索引记录 (trigger 与 corrected_valid 同拍) ----
    always @(posedge clk) begin
        if (rst_n && trigger)
            trig_idx = sent_cnt - 1;   // 最近一个已发送样点
    end

    // ---- 修复回归: disp_origin 只允许在 CAPTURE→HOLD (采集完成提交) 时变化 ----
    //   若在触发瞬间 (ARMED→CAPTURE) 或采集中变化 = 屏幕提前切到写入中的 bank
    reg [8:0] disp_origin_q;
    reg [1:0] state_q;
    always @(posedge clk) begin
        if (rst_n) begin
            if ((disp_origin !== disp_origin_q) &&
                !(state_q == ST_CAPTURE && cap_state == ST_HOLD)) begin
                errors = errors + 1;
                $display("[%0t] FAIL: disp_origin 在状态 %0d→%0d 时被修改 (采集未完成即切换显示)",
                         $time, state_q, cap_state);
            end
        end
        disp_origin_q <= disp_origin;
        state_q       <= cap_state;
    end

    // ------------------------------------------------------------------------
    // 校验任务
    // ------------------------------------------------------------------------
    // RAM 内容与影子历史逐点比对:
    //   RAM[{bank, origin8+k}] == hist[trig_idx - pre + k][23:8], k=0..len-1
    task check_ram(input integer len, input integer pre, input exp_bank);
        integer k;
        reg [8:0]         addr;
        reg signed [15:0] exp16;
        begin
            if (disp_origin[8] !== exp_bank) begin
                errors = errors + 1;
                $display("[%0t] FAIL: disp_origin bank = %b, 期望 %b",
                         $time, disp_origin[8], exp_bank);
            end
            for (k = 0; k < len; k = k + 1) begin
                addr  = {disp_origin[8], disp_origin[7:0] + k[7:0]};
                exp16 = hist[trig_idx - pre + k][23:8];
                if (u_buf.ram[addr] !== exp16) begin
                    errors = errors + 1;
                    if (errors < 40)
                        $display("[%0t] FAIL: RAM[%0d](点%0d) = %h, 期望 %h",
                                 $time, addr, k, u_buf.ram[addr], exp16);
                end
            end
            $display("[%0t] RAM %0d 点逐点比对完成 (预触发 %0d 点, bank%b)",
                     $time, len, pre, exp_bank);
        end
    endtask

    // 状态断言
    task expect_state(input [1:0] exp, input [127:0] msg);
        begin
            if (cap_state !== exp) begin
                errors = errors + 1;
                $display("[%0t] FAIL: %0s: state = %0d, 期望 %0d",
                         $time, msg, cap_state, exp);
            end
        end
    endtask

    // ---- P2 自动增益影子校验: 峰值覆盖完整 N 点显示窗口 |hist[23:8]| ----
    //   (预触发 pre 点 + 触发样点起 len-pre 点), 映射与 wave_capture 相同 (§13.2)
    //   注: DUT 预触发用双 32 点窗口 (只会更保守); 本 TB 各测试保证
    //   [trig-64, trig-pre) 区间安静, 期望值与 DUT 一致
    task check_auto_gain(input integer len, input integer pre);
        integer k;
        reg signed [15:0] v16;
        reg [15:0]        mag, pk;
        reg [2:0]         eg;
        begin
            pk = 16'd0;
            for (k = 0; k < len; k = k + 1) begin
                v16 = hist[trig_idx - pre + k][23:8];
                mag = v16[15] ? (~v16 + 1'b1) : v16;
                if (mag > pk) pk = mag;
            end
            eg = (pk >= 16'd16384) ? 3'd0 :
                 (pk >= 16'd8192)  ? 3'd1 :
                 (pk >= 16'd4096)  ? 3'd2 :
                 (pk >= 16'd2048)  ? 3'd3 : 3'd4;
            if (auto_gain !== eg) begin
                errors = errors + 1;
                $display("[%0t] FAIL: auto_gain = %0d, 期望 %0d (峰值 %0d)",
                         $time, auto_gain, eg, pk);
            end else
                $display("[%0t] auto_gain = %0d (峰值 %0d) — 正确", $time, auto_gain, pk);
        end
    endtask

    // 等待进入 HOLD (带超时)
    task wait_hold;
        integer t;
        begin
            t = 0;
            while (cap_state !== ST_HOLD && t < 200000) begin
                @(posedge clk);
                t = t + 1;
            end
            if (cap_state !== ST_HOLD) begin
                errors = errors + 1;
                $display("[%0t] FAIL: 等待 HOLD 超时", $time);
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // 激励序列
    // ------------------------------------------------------------------------
    integer i;
    integer trig_expect;                  // 期望的触发计数
    integer trig_seen;                    // 实际观测到的 trigger 脉冲数
    integer done_seen;                    // cap_done 脉冲计数 (每次采集恰 1 个)
    integer rearm_seen;                   // 自动/手动重新武装脉冲计数
    reg signed [15:0] ram_snap [0:511];   // HOLD 稳定性快照

    always @(posedge clk)
        if (rst_n && trigger) trig_seen = trig_seen + 1;

    always @(posedge clk)
        if (rst_n && cap_done) begin
            done_seen = done_seen + 1;
            if (cap_state !== ST_HOLD) begin   // cap_done 与 CAPTURE→HOLD 同拍寄存
                errors = errors + 1;
                $display("[%0t] FAIL: cap_done 脉冲不在采集完成拍", $time);
            end
        end

    always @(posedge clk)
        if (rst_n && rearm_pulse)
            rearm_seen = rearm_seen + 1;

    initial begin
        rst_n        = 1'b0;
        sample_data  = 24'd0;
        sample_valid = 1'b0;
        len_sel      = 2'b00;
        force_rearm  = 1'b0;
        sh_base      = 24'sd0;
        sent_cnt     = 0;
        trig_idx     = -1;
        chk_cnt      = 0;
        errors       = 0;
        trig_seen    = 0;
        done_seen    = 0;
        rearm_seen   = 0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // ====================================================================
        // 测试1: 64 点采集 (正向冲击)
        // ====================================================================
        $display("========== 测试1: 64 点采集 ==========");
        len_sel = 2'b00;

        // ---- 修复回归: IDLE 预热期间 force_rearm 应被忽略 (不绕过预热) ----
        for (i = 0; i < 20; i = i + 1)
            send_sample(24'sd0);
        expect_state(ST_IDLE, "预热中应为 IDLE");
        @(negedge clk); force_rearm = 1'b1;
        @(negedge clk); force_rearm = 1'b0;
        repeat (4) @(posedge clk);
        expect_state(ST_IDLE, "IDLE 中 force_rearm 应被忽略");

        // 预热 + 预触发填充 (零输入, 基线保持 0)
        for (i = 0; i < TB_WARMUP + 20; i = i + 1)
            send_sample(24'sd0);
        expect_state(ST_ARMED, "预热后应为 ARMED");
        if (trig_seen != 0) begin
            errors = errors + 1;
            $display("FAIL: 静态输入产生了触发");
        end

        // 正向冲击: 首点 400000 (> TH_MIN 20000), 递减振铃
        for (i = 0; i < 120; i = i + 1)
            send_sample(24'sd400000 - i * 24'sd1200);
        wait_hold;
        expect_state(ST_HOLD, "冲击后应进入 HOLD");
        if (trig_seen != 1) begin
            errors = errors + 1;
            $display("FAIL: 触发次数 = %0d, 期望 1 (一次敲击只触发一次)", trig_seen);
        end
        if (disp_len_sel !== 2'b00) begin
            errors = errors + 1;
            $display("FAIL: disp_len_sel = %b, 期望 00", disp_len_sel);
        end
        check_ram(64, 8, 1'b0);           // 第一次采集: bank0
        check_auto_gain(64, 8);           // 峰值~1562 → ×16 档
        if (done_seen != 1) begin
            errors = errors + 1;
            $display("FAIL: cap_done 计数 = %0d, 期望 1", done_seen);
        end

        // ---- HOLD 后 RAM 不被覆盖 (大信号轰击) ----
        for (i = 0; i < 512; i = i + 1)
            ram_snap[i] = u_buf.ram[i];
        for (i = 0; i < 30; i = i + 1)
            send_sample(24'sd500000);
        expect_state(ST_HOLD, "HOLD 中大信号不应重新采集");
        for (i = 0; i < 512; i = i + 1) begin
            if (u_buf.ram[i] !== ram_snap[i]) begin
                errors = errors + 1;
                if (errors < 50)
                    $display("FAIL: HOLD 期间 RAM[%0d] 被改写", i);
            end
        end
        $display("[%0t] HOLD 覆盖保护检查完成", $time);

        // ---- 静默重新武装 (含 HOLD 期基线重收敛: 残差 <15000 需 ~250 点) ----
        for (i = 0; i < TB_REARM_PTS + 400; i = i + 1)
            send_sample(24'sd0);
        expect_state(ST_ARMED, "静默后应重新武装");
        if (display_en !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: 重新武装后旧波形应保持显示");
        end

        // ====================================================================
        // 测试2: 128 点采集 (负向冲击, 验证绝对值触发 + bank 翻转)
        // ====================================================================
        $display("========== 测试2: 128 点采集 (负向) ==========");
        len_sel = 2'b01;
        for (i = 0; i < 40; i = i + 1)     // 预触发缓冲填充
            send_sample(24'sd0);
        trig_expect = trig_seen + 1;
        for (i = 0; i < 200; i = i + 1)
            send_sample(-24'sd400000 + i * 24'sd1500);
        wait_hold;
        if (trig_seen != trig_expect) begin
            errors = errors + 1;
            $display("FAIL: 负向冲击触发次数异常");
        end
        if (disp_len_sel !== 2'b01) begin
            errors = errors + 1;
            $display("FAIL: disp_len_sel = %b, 期望 01", disp_len_sel);
        end
        check_ram(128, 16, 1'b1);          // 第二次采集: bank1 (乒乓)
        check_auto_gain(128, 16);

        for (i = 0; i < TB_REARM_PTS + 400; i = i + 1)
            send_sample(24'sd0);
        expect_state(ST_ARMED, "测试2后重新武装");

        // ====================================================================
        // 测试3: 256 点采集 (bank 乒乓回 bank0)
        // ====================================================================
        $display("========== 测试3: 256 点采集 ==========");
        len_sel = 2'b10;
        for (i = 0; i < 40; i = i + 1)
            send_sample(24'sd0);
        // 大幅冲击 (峰值 4000000>>8 = 15625 → 自动增益 ×2 档, 覆盖不同档位)
        for (i = 0; i < 300; i = i + 1)
            send_sample(24'sd4000000 - i * 24'sd10000);
        wait_hold;
        if (disp_len_sel !== 2'b10) begin
            errors = errors + 1;
            $display("FAIL: disp_len_sel = %b, 期望 10", disp_len_sel);
        end
        check_ram(256, 32, 1'b0);          // 第三次采集: 回 bank0
        check_auto_gain(256, 32);          // 峰值~15625 → ×2 档
        if (done_seen != 3) begin
            errors = errors + 1;
            $display("FAIL: cap_done 计数 = %0d, 期望 3", done_seen);
        end

        for (i = 0; i < TB_REARM_PTS + 400; i = i + 1)
            send_sample(24'sd0);
        expect_state(ST_ARMED, "测试3后重新武装");

        // ====================================================================
        // 测试4: 自适应阈值 (冻结版 noise_ema, §4.2)
        //   噪声 ±8000 → noise_ema≈8000 → threshold=ema×6≈45000~48000
        //   30000 冲击 (>TH_MIN=20000 但 <自适应阈值) 不得触发
        // ====================================================================
        $display("========== 测试4: 自适应阈值 ==========");
        // 先送零恢复基线 (noise_ema 衰减 → threshold 回 TH_MIN),
        // 再送交替噪声跑 ~3×EMA 时间常数 (192 点 → ema≈0.95×8000)
        for (i = 0; i < 64; i = i + 1)
            send_sample(24'sd0);
        for (i = 0; i < TB_NOISE_PTS; i = i + 1)
            send_sample((i & 1) ? -24'sd8000 : 24'sd8000);
        if ((dbg_threshold < 24'd40000) || (dbg_threshold > 24'd48000)) begin
            errors = errors + 1;
            $display("FAIL: 自适应阈值 = %0d, 期望 ≈45000 (noise_ema=%0d)",
                     dbg_threshold, dbg_noise_ema);
        end
        trig_expect = trig_seen;
        for (i = 0; i < 5; i = i + 1)      // 30000 < threshold → 不触发
            send_sample(24'sd30000);
        if (trig_seen != trig_expect) begin
            errors = errors + 1;
            $display("FAIL: 低于自适应阈值的信号触发了采集");
        end else
            $display("[%0t] 30000 冲击未触发 (阈值=%0d) — 正确", $time, dbg_threshold);
        // 300000 > threshold → 正常触发
        for (i = 0; i < 300; i = i + 1)
            send_sample(24'sd300000 - i * 24'sd800);
        wait_hold;
        if (trig_seen != trig_expect + 1) begin
            errors = errors + 1;
            $display("FAIL: 高于自适应阈值的信号未触发");
        end else
            $display("[%0t] 300000 冲击正常触发", $time);

        // ====================================================================
        // 测试5: force_rearm (按键强制重新武装)
        // ====================================================================
        $display("========== 测试5: force_rearm ==========");
        expect_state(ST_HOLD, "测试5前应在 HOLD");
        @(negedge clk);
        force_rearm = 1'b1;
        @(negedge clk);
        force_rearm = 1'b0;
        repeat (4) @(posedge clk);
        expect_state(ST_ARMED, "force_rearm 后应为 ARMED");
        if (display_en !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL: force_rearm 后应清屏 (display_en=0)");
        end

        // ====================================================================
        // 测试6: 预触发窗口孤立尖峰计入自动增益 (完整 N 点窗口, 修复回归)
        //   尖峰 1500000 单点 (TRIG_CONFIRM=2 → 不触发), 10 个零点后
        //   300000 冲击触发; 显示窗口 (256 点, 预触发 32) 含尖峰 →
        //   期望峰值 ~5858 → ×4 档(2); 修复前仅统计 post 段 (~1171 → ×16 档)
        // ====================================================================
        $display("========== 测试6: 预触发尖峰计入增益 ==========");
        len_sel = 2'b10;
        // 静默恢复: noise_ema 从测试4尾部指数衰减 (384 点 = 6×EMA 时间常数
        // → ema≈0, threshold 回 TH_MIN); 孤立尖峰期间 over_half 冻结 ema,
        // 不会抬高阈值
        for (i = 0; i < 384; i = i + 1)
            send_sample(24'sd0);
        expect_state(ST_ARMED, "测试6静默后应保持 ARMED");
        trig_expect = trig_seen;
        send_sample(24'sd1500000);        // 孤立尖峰: 超阈值仅 1 点
        for (i = 0; i < 10; i = i + 1)
            send_sample(24'sd0);
        if (trig_seen != trig_expect) begin
            errors = errors + 1;
            $display("FAIL: 孤立尖峰不应触发 (TRIG_CONFIRM=2)");
        end
        for (i = 0; i < 300; i = i + 1)
            send_sample(24'sd300000 - i * 24'sd800);
        wait_hold;
        if (trig_seen != trig_expect + 1) begin
            errors = errors + 1;
            $display("FAIL: 测试6冲击未正常触发");
        end
        check_ram(256, 32, 1'b0);          // 第五次采集: 乒乓回 bank0
        check_auto_gain(256, 32);          // 窗口含尖峰 → ×4 档 (修复前 ×16)
        if (done_seen != 5) begin
            errors = errors + 1;
            $display("FAIL: cap_done 计数 = %0d, 期望 5", done_seen);
        end

        // ====================================================================
        // 测试7: HOLD 超时兜底，持续余振/底噪不能造成永久卡死
        //   交替大信号使 quiet 始终不成立；256 个有效点后必须强制重武装。
        //   随后恢复静默并再次敲击，必须产生第 6 次采集。
        // ====================================================================
        $display("========== 测试7: HOLD 超时恢复 ==========");
        expect_state(ST_HOLD, "测试7前应在 HOLD");
        trig_expect = rearm_seen + 1;
        for (i = 0; i < 256; i = i + 1)
            send_sample((i & 1) ? -24'sd500000 : 24'sd500000);
        repeat (4) @(posedge clk);
        expect_state(ST_ARMED, "持续非静默达到超时后应重新武装");
        if (rearm_seen != trig_expect) begin
            errors = errors + 1;
            $display("FAIL: HOLD 超时未产生 rearm_pulse");
        end
        if (display_en !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: 自动超时重武装后旧波形应保持显示");
        end

        // 给基线足够时间从持续大信号恢复，随后验证新一击确实可再次采集。
        for (i = 0; i < 400; i = i + 1)
            send_sample(24'sd0);
        trig_expect = trig_seen + 1;
        for (i = 0; i < 300; i = i + 1)
            send_sample(24'sd400000 - i * 24'sd1200);
        wait_hold;
        if (trig_seen != trig_expect || done_seen != 6) begin
            errors = errors + 1;
            $display("FAIL: 超时恢复后的再次敲击未完成采集 (trig=%0d, done=%0d)",
                     trig_seen, done_seen);
        end

        // ====================================================================
        repeat (20) @(posedge clk);
        $display("");
        if (errors == 0)
            $display("=============== 全部测试 PASS ===============");
        else
            $display("=============== FAIL: %0d 个错误 ===============", errors);
        $finish;
    end

    // 全局超时保护
    initial begin
        #200_000_000;
        $display("FAIL: 仿真全局超时");
        $finish;
    end

endmodule
