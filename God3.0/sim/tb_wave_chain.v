`timescale 1ns / 1ps
// ============================================================================
// Testbench: tb_wave_chain
// 对象: 完整采集导出链路集成测试 (审查意见: 分模块测试无法防一拍偏移)
//   wave_trigger → wave_smooth → wave_capture → wave_buffer_dp
//                                             ↘ wave_uart_export → UART
//
// 测试策略:
//   - 平滑开启 (bypass=0): 显示链路走平滑数据, UART 影子链路走原始数据
//   - 两次冲击 (乒乓 bank0/bank1), 各自动导出一帧 64 点
//   - 帧数据逐字节与 TB 激励原值 sent[] 比对: raw 通道与基线/平滑无关,
//     任何一拍偏移或错样点都会精确暴露 (无建模误差)
//   - 触发样点索引断言 (trig==第 2 个超阈值样点): 锁定 smooth 延迟对齐
//
// 时间轴 (样点间隔 60 clk = 1.5µs):
//   sent[0..263]     零 (预热 64 + ARMED 填充)
//   sent[264..293]   冲击1: 2000000 - j*20000 (30 点) → trig @265, bank0
//   sent[294..1599]  零 (静默重武装; 帧1 985µs ≈ 657 样点 << 间隔 1306)
//   sent[1600..1629] 冲击2: 同形态 → trig @1601, bank1
//   sent[1630..2047] 零 (冲击2 采集完成 + 帧2 发送)
//
// 加速参数: REARM_POINTS=128, WARMUP_POINTS=64 (trigger 其余参数默认:
//   TH_MIN=500000, TRIG_CONFIRM=2, SETTLE_POINTS=16)
// UART 接收: 2Mbps (BIT_NS=500), 起始位下降沿 + 半位对准
// ============================================================================
module tb_wave_chain;

    reg clk, rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;    // 40MHz

    localparam SAMPLE_GAP = 60;     // 样点间隔 (clk)
    localparam BIT_NS     = 500;    // 2Mbps

    // ------------------------------------------------------------------------
    // DUT 链路 (接线复刻 vga_dual_mode_top)
    // ------------------------------------------------------------------------
    reg  signed [23:0] sample_data;
    reg                sample_valid;
    wire               base_en, track_en, arm_en;
    wire signed [23:0] corrected_data, trig_raw_data;
    wire               corrected_valid;
    wire               wave_trig, wave_rearm_ok;
    wire               capture_rearm_pulse;
    wire [23:0]        dbg_threshold, dbg_noise_ema, dbg_magnitude, dbg_baseline;

    wave_trigger #(
        .REARM_POINTS (128)
    ) u_trig (
        .clk             (clk),
        .rst_n           (rst_n),
        .sample_data     (sample_data),
        .sample_valid    (sample_valid),
        .base_en         (base_en),
        .track_en        (track_en),
        .arm_en          (arm_en),
        .clear_history   (capture_rearm_pulse),
        .corrected_data  (corrected_data),
        .raw_data        (trig_raw_data),
        .corrected_valid (corrected_valid),
        .trigger         (wave_trig),
        .rearm_ok        (wave_rearm_ok),
        .dbg_threshold   (dbg_threshold),
        .dbg_noise_ema   (dbg_noise_ema),
        .dbg_magnitude   (dbg_magnitude),
        .dbg_baseline    (dbg_baseline)
    );

    wire signed [23:0] smooth_data, smooth_raw;
    wire               smooth_valid, smooth_trig;

    wave_smooth u_smooth (
        .clk         (clk),
        .rst_n       (rst_n),
        .bypass      (1'b0),           // 平滑开启: 验证 raw 通道不被平滑
        .in_data     (corrected_data),
        .in_raw      (trig_raw_data),
        .in_valid    (corrected_valid),
        .in_trigger  (wave_trig),
        .out_data    (smooth_data),
        .out_raw     (smooth_raw),
        .out_valid   (smooth_valid),
        .out_trigger (smooth_trig)
    );

    wire               buf_wr_en;
    wire [8:0]         buf_wr_addr;
    wire signed [15:0] buf_wr_data;
    wire               display_en, cap_done;
    wire [8:0]         disp_origin;
    wire [1:0]         disp_len_sel;
    wire [2:0]         auto_gain;
    wire [1:0]         cap_state;

    wave_capture #(
        .WARMUP_POINTS       (64),
        .HOLD_TIMEOUT_POINTS (512)
    ) u_cap (
        .clk             (clk),
        .rst_n           (rst_n),
        .corrected_data  (smooth_data),
        .corrected_valid (smooth_valid),
        .trigger         (smooth_trig),
        .rearm_ok        (wave_rearm_ok),
        .len_sel         (2'b00),       // 64 点, 预触发 8
        .force_rearm     (1'b0),
        .base_en         (base_en),
        .track_en        (track_en),
        .arm_en          (arm_en),
        .wr_en           (buf_wr_en),
        .wr_addr         (buf_wr_addr),
        .wr_data         (buf_wr_data),
        .display_en      (display_en),
        .disp_origin     (disp_origin),
        .disp_len_sel    (disp_len_sel),
        .auto_gain       (auto_gain),
        .cap_done        (cap_done),
        .rearm_pulse     (capture_rearm_pulse),
        .dbg_state       (cap_state)
    );

    wire signed [15:0] rd_data;
    wave_buffer_dp u_buf (
        .clk     (clk),
        .wr_en   (buf_wr_en),
        .wr_addr (buf_wr_addr),
        .wr_data (buf_wr_data),
        .rd_addr (9'd0),
        .rd_data (rd_data)
    );

    wire busy, uart_txd;
    wave_uart_export #(
        .UART_BPS (2_000_000)      // 仿真加速: 硬件 115200
    ) u_export (
        .clk          (clk),
        .rst_n        (rst_n),
        .wr_en        (buf_wr_en),
        .wr_addr      (buf_wr_addr),
        .wr_raw       (smooth_raw),
        .cap_done     (cap_done),
        .disp_origin  (disp_origin),
        .disp_len_sel (disp_len_sel),
        // 本 TB 不验证结果段内容 (固定 0, 仅消费版本化帧尾)
        .result_ready      (1'b0),
        .result_state      (2'd0),
        .confidence        (2'd0),
        .impact_index      (9'd0),
        .defect_index      (9'd0),
        .bottom_index      (9'd0),
        .distance_mm       (12'd0),
        .trigger_threshold (24'd0),
        .busy         (busy),
        .uart_txd     (uart_txd)
    );

    // ------------------------------------------------------------------------
    // 激励历史 + 触发样点索引记录
    // ------------------------------------------------------------------------
    reg signed [23:0] sent [0:2047];
    integer sent_cnt, errors;
    reg     stim_done, rx_done;

    task send_sample(input signed [23:0] val);
        begin
            @(negedge clk);
            sample_data  = val;
            sample_valid = 1'b1;
            sent[sent_cnt] = val;
            sent_cnt = sent_cnt + 1;
            @(negedge clk);
            sample_valid = 1'b0;
            repeat (SAMPLE_GAP) @(posedge clk);
        end
    endtask

    // smooth 输出第 m 个样点 ↔ sent[m+1] (前 2 点填充, 每输入 1 出 1)
    integer out_cnt, trig_cnt;
    integer trig_sent [1:2];    // 第 1/2 次触发对应的 sent 索引
    always @(posedge clk) begin
        if (rst_n && smooth_valid) begin
            if (smooth_trig) begin
                trig_cnt = trig_cnt + 1;
                if (trig_cnt <= 2)
                    trig_sent[trig_cnt] = out_cnt + 1;
                $display("[%0t] 触发 #%0d: sent 索引 %0d", $time, trig_cnt, out_cnt + 1);
            end
            out_cnt = out_cnt + 1;
        end
    end

    // ------------------------------------------------------------------------
    // TB UART 接收器
    // ------------------------------------------------------------------------
    reg [7:0] rx_byte;
    integer bi;
    task recv_byte;
        begin
            @(negedge uart_txd);
            #(BIT_NS / 2);
            if (uart_txd !== 1'b0) begin
                errors = errors + 1;
                $display("[%0t] FAIL: 起始位异常", $time);
            end
            for (bi = 0; bi < 8; bi = bi + 1) begin
                #(BIT_NS);
                rx_byte[bi] = uart_txd;
            end
            #(BIT_NS);
            if (uart_txd !== 1'b1) begin
                errors = errors + 1;
                $display("[%0t] FAIL: 停止位异常", $time);
            end
        end
    endtask

    task expect_byte(input [7:0] exp, input [127:0] msg);
        begin
            recv_byte;
            if (rx_byte !== exp) begin
                errors = errors + 1;
                $display("[%0t] FAIL: %0s: 收到 %h, 期望 %h", $time, msg, rx_byte, exp);
            end
        end
    endtask

    // 帧校验: 数据 == 激励原值 sent[torig+k] (穿越全链路后仍是原始 24 位)
    task check_frame(input integer fid);
        integer torig, k;
        reg [7:0] sum;
        reg signed [23:0] s;
        begin
            expect_byte(8'hAA, "帧头0");
            expect_byte(8'h55, "帧头1");
            expect_byte(8'h00, "长度字段(64点)");
            // 帧头已到 → 本帧触发索引必已记录
            if (trig_cnt < fid) begin
                errors = errors + 1;
                $display("[%0t] FAIL: 帧 %0d 开始时未见对应触发", $time, fid);
            end
            torig = trig_sent[fid] - 8;    // 显示起点 = 触发样点 - 预触发 8
            sum = 8'd0;                    // len_sel 字节 = 0, 直接从 0 累计
            for (k = 0; k < 64; k = k + 1) begin
                s = sent[torig + k];
                expect_byte(s[23:16], "样点[23:16]");
                expect_byte(s[15:8],  "样点[15:8]");
                expect_byte(s[7:0],   "样点[7:0]");
                sum = sum + s[23:16] + s[15:8] + s[7:0];
            end
            expect_byte(sum, "校验和");
            // 帧尾 14 字节版本化结果段 (ver=0x02; 本 TB 输入固定 0 →
            // 除标记/版本/rsum 外全 0, rsum = ver 起 11 字节累加 = 0x02)
            expect_byte(8'h5A, "结果段标记");
            expect_byte(8'h02, "版本号");
            expect_byte(8'h00, "status");
            expect_byte(8'h00, "idx_h");
            expect_byte(8'h00, "impact_index");
            expect_byte(8'h00, "defect_index");
            expect_byte(8'h00, "bottom_index");
            expect_byte(8'h00, "dist[11:8]");
            expect_byte(8'h00, "dist[7:0]");
            expect_byte(8'h00, "thr[23:16]");
            expect_byte(8'h00, "thr[15:8]");
            expect_byte(8'h00, "thr[7:0]");
            expect_byte(8'h02, "结果段校验和");
            $display("[%0t] 帧 %0d 校验完成: 64 点原始数据全对 (torig=%0d)",
                     $time, fid, torig);
        end
    endtask

    // ------------------------------------------------------------------------
    // 并行接收线程: 两帧
    // ------------------------------------------------------------------------
    initial begin
        rx_done = 1'b0;
        wait (rst_n === 1'b1);
        check_frame(1);
        check_frame(2);
        rx_done = 1'b1;
    end

    // ------------------------------------------------------------------------
    // 激励主线程
    // ------------------------------------------------------------------------
    integer i;
    initial begin
        rst_n        = 1'b0;
        sample_data  = 24'sd0;
        sample_valid = 1'b0;
        sent_cnt     = 0;
        out_cnt      = 0;
        trig_cnt     = 0;
        errors       = 0;
        stim_done    = 1'b0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        $display("========== 全链路: 冲击1 (bank0) ==========");
        for (i = 0; i < 264; i = i + 1)            // 预热 + ARMED 填充
            send_sample(24'sd0);
        for (i = 0; i < 30; i = i + 1)             // 冲击1 → trig @265
            send_sample(24'sd2000000 - i * 24'sd20000);

        $display("========== 全链路: 静默重武装 + 冲击2 (bank1) ==========");
        for (i = 0; i < 1306; i = i + 1)           // sent[294..1599] 零
            send_sample(24'sd0);
        for (i = 0; i < 30; i = i + 1)             // 冲击2 → trig @1601
            send_sample(24'sd2000000 - i * 24'sd20000);
        for (i = 0; i < 418; i = i + 1)            // 采集完成 + 帧2 发送期
            send_sample(24'sd0);
        stim_done = 1'b1;

        wait (rx_done === 1'b1);
        repeat (100) @(posedge clk);

        // ---- 触发对齐断言: 锁定 smooth 一样点延迟的对齐关系 ----
        if (trig_cnt !== 2) begin
            errors = errors + 1;
            $display("FAIL: 触发次数 = %0d, 期望 2", trig_cnt);
        end
        if (trig_sent[1] !== 265) begin
            errors = errors + 1;
            $display("FAIL: 触发1 索引 = %0d, 期望 265 (第 2 个超阈值样点)",
                     trig_sent[1]);
        end
        if (trig_sent[2] !== 1601) begin
            errors = errors + 1;
            $display("FAIL: 触发2 索引 = %0d, 期望 1601", trig_sent[2]);
        end
        // 帧 1/2 分别读 bank0/bank1 (数据比对已隐式验证乒乓)
        if (disp_origin[8] !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL: 第二次采集 bank = %b, 期望 1 (乒乓)", disp_origin[8]);
        end
        if (busy) begin
            errors = errors + 1;
            $display("FAIL: 两帧发送完成后 busy 未落下");
        end

        $display("");
        if (errors == 0)
            $display("=============== 全部测试 PASS ===============");
        else
            $display("=============== FAIL: %0d 个错误 ===============", errors);
        $finish;
    end

    // 全局超时 (激励 ~3.1ms + 帧2 ~1ms)
    initial begin
        #10_000_000;
        $display("FAIL: 仿真全局超时 (stim_done=%b rx_done=%b)", stim_done, rx_done);
        $finish;
    end

endmodule
