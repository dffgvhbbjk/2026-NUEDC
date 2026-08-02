`timescale 1ns / 1ps
// ============================================================================
// Testbench: tb_defect_analyzer
// 对象: 连续波包候选 + 候选自相关微调的 defect_analyzer
//
// 覆盖:
//   1. 正常棒比例特征 -> NORMAL
//   2. 305 mm 特征 -> DEFECT，连续候选约 29
//   3. 695 mm 特征 -> DEFECT，连续候选约 71
//   4. 非固定 520 mm 特征 -> DEFECT，且 2×Ddef > Dbottom
//   4. 305 mm 无棒底 -> DEFECT/低置信
//   5. 弱帧 -> INVALID
//   6. 饱和帧 -> INVALID
//   7. rearm 中止且可再次触发
// ============================================================================
module tb_defect_analyzer;

    reg clk, rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;

    reg  signed [23:0] sample_data;
    reg                sample_valid;
    reg                trigger;
    reg                capture_done;
    reg                rearm;
    reg  [23:0]        trigger_threshold;
    reg  [8:0]         d_ref;

    wire               result_valid;
    wire [1:0]         result_state;
    wire               measurement_valid;
    wire [1:0]         confidence;
    wire               bottom_found;
    wire [8:0]         impact_peak_index;
    wire [8:0]         defect_peak_index;
    wire [8:0]         bottom_peak_index;
    wire [8:0]         defect_delta;
    wire [8:0]         bottom_delta;
    wire [10:0]        defect_delta_q2;
    wire [10:0]        bottom_delta_q2;
    wire [23:0]        impact_peak_value;
    wire [23:0]        defect_peak_value;
    wire [23:0]        dbg_envelope;
    wire [23:0]        dbg_reflect_threshold;
    wire [2:0]         dbg_state;

    localparam [1:0] RES_INVALID = 2'd0;
    localparam [1:0] RES_NORMAL  = 2'd1;
    localparam [1:0] RES_DEFECT  = 2'd2;

    defect_analyzer u_dut (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .sample_data           (sample_data),
        .sample_valid          (sample_valid),
        .trigger               (trigger),
        .capture_done          (capture_done),
        .rearm                 (rearm),
        .trigger_threshold     (trigger_threshold),
        .d_ref                 (d_ref),
        .result_valid          (result_valid),
        .result_state          (result_state),
        .measurement_valid     (measurement_valid),
        .confidence            (confidence),
        .bottom_found          (bottom_found),
        .impact_peak_index     (impact_peak_index),
        .defect_peak_index     (defect_peak_index),
        .bottom_peak_index     (bottom_peak_index),
        .defect_delta          (defect_delta),
        .bottom_delta          (bottom_delta),
        .defect_delta_q2       (defect_delta_q2),
        .bottom_delta_q2       (bottom_delta_q2),
        .impact_peak_value     (impact_peak_value),
        .defect_peak_value     (defect_peak_value),
        .dbg_envelope          (dbg_envelope),
        .dbg_reflect_threshold (dbg_reflect_threshold),
        .dbg_state             (dbg_state)
    );

    integer errors;
    integer post_cnt;
    integer rv_cnt;
    integer rv_before;

    always @(posedge clk)
        if (rst_n && result_valid)
            rv_cnt = rv_cnt + 1;

    task send_sample(input signed [23:0] data, input trig);
        begin
            @(negedge clk);
            sample_data  = data;
            sample_valid = 1'b1;
            trigger      = trig;
            @(negedge clk);
            sample_valid = 1'b0;
            trigger      = 1'b0;
            sample_data  = 24'sd0;
            repeat (14) @(negedge clk);
        end
    endtask

    task send_burst(input [23:0] amp, input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                send_sample(i[0] ? -$signed({1'b0, amp}) :
                                   $signed({1'b0, amp}), 1'b0);
                post_cnt = post_cnt + 1;
            end
        end
    endtask

    task send_quiet(input integer n);
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                send_sample(24'sd0, 1'b0);
                post_cnt = post_cnt + 1;
            end
        end
    endtask

    task start_capture(input [23:0] impact_amp);
        integer i;
        begin
            for (i = 0; i < 16; i = i + 1)
                send_sample(i[0] ? -24'sd800 : 24'sd800, 1'b0);
            post_cnt = 0;
            send_sample($signed({1'b0, impact_amp}), 1'b1);
            post_cnt = 1;
        end
    endtask

    task finish_capture;
        integer wait_count;
        begin
            if (post_cnt < 224)
                send_quiet(224 - post_cnt);
            @(negedge clk);
            capture_done = 1'b1;
            @(negedge clk);
            capture_done = 1'b0;
            wait_count = 0;
            while (!result_valid && wait_count < 30000) begin
                @(negedge clk);
                wait_count = wait_count + 1;
            end
            if (!result_valid) begin
                errors = errors + 1;
                $display("FAIL: analyzer correlation timeout");
            end
        end
    endtask

    task check_result(input [1:0] exp_state, input [1:0] exp_conf,
                      input exp_meas, input exp_bottom,
                      input [255:0] name);
        begin
            if (result_state !== exp_state) begin
                errors = errors + 1;
                $display("FAIL(%0s): state=%0d exp=%0d",
                         name, result_state, exp_state);
            end
            if (confidence !== exp_conf) begin
                errors = errors + 1;
                $display("FAIL(%0s): conf=%0d exp=%0d",
                         name, confidence, exp_conf);
            end
            if (measurement_valid !== exp_meas) begin
                errors = errors + 1;
                $display("FAIL(%0s): measurement_valid=%b exp=%b",
                         name, measurement_valid, exp_meas);
            end
            if (bottom_found !== exp_bottom) begin
                errors = errors + 1;
                $display("FAIL(%0s): bottom_found=%b exp=%b",
                         name, bottom_found, exp_bottom);
            end
            $display("%0s: state=%0d conf=%0d imp=%0d def=%0d bot=%0d ddelta=%0d bdelta=%0d peak=%0d",
                     name, result_state, confidence, impact_peak_index,
                     defect_peak_index, bottom_peak_index, defect_delta,
                     bottom_delta, impact_peak_value);
            $display("  candidates: best=%0d/%0d second=%0d/%0d bottom=%0d/%0d count=%0d list=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                     u_dut.best_candidate_lag, u_dut.best_candidate_score,
                     u_dut.second_candidate_lag, u_dut.second_candidate_score,
                     u_dut.best_bottom_lag, u_dut.best_bottom_score,
                     u_dut.packet_candidate_count,
                     u_dut.packet_candidate_0, u_dut.packet_candidate_1,
                     u_dut.packet_candidate_2, u_dut.packet_candidate_3,
                     u_dut.packet_candidate_4, u_dut.packet_candidate_5,
                     u_dut.packet_candidate_6, u_dut.packet_candidate_7);
        end
    endtask

    task check_range(input [8:0] val, input integer lo, input integer hi,
                     input [255:0] name);
        begin
            if (val < lo || val > hi) begin
                errors = errors + 1;
                $display("FAIL(%0s): val=%0d exp=[%0d,%0d]",
                         name, val, lo, hi);
            end
        end
    endtask

    initial begin
        errors            = 0;
        rv_cnt            = 0;
        sample_data       = 24'sd0;
        sample_valid      = 1'b0;
        trigger           = 1'b0;
        capture_done      = 1'b0;
        rearm             = 1'b0;
        trigger_threshold = 24'd500000;
        d_ref             = 9'd105;
        rst_n             = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // S1: NORMAL。MID=80% > 66%，REF 同时给出棒底。
        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);       // index 1..15
        send_quiet(10);                    // index 16..25
        send_burst(24'd1600000, 8);        // NEAR peak≈29 (40%)
        send_quiet(31);                    // MID 从 index65 开始
        send_burst(24'd3200000, 8);        // MID=80%
        send_quiet(27);
        send_burst(24'd3300000, 8);        // REF/bottom index100..
        finish_capture;
        check_result(RES_NORMAL, 2'd2, 1'b1, 1'b1, "S1_NORMAL");
        check_range(bottom_delta, 100, 108, "S1_BOTTOM_VALLEY");

        // S2: 305。距离不再使用固定 +5 点补偿。
        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);
        send_quiet(10);
        send_burst(24'd2500000, 8);        // NEAR=62.5%
        send_quiet(26);
        send_burst(24'd1000000, 8);        // MID=25%
        send_quiet(32);
        send_burst(24'd1900000, 8);        // REF=47.5%
        finish_capture;
        check_result(RES_DEFECT, 2'd2, 1'b1, 1'b1, "S2_305");
        check_range(defect_peak_index, 20, 40, "S2_RED_PEAK");
        check_range(defect_delta, 27, 33, "S2_CONTINUOUS_DELTA");
        check_range(bottom_delta, 100, 108, "S2_BOTTOM_VALLEY");
        if (u_dut.defect_after_half !== 1'b0) begin
            errors = errors + 1;
            $display("FAIL(S2_HALF): expected before 500mm");
        end

        // S2B: 305 的中段振铃超过 60%。旧 OR 判据会误判成 695；
        // 新判据要求 NEAR 或 REF 独立证据，仍应落入近端。
        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);
        send_quiet(10);
        send_burst(24'd2500000, 8);        // NEAR=62.5%
        send_quiet(26);
        send_burst(24'd2600000, 8);        // MID=65%, 仅 MID 命中旧远端条件
        send_quiet(32);
        send_burst(24'd1900000, 8);        // REF=47.5%
        finish_capture;
        check_result(RES_DEFECT, 2'd2, 1'b1, 1'b1, "S2B_305_MID_RING");
        check_range(defect_delta, 27, 33, "S2B_STAYS_FIRST_PACKET");

        // S3: 695。REF=60% > 55%，缺陷位置取 MID 峰约71。
        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);
        send_quiet(10);
        send_burst(24'd3000000, 8);        // NEAR=75%
        send_quiet(34);                    // MID 从 index68 开始
        send_burst(24'd2100000, 8);        // MID=52.5%
        send_quiet(24);
        send_burst(24'd2400000, 8);        // REF=60%
        finish_capture;
        check_result(RES_DEFECT, 2'd2, 1'b1, 1'b1, "S3_695");
        check_range(defect_delta, 69, 73, "S3_FAR_DELTA");
        check_range(bottom_delta, 100, 108, "S3_BOTTOM_VALLEY");
        if (u_dut.defect_after_half !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL(S3_HALF): expected after 500mm");
        end

        // S3B: 约 520mm，不属于原 300/700 两个窗口。候选中心 55，
        // 棒底约 105，应满足 2×Ddef > Dbottom。
        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);
        send_quiet(36);                    // 缺陷波包从 index52 开始
        send_burst(24'd1800000, 8);        // 候选中心约55
        send_quiet(36);
        send_burst(24'd2400000, 8);        // 棒底约100
        finish_capture;
        check_result(RES_DEFECT, 2'd2, 1'b1, 1'b1, "S3B_520");
        check_range(defect_delta, 52, 58, "S3B_CONTINUOUS_DELTA");
        if (u_dut.defect_after_half !== 1'b1) begin
            errors = errors + 1;
            $display("FAIL(S3B_HALF): expected after 500mm");
        end

        // S4: 305 无棒底，仍判缺陷但低置信。
        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);
        send_quiet(10);
        send_burst(24'd2500000, 8);
        send_quiet(26);
        send_burst(24'd1000000, 8);
        finish_capture;
        check_result(RES_DEFECT, 2'd1, 1'b1, 1'b0, "S4_305_NO_BOTTOM");

        // S5: 即使外部误触发，整帧峰值不足 500k 也必须 INVALID。
        start_capture(24'd300000);
        send_burst(24'd300000, 15);
        finish_capture;
        check_result(RES_INVALID, 2'd0, 1'b0, 1'b0, "S5_WEAK");

        // S6: 饱和帧作废。
        start_capture(24'd8388000);
        send_burst(24'd8388000, 15);
        send_quiet(49);
        send_burst(24'd7000000, 8);
        send_quiet(27);
        send_burst(24'd7000000, 8);
        finish_capture;
        check_result(RES_INVALID, 2'd0, 1'b0, 1'b0, "S6_SATURATED");

        // S7: rearm 中止，不输出；随后重新触发正常。
        // 让上一结果脉冲完整离开，避免在同一拍读取计数器产生竞态。
        repeat (2) @(negedge clk);
        rv_before = rv_cnt;
        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);
        send_quiet(10);
        @(negedge clk);
        rearm = 1'b1;
        @(negedge clk);
        rearm = 1'b0;
        repeat (4) @(negedge clk);
        if (rv_cnt != rv_before) begin
            errors = errors + 1;
            $display("FAIL(S7_REARM): aborted frame produced result");
        end

        start_capture(24'd4000000);
        send_burst(24'd4000000, 15);
        send_quiet(10);
        send_burst(24'd1600000, 8);
        send_quiet(31);
        send_burst(24'd3200000, 8);
        send_quiet(27);
        send_burst(24'd3300000, 8);
        finish_capture;
        check_result(RES_NORMAL, 2'd2, 1'b1, 1'b1, "S7_RETRIGGER");

        if (errors == 0)
            $display("==== tb_defect_analyzer: ALL PASS ====");
        else
            $display("==== tb_defect_analyzer: %0d ERRORS ====", errors);
        $finish;
    end

    initial begin
        #40_000_000;
        $display("==== tb_defect_analyzer: TIMEOUT ====");
        $finish;
    end

endmodule
