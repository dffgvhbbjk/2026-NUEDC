`timescale 1ns / 1ps
// ============================================================================
// Testbench: tb_defect_distance_calc
// 对象: defect_distance_calc 双模式 Q2/23 周期顺序除法器
//
// 测试项:
//   1. 模式A (bottom_found=1): 42×1000/83 = 506, low_confidence=0
//   2. 模式B (bottom_found=0): 42×1000/83 = 506, low_confidence=1
//   3. 双模式选择正确性: 模式A 用 bottom_delta/l_test_mm,
//      模式B 用 d_ref/l_ref_mm (两组参数故意不同, 交叉验证)
//   4. 除数为零: dist_invalid=1, 立即 done, 不锁死
//   5. 钳位: 商超过所选棒长时输出该棒长
//   6. busy 持续 21 拍, done 单脉冲
//   7. 随机 200 组 (随机模式) 与 TB 参考模型对比
// ============================================================================
module tb_defect_distance_calc;

    reg clk, rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;    // 40MHz

    reg         start;
    reg  [10:0] defect_delta_q2;
    reg         bottom_found;
    reg  [10:0] bottom_delta_q2;
    reg  [8:0]  d_ref;
    reg  [11:0] l_test_mm;
    reg  [11:0] l_ref_mm;
    wire        busy;
    wire        done;
    wire [11:0] distance_mm;
    wire        low_confidence;
    wire        dist_invalid;

    defect_distance_calc u_dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .start          (start),
        .defect_delta_q2(defect_delta_q2),
        .bottom_found   (bottom_found),
        .bottom_delta_q2(bottom_delta_q2),
        .d_ref          (d_ref),
        .l_test_mm      (l_test_mm),
        .l_ref_mm       (l_ref_mm),
        .busy           (busy),
        .done           (done),
        .distance_mm    (distance_mm),
        .low_confidence (low_confidence),
        .dist_invalid   (dist_invalid)
    );

    integer errors;

    // 参考模型: 双模式选择 + 截断除法 + 棒长钳位
    function [11:0] exp_dist;
        input [10:0] d;
        input        bf;
        input [10:0] bd;
        input [8:0]  dr;
        input [11:0] lt;
        input [11:0] lr;
        reg   [10:0] divisor;
        reg   [11:0] len;
        integer q;
        begin
            divisor = bf ? bd : {dr, 2'b00};
            len     = bf ? lt : lr;
            if (divisor == 0)
                exp_dist = 12'd0;
            else begin
                q = (d * len) / divisor;
                exp_dist = (q > len) ? len : q[11:0];
            end
        end
    endfunction

    // 所选除数 (判断 dist_invalid 期望)
    function [10:0] sel_div;
        input        bf;
        input [10:0] bd;
        input [8:0]  dr;
        begin
            sel_div = bf ? bd : {dr, 2'b00};
        end
    endfunction

    // 单次运算并校验 (含 busy/done 时序 + 双模式标志)
    task run_case(input [10:0] d, input bf, input [10:0] bd, input [8:0] dr,
                  input [11:0] lt, input [11:0] lr);
        integer busy_cycles;
        reg exp_inv;
        begin
            @(negedge clk);
            defect_delta_q2 = d;
            bottom_found = bf;
            bottom_delta_q2 = bd;
            d_ref        = dr;
            l_test_mm    = lt;
            l_ref_mm     = lr;
            start        = 1'b1;
            @(negedge clk);
            start = 1'b0;
            exp_inv = (sel_div(bf, bd, dr) == 0);

            busy_cycles = 0;
            while (!done) begin
                @(negedge clk);
                if (busy) busy_cycles = busy_cycles + 1;
                if (busy_cycles > 100) begin
                    errors = errors + 1;
                    $display("[%0t] FAIL: d=%0d bf=%0b 超时未完成", $time, d, bf);
                    $finish;
                end
            end

            if (distance_mm !== exp_dist(d, bf, bd, dr, lt, lr)) begin
                errors = errors + 1;
                $display("[%0t] FAIL: d=%0d bf=%0b bd=%0d dr=%0d = %0d, 期望 %0d",
                         $time, d, bf, bd, dr, distance_mm,
                         exp_dist(d, bf, bd, dr, lt, lr));
            end
            if (low_confidence !== ~bf) begin
                errors = errors + 1;
                $display("[%0t] FAIL: low_confidence=%0b, 期望 %0b (bf=%0b)",
                         $time, low_confidence, ~bf, bf);
            end
            if (dist_invalid !== exp_inv) begin
                errors = errors + 1;
                $display("[%0t] FAIL: dist_invalid=%0b, 期望 %0b",
                         $time, dist_invalid, exp_inv);
            end
            if (!exp_inv && (busy_cycles != 22)) begin
                errors = errors + 1;
                // busy 实际持续 23 拍; start 后下一 negedge 起采样为 22。
                $display("[%0t] FAIL: busy sampled %0d cycles, exp 22",
                         $time, busy_cycles);
            end
            if (exp_inv && (busy_cycles != 0)) begin
                errors = errors + 1;
                $display("[%0t] FAIL: 除零应立即 done, busy=%0d", $time, busy_cycles);
            end
            @(negedge clk);
            if (done) begin
                errors = errors + 1;
                $display("[%0t] FAIL: done 不是单周期脉冲", $time);
            end
        end
    endtask

    integer i;
    reg [8:0] rd, rb;
    reg       rf;
    initial begin
        errors       = 0;
        start        = 1'b0;
        defect_delta_q2 = 11'd0;
        bottom_found = 1'b0;
        bottom_delta_q2 = 11'd0;
        d_ref        = 9'd0;
        l_test_mm    = 12'd0;
        l_ref_mm     = 12'd0;
        rst_n        = 1'b0;
        repeat (5) @(negedge clk);
        rst_n = 1'b1;
        repeat (5) @(negedge clk);

        // ---- 模式A (高置信): 用 bottom_delta / l_test_mm ----
        // d_ref/l_ref_mm 故意填干扰值, 验证不会被误用
        run_case(11'd168, 1'b1, 11'd332, 9'd50, 12'd1000, 12'd500); // 506
        run_case(11'd332, 1'b1, 11'd332, 9'd50, 12'd1000, 12'd500); // 1000
        run_case(11'd0,   1'b1, 11'd332, 9'd50, 12'd1000, 12'd500); // 0
        run_case(11'd4,   1'b1, 11'd332, 9'd50, 12'd1000, 12'd500); // 12
        run_case(11'd400, 1'b1, 11'd332, 9'd50, 12'd1000, 12'd500); // clamp
        run_case(11'd892, 1'b1, 11'd332, 9'd50, 12'd1000, 12'd500); // clamp
        run_case(11'd160, 1'b1, 11'd320, 9'd50, 12'd980,  12'd500); // 490

        // ---- 模式B (低置信): 用 d_ref / l_ref_mm ----
        // bottom_delta/l_test_mm 故意填干扰值
        run_case(11'd168,  1'b0, 11'd800, 9'd83, 12'd2000, 12'd1000); // 506
        run_case(11'd332,  1'b0, 11'd800, 9'd83, 12'd2000, 12'd1000); // 1000
        run_case(11'd400,  1'b0, 11'd800, 9'd83, 12'd2000, 12'd1000); // clamp
        run_case(11'd2044, 1'b0, 11'd800, 9'd1,  12'd2000, 12'd4095); // clamp

        // ---- 除零: 模式A bottom_delta=0 / 模式B d_ref=0 ----
        run_case(11'd168, 1'b1, 11'd0,   9'd83, 12'd1000, 12'd1000);
        run_case(11'd168, 1'b0, 11'd332, 9'd0,  12'd1000, 12'd1000);
        // 除零后正常运算恢复 (不锁死)
        run_case(11'd168, 1'b1, 11'd332, 9'd83, 12'd1000, 12'd1000);

        // ---- 随机 200 组 (随机模式/除数/被除数) ----
        for (i = 0; i < 200; i = i + 1) begin
            rd = $random;
            rb = $random;
            rf = $random;
            if (rb == 0) rb = 9'd83;
            run_case({rd, 2'b00}, rf, {rb, 2'b00}, rb,
                     12'd1000, 12'd1000);
        end

        if (errors == 0)
            $display("==== tb_defect_distance_calc: ALL PASS ====");
        else
            $display("==== tb_defect_distance_calc: %0d ERRORS ====", errors);
        $finish;
    end

    initial begin
        #10_000_000;
        $display("==== tb_defect_distance_calc: TIMEOUT ====");
        $finish;
    end

endmodule
