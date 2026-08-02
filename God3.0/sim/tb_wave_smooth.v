`timescale 1ns / 1ps
// ============================================================================
// Testbench: tb_wave_smooth
// 对象: wave_smooth 可旁路三点平滑 (文档 §14.2, P2)
//
// 测试项:
//   1. bypass=1: 输出 == 输入延迟 1 样点 (原值不变), trigger 同步延迟
//   2. bypass=0: 输出 == (x[n-1] + 2x[n] + x[n+1] + 2) >>> 2, trigger 对齐
//   3. 极值 ±(2^23-1) 交替: 中间和不溢出, 输出仍为凸组合
//   4. 复位后重新填充: 前 2 个输入样点不产生输出
//   5. (贯穿全部段) 原始随动通道: out_raw == 中心样点的 in_raw 原值,
//      两种模式下均不受平滑影响
//
// 影子模型: TB 保存输入序列 xs[], 第 m 个输出对应中心样点 x[m+1]
//   (前 2 个样点填充移位寄存器, 第 3 个输入起每输入 1 点输出 1 点)
// ============================================================================
module tb_wave_smooth;

    reg clk, rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;    // 40MHz

    reg                bypass;
    reg  signed [23:0] in_data;
    reg  signed [23:0] in_raw;
    reg                in_valid;
    reg                in_trigger;
    wire signed [23:0] out_data;
    wire signed [23:0] out_raw;
    wire               out_valid;
    wire               out_trigger;

    wave_smooth u_dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .bypass      (bypass),
        .in_data     (in_data),
        .in_raw      (in_raw),
        .in_valid    (in_valid),
        .in_trigger  (in_trigger),
        .out_data    (out_data),
        .out_raw     (out_raw),
        .out_valid   (out_valid),
        .out_trigger (out_trigger)
    );

    // ------------------------------------------------------------------------
    // 影子模型
    // ------------------------------------------------------------------------
    reg  signed [23:0] xs   [0:1023];   // 输入历史
    reg  signed [23:0] raws [0:1023];   // 输入 raw 历史
    reg                trgs [0:1023];   // 输入 trigger 历史
    integer            sent, ocnt, errors;
    reg                mode_smooth;     // 当前段模式 (0=bypass, 1=平滑)

    // 期望输出: 中心样点 x[m+1]
    function signed [23:0] exp_out;
        input integer m;
        reg signed [25:0] s;
        begin
            if (mode_smooth) begin
                s = {{2{xs[m][23]}},   xs[m]}
                  + {{1{xs[m+1][23]}}, xs[m+1], 1'b0}
                  + {{2{xs[m+2][23]}}, xs[m+2]}
                  + 26'sd2;
                exp_out = s[25:2];
            end else
                exp_out = xs[m+1];
        end
    endfunction

    // 输出校验 (out_valid 拍)
    always @(posedge clk) begin
        if (rst_n && out_valid) begin
            if (out_data !== exp_out(ocnt)) begin
                errors = errors + 1;
                if (errors < 20)
                    $display("[%0t] FAIL: out[%0d] = %h, 期望 %h (mode=%0d)",
                             $time, ocnt, out_data, exp_out(ocnt), mode_smooth);
            end
            if (out_raw !== raws[ocnt+1]) begin
                errors = errors + 1;
                if (errors < 20)
                    $display("[%0t] FAIL: out_raw[%0d] = %h, 期望 %h (原始通道被改动)",
                             $time, ocnt, out_raw, raws[ocnt+1]);
            end
            if (out_trigger !== trgs[ocnt+1]) begin
                errors = errors + 1;
                $display("[%0t] FAIL: out_trigger[%0d] = %b, 期望 %b (触发点漂移)",
                         $time, ocnt, out_trigger, trgs[ocnt+1]);
            end
            ocnt = ocnt + 1;
        end
    end

    // out_valid 只能跟随 in_valid 出现 (每输入至多 1 个输出)
    // (由 ocnt 与输入数的关系在各段末尾检查)

    // ---- 发送样点 (raw 通道由 val 确定性派生, 与 in_data 不同值) ----
    task send(input signed [23:0] val, input trg);
        begin
            @(negedge clk);
            in_data    = val;
            in_raw     = val ^ 24'hA5C3F0;
            in_valid   = 1'b1;
            in_trigger = trg;
            xs[sent]   = val;
            raws[sent] = val ^ 24'hA5C3F0;
            trgs[sent] = trg;
            sent = sent + 1;
            @(negedge clk);
            in_valid   = 1'b0;
            in_trigger = 1'b0;
            repeat (8) @(posedge clk);
        end
    endtask

    // ---- 段复位: DUT 复位 + 影子计数清零 ----
    task restart(input smooth);
        begin
            @(negedge clk);
            rst_n = 1'b0;
            repeat (4) @(posedge clk);
            @(negedge clk);
            mode_smooth = smooth;
            bypass      = ~smooth;
            sent        = 0;
            ocnt        = 0;
            rst_n       = 1'b1;
            repeat (4) @(posedge clk);
        end
    endtask

    integer i;
    reg signed [23:0] rv;

    initial begin
        rst_n       = 1'b0;
        bypass      = 1'b1;
        in_data     = 24'sd0;
        in_raw      = 24'sd0;
        in_valid    = 1'b0;
        in_trigger  = 1'b0;
        mode_smooth = 1'b0;
        errors      = 0;
        sent = 0; ocnt = 0;

        // ============ 测试1: bypass 随机 200 点 ============
        $display("========== 测试1: bypass 原始模式 ==========");
        restart(1'b0);
        for (i = 0; i < 200; i = i + 1) begin
            rv = $random;
            send(rv, (i == 57) || (i == 130));   // 两个 trigger 标记点
        end
        if (ocnt != 198) begin
            errors = errors + 1;
            $display("FAIL: bypass 输出数 = %0d, 期望 198 (200 输入 - 2 填充)", ocnt);
        end

        // ============ 测试2: 三点平滑 随机 200 点 ============
        $display("========== 测试2: 三点平滑 ==========");
        restart(1'b1);
        for (i = 0; i < 200; i = i + 1) begin
            rv = $random;
            send(rv, (i == 33));
        end
        if (ocnt != 198) begin
            errors = errors + 1;
            $display("FAIL: 平滑输出数 = %0d, 期望 198", ocnt);
        end

        // ============ 测试3: 极值交替 (溢出防护) ============
        $display("========== 测试3: 极值防溢出 ==========");
        restart(1'b1);
        for (i = 0; i < 40; i = i + 1)
            send((i & 1) ? -24'sd8388607 : 24'sd8388607, 1'b0);
        for (i = 0; i < 40; i = i + 1)
            send((i & 1) ? 24'sd8388607 : -24'sd8388608, 1'b0);

        // ============ 测试4: 复位后重新填充 ============
        $display("========== 测试4: 复位重填充 ==========");
        restart(1'b0);
        send(24'sd111, 1'b0);
        send(24'sd222, 1'b0);
        repeat (10) @(posedge clk);
        if (ocnt != 0) begin
            errors = errors + 1;
            $display("FAIL: 前 2 个填充样点不应产生输出 (ocnt=%0d)", ocnt);
        end
        send(24'sd333, 1'b0);      // 第 3 个输入 → 输出 x[1]=222
        repeat (10) @(posedge clk);
        if (ocnt != 1) begin
            errors = errors + 1;
            $display("FAIL: 第 3 个输入后应有 1 个输出 (ocnt=%0d)", ocnt);
        end

        repeat (20) @(posedge clk);
        $display("");
        if (errors == 0)
            $display("=============== 全部测试 PASS ===============");
        else
            $display("=============== FAIL: %0d 个错误 ===============", errors);
        $finish;
    end

    initial begin
        #10_000_000;
        $display("FAIL: 仿真全局超时");
        $finish;
    end

endmodule
