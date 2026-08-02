// ============================================================================
// tb_fifo_sample_reader — FIFO 读端 q/valid 对齐仿真 (文档 §17.2)
//
// 测试内容:
//   1. 每次只积压 1 个数据 (写 1 → 等读出 → 再写 1)
//   2. 一次积压 16 个数据 (连续写 16 → 等全部读出)
//   3. 真正的读暂停/恢复: rd_en=0 期间写入积压, 验证读端确实停止,
//      rd_en=1 后全部按序恢复读出 (修复旧版"假暂停"测试)
//   4. 快速连续写入 100 个 (压力测试)
//   5. 复位中断: 积压数据后系统复位, 验证 FIFO 被 aclr 清空,
//      复位释放后无残留旧样点输出, 新数据正常收发
//   6. 满 FIFO 恢复: 暂停读并写满 256 深度 + 4 个溢出写,
//      验证 wrfull 置位、溢出写被丢弃且 overflow_cnt 记录,
//      恢复读后 256 个数据完整按序读出
//
// 自动检查:
//   - 输出序列 == 输入序列 (无丢失、无重复、无乱序)
//   - 每次 sample_valid 恰好对应一个 sample_data
//   - 暂停期间 sample_valid 不产生
//   - 复位后 FIFO 无残留
//   - 溢出计数准确
//
// 依赖: altera_mf 库 (dcfifo 原语)
// 仿真器: ModelSim-Altera / Intel FPGA Simulator
// ============================================================================
`timescale 1ns / 1ps

module tb_fifo_sample_reader;

    // ---- 时钟参数 ----
    localparam WRCLK_PERIOD = 163;    // BCLK 6.136MHz
    localparam RDCLK_PERIOD = 25;     // VGA 40MHz

    // ---- DUT 信号 ----
    reg         wrclk;
    reg         rdclk;
    reg         rst_n;
    reg         wr_rst_n;
    reg         rd_en;
    reg  [23:0] wr_data;
    reg         wrreq;
    wire [23:0] sample_data;
    wire        sample_valid;
    wire        fifo_full;
    wire [7:0]  fifo_level;
    wire [7:0]  overflow_cnt;

    // ---- DUT 实例 ----
    fifo_sample_reader #(
        .DATA_WIDTH (24),
        .ADDR_WIDTH (8)
    ) DUT (
        .rdclk        (rdclk),
        .rst_n        (rst_n),
        .wr_rst_n     (wr_rst_n),
        .rd_en        (rd_en),
        .wr_data      (wr_data),
        .wrreq        (wrreq),
        .wrclk        (wrclk),
        .sample_data  (sample_data),
        .sample_valid (sample_valid),
        .fifo_full    (fifo_full),
        .fifo_level   (fifo_level),
        .overflow_cnt (overflow_cnt)
    );

    // ---- 时钟生成 ----
    initial wrclk = 0;
    always #(WRCLK_PERIOD/2) wrclk = ~wrclk;

    initial rdclk = 0;
    always #(RDCLK_PERIOD/2) rdclk = ~rdclk;

    // ---- 持续监控 sample_valid 并缓存到队列 ----
    // 这样无论 check_receive 何时调用, 都不会错过数据
    reg [23:0] collected_data [0:1023];
    integer    collected_count;
    integer    check_index;   // check 进度指针

    always @(posedge rdclk) begin
        if (sample_valid) begin
            collected_data[collected_count] = sample_data;
            collected_count = collected_count + 1;
        end
    end

    // ---- 写任务 ----
    task write_one;
        input [23:0] data;
        begin
            @(posedge wrclk);
            wr_data = data;
            wrreq   = 1'b1;
            @(posedge wrclk);
            wrreq   = 1'b0;
        end
    endtask

    // ---- 连续写 N 个 ----
    task write_burst;
        input integer count;
        input [23:0] base_val;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                @(posedge wrclk);
                wr_data = base_val + i;
                wrreq   = 1'b1;
            end
            @(posedge wrclk);
            wrreq = 1'b0;
        end
    endtask

    // ---- 错误计数 ----
    integer errors;

    // ---- 检查任务: 等待 expected_count 个数据被收集, 然后比对 ----
    task check_receive;
        input integer expected_count;
        input [23:0] base_val;
        integer k;
        reg [23:0] expected;
        integer target_count;
        integer timeout_cnt;
        begin
            // 等待收集到足够的数据
            target_count = check_index + expected_count;
            timeout_cnt = 0;
            while (collected_count < target_count && timeout_cnt < 100000) begin
                @(posedge rdclk);
                timeout_cnt = timeout_cnt + 1;
            end
            if (timeout_cnt >= 100000) begin
                $display("FAIL: timeout waiting for %0d items (got %0d of %0d)",
                         expected_count, collected_count - check_index, expected_count);
                errors = errors + 1;
                // 跳过剩余检查
                check_index = collected_count;
            end else begin
                // 比对收集到的数据
                for (k = 0; k < expected_count; k = k + 1) begin
                    expected = base_val + k;
                    if (collected_data[check_index + k] !== expected) begin
                        $display("FAIL: data[%0d] got %h, exp %h",
                                 check_index + k, collected_data[check_index + k], expected);
                        errors = errors + 1;
                    end
                end
                check_index = check_index + expected_count;
            end
        end
    endtask

    // ---- 主测试流程 ----
    integer i;
    integer paused_count;   // 暂停时刻的收集计数快照

    initial begin
        // 初始化
        rst_n    = 0;
        wr_rst_n = 0;
        rd_en    = 1;
        wr_data  = 0;
        wrreq    = 0;
        errors   = 0;
        collected_count = 0;
        check_index = 0;

        // 复位
        #(RDCLK_PERIOD * 5);
        @(negedge rdclk);
        rst_n    = 1;
        wr_rst_n = 1;
        #(RDCLK_PERIOD * 5);

        // 等待 aclr 释放在读写两域同步完成
        #(WRCLK_PERIOD * 5);

        // ============================================================
        // 测试 1: 每次积压 1 个数据
        // ============================================================
        $display("=== Test 1: One item at a time (10 items) ===");
        collected_count = 0;
        check_index = 0;
        for (i = 0; i < 10; i = i + 1) begin
            write_one(24'h1000 + i);
            check_receive(1, 24'h1000 + i);
        end
        $display("  Test 1 done: %0d items received", collected_count);

        // ============================================================
        // 测试 2: 一次积压 16 个数据
        // ============================================================
        $display("=== Test 2: 16 items backlogged ===");
        collected_count = 0;
        check_index = 0;
        write_burst(16, 24'h2000);
        check_receive(16, 24'h2000);
        $display("  Test 2 done: %0d items received", collected_count);

        // ============================================================
        // 测试 3: 真正的读暂停/恢复 (rd_en=0 → 写入积压 → rd_en=1)
        // ============================================================
        $display("=== Test 3: TRUE read pause / resume ===");
        collected_count = 0;
        check_index = 0;

        // 暂停读端
        @(negedge rdclk);
        rd_en = 1'b0;
        #(RDCLK_PERIOD * 5);
        paused_count = collected_count;

        // 暂停期间写入 32 个数据
        write_burst(32, 24'h3000);

        // 等待 rdusedw 跨域同步稳定
        #(WRCLK_PERIOD * 6);

        // 验证: 暂停期间读端确实没有输出
        if (collected_count !== paused_count) begin
            $display("FAIL: read not paused! collected %0d new items during pause",
                     collected_count - paused_count);
            errors = errors + 1;
        end
        // 验证: 32 个数据全部积压在 FIFO 中
        if (fifo_level !== 8'd32) begin
            $display("FAIL: fifo_level=%0d during pause, exp 32", fifo_level);
            errors = errors + 1;
        end

        // 恢复读端, 验证 32 个数据按序读出
        @(negedge rdclk);
        rd_en = 1'b1;
        check_receive(32, 24'h3000);
        $display("  Test 3 done: %0d items received after resume", collected_count);

        // ============================================================
        // 测试 4: 快速连续写入 100 个数据 (压力测试)
        // ============================================================
        $display("=== Test 4: Burst 100 items (stress) ===");
        collected_count = 0;
        check_index = 0;
        write_burst(100, 24'h4000);
        check_receive(100, 24'h4000);
        $display("  Test 4 done: %0d items received", collected_count);

        // ============================================================
        // 测试 5: 复位中断 — 积压数据后复位, 验证 FIFO 被清空
        // ============================================================
        $display("=== Test 5: Reset clears FIFO (no residual data) ===");
        collected_count = 0;
        check_index = 0;

        // 暂停读端并写入 8 个"旧"数据 (制造残留)
        @(negedge rdclk);
        rd_en = 1'b0;
        write_burst(8, 24'h5A00);
        #(WRCLK_PERIOD * 6);
        if (fifo_level !== 8'd8) begin
            $display("FAIL: pre-reset fifo_level=%0d, exp 8", fifo_level);
            errors = errors + 1;
        end

        // 系统复位 (读写两域同时) → 内部 aclr 清空 FIFO
        rst_n    = 0;
        wr_rst_n = 0;
        #(RDCLK_PERIOD * 5);
        @(negedge rdclk);
        rst_n    = 1;
        wr_rst_n = 1;

        // 等待 aclr 释放同步完成, 恢复读端
        #(WRCLK_PERIOD * 5);
        @(negedge rdclk);
        rd_en = 1'b1;
        collected_count = 0;    // 复位后重新计数
        check_index = 0;

        // 验证: 复位后无残留旧样点输出
        #(RDCLK_PERIOD * 50);
        if (collected_count !== 0) begin
            $display("FAIL: %0d residual items output after reset", collected_count);
            errors = errors + 1;
        end
        if (fifo_level !== 8'd0) begin
            $display("FAIL: post-reset fifo_level=%0d, exp 0", fifo_level);
            errors = errors + 1;
        end

        // 验证: 复位后新数据正常收发
        write_burst(4, 24'h5B00);
        check_receive(4, 24'h5B00);
        $display("  Test 5 done: FIFO cleared by reset, %0d new items OK", collected_count);

        // ============================================================
        // 测试 6: 满 FIFO + 溢出丢弃 + 恢复
        // ============================================================
        $display("=== Test 6: Full FIFO, overflow drop, recovery ===");
        collected_count = 0;
        check_index = 0;

        // 暂停读端, 写入 256 (写满) + 4 (溢出)
        @(negedge rdclk);
        rd_en = 1'b0;
        write_burst(260, 24'h6000);
        #(WRCLK_PERIOD * 6);

        // 验证: wrfull 置位
        if (fifo_full !== 1'b1) begin
            $display("FAIL: fifo_full=%b after 260 writes, exp 1", fifo_full);
            errors = errors + 1;
        end
        // 验证: 4 个溢出写被记录
        if (overflow_cnt !== 8'd4) begin
            $display("FAIL: overflow_cnt=%0d, exp 4", overflow_cnt);
            errors = errors + 1;
        end

        // 恢复读端, 验证 256 个数据完整按序读出 (溢出的 4 个被丢弃)
        @(negedge rdclk);
        rd_en = 1'b1;
        check_receive(256, 24'h6000);

        // 验证: 没有多余数据 (溢出写没有破坏 FIFO 内容)
        #(RDCLK_PERIOD * 50);
        if (collected_count !== 256) begin
            $display("FAIL: total collected %0d, exp exactly 256", collected_count);
            errors = errors + 1;
        end
        if (fifo_level !== 8'd0) begin
            $display("FAIL: post-drain fifo_level=%0d, exp 0", fifo_level);
            errors = errors + 1;
        end
        $display("  Test 6 done: full/overflow/recovery OK, overflow_cnt=%0d", overflow_cnt);

        // ============================================================
        // 结束
        // ============================================================
        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== TESTS FAILED: %0d errors ===", errors);

        $finish;
    end

    // 超时保护
    initial begin
        #(RDCLK_PERIOD * 500000);
        $display("TIMEOUT");
        $finish;
    end

endmodule
