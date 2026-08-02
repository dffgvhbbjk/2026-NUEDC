// ============================================================================
// tb_pcm1808_i2s_rx — PCM1808 I2S 接收器自校验仿真 (文档 §17.1)
//
// 测试内容:
//   1. 边界码测试: 0x000000, 0x000001, 0x7FFFFF, 0x800000, 0x800001, 0xFFFFFF
//   2. 固定码: 0x123456, 0xABCDEF
//   3. 随机码连续 1000 帧
//   4. 复位在帧中间拉低, 验证恢复
//   5. LRCLK 初始电平分别为 0 和 1
//
// 自动检查:
//   - 接收值 == 发送值
//   - 每帧 left_valid == 1 次
//   - 每帧 right_valid == 1 次
//   - 不存在额外 valid
//
// 仿真器: ModelSim-Altera / Intel FPGA Simulator / 任何标准 Verilog 仿真器
// 用法: 编译 rtl/pcm1808_i2s_rx.v 和本文件, 运行仿真
// ============================================================================
`timescale 1ns / 1ps

module tb_pcm1808_i2s_rx;

    // ---- 时钟参数 ----
    localparam BCLK_PERIOD = 163;      // 6.136MHz → ~163ns
    localparam BCLK_HALF   = BCLK_PERIOD / 2;

    // ---- DUT 信号 ----
    reg         bclk;
    reg         rst_n;
    reg         lrclk;
    reg         sdin;
    wire [23:0] left_data;
    wire [23:0] right_data;
    wire        left_valid;
    wire        right_valid;
    wire        frame_error;

    // ---- DUT 实例 ----
    pcm1808_i2s_rx #(
        .DATA_WIDTH (24),
        .I2S_MODE   (1)
    ) DUT (
        .rst_n       (rst_n),
        .bclk        (bclk),
        .lrclk       (lrclk),
        .sdin        (sdin),
        .left_data   (left_data),
        .right_data  (right_data),
        .left_valid  (left_valid),
        .right_valid (right_valid),
        .frame_error (frame_error)
    );

    // ---- BCLK 生成 ----
    initial bclk = 0;
    always #(BCLK_HALF) bclk = ~bclk;

    // ---- LRCLK 生成 (negedge bclk 翻转, 每 32 BCLK) ----
    reg [5:0] lrclk_div;
    always @(negedge bclk) begin
        if (!rst_n) begin
            lrclk_div <= 5'd0;
            lrclk     <= 1'b0;
        end else if (lrclk_div == 5'd31) begin
            lrclk_div <= 5'd0;
            lrclk     <= ~lrclk;
        end else begin
            lrclk_div <= lrclk_div + 1'b1;
        end
    end

    // ---- 错误计数与 valid 监控 (声明在前, 供 task 访问) ----
    integer errors;
    integer lv_count;   // 每帧 left_valid 计数
    integer rv_count;   // 每帧 right_valid 计数

    // ---- I2S 发送一帧 (左+右) ----
    // 必须等待 LRCLK 的真正边沿 (1→0 进入 left, 0→1 进入 right)
    // 否则 TB 发送与 DUT 的 slot_count 会对不齐
    //
    // 标准 I2S 时序 (NXP UM11732: WS 比 MSB 提前一个完整位时钟改变):
    //   N0: LRCLK 在 negedge 翻转 (此周期即 1 位延迟位)
    //   N1: 发送端在下一个 negedge 移出 MSB
    //   P1: 接收端在随后的 posedge 采样 MSB (DUT slot_count=0)
    // 修复: 旧版在边沿后额外空等一个 negedge 才发 MSB, 与 DUT 犯了
    //       同样的晚一位错误, 导致错误 DUT 也能 PASS. 现按标准时序驱动.
    task send_i2s_frame;
        input [23:0] l_val;
        input [23:0] r_val;
        integer i;
        begin
            // ---- 等待 LRCLK 真正下降沿 (1→0, 进入 left) ----
            // 先确保在 right (lrclk=1), 再等到 left (lrclk=0)
            @(posedge bclk);
            while (lrclk !== 1'b1) @(posedge bclk);   // 先等到 right
            @(posedge bclk);
            while (lrclk !== 1'b0) @(posedge bclk);   // 等到 left (真正下降沿)

            // LRCLK 刚切换到 left, DUT 在此 posedge 检测到 lrclk_edge
            //   DUT: slot_count <= 0, channel <= 0
            // 前一帧 left_valid 早已结束, 此处清零 left_valid 计数
            lv_count = 0;

            // 边沿后第 1 个 negedge 起依次驱动 MSB..LSB
            //   (DUT 在 slot_count=0..23 的 posedge 采样)
            for (i = 23; i >= 0; i = i - 1) begin
                @(negedge bclk);
                sdin = l_val[i];
            end

            // ---- 等待 LRCLK 真正上升沿 (0→1, 进入 right) ----
            @(posedge bclk);
            while (lrclk !== 1'b1) @(posedge bclk);

            // 清零 right_valid 计数 (前一帧 right_valid 早已结束)
            rv_count = 0;

            for (i = 23; i >= 0; i = i - 1) begin
                @(negedge bclk);
                sdin = r_val[i];
            end
        end
    endtask

    // ---- 错误计数与 valid 监控 ----
    // 在 posedge bclk 监控 valid 脉冲
    always @(posedge bclk) begin
        if (left_valid)
            lv_count = lv_count + 1;
        if (right_valid)
            rv_count = rv_count + 1;
    end

    // ---- 检查接收结果 ----
    task check_frame;
        input [23:0] exp_l;
        input [23:0] exp_r;
        input [255:0] tag;
        begin
            // 等待 DUT 处理完成 (一帧 = 64 BCLK, 给足余量)
            #(BCLK_PERIOD * 10);

            if (left_data !== exp_l) begin
                $display("FAIL [%0s] left: got %h, exp %h", tag, left_data, exp_l);
                errors = errors + 1;
            end
            if (right_data !== exp_r) begin
                $display("FAIL [%0s] right: got %h, exp %h", tag, right_data, exp_r);
                errors = errors + 1;
            end
            if (lv_count !== 1) begin
                $display("FAIL [%0s] left_valid count: %0d (exp 1)", tag, lv_count);
                errors = errors + 1;
            end
            if (rv_count !== 1) begin
                $display("FAIL [%0s] right_valid count: %0d (exp 1)", tag, rv_count);
                errors = errors + 1;
            end
        end
    endtask

    // ---- 主测试流程 ----
    integer i;
    reg [23:0] rand_l, rand_r;

    initial begin
        // 初始化
        rst_n   = 0;
        lrclk   = 0;
        sdin    = 0;
        lrclk_div = 0;
        errors  = 0;
        lv_count = 0;
        rv_count = 0;

        // 复位
        #(BCLK_PERIOD * 5);
        @(negedge bclk);
        rst_n = 1;
        #(BCLK_PERIOD * 2);

        // ============================================================
        // 测试 1: 边界码
        // ============================================================
        $display("=== Test 1: Boundary codes ===");

        lv_count = 0; rv_count = 0;
        send_i2s_frame(24'h000000, 24'h000001);
        check_frame(24'h000000, 24'h000001, "boundary 0/1");

        lv_count = 0; rv_count = 0;
        send_i2s_frame(24'h7FFFFF, 24'h800000);
        check_frame(24'h7FFFFF, 24'h800000, "boundary 7F/80");

        lv_count = 0; rv_count = 0;
        send_i2s_frame(24'h800001, 24'hFFFFFF);
        check_frame(24'h800001, 24'hFFFFFF, "boundary 80/FF");

        // ============================================================
        // 测试 2: 固定码
        // ============================================================
        $display("=== Test 2: Fixed codes ===");
        lv_count = 0; rv_count = 0;
        send_i2s_frame(24'h123456, 24'hABCDEF);
        check_frame(24'h123456, 24'hABCDEF, "fixed 123/ABC");

        // ============================================================
        // 测试 3: 随机码 1000 帧
        // ============================================================
        $display("=== Test 3: Random 1000 frames ===");
        for (i = 0; i < 1000; i = i + 1) begin
            rand_l = $random;
            rand_r = $random;
            lv_count = 0; rv_count = 0;
            send_i2s_frame(rand_l, rand_r);
            check_frame(rand_l, rand_r, "random");

            if ((i + 1) % 200 == 0)
                $display("  ... %0d frames done", i + 1);
        end

        // ============================================================
        // 测试 4: 复位在帧中间, 验证恢复
        // ============================================================
        $display("=== Test 4: Mid-frame reset ===");
        // 开始发送一帧, 在中途拉低复位
        fork
            begin
                send_i2s_frame(24'hA5A5A5, 24'h5A5A5A);
            end
            begin
                #(BCLK_PERIOD * 15);
                rst_n = 0;
                lrclk = 0;
                lrclk_div = 0;
                #(BCLK_PERIOD * 3);
                @(negedge bclk);
                rst_n = 1;
            end
        join

        // 复位后重新发送验证恢复
        #(BCLK_PERIOD * 10);
        lv_count = 0; rv_count = 0;
        send_i2s_frame(24'hDEAD24, 24'hCAFE24);
        check_frame(24'hDEAD24, 24'hCAFE24, "post-reset");

        // ============================================================
        // 测试 5: LRCLK 初始电平 1
        // ============================================================
        $display("=== Test 5: LRCLK initial high ===");
        rst_n = 0;
        lrclk = 1;
        lrclk_div = 0;
        sdin = 0;
        lv_count = 0; rv_count = 0;
        #(BCLK_PERIOD * 5);
        @(negedge bclk);
        rst_n = 1;
        #(BCLK_PERIOD * 2);

        send_i2s_frame(24'h112233, 24'h445566);
        check_frame(24'h112233, 24'h445566, "lrclk init high");

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
        #(BCLK_PERIOD * 200000);
        $display("TIMEOUT");
        $finish;
    end

endmodule
