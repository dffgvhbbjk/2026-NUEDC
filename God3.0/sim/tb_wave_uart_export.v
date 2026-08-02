`timescale 1ns / 1ps
// ============================================================================
// Testbench: tb_wave_uart_export
// 对象: wave_uart_export (P2 + 冻结版 §12.7 版本化结果段)
//
// 测试项:
//   1. 64 点帧 (bank0, origin=0): 帧头 AA 55 + len + 192 数据字节 + 校验和
//      + 帧尾 14 字节版本化结果段 (ver=0x02), 数据与影子 RAM 逐字节比对
//   2. 64 点帧回绕 (bank1, origin=200): 地址 {1, (200+k) mod 256},
//      DEFECT 高置信结果段 (含索引高位 bit / 触发阈值)
//   3. 导出期间 cap_done 被忽略 (帧计数不增加, 当前帧不受损)
//   4. 256 点帧: 总长 4+768+14 字节, 校验和正确, NORMAL 结果段
//
// TB UART 接收: 2Mbps (bit 周期 500ns), 起始位下降沿 + 半位对准采样
// ============================================================================
module tb_wave_uart_export;

    reg clk, rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;    // 40MHz

    localparam BIT_NS = 500;    // 2Mbps

    reg                wr_en;
    reg  [8:0]         wr_addr;
    reg  signed [23:0] wr_raw;
    reg                cap_done;
    reg  [8:0]         disp_origin;
    reg  [1:0]         disp_len_sel;
    reg                result_ready;
    reg  [1:0]         result_state;
    reg  [1:0]         confidence;
    reg  [8:0]         impact_index;
    reg  [8:0]         defect_index;
    reg  [8:0]         bottom_index;
    reg  [11:0]        distance_mm;
    reg  [23:0]        trigger_threshold;
    wire               busy;
    wire               uart_txd;

    wave_uart_export #(
        .UART_BPS (2_000_000)      // 仿真加速: 硬件 115200
    ) u_dut (
        .clk               (clk),
        .rst_n             (rst_n),
        .wr_en             (wr_en),
        .wr_addr           (wr_addr),
        .wr_raw            (wr_raw),
        .cap_done          (cap_done),
        .disp_origin       (disp_origin),
        .disp_len_sel      (disp_len_sel),
        .result_ready      (result_ready),
        .result_state      (result_state),
        .confidence        (confidence),
        .impact_index      (impact_index),
        .defect_index      (defect_index),
        .bottom_index      (bottom_index),
        .distance_mm       (distance_mm),
        .trigger_threshold (trigger_threshold),
        .busy              (busy),
        .uart_txd          (uart_txd)
    );

    // ------------------------------------------------------------------------
    // 影子 RAM (与写入序列同步维护)
    // ------------------------------------------------------------------------
    reg signed [23:0] shadow [0:511];
    integer errors;

    task ram_write(input [8:0] a, input signed [23:0] d);
        begin
            @(negedge clk);
            wr_en   = 1'b1;
            wr_addr = a;
            wr_raw  = d;
            shadow[a] = d;
            @(negedge clk);
            wr_en = 1'b0;
        end
    endtask

    // ------------------------------------------------------------------------
    // TB UART 接收器: 等起始位 → 半位对准 → 逐位采样
    // ------------------------------------------------------------------------
    reg [7:0] rx_byte;
    integer bi;
    task recv_byte;
        begin
            @(negedge uart_txd);          // 起始位下降沿
            #(BIT_NS / 2);
            if (uart_txd !== 1'b0) begin
                errors = errors + 1;
                $display("[%0t] FAIL: 起始位异常", $time);
            end
            for (bi = 0; bi < 8; bi = bi + 1) begin
                #(BIT_NS);
                rx_byte[bi] = uart_txd;   // LSB first
            end
            #(BIT_NS);
            if (uart_txd !== 1'b1) begin
                errors = errors + 1;
                $display("[%0t] FAIL: 停止位异常", $time);
            end
        end
    endtask

    // 期望字节比对
    task expect_byte(input [7:0] exp, input [127:0] msg);
        begin
            recv_byte;
            if (rx_byte !== exp) begin
                errors = errors + 1;
                $display("[%0t] FAIL: %0s: 收到 %h, 期望 %h", $time, msg, rx_byte, exp);
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // 帧校验任务: 接收完整一帧并与影子 RAM 比对 (3 字节/点, MSB first)
    // ------------------------------------------------------------------------
    task check_frame(input bank, input [7:0] org, input [1:0] lsel);
        integer n, k;
        reg [8:0]  a;
        reg [7:0]  sum;
        reg signed [23:0] s;
        begin
            n = (lsel == 2'b00) ? 64 : (lsel == 2'b01) ? 128 : 256;
            sum = 8'd0;
            expect_byte(8'hAA, "帧头0");
            expect_byte(8'h55, "帧头1");
            expect_byte({6'd0, lsel}, "长度字段");
            sum = sum + {6'd0, lsel};
            for (k = 0; k < n; k = k + 1) begin
                a = {bank, org + k[7:0]};
                s = shadow[a];
                expect_byte(s[23:16], "样点[23:16]");
                expect_byte(s[15:8],  "样点[15:8]");
                expect_byte(s[7:0],   "样点[7:0]");
                sum = sum + s[23:16] + s[15:8] + s[7:0];
            end
            expect_byte(sum, "校验和");
            check_result_seg;
            $display("[%0t] 帧校验完成: bank%b origin=%0d %0d 点", $time, bank, org, n);
        end
    endtask

    // ------------------------------------------------------------------------
    // 结果段校验: 5A ver status idx_h imp def bot dist_h dist_l th2 th1 th0 rsum
    //   (帧尾 14 字节, ver=0x02; rsum = ver 起 11 字节累加, 不含 5A)
    //   期望值按当前 TB 输入计算 (DUT 在波形段校验和发完时锁存)
    // ------------------------------------------------------------------------
    task check_result_seg;
        reg [7:0] e_st, e_ih, e_im, e_df, e_bt, e_dh, e_d0, e_t2, e_t1, e_t0;
        begin
            e_st = {3'd0, result_ready, confidence, result_state};
            e_ih = {5'd0, bottom_index[8], defect_index[8], impact_index[8]};
            e_im = impact_index[7:0];
            e_df = defect_index[7:0];
            e_bt = bottom_index[7:0];
            e_dh = {4'd0, distance_mm[11:8]};
            e_d0 = distance_mm[7:0];
            e_t2 = trigger_threshold[23:16];
            e_t1 = trigger_threshold[15:8];
            e_t0 = trigger_threshold[7:0];
            expect_byte(8'h5A, "结果段标记");
            expect_byte(8'h02, "版本号");
            expect_byte(e_st,  "status");
            expect_byte(e_ih,  "idx_h");
            expect_byte(e_im,  "impact_index");
            expect_byte(e_df,  "defect_index");
            expect_byte(e_bt,  "bottom_index");
            expect_byte(e_dh,  "dist[11:8]");
            expect_byte(e_d0,  "dist[7:0]");
            expect_byte(e_t2,  "thr[23:16]");
            expect_byte(e_t1,  "thr[15:8]");
            expect_byte(e_t0,  "thr[7:0]");
            expect_byte(8'h02 + e_st + e_ih + e_im + e_df + e_bt
                        + e_dh + e_d0 + e_t2 + e_t1 + e_t0, "结果段校验和");
        end
    endtask

    // 触发导出
    task fire(input [8:0] org, input [1:0] lsel);
        begin
            @(negedge clk);
            disp_origin  = org;
            disp_len_sel = lsel;
            cap_done     = 1'b1;
            @(negedge clk);
            cap_done     = 1'b0;
        end
    endtask

    // ------------------------------------------------------------------------
    // 激励
    // ------------------------------------------------------------------------
    integer i;
    integer t;

    initial begin
        rst_n             = 1'b0;
        wr_en             = 1'b0;
        wr_addr           = 9'd0;
        wr_raw            = 24'sd0;
        cap_done          = 1'b0;
        disp_origin       = 9'd0;
        disp_len_sel      = 2'b00;
        result_ready      = 1'b0;
        result_state      = 2'd0;
        confidence        = 2'd0;
        impact_index      = 9'd0;
        defect_index      = 9'd0;
        bottom_index      = 9'd0;
        distance_mm       = 12'd0;
        trigger_threshold = 24'd0;
        errors            = 0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // 填充两个 bank: bank0 = 递增斜坡(含负值, 24 位量级), bank1 = 伪随机
        for (i = 0; i < 256; i = i + 1)
            ram_write({1'b0, i[7:0]}, -24'sd5000000 + i * 24'sd40157);
        for (i = 0; i < 256; i = i + 1)
            ram_write({1'b1, i[7:0]}, $random);

        // ============ 测试1: 64 点, bank0, origin=0 (无检测结果) ============
        $display("========== 测试1: 64 点帧 (bank0) ==========");
        fire({1'b0, 8'd0}, 2'b00);
        check_frame(1'b0, 8'd0, 2'b00);
        // 帧结束后 busy 应落下
        t = 0;
        while (busy && t < 2000) begin @(posedge clk); t = t + 1; end
        if (busy) begin
            errors = errors + 1;
            $display("FAIL: 帧发送完成后 busy 未落下");
        end

        // ============ 测试2: 64 点回绕, bank1, origin=200 (DEFECT 高置信) ============
        $display("========== 测试2: 64 点回绕帧 (bank1, origin=200) ==========");
        result_ready      = 1'b1;
        result_state      = 2'd2;          // DEFECT
        confidence        = 2'd2;          // 高置信 (有棒底)
        impact_index      = 9'd7;
        defect_index      = 9'd49;
        bottom_index      = 9'd300;        // bit8=1, 验证 idx_h
        distance_mm       = 12'd506;
        trigger_threshold = 24'd120000;    // 0x01D4C0
        fire({1'b1, 8'd200}, 2'b00);
        check_frame(1'b1, 8'd200, 2'b00);

        // ============ 测试3: 导出中 cap_done 被忽略 ============
        $display("========== 测试3: busy 期间 cap_done 忽略 ==========");
        t = 0;
        while (busy && t < 2000) begin @(posedge clk); t = t + 1; end
        fire({1'b0, 8'd10}, 2'b00);
        fork
            // 接收整帧: 当前帧不受第二次 cap_done 影响
            check_frame(1'b0, 8'd10, 2'b00);
            // 帧发到 ~20 字节处再打一次 cap_done (应被忽略)
            begin
                #(BIT_NS * 10 * 20);
                fire({1'b1, 8'd99}, 2'b10);
            end
        join
        // 帧结束后不应自动开始第二帧
        #(BIT_NS * 30);
        if (busy) begin
            errors = errors + 1;
            $display("FAIL: 被忽略的 cap_done 启动了第二帧");
        end

        // ============ 测试4: 256 点帧 (NORMAL 结果) ============
        $display("========== 测试4: 256 点帧 ==========");
        result_ready      = 1'b1;
        result_state      = 2'd1;          // NORMAL
        confidence        = 2'd2;
        impact_index      = 9'd7;
        defect_index      = 9'd0;
        bottom_index      = 9'd87;
        distance_mm       = 12'd0;
        trigger_threshold = 24'd45000;
        fire({1'b1, 8'd0}, 2'b10);
        check_frame(1'b1, 8'd0, 2'b10);

        repeat (100) @(posedge clk);
        $display("");
        if (errors == 0)
            $display("=============== 全部测试 PASS ===============");
        else
            $display("=============== FAIL: %0d 个错误 ===============", errors);
        $finish;
    end

    initial begin
        #50_000_000;    // 50ms 全局超时 (4 帧总时长 ~8ms)
        $display("FAIL: 仿真全局超时");
        $finish;
    end

endmodule
