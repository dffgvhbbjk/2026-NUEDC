// ============================================================================
// Testbench: tb_vga_status_overlay
// 验证 vga_status_overlay (冻结版方案 §8 三态结果显示):
//   1. WAIT 场景: 无结果 → 无标记线, 右栏白色 "----"
//   2. DEFECT 高置信场景: 红色缺陷整高线 / 黄色主冲击短线 / 紫色棒底虚线
//      像素位置与数量精确匹配 x = 144 + (预触发+索引)×2 (256点模式)
//      右栏 DEFECT/DIST 红色, BTM(有棒底)/CONF HIGH 绿色
//   3. NORMAL 场景: 无红线, 主冲击/棒底线保留, 右栏绿色 NORMAL
//   4. bottom_delta 83→90: 紫线移位 + end_bcd 刷新 (层次引用)
//   5. INVALID 场景: 无任何标记线, 右栏黄色 RETRY (§5.5)
//   6. BCD 转换: dist=506 → 0506, delta=42 → 042 (层次引用)
//   7. 透传: 无叠加区域 rgb_out == wave_rgb
// 扫描方式: 模拟 VGA 逐行扫描 (行末 8 拍消隐), 计数按 3 拍延迟对齐
// ============================================================================
`timescale 1ns/1ps

module tb_vga_status_overlay;

    reg clk   = 1'b0;
    reg rst_n = 1'b0;
    always #12.5 clk = ~clk;          // 40MHz

    reg  [9:0]  pix_x = 10'd0;
    reg  [9:0]  pix_y = 10'd0;
    reg         rgb_valid = 1'b0;
    reg  [11:0] wave_rgb = 12'h123;   // 波形层常量, 用于透传检查
    reg  [1:0]  cap_state = 2'd3;     // HOLD
    reg         display_en = 1'b1;
    reg  [1:0]  disp_len_sel = 2'b10; // 256 点
    reg  [1:0]  len_sel_cfg  = 2'b10;
    reg         smooth_on = 1'b0;
    reg         result_ready = 1'b0;
    reg  [1:0]  result_state = 2'd0;   // 0=INVALID 1=NORMAL 2=DEFECT
    reg  [1:0]  confidence   = 2'd0;
    reg         bottom_found = 1'b0;
    reg  [8:0]  defect_delta = 9'd0;
    reg  [11:0] distance_mm  = 12'd0;
    reg  [8:0]  impact_index = 9'd0;
    reg  [8:0]  defect_index = 9'd0;
    reg  [8:0]  bottom_delta = 9'd83;  // 固化 D_REF
    wire [11:0] rgb_out;

    vga_status_overlay dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .pix_x        (pix_x),
        .pix_y        (pix_y),
        .rgb_valid    (rgb_valid),
        .wave_rgb     (wave_rgb),
        .cap_state    (cap_state),
        .display_en   (display_en),
        .disp_len_sel (disp_len_sel),
        .len_sel_cfg  (len_sel_cfg),
        .smooth_on    (smooth_on),
        .result_ready (result_ready),
        .result_state (result_state),
        .confidence   (confidence),
        .bottom_found (bottom_found),
        .defect_delta (defect_delta),
        .distance_mm  (distance_mm),
        .impact_index (impact_index),
        .defect_index (defect_index),
        .bottom_delta (bottom_delta),
        .rgb_out      (rgb_out)
    );

    // ---- 3 拍延迟坐标 (与 DUT 流水线对齐) ----
    reg [9:0] px_d1, px_d2, px_d3, py_d1, py_d2, py_d3;
    reg       v_d1, v_d2, v_d3;
    always @(posedge clk) begin
        px_d1 <= pix_x;  py_d1 <= pix_y;  v_d1 <= rgb_valid;
        px_d2 <= px_d1;  py_d2 <= py_d1;  v_d2 <= v_d1;
        px_d3 <= px_d2;  py_d3 <= py_d2;  v_d3 <= v_d2;
    end

    // ---- 像素计数器 ----
    integer cnt_red_col;      // 缺陷线列红色像素
    integer cnt_yel_col;      // 主冲击线列黄色像素
    integer cnt_pur_col;      // 棒底线列紫色像素
    integer cnt_red_wave_bad; // 波形区其他列的红色像素 (应为 0)
    integer cnt_red_right;    // 右栏红色文字像素
    integer cnt_green_right;  // 右栏绿色文字像素
    integer cnt_yel_right;    // 右栏黄色文字像素 (RETRY/LOW)
    integer cnt_white_right;  // 右栏白色文字像素
    integer cnt_white_bottom; // 底栏白色文字像素
    integer cnt_pass_bad;     // 透传检查失败像素
    reg [9:0] exp_def_x, exp_imp_x, exp_end_x;
    reg       chk_en = 1'b0;
    integer   err = 0;

    always @(posedge clk) begin
        if (chk_en && v_d3) begin
            if (rgb_out == 12'hF00) begin
                if (px_d3 == exp_def_x && py_d3 >= 30 && py_d3 <= 569)
                    cnt_red_col = cnt_red_col + 1;
                else if (px_d3 >= 672 && px_d3 < 784)
                    cnt_red_right = cnt_red_right + 1;
                else if (px_d3 >= 144 && px_d3 < 656)
                    cnt_red_wave_bad = cnt_red_wave_bad + 1;
            end
            if (rgb_out == 12'hFF0 && px_d3 == exp_imp_x)
                cnt_yel_col = cnt_yel_col + 1;
            if (rgb_out == 12'hFF0 && px_d3 >= 672 && px_d3 < 784)
                cnt_yel_right = cnt_yel_right + 1;
            if (rgb_out == 12'hF0F && px_d3 == exp_end_x)
                cnt_pur_col = cnt_pur_col + 1;
            if (rgb_out == 12'h0F0 && px_d3 >= 672 && px_d3 < 784)
                cnt_green_right = cnt_green_right + 1;
            if (rgb_out == 12'hFFF && px_d3 >= 672 && px_d3 < 784)
                cnt_white_right = cnt_white_right + 1;
            if (rgb_out == 12'hFFF && py_d3 >= 584 && py_d3 < 592)
                cnt_white_bottom = cnt_white_bottom + 1;
            // 透传: x=400 列 y=240~260 无任何叠加内容
            if (px_d3 == 10'd400 && py_d3 >= 240 && py_d3 <= 260 &&
                rgb_out !== wave_rgb) begin
                cnt_pass_bad = cnt_pass_bad + 1;
            end
        end
    end

    task clear_counts;
        begin
            cnt_red_col      = 0;
            cnt_yel_col      = 0;
            cnt_pur_col      = 0;
            cnt_red_wave_bad = 0;
            cnt_red_right    = 0;
            cnt_green_right  = 0;
            cnt_yel_right    = 0;
            cnt_white_right  = 0;
            cnt_white_bottom = 0;
            cnt_pass_bad     = 0;
        end
    endtask

    // ---- 全帧扫描 (行末 8 拍消隐) ----
    integer sx, sy, sg;
    task scan_frame;
        begin
            for (sy = 0; sy < 600; sy = sy + 1) begin
                for (sx = 0; sx < 800; sx = sx + 1) begin
                    @(negedge clk);
                    pix_x     <= sx[9:0];
                    pix_y     <= sy[9:0];
                    rgb_valid <= 1'b1;
                end
                for (sg = 0; sg < 8; sg = sg + 1) begin
                    @(negedge clk);
                    pix_x     <= 10'd0;
                    pix_y     <= 10'd0;
                    rgb_valid <= 1'b0;
                end
            end
            repeat (8) @(negedge clk);
        end
    endtask

    task check_eq;
        input [8*20-1:0] name;
        input integer got, exp;
        begin
            if (got !== exp) begin
                err = err + 1;
                $display("  [FAIL] %0s: got %0d, expect %0d", name, got, exp);
            end else
                $display("  [PASS] %0s = %0d", name, got);
        end
    endtask

    task check_gt0;
        input [8*20-1:0] name;
        input integer got;
        begin
            if (got <= 0) begin
                err = err + 1;
                $display("  [FAIL] %0s: got %0d, expect >0", name, got);
            end else
                $display("  [PASS] %0s = %0d (>0)", name, got);
        end
    endtask

    // 棒底虚线期望像素数: y in [30,569] 且 y[2]==0
    integer exp_pur, ey;

    initial begin
        exp_pur = 0;
        for (ey = 30; ey <= 569; ey = ey + 1)
            if ((ey % 8) < 4) exp_pur = exp_pur + 1;

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);

        // ================= 场景 1: WAIT (无结果) =================
        $display("=== S1: WAIT (result_ready=0) ===");
        result_ready = 1'b0;
        exp_def_x = 10'd298; exp_imp_x = 10'd214; exp_end_x = 10'd380;
        clear_counts;
        chk_en = 1'b1;
        scan_frame;
        chk_en = 1'b0;
        check_eq ("S1 red line",    cnt_red_col, 0);
        check_eq ("S1 yellow line", cnt_yel_col, 0);
        check_eq ("S1 purple line", cnt_pur_col, 0);
        check_eq ("S1 red stray",   cnt_red_wave_bad, 0);
        check_gt0("S1 white right", cnt_white_right);   // WAIT + "----"
        check_gt0("S1 white bottom",cnt_white_bottom);  // AUTO MODE/HOLD 等
        check_eq ("S1 passthru bad",cnt_pass_bad, 0);

        // ================= 场景 2: DEFECT 高置信 =================
        // 256点/预触发32: imp=3→x=214, def=45→x=298, end=3+83→x=380
        $display("=== S2: DEFECT delta=42 dist=506 conf=HIGH ===");
        result_ready = 1'b1;
        result_state = 2'd2;           // RES_DEFECT
        confidence   = 2'd2;           // 高置信 (有棒底)
        bottom_found = 1'b1;
        defect_delta = 9'd42;
        distance_mm  = 12'd506;
        impact_index = 9'd3;
        defect_index = 9'd45;
        repeat (40) @(negedge clk);    // 等 BCD 转换器刷新 (≤26 拍)
        if (dut.dist_bcd !== 16'h0506) begin
            err = err + 1;
            $display("  [FAIL] dist_bcd: got %h, expect 0506", dut.dist_bcd);
        end else
            $display("  [PASS] dist_bcd = %h", dut.dist_bcd);
        if (dut.delta_bcd !== 12'h042) begin
            err = err + 1;
            $display("  [FAIL] delta_bcd: got %h, expect 042", dut.delta_bcd);
        end else
            $display("  [PASS] delta_bcd = %h", dut.delta_bcd);

        clear_counts;
        chk_en = 1'b1;
        scan_frame;
        chk_en = 1'b0;
        check_eq ("S2 red line",    cnt_red_col, 540);  // y=30~569 整高
        check_eq ("S2 yellow line", cnt_yel_col, 81);   // Y_MID±40
        check_eq ("S2 purple line", cnt_pur_col, exp_pur);
        check_eq ("S2 red stray",   cnt_red_wave_bad, 0);
        check_gt0("S2 red right",   cnt_red_right);     // DEFECT + 0506
        check_gt0("S2 green right", cnt_green_right);   // BTM 绿 + CONF HIGH
        check_eq ("S2 yel right",   cnt_yel_right, 0);
        check_eq ("S2 passthru bad",cnt_pass_bad, 0);

        // ================= 场景 3: NORMAL =================
        // delta=80 (仅显示), 无缺陷线; imp/end 线保留
        $display("=== S3: NORMAL delta=80 ===");
        result_state = 2'd1;           // RES_NORMAL
        defect_delta = 9'd80;
        distance_mm  = 12'd0;
        defect_index = 9'd86;
        repeat (40) @(negedge clk);
        clear_counts;
        chk_en = 1'b1;
        scan_frame;
        chk_en = 1'b0;
        check_eq ("S3 red line",    cnt_red_col, 0);    // def 线关闭
        check_eq ("S3 yellow line", cnt_yel_col, 81);
        check_eq ("S3 purple line", cnt_pur_col, exp_pur);
        check_eq ("S3 red stray",   cnt_red_wave_bad, 0);
        check_eq ("S3 red right",   cnt_red_right, 0);
        check_gt0("S3 green right", cnt_green_right);   // NORMAL 绿色
        check_eq ("S3 passthru bad",cnt_pass_bad, 0);

        // ================= 场景 4: 棒底间隔 83→90 (紫线跟随) =================
        // imp=3, end=3+90; 256点模式 x=144+(32+93)*2=394
        $display("=== S4: bottom_delta 83 -> 90 ===");
        bottom_delta = 9'd90;
        exp_end_x = 10'd394;
        repeat (40) @(negedge clk);
        if (dut.end_bcd !== 12'h090) begin
            err = err + 1;
            $display("  [FAIL] end_bcd: got %h, expect 090", dut.end_bcd);
        end else
            $display("  [PASS] end_bcd = %h", dut.end_bcd);
        clear_counts;
        chk_en = 1'b1;
        scan_frame;
        chk_en = 1'b0;
        check_eq ("S4 purple line", cnt_pur_col, exp_pur);

        // ================= 场景 5: INVALID → RETRY (§5.5) =================
        // 无任何标记线 (mark_valid=0), 右栏黄色 RETRY, CONF "---"
        $display("=== S5: INVALID (RETRY) ===");
        result_state = 2'd0;           // RES_INVALID
        confidence   = 2'd0;
        bottom_found = 1'b0;
        repeat (40) @(negedge clk);
        clear_counts;
        chk_en = 1'b1;
        scan_frame;
        chk_en = 1'b0;
        check_eq ("S5 red line",    cnt_red_col, 0);
        check_eq ("S5 yellow line", cnt_yel_col, 0);
        check_eq ("S5 purple line", cnt_pur_col, 0);
        check_eq ("S5 red stray",   cnt_red_wave_bad, 0);
        check_eq ("S5 red right",   cnt_red_right, 0);
        check_eq ("S5 green right", cnt_green_right, 0);
        check_gt0("S5 yel right",   cnt_yel_right);      // RETRY 黄色
        check_eq ("S5 passthru bad",cnt_pass_bad, 0);

        // ================= 汇总 =================
        if (err == 0)
            $display("=== tb_vga_status_overlay: ALL PASS ===");
        else
            $display("=== tb_vga_status_overlay: %0d FAIL ===", err);
        $finish;
    end

endmodule
