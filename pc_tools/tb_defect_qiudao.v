`timescale 1ns/1ps
// ============================================================================
// testbench: 用真实 CSV 数据驱动 defect_qiudao_classifier, 检查 B2 距离输出
// 用法: iverilog -g2012 -o tb.out defect_qiudao_classifier.v tb_defect_qiudao.v
//       vvp tb.out
// ============================================================================
module tb_defect_qiudao;

    reg                clk = 0;
    reg                rst_n = 0;
    reg                wr_en = 0;
    reg  [9:0]         wr_addr = 0;
    reg  signed [23:0] wr_raw = 0;
    reg                cap_done = 0;
    reg  [9:0]         disp_origin = 0;
    reg  [1:0]         disp_len_sel = 0;
    reg                rearm = 0;

    wire               result_valid;
    wire [1:0]         result_state;
    wire [1:0]         confidence;
    wire [2:0]         defect_class;
    wire [11:0]        distance_mm;
    wire [8:0]         impact_index;
    wire               busy;
    wire [3:0]         dbg_state;

    // 顶层被测试模块
    defect_qiudao_classifier uut (
        .clk          (clk),
        .rst_n        (rst_n),
        .wr_en        (wr_en),
        .wr_addr      (wr_addr),
        .wr_raw       (wr_raw),
        .cap_done     (cap_done),
        .disp_origin  (disp_origin),
        .disp_len_sel (disp_len_sel),
        .rearm        (rearm),
        .result_valid (result_valid),
        .result_state (result_state),
        .confidence   (confidence),
        .defect_class (defect_class),
        .distance_mm  (distance_mm),
        .impact_index (impact_index),
        .busy         (busy),
        .dbg_state    (dbg_state)
    );

    // 波形样本 (从 CSV 生成, 512 点)
    reg signed [23:0] samples [0:1023];
    integer           n_samples;

    // 从 $readmemh 读入样本 (每行一个十六进制)
    initial begin
        n_samples = 512;
        $readmemh("tb_samples.hex", samples);
    end

    always #5 clk = ~clk;   // 100MHz

    integer i;

    // 逐拍跟踪 chk_cnt 88~152 区间
    always @(posedge clk) begin
        if (uut.chk_cnt >= 88 && uut.chk_cnt <= 152) begin
            $display("[%0t] chk=%0d edge=%0d e_cnt=%0d e1=%0d e2=%0d valid_i=%0d dist=%0d qiudao=%0d q_prev=%0d",
                     $time, uut.chk_cnt, uut.qiudao_edge, uut.edge_after_cnt,
                     uut.e1_after_pos, uut.e2_after_pos, uut.result_valid_i,
                     uut.result_dist, uut.qiudao_panjue, uut.qiudao_prev);
        end
    end

    initial begin
        $dumpfile("tb_defect_qiudao.vcd");
        $dumpvars(0, tb_defect_qiudao);

        // 复位
        #20 rst_n = 1;

        // 写 512 个样本到影子 RAM (bank0, addr 0..511)
        for (i = 0; i < n_samples; i = i + 1) begin
            @(posedge clk);
            wr_en   <= 1;
            wr_addr <= {1'b0, i[8:0]};
            wr_raw  <= samples[i];
        end
        @(posedge clk);
        wr_en <= 0;

        // 触发采集完成
        @(posedge clk);
        disp_origin  <= 10'd0;   // bank0, origin 0
        disp_len_sel <= 2'b11;   // 512 点
        cap_done     <= 1'b1;
        @(posedge clk);
        cap_done <= 1'b0;

        // 等结果
        wait (result_valid == 1'b1);
        #20;
        $display("===== RESULT =====");
        $display("result_state = %0d (1=NORMAL 2=DEFECT 0=INVALID)", result_state);
        $display("distance_mm  = %0d", distance_mm);
        $display("impact_index = %0d", impact_index);
        $display("confidence   = %0d", confidence);
        // 内部调试信号 (层级引用)
        $display("---- 内部信号 ----");
        $display("imp           = %0d", uut.imp);
        $display("win_edge_cnt  = %0d  (阈值 %0d)", uut.win_edge_cnt, uut.EDGE_THRESHOLD);
        $display("e1_after_pos  = %0d", uut.e1_after_pos);
        $display("e2_after_pos  = %0d", uut.e2_after_pos);
        $display("edge_after_cnt= %0d", uut.edge_after_cnt);
        $display("result_dist   = %0d  (chk-imp-3 = %0d)", uut.result_dist, uut.e2_after_pos - uut.imp - 3);
        $display("peak_mag      = %0d", uut.peak_mag);
        $finish;
    end

endmodule
