`timescale 1ns/1ps
// ============================================================================
// tb_defect_ac_classifier - 新缺陷分类器 RTL 仿真, 逐样本比对 Python 定点预期
//
//   数据来源: tools/fpga_model.py 导出的 sim/lda_vectors/hit_NN.memh (512×24位)
//   预期结果: sim/lda_vectors/manifest.txt (此处按顺序硬编码为期望数组)
//
//   流程: 每个向量 -> 以 bank0/org0 顺序写入影子RAM(wr_en/wr_addr/wr_raw)
//         -> cap_done 脉冲(disp_origin=0, len_sel=11) -> 等 result_valid
//         -> 比对 result_state/defect_class 与期望
// ============================================================================
module tb_defect_ac_classifier;

    localparam integer NVEC = 12;

    reg               clk = 1'b0;
    reg               rst_n = 1'b0;
    reg               wr_en = 1'b0;
    reg  [9:0]        wr_addr = 10'd0;
    reg signed [23:0] wr_raw = 24'd0;
    reg               cap_done = 1'b0;
    reg  [9:0]        disp_origin = 10'd0;
    reg  [1:0]        disp_len_sel = 2'b11;
    reg               rearm = 1'b0;

    wire              result_valid;
    wire [1:0]        result_state;
    wire [1:0]        confidence;
    wire [2:0]        defect_class;
    wire [11:0]       distance_mm;
    wire [8:0]        impact_index;
    wire              busy;
    wire [3:0]        dbg_state;

    // 期望结果 (与 manifest.txt 顺序一致)
    // state: 0=INVALID 1=NORMAL 2=DEFECT ; class: 0..4
    reg [1:0] exp_state [0:NVEC-1];
    reg [2:0] exp_class [0:NVEC-1];

    integer pass_cnt, fail_cnt;

    defect_ac_classifier dut (
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

    always #12.5 clk = ~clk;    // 40MHz

    reg signed [23:0] vec_mem [0:511];
    reg [8*128:1] fname;
    integer i;

    // 载入一个向量并跑一次分析, 返回时结果已稳定
    task run_vector(input integer n);
        begin
            // 写影子RAM (bank0/org0 -> addr=pt)
            @(posedge clk);
            for (i = 0; i < 512; i = i + 1) begin
                wr_en   <= 1'b1;
                wr_addr <= i[9:0];
                wr_raw  <= vec_mem[i];
                @(posedge clk);
            end
            wr_en <= 1'b0;
            @(posedge clk);
            // 触发分析
            disp_origin  <= 10'd0;
            disp_len_sel <= 2'b11;
            cap_done     <= 1'b1;
            @(posedge clk);
            cap_done <= 1'b0;
            // 等结果
            wait (result_valid == 1'b1);
            @(posedge clk);
        end
    endtask

    task check_vector(input integer n);
        begin
            if (result_state === exp_state[n] &&
                (exp_state[n] != 2'd2 || defect_class === exp_class[n])) begin
                $display("[PASS] vec %0d: state=%0d class=%0d dist=%0d conf=%0d imp=%0d",
                         n, result_state, defect_class, distance_mm, confidence, impact_index);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("[FAIL] vec %0d: got state=%0d class=%0d ; exp state=%0d class=%0d",
                         n, result_state, defect_class, exp_state[n], exp_class[n]);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    integer v;
    initial begin
        // 期望 (manifest.txt):
        // 00 NORMAL c0 | 01 NORMAL c0 | 02 DEFECT c1 | 03 DEFECT c1
        // 04 DEFECT c2 | 05 DEFECT c2 | 06 DEFECT c3 | 07 DEFECT c3
        // 08 DEFECT c4 | 09 DEFECT c4 | 10 INVALID    | 11 INVALID
        exp_state[0]=2'd1; exp_class[0]=3'd0;
        exp_state[1]=2'd1; exp_class[1]=3'd0;
        exp_state[2]=2'd2; exp_class[2]=3'd1;
        exp_state[3]=2'd2; exp_class[3]=3'd1;
        exp_state[4]=2'd2; exp_class[4]=3'd2;
        exp_state[5]=2'd2; exp_class[5]=3'd2;
        exp_state[6]=2'd2; exp_class[6]=3'd3;
        exp_state[7]=2'd2; exp_class[7]=3'd3;
        exp_state[8]=2'd2; exp_class[8]=3'd4;
        exp_state[9]=2'd2; exp_class[9]=3'd4;
        exp_state[10]=2'd0; exp_class[10]=3'd0;
        exp_state[11]=2'd0; exp_class[11]=3'd0;

        pass_cnt = 0; fail_cnt = 0;
        rst_n = 1'b0;
        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        for (v = 0; v < NVEC; v = v + 1) begin
            $sformat(fname, "sim/lda_vectors/hit_%0d%0d.memh", v/10, v%10);
            $readmemh(fname, vec_mem);
            run_vector(v);
            check_vector(v);
        end

        $display("==== defect_ac_classifier TB: PASS=%0d FAIL=%0d / %0d ====",
                 pass_cnt, fail_cnt, NVEC);
        if (fail_cnt == 0) $display("RESULT: ALL PASS");
        else               $display("RESULT: HAS FAILURES");
        $finish;
    end

    // 看门狗: 防止死锁
    initial begin
        #50_000_000;   // 50ms 仿真上限
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
