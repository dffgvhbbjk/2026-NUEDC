`timescale 1ns/1ns
//======================================================================
// tb_downsample.v — 验证降采样 64×48 硬边界裁剪修复
//
// 背景: 摄像头分辨率寄存器未生效, 实际输出 ~1280×960 (经采集模块
//       约束后 pix_x 0-1023, pix_y 0-599), 原 pix_x<800 网格每行出
//       80 采样 → 缓冲溢出 → 花屏。新增 x_div10<64 && y_div10<48
//       强制只取前 64×48 网格点。
//
// 验证:
//   A. 1280×960 输入 (pix_x 0-1023, pix_y 0-599): 应恰好 3072 采样,
//      ds_pix_x 0-63, ds_pix_y 0-47, y_data 透传正确
//   B. 640×480 输入 (pix_x 0-639, pix_y 0-479): 应恰好 3072 采样, 完整图像
//======================================================================
module tb_downsample;

    reg        pclk = 0;
    reg        rst_n = 0;
    reg  [7:0] y_data = 0;
    reg        y_valid = 0;
    reg  [9:0] pix_x = 0;
    reg  [9:0] pix_y = 0;
    wire [7:0] ds_y_data;
    wire       ds_y_valid;
    wire [6:0] ds_pix_x;
    wire [5:0] ds_pix_y;

    always #10 pclk = ~pclk;   // 50MHz (仅仿真)

    downsample u_dut (
        .pclk       (pclk),
        .rst_n      (rst_n),
        .y_data     (y_data),
        .y_valid    (y_valid),
        .pix_x      (pix_x),
        .pix_y      (pix_y),
        .ds_y_data  (ds_y_data),
        .ds_y_valid (ds_y_valid),
        .ds_pix_x   (ds_pix_x),
        .ds_pix_y   (ds_pix_y)
    );

    //---- 统计/检查 ----
    integer ds_count = 0;
    integer ds_min_x = 999, ds_max_x = 0;
    integer ds_min_y = 999, ds_max_y = 0;
    integer bad_data = 0;

    always @(posedge pclk) begin
        if (ds_y_valid) begin
            ds_count = ds_count + 1;
            if (ds_pix_x < ds_min_x) ds_min_x = ds_pix_x;
            if (ds_pix_x > ds_max_x) ds_max_x = ds_pix_x;
            if (ds_pix_y < ds_min_y) ds_min_y = ds_pix_y;
            if (ds_pix_y > ds_max_y) ds_max_y = ds_pix_y;
            // y_data 透传 + 位置验证: 采样点 pix_x = ds_pix_x*10, y_data = pix_x & 0xFF
            if (ds_y_data !== ((ds_pix_x * 10) & 8'hFF))
                bad_data = bad_data + 1;
        end
    end

    //---- 生成一帧像素流 (模拟采集模块输出) ----
    task gen_frame(input integer max_x, input integer max_y);
        integer x, y;
        begin
            for (y = 0; y <= max_y; y = y + 1) begin
                for (x = 0; x <= max_x; x = x + 1) begin
                    @(posedge pclk);
                    y_valid = 1'b1;
                    pix_x   = x;
                    pix_y   = y;
                    y_data  = x & 8'hFF;
                end
            end
            @(posedge pclk);
            y_valid = 1'b0;
            // 等流水线输出完
            repeat (10) @(posedge pclk);
        end
    endtask

    //---- 检查并打印 ----
    task check_frame(input [127:0] name, input integer expect_n);
        begin
            $display("--- %s ---", name);
            $display("  采样数 = %0d (期望 %0d)", ds_count, expect_n);
            $display("  ds_pix_x 范围 = %0d ~ %0d (期望 0~63)", ds_min_x, ds_max_x);
            $display("  ds_pix_y 范围 = %0d ~ %0d (期望 0~47)", ds_min_y, ds_max_y);
            $display("  y_data 错误 = %0d", bad_data);
            if (ds_count == expect_n && ds_min_x == 0 && ds_max_x == 63 &&
                ds_min_y == 0 && ds_max_y == 47 && bad_data == 0)
                $display("  [PASS] %s", name);
            else
                $display("  [FAIL] %s", name);
        end
    endtask

    //---- 主流程 ----
    initial begin
        $display("=============================================");
        $display(" tb_downsample: 64×48 硬边界裁剪验证");
        $display("=============================================");
        #200 rst_n = 1;

        // ---- 测试 A: 1280×960 输入 (采集约束后 pix_x 0-1023, pix_y 0-599) ----
        ds_count = 0; ds_min_x = 999; ds_max_x = 0; ds_min_y = 999; ds_max_y = 0; bad_data = 0;
        gen_frame(1023, 599);
        check_frame("A: 1280x960 (应裁剪为左上64x48, 3072采样)", 3072);

        // 复位, 测试 B: 640×480 输入
        #100 rst_n = 0; #100 rst_n = 1;
        ds_count = 0; ds_min_x = 999; ds_max_x = 0; ds_min_y = 999; ds_max_y = 0; bad_data = 0;
        gen_frame(639, 479);
        check_frame("B: 640x480 (应完整64x48, 3072采样)", 3072);

        $finish;
    end

endmodule
