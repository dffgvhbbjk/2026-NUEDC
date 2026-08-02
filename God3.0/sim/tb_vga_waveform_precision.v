`timescale 1ns / 1ps
// ============================================================================
// Testbench: tb_vga_waveform_precision
// 对象: vga_waveform_precision + wave_buffer_dp 显示链
//
// 测试项 (审查建议第 5 条):
//   0. display_en=0: 全屏无波形绿色, 消隐期 RGB=0 (rgb_valid_d2 对齐)
//   1. 256 点映射 (2px/点) + 正负方向数值锚点 (+12800→y200, -12800→y400)
//   2. 64 点映射 (8px/点) + bank1 + origin=200 bank 内回绕
//   3. 128 点映射 (4px/点) + ×16 增益上下饱和 (钳位 31/568, 不绕回)
//   4. 显示稳定性: 显示 bank0 期间向 bank1 写入 (模拟第二次锤击采集),
//      两帧逐像素比对必须完全一致
//
// 校验方法: TB 复刻 RAM 读延迟/像素映射/增益缩放数学 → exp_y_col[512],
//   按 3 拍显示延迟逐像素检查:
//   - 必须绿: (x,exp_y) 处必须输出 12'h0F0 (验证映射/延迟/方向/增益)
//   - 杂散绿: 绿色只允许出现在 [min(ey,epy)-1, max(ey,epy)+1] 竖线窗口内
//     (epy = 前一点 Y, 仅点切换后前 2 列; 第 0 点禁止连线)
// ============================================================================
module tb_vga_waveform_precision;

    // ---- 时钟/复位 ----
    reg clk;
    reg rst_n;
    initial clk = 1'b0;
    always #12.5 clk = ~clk;    // 40MHz

    // ---- 显示区域常量 (与 DUT 一致) ----
    localparam X_LEFT = 144;
    localparam X_RIGHT = 656;
    localparam Y_TOP  = 30;
    localparam Y_BTM  = 569;
    localparam Y_MID  = 300;
    localparam H_TOT  = 1056;   // 800x600@60 行总长
    localparam V_TOT  = 628;    // 场总行数

    // ---- VGA 扫描发生器 (模拟 vga_ctrl_simple 的行为) ----
    reg        scan_en;
    reg [10:0] hcnt;
    reg [9:0]  vcnt;
    always @(posedge clk) begin
        if (!scan_en) begin
            hcnt <= 11'd0;
            vcnt <= 10'd0;
        end else if (hcnt == H_TOT - 1) begin
            hcnt <= 11'd0;
            vcnt <= (vcnt == V_TOT - 1) ? 10'd0 : vcnt + 1'b1;
        end else
            hcnt <= hcnt + 1'b1;
    end
    wire [9:0] pix_x     = hcnt[9:0];
    wire [9:0] pix_y     = vcnt;
    wire       rgb_valid = (hcnt < 11'd800) && (vcnt < 10'd600);

    // ---- 显示配置 ----
    reg        cfg_disp_en;
    reg [1:0]  cfg_len;
    reg        cfg_bank;
    reg [7:0]  cfg_origin;
    reg [2:0]  cfg_gain;

    // ---- RAM 写口 (稳定性测试用) ----
    reg               wr_en_r;
    reg  [8:0]        wr_addr_r;
    reg signed [15:0] wr_data_r;

    // bank1 后台写入进程 (writer_en=1 时每 ~21 clk 写一个随机数, Verilog-2001 无 join_any)
    reg       writer_en;
    reg [7:0] wcnt;
    reg [4:0] wgap;
    always @(posedge clk) begin
        if (!writer_en) begin
            wr_en_r <= 1'b0;
            wgap    <= 5'd0;
        end else if (wgap == 5'd0) begin
            wr_en_r   <= 1'b1;
            wr_addr_r <= {1'b1, wcnt};
            wr_data_r <= $random;
            wcnt      <= wcnt + 1'b1;
            wgap      <= 5'd20;
        end else begin
            wr_en_r <= 1'b0;
            wgap    <= wgap - 1'b1;
        end
    end

    // ---- DUT ----
    wire [8:0]         rd_addr;
    wire signed [15:0] rd_data;
    wire [11:0]        rgb;

    wave_buffer_dp u_buf (
        .clk     (clk),
        .wr_en   (wr_en_r),
        .wr_addr (wr_addr_r),
        .wr_data (wr_data_r),
        .rd_addr (rd_addr),
        .rd_data (rd_data)
    );

    vga_waveform_precision u_dut (
        .vga_clk      (clk),
        .rst_n        (rst_n),
        .rd_addr      (rd_addr),
        .rd_data      (rd_data),
        .display_en   (cfg_disp_en),
        .disp_origin  ({cfg_bank, cfg_origin}),
        .disp_len_sel (cfg_len),
        .len_sel_cfg  (cfg_len),
        .gain_shift   (cfg_gain),
        .gain_is_auto (1'b0),
        .pix_x        (pix_x),
        .pix_y        (pix_y),
        .rgb_valid    (rgb_valid),
        .rgb          (rgb)
    );

    // ------------------------------------------------------------------------
    // 影子模型: 复刻 DUT 纵坐标数学 (增益/四舍五入/钳位)
    // ------------------------------------------------------------------------
    function [9:0] calc_y;
        input signed [15:0] s;
        input [2:0]         g;
        reg signed [20:0] ext, scl, bias, spx;
        reg signed [21:0] yc;
        begin
            ext  = s;
            scl  = ext <<< g;
            bias = scl[20] ? ((21'sd1 <<< 7) - 21'sd1) : (21'sd1 <<< 6);
            spx  = (scl + bias) >>> 7;
            yc   = 22'sd300 - spx;
            if (yc < 31)       calc_y = 10'd31;
            else if (yc > 568) calc_y = 10'd568;
            else               calc_y = yc[9:0];
        end
    endfunction

    // 列 → 采样点序号 (与 DUT idx 一致)
    function [7:0] pt_of;
        input [9:0] xs;
        input [1:0] len;
        begin
            pt_of = (len == 2'b00) ? {2'd0, xs[8:3]} :
                    (len == 2'b01) ? {1'd0, xs[8:2]} : xs[8:1];
        end
    endfunction

    // 点内列号 (前 2 列 = 连线窗口)
    function [2:0] col_of;
        input [9:0] xs;
        input [1:0] len;
        begin
            col_of = (len == 2'b00) ? xs[2:0] :
                     (len == 2'b01) ? {1'b0, xs[1:0]} : {2'b00, xs[0]};
        end
    endfunction

    // 每列期望 Y (prep_expect 填充)
    reg [9:0] exp_y_col [0:511];

    task prep_expect;
        integer xs;
        reg [7:0] pt;
        reg [8:0] a;
        begin
            for (xs = 0; xs < 512; xs = xs + 1) begin
                pt = pt_of(xs[9:0], cfg_len);
                a  = {cfg_bank, cfg_origin + pt};
                exp_y_col[xs] = calc_y(u_buf.ram[a], cfg_gain);
            end
        end
    endtask

    // ------------------------------------------------------------------------
    // 逐像素检查 (3 拍显示延迟对齐)
    // ------------------------------------------------------------------------
    reg        check_en;
    reg [1:0]  fmode;          // 0=常规检查 1=记录帧 2=比对帧
    reg [9:0]  x_d1, x_d2, x_d3, y_d1, y_d2, y_d3;
    reg        v_d1, v_d2, v_d3;
    integer    errors;
    reg [11:0] fmap [0:479999];   // 800x600 帧缓存 (稳定性测试)

    always @(posedge clk) begin
        x_d1 <= pix_x;  y_d1 <= pix_y;  v_d1 <= rgb_valid;
        x_d2 <= x_d1;   y_d2 <= y_d1;   v_d2 <= v_d1;
        x_d3 <= x_d2;   y_d3 <= y_d2;   v_d3 <= v_d2;
    end

    reg [9:0] ey, epy, lo, hi;
    reg [9:0] xs_c;
    reg [7:0] pt_c;
    reg [2:0] col_c;
    integer   fidx;

    always @(negedge clk) begin
        if (rst_n && check_en) begin
            if (!v_d3) begin
                // 消隐期: RGB 必须为 0 (验证 rgb_valid 两拍对齐)
                if (rgb !== 12'h000) begin
                    errors = errors + 1;
                    if (errors < 20)
                        $display("[%0t] FAIL: 消隐期 rgb=%h (x=%0d y=%0d)",
                                 $time, rgb, x_d3, y_d3);
                end
            end else begin
                // ---- 帧记录/比对 (稳定性测试) ----
                fidx = y_d3 * 800 + x_d3;
                if (fmode == 2'd1)
                    fmap[fidx] = rgb;
                else if (fmode == 2'd2) begin
                    if (fmap[fidx] !== rgb) begin
                        errors = errors + 1;
                        if (errors < 20)
                            $display("[%0t] FAIL: 帧比对不一致 (x=%0d y=%0d) %h→%h",
                                     $time, x_d3, y_d3, fmap[fidx], rgb);
                    end
                end

                // ---- 波形绿色检查 ----
                if (cfg_disp_en && x_d3 >= X_LEFT && x_d3 < X_RIGHT &&
                    y_d3 >= Y_TOP && y_d3 <= Y_BTM) begin
                    xs_c  = x_d3 - X_LEFT;
                    pt_c  = pt_of(xs_c, cfg_len);
                    col_c = col_of(xs_c, cfg_len);
                    ey    = exp_y_col[xs_c];
                    // 连线窗口: 点切换后前 2 列, 第 0 点无连线
                    if (col_c < 3'd2 && pt_c != 8'd0)
                        epy = exp_y_col[xs_c - {7'd0, col_c} - 10'd1];
                    else
                        epy = ey;
                    lo = ((ey < epy) ? ey : epy) - 10'd1;
                    hi = ((ey > epy) ? ey : epy) + 10'd1;
                    // 必须绿: 期望轨迹上的像素
                    if (y_d3 == ey && rgb !== 12'h0F0) begin
                        errors = errors + 1;
                        if (errors < 20)
                            $display("[%0t] FAIL: (x=%0d,y=%0d) 期望波形绿, 实际 %h",
                                     $time, x_d3, y_d3, rgb);
                    end
                    // 杂散绿: 窗口外不允许出现绿色
                    if (rgb === 12'h0F0 && (y_d3 < lo || y_d3 > hi)) begin
                        errors = errors + 1;
                        if (errors < 20)
                            $display("[%0t] FAIL: (x=%0d,y=%0d) 杂散绿色 (窗口 %0d~%0d)",
                                     $time, x_d3, y_d3, lo, hi);
                    end
                end else begin
                    // 波形区外 / display_en=0: 不允许波形绿色
                    if (rgb === 12'h0F0) begin
                        errors = errors + 1;
                        if (errors < 20)
                            $display("[%0t] FAIL: 波形区外出现绿色 (x=%0d y=%0d)",
                                     $time, x_d3, y_d3);
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // 帧扫描任务: 对齐帧起点后完整检查一帧
    // ------------------------------------------------------------------------
    task scan_frame(input [1:0] mode);
        begin
            @(posedge clk);
            while (!(hcnt == 11'd0 && vcnt == 10'd0)) @(posedge clk);
            repeat (4) @(posedge clk);      // 冲刷 3 拍流水
            fmode    = mode;
            check_en = 1'b1;
            repeat (H_TOT * V_TOT) @(posedge clk);
            check_en = 1'b0;
            fmode    = 2'd0;
        end
    endtask

    // RAM 直接预载 (层次引用)
    task load_ram_word(input [8:0] a, input signed [15:0] d);
        begin
            u_buf.ram[a] = d;
        end
    endtask

    // ------------------------------------------------------------------------
    // 激励序列
    // ------------------------------------------------------------------------
    integer i;

    initial begin
        rst_n       = 1'b0;
        scan_en     = 1'b0;
        check_en    = 1'b0;
        fmode       = 2'd0;
        cfg_disp_en = 1'b0;
        cfg_len     = 2'b10;
        cfg_bank    = 1'b0;
        cfg_origin  = 8'd0;
        cfg_gain    = 3'd0;
        wr_en_r     = 1'b0;
        wr_addr_r   = 9'd0;
        wr_data_r   = 16'sd0;
        writer_en   = 1'b0;
        wcnt        = 8'd0;
        errors      = 0;

        // 全 RAM 预载伪随机数据
        for (i = 0; i < 512; i = i + 1)
            load_ram_word(i[8:0], $random);

        repeat (10) @(posedge clk);
        rst_n   = 1'b1;
        scan_en = 1'b1;
        repeat (10) @(posedge clk);

        // ====================================================================
        // 测试0: display_en=0 → 全屏无绿, 消隐期 RGB=0
        // ====================================================================
        $display("========== 测试0: display_en=0 + 消隐对齐 ==========");
        scan_frame(2'd0);
        $display("[%0t] 测试0 完成", $time);

        // ====================================================================
        // 测试1: 256 点 (2px/点), bank0, origin=0, ×1 + 方向锚点
        // ====================================================================
        $display("========== 测试1: 256 点映射 + 正负方向锚点 ==========");
        for (i = 0; i < 256; i = i + 1)
            load_ram_word(i[8:0], -16'sd20000 + i * 16'sd160);  // 满幅斜坡
        load_ram_word(9'd10, 16'sd12800);    // 锚点 A: +12800 → y=200 (上方)
        load_ram_word(9'd20, -16'sd12800);   // 锚点 B: -12800 → y=400 (下方)
        cfg_disp_en = 1'b1;  cfg_len = 2'b10;
        cfg_bank = 1'b0;     cfg_origin = 8'd0;  cfg_gain = 3'd0;
        prep_expect;
        // 独立数值锚点 (不依赖影子模型公式对错的硬编码检查)
        if (exp_y_col[20] !== 10'd200) begin
            errors = errors + 1;
            $display("FAIL: 正样本 +12800 期望 y=200, 影子模型 = %0d", exp_y_col[20]);
        end
        if (exp_y_col[40] !== 10'd400) begin
            errors = errors + 1;
            $display("FAIL: 负样本 -12800 期望 y=400, 影子模型 = %0d", exp_y_col[40]);
        end
        scan_frame(2'd0);
        $display("[%0t] 测试1 完成", $time);

        // ====================================================================
        // 测试2: 64 点 (8px/点), bank1, origin=200 (bank 内回绕)
        // ====================================================================
        $display("========== 测试2: 64 点 + bank1 + origin 回绕 ==========");
        for (i = 0; i < 256; i = i + 1)
            load_ram_word(9'd256 + i[8:0], ((i * 37) % 20000) - 16'sd10000);
        cfg_len = 2'b00;  cfg_bank = 1'b1;  cfg_origin = 8'd200;  cfg_gain = 3'd0;
        prep_expect;
        scan_frame(2'd0);
        $display("[%0t] 测试2 完成", $time);

        // ====================================================================
        // 测试3: 128 点 (4px/点), ×16 增益上下饱和
        // ====================================================================
        $display("========== 测试3: 128 点 + x16 增益饱和 ==========");
        for (i = 0; i < 128; i = i + 1) begin
            case (i % 4)
                0: load_ram_word(8'd100 + i[7:0],  16'sd3000);   // ×16 → 上饱和
                1: load_ram_word(8'd100 + i[7:0],  16'sd500);    // ×16 → 不饱和
                2: load_ram_word(8'd100 + i[7:0], -16'sd3000);   // ×16 → 下饱和
                3: load_ram_word(8'd100 + i[7:0], -16'sd500);
            endcase
        end
        cfg_len = 2'b01;  cfg_bank = 1'b0;  cfg_origin = 8'd100;  cfg_gain = 3'd4;
        prep_expect;
        // 饱和锚点: +3000×16 钳位到 Y_TOP+1=31, -3000×16 钳位到 Y_BTM-1=568
        if (exp_y_col[0] !== 10'd31) begin
            errors = errors + 1;
            $display("FAIL: 上饱和期望钳位 y=31, 影子模型 = %0d", exp_y_col[0]);
        end
        if (exp_y_col[8] !== 10'd568) begin
            errors = errors + 1;
            $display("FAIL: 下饱和期望钳位 y=568, 影子模型 = %0d", exp_y_col[8]);
        end
        scan_frame(2'd0);
        $display("[%0t] 测试3 完成", $time);

        // ====================================================================
        // 测试4: 显示稳定性 — 显示 bank0 时向 bank1 写入 (模拟二次锤击采集)
        // ====================================================================
        $display("========== 测试4: 二次锤击期间旧波形稳定 ==========");
        cfg_len = 2'b10;  cfg_bank = 1'b0;  cfg_origin = 8'd0;  cfg_gain = 3'd0;
        prep_expect;
        scan_frame(2'd1);                        // 记录基准帧
        writer_en = 1'b1;                        // 开启 bank1 后台写入
        scan_frame(2'd2);                        // 比对帧 (期间 bank1 被持续改写)
        writer_en = 1'b0;
        $display("[%0t] 测试4 完成", $time);

        // ====================================================================
        repeat (20) @(posedge clk);
        $display("");
        if (errors == 0)
            $display("=============== 全部测试 PASS ===============");
        else
            $display("=============== FAIL: %0d 个错误 ===============", errors);
        $finish;
    end

    // 全局超时保护
    initial begin
        #400_000_000;
        $display("FAIL: 仿真全局超时");
        $finish;
    end

endmodule
