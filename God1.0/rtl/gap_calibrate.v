//======================================================================
// Module: gap_calibrate
// Function: 像素宽度 → 毫米换算
//   gap_mm_x10 = gap_pix × calib_coef / 1000
//   calib_coef = K × 10000 (Q12.4 定点, K 单位 mm/pix)
//   结果 gap_mm_x10 为毫米值 ×10 (例 123 表示 12.3mm)
//
//   3 级流水线 + 加法树 (修复时序违规):
//     原 mul_10x16 for 循环综合为 16 级串联加法器, 路径延迟 48ns,
//     导致 cam_pclk Fmax 仅 20.9MHz (< 24MHz 约束).
//     改为显式加法树 (4 级深度) + 3 级流水线寄存,
//     每级组合路径 < 15ns, 满足 24MHz 乃至 44MHz.
//
//   流水线延迟: 3 拍 (pclk 域)
//     valid → mm_valid 延迟 3 拍, 不影响帧级锁存 (frame_sync 时已稳定)
//
//   用 shift-add 实现, 完全避开 Quartus HDL * 推断 (cbx_lpm_mult.dll 问题)
// Clock:  pclk 域 (与 gap_detect 同源)
//======================================================================
module gap_calibrate (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [9:0]  gap_pix,        // 像素宽度 (0-1023)
    input  wire        valid,          // 有效脉冲
    input  wire [15:0] calib_coef,     // 标定系数 K×10000
    output reg  [15:0] gap_mm_x10,     // 毫米×10
    output reg         mm_valid
);

    //==================================================================
    // 级 1: 输入寄存 + 部分积加法树
    //   16 项移位加法, 拆成 8 对 + 4 组 + 2 半, 加法树深度 4 级
    //   每级 26-bit 加法器 ~3ns, 总共 ~12ns (远小于 41.7ns 周期)
    //==================================================================
    reg [9:0]  a_r;       // gap_pix 寄存
    reg [15:0] b_r;       // calib_coef 寄存
    reg        v_r;       // valid 流水线

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_r <= 10'd0;
            b_r <= 16'd0;
            v_r <= 1'b0;
        end else begin
            a_r <= gap_pix;
            b_r <= calib_coef;
            v_r <= valid;
        end
    end

    // 16 项部分积, 每项 = b_r[i] ? (a_r << i) : 0
    // 两两配对求和 (1 级加法)
    wire [25:0] p00 = (b_r[0]  ? {16'd0, a_r}        : 26'd0)
                    + (b_r[1]  ? {15'd0, a_r, 1'b0}  : 26'd0);
    wire [25:0] p01 = (b_r[2]  ? {14'd0, a_r, 2'b0}  : 26'd0)
                    + (b_r[3]  ? {13'd0, a_r, 3'b0}  : 26'd0);
    wire [25:0] p02 = (b_r[4]  ? {12'd0, a_r, 4'b0}  : 26'd0)
                    + (b_r[5]  ? {11'd0, a_r, 5'b0}  : 26'd0);
    wire [25:0] p03 = (b_r[6]  ? {10'd0, a_r, 6'b0}  : 26'd0)
                    + (b_r[7]  ? {9'd0,  a_r, 7'b0}  : 26'd0);
    wire [25:0] p04 = (b_r[8]  ? {8'd0,  a_r, 8'b0}  : 26'd0)
                    + (b_r[9]  ? {7'd0,  a_r, 9'b0}  : 26'd0);
    wire [25:0] p05 = (b_r[10] ? {6'd0,  a_r, 10'b0} : 26'd0)
                    + (b_r[11] ? {5'd0,  a_r, 11'b0} : 26'd0);
    wire [25:0] p06 = (b_r[12] ? {4'd0,  a_r, 12'b0} : 26'd0)
                    + (b_r[13] ? {3'd0,  a_r, 13'b0} : 26'd0);
    wire [25:0] p07 = (b_r[14] ? {2'd0,  a_r, 14'b0} : 26'd0)
                    + (b_r[15] ? {1'd0,  a_r, 15'b0} : 26'd0);

    // 4 组求和 (1 级加法)
    wire [25:0] q0 = p00 + p01;
    wire [25:0] q1 = p02 + p03;
    wire [25:0] q2 = p04 + p05;
    wire [25:0] q3 = p06 + p07;

    // 2 半求和 (1 级加法)
    wire [25:0] s_lo = q0 + q1;   // 低 8 位部分积
    wire [25:0] s_hi = q2 + q3;   // 高 8 位部分积

    // 级 1 寄存
    reg [25:0] s_lo_r, s_hi_r;
    reg        v_r1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_lo_r <= 26'd0;
            s_hi_r <= 26'd0;
            v_r1   <= 1'b0;
        end else begin
            s_lo_r <= s_lo;
            s_hi_r <= s_hi;
            v_r1   <= v_r;
        end
    end

    //==================================================================
    // 级 2: 合并部分积 → product + scaled 计算
    //   product = s_lo_r + s_hi_r           (1 级 26-bit 加法 ~3ns)
    //   scaled = product×4194 = (p<<12)+(p<<6)+(p<<5)+(p<<1)
    //         两两配对: t0=(p<<12)+(p<<6), t1=(p<<5)+(p<<1)
    //         scaled = t0 + t1              (2 级 39-bit 加法 ~6ns)
    //   总组合路径: 3 级加法 ~9ns
    //==================================================================
    wire [25:0] product = s_lo_r + s_hi_r;

    wire [38:0] t0 = ({13'd0, product} << 12) + ({13'd0, product} << 6);
    wire [38:0] t1 = ({13'd0, product} << 5)  + ({13'd0, product} << 1);
    wire [38:0] scaled = t0 + t1;
    wire [16:0] mm_result = scaled[38:22];   // >> 22, 取 17 bit

    // 级 2 寄存
    reg [16:0] mm_result_r;
    reg        v_r2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mm_result_r <= 17'd0;
            v_r2        <= 1'b0;
        end else begin
            mm_result_r <= mm_result;
            v_r2        <= v_r1;
        end
    end

    //==================================================================
    // 级 3: 输出寄存
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gap_mm_x10 <= 16'd0;
            mm_valid   <= 1'b0;
        end else begin
            mm_valid   <= v_r2;
            if (v_r2)
                gap_mm_x10 <= mm_result_r[15:0];   // 最大 65535 (6553.5mm)
        end
    end

endmodule
