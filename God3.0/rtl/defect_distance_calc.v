// ============================================================================
// Module: defect_distance_calc
// Description: 双模式缺陷距离计算 (冻结版方案 §7)
//
//   模式A (bottom_found=1, 本次找到棒底, 高置信):
//       distance_mm = defect_delta_q2 × l_test_mm / bottom_delta_q2
//       用本次棒底间隔归一化时间, 抵消材料波速/采样率差异
//
//   模式B (bottom_found=0, 棒底缺失, 低置信):
//       distance_mm = defect_delta_q2 × l_ref_mm / (d_ref×4)
//       退化为赛前固化模板比例, low_confidence 置 1
//
// 输入延迟为 Q2 (1/4 采样点)，使用 23 周期移位减法除法器。
// 分母为 0 时输出 dist_invalid=1 (不再静默输出 0)。
// ============================================================================
module defect_distance_calc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [10:0] defect_delta_q2,  // (Tdef - T0)×4
    input  wire        bottom_found,     // 本次是否找到棒底
    input  wire [10:0] bottom_delta_q2,  // (Tend - T0)×4
    input  wire [8:0]  d_ref,            // 赛前固化正常棒底间隔
    input  wire [11:0] l_test_mm,        // 现场棒标称长度 (模式A)
    input  wire [11:0] l_ref_mm,         // 自有棒实测长度 (模式B)
    output reg         busy,
    output reg         done,
    output reg  [11:0] distance_mm,
    output reg         low_confidence,   // 1 = 模式B结果, 不可与模式A等价展示
    output reg         dist_invalid      // 1 = 分母为0, 距离不可用
);

    // ------------------------------------------------------------------------
    // 双模式选择 (仅 start 拍使用, 之后全部用锁存值)
    // ------------------------------------------------------------------------
    wire [10:0] divisor_sel =
        bottom_found ? bottom_delta_q2 : {d_ref, 2'b00};
    wire [11:0] length_sel  = bottom_found ? l_test_mm    : l_ref_mm;

    reg [22:0] dividend;
    reg [22:0] quotient;
    reg [11:0] remainder;
    reg [10:0] divisor;
    reg [5:0]  bit_count;
    reg [11:0] length_latched;

    // 移位加法展开乘积 (Q2 延迟 11 位 × length 12 位 = 23 位):
    // 规避 `*` 推断 lpm_mult (本机 Quartus 无法加载 cbx_lpm_mult.dll),
    // 11 个部分积组合求和, 仅在 start 拍锁存一次, 非关键路径
    wire [22:0] len_x = {11'd0, length_sel};
    wire [22:0] product =
        (defect_delta_q2[0]  ? len_x         : 23'd0) +
        (defect_delta_q2[1]  ? (len_x << 1)  : 23'd0) +
        (defect_delta_q2[2]  ? (len_x << 2)  : 23'd0) +
        (defect_delta_q2[3]  ? (len_x << 3)  : 23'd0) +
        (defect_delta_q2[4]  ? (len_x << 4)  : 23'd0) +
        (defect_delta_q2[5]  ? (len_x << 5)  : 23'd0) +
        (defect_delta_q2[6]  ? (len_x << 6)  : 23'd0) +
        (defect_delta_q2[7]  ? (len_x << 7)  : 23'd0) +
        (defect_delta_q2[8]  ? (len_x << 8)  : 23'd0) +
        (defect_delta_q2[9]  ? (len_x << 9)  : 23'd0) +
        (defect_delta_q2[10] ? (len_x << 10) : 23'd0);
    wire [11:0] rem_shift = {remainder[10:0], dividend[22]};
    wire        subtract_ok = rem_shift >= {1'b0, divisor};
    wire [11:0] rem_next =
        subtract_ok ? (rem_shift - {1'b0, divisor}) : rem_shift;
    wire [22:0] quotient_next =
        {quotient[21:0], subtract_ok};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy           <= 1'b0;
            done           <= 1'b0;
            distance_mm    <= 12'd0;
            low_confidence <= 1'b0;
            dist_invalid   <= 1'b0;
            dividend       <= 23'd0;
            quotient       <= 23'd0;
            remainder      <= 12'd0;
            divisor        <= 11'd0;
            bit_count      <= 6'd0;
            length_latched <= 12'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                low_confidence <= ~bottom_found;
                if (divisor_sel == 0) begin
                    // 分母为 0: 距离不可计算, 显式标无效
                    distance_mm  <= 12'd0;
                    dist_invalid <= 1'b1;
                    done         <= 1'b1;
                end else begin
                    busy           <= 1'b1;
                    dist_invalid   <= 1'b0;
                    dividend       <= product;
                    quotient       <= 23'd0;
                    remainder      <= 12'd0;
                    divisor        <= divisor_sel;
                    bit_count      <= 6'd0;
                    length_latched <= length_sel;
                end
            end else if (busy) begin
                dividend  <= {dividend[21:0], 1'b0};
                quotient  <= quotient_next;
                remainder <= rem_next;

                if (bit_count == 6'd22) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    if (quotient_next > {11'd0, length_latched})
                        distance_mm <= length_latched;
                    else
                        distance_mm <= quotient_next[11:0];
                end else begin
                    bit_count <= bit_count + 1'b1;
                end
            end
        end
    end

endmodule
