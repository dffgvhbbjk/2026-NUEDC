//======================================================================
// Module: pile_length_calc
// Function: 桩长计算 — 低应变反射波法 L = c × ΔT / 2
//
// 采样率: 95.88kHz (PLL: 50×27/220/64)
//
// 算法流程:
//   IDLE → 入射波峰检测 → 平静段 → 反射波峰检测 → 计算桩长
//
// 计算:
//   ΔT = ΔN / 95880 (ADC采样率)
//   L  = c × ΔT / 2 (米)
//   L_cm = wave_speed × ΔN × 5 / 95880 = wave_speed × ΔN / 19176
//   用定点乘法: ×(2^32/19176) ≈ ×224005, 右移32位
//
// Date:    2026-07-16, 更新 2026-07-22 (96kHz)
//======================================================================

module pile_length_calc (
    input  wire         clk,
    input  wire         rst_n,

    // ADC数据 (24-bit补码)
    input  wire [23:0]  adc_data,
    input  wire         adc_data_vld,

    // 波速设置 (×10, 38000 = 3800.0 m/s)
    input  wire [15:0]  wave_speed,

    // 结果
    output wire [15:0]  pile_length,     // 桩长×10 dm
    output wire         length_valid
);

//======================================================================
// 新样本检测 — FIFO会重复输出同一采样, 只在adc_data_vld上升沿计为新样本
//======================================================================
reg  adc_vld_d1;
wire adc_new;

always @(posedge clk) begin
    adc_vld_d1 <= adc_data_vld;
end

// 加20周期消抖: 同一ADC采样可能产生多个vld脉冲, 拉开间距
reg [4:0] debounce;
reg       adc_new_sample;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        debounce       <= 5'd0;
        adc_new_sample <= 1'b0;
    end
    else begin
        adc_new_sample <= 1'b0;
        if (debounce > 0)
            debounce <= debounce - 1'b1;
        else if (adc_data_vld && !adc_vld_d1) begin
            adc_new_sample <= 1'b1;
            debounce       <= 5'd20;   // 20周期×25ns=500ns << 10.4µs采样间隔
        end
    end
end

//======================================================================
// 状态机
//======================================================================
localparam ST_IDLE     = 3'd0;
localparam ST_INC_PEAK = 3'd1;  // 追踪入射波峰
localparam ST_QUIET    = 3'd2;  // 平静段
localparam ST_REF_PEAK = 3'd3;  // 追踪反射波峰
localparam ST_DONE     = 3'd4;  // 计算完成

reg [2:0]  state;

// 入射/反射波检测
reg [23:0] inc_peak_val;     // 入射波峰值幅值(绝对值)
reg [23:0] inc_peak_pos;     // 入射波峰样本序号
reg [23:0] ref_peak_val;     // 反射波峰值幅值
reg [23:0] ref_peak_pos;     // 反射波峰样本序号

// 样本计数器
reg [23:0] sample_cnt;

// 平静段/超时
reg [23:0] quiet_cnt;

// 上一周期幅值 (判断上升/下降)
reg [23:0] prev_abs;

// 幅值 (绝对值)
wire [23:0] adc_abs;
assign adc_abs = adc_data[23] ? (~adc_data + 1'b1) : adc_data;

// 触发阈值
wire above_trig;
assign above_trig = adc_abs > 24'h020000;

// 上升沿
wire rising;
assign rising = adc_abs > prev_abs;

// 峰值下降阈值: 幅值降到峰顶的 1/4 以下视为该波结束
wire peak_ended_inc;
wire peak_ended_ref;
assign peak_ended_inc = (inc_peak_val > 24'd100) && (adc_abs < {2'b0, inc_peak_val[23:2]});
assign peak_ended_ref = (ref_peak_val > 24'd100) && (adc_abs < {2'b0, ref_peak_val[23:2]});

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        inc_peak_val <= 24'd0;
        inc_peak_pos <= 24'd0;
        ref_peak_val <= 24'd0;
        ref_peak_pos <= 24'd0;
        sample_cnt   <= 24'd0;
        quiet_cnt    <= 24'd0;
        prev_abs     <= 24'd0;
    end
    else if (adc_new_sample) begin
        prev_abs   <= adc_abs;
        sample_cnt <= sample_cnt + 1'b1;

        case (state)
            ST_IDLE: begin
                if (above_trig) begin
                    state        <= ST_INC_PEAK;
                    sample_cnt   <= 24'd1;
                    inc_peak_val <= adc_abs;
                    inc_peak_pos <= 24'd1;
                    quiet_cnt    <= 24'd0;
                end
            end

            ST_INC_PEAK: begin
                if (rising) begin
                    // 持续上升 → 更新峰顶
                    inc_peak_val <= adc_abs;
                    inc_peak_pos <= sample_cnt;
                end
                else if (peak_ended_inc) begin
                    // 幅值降到 1/4 以下 → 入射波结束
                    state     <= ST_QUIET;
                    quiet_cnt <= 24'd0;
                end
            end

            ST_QUIET: begin
                quiet_cnt <= quiet_cnt + 1'b1;
                // 再次超过入射峰值的1/4 → 可能是反射波
                if (adc_abs > {2'b0, inc_peak_val[23:2]}) begin
                    state        <= ST_REF_PEAK;
                    ref_peak_val <= adc_abs;
                    ref_peak_pos <= sample_cnt;
                end
                else if (quiet_cnt > 24'd96000) begin
                    // ~1秒无反射 → 超时回IDLE
                    state <= ST_IDLE;
                end
            end

            ST_REF_PEAK: begin
                if (rising) begin
                    ref_peak_val <= adc_abs;
                    ref_peak_pos <= sample_cnt;
                end
                else if (peak_ended_ref) begin
                    state <= ST_DONE;
                end
            end

            ST_DONE: begin
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

//======================================================================
// 桩长计算 (定点乘法, 3级流水线, ST_DONE窗口约20.8µs, 时间充裕)
//
//   L(cm) = wave_speed × ΔN / 9600   (wave_speed = 波速×10, 如38000=3800.0m/s)
//   用定点: ×(2^32 / 9600) ≈ ×447392 → 右移32位
//   例: c=3800m/s, ΔN=252 → L=38000×252/9600=997cm=9.97m
//
//   位宽: wave_speed[15:0] × delta_n[23:0] = 40-bit
//         mult_a[39:0] × FIXED_K[31:0] = 72-bit, 取[47:32]即(>>32)的低16位
//   ⚠️ 必须显式声明中间位宽! 若直接写 (a*b*c)>>32, Verilog按32位上下文
//      计算, 中间结果溢出截断, 桩长恒为0 (已修复, 勿回退)
//======================================================================
reg [15:0] length_reg;    // 桩长 cm (如 1250 = 12.50m)
reg        valid_reg;
reg [23:0] delta_n;
reg [39:0] mult_a;
reg [71:0] mult_b;
reg [2:0]  done_pipe;

// 定点常数: 2^32 / 19176 ≈ 224005  (fs=95.88kHz)
localparam FIXED_K = 32'd224005;

//----------------------------------------------------------------------
// 16-bit × 24-bit 无符号乘法 (shift-add, 避免推断 lpm_mult)
//   遍历 wave_speed 的 16 位, 为 1 则累加 delta_n << bit_idx
//   综合后为 16 个 40-bit 加法器, 资源约 480 LE, 无需 DSP
//----------------------------------------------------------------------
function [39:0] mul_16x24;
    input [15:0] a;      // wave_speed
    input [23:0] b;      // delta_n
    reg   [39:0] acc;
    integer i;
    begin
        acc = 40'd0;
        for (i = 0; i < 16; i = i + 1) begin
            if (a[i])
                acc = acc + ({16'd0, b} << i);
        end
        mul_16x24 = acc;
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        length_reg <= 16'd0;
        valid_reg  <= 1'b0;
        delta_n    <= 24'd0;
        mult_a     <= 40'd0;
        mult_b     <= 72'd0;
        done_pipe  <= 3'b000;
    end
    else begin
        // ST_DONE 持续1个采样周期 (~10.4µs@95.88kHz), 足够3级流水完成
        done_pipe <= {done_pipe[1:0], (state == ST_DONE)};

        if (state == ST_DONE)
            delta_n <= ref_peak_pos - inc_peak_pos;
        if (done_pipe[0])
            // wave_speed × delta_n (16b × 24b = 40b)
            //   用 shift-add 实现, 避免推断 lpm_mult (cbx_lpm_mult.dll 加载失败)
            //   遍历 wave_speed 每一位, 为 1 则累加 delta_n << bit_idx
            mult_a <= mul_16x24(wave_speed, delta_n);
        if (done_pipe[1])
            // mult_a × FIXED_K (224005)
            //   224005 = 2^17 + 2^16 + 2^14 + 2^13 + 2^11 + 2^9 + 2^8 + 2^2 + 2^0
            //   用 shift-add 实现, 避免推断 lpm_mult (cbx_lpm_mult.dll 缺失)
            mult_b <= (mult_a << 17)
                    + (mult_a << 16)
                    + (mult_a << 14)
                    + (mult_a << 13)
                    + (mult_a << 11)
                    + (mult_a <<  9)
                    + (mult_a <<  8)
                    + (mult_a <<  2)
                    + (mult_a <<  0);
        if (done_pipe[2]) begin
            length_reg <= mult_b[47:32];        // (>>32) 后取低16位
            valid_reg  <= 1'b1;
        end

        // 新一轮测量开始 → 清除有效标志
        if (state == ST_IDLE && above_trig && adc_new_sample)
            valid_reg <= 1'b0;
    end
end

assign pile_length  = length_reg;
assign length_valid = valid_reg;

endmodule
