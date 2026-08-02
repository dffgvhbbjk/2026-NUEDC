// ============================================================================
// Module: defect_analyzer
// Description: 96kHz 连续缺陷反射定位器
//
//   1. trigger 固定为 T0，幅值比例只负责 NORMAL/DEFECT 分类；
//   2. 在主冲击保护区之后至棒底保护区之前连续搜索相关局部峰；
//   3. 棒底只在 d_ref±8 点的物理先验窗内独立搜索；
//   4. 保存连续区间内最强的两个候选。仅当二者关于棒底近似互补时，
//      才使用实测幅值证据区分直接反射与对称多径，不再先选 300/700 窗；
//   5. Q2 相关延迟只用于测距；红/紫标记另从原始样点 RAM 搜实际负波谷；
//   6. 500mm 前后由 2×Ddef 与 Dbottom 的比较得到，不使用固定距离类别。
//
// 当前 350 帧只覆盖 305/695/正常/干扰；连续位置仍需增加中间位置实测
// 数据回归，但 RTL 数据流已经不再把 300/700 写成两个固定输出类别。
// ============================================================================
module defect_analyzer #(
    parameter [23:0] MIN_FRAME_PEAK   = 24'd500000,
    parameter [23:0] MIN_BOTTOM_PEAK  = 24'd500000,
    parameter [23:0] MIN_PACKET_ENV   = 24'd30000,
    parameter [23:0] IMPACT_SAT_LIMIT = 24'd8300000
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [23:0] sample_data,
    input  wire               sample_valid,
    input  wire               trigger,
    input  wire               capture_done,
    input  wire               rearm,
    input  wire [23:0]        trigger_threshold,
    input  wire [8:0]         d_ref,

    output reg                result_valid,
    output reg  [1:0]         result_state,
    output reg                measurement_valid,
    output reg  [1:0]         confidence,
    output reg                bottom_found,
    output reg  [8:0]         impact_peak_index,
    output reg  [8:0]         defect_peak_index,
    output reg  [8:0]         bottom_peak_index,
    output reg  [8:0]         defect_delta,
    output reg  [8:0]         bottom_delta,
    output reg  [10:0]        defect_delta_q2,
    output reg  [10:0]        bottom_delta_q2,
    output reg  [23:0]        impact_peak_value,
    output reg  [23:0]        defect_peak_value,
    output wire [23:0]        dbg_envelope,
    output wire [23:0]        dbg_reflect_threshold,
    output reg  [2:0]         dbg_state
);

    localparam [1:0] RES_INVALID = 2'd0;
    localparam [1:0] RES_NORMAL  = 2'd1;
    localparam [1:0] RES_DEFECT  = 2'd2;

    localparam [2:0] ST_IDLE      = 3'd0;
    localparam [2:0] ST_COLLECT   = 3'd1;
    localparam [2:0] ST_CORR_PREP = 3'd2;
    localparam [2:0] ST_CORR_WAIT = 3'd3;
    localparam [2:0] ST_CORR_MUL  = 3'd4;
    localparam [2:0] ST_CLASSIFY  = 3'd5;
    localparam [2:0] ST_MARK      = 3'd6;
    localparam [2:0] ST_DONE      = 3'd7;

    localparam [6:0] CORR_LO = 7'd20;
    localparam [6:0] BOTTOM_HALF_WIDTH = 7'd8;
    localparam [6:0] BOTTOM_GUARD = 7'd8;
    localparam [7:0] SYMMETRY_TOL = 8'd12;

    // 以下三个幅值区只服务于三态分类与“已检测到对称候选”后的消歧，
    // 绝不直接产生缺陷延迟。
    localparam [8:0] NEAR_LO = 9'd20;
    localparam [8:0] NEAR_HI = 9'd40;
    localparam [8:0] MID_LO  = 9'd55;
    localparam [8:0] MID_HI  = 9'd85;
    localparam [8:0] REF_LO  = 9'd95;
    localparam [8:0] REF_HI  = 9'd120;

    // ------------------------------------------------------------------------
    // 4 点因果包络：只用于能量比例分类，不改变 VGA/UART 原始波形。
    // ------------------------------------------------------------------------
    reg [23:0] abs_hist [0:3];
    reg [25:0] env_sum;
    wire [23:0] magnitude =
        sample_data[23] ? (~sample_data[23:0] + 1'b1) : sample_data[23:0];
    wire [25:0] env_sum_next = env_sum
                              - {2'd0, abs_hist[3]}
                              + {2'd0, magnitude};
    wire [23:0] envelope_next = env_sum_next[25:2];

    assign dbg_envelope = env_sum[25:2];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            env_sum     <= 26'd0;
            abs_hist[0] <= 24'd0;
            abs_hist[1] <= 24'd0;
            abs_hist[2] <= 24'd0;
            abs_hist[3] <= 24'd0;
        end else if (sample_valid) begin
            env_sum <= env_sum_next;
            abs_hist[3] <= abs_hist[2];
            abs_hist[2] <= abs_hist[1];
            abs_hist[1] <= abs_hist[0];
            abs_hist[0] <= magnitude;
        end
    end

    reg [8:0] sample_index;
    reg       frame_saturated;
    reg [23:0] frame_peak, near_peak, mid_peak, ref_peak;
    reg [23:0] bottom_region_peak;
    reg [6:0]  bottom_region_index;
    reg [8:0]  near_index, mid_index, ref_index;

    // ------------------------------------------------------------------------
    // 独立反射波包候选提取
    //
    // edge:     包络从阈值下方进入；
    // trend:    包内峰后连续 2 点下降；
    // duration: 阈值以上至少持续 4 点；
    // guard:    触发后 20 点内只视为主冲击尾部，不产生缺陷候选。
    //
    // 候选按到达顺序保存到 8 项小表。后续自相关仍连续计算相关曲线，
    // 但只有表内中心点才允许进入缺陷候选排名。
    // ------------------------------------------------------------------------
    reg [6:0] packet_candidate_0, packet_candidate_1;
    reg [6:0] packet_candidate_2, packet_candidate_3;
    reg [6:0] packet_candidate_4, packet_candidate_5;
    reg [6:0] packet_candidate_6, packet_candidate_7;
    reg [3:0] packet_candidate_count;
    reg         packet_active;
    reg [5:0]   packet_width;
    reg [1:0]   packet_fall_count;
    reg [23:0]  packet_peak;
    reg [6:0]   packet_peak_index;
    reg [23:0]  prev_envelope;

    task store_packet_candidate;
        input [6:0] candidate_index;
        begin
            case (packet_candidate_count)
                4'd0: packet_candidate_0 <= candidate_index;
                4'd1: packet_candidate_1 <= candidate_index;
                4'd2: packet_candidate_2 <= candidate_index;
                4'd3: packet_candidate_3 <= candidate_index;
                4'd4: packet_candidate_4 <= candidate_index;
                4'd5: packet_candidate_5 <= candidate_index;
                4'd6: packet_candidate_6 <= candidate_index;
                4'd7: packet_candidate_7 <= candidate_index;
                default: packet_candidate_7 <= packet_candidate_7;
            endcase
            if (packet_candidate_count < 4'd8)
                packet_candidate_count <= packet_candidate_count + 1'b1;
        end
    endtask

    // 锤击触发门限已经完成“有效敲击”过滤；波包层只把它作为噪声下限，
    // 不能再乘 4，否则中间位置的较弱直接反射会在候选层被提前删除。
    wire [23:0] trigger_packet_floor = trigger_threshold;
    wire [23:0] impact_packet_floor = frame_peak >> 4;
    wire [23:0] packet_threshold_a =
        (trigger_packet_floor > MIN_PACKET_ENV)
            ? trigger_packet_floor : MIN_PACKET_ENV;
    wire [23:0] packet_threshold =
        (impact_packet_floor > packet_threshold_a)
            ? impact_packet_floor : packet_threshold_a;
    assign dbg_reflect_threshold = packet_threshold;

    // 比例比较（移位加法，避免除法/通用乘法）。
    wire [30:0] frame_ext = {7'd0, frame_peak};
    wire [30:0] near_ext  = {7'd0, near_peak};
    wire [30:0] mid_ext   = {7'd0, mid_peak};
    wire [30:0] ref_ext   = {7'd0, ref_peak};
    wire [30:0] near_x100 = (near_ext << 6) + (near_ext << 5)
                          + (near_ext << 2);
    wire [30:0] mid_x100  = (mid_ext << 6) + (mid_ext << 5)
                          + (mid_ext << 2);
    wire [30:0] ref_x100  = (ref_ext << 6) + (ref_ext << 5)
                          + (ref_ext << 2);
    wire [30:0] frame_x66 = (frame_ext << 6) + (frame_ext << 1);
    wire [30:0] frame_x82 = (frame_ext << 6) + (frame_ext << 4)
                          + (frame_ext << 1);
    wire [30:0] frame_x55 = (frame_ext << 5) + (frame_ext << 4)
                          + (frame_ext << 2) + (frame_ext << 1)
                          + frame_ext;

    wire strong_frame = (frame_peak >= MIN_FRAME_PEAK) && !frame_saturated;
    wire normal_class = (mid_x100 > frame_x66);
    // MID 窗不能单独判远端：实测中 305mm 的两次误报都只命中 MID，
    // 而 695mm 没有一帧仅靠 MID 命中。借鉴官方“趋势分段”思想后可见，
    // 这里主要是近端反射后的持续振铃，不是独立的远端反射。
    wire late_direct_evidence = (near_x100 > frame_x82)
                              || (ref_x100 > frame_x55);

    wire [8:0] bottom_lo_9 =
        (d_ref > {2'd0, BOTTOM_HALF_WIDTH})
            ? (d_ref - {2'd0, BOTTOM_HALF_WIDTH}) : 9'd1;
    wire [8:0] bottom_hi_unclamped =
        d_ref + {2'd0, BOTTOM_HALF_WIDTH};
    wire [6:0] bottom_lo = bottom_lo_9[6:0];
    wire [6:0] bottom_hi =
        (bottom_hi_unclamped > 9'd116)
            ? 7'd116 : bottom_hi_unclamped[6:0];
    wire [6:0] candidate_hi =
        (bottom_lo > BOTTOM_GUARD)
            ? (bottom_lo - BOTTOM_GUARD) : CORR_LO;

    // ------------------------------------------------------------------------
    // 自相关数据：保存 16 位一阶差分到同步 RAM；另存原始校正样点供
    // 红/紫标记扫描。d[n] = x[n+1]-x[n]。
    // ------------------------------------------------------------------------
    reg signed [16:0] diff_mem [0:127];
    reg signed [23:0] sample_mem [0:127];
    reg signed [16:0] tpl_diff [0:11];
    reg signed [15:0] prev_sample16;
    wire signed [15:0] sample16 = sample_data[23:8];
    wire signed [16:0] diff_now =
        {sample16[15], sample16} - {prev_sample16[15], prev_sample16};

    reg [6:0] corr_rd_addr;
    reg signed [16:0] corr_diff_q;
    reg [6:0] marker_rd_addr;
    reg signed [23:0] marker_sample_q;

    // RAM 写读块无异步复位，便于 Quartus 推断嵌入式 RAM。
    always @(posedge clk) begin
        if (sample_valid && (dbg_state == ST_COLLECT) &&
            (sample_index < 9'd128)) begin
            diff_mem[sample_index[6:0]] <= diff_now;
            sample_mem[sample_index[6:0]] <= sample_data;
        end
        corr_diff_q <= diff_mem[corr_rd_addr];
        marker_sample_q <= sample_mem[marker_rd_addr];
    end

    reg [6:0] corr_lag;
    reg [3:0] mac_index;
    reg signed [39:0] corr_acc;

    // 本机 Quartus 18.1 缺少 cbx_lpm_mult.dll，不能使用 `*` 推断
    // lpm_mult。每一项改用 17 周期无符号移位累加，最后恢复符号。
    // 一帧最坏约 87×12×17=17748 拍（40MHz 下 0.45ms）。
    reg        corr_wait_phase;
    reg [4:0]  mul_bit;
    reg [16:0] mul_multiplier;
    reg [33:0] mul_multiplicand;
    reg [33:0] mul_acc;
    reg        mul_negative;

    function signed [16:0] template_at;
        input [3:0] index;
        begin
            case (index)
                4'd0:  template_at = tpl_diff[0];
                4'd1:  template_at = tpl_diff[1];
                4'd2:  template_at = tpl_diff[2];
                4'd3:  template_at = tpl_diff[3];
                4'd4:  template_at = tpl_diff[4];
                4'd5:  template_at = tpl_diff[5];
                4'd6:  template_at = tpl_diff[6];
                4'd7:  template_at = tpl_diff[7];
                4'd8:  template_at = tpl_diff[8];
                4'd9:  template_at = tpl_diff[9];
                4'd10: template_at = tpl_diff[10];
                default: template_at = tpl_diff[11];
            endcase
        end
    endfunction

    wire signed [16:0] tpl_now = template_at(mac_index);
    wire [16:0] tpl_abs =
        tpl_now[16] ? (~tpl_now[16:0] + 1'b1) : tpl_now[16:0];
    wire [16:0] corr_abs =
        corr_diff_q[16] ? (~corr_diff_q[16:0] + 1'b1)
                        : corr_diff_q[16:0];
    wire [33:0] mul_acc_next =
        mul_acc + (mul_multiplier[0] ? mul_multiplicand : 34'd0);
    wire signed [33:0] mul_product_signed =
        mul_negative ? -$signed(mul_acc_next) : $signed(mul_acc_next);
    wire signed [39:0] corr_term_sum =
        corr_acc + {{6{mul_product_signed[33]}}, mul_product_signed};
    wire [39:0] corr_score_now =
        corr_term_sum[39] ? (~corr_term_sum + 1'b1) : corr_term_sum;

    reg [1:0]  score_history_count;
    reg [6:0]  prev_lag;
    reg [39:0] prev2_score, prev_score;

    reg [6:0]  best_candidate_lag, second_candidate_lag, best_bottom_lag;
    reg [39:0] best_candidate_score, second_candidate_score;
    reg [39:0] best_bottom_score;

    wire prev_is_local_peak = (score_history_count >= 2'd2)
                            && (prev_score > prev2_score)
                            && (prev_score >= corr_score_now);
    wire prev_matches_packet =
           ((packet_candidate_count > 4'd0) &&
            (prev_lag == packet_candidate_0))
        || ((packet_candidate_count > 4'd1) &&
            (prev_lag == packet_candidate_1))
        || ((packet_candidate_count > 4'd2) &&
            (prev_lag == packet_candidate_2))
        || ((packet_candidate_count > 4'd3) &&
            (prev_lag == packet_candidate_3))
        || ((packet_candidate_count > 4'd4) &&
            (prev_lag == packet_candidate_4))
        || ((packet_candidate_count > 4'd5) &&
            (prev_lag == packet_candidate_5))
        || ((packet_candidate_count > 4'd6) &&
            (prev_lag == packet_candidate_6))
        || ((packet_candidate_count > 4'd7) &&
            (prev_lag == packet_candidate_7));
    wire prev_in_candidate = (prev_lag >= CORR_LO)
                           && (prev_lag <= candidate_hi)
                           && prev_matches_packet;
    wire prev_in_bot = (prev_lag >= bottom_lo) && (prev_lag <= bottom_hi);

    // EP4CE6 资源不足以同时保留三套 40 位三点 Q2 细化网络。相关仍负责
    // 候选评分，测距延迟降为整数采样点×4；外部距离公式保持不变。
    wire [10:0] best_candidate_q2 =
        {2'd0, best_candidate_lag, 2'b00};
    wire [10:0] second_candidate_q2 =
        {2'd0, second_candidate_lag, 2'b00};
    wire [10:0] bottom_corr_q2 =
        {2'd0, best_bottom_lag, 2'b00};
    wire [6:0] bottom_measure_lag =
        (best_bottom_score != 40'd0)
            ? best_bottom_lag : bottom_region_index;
    wire [10:0] bottom_measure_q2 =
        (best_bottom_score != 40'd0)
            ? bottom_corr_q2
            : {2'd0, bottom_region_index, 2'b00};

    wire candidate_valid = (best_candidate_score != 40'd0);
    wire second_candidate_valid = (second_candidate_score != 40'd0);
    wire bottom_valid = (bottom_region_peak >= MIN_BOTTOM_PEAK);

    wire best_is_early =
        best_candidate_lag <= second_candidate_lag;
    wire [7:0] candidate_sum =
        {1'b0, best_candidate_lag} + {1'b0, second_candidate_lag};
    wire [7:0] bottom_lag_ext = {1'b0, bottom_measure_lag};
    wire [7:0] symmetry_error =
        (candidate_sum >= bottom_lag_ext)
            ? (candidate_sum - bottom_lag_ext)
            : (bottom_lag_ext - candidate_sum);
    wire symmetric_pair = second_candidate_valid
                        && bottom_valid
                        && (symmetry_error <= SYMMETRY_TOL);
    wire choose_second =
        symmetric_pair
            ? ((late_direct_evidence && best_is_early)
                || (!late_direct_evidence && !best_is_early))
            : 1'b0;
    wire [6:0] chosen_candidate_lag =
        choose_second ? second_candidate_lag : best_candidate_lag;
    wire [10:0] chosen_candidate_q2 =
        choose_second ? second_candidate_q2 : best_candidate_q2;
    // 保留为 SignalTap 可观测的 500mm 前后半段判据：
    // 0: 2×Ddef <= Dbottom，1: 2×Ddef > Dbottom。
    (* keep = "true" *) wire defect_after_half =
        ({1'b0, chosen_candidate_q2} << 1)
            > {1'b0, bottom_measure_q2};

    // 负波谷标记扫描控制。算法延迟与显示索引分开保存。
    reg        marker_is_bottom;
    reg        marker_wait;
    reg [3:0]  marker_count;
    reg signed [23:0] marker_min_sample;
    reg [6:0]  marker_min_index;
    wire signed [23:0] marker_min_next =
        (marker_sample_q < marker_min_sample)
            ? marker_sample_q : marker_min_sample;
    wire [6:0] marker_min_index_next =
        (marker_sample_q < marker_min_sample)
            ? marker_rd_addr : marker_min_index;

    wire [23:0] marker_min_magnitude =
        marker_min_next[23]
            ? (~marker_min_next[23:0] + 1'b1)
            : marker_min_next[23:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result_valid       <= 1'b0;
            result_state       <= RES_INVALID;
            measurement_valid  <= 1'b0;
            confidence         <= 2'd0;
            bottom_found       <= 1'b0;
            impact_peak_index  <= 9'd0;
            defect_peak_index  <= 9'd0;
            bottom_peak_index  <= 9'd0;
            defect_delta       <= 9'd0;
            bottom_delta       <= 9'd0;
            defect_delta_q2    <= 11'd0;
            bottom_delta_q2    <= 11'd0;
            impact_peak_value  <= 24'd0;
            defect_peak_value  <= 24'd0;
            sample_index       <= 9'd0;
            frame_saturated    <= 1'b0;
            frame_peak         <= 24'd0;
            near_peak          <= 24'd0;
            mid_peak           <= 24'd0;
            ref_peak           <= 24'd0;
            bottom_region_peak <= 24'd0;
            bottom_region_index <= 7'd0;
            near_index         <= 9'd0;
            mid_index          <= 9'd0;
            ref_index          <= 9'd0;
            packet_candidate_0 <= 7'd0;
            packet_candidate_1 <= 7'd0;
            packet_candidate_2 <= 7'd0;
            packet_candidate_3 <= 7'd0;
            packet_candidate_4 <= 7'd0;
            packet_candidate_5 <= 7'd0;
            packet_candidate_6 <= 7'd0;
            packet_candidate_7 <= 7'd0;
            packet_candidate_count <= 4'd0;
            packet_active      <= 1'b0;
            packet_width       <= 6'd0;
            packet_fall_count  <= 2'd0;
            packet_peak        <= 24'd0;
            packet_peak_index  <= 7'd0;
            prev_envelope      <= 24'd0;
            prev_sample16      <= 16'sd0;
            corr_rd_addr       <= 7'd0;
            marker_rd_addr     <= 7'd0;
            corr_lag           <= CORR_LO;
            mac_index          <= 4'd0;
            corr_acc           <= 40'sd0;
            corr_wait_phase    <= 1'b0;
            mul_bit            <= 5'd0;
            mul_multiplier     <= 17'd0;
            mul_multiplicand   <= 34'd0;
            mul_acc            <= 34'd0;
            mul_negative       <= 1'b0;
            score_history_count <= 2'd0;
            prev_lag           <= 7'd0;
            prev2_score        <= 40'd0;
            prev_score         <= 40'd0;
            best_candidate_lag <= CORR_LO;
            second_candidate_lag <= CORR_LO;
            best_bottom_lag    <= 7'd105;
            best_candidate_score <= 40'd0;
            second_candidate_score <= 40'd0;
            best_bottom_score  <= 40'd0;
            marker_is_bottom   <= 1'b0;
            marker_wait        <= 1'b0;
            marker_count       <= 4'd0;
            marker_min_sample  <= 24'sh7FFFFF;
            marker_min_index   <= 7'd0;
            dbg_state          <= ST_IDLE;
        end else begin
            result_valid <= 1'b0;

            if (rearm && (dbg_state != ST_IDLE) && (dbg_state != ST_DONE)) begin
                dbg_state <= ST_IDLE;
            end else begin
                case (dbg_state)
                    ST_IDLE, ST_DONE: begin
                        if (sample_valid && trigger) begin
                            sample_index       <= 9'd0;
                            prev_sample16      <= sample16;
                            frame_saturated    <= (magnitude >= IMPACT_SAT_LIMIT);
                            frame_peak         <= envelope_next;
                            near_peak          <= 24'd0;
                            mid_peak           <= 24'd0;
                            ref_peak           <= 24'd0;
                            bottom_region_peak <= 24'd0;
                            bottom_region_index <= d_ref[6:0];
                            near_index         <= 9'd0;
                            mid_index          <= 9'd0;
                            ref_index          <= 9'd0;
                            packet_candidate_0 <= 7'd0;
                            packet_candidate_1 <= 7'd0;
                            packet_candidate_2 <= 7'd0;
                            packet_candidate_3 <= 7'd0;
                            packet_candidate_4 <= 7'd0;
                            packet_candidate_5 <= 7'd0;
                            packet_candidate_6 <= 7'd0;
                            packet_candidate_7 <= 7'd0;
                            packet_candidate_count <= 4'd0;
                            packet_active      <= 1'b0;
                            packet_width       <= 6'd0;
                            packet_fall_count  <= 2'd0;
                            packet_peak        <= 24'd0;
                            packet_peak_index  <= 7'd0;
                            prev_envelope      <= envelope_next;
                            dbg_state          <= ST_COLLECT;
                        end
                    end

                    ST_COLLECT: begin
                        if (capture_done) begin
                            if (packet_active &&
                                (packet_width >= 6'd4))
                                store_packet_candidate(packet_peak_index);
                            corr_lag            <= CORR_LO;
                            score_history_count <= 2'd0;
                            prev2_score         <= 40'd0;
                            prev_score          <= 40'd0;
                            best_candidate_lag  <= CORR_LO;
                            second_candidate_lag <= CORR_LO;
                            best_bottom_lag     <= d_ref[6:0];
                            best_candidate_score <= 40'd0;
                            second_candidate_score <= 40'd0;
                            best_bottom_score   <= 40'd0;
                            dbg_state           <= ST_CORR_PREP;
                        end else if (sample_valid) begin
                            if (sample_index < 9'd12)
                                tpl_diff[sample_index[3:0]] <= diff_now;
                            prev_sample16 <= sample16;
                            sample_index  <= sample_index + 1'b1;

                            if (magnitude >= IMPACT_SAT_LIMIT)
                                frame_saturated <= 1'b1;
                            if (envelope_next > frame_peak)
                                frame_peak <= envelope_next;
                            prev_envelope <= envelope_next;

                            // 官方代码式“边沿 + 趋势 + 持续宽度”候选层。
                            if ((sample_index + 1'b1) < 9'd20) begin
                                packet_active     <= 1'b0;
                                packet_width      <= 6'd0;
                                packet_fall_count <= 2'd0;
                            end else if (!packet_active) begin
                                if ((envelope_next > packet_threshold) &&
                                    (envelope_next > prev_envelope)) begin
                                    packet_active     <= 1'b1;
                                    packet_width      <= 6'd1;
                                    packet_fall_count <= 2'd0;
                                    packet_peak       <= envelope_next;
                                    packet_peak_index <=
                                        (sample_index + 1'b1);
                                end
                            end else begin
                                if (packet_width != 6'h3F)
                                    packet_width <= packet_width + 1'b1;

                                if (envelope_next > packet_peak) begin
                                    packet_peak       <= envelope_next;
                                    packet_peak_index <=
                                        (sample_index + 1'b1);
                                    packet_fall_count <= 2'd0;
                                end else if (envelope_next <
                                             prev_envelope) begin
                                    if (((packet_fall_count >= 2'd1) ||
                                         (envelope_next <=
                                            packet_threshold)) &&
                                        (packet_width >= 6'd3)) begin
                                        store_packet_candidate(
                                            packet_peak_index);
                                        packet_active     <= 1'b0;
                                        packet_width      <= 6'd0;
                                        packet_fall_count <= 2'd0;
                                    end else begin
                                        packet_fall_count <=
                                            packet_fall_count + 1'b1;
                                    end
                                end else begin
                                    packet_fall_count <= 2'd0;
                                end
                            end

                            if ((sample_index + 1'b1) >= bottom_lo_9 &&
                                (sample_index + 1'b1) <=
                                    {2'd0, bottom_hi} &&
                                envelope_next > bottom_region_peak) begin
                                bottom_region_peak <= envelope_next;
                                bottom_region_index <=
                                    (sample_index + 1'b1);
                            end

                            if ((sample_index + 1'b1) >= NEAR_LO &&
                                (sample_index + 1'b1) <= NEAR_HI &&
                                envelope_next > near_peak) begin
                                near_peak  <= envelope_next;
                                near_index <= sample_index + 1'b1;
                            end
                            if ((sample_index + 1'b1) >= MID_LO &&
                                (sample_index + 1'b1) <= MID_HI &&
                                envelope_next > mid_peak) begin
                                mid_peak  <= envelope_next;
                                mid_index <= sample_index + 1'b1;
                            end
                            if ((sample_index + 1'b1) >= REF_LO &&
                                (sample_index + 1'b1) <= REF_HI &&
                                envelope_next > ref_peak) begin
                                ref_peak  <= envelope_next;
                                ref_index <= sample_index + 1'b1;
                            end
                        end
                    end

                    ST_CORR_PREP: begin
                        mac_index    <= 4'd0;
                        corr_acc     <= 40'sd0;
                        corr_rd_addr <= corr_lag;
                        corr_wait_phase <= 1'b0;
                        dbg_state    <= ST_CORR_WAIT;
                    end

                    ST_CORR_WAIT: begin
                        if (!corr_wait_phase) begin
                            // 给同步 diff RAM 一拍建立读数据。
                            corr_wait_phase <= 1'b1;
                        end else begin
                            mul_bit          <= 5'd0;
                            mul_multiplier   <= tpl_abs;
                            mul_multiplicand <= {17'd0, corr_abs};
                            mul_acc          <= 34'd0;
                            mul_negative     <= tpl_now[16] ^ corr_diff_q[16];
                            dbg_state        <= ST_CORR_MUL;
                        end
                    end

                    ST_CORR_MUL: begin
                        mul_acc          <= mul_acc_next;
                        mul_multiplier   <= {1'b0, mul_multiplier[16:1]};
                        mul_multiplicand <= mul_multiplicand << 1;

                        if (mul_bit == 5'd16) begin
                            if (mac_index == 4'd11) begin
                                if ((score_history_count >= 2'd2) &&
                                    prev_in_candidate) begin
                                    if (prev_score >
                                        best_candidate_score) begin
                                        second_candidate_score <=
                                            best_candidate_score;
                                        second_candidate_lag <=
                                            best_candidate_lag;
                                        best_candidate_score <= prev_score;
                                        best_candidate_lag   <= prev_lag;
                                    end else if (prev_score >
                                        second_candidate_score) begin
                                        second_candidate_score <= prev_score;
                                        second_candidate_lag   <= prev_lag;
                                    end
                                end
                                if (prev_is_local_peak && prev_in_bot &&
                                    (prev_score > best_bottom_score)) begin
                                        best_bottom_score <= prev_score;
                                        best_bottom_lag   <= prev_lag;
                                end

                                prev2_score <= prev_score;
                                prev_score  <= corr_score_now;
                                prev_lag    <= corr_lag;
                                if (score_history_count != 2'd3)
                                    score_history_count <= score_history_count + 1'b1;

                                if (corr_lag == bottom_hi)
                                    dbg_state <= ST_CLASSIFY;
                                else begin
                                    corr_lag <= corr_lag + 1'b1;
                                    dbg_state <= ST_CORR_PREP;
                                end
                            end else begin
                                corr_acc       <= corr_term_sum;
                                mac_index      <= mac_index + 1'b1;
                                corr_rd_addr   <= corr_lag + mac_index + 1'b1;
                                corr_wait_phase <= 1'b0;
                                dbg_state      <= ST_CORR_WAIT;
                            end
                        end else begin
                            mul_bit <= mul_bit + 1'b1;
                        end
                    end

                    ST_CLASSIFY: begin
                        impact_peak_index <= 9'd0;
                        impact_peak_value <= frame_peak;

                        if (!strong_frame) begin
                            result_state       <= RES_INVALID;
                            measurement_valid  <= 1'b0;
                            confidence         <= 2'd0;
                            bottom_found       <= 1'b0;
                            defect_peak_index  <= 9'd0;
                            bottom_peak_index  <= 9'd0;
                            defect_delta       <= 9'd0;
                            bottom_delta       <= 9'd0;
                            defect_delta_q2    <= 11'd0;
                            bottom_delta_q2    <= 11'd0;
                            defect_peak_value  <= 24'd0;
                            result_valid       <= 1'b1;
                            dbg_state          <= ST_DONE;
                        end else if (normal_class) begin
                            result_state       <= RES_NORMAL;
                            measurement_valid  <= 1'b1;
                            confidence         <= bottom_valid ? 2'd2 : 2'd1;
                            bottom_found       <= bottom_valid;
                            defect_peak_index  <= 9'd0;
                            bottom_peak_index  <= 9'd0;
                            defect_delta       <= 9'd0;
                            bottom_delta       <= 9'd0;
                            defect_delta_q2    <= 11'd0;
                            bottom_delta_q2    <=
                                bottom_valid ? bottom_measure_q2 : 11'd0;
                            defect_peak_value  <= 24'd0;
                            if (bottom_valid) begin
                                marker_is_bottom  <= 1'b1;
                                marker_rd_addr    <= bottom_measure_lag;
                                marker_count      <= 4'd0;
                                marker_wait       <= 1'b0;
                                marker_min_sample <= 24'sh7FFFFF;
                                marker_min_index  <= bottom_measure_lag;
                                dbg_state         <= ST_MARK;
                            end else begin
                                result_valid <= 1'b1;
                                dbg_state    <= ST_DONE;
                            end
                        end else if (candidate_valid) begin
                            result_state       <= RES_DEFECT;
                            measurement_valid  <= 1'b1;
                            confidence         <= bottom_valid ? 2'd2 : 2'd1;
                            bottom_found       <= bottom_valid;
                            defect_peak_index  <= 9'd0;
                            bottom_peak_index  <= 9'd0;
                            defect_delta       <=
                                (chosen_candidate_q2 + 11'd2) >> 2;
                            bottom_delta       <= 9'd0;
                            defect_delta_q2    <= chosen_candidate_q2;
                            bottom_delta_q2    <=
                                bottom_valid ? bottom_measure_q2 : 11'd0;
                            defect_peak_value  <= 24'd0;
                            marker_is_bottom  <= 1'b0;
                            marker_rd_addr    <= chosen_candidate_lag;
                            marker_count      <= 4'd0;
                            marker_wait       <= 1'b0;
                            marker_min_sample <= 24'sh7FFFFF;
                            marker_min_index  <= chosen_candidate_lag;
                            dbg_state         <= ST_MARK;
                        end else begin
                            result_state       <= RES_INVALID;
                            measurement_valid  <= 1'b0;
                            confidence         <= 2'd0;
                            bottom_found       <= bottom_valid;
                            defect_peak_index  <= 9'd0;
                            bottom_peak_index  <= 9'd0;
                            defect_delta       <= 9'd0;
                            bottom_delta       <= 9'd0;
                            defect_delta_q2    <= 11'd0;
                            bottom_delta_q2    <=
                                bottom_valid ? bottom_measure_q2 : 11'd0;
                            defect_peak_value  <= 24'd0;
                            result_valid       <= 1'b1;
                            dbg_state          <= ST_DONE;
                        end
                    end

                    ST_MARK: begin
                        if (!marker_wait) begin
                            // 同步 sample RAM 建立读数据。
                            marker_wait <= 1'b1;
                        end else begin
                            marker_min_sample <= marker_min_next;
                            marker_min_index  <= marker_min_index_next;
                            if (marker_count == 4'd11) begin
                                if (!marker_is_bottom) begin
                                    defect_peak_index <=
                                        {2'd0, marker_min_index_next};
                                    defect_peak_value <= marker_min_magnitude;
                                    if (bottom_valid) begin
                                        marker_is_bottom  <= 1'b1;
                                        marker_rd_addr    <= bottom_measure_lag;
                                        marker_count      <= 4'd0;
                                        marker_wait       <= 1'b0;
                                        marker_min_sample <= 24'sh7FFFFF;
                                        marker_min_index  <= bottom_measure_lag;
                                    end else begin
                                        result_valid <= 1'b1;
                                        dbg_state    <= ST_DONE;
                                    end
                                end else begin
                                    bottom_peak_index <=
                                        {2'd0, marker_min_index_next};
                                    bottom_delta <=
                                        {2'd0, marker_min_index_next};
                                    result_valid <= 1'b1;
                                    dbg_state    <= ST_DONE;
                                end
                            end else begin
                                marker_count   <= marker_count + 1'b1;
                                marker_rd_addr <= marker_rd_addr + 1'b1;
                            end
                        end
                    end

                    default: dbg_state <= ST_IDLE;
                endcase
            end
        end
    end

endmodule
