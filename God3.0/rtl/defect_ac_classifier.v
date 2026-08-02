// ============================================================================
// Module: defect_ac_classifier
// Description: 冲击回波自相关 + LDA 线性判别缺陷分类器 (新算法, 替代 defect_analyzer)
//
// 背景 (数据驱动, God3.0/data 1.6万次敲击离线分析):
//   好棒基波周期 ~96-98 采样点(983Hz), 坏棒 100-107点(890-936Hz);
//   裂缝改变刚度→固有频率偏移, 且不同缺陷位置在自相关曲线上留下独特"指纹"
//   (d800: T/2 深谷; d695: T/3 宽负区; d307: 曲线平坦基波峰低; good: 平缓正值)。
//   新算法: 冲击后 384 点自相关向量(lag 8..135) 经岭回归 LDA 线性判别 5 类。
//   FPGA 只需乘累加 + argmax, 无除法。离线定点验证:
//     单击 acc5=0.919 好判好0.993 坏判坏0.979 定位0.906;
//     5击投票 acc5=0.940 好判好1.000 坏判坏0.991。
//   (旧包络波包法检出率仅 30-50%, 误差 ±50-100mm)
//
// 与 pc_tools/core/fpga_model.py bit-accurate 一一对应:
//   1) 512点采集帧内找 |x| 峰值位置 imp (主冲击)
//   2) 有效性: peak<MIN_PEAK 或 imp>=IMP_WIN(二次弹跳/双击) 或 imp+400>512 -> INVALID
//   3) 自动缩放 sh=max(0,bits(peak)-15); s[i]=(x[imp+16+i])>>>sh  i=0..383 (16位)
//   4) 整数自相关 r[k]=sum s[i]*s[i+k], k=0 及 k=8..135
//   5) 二次缩放 rsh=max(0,bits(r0)-30); rs[k]=r[k]>>>rsh (31位内)
//   6) 打分 score_c = sum_{k=8}^{135} w_c[k]*rs[k] + w_c[128]*rs0  (Q12权重)
//   7) argmax -> pred; margin=top1-top2 < rs0*MARGIN_Q -> 低置信
//   类别 0=good(NORMAL) 1=d200 2=d307 3=d695 4=d800 (DEFECT + 距离查表)
//
// 800mm 二次判别 (方案A, 2026-07-30新增):
//   800mm 缺陷波形与好棒高度相似(缺陷反射lag77 vs 基波lag96), LDA 易误判 good。
//   实测 T0 统计 (30样本/类): 好棒 T0 max=102, 800mm缺陷 T0 min=103。
//   所有好棒判定都扫描 r[90..110] 找基波周期峰值位置 T0;
//   T0>=103 则改判 d800。新增状态 S_D800/S_D800J, 扫描42拍@40MHz≈1us。
//
// 资源 (EP4CE6, 30×M9K/15×mult): 独立影子RAM 1024×24 + seg 512×16 + r 256×48,
//   自相关串行单乘法器, 打分单乘法器串行, argmax串行扫描。
//   周期 ~46k @40MHz ≈ 1.15ms, 远小于重武装静默期 ~85ms。
//
// 时钟域: vga_clk 40MHz (与 wave_capture 同域, 无 CDC)
// ============================================================================
module defect_ac_classifier (
    input  wire               clk,          // vga_clk 40MHz
    input  wire               rst_n,        // 低有效

    // 镜像 wave_capture 写口 (与 wave_uart_export 同址同拍, 原始24位样点)
    input  wire               wr_en,
    input  wire [9:0]         wr_addr,      // {bank, addr}
    input  wire signed [23:0] wr_raw,       // smooth_raw 原始样点

    // 采集完成通知 (与 disp_origin/disp_len_sel 同拍)
    input  wire               cap_done,
    input  wire [9:0]         disp_origin,  // {bank, origin}
    input  wire [1:0]         disp_len_sel, // 11=512点 (本算法只处理512点帧)

    input  wire               rearm,        // 重武装脉冲: 中止当前分析

    // 结果输出 (编码与 defect_analyzer/overlay/UART 兼容)
    output reg                result_valid, // 1拍脉冲: 本次分析出结果
    output reg  [1:0]         result_state, // 0=INVALID 1=NORMAL 2=DEFECT
    output reg  [1:0]         confidence,   // 0=无 1=低置信 2=高置信
    output reg  [2:0]         defect_class, // 0=good 1..4=d200/d307/d695/d800
    output reg  [11:0]        distance_mm,  // 缺陷距离 mm (NORMAL/INVALID=0)
    output reg  [8:0]         impact_index, // 主冲击峰索引 (VGA黄色标记线)
    output reg                busy,         // 分析进行中 (SignalTap 可观测)
    output wire [3:0]         dbg_state     // SignalTap 观测
);

    // ---- 算法常量 (与 fpga_model.py 一致) ----
    localparam integer SKIP    = 16;
    localparam integer SEGLEN  = 384;
    localparam integer NLAG    = 136;      // r[0..135]
    localparam integer LAG0    = 8;
    localparam [9:0]   IMP_WIN = 10'd112;
    localparam [23:0]  MIN_PEAK = 24'd300000;
    localparam integer MARGIN_Q = 82;      // 0.02*4096
    localparam [7:0] D800_LAG_LO = 8'd90;  // 基波周期扫描下界
    localparam [7:0] D800_LAG_HI = 8'd110; // 基波周期扫描上界
    // 实测 T0 统计 (30样本/类):
    //   好棒: avg=95-99, max=102
    //   800mm缺陷: avg=102-107, min=103 (重敲铁头有异常值min=90)
    // 阈值 103: 好棒不误判(max=102), 800mm缺陷大部分能检测(min=103)
    localparam [7:0] D800_T_TH   = 8'd103; // 800mm 判别: 基波周期阈值

    localparam [1:0] RES_INVALID = 2'd0;
    localparam [1:0] RES_NORMAL  = 2'd1;
    localparam [1:0] RES_DEFECT  = 2'd2;

    // 距离查表 (mm)
    function [11:0] dist_lut;
        input [2:0] c;
        begin
            case (c)
                3'd1: dist_lut = 12'd200;
                3'd2: dist_lut = 12'd307;
                3'd3: dist_lut = 12'd695;
                3'd4: dist_lut = 12'd800;
                default: dist_lut = 12'd0;
            endcase
        end
    endfunction

    // ---- LDA 权重 ROM: 由 pc_tools/core/fpga_model.py 生成 defect_lda_weights.mem ----
    //   线性地址 addr = cls*256 + idx = {cls[2:0], idx[7:0]} (1280×16, M9K ROM)
    //   旧版用 case 函数直驱 w_q 未能推断为 ROM, 综合为 ~3700 LE 巨型 LUT 多路器,
    //   导致 EP4CE6 (6272 LE) 装不下; 改用 $readmemh 初始化数组可靠推断 M9K。

    // ========================================================================
    // 影子 RAM: 1024×24 (乒乓双 bank, 与 wave_uart_export 同址同拍写入)
    //   读口专用于分析扫描; 分析期间下一帧写入另一 bank, 当前 bank 保持完整
    // ========================================================================
    (* ramstyle = "M9K" *) reg signed [23:0] shadow_ram [0:1023];
    reg  [9:0]         sh_rd_addr;
    reg  signed [23:0] sh_rd_q;

    always @(posedge clk) begin
        if (wr_en) shadow_ram[wr_addr] <= wr_raw;
    end
    always @(posedge clk) begin
        sh_rd_q <= shadow_ram[sh_rd_addr];
    end

    // ========================================================================
    // seg RAM: 512×16 缩放后段 (true dual-port: 载入写A, 自相关读A/B)
    // ========================================================================
    (* ramstyle = "M9K" *) reg signed [15:0] seg_ram [0:511];
    reg  [8:0]         segA_addr, segB_addr;
    reg                seg_we;
    reg  signed [15:0] seg_wd;
    reg  signed [15:0] segA_q, segB_q;

    always @(posedge clk) begin
        if (seg_we) seg_ram[segA_addr] <= seg_wd;   // 载入阶段写口 (端口A)
        segA_q <= seg_ram[segA_addr];
    end
    always @(posedge clk) begin
        segB_q <= seg_ram[segB_addr];
    end

    // ========================================================================
    // r RAM: 256×48 自相关结果 (地址=lag k)
    // ========================================================================
    (* ramstyle = "M9K" *) reg signed [47:0] r_ram [0:255];
    reg  [7:0]         r_ra, r_wa;
    reg                r_we;
    reg  signed [47:0] r_wd;
    reg  signed [47:0] r_q;

    always @(posedge clk) begin
        if (r_we) r_ram[r_wa] <= r_wd;
        r_q <= r_ram[r_ra];
    end

    // ========================================================================
    // 状态机
    // ========================================================================
    localparam [3:0]
        S_IDLE  = 4'd0,
        S_PSCAN = 4'd1,   // 峰值扫描 512点
        S_VCHK  = 4'd2,   // 有效性判定 + sh
        S_LOAD  = 4'd3,   // 载入缩放段 384点 -> seg_ram
        S_ACSET = 4'd4,   // 设置下一个 lag k
        S_ACRUN = 4'd5,   // 自相关 MAC (当前 k)
        S_ACSTO = 4'd6,   // 存 r[k]
        S_SCLR0 = 4'd7,   // 读 r0
        S_SCAL  = 4'd8,   // 求 rsh/rs0/rs0_82
        S_SCRW  = 4'd13,  // 打分: 等 r_ram 读延迟 (RAM 输出2拍才稳定)
        S_SCRD  = 4'd9,   // 打分: 锁存 rsk
        S_SCMAC = 4'd10,  // 打分: 5类 MAC
        S_ARG   = 4'd11,  // argmax 扫描
        S_DONE  = 4'd12,
        S_D800  = 4'd14,  // 800mm 二次判别: 扫描基波周期
        S_D800J = 4'd15;  // 800mm 判决

    reg [3:0] state;
    assign dbg_state = state;

    // 锁存的帧参数
    reg        f_bank;
    reg [8:0]  f_org;

    // 峰值扫描
    reg [9:0]  ps_cnt;
    reg [23:0] peak_mag;
    reg [9:0]  imp;

    // 缩放
    reg [3:0]  sh;         // 0..9
    reg [4:0]  rsh;        // 0..18

    // 载入
    reg [9:0]  ld_cnt;     // 0..384

    // 自相关
    reg [7:0]  ac_k;       // 当前 lag: 0, 8..135
    reg [9:0]  ac_i;       // 0..(SEGLEN-k)+1
    reg [9:0]  ac_nk;      // SEGLEN-1-k (最后有效 i)
    reg signed [47:0] ac_acc;

    // 缩放中间
    reg signed [47:0] r0_val;
    reg [30:0] rs0;
    reg [63:0] rs0_82;

    // 打分
    reg [7:0]  sc_idx;     // 0..128
    reg [2:0]  sc_c;       // 0..4
    reg signed [32:0] rsk; // r[k]>>>rsh (含符号)
    reg signed [63:0] score0, score1, score2, score3, score4;

    // ---- LDA 权重 M9K ROM ($readmemh 初始化数组 = 可靠 M9K 推断) ----
    //   同步 ROM: 组合地址输入 + 寄存输出, 读延迟 1 拍。打分流水:
    //   S_SCRD 预取 class0, S_SCMAC 当拍预取 (sc_c+1) -> w_q 与 sc_c 累加一一对齐。
    (* ramstyle = "M9K" *) reg signed [15:0] w_rom [0:1279];
    initial $readmemh("defect_lda_weights.mem", w_rom);

    reg [2:0]  w_c_addr;
    reg [7:0]  w_idx_addr;
    reg signed [15:0] w_q;
    always @(*) begin
        w_idx_addr = sc_idx;
        w_c_addr   = (state == S_SCRD) ? 3'd0 : (sc_c + 3'd1);
    end
    always @(posedge clk) begin
        w_q <= w_rom[{w_c_addr, w_idx_addr}];   // addr = cls*256 + idx
    end

    // argmax
    reg [2:0]  am_c;
    reg signed [63:0] best_val, sec_val;
    reg [2:0]  best_i;

    // 800mm 二次判别寄存器
    reg [7:0]  d800_lag;        // 当前扫描 lag
    reg [7:0]  d800_peak_lag;   // 自相关峰位置 (基波周期 T0)
    reg signed [47:0] d800_peak_val;  // 自相关峰值
    reg        d800_phase;      // 0=等r_q稳定 1=读取比较

    // ---- 组合: 由 peak_mag 求 sh = max(0, bits(peak)-15) ----
    wire [3:0] sh_comb =
        peak_mag[23] ? 4'd9 : peak_mag[22] ? 4'd8 : peak_mag[21] ? 4'd7 :
        peak_mag[20] ? 4'd6 : peak_mag[19] ? 4'd5 : peak_mag[18] ? 4'd4 :
        peak_mag[17] ? 4'd3 : peak_mag[16] ? 4'd2 : peak_mag[15] ? 4'd1 : 4'd0;

    // ---- 组合: 当前样点绝对值 (峰值扫描用) ----
    wire signed [23:0] ps_sample = sh_rd_q;
    wire [23:0] ps_abs = ps_sample[23] ? (~ps_sample + 1'b1) : ps_sample;

    // ---- 组合: 打分乘积 (w_q 为 M9K ROM 读出的权重, rsk 已含 r>>>rsh) ----
    wire signed [63:0] mac_prod = $signed(w_q) * $signed(rsk);

    // ---- 组合: 自相关乘积 ----
    wire signed [31:0] ac_prod = segA_q * segB_q;

    // ---- argmax 组合选值 (在 S_ARG 顺序比较, 需在主 FSM 前声明) ----
    reg signed [63:0] am_val;
    always @(*) begin
        case (am_c)
            3'd0: am_val = score0;
            3'd1: am_val = score1;
            3'd2: am_val = score2;
            3'd3: am_val = score3;
            default: am_val = score4;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            busy         <= 1'b0;
            result_valid <= 1'b0;
            result_state <= RES_INVALID;
            confidence   <= 2'd0;
            defect_class <= 3'd0;
            distance_mm  <= 12'd0;
            impact_index <= 9'd0;
            seg_we       <= 1'b0;
            r_we         <= 1'b0;
        end else begin
            result_valid <= 1'b0;
            seg_we       <= 1'b0;
            r_we         <= 1'b0;

            // 分析进行中遇重武装 -> 中止
            if (rearm && state != S_IDLE) begin
                state <= S_IDLE;
                busy  <= 1'b0;
            end else begin
            case (state)
                // ---- 空闲: 等 512点采集完成 ----
                S_IDLE: begin
                    busy <= 1'b0;
                    if (cap_done && disp_len_sel == 2'b11) begin
                        f_bank     <= disp_origin[9];
                        f_org      <= disp_origin[8:0];
                        ps_cnt     <= 10'd0;
                        peak_mag   <= 24'd0;
                        imp        <= 10'd0;
                        sh_rd_addr <= {disp_origin[9], disp_origin[8:0]}; // pt=0
                        busy       <= 1'b1;
                        state      <= S_PSCAN;
                    end
                end

                // ---- 峰值扫描: 遍历 512 点找 |x| 峰值及其索引 ----
                //   发地址 ps_cnt(<512); 数据对应 ps_cnt-1, 于 ps_cnt>=1 处理
                S_PSCAN: begin
                    if (ps_cnt < 10'd512)
                        sh_rd_addr <= {f_bank, f_org + ps_cnt[8:0]};
                    if (ps_cnt >= 10'd1) begin
                        if (ps_abs > peak_mag) begin
                            peak_mag <= ps_abs;
                            imp      <= ps_cnt - 10'd1;
                        end
                    end
                    if (ps_cnt == 10'd512)
                        state <= S_VCHK;
                    else
                        ps_cnt <= ps_cnt + 10'd1;
                end

                // ---- 有效性判定 + 求 sh ----
                S_VCHK: begin
                    if (peak_mag < MIN_PEAK || imp >= IMP_WIN ||
                        (imp + SKIP[9:0] + SEGLEN[9:0]) > 10'd512) begin
                        // INVALID -> 直接出结果
                        result_state <= RES_INVALID;
                        confidence   <= 2'd0;
                        defect_class <= 3'd0;
                        distance_mm  <= 12'd0;
                        impact_index <= imp[8:0];
                        state        <= S_DONE;
                    end else begin
                        sh         <= sh_comb;
                        ld_cnt     <= 10'd0;
                        // 预取 seg 第0点: pt = imp+SKIP
                        sh_rd_addr <= {f_bank, f_org + imp[8:0] + SKIP[8:0]};
                        state      <= S_LOAD;
                    end
                end

                // ---- 载入缩放段: seg_ram[i] = x[imp+16+i] >>> sh, i=0..383 ----
                //   发地址 ld_cnt(<384 对应 pt=imp+16+ld_cnt+1 预取); 写 i=ld_cnt-1
                S_LOAD: begin
                    if (ld_cnt < 10'd384)
                        sh_rd_addr <= {f_bank,
                                       f_org + imp[8:0] + SKIP[8:0] + ld_cnt[8:0] + 9'd1};
                    if (ld_cnt >= 10'd1) begin
                        seg_we    <= 1'b1;
                        segA_addr <= ld_cnt[8:0] - 9'd1;
                        seg_wd    <= (sh_rd_q >>> sh);   // 算术右移取低16位
                    end
                    if (ld_cnt == 10'd384) begin
                        ac_k  <= 8'd0;                    // 第一个 lag = 0
                        state <= S_ACSET;
                    end else begin
                        ld_cnt <= ld_cnt + 10'd1;
                    end
                end

                // ---- 自相关: 设置当前 lag k ----
                S_ACSET: begin
                    ac_nk    <= SEGLEN[9:0] - 10'd1 - {2'd0, ac_k}; // 383-k
                    ac_i     <= 10'd0;
                    ac_acc   <= 48'sd0;
                    segA_addr <= 9'd0;
                    segB_addr <= {1'b0, ac_k};
                    state    <= S_ACRUN;
                end

                // ---- 自相关 MAC: acc += s[i]*s[i+k], i=0..383-k ----
                //   发地址 ac_i(<=nk); 数据对应 ac_i-1, 于 ac_i>=1 累加
                S_ACRUN: begin
                    if (ac_i <= ac_nk) begin
                        segA_addr <= ac_i[8:0];
                        segB_addr <= ac_i[8:0] + {1'b0, ac_k};
                    end
                    if (ac_i >= 10'd1)
                        ac_acc <= ac_acc + {{16{ac_prod[31]}}, ac_prod};
                    if (ac_i == ac_nk + 10'd1) begin
                        state <= S_ACSTO;
                    end else begin
                        ac_i <= ac_i + 10'd1;
                    end
                end

                // ---- 存 r[k], 推进到下一个 lag ----
                S_ACSTO: begin
                    r_we <= 1'b1;
                    r_wa <= ac_k;
                    r_wd <= ac_acc;
                    if (ac_k == 8'd0) begin
                        ac_k  <= LAG0[7:0];               // 0 -> 8
                        state <= S_ACSET;
                    end else if (ac_k == (NLAG[7:0]-8'd1)) begin
                        // 已完成 k=135, 进入缩放
                        r_ra  <= 8'd0;                    // 预读 r0
                        state <= S_SCLR0;
                    end else begin
                        ac_k  <= ac_k + 8'd1;
                        state <= S_ACSET;
                    end
                end

                // ---- 读 r0 (r_ra=0 已在上一拍设置, 此拍等 r_q) ----
                S_SCLR0: begin
                    state <= S_SCAL;
                end

                // ---- 二次缩放: rsh, rs0, rs0_82 ----
                S_SCAL: begin
                    r0_val <= r_q;
                    // rsh_comb 依赖 r0_val, 需再等一拍稳定 -> 用当前 r_q 直接算
                    rsh <= (r_q[47] ? 5'd18 : r_q[46] ? 5'd17 : r_q[45] ? 5'd16 :
                            r_q[44] ? 5'd15 : r_q[43] ? 5'd14 : r_q[42] ? 5'd13 :
                            r_q[41] ? 5'd12 : r_q[40] ? 5'd11 : r_q[39] ? 5'd10 :
                            r_q[38] ? 5'd9  : r_q[37] ? 5'd8  : r_q[36] ? 5'd7  :
                            r_q[35] ? 5'd6  : r_q[34] ? 5'd5  : r_q[33] ? 5'd4  :
                            r_q[32] ? 5'd3  : r_q[31] ? 5'd2  : r_q[30] ? 5'd1  : 5'd0);
                    // 初始化打分
                    score0 <= 64'sd0; score1 <= 64'sd0; score2 <= 64'sd0;
                    score3 <= 64'sd0; score4 <= 64'sd0;
                    sc_idx <= 8'd0;
                    r_ra   <= LAG0[7:0];      // idx=0 -> lag8
                    state  <= S_SCRW;
                end

                // ---- 打分: 等 r_ram 读出稳定 (r_ra 设置后需 2 拍) ----
                S_SCRW: begin
                    state <= S_SCRD;
                end

                // ---- 打分: 锁存 rsk, 首次计算 rs0 ----
                S_SCRD: begin
                    if (sc_idx == 8'd0) begin
                        rs0       <= (r0_val >>> rsh);
                        rs0_82    <= (r0_val >>> rsh) * MARGIN_Q;
                    end
                    // r_q 为当前 idx 对应的 r[lag] (已稳定)
                    rsk   <= (r_q >>> rsh);
                    sc_c  <= 3'd0;
                    state <= S_SCMAC;
                end

                // ---- 打分: 对当前 idx 的 5 类 MAC ----
                S_SCMAC: begin
                    case (sc_c)
                        3'd0: score0 <= score0 + mac_prod;
                        3'd1: score1 <= score1 + mac_prod;
                        3'd2: score2 <= score2 + mac_prod;
                        3'd3: score3 <= score3 + mac_prod;
                        default: score4 <= score4 + mac_prod;
                    endcase
                    if (sc_c == 3'd4) begin
                        if (sc_idx == 8'd128) begin
                            // 打分完成 -> argmax
                            am_c     <= 3'd0;
                            best_val <= 64'sh8000000000000000;
                            sec_val  <= 64'sh8000000000000000;
                            best_i   <= 3'd0;
                            state    <= S_ARG;
                        end else begin
                            sc_idx <= sc_idx + 8'd1;
                            // 预取下一 idx 的 r[lag]
                            r_ra   <= (sc_idx + 8'd1 == 8'd128) ? 8'd0
                                                               : (sc_idx + 8'd1 + LAG0[7:0]);
                            state  <= S_SCRW;
                        end
                    end else begin
                        sc_c <= sc_c + 3'd1;
                    end
                end

                // ---- argmax: 顺序扫描 5 个 score, 记录 top1/top2 ----
                S_ARG: begin
                    if (am_c == 3'd5) begin
                        // ---- 800mm 二次判别入口 ----
                        //   800mm 缺陷波形与好棒高度相似, LDA 易误判 good;
                        //   实测好棒 T0<=102, 800mm缺陷 T0>=103;
                        //   所有好棒判定都扫描 T0 二次确认 (仅多 42 拍≈1us)
                        if (best_i == 3'd0) begin
                            d800_lag      <= D800_LAG_LO;
                            d800_peak_lag <= D800_LAG_LO;
                            d800_peak_val <= 48'sh800000000000;
                            d800_phase    <= 1'b0;
                            r_ra          <= D800_LAG_LO;
                            state         <= S_D800;
                        end else begin
                            // 缺陷类直接出结果
                            result_state <= RES_DEFECT;
                            confidence   <= ((best_val - sec_val) < $signed(rs0_82))
                                            ? 2'd1 : 2'd2;   // margin<rs0*82 -> 低置信
                            defect_class <= best_i;
                            distance_mm  <= dist_lut(best_i);
                            impact_index <= imp[8:0];
                            state        <= S_DONE;
                        end
                    end else begin
                        // 顺序比较当前 am_c 的 score, 更新 top1/top2
                        if (am_val > best_val) begin
                            sec_val  <= best_val;
                            best_val <= am_val;
                            best_i   <= am_c;
                        end else if (am_val > sec_val) begin
                            sec_val  <= am_val;
                        end
                        am_c <= am_c + 3'd1;
                    end
                end

                // ---- 800mm 二次判别: 扫描 r[90..110] 找基波周期峰值位置 ----
                //   r_ram 读延迟1拍: r_ra设置后下一拍 r_q 才稳定, 用 d800_phase 控制
                S_D800: begin
                    if (d800_phase == 1'b0) begin
                        // 等 r_q 稳定
                        d800_phase <= 1'b1;
                    end else begin
                        // r_q 已稳定, 读取比较
                        if (r_q > d800_peak_val) begin
                            d800_peak_val <= r_q;
                            d800_peak_lag <= d800_lag;
                        end
                        if (d800_lag == D800_LAG_HI) begin
                            state <= S_D800J;
                        end else begin
                            d800_lag   <= d800_lag + 8'd1;
                            r_ra       <= d800_lag + 8'd1;
                            d800_phase <= 1'b0;
                        end
                    end
                end

                // ---- 800mm 判决: T0 >= 100 且 good/d800 得分接近 -> d800 ----
                S_D800J: begin
                    if (d800_peak_lag >= D800_T_TH) begin
                        // 基波周期偏长 -> 800mm 缺陷
                        result_state <= RES_DEFECT;
                        confidence   <= 2'd1;   // 低置信 (二次判别)
                        defect_class <= 3'd4;   // d800
                        distance_mm  <= 12'd800;
                    end else begin
                        // 基波周期正常 -> 维持 good 判定
                        result_state <= RES_NORMAL;
                        confidence   <= ((best_val - sec_val) < $signed(rs0_82))
                                        ? 2'd1 : 2'd2;
                        defect_class <= 3'd0;
                        distance_mm  <= 12'd0;
                    end
                    impact_index <= imp[8:0];
                    state        <= S_DONE;
                end

                // ---- 出结果 (1拍脉冲) ----
                S_DONE: begin
                    result_valid <= 1'b1;
                    busy         <= 1'b0;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
            end
        end
    end

endmodule
