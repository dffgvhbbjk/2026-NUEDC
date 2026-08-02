// ============================================================================
// Module: wave_capture
// Description: 64/128/256/512 点触发采集状态机 (文档 §11 第五阶段)
//
// 状态机 (文档 §11.1):
//   IDLE    上电/复位后基线预热 (WARMUP_POINTS 个样点), 期间持续写 RAM
//   ARMED   循环写入当前 bank (预触发数据), 等待 wave_trigger 触发
//   CAPTURE 继续写入, 由 sample_valid 计数收满 N 点 (禁止 VGA 定时, §3.6)
//   HOLD    停止写入, 冻结显示; rearm_ok (静默迟滞) 后回 ARMED
//
// 采集长度 (文档 §11.2): len_sel 00→64, 01→128, 10→256, 11→512
//   触发瞬间锁存长度, 采集中按键不影响本次采集
//
// 预触发 (文档 §11.3): 64→8, 128→16, 256→32, 512→64
//   display_origin = trigger_ptr - pretrigger_count
//   post_target    = 采集长度 - 预触发点数 (触发样点计为第 1 个 post 点)
//   触发时起点先存入 pending_origin, 采满最后一点才提交到 disp_origin,
//   避免第二次锤击时屏幕提前切到正在写入的 bank (撕裂/残缺波形)
//
// RAM 组织 (1024×16 双口, 文档 §12.2 推荐方案):
//   按 512 点分为 bank0 (0~511) / bank1 (512~1023) 乒乓使用:
//   - ARMED/CAPTURE 循环写入活动 bank (bank 内 8 位指针回绕)
//   - HOLD 完成时: 显示 bank ← 活动 bank, 活动 bank 翻转
//   显示与写入分离 → 显示期间 RAM 数据不被覆盖 (验收标准 §21),
//   旧波形保持显示直到新采集完成 (无需额外双缓冲 RAM)
//
// 时钟域: vga_clk 40MHz
// ============================================================================
module wave_capture #(
    parameter WARMUP_POINTS      = 2048,  // IDLE 基线预热点数 (~21ms @95.88kHz)
    parameter HOLD_TIMEOUT_POINTS = 16384 // HOLD 最长等待 (~171ms), 防底噪导致永久卡死
) (
    input  wire               clk,             // vga_clk 40MHz
    input  wire               rst_n,           // 低有效异步复位

    // 来自 wave_trigger (三者同拍)
    input  wire signed [23:0] corrected_data,  // 基线扣除后数据
    input  wire               corrected_valid, // 数据有效脉冲
    input  wire               trigger,         // 触发脉冲
    input  wire               rearm_ok,        // 静默达标电平 (重新武装条件)

    // 配置/控制
    input  wire [1:0]         len_sel,         // 采集长度: 00=64 01=128 10=256
    input  wire               force_rearm,     // 按键强制重新武装 (清屏回 ARMED)

    // 到 wave_trigger 的控制
    output wire               base_en,         // 基线跟踪使能 (仅 CAPTURE 冻结)
    output wire               track_en,        // 噪声窗口跟踪使能 (IDLE/ARMED)
    output wire               arm_en,          // 触发使能 (ARMED 且预触发已填满)

    // RAM 写口 (wave_buffer_dp)
    output wire               wr_en,           // RAM 写使能
    output wire [9:0]         wr_addr,         // RAM 写地址 {bank, ptr}
    output wire signed [15:0] wr_data,         // 写数据 = corrected[23:8] (§12.1)

    // 到显示模块 (HOLD 完成时更新, 显示期间稳定)
    output reg                display_en,      // 1=显示波形
    output reg  [9:0]         disp_origin,     // 显示起点 {bank, origin}
    output reg  [1:0]         disp_len_sel,    // 本次采集长度选择 (已锁存)

    // 自动增益 (文档 §13.2, P2): 采集完成时按本次波形峰值选档, 与
    // disp_origin 同拍提交, 显示期间稳定; 只影响显示不改 RAM
    output reg  [2:0]         auto_gain,       // 0~4 → ×1~×16
    output reg                cap_done,        // 采集完成单周期脉冲 (UART 导出用)
    output reg                rearm_pulse,     // 重武装单周期脉冲, 清 wave_trigger 历史

    // 调试观测
    output reg  [1:0]         dbg_state        // 0=IDLE 1=ARMED 2=CAPTURE 3=HOLD
);

    localparam ST_IDLE    = 2'd0;
    localparam ST_ARMED   = 2'd1;
    localparam ST_CAPTURE = 2'd2;
    localparam ST_HOLD    = 2'd3;

    // ------------------------------------------------------------------------
    // 长度/预触发解码 (触发瞬间锁存, 文档 §11.2/§11.3)
    // ------------------------------------------------------------------------
    wire [9:0] len_dec  = (len_sel == 2'b00) ? 10'd64  :
                          (len_sel == 2'b01) ? 10'd128 :
                          (len_sel == 2'b10) ? 10'd256 : 10'd512;
    wire [6:0] pre_dec  = (len_sel == 2'b00) ? 7'd8   :
                          (len_sel == 2'b01) ? 7'd16  :
                          (len_sel == 2'b10) ? 7'd32  : 7'd64;

    reg [9:0] post_target;    // 触发后需采点数 = len - pretrig (含触发样点)
    reg [9:0] post_cnt;       // 已采 post 点数
    reg [1:0] cap_len_r;      // 本次采集长度选择 (锁存)
    reg [9:0] pending_origin; // 本次采集显示起点 (采集完成时才提交到 disp_origin)

    // ------------------------------------------------------------------------
    // 写指针: bank 内 9 位循环 (512 点), wr_bank 选择上/下半区
    // ------------------------------------------------------------------------
    reg       wr_bank;        // 活动写 bank
    reg [8:0] wr_ptr;         // bank 内写指针
    reg [6:0] fill_cnt;       // 进入 ARMED 后已写点数 (饱和), 保证预触发数据有效

    assign wr_en   = corrected_valid && (dbg_state != ST_HOLD);
    assign wr_addr = {wr_bank, wr_ptr};
    assign wr_data = corrected_data[23:8];

    // 预触发缓冲填满判断: 按最大预触发 64 点要求, 与长度选择无关
    wire fill_ok = (fill_cnt >= 7'd64);

    // ------------------------------------------------------------------------
    // 自动增益峰值跟踪 (文档 §13.2): 覆盖完整 N 点显示窗口
    //   - CAPTURE 段: 触发样点起逐点跟踪 |wr_data| 最大值
    //   - 预触发段: ARMED 期间用双 32 点窗口跟踪近期峰值 (覆盖最近 32~64 点,
    //     ≥ 最大预触发 32 点), 触发瞬间并入 cap_peak。窗口可能多覆盖至多
    //     32 个窗口外样点 → 档位只会偏保守 (更小增益), 不会削顶
    //   采集完成时按峰值选档 (峰值占纵向显示区约 70%~85%)
    // ------------------------------------------------------------------------
    reg  [15:0] cap_peak;
    reg  [15:0] pre_win_cur;      // 当前累计窗口峰值
    reg  [15:0] pre_win_prev;     // 上一完整窗口峰值
    reg  [4:0]  pre_win_cnt;
    wire [15:0] samp_mag = wr_data[15] ? (~wr_data + 1'b1) : wr_data;
    wire [15:0] peak_new = (samp_mag > cap_peak) ? samp_mag : cap_peak;
    wire [15:0] pre_peak = (pre_win_cur > pre_win_prev) ? pre_win_cur : pre_win_prev;
    wire [15:0] trig_peak =                        // 触发拍: 预触发峰值 ∪ 触发样点
        (samp_mag > pre_peak) ? samp_mag : pre_peak;
    wire [2:0]  gain_from_peak =
        (peak_new >= 16'd16384) ? 3'd0 :      // ×1
        (peak_new >= 16'd8192)  ? 3'd1 :      // ×2
        (peak_new >= 16'd4096)  ? 3'd2 :      // ×4
        (peak_new >= 16'd2048)  ? 3'd3 :      // ×8
                                  3'd4;       // ×16

    // 预触发峰值双窗口 (仅 ARMED 累计, 离开 ARMED 清零)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pre_win_cur  <= 16'd0;
            pre_win_prev <= 16'd0;
            pre_win_cnt  <= 5'd0;
        end else if (dbg_state != ST_ARMED) begin
            pre_win_cur  <= 16'd0;
            pre_win_prev <= 16'd0;
            pre_win_cnt  <= 5'd0;
        end else if (corrected_valid) begin
            if (pre_win_cnt == 5'd31) begin
                pre_win_cnt  <= 5'd0;
                pre_win_prev <= (samp_mag > pre_win_cur) ? samp_mag : pre_win_cur;
                pre_win_cur  <= 16'd0;
            end else begin
                pre_win_cnt <= pre_win_cnt + 1'b1;
                if (samp_mag > pre_win_cur)
                    pre_win_cur <= samp_mag;
            end
        end
    end

    assign base_en  = (dbg_state != ST_CAPTURE);   // HOLD 恢复基线跟踪 (防死锁)
    assign track_en = (dbg_state == ST_IDLE) || (dbg_state == ST_ARMED);
    assign arm_en   = (dbg_state == ST_ARMED) && fill_ok;

    // ------------------------------------------------------------------------
    // 预热计数 (IDLE): 让基线/噪声峰值收敛后再武装
    // ------------------------------------------------------------------------
    reg [11:0] warmup_cnt;
    reg [15:0] hold_cnt;

    // ------------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_state    <= ST_IDLE;
            warmup_cnt   <= 12'd0;
            wr_bank      <= 1'b0;
            wr_ptr       <= 9'd0;
            fill_cnt     <= 7'd0;
            post_target  <= 10'd0;
            post_cnt     <= 10'd0;
            cap_len_r    <= 2'b00;
            pending_origin <= 10'd0;
            cap_peak     <= 16'd0;
            auto_gain    <= 3'd0;
            cap_done     <= 1'b0;
            rearm_pulse  <= 1'b0;
            hold_cnt     <= 16'd0;
            display_en   <= 1'b0;
            disp_origin  <= 10'd0;
            disp_len_sel <= 2'b00;
        end else begin
            cap_done    <= 1'b0;
            rearm_pulse <= 1'b0;
            // ---- 按键强制重新武装: 清屏, 指针复位, 回 ARMED (最高优先级) ----
            // IDLE (基线预热) 期间忽略, 防止绕过 ~21ms 预热导致误触发
            if (force_rearm && (dbg_state != ST_IDLE)) begin
                dbg_state  <= ST_ARMED;
                display_en <= 1'b0;
                wr_ptr     <= 9'd0;
                fill_cnt   <= 7'd0;
                hold_cnt   <= 16'd0;
                rearm_pulse <= 1'b1;
            end else begin
                case (dbg_state)
                    // ---- 基线预热: 持续写入 bank0, 计满后武装 ----
                    ST_IDLE: begin
                        if (corrected_valid) begin
                            wr_ptr <= wr_ptr + 1'b1;
                            if (warmup_cnt == WARMUP_POINTS - 1)
                                dbg_state <= ST_ARMED;
                            else
                                warmup_cnt <= warmup_cnt + 1'b1;
                        end
                    end

                    // ---- 武装: 循环写预触发数据, 等待触发 ----
                    ST_ARMED: begin
                        if (corrected_valid) begin
                            wr_ptr <= wr_ptr + 1'b1;
                            if (!fill_ok)
                                fill_cnt <= fill_cnt + 1'b1;

                            // 触发 (与 corrected_valid 同拍): 触发样点本拍已写入
                            if (trigger && fill_ok) begin
                                dbg_state   <= ST_CAPTURE;
                                cap_len_r   <= len_sel;
                                post_target <= len_dec - {3'd0, pre_dec};
                                post_cnt    <= 10'd1;   // 触发样点 = 第 1 个 post 点
                                // 显示起点 = 触发指针 - 预触发点数 (bank 内回绕)
                                // 先存 pending, 采集完成时才切换显示 (防撕裂)
                                pending_origin <= {wr_bank, wr_ptr - {2'd0, pre_dec}};
                                // 峰值跟踪: 预触发窗口峰值 ∪ 触发样点
                                cap_peak    <= trig_peak;
                            end
                        end
                    end

                    // ---- 采集: 按 sample_valid 计数收满 post_target 点 ----
                    ST_CAPTURE: begin
                        if (corrected_valid) begin
                            wr_ptr   <= wr_ptr + 1'b1;
                            post_cnt <= post_cnt + 1'b1;
                            cap_peak <= peak_new;
                            if (post_cnt == post_target - 1) begin
                                // 本拍写入最后一点 → 提交显示切换, 翻转 bank
                                dbg_state    <= ST_HOLD;
                                display_en   <= 1'b1;
                                disp_origin  <= pending_origin;
                                disp_len_sel <= cap_len_r;
                                auto_gain    <= gain_from_peak;   // 含最后一点
                                cap_done     <= 1'b1;
                                wr_bank      <= ~wr_bank;
                                wr_ptr       <= 9'd0;
                                fill_cnt     <= 7'd0;
                                hold_cnt     <= 16'd0;
                            end
                        end
                    end

                    // ---- 冻结: 静默达标后重新武装; 超时兜底防永久卡死 ----
                    ST_HOLD: begin
                        if (rearm_ok) begin
                            dbg_state <= ST_ARMED;
                            fill_cnt <= 7'd0;
                            hold_cnt <= 16'd0;
                            rearm_pulse <= 1'b1;
                        end else if (corrected_valid) begin
                            if (hold_cnt == HOLD_TIMEOUT_POINTS - 1) begin
                                dbg_state <= ST_ARMED;
                                fill_cnt <= 7'd0;
                                hold_cnt <= 16'd0;
                                rearm_pulse <= 1'b1;
                            end else begin
                                hold_cnt <= hold_cnt + 1'b1;
                            end
                        end
                    end

                    default: dbg_state <= ST_IDLE;
                endcase
            end
        end
    end

endmodule
