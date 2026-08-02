// ============================================================================
// Module: wave_uart_export
// Description: 采集波形 UART 导出 (P2 第 5 项 + 冻结版方案 §12.7 版本化结果段)
//
// 原理:
//   - 内部 1024×24 原始影子 RAM: 与 wave_capture 的 RAM 写口同址同拍,
//     写入的是未经基线扣除/平滑的 PCM1808 原始 24 位样点 (wr_raw,
//     由 wave_trigger/wave_smooth 的随动通道对齐到同一样点)
//   - 显示 RAM 存处理后 16 位数据用于 VGA; 影子 RAM 存原始数据供
//     PC 端模板制作/阈值标定/缺陷识别使用, 两者地址一一对应
//   - cap_done 脉冲到来时锁存 disp_origin/disp_len_sel, 自动导出一帧
//   - 导出期间新的 cap_done 被忽略 (115200bps 一帧 256 点 ≈ 67ms,
//     仍小于重新武装静默期 ~85ms, 正常流程不会发生重叠)
//
// 帧格式 (共 4 + 3N + 14 字节, MSB first):
//   0xAA 0x55        同步头
//   len_sel          0=64 点, 1=128 点, 2=256 点, 3=512 点
//   sample[k][23:16], [15:8], [7:0] ... (有符号 24 位原始样点)
//   checksum         len_sel 与全部数据字节的 8 位累加和
//   ---- 版本化结果段 (冻结版 §12.7, 13 字节) ----
//   0x5A             结果段标记
//   ver              0x02 = 冻结版三态格式 (旧 6 字节格式无版本号)
//   status           bit[1:0]=result_state (0=INVALID 1=NORMAL 2=DEFECT)
//                    bit[3:2]=confidence   (0=无 1=低 2=高)
//                    bit4    =result_ready
//   idx_h            bit0=impact[8] bit1=defect[8] bit2=bottom[8]
//   impact_index[7:0]
//   defect_index[7:0]
//   bottom_index[7:0]
//   dist[11:8], dist[7:0]   缺陷距离 mm
//   thr[23:16], [15:8], [7:0]  本次触发阈值 (模板/复盘用)
//   rsum             ver 起 11 字节的 8 位累加和
//
// 时钟域: vga_clk 40MHz (与 wave_capture 同域, 无 CDC)
// UART: 115200bps, 8N1, uart_tx 参数 CLK_FREQ=40MHz
// ============================================================================
module wave_uart_export #(
    parameter UART_BPS = 115_200       // 硬件 115200; TB 覆盖为 2M 加速仿真
) (
    input  wire               clk,             // vga_clk 40MHz
    input  wire               rst_n,           // 低有效异步复位

    // 镜像 wave_capture 的 RAM 写口 (地址/使能相同, 数据为原始 24 位)
    input  wire               wr_en,
    input  wire [9:0]         wr_addr,
    input  wire signed [23:0] wr_raw,

    // 采集完成通知 (与 disp_origin/disp_len_sel 同拍有效)
    input  wire               cap_done,
    input  wire [9:0]         disp_origin,     // {bank, origin}
    input  wire [1:0]         disp_len_sel,    // 00=64 01=128 10=256 11=512

    // 缺陷检测结果 (帧尾发送时采样; 结果在波形段发送期间早已稳定,
    //   analyzer 结果晚 cap_done ~3 拍, 距离晚 ~25 拍, 首字节耗时 ~87µs)
    input  wire               result_ready,      // 1=已有分析结果 (电平保持)
    input  wire [1:0]         result_state,      // 三态结果
    input  wire [1:0]         confidence,        // 置信度
    input  wire [8:0]         impact_index,      // 主冲击峰索引
    input  wire [8:0]         defect_index,      // 缺陷反射峰索引
    input  wire [8:0]         bottom_index,      // 棒底反射峰索引
    input  wire [11:0]        distance_mm,       // 缺陷距离 mm
    input  wire [23:0]        trigger_threshold, // 本次触发阈值 (锁存)

    output wire               busy,            // 导出进行中 (SignalTap 可观测)
    output wire               uart_txd         // 115200bps 串行输出
);

    localparam [7:0] RESULT_VER = 8'h02;       // 冻结版三态帧尾格式

    // ------------------------------------------------------------------------
    // 原始影子 RAM: 1024×24, 读口专用于导出 (推断 M9K, 无复位)
    // ------------------------------------------------------------------------
    (* ramstyle = "M9K" *) reg signed [23:0] shadow_ram [0:1023];
    reg signed [23:0] rd_q;
    reg [9:0]         rd_addr;

    always @(posedge clk) begin
        if (wr_en)
            shadow_ram[wr_addr] <= wr_raw;
    end

    always @(posedge clk) begin
        rd_q <= shadow_ram[rd_addr];
    end

    // ------------------------------------------------------------------------
    // UART TX (115200bps @ 40MHz, 实际约 115274bps)
    // ------------------------------------------------------------------------
    reg  [7:0] tx_data;
    reg        tx_flag;
    wire       tx_ready;

    uart_tx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (40_000_000)
    ) u_tx (
        .sys_clk   (clk),
        .sys_rst_n (rst_n),
        .pi_data   (tx_data),
        .pi_flag   (tx_flag),
        .ready     (tx_ready),
        .tx        (uart_txd)
    );

    // ------------------------------------------------------------------------
    // 导出状态机
    //   字节序列: HDR0(AA) → HDR1(55) → LEN → {S2,S1,S0}×N → SUM
    //             → RMK(5A) → VER → RST → IDXH → IMP → DEF → BOT
    //             → RDH → RD0 → TH2 → TH1 → TH0 → RSUM
    //   每字节: SEND (pi_flag 脉冲) → WAIT (ready 落下再抬起) → 下一字节
    // ------------------------------------------------------------------------
    localparam ST_IDLE   = 2'd0;
    localparam ST_SEND   = 2'd1;   // 发出当前字节
    localparam ST_WAIT   = 2'd2;   // 等 uart_tx 完成 (ready 0→1)
    localparam ST_FETCH  = 2'd3;   // 读影子 RAM (1 拍延迟)

    // 字节阶段
    localparam PH_HDR0 = 5'd0;
    localparam PH_HDR1 = 5'd1;
    localparam PH_LEN  = 5'd2;
    localparam PH_S2   = 5'd3;     // 样点 [23:16]
    localparam PH_S1   = 5'd4;     // 样点 [15:8]
    localparam PH_S0   = 5'd5;     // 样点 [7:0]
    localparam PH_SUM  = 5'd6;
    localparam PH_RMK  = 5'd7;     // 结果段标记 0x5A
    localparam PH_VER  = 5'd8;     // 版本号 0x02
    localparam PH_RST  = 5'd9;     // status (三态/置信度/ready)
    localparam PH_IDXH = 5'd10;    // 索引高位集合
    localparam PH_IMP  = 5'd11;    // impact_index[7:0]
    localparam PH_DEF  = 5'd12;    // defect_index[7:0]
    localparam PH_BOT  = 5'd13;    // bottom_index[7:0]
    localparam PH_RDH  = 5'd14;    // distance[11:8]
    localparam PH_RD0  = 5'd15;    // distance[7:0]
    localparam PH_TH2  = 5'd16;    // trigger_threshold[23:16]
    localparam PH_TH1  = 5'd17;    // trigger_threshold[15:8]
    localparam PH_TH0  = 5'd18;    // trigger_threshold[7:0]
    localparam PH_RSUM = 5'd19;    // 结果段校验和

    reg [1:0] state;
    reg [4:0] phase;
    reg       ready_d;             // tx_ready 打拍, 检测上升沿

    reg       exp_bank;            // 锁存的 bank
    reg [8:0] exp_org;             // 锁存的 bank 内起点
    reg [1:0] exp_len_sel;
    reg [9:0] exp_total;           // 总点数 64/128/256/512
    reg [9:0] pt;                  // 当前样点序号
    reg [7:0] chksum;              // 波形段 8 位累加和 (len + 数据字节)
    reg [7:0] rsum;                // 结果段 8 位累加和 (ver..th0)

    // 结果段锁存 (PH_SUM 完成时采样, 保证 13 字节内部一致)
    reg        r_ready;
    reg [1:0]  r_state;
    reg [1:0]  r_conf;
    reg [8:0]  r_imp;
    reg [8:0]  r_def;
    reg [8:0]  r_bot;
    reg [11:0] r_dist;
    reg [23:0] r_thr;

    // 结果段字节 (锁存值组合译码)
    wire [7:0] b_status = {3'd0, r_ready, r_conf, r_state};
    wire [7:0] b_idxh   = {5'd0, r_bot[8], r_def[8], r_imp[8]};

    wire [9:0] len_total = (exp_len_sel == 2'b00) ? 10'd64  :
                           (exp_len_sel == 2'b01) ? 10'd128 :
                           (exp_len_sel == 2'b10) ? 10'd256 : 10'd512;

    assign busy = (state != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            phase       <= PH_HDR0;
            tx_data     <= 8'd0;
            tx_flag     <= 1'b0;
            ready_d     <= 1'b1;
            exp_bank    <= 1'b0;
            exp_org     <= 9'd0;
            exp_len_sel <= 2'b00;
            exp_total   <= 10'd0;
            pt          <= 10'd0;
            chksum      <= 8'd0;
            rsum        <= 8'd0;
            rd_addr     <= 10'd0;
            r_ready     <= 1'b0;
            r_state     <= 2'd0;
            r_conf      <= 2'd0;
            r_imp       <= 9'd0;
            r_def       <= 9'd0;
            r_bot       <= 9'd0;
            r_dist      <= 12'd0;
            r_thr       <= 24'd0;
        end else begin
            tx_flag <= 1'b0;
            ready_d <= tx_ready;

            case (state)
                // ---- 空闲: 等采集完成脉冲 ----
                ST_IDLE: begin
                    if (cap_done) begin
                        exp_bank    <= disp_origin[9];
                        exp_org     <= disp_origin[8:0];
                        exp_len_sel <= disp_len_sel;
                        phase       <= PH_HDR0;
                        pt          <= 10'd0;
                        chksum      <= 8'd0;
                        rsum        <= 8'd0;
                        state       <= ST_SEND;
                    end
                end

                // ---- 发出当前字节 ----
                ST_SEND: begin
                    case (phase)
                        PH_HDR0: tx_data <= 8'hAA;
                        PH_HDR1: tx_data <= 8'h55;
                        PH_LEN:  tx_data <= {6'd0, exp_len_sel};
                        PH_S2:   tx_data <= rd_q[23:16];
                        PH_S1:   tx_data <= rd_q[15:8];
                        PH_S0:   tx_data <= rd_q[7:0];
                        PH_SUM:  tx_data <= chksum;
                        PH_RMK:  tx_data <= 8'h5A;
                        PH_VER:  tx_data <= RESULT_VER;
                        PH_RST:  tx_data <= b_status;
                        PH_IDXH: tx_data <= b_idxh;
                        PH_IMP:  tx_data <= r_imp[7:0];
                        PH_DEF:  tx_data <= r_def[7:0];
                        PH_BOT:  tx_data <= r_bot[7:0];
                        PH_RDH:  tx_data <= {4'd0, r_dist[11:8]};
                        PH_RD0:  tx_data <= r_dist[7:0];
                        PH_TH2:  tx_data <= r_thr[23:16];
                        PH_TH1:  tx_data <= r_thr[15:8];
                        PH_TH0:  tx_data <= r_thr[7:0];
                        default: tx_data <= rsum;     // PH_RSUM
                    endcase
                    tx_flag <= 1'b1;
                    state   <= ST_WAIT;
                    // 锁存总点数 (LEN 阶段起 exp_len_sel 已稳定)
                    exp_total <= len_total;
                end

                // ---- 等待发送完成: ready 上升沿 ----
                ST_WAIT: begin
                    // 累加校验和 (波形段: len 与数据字节)
                    if (tx_flag && ((phase == PH_LEN) || (phase == PH_S2) ||
                        (phase == PH_S1) || (phase == PH_S0)))
                        chksum <= chksum + tx_data;
                    // 累加结果段校验和 (ver..th0, 不含 5A 标记)
                    if (tx_flag && (phase >= PH_VER) && (phase <= PH_TH0))
                        rsum <= rsum + tx_data;

                    if (tx_ready && !ready_d) begin
                        case (phase)
                            PH_HDR0: begin phase <= PH_HDR1; state <= ST_SEND; end
                            PH_HDR1: begin
                                // 预取第 0 个样点
                                rd_addr <= {exp_bank, exp_org};
                                phase   <= PH_LEN;
                                state   <= ST_SEND;
                            end
                            PH_LEN:  begin phase <= PH_S2; state <= ST_FETCH; end
                            PH_S2:   begin phase <= PH_S1; state <= ST_SEND; end
                            PH_S1:   begin phase <= PH_S0; state <= ST_SEND; end
                            PH_S0: begin
                                if (pt == exp_total - 1) begin
                                    phase <= PH_SUM;
                                    state <= ST_SEND;
                                end else begin
                                    pt      <= pt + 1'b1;
                                    rd_addr <= {exp_bank, exp_org + pt[8:0] + 9'd1};
                                    phase   <= PH_S2;
                                    state   <= ST_FETCH;
                                end
                            end
                            PH_SUM: begin
                                // 波形段完毕, 锁存结果段 (保证 13 字节内部一致)
                                r_ready <= result_ready;
                                r_state <= result_state;
                                r_conf  <= confidence;
                                r_imp   <= impact_index;
                                r_def   <= defect_index;
                                r_bot   <= bottom_index;
                                r_dist  <= distance_mm;
                                r_thr   <= trigger_threshold;
                                phase   <= PH_RMK;
                                state   <= ST_SEND;
                            end
                            PH_RMK:  begin phase <= PH_VER;  state <= ST_SEND; end
                            PH_VER:  begin phase <= PH_RST;  state <= ST_SEND; end
                            PH_RST:  begin phase <= PH_IDXH; state <= ST_SEND; end
                            PH_IDXH: begin phase <= PH_IMP;  state <= ST_SEND; end
                            PH_IMP:  begin phase <= PH_DEF;  state <= ST_SEND; end
                            PH_DEF:  begin phase <= PH_BOT;  state <= ST_SEND; end
                            PH_BOT:  begin phase <= PH_RDH;  state <= ST_SEND; end
                            PH_RDH:  begin phase <= PH_RD0;  state <= ST_SEND; end
                            PH_RD0:  begin phase <= PH_TH2;  state <= ST_SEND; end
                            PH_TH2:  begin phase <= PH_TH1;  state <= ST_SEND; end
                            PH_TH1:  begin phase <= PH_TH0;  state <= ST_SEND; end
                            PH_TH0:  begin phase <= PH_RSUM; state <= ST_SEND; end
                            default: begin              // PH_RSUM 完成
                                state <= ST_IDLE;
                            end
                        endcase
                    end
                end

                // ---- 影子 RAM 读延迟 1 拍 ----
                ST_FETCH: begin
                    state <= ST_SEND;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
