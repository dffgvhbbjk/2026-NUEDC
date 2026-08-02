`timescale 1ns/1ns
//======================================================================
// tb_ov5640_config.v — 验证当前 ov5640_config (官方 i2c_ctrl + 257 寄存器表)
//
// 方法:
//   - 实例化 DUT ov5640_config (含官方 i2c_ctrl 控制器 + 配置顺序器)
//   - 标准 SCCB/I2C 从机模型模拟 OV5640:
//       * SCL 高时 SDA 下降沿 = START
//       * SCL 高时 SDA 上升沿 = STOP
//       * SCL 上升沿采样数据位
//       * 每字节第 9 拍 ACK (拉低 SDA)
//       * 记录收到的 {16-bit 寄存器地址, 8-bit 数据}
//   - 等 config_done (带 150ms 超时)
//   - 逐条比对收到的写入与 DUT 内部 config_rom, 打印 PASS/FAIL
//
// 通过标准: 收到 257 个写事务 且 全部与配置表一致 且 config_done 拉高
//           且设备地址字节恒为 0x78
//======================================================================
module tb_ov5640_config;

    reg  sys_clk = 0;
    reg  rst_n   = 0;
    wire scl;
    wire sda;
    wire config_done;

    always #10 sys_clk = ~sys_clk;   // 50MHz

    // ---- DUT: 当前 ov5640_config ----
    ov5640_config u_dut (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .sclk        (scl),
        .sdat        (sda),
        .config_done (config_done)
    );

    //==================================================================
    // SCCB 从机模型 (标准 I2C 语义, 模拟 OV5640)
    //==================================================================
    reg         scl_d = 0;
    reg         sda_d = 0;
    reg         in_txn = 0;
    reg  [3:0]  bit_cnt = 0;    // 本字节已收数据位数 0..8 (8 后为 ACK 拍)
    reg  [7:0]  byte_buf = 0;
    reg  [1:0]  byte_idx = 0;   // 0=dev, 1=reg_hi, 2=reg_lo, 3=data
    reg  [15:0] reg_addr_r = 0;
    reg         ack_pending = 0;

    wire scl_rise = in_txn && scl && !scl_d;
    wire scl_fall = in_txn && !scl && scl_d;

    assign sda = ack_pending ? 1'b0 : 1'bz;   // 从机 ACK 驱动 (开漏)

    // 写入日志
    reg [15:0] wr_addr_log [0:511];
    reg  [7:0] wr_data_log [0:511];
    reg  [9:0] wr_count = 0;
    reg        bad_dev_addr = 0;

    always @(posedge sys_clk) begin
        scl_d <= scl;
        sda_d <= sda;
        if (!rst_n) begin
            in_txn      <= 1'b0;
            bit_cnt     <= 4'd0;
            byte_idx    <= 2'd0;
            byte_buf    <= 8'd0;
            reg_addr_r  <= 16'd0;
            ack_pending <= 1'b0;
            wr_count    <= 10'd0;
            bad_dev_addr<= 1'b0;
        end else begin
            // ---- START: SDA 下降沿 且 SCL 高 ----
            if (scl && sda_d && !sda) begin
                in_txn     <= 1'b1;
                bit_cnt    <= 4'd0;
                byte_idx   <= 2'd0;
                reg_addr_r <= 16'd0;
            end
            // ---- STOP: SDA 上升沿 且 SCL 高 ----
            else if (scl && !sda_d && sda) begin
                in_txn      <= 1'b0;
                ack_pending <= 1'b0;
            end
            else if (in_txn) begin
                if (scl_rise) begin
                    if (bit_cnt < 8) begin
                        byte_buf <= {byte_buf[6:0], sda};
                        bit_cnt  <= bit_cnt + 1'b1;
                    end else begin
                        // 第 9 拍 (ACK 时钟): 完整字节已收, 处理
                        bit_cnt <= 4'd0;
                        case (byte_idx)
                            2'd0: begin   // 设备地址 (写 = 0x78)
                                if (byte_buf != 8'h78) bad_dev_addr <= 1'b1;
                            end
                            2'd1: reg_addr_r[15:8] <= byte_buf;
                            2'd2: reg_addr_r[7:0]  <= byte_buf;
                            2'd3: begin   // 数据字节 → 记录一次寄存器写入
                                wr_addr_log[wr_count] <= reg_addr_r;
                                wr_data_log[wr_count] <= byte_buf;
                                wr_count <= wr_count + 1'b1;
                            end
                        endcase
                        byte_idx <= byte_idx + 1'b1;
                    end
                end
                else if (scl_fall) begin
                    // 第 8 位结束后拉低 SDA → ACK; ACK 时钟结束后释放
                    if (bit_cnt == 8)     ack_pending <= 1'b1;
                    else if (bit_cnt == 0) ack_pending <= 1'b0;
                end
            end
        end
    end

    //==================================================================
    // 检查流程
    //==================================================================
    integer cmp_i;
    reg     all_ok;

    initial begin
        $display("=======================================================");
        $display(" tb_ov5640_config: 当前 ov5640_config SCCB 配置仿真");
        $display(" DUT = ov5640_config (官方 i2c_ctrl, 257 寄存器)");
        $display("=======================================================");
        #300;
        rst_n = 1;
        $display("[tb] 复位释放, 等待 config_done ...");

        // 等待 config_done (带 150ms 看门狗, 见下方独立 initial)
        while (!config_done) begin
            #10000;   // 每 10us 轮询一次
        end

        $display("[tb] config_done = 1, 开始比对写入 ...");
        all_ok = 1'b1;

        if (wr_count != 257) begin
            $display("[FAIL] 写事务数 = %0d, 期望 257", wr_count);
            all_ok = 1'b0;
        end else begin
            $display("[OK  ] 写事务数 = 257");
        end

        for (cmp_i = 0; cmp_i < 257 && cmp_i < wr_count; cmp_i = cmp_i + 1) begin
            if (wr_addr_log[cmp_i] != u_dut.config_rom[cmp_i][23:8] ||
                wr_data_log[cmp_i] != u_dut.config_rom[cmp_i][7:0]) begin
                $display("[FAIL] 写入 #%0d 不匹配: 收到 %04X:%02X, 期望 %04X:%02X",
                         cmp_i,
                         wr_addr_log[cmp_i], wr_data_log[cmp_i],
                         u_dut.config_rom[cmp_i][23:8],
                         u_dut.config_rom[cmp_i][7:0]);
                all_ok = 1'b0;
            end
        end
        if (wr_count > 257)
            $display("[FAIL] 多余的写入 (计数 > 257), 可能是 STOP 未被从机识别导致字节错位");

        if (bad_dev_addr) begin
            $display("[FAIL] 检测到非 0x78 的设备地址字节");
            all_ok = 1'b0;
        end

        // 打印前 8 个写入供人工核对 (对照配置表开头)
        $display("[INFO] 前 8 个写入 (reg:data):");
        for (cmp_i = 0; cmp_i < 8 && cmp_i < wr_count; cmp_i = cmp_i + 1) begin
            $display("       #%0d: %04h : %02h   (期望 %04h : %02h)",
                     cmp_i,
                     wr_addr_log[cmp_i], wr_data_log[cmp_i],
                     u_dut.config_rom[cmp_i][23:8],
                     u_dut.config_rom[cmp_i][7:0]);
        end
        $display("       ... (共 %0d 个)", wr_count);

        $display("-------------------------------------------------------");
        if (all_ok)
            $display("[PASS] 257 个寄存器全部按序写入, 与配置表一致, config_done 拉高");
        else
            $display("[FAIL] 存在不一致, 见上方信息");
        $display("-------------------------------------------------------");
        $finish;
    end

    // 超时看门狗 (160ms, 单位 ns; 防止轮询挂死)
    initial begin
        #160000000;
        $display("[FAIL] 看门狗超时 160ms, 仿真强制结束");
        $finish;
    end

endmodule
