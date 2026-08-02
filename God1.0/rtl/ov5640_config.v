//======================================================================
// Module: ov5640_config
// Function: SCCB 配置 OV5640 寄存器
//   2026-08-01 重构: 手写 SCCB 状态机存在两个缺陷, 导致寄存器配置
//   未真正写入摄像头 (无论换哪张配置表都出不了图):
//     1) 不检查 ACK — 摄像头不应答也照写, config_done 照样拉高
//     2) STOP 时序错误 — SCL 与 SDA 同拍跳变, 从机可能把 STOP 误判为
//        数据位, 导致后续寄存器写入错位/串行坏掉
//   本版本改用野火官方 52_ov5640_vga_640x480 例程验证过的 i2c_ctrl.v:
//     - 250kHz I2C 主控制器 (纯逻辑, EP4CE6/EP4CE10 通用)
//     - 带 ACK 检查: 从机不应答 (ack==1) 则状态机停住, cfg_end 不产生,
//       config_done 拉不高 → 可直接诊断 SCCB 是否真的写进去了
//     - 标准 START/STOP 时序
//   配置表保持当前 257 寄存器:
//     PLL 0x3035=0x21 / 0x3036=0x69 (PCLK≈41MHz, 在本板 SDC 44MHz
//     约束内; 官方 0x11/0x46 会到 ~55MHz 超出约束, 故不回退)
//
// 配置时序:
//   rst_n (来自 poweron_done, 上电 29ms 后) → 等 20000 个 i2c_clk
//   (~20ms, 官方 CNT_WAIT_MAX) → 逐个写寄存器 → config_done
//
// Clock: clk = 50MHz, i2c_clk = 1MHz (i2c_ctrl 内部 50M/50 分频,
//        SCL = 250kHz)
//==================================================================
module ov5640_config (
    input  wire        clk,           // 50MHz
    input  wire        rst_n,
    output wire        sclk,          // SCCB 时钟
    inout  wire        sdat,          // SCCB 数据 (开漏)
    output reg         config_done,   // 配置完成
    output wire [8:0]  cfg_progress   // 已写入的寄存器数 (诊断用, 停滞时看写到哪)
);

    localparam REG_NUM = 257;

    reg [23:0] config_rom [0:REG_NUM-1];

    // 配置表: 官方 251 项 + 手动曝光初值 6 项 (保持当前版本不变)
    initial begin
        config_rom[0] = {16'h3103, 8'h11};
        config_rom[1] = {16'h3008, 8'h82};
        config_rom[2] = {16'h3008, 8'h42};
        config_rom[3] = {16'h3103, 8'h03};
        config_rom[4] = {16'h3017, 8'hFF};
        config_rom[5] = {16'h3018, 8'hFF};
        config_rom[6] = {16'h3034, 8'h1A};
        config_rom[7] = {16'h3037, 8'h13};
        config_rom[8] = {16'h3108, 8'h01};
        config_rom[9] = {16'h3630, 8'h36};
        config_rom[10] = {16'h3631, 8'h0E};
        config_rom[11] = {16'h3632, 8'hE2};
        config_rom[12] = {16'h3633, 8'h12};
        config_rom[13] = {16'h3621, 8'hE0};
        config_rom[14] = {16'h3704, 8'hA0};
        config_rom[15] = {16'h3703, 8'h5A};
        config_rom[16] = {16'h3715, 8'h78};
        config_rom[17] = {16'h3717, 8'h01};
        config_rom[18] = {16'h370B, 8'h60};
        config_rom[19] = {16'h3705, 8'h1A};
        config_rom[20] = {16'h3905, 8'h02};
        config_rom[21] = {16'h3906, 8'h10};
        config_rom[22] = {16'h3901, 8'h0A};
        config_rom[23] = {16'h3731, 8'h12};
        config_rom[24] = {16'h3600, 8'h08};
        config_rom[25] = {16'h3601, 8'h33};
        config_rom[26] = {16'h302D, 8'h60};
        config_rom[27] = {16'h3620, 8'h52};
        config_rom[28] = {16'h371B, 8'h20};
        config_rom[29] = {16'h471C, 8'h50};
        config_rom[30] = {16'h3A13, 8'h43};
        config_rom[31] = {16'h3A18, 8'h00};
        config_rom[32] = {16'h3A19, 8'hF8};
        config_rom[33] = {16'h3635, 8'h13};
        config_rom[34] = {16'h3636, 8'h03};
        config_rom[35] = {16'h3634, 8'h40};
        config_rom[36] = {16'h3622, 8'h01};
        config_rom[37] = {16'h3C01, 8'h34};
        config_rom[38] = {16'h3C04, 8'h28};
        config_rom[39] = {16'h3C05, 8'h98};
        config_rom[40] = {16'h3C06, 8'h00};
        config_rom[41] = {16'h3C07, 8'h08};
        config_rom[42] = {16'h3C08, 8'h00};
        config_rom[43] = {16'h3C09, 8'h1C};
        config_rom[44] = {16'h3C0A, 8'h9C};
        config_rom[45] = {16'h3C0B, 8'h40};
        config_rom[46] = {16'h3810, 8'h00};
        config_rom[47] = {16'h3811, 8'h10};
        config_rom[48] = {16'h3812, 8'h00};
        config_rom[49] = {16'h3708, 8'h64};
        config_rom[50] = {16'h4001, 8'h02};
        config_rom[51] = {16'h4005, 8'h1A};
        config_rom[52] = {16'h3000, 8'h00};
        config_rom[53] = {16'h3004, 8'hFF};
        config_rom[54] = {16'h300E, 8'h58};
        config_rom[55] = {16'h302E, 8'h00};
        config_rom[56] = {16'h4300, 8'h61};
        config_rom[57] = {16'h501F, 8'h01};  // RGB565 输出标准搭配 (官方 STM32/IMXRT/FPGA 全部 0x01)
        config_rom[58] = {16'h440E, 8'h00};
        config_rom[59] = {16'h5000, 8'hA7};
        config_rom[60] = {16'h3A0F, 8'h30};
        config_rom[61] = {16'h3A10, 8'h28};
        config_rom[62] = {16'h3A1B, 8'h30};
        config_rom[63] = {16'h3A1E, 8'h26};
        config_rom[64] = {16'h3A11, 8'h60};
        config_rom[65] = {16'h3A1F, 8'h14};
        config_rom[66] = {16'h5800, 8'h23};
        config_rom[67] = {16'h5801, 8'h14};
        config_rom[68] = {16'h5802, 8'h0F};
        config_rom[69] = {16'h5803, 8'h0F};
        config_rom[70] = {16'h5804, 8'h12};
        config_rom[71] = {16'h5805, 8'h26};
        config_rom[72] = {16'h5806, 8'h0C};
        config_rom[73] = {16'h5807, 8'h08};
        config_rom[74] = {16'h5808, 8'h05};
        config_rom[75] = {16'h5809, 8'h05};
        config_rom[76] = {16'h580A, 8'h08};
        config_rom[77] = {16'h580B, 8'h0D};
        config_rom[78] = {16'h580C, 8'h08};
        config_rom[79] = {16'h580D, 8'h03};
        config_rom[80] = {16'h580E, 8'h00};
        config_rom[81] = {16'h580F, 8'h00};
        config_rom[82] = {16'h5810, 8'h03};
        config_rom[83] = {16'h5811, 8'h09};
        config_rom[84] = {16'h5812, 8'h07};
        config_rom[85] = {16'h5813, 8'h03};
        config_rom[86] = {16'h5814, 8'h00};
        config_rom[87] = {16'h5815, 8'h01};
        config_rom[88] = {16'h5816, 8'h03};
        config_rom[89] = {16'h5817, 8'h08};
        config_rom[90] = {16'h5818, 8'h0D};
        config_rom[91] = {16'h5819, 8'h08};
        config_rom[92] = {16'h581A, 8'h05};
        config_rom[93] = {16'h581B, 8'h06};
        config_rom[94] = {16'h581C, 8'h08};
        config_rom[95] = {16'h581D, 8'h0E};
        config_rom[96] = {16'h581E, 8'h29};
        config_rom[97] = {16'h581F, 8'h17};
        config_rom[98] = {16'h5820, 8'h11};
        config_rom[99] = {16'h5821, 8'h11};
        config_rom[100] = {16'h5822, 8'h15};
        config_rom[101] = {16'h5823, 8'h28};
        config_rom[102] = {16'h5824, 8'h46};
        config_rom[103] = {16'h5825, 8'h26};
        config_rom[104] = {16'h5826, 8'h08};
        config_rom[105] = {16'h5827, 8'h26};
        config_rom[106] = {16'h5828, 8'h64};
        config_rom[107] = {16'h5829, 8'h26};
        config_rom[108] = {16'h582A, 8'h24};
        config_rom[109] = {16'h582B, 8'h22};
        config_rom[110] = {16'h582C, 8'h24};
        config_rom[111] = {16'h582D, 8'h24};
        config_rom[112] = {16'h582E, 8'h06};
        config_rom[113] = {16'h582F, 8'h22};
        config_rom[114] = {16'h5830, 8'h40};
        config_rom[115] = {16'h5831, 8'h42};
        config_rom[116] = {16'h5832, 8'h24};
        config_rom[117] = {16'h5833, 8'h26};
        config_rom[118] = {16'h5834, 8'h24};
        config_rom[119] = {16'h5835, 8'h22};
        config_rom[120] = {16'h5836, 8'h22};
        config_rom[121] = {16'h5837, 8'h26};
        config_rom[122] = {16'h5838, 8'h44};
        config_rom[123] = {16'h5839, 8'h24};
        config_rom[124] = {16'h583A, 8'h26};
        config_rom[125] = {16'h583B, 8'h28};
        config_rom[126] = {16'h583C, 8'h42};
        config_rom[127] = {16'h583D, 8'hCE};
        config_rom[128] = {16'h5180, 8'hFF};
        config_rom[129] = {16'h5181, 8'hF2};
        config_rom[130] = {16'h5182, 8'h00};
        config_rom[131] = {16'h5183, 8'h14};
        config_rom[132] = {16'h5184, 8'h25};
        config_rom[133] = {16'h5185, 8'h24};
        config_rom[134] = {16'h5186, 8'h09};
        config_rom[135] = {16'h5187, 8'h09};
        config_rom[136] = {16'h5188, 8'h09};
        config_rom[137] = {16'h5189, 8'h75};
        config_rom[138] = {16'h518A, 8'h54};
        config_rom[139] = {16'h518B, 8'hE0};
        config_rom[140] = {16'h518C, 8'hB2};
        config_rom[141] = {16'h518D, 8'h42};
        config_rom[142] = {16'h518E, 8'h3D};
        config_rom[143] = {16'h518F, 8'h56};
        config_rom[144] = {16'h5190, 8'h46};
        config_rom[145] = {16'h5191, 8'hF8};
        config_rom[146] = {16'h5192, 8'h04};
        config_rom[147] = {16'h5193, 8'h70};
        config_rom[148] = {16'h5194, 8'hF0};
        config_rom[149] = {16'h5195, 8'hF0};
        config_rom[150] = {16'h5196, 8'h03};
        config_rom[151] = {16'h5197, 8'h01};
        config_rom[152] = {16'h5198, 8'h04};
        config_rom[153] = {16'h5199, 8'h12};
        config_rom[154] = {16'h519A, 8'h04};
        config_rom[155] = {16'h519B, 8'h00};
        config_rom[156] = {16'h519C, 8'h06};
        config_rom[157] = {16'h519D, 8'h82};
        config_rom[158] = {16'h519E, 8'h38};
        config_rom[159] = {16'h5480, 8'h01};
        config_rom[160] = {16'h5481, 8'h08};
        config_rom[161] = {16'h5482, 8'h14};
        config_rom[162] = {16'h5483, 8'h28};
        config_rom[163] = {16'h5484, 8'h51};
        config_rom[164] = {16'h5485, 8'h65};
        config_rom[165] = {16'h5486, 8'h71};
        config_rom[166] = {16'h5487, 8'h7D};
        config_rom[167] = {16'h5488, 8'h87};
        config_rom[168] = {16'h5489, 8'h91};
        config_rom[169] = {16'h548A, 8'h9A};
        config_rom[170] = {16'h548B, 8'hAA};
        config_rom[171] = {16'h548C, 8'hB8};
        config_rom[172] = {16'h548D, 8'hCD};
        config_rom[173] = {16'h548E, 8'hDD};
        config_rom[174] = {16'h548F, 8'hEA};
        config_rom[175] = {16'h5490, 8'h1D};
        config_rom[176] = {16'h5381, 8'h1E};
        config_rom[177] = {16'h5382, 8'h5B};
        config_rom[178] = {16'h5383, 8'h08};
        config_rom[179] = {16'h5384, 8'h0A};
        config_rom[180] = {16'h5385, 8'h7E};
        config_rom[181] = {16'h5386, 8'h88};
        config_rom[182] = {16'h5387, 8'h7C};
        config_rom[183] = {16'h5388, 8'h6C};
        config_rom[184] = {16'h5389, 8'h10};
        config_rom[185] = {16'h538A, 8'h01};
        config_rom[186] = {16'h538B, 8'h98};
        config_rom[187] = {16'h5580, 8'h06};
        config_rom[188] = {16'h5583, 8'h40};
        config_rom[189] = {16'h5584, 8'h10};
        config_rom[190] = {16'h5589, 8'h10};
        config_rom[191] = {16'h558A, 8'h00};
        config_rom[192] = {16'h558B, 8'hF8};
        config_rom[193] = {16'h501D, 8'h40};
        config_rom[194] = {16'h5300, 8'h08};
        config_rom[195] = {16'h5301, 8'h30};
        config_rom[196] = {16'h5302, 8'h10};
        config_rom[197] = {16'h5303, 8'h00};
        config_rom[198] = {16'h5304, 8'h08};
        config_rom[199] = {16'h5305, 8'h30};
        config_rom[200] = {16'h5306, 8'h08};
        config_rom[201] = {16'h5307, 8'h16};
        config_rom[202] = {16'h5309, 8'h08};
        config_rom[203] = {16'h530A, 8'h30};
        config_rom[204] = {16'h530B, 8'h04};
        config_rom[205] = {16'h530C, 8'h06};
        config_rom[206] = {16'h5025, 8'h00};
        config_rom[207] = {16'h3008, 8'h02};  // 退出软件掉电, 正常出流
        config_rom[208] = {16'h3035, 8'h41};  // PLL: 野火官方 STM32 15fps 配置 (SDIV0=4)
        config_rom[209] = {16'h3036, 8'h72};  // PLL: 倍频 0x72 → PCLK≈28.5MHz, 640×480 约 15fps
                                             // 关键: 板子采集逻辑按 44MHz 约束 (SDC), 而 30fps+ 需要
                                             // 52-70MHz PCLK 板子接不住 (href 采样不到/图像全零)。
                                             // 官方 15fps (28.5MHz) 在板子承受范围内, 已实测能出图。
                                             // 对比: 30fps=0x21/0x72(57MHz), 官方FPGA=0x11/0x46(70MHz)
        config_rom[210] = {16'h3C07, 8'h08};
        config_rom[211] = {16'h3820, 8'h47};
        config_rom[212] = {16'h3821, 8'h00};
        config_rom[213] = {16'h3814, 8'h31};
        config_rom[214] = {16'h3815, 8'h31};
        config_rom[215] = {16'h3800, 8'h00};
        config_rom[216] = {16'h3801, 8'h00};
        config_rom[217] = {16'h3802, 8'h00};
        config_rom[218] = {16'h3803, 8'h04};
        config_rom[219] = {16'h3804, 8'h0A};
        config_rom[220] = {16'h3805, 8'h3F};
        config_rom[221] = {16'h3806, 8'h07};
        config_rom[222] = {16'h3807, 8'h9B};
        config_rom[223] = {16'h3808, 8'h02};
        config_rom[224] = {16'h3809, 8'h80};
        config_rom[225] = {16'h380A, 8'h01};
        config_rom[226] = {16'h380B, 8'hE0};
        config_rom[227] = {16'h380C, 8'h07};
        config_rom[228] = {16'h380D, 8'h68};
        config_rom[229] = {16'h380E, 8'h03};
        config_rom[230] = {16'h380F, 8'hD8};
        config_rom[231] = {16'h3813, 8'h06};
        config_rom[232] = {16'h3618, 8'h00};
        config_rom[233] = {16'h3612, 8'h29};
        config_rom[234] = {16'h3709, 8'h52};
        config_rom[235] = {16'h370C, 8'h03};
        config_rom[236] = {16'h3A02, 8'h17};
        config_rom[237] = {16'h3A03, 8'h10};
        config_rom[238] = {16'h3A14, 8'h17};
        config_rom[239] = {16'h3A15, 8'h10};
        config_rom[240] = {16'h4004, 8'h02};
        config_rom[241] = {16'h3002, 8'h1C};
        config_rom[242] = {16'h3006, 8'hC3};
        config_rom[243] = {16'h4713, 8'h03};
        config_rom[244] = {16'h4407, 8'h04};
        config_rom[245] = {16'h460B, 8'h35};
        config_rom[246] = {16'h460C, 8'h22};
        config_rom[247] = {16'h4837, 8'h22};
        config_rom[248] = {16'h3824, 8'h02};
        config_rom[249] = {16'h5001, 8'hA3};
        config_rom[250] = {16'h3503, 8'h00};  // AEC/AGC 自动
        // 手动曝光/增益初值 (写入后 0x3503=0x00 重新开启 AEC, 从初值起调)
        config_rom[251] = {16'h3500, 8'h00};
        config_rom[252] = {16'h3501, 8'h01};
        config_rom[253] = {16'h3502, 8'h90};
        config_rom[254] = {16'h350A, 8'h00};
        config_rom[255] = {16'h350B, 8'h02};
        config_rom[256] = {16'h3503, 8'h00};
        // 注: 曾把唤醒 0x02 挪到末尾 (掉电期间写分辨率), 实测摄像头进掉电不输出 (0帧),
        //     已撤销。分辨率改为采集适配 (1280×960), 不再强求配置生效。
    end

    //==================================================================
    // 官方 i2c_ctrl 控制器 (野火 52_ov5640_vga_640x480, SCL=250kHz)
    //   带 ACK 检查 + 标准 START/STOP 时序
    //==================================================================
    reg         cfg_start;       // 触发一次写事务 (1 拍脉冲, i2c_clk 域)
    wire        cfg_end;         // 一次写事务完成 (i2c_ctrl 输出)
    wire        i2c_clk;         // i2c 驱动时钟 (1MHz)
    wire [23:0] cfg_data;        // {reg_hi, reg_lo, data}

    i2c_ctrl #(
        .DEVICE_ADDR (7'h3C),          // OV5640 SCCB 地址 (写 = 0x78)
        .SYS_CLK_FREQ (26'd50_000_000),
        .SCL_FREQ     (18'd250_000)
    ) u_i2c_ctrl (
        .sys_clk   (clk),
        .sys_rst_n (rst_n),
        .wr_en     (1'b1),             // 只写不读
        .rd_en     (1'b0),
        .i2c_start (cfg_start),
        .addr_num  (1'b1),             // 16-bit 寄存器地址
        .byte_addr (cfg_data[23:8]),
        .wr_data   (cfg_data[7:0]),
        .rd_data   (),
        .i2c_end   (cfg_end),
        .i2c_clk   (i2c_clk),
        .i2c_scl   (sclk),
        .i2c_sda   (sdat)
    );

    //==================================================================
    // 配置顺序器 (移植官方 ov5640_cfg 逻辑, 时钟 = i2c_clk 1MHz)
    //   cnt_wait : 上电稳定等待 20000 个 i2c_clk (~20ms)
    //   reg_num  : 当前正在写的寄存器索引 (0-256)
    //   cfg_start: 触发一次写事务 (1 拍脉冲)
    //   cfg_done : 全部写完 (仅当摄像头对每个寄存器都 ACK 才置位)
    //   注: 官方 ov5640_cfg 在末尾会多触发一次越界写, 这里改为
    //       写完最后一个 (reg_num==REG_NUM-1) 即 cfg_done, 无冗余写。
    //==================================================================
    localparam [14:0] CNT_WAIT_MAX = 15'd20000;

    reg [14:0] cnt_wait;
    reg [8:0]  reg_num;        // 0-256
    reg        cfg_done;

    always @(posedge i2c_clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_wait  <= 15'd0;
            reg_num   <= 9'd0;
            cfg_start <= 1'b0;
            cfg_done  <= 1'b0;
        end else begin
            cfg_start <= 1'b0;   // 默认低 (脉冲)

            // cnt_wait: 只在前 20000 拍计数
            if (cnt_wait < CNT_WAIT_MAX)
                cnt_wait <= cnt_wait + 1'b1;

            // 首个写事务: 等待结束后触发 (reg_num=0)
            if (cnt_wait == (CNT_WAIT_MAX - 1'b1))
                cfg_start <= 1'b1;

            // 每个写事务完成: 最后一个则置 cfg_done, 否则推进到下一个
            if (cfg_end) begin
                if (reg_num == (REG_NUM - 1)) begin
                    cfg_done <= 1'b1;          // 257 个寄存器全部写完
                end else begin
                    reg_num   <= reg_num + 1'b1;
                    cfg_start <= 1'b1;          // 触发下一个写事务
                end
            end
        end
    end

    // cfg_data: 当前要写的寄存器 (cfg_done 后输出 0; 越界索引保护)
    wire [8:0]  rom_idx  = (reg_num < REG_NUM) ? reg_num : 9'd0;
    assign cfg_data = (cfg_done) ? 24'b0 : config_rom[rom_idx];

    //==================================================================
    // config_done 同步到 50MHz 域 (2 级同步器, 电平信号)
    //==================================================================
    reg cfg_done_d1, cfg_done_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_done_d1 <= 1'b0;
            cfg_done_d2 <= 1'b0;
            config_done <= 1'b0;
        end else begin
            cfg_done_d1 <= cfg_done;
            cfg_done_d2 <= cfg_done_d1;
            config_done <= cfg_done_d2;
        end
    end

    // 配置进度 (诊断): 已写入的寄存器数, 停滞时能看出写到第几个
    //   cfg_done=0 且 cfg_progress 停在 N → 摄像头在写第 N+1 个寄存器时没 ACK
    assign cfg_progress = reg_num;

endmodule
