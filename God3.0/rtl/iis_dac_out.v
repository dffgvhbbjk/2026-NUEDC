//======================================================================
// Module: iis_dac_out
// Function: I2S实时回放发送器
//   将 PCM1808 右声道 24bit 采样按 PCM5102 的 I2S 格式发送到 DIN。
//
// 时序要求:
//   - I2S 标准格式: LRCK 边沿后延迟 1 个 BCK 输出 MSB
//   - PCM5102 在 BCK 上升沿采样 DIN, 因此 DIN 在下降沿更新
//   - 左右 slot 发送同一个右声道采样, 实现单声道回放
//======================================================================

module iis_dac_out (
    input  wire        ad_bck,        // 约 6.144MHz 位时钟
    input  wire        rst_n,         // BCK域复位, 低有效
    input  wire        lrck,          // 约 96kHz 帧时钟
    input  wire [23:0] sample_in,     // 右声道 24bit 采样
    input  wire        sample_valid,  // 采样字有效脉冲
    output reg         dac_data       // I2S DIN → PCM5102
);

// 采样数据只在有效脉冲锁存，避免在发送过程中读取正在变化的接收寄存器。
reg [23:0] sample_reg;
always @(posedge ad_bck or negedge rst_n) begin
    if (!rst_n)
        sample_reg <= 24'd0;
    else if (sample_valid)
        sample_reg <= sample_in;
end

// LRCK任一边沿装载采样字；24bit发送结束后自动补零填满32bit slot。
reg        lrck_d1;
reg [23:0] shift;
reg        din_int;

always @(posedge ad_bck or negedge rst_n) begin
    if (!rst_n) begin
        lrck_d1 <= 1'b0;
        shift   <= 24'd0;
        din_int <= 1'b0;
    end else begin
        lrck_d1 <= lrck;
        if (lrck != lrck_d1) begin
            // I2S的1-BCK延迟: 当前拍先准备MSB, 下一个上升沿采样MSB。
            din_int <= sample_reg[23];
            shift   <= {sample_reg[22:0], 1'b0};
        end else begin
            din_int <= shift[23];
            shift   <= {shift[22:0], 1'b0};
        end
    end
end

// DIN只在下降沿改变，保证PCM5102在上升沿采样时有稳定建立时间。
always @(negedge ad_bck or negedge rst_n) begin
    if (!rst_n)
        dac_data <= 1'b0;
    else
        dac_data <= din_int;
end

endmodule
