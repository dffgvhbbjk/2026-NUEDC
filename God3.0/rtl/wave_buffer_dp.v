// ============================================================================
// Module: wave_buffer_dp
// Description: 16 位 × 1024 双口波形 RAM (文档 §12 第六阶段)
//
//   - 推荐方案 1024×16 (§12.2): 支持 512 点采集 + 乒乓 bank + 预触发余量
//   - 写口: wave_capture (采集链路), 读口: vga_waveform_precision (显示)
//   - RAM 读写块不带异步复位, 保证 Cyclone IV 推断 M9K (§12.3)
//   - 保存基线扣除后的有符号高 16 位 corrected[23:8] (§12.1)
//
// 时钟域: 读写同为 vga_clk 40MHz (简单双口)
// ============================================================================
module wave_buffer_dp (
    input  wire               clk,        // vga_clk 40MHz

    // 写口 (wave_capture)
    input  wire               wr_en,      // 写使能
    input  wire [9:0]         wr_addr,    // 写地址
    input  wire signed [15:0] wr_data,    // 写数据 (有符号 16 位)

    // 读口 (显示, 1 拍延迟)
    input  wire [9:0]         rd_addr,    // 读地址
    output reg  signed [15:0] rd_data     // 读数据 (地址后 1 拍有效)
);

    (* ramstyle = "M9K" *) reg signed [15:0] ram [0:1023];

    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1)
            ram[i] = 16'sd0;
    end

    always @(posedge clk) begin
        if (wr_en)
            ram[wr_addr] <= wr_data;
    end

    always @(posedge clk) begin
        rd_data <= ram[rd_addr];
    end

endmodule
