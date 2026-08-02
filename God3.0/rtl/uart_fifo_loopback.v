//======================================================================
// Module: uart_fifo_loopback
// Function: FIFO + UART 回环测试
//   - UART RX 收到字节 → 写入 DCFIFO(16bit写/8bit读)
//   - 读端 2 状态 FSM: FIFO有数据+TX空闲 → 读FIFO → 发UART TX
//   - 验证 FIFO + 高速 UART 收/发链路
//======================================================================
module uart_fifo_loopback (
    input  wire        clk,         // sys_clk 50MHz
    input  wire        rst_n,       // 异步复位(低有效)
    input  wire        rxd,         // UART RX → PC
    output wire        txd          // UART TX → PC
);

    //==================================================================
    // UART RX (2Mbps) — 接收 PC 发来的字节
    //==================================================================
    wire [7:0]  rx_data;
    wire        rx_flag;

    uart_rx #(
        .UART_BPS (2000000),
        .CLK_FREQ (50_000_000)
    ) u_rx (
        .sys_clk    (clk),
        .sys_rst_n  (rst_n),
        .rx         (rxd),
        .po_data    (rx_data),
        .po_flag    (rx_flag)
    );

    //==================================================================
    // 写端: 收到 UART 字节后写入 FIFO
    //   FIFO 写口 16bit → 高 8bit 补 0, 低 8bit = rx_data
    //==================================================================
    reg  [15:0] fifo_wrdata;
    reg         fifo_wrreq;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_wrdata <= 16'd0;
            fifo_wrreq  <= 1'b0;
        end else begin
            fifo_wrreq <= 1'b0;
            if (rx_flag) begin
                fifo_wrdata <= {8'd0, rx_data};
                fifo_wrreq  <= 1'b1;
            end
        end
    end

    //==================================================================
    // DCFIFO: 16bit写 → 8bit读, 深2048
    //==================================================================
    wire [7:0]  fifo_q;
    wire [11:0] fifo_rdusedw;
    wire [10:0] fifo_wrusedw;

    uart_fifo u_uart_fifo (
        .aclr      (~rst_n),
        .data      (fifo_wrdata),
        .wrclk     (clk),
        .wrreq     (fifo_wrreq),
        .wrusedw   (fifo_wrusedw),
        .rdclk     (clk),
        .rdreq     (fifo_rdreq),
        .q         (fifo_q),
        .rdusedw   (fifo_rdusedw)
    );

    //==================================================================
    // 读端 FSM: FIFO有数据 + UART空闲 → 读一字节 → 发一字节
    //==================================================================
    localparam IDLE = 1'b0, SEND = 1'b1;

    reg        state;
    reg        fifo_rdreq;
    reg [7:0]  tx_data;
    reg        tx_flag;
    wire       tx_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            fifo_rdreq  <= 1'b0;
            tx_data     <= 8'd0;
            tx_flag     <= 1'b0;
        end else begin
            fifo_rdreq <= 1'b0;
            tx_flag    <= 1'b0;

            case (state)
                IDLE: begin
                    if (fifo_rdusedw > 0 && tx_ready) begin
                        fifo_rdreq <= 1'b1;
                        state      <= SEND;
                    end
                end

                SEND: begin
                    tx_data  <= fifo_q;
                    tx_flag  <= 1'b1;
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

    //==================================================================
    // UART TX (2Mbps)
    //==================================================================
    uart_tx #(
        .UART_BPS (2000000),
        .CLK_FREQ (50_000_000)
    ) u_tx (
        .sys_clk    (clk),
        .sys_rst_n  (rst_n),
        .pi_data    (tx_data),
        .pi_flag    (tx_flag),
        .ready      (tx_ready),
        .tx         (txd)
    );

endmodule
