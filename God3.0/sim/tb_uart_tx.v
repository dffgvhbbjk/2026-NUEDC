`timescale 1ns/1ps

module tb_uart_tx;
    localparam CLK_FREQ = 40_000_000;
    localparam UART_BPS = 115_200;
    localparam BIT_CLKS = CLK_FREQ / UART_BPS;

    reg        clk;
    reg        rst_n;
    reg  [7:0] pi_data;
    reg        pi_flag;
    wire       ready;
    wire       tx;

    integer errors;
    integer i;

    uart_tx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (CLK_FREQ)
    ) dut (
        .sys_clk   (clk),
        .sys_rst_n (rst_n),
        .pi_data   (pi_data),
        .pi_flag   (pi_flag),
        .ready     (ready),
        .tx        (tx)
    );

    initial clk = 1'b0;
    always #12.5 clk = ~clk;

    task send_byte;
        input [7:0] value;
        begin
            wait (ready);
            @(posedge clk);
            pi_data <= value;
            pi_flag <= 1'b1;
            @(posedge clk);
            pi_flag <= 1'b0;
            wait (!ready);
            wait (ready);
        end
    endtask

    task check_byte;
        input [7:0] expected;
        begin
            wait (tx == 1'b0);
            repeat (BIT_CLKS / 2) @(posedge clk);
            if (tx !== 1'b0) begin
                $display("FAIL: start bit");
                errors = errors + 1;
            end

            for (i = 0; i < 8; i = i + 1) begin
                repeat (BIT_CLKS) @(posedge clk);
                if (tx !== expected[i]) begin
                    $display("FAIL: data bit %0d expected=%0b got=%0b",
                             i, expected[i], tx);
                    errors = errors + 1;
                end
            end

            repeat (BIT_CLKS) @(posedge clk);
            if (tx !== 1'b1) begin
                $display("FAIL: stop bit is not high");
                errors = errors + 1;
            end
            if (ready !== 1'b0) begin
                $display("FAIL: ready rose before stop-bit completion");
                errors = errors + 1;
            end

            repeat ((BIT_CLKS + 1) / 2) @(posedge clk);
        end
    endtask

    initial begin
        rst_n   = 1'b0;
        pi_data = 8'd0;
        pi_flag = 1'b0;
        errors  = 0;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);

        fork
            begin
                send_byte(8'h55);
                send_byte(8'hA3);
            end
            begin
                check_byte(8'h55);
                check_byte(8'hA3);
            end
        join

        wait (ready);
        if (errors == 0)
            $display("PASS: UART 115200 8N1, two continuous bytes");
        else
            $display("FAIL: %0d UART errors", errors);
        $finish;
    end
endmodule
