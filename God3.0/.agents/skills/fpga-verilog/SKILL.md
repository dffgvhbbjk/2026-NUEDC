---
name: fpga-verilog
description: |
  FPGA Verilog 编码规范和最佳实践指南。当用户编写、审查或调试 Verilog 代码时自动触发。
  包括时钟域处理、状态机设计、资源优化、代码风格等主题。
  适用于 Cyclone IV 系列 FPGA 开发。
---

# FPGA Verilog 编码技能

本技能提供 Altera Cyclone IV FPGA 的 Verilog HDL 编码规范和最佳实践。

---

## 一、命名规范

### 1.1 模块命名

| 类型 | 规则 | 示例 |
|------|------|------|
| 模块名 | 小写下划线 | `vga_ctrl_simple`, `iis_slave_rx` |
| 参数/宏 | 大写下划线 | `DATA_WIDTH`, `FIFO_DEPTH` |
| 局部参数 | 大写下划线 | `IDLE`, `START`, `DONE` |

### 1.2 信号命名后缀

| 后缀 | 含义 | 示例 |
|------|------|------|
| `_n` | 低电平有效 | `rst_n`, `cs_n`, `wr_n` |
| `_vld` | 数据有效脉冲 | `ad_data_vld`, `l_valid` |
| `_en` | 使能信号 | `rd_en`, `wr_en` |
| `_cnt` | 计数器 | `h_cnt`, `v_cnt` |
| `_reg` | 寄存器输出 | `state_reg`, `data_reg` |
| `_next` | 下一状态 | `state_next` |
| `_d1`, `_d2` | 同步器延迟 | `signal_d1`, `signal_d2` |

### 1.3 端口顺序

```verilog
module module_name #(
    parameter PARAM1 = DEFAULT1,
    parameter PARAM2 = DEFAULT2
) (
    // 1. 时钟与复位 (全局)
    input  wire        clk,
    input  wire        rst_n,
    
    // 2. 输入数据信号
    input  wire [WIDTH-1:0] din,
    input  wire        din_vld,
    
    // 3. 输出数据信号
    output reg  [WIDTH-1:0] dout,
    output reg         dout_vld,
    
    // 4. 控制信号
    input  wire        enable,
    output wire        busy
);
```

---

## 二、时钟与复位

### 2.1 复位策略

**推荐**: 异步复位、同步释放

```verilog
// 同步器用于复位释放
reg rst_n_sync;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rst_n_sync <= 1'b0;
    end else begin
        rst_n_sync <= 1'b1;
    end
end

// 使用同步后的复位
always @(posedge clk or negedge rst_n_sync) begin
    if (!rst_n_sync) begin
        // 复位逻辑
    end else begin
        // 正常逻辑
    end
end
```

### 2.2 时钟域处理

#### 单比特信号同步

```verilog
// 两级同步器 (打两拍)
reg signal_d1, signal_d2;

always @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) begin
        signal_d1 <= 1'b0;
        signal_d2 <= 1'b0;
    end else begin
        signal_d1 <= src_signal;
        signal_d2 <= signal_d1;
    end
end

// 边沿检测
wire signal_posedge = signal_d2 & ~signal_d1;
wire signal_negedge = ~signal_d2 & signal_d1;
```

#### 多比特数据同步

```verilog
// 使用 DCFIFO (Altera 原语)
dcfifo dcfifo_inst (
    .data    (wr_data),
    .rdclk   (rd_clk),
    .rdreq   (rd_req),
    .wrclk   (wr_clk),
    .wrreq   (wr_req),
    .q       (rd_data),
    .rdempty (rd_empty),
    .wrfull  (wr_full)
);
defparam dcfifo_inst.lpm_width = 24;
defparam dcfifo_inst.lpm_numwords = 256;
```

---

## 三、状态机设计

### 3.1 三段式状态机

```verilog
// 状态定义
localparam IDLE  = 3'd0;
localparam START = 3'd1;
localparam RUN   = 3'd2;
localparam DONE  = 3'd3;

reg [2:0] state, next_state;

// 状态寄存器 (时序逻辑)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end

// 状态转移 (组合逻辑)
always @(*) begin
    next_state = state;  // 默认保持
    case (state)
        IDLE:  if (start) next_state = START;
        START: next_state = RUN;
        RUN:   if (done) next_state = DONE;
        DONE:  next_state = IDLE;
        default: next_state = IDLE;
    endcase
end

// 输出逻辑 (时序逻辑)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy <= 1'b0;
        data_out <= 8'd0;
    end else begin
        case (state)
            IDLE: begin
                busy <= 1'b0;
            end
            START: begin
                busy <= 1'b1;
            end
            RUN: begin
                data_out <= data_in;
            end
            DONE: begin
                busy <= 1'b0;
            end
        endcase
    end
end
```

### 3.2 状态编码选择

| 编码方式 | 适用场景 | 特点 |
|----------|----------|------|
| 独热码 (One-Hot) | 高速状态机 | 速度快，资源多 |
| 二进制 | 状态较多 | 资源少，速度慢 |
| 格雷码 | 低功耗设计 | 翻转少，功耗低 |

---

## 四、存储器设计

### 4.1 M9K 推断

```verilog
// 单端口 RAM
module single_port_ram #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 10,
    parameter DEPTH = 1024
) (
    input  wire                    clk,
    input  wire                    we,
    input  wire [ADDR_WIDTH-1:0]   addr,
    input  wire [DATA_WIDTH-1:0]   din,
    output reg  [DATA_WIDTH-1:0]   dout
);
    reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];
    
    always @(posedge clk) begin
        if (we)
            ram[addr] <= din;
        dout <= ram[addr];
    end
endmodule
```

### 4.2 双端口 RAM

```verilog
// 真双端口 RAM
module dual_port_ram #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 10
) (
    input  wire                    clk_a,
    input  wire                    we_a,
    input  wire [ADDR_WIDTH-1:0]   addr_a,
    input  wire [DATA_WIDTH-1:0]   din_a,
    output reg  [DATA_WIDTH-1:0]   dout_a,
    
    input  wire                    clk_b,
    input  wire                    we_b,
    input  wire [ADDR_WIDTH-1:0]   addr_b,
    input  wire [DATA_WIDTH-1:0]   din_b,
    output reg  [DATA_WIDTH-1:0]   dout_b
);
    reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];
    
    // 端口 A
    always @(posedge clk_a) begin
        if (we_a) ram[addr_a] <= din_a;
        dout_a <= ram[addr_a];
    end
    
    // 端口 B
    always @(posedge clk_b) begin
        if (we_b) ram[addr_b] <= din_b;
        dout_b <= ram[addr_b];
    end
endmodule
```

---

## 五、资源优化

### 5.1 乘法器使用

```verilog
// 使用 DSP 乘法器
wire [15:0] a, b;
wire [31:0] product = a * b;  // 自动推断 DSP

// 复用乘法器 (时分复用)
reg [15:0] mult_a, mult_b;
reg [31:0] mult_result;
reg [1:0] mult_state;

always @(posedge clk) begin
    case (mult_state)
        2'd0: begin
            mult_a <= data_a0;
            mult_b <= data_b0;
            mult_state <= 2'd1;
        end
        2'd1: begin
            mult_result <= mult_a * mult_b;
            mult_a <= data_a1;
            mult_b <= data_b1;
            mult_state <= 2'd2;
        end
        2'd2: begin
            result0 <= mult_result;
            mult_result <= mult_a * mult_b;
            mult_state <= 2'd3;
        end
        // ...
    endcase
end
```

### 5.2 移位寄存器推断

```verilog
// 移位寄存器 (MLAB 推断)
reg [DATA_WIDTH-1:0] shift_reg [0:TAPS-1];

always @(posedge clk) begin
    shift_reg[0] <= din;
    for (i = 1; i < TAPS; i = i + 1) begin
        shift_reg[i] <= shift_reg[i-1];
    end
end

assign dout = shift_reg[TAPS-1];
```

---

## 六、常见问题

### 6.1 锁存器推断

**错误示例**:
```verilog
// 会推断锁存器!
always @(*) begin
    if (condition)
        q = data;
    // 没有 else 分支
end
```

**正确做法**:
```verilog
always @(*) begin
    q = default_value;  // 先赋默认值
    if (condition)
        q = data;
end
```

### 6.2 位宽不匹配

```verilog
// 错误: 位宽不匹配
wire [7:0] a;
wire [15:0] b;
assign b = a;  // 高8位为未知值

// 正确: 显式零扩展
assign b = {8'd0, a};

// 或符号扩展
assign b = {{8{a[7]}}, a};
```

### 6.3 时钟门控

```verilog
// 错误: 组合逻辑门控时钟
assign gated_clk = clk & enable;  // 禁止!

// 正确: 使用时钟使能
always @(posedge clk) begin
    if (enable) begin
        // 逻辑
    end
end
```

---

## 七、调试技巧

### 7.1 内建测试点

```verilog
// 输出内部状态用于调试
output wire [7:0] debug_state;
output wire       debug_signal;

assign debug_state = state;
assign debug_signal = internal_flag;
```

### 7.2 仿真友好设计

```verilog
// 使用参数控制仿真行为
parameter SIMULATION = 0;

generate
    if (SIMULATION) begin
        // 仿真专用逻辑 (缩短等待时间等)
        localparam WAIT_CYCLES = 10;
    end else begin
        // 实际硬件逻辑
        localparam WAIT_CYCLES = 1000000;
    end
endgenerate
```

---

## 八、参考资源

- Altera Cyclone IV Device Handbook
- Quartus II Handbook: Design Compilation
- Verilog-2001 Standard (IEEE 1364-2001)
- 项目 AGENTS.md 中的具体模块规范
