// 修改：body和tail独立成模块，输入数据斜化放进mac内部
module conv_mac_body #(
    parameter int unsigned A_BIT = 8,
    parameter int unsigned W_BIT = 8,
    parameter int unsigned B_BIT = 32
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   en,
    input  logic        [A_BIT-1:0] x,
    input  logic signed [W_BIT-1:0] w,
    input  logic signed [B_BIT-1:0] acc_in,
    output logic signed [B_BIT-1:0] acc_out
);
 
    logic signed [B_BIT-1:0] prod;
    logic signed [B_BIT-1:0] acc_r;

    assign prod = x * w;         
            
    always_ff @(posedge clk or negedge rst_n) begin     
        if (!rst_n) begin
            acc_r <= '0;
        end else if (en) begin       // 斜化后只有第一个数和vld对齐，应该vld不控制乘积与cascade累加，vld再延时PICH个周期到尾项，invalid的累加和舍弃
            acc_r <= prod + acc_in;     
        end
    end
    assign acc_out = acc_r;  
endmodule

module conv_mac_tail #(
    parameter int unsigned A_BIT = 8,
    parameter int unsigned W_BIT = 8,
    parameter int unsigned B_BIT = 32
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   en,
    input  logic                   dat_vld,
    input  logic                   clr,
    input  logic        [A_BIT-1:0] x_tail,
    input  logic signed [W_BIT-1:0] w_tail,
    input  logic signed [B_BIT-1:0] acc_in,
    output logic signed [B_BIT-1:0] acc_out
);

    logic signed [B_BIT-1:0] tail_prod;
    logic signed [B_BIT-1:0] tail_acc_r;    // 累加寄存器

    assign tail_prod = x_tail * w_tail;       

    // 清零且数据无效，清零指舍弃之前的accr，数据无效指cascade内数据舍弃，也就是直接把accr赋值0
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tail_acc_r <= '0;
        end else if (en) begin
            case ({clr, dat_vld})
                2'b00: tail_acc_r <= tail_acc_r;        
                2'b01: tail_acc_r <= tail_acc_r + acc_in + tail_prod;   // 累加和 + 前I-1级的和 + 尾项乘积
                2'b10: tail_acc_r <= '0;                                // 清零丢弃累加和，无效没有数据相加
                2'b11: tail_acc_r <= acc_in + tail_prod;                // 清零丢弃累加和，有效开始累加前I-1级的和 + 尾项乘积
            endcase
        end
    end
    assign acc_out = tail_acc_r;  
endmodule

module conv_mac_array #(
    parameter int unsigned P_ICH = 4,
    parameter int unsigned A_BIT = 8,
    parameter int unsigned W_BIT = 8,
    parameter int unsigned B_BIT = 32
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   en,
    input  logic                   dat_vld,            // mac的输入有效
    input  logic                   clr,                // 第一个元素位置，清零
    input  logic        [A_BIT-1:0] x_vec  [P_ICH],
    input  logic signed [W_BIT-1:0] w_vec  [P_ICH],
    output logic signed [B_BIT-1:0] acc
);

    logic [A_BIT-1:0] x_vec_delayed [P_ICH];
    logic [W_BIT-1:0] w_vec_delayed [P_ICH];

    generate
        for (genvar i = 0; i < P_ICH; i++) begin : gen_skew
            delayline #(
                .WIDTH(A_BIT), 
                .DEPTH(i)
            ) u_dx (
                .clk(clk), 
                .rst_n(rst_n), 
                .en(en),
                .data_in(x_vec[i]), 
                .data_out(x_vec_delayed[i])
            );
            
            delayline #(
                .WIDTH(W_BIT), 
                .DEPTH(i)
            ) u_dw (
                .clk(clk), 
                .rst_n(rst_n), 
                .en(en),
                .data_in(w_vec[i]), 
                .data_out(w_vec_delayed[i])
            ); 
        end
    endgenerate

    logic signed [B_BIT-1:0] mac_cascade[P_ICH+1];
    assign mac_cascade[0] = '0;

    generate
        for (genvar i = 0; i < P_ICH - 1; i++) begin : gen_mac  
            conv_mac_body #(
                .A_BIT(A_BIT),
                .W_BIT(W_BIT),
                .B_BIT(B_BIT)
            ) u_mac_body (
                .clk    (clk),
                .rst_n  (rst_n),
                .en     (en),
                .x      (x_vec_delayed[i]),
                .w      (w_vec_delayed[i]),
                .acc_in (mac_cascade[i]),
                .acc_out(mac_cascade[i+1])
            );
        end
    endgenerate

    conv_mac_tail #(
        .A_BIT(A_BIT),
        .W_BIT(W_BIT),
        .B_BIT(B_BIT)
    ) u_mac_tail (
        .clk    (clk),
        .rst_n  (rst_n),
        .en     (en),
        .dat_vld(dat_vld),
        .clr    (clr),
        .x_tail (x_vec_delayed[P_ICH-1]),
        .w_tail (w_vec_delayed[P_ICH-1]),
        .acc_in (mac_cascade[P_ICH-1]),
        .acc_out(mac_cascade[P_ICH])
    );

    assign acc = mac_cascade[P_ICH];  

endmodule
