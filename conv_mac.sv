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
    input  logic                   clr,                // 第一个元素位置，清零（流水线为什么要清零？）
    input  logic        [A_BIT-1:0] x_vec  [P_ICH],     // P_ICH个元素一起输入，一组K*K个
    input  logic signed [W_BIT-1:0] w_vec  [P_ICH],
    output logic signed [B_BIT-1:0] acc
);

    logic signed [B_BIT-1:0] mac_cascade[P_ICH+1];  // P_ICH+2个寄存器，第P_ICH+2没用

    assign mac_cascade[0] = '0;

    generate
        for (genvar i = 0; i < P_ICH - 1; i++) begin : gen_mac  
            logic signed [B_BIT-1:0] prod;
            logic signed [B_BIT-1:0] acc_r;

            assign prod = x_vec[i] * w_vec[i];         
            
            always_ff @(posedge clk or negedge rst_n) begin     
                if (!rst_n) begin
                    acc_r <= '0;
                end else if (en) begin       // 斜化后只有第一个数和vld对齐，应该vld不控制乘积与cascade累加，vld再延时PICH个周期到尾项，invalid的累加和舍弃
                    acc_r <= prod + mac_cascade[i];     
                end
            end
            assign mac_cascade[i+1] = acc_r;  
        end
    endgenerate

    logic signed [B_BIT-1:0] tail_prod;
    logic signed [B_BIT-1:0] tail_acc_r;

    assign tail_prod = x_vec[P_ICH-1] * w_vec[P_ICH-1];       

// 清零且数据无效，清零指舍弃之前的accr，数据无效指cascade内数据舍弃，也就是直接把accr赋值0
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tail_acc_r <= '0;
        end else if (en) begin
            case ({clr, dat_vld})
                2'b00: tail_acc_r <= tail_acc_r;        
                2'b01: tail_acc_r <= tail_acc_r + mac_cascade[P_ICH-1] + tail_prod;     // 加上 tail_acc_r 的历史累加值，因为每PICH个周期计算出的是一个位置的和，还有一级累加没做，tail_acc_r就是那个寄存器
                2'b10: tail_acc_r <= '0;        // 新的输出位置，且没有新的输入，等待有效的x*w
                2'b11: tail_acc_r <= mac_cascade[P_ICH-1] + tail_prod;      // 新的输出位置，丢掉之前的累加和
            endcase
        end
    end
    assign mac_cascade[P_ICH] = tail_acc_r;  

    assign acc = mac_cascade[P_ICH];  

endmodule