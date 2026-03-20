module mac_array #(
    parameter int unsigned P_ICH = 4,
    parameter int unsigned A_BIT = 8,
    parameter int unsigned W_BIT = 8,
    parameter int unsigned B_BIT = 32
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    en,
    input  logic                    dat_vld,
    input  logic                    clr,
    input  logic        [A_BIT-1:0] x_vec  [P_ICH],
    input  logic signed [W_BIT-1:0] w_vec  [P_ICH],
    output logic signed [B_BIT-1:0] acc
);

    logic signed [B_BIT-1:0] mac_cascade[P_ICH+1];

    assign mac_cascade[0] = '0;

    generate
        for (genvar i = 0; i < P_ICH - 1; i++) begin : gen_mac
            logic signed [B_BIT-1:0] prod;
            logic signed [B_BIT-1:0] acc_r;

            assign prod = $signed({x_vec[i]}) * w_vec[i];

            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    acc_r <= '0;
                end else if (en) begin
                    acc_r <= prod + mac_cascade[i];
                end
            end
            assign mac_cascade[i+1] = acc_r;
        end
    endgenerate

    logic signed [B_BIT-1:0] tail_prod;
    logic signed [B_BIT-1:0] tail_acc_r;

    assign tail_prod = $signed({x_vec[P_ICH-1]}) * w_vec[P_ICH-1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tail_acc_r <= '0;
        end else if (en) begin
            case ({clr, dat_vld})
                2'b00: tail_acc_r <= tail_acc_r;
                2'b01: tail_acc_r <= tail_acc_r + mac_cascade[P_ICH-1] + tail_prod;
                2'b10: tail_acc_r <= '0;
                2'b11: tail_acc_r <= mac_cascade[P_ICH-1] + tail_prod;
            endcase
        end
    end
    assign mac_cascade[P_ICH] = tail_acc_r;

    assign acc = mac_cascade[P_ICH];

endmodule
