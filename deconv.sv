module deconv #(
    parameter int unsigned P_ICH      = 4,
    parameter int unsigned P_OCH      = 4,
    parameter int unsigned N_ICH      = 16,
    parameter int unsigned N_OCH      = 16,
    parameter int unsigned N_IH       = 8,
    parameter int unsigned N_IW       = 8,
    parameter int unsigned K          = 3,
    parameter int unsigned P          = 1,
    parameter int unsigned S          = 2,
    parameter int unsigned O_P        = 0,
    parameter int unsigned Z_NUM      = 0,
    parameter int unsigned A_BIT      = 8,
    parameter int unsigned W_BIT      = 8,
    parameter int unsigned B_BIT      = 32,
    parameter string       W_FILE     = "",
    parameter              W_ROM_TYPE = "block"
) (
    input logic clk,
    input logic rst_n,

    input  logic [P_ICH*A_BIT-1:0] in_data,
    input  logic                   in_valid,
    output logic                   in_ready,

    output logic [P_OCH*B_BIT-1:0] out_data,
    output logic                   out_valid,
    input  logic                   out_ready
);

    localparam int unsigned N_OH = (N_IH - 1) * S + K - 2 * P + O_P;
    localparam int unsigned N_OW = (N_IW - 1) * S + K - 2 * P + O_P;
    localparam int unsigned FOLD_I = N_ICH / P_ICH;
    localparam int unsigned FOLD_O = N_OCH / P_OCH;
    localparam int unsigned KK = K * K;
    localparam int unsigned WEIGHT_DEPTH = FOLD_O * FOLD_I * KK;
    localparam int unsigned LB_DEPTH = N_IH * N_IW * FOLD_I;
    localparam int unsigned LB_AWIDTH = $clog2(LB_DEPTH);

    typedef enum logic [1:0] {
        ST_INIT,
        ST_PROC,
        ST_DRAIN
    } state_t;

    state_t                             state;
    logic        [  $clog2(N_IH+1)-1:0] cntr_init_h;
    logic        [  $clog2(N_IW+1)-1:0] cntr_init_w;
    logic        [$clog2(FOLD_I+1)-1:0] cntr_init_fi;
    logic        [  $clog2(N_OH+1)-1:0] cntr_oh;
    logic        [  $clog2(N_OW+1)-1:0] cntr_ow;
    logic        [$clog2(FOLD_O+1)-1:0] cntr_fo;
    logic        [     $clog2(K+1)-1:0] cntr_kh;
    logic        [     $clog2(K+1)-1:0] cntr_kw;
    logic        [$clog2(FOLD_I+1)-1:0] cntr_fi;
    logic        [ $clog2(P_ICH+2)-1:0] cntr_drain;

    logic [$clog2(WEIGHT_DEPTH)-1:0] weight_addr;
    logic [   P_OCH*P_ICH*W_BIT-1:0] weight_data;
    logic                            line_buffer_we;
    logic [  LB_AWIDTH-1:0]          line_buffer_waddr;
    logic [P_ICH*A_BIT-1:0]          line_buffer_wdata;
    logic                            line_buffer_re;
    logic [  LB_AWIDTH-1:0]          line_buffer_raddr;
    logic [P_ICH*A_BIT-1:0]          line_buffer_rdata;

    logic                            proc_step_en;
    logic                            issue_first;
    logic                            issue_last;
    logic                            issue_valid_pos_d;
    logic                            issue_first_d;
    logic                            issue_last_d;
    logic                            mac_out_vld_d;
    logic [P_OCH*B_BIT-1:0]          out_data_r;

    logic signed [$clog2(N_OH+K)+1:0] h_temp;
    logic signed [$clog2(N_OW+K)+1:0] w_temp;
    logic signed [  $clog2(N_IH)+1:0] ih;
    logic signed [  $clog2(N_IW)+1:0] iw;
    logic                              valid_pos;
    logic        [A_BIT-1:0]           x_vec[P_ICH];
    logic signed [W_BIT-1:0]           w_vec[P_OCH][P_ICH];
    logic        [A_BIT-1:0]           x_vec_delayed[P_ICH];
    logic        [W_BIT-1:0]           w_vec_delayed[P_OCH][P_ICH];
    logic                              mac_dat_vld;
    logic                              mac_clr;
    logic                              mac_last;
    logic signed [B_BIT-1:0]           mac_acc[P_OCH];

    rom #(
        .DWIDTH(P_OCH * P_ICH * W_BIT),
        .AWIDTH($clog2(WEIGHT_DEPTH)),
        .MEM_SIZE(WEIGHT_DEPTH),
        .INIT_FILE(W_FILE),
        .ROM_TYPE(W_ROM_TYPE)
    ) u_weight_rom (
        .clk  (clk),
        .ce0  (proc_step_en),
        .addr0(weight_addr),
        .q0   (weight_data)
    );

    ram #(
        .DWIDTH(P_ICH * A_BIT),
        .AWIDTH(LB_AWIDTH),
        .MEM_SIZE(LB_DEPTH)
    ) u_line_buffer (
        .clk  (clk),
        .we   (line_buffer_we),
        .waddr(line_buffer_waddr),
        .wdata(line_buffer_wdata),
        .re   (line_buffer_re),
        .raddr(line_buffer_raddr),
        .rdata(line_buffer_rdata)
    );

    generate
        for (genvar i = 0; i < P_ICH; i++) begin : gen_skew
            delayline #(
                .WIDTH(A_BIT),
                .DEPTH(i)
            ) u_dx (
                .clk     (clk),
                .rst_n   (rst_n),
                .en      (proc_step_en),
                .data_in (x_vec[i]),
                .data_out(x_vec_delayed[i])
            );

            for (genvar o = 0; o < P_OCH; o++) begin : gen_w_skew
                delayline #(
                    .WIDTH(W_BIT),
                    .DEPTH(i)
                ) u_dw (
                    .clk     (clk),
                    .rst_n   (rst_n),
                    .en      (proc_step_en),
                    .data_in (w_vec[o][i]),
                    .data_out(w_vec_delayed[o][i])
                );
            end
        end
    endgenerate

    delayline #(
        .WIDTH(1),
        .DEPTH(P_ICH - 1)
    ) u_mac_vld (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (proc_step_en),
        .data_in (issue_valid_pos_d),
        .data_out(mac_dat_vld)
    );

    delayline #(
        .WIDTH(1),
        .DEPTH(P_ICH - 1)
    ) u_clr_skew (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (proc_step_en),
        .data_in (issue_first_d),
        .data_out(mac_clr)
    );

    delayline #(
        .WIDTH(1),
        .DEPTH(P_ICH)
    ) u_last_skew (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (proc_step_en),
        .data_in (issue_last_d),
        .data_out(mac_last)
    );

    generate
        for (genvar o = 0; o < P_OCH; o++) begin : gen_mac_array
            mac_array #(
                .P_ICH(P_ICH),
                .A_BIT(A_BIT),
                .W_BIT(W_BIT),
                .B_BIT(B_BIT)
            ) u_mac_array (
                .clk    (clk),
                .rst_n  (rst_n),
                .en     (proc_step_en),
                .dat_vld(mac_dat_vld),
                .clr    (mac_clr),
                .x_vec  (x_vec_delayed),
                .w_vec  (w_vec_delayed[o]),
                .acc    (mac_acc[o])
            );
        end
    endgenerate

    assign in_ready = (state == ST_INIT);
    assign proc_step_en = ((state == ST_PROC) || (state == ST_DRAIN)) && out_ready;

    // 从输出坐标反推输入坐标
    assign h_temp = cntr_oh - cntr_kh + P;
    assign w_temp = cntr_ow - cntr_kw + P;
    assign ih = h_temp / S;
    assign iw = w_temp / S;
    
    assign valid_pos = (state == ST_PROC) &&
                       (h_temp >= 0) && (w_temp >= 0) &&
                       ((h_temp % S) == 0) && ((w_temp % S) == 0) &&
                       (ih >= 0) && (ih < N_IH) &&
                       (iw >= 0) && (iw < N_IW);

    assign issue_first = (state == ST_PROC) && (cntr_kh == K - 1) && (cntr_kw == K - 1) && (cntr_fi == 0);
    assign issue_last = (state == ST_PROC) && (cntr_kh == 0) && (cntr_kw == 0) && (cntr_fi == FOLD_I - 1);

    assign weight_addr = (cntr_fo * KK * FOLD_I) + (cntr_fi * KK) + (cntr_kh * K + cntr_kw);
    assign line_buffer_we = (state == ST_INIT) && in_valid;
    assign line_buffer_waddr = cntr_init_h * N_IW * FOLD_I + cntr_init_w * FOLD_I + cntr_init_fi;
    assign line_buffer_wdata = in_data;
    assign line_buffer_re = proc_step_en;
    assign line_buffer_raddr = ih * N_IW * FOLD_I + iw * FOLD_I + cntr_fi;

    always_comb begin
        for (int i = 0; i < P_ICH; i++) begin
            x_vec[i] = line_buffer_rdata[i*A_BIT+:A_BIT];
        end
    end

    always_comb begin
        for (int o = 0; o < P_OCH; o++) begin
            for (int i = 0; i < P_ICH; i++) begin
                w_vec[o][i] = weight_data[(P_ICH*o+i)*W_BIT+:W_BIT];
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            issue_valid_pos_d <= 1'b0;
            issue_first_d     <= 1'b0;
            issue_last_d      <= 1'b0;
        end else if (proc_step_en) begin
                issue_valid_pos_d <= valid_pos;
                issue_first_d     <= issue_first;
                issue_last_d      <= issue_last;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_INIT;
            cntr_init_h  <= 0;
            cntr_init_w  <= 0;
            cntr_init_fi <= 0;
            cntr_oh      <= 0;
            cntr_ow      <= 0;
            cntr_fo      <= 0;
            cntr_kh      <= K - 1;
            cntr_kw      <= K - 1;
            cntr_fi      <= 0;
            cntr_drain   <= 0;
        end else begin
            case (state)
                ST_INIT: begin          // 初始化，遍历一块输入的所有值
                    if (in_valid) begin
                        if (cntr_init_fi == FOLD_I - 1) begin
                            cntr_init_fi <= 0;
                            if (cntr_init_w == N_IW - 1) begin
                                cntr_init_w <= 0;
                                if (cntr_init_h == N_IH - 1) begin
                                    state <= ST_PROC;
                                    cntr_oh <= 0;
                                    cntr_ow <= 0;
                                    cntr_fo <= 0;
                                    cntr_kh <= K - 1;
                                    cntr_kw <= K - 1;
                                    cntr_fi <= 0;
                                    cntr_drain <= 0;
                                end else begin
                                    cntr_init_h <= cntr_init_h + 1;
                                end
                            end else begin
                                cntr_init_w <= cntr_init_w + 1;
                            end
                        end else begin
                            cntr_init_fi <= cntr_init_fi + 1;
                        end
                    end
                end

                ST_PROC: begin
                    if (proc_step_en) begin
                        if (cntr_fi == FOLD_I - 1) begin
                            cntr_fi <= 0;
                            if (cntr_kw == 0) begin
                                cntr_kw <= K - 1;
                                if (cntr_kh == 0) begin
                                    cntr_kh <= K - 1;
                                    if (cntr_fo == FOLD_O - 1) begin
                                        cntr_fo <= 0;
                                        if (cntr_ow == N_OW - 1) begin
                                            cntr_ow <= 0;
                                            if (cntr_oh == N_OH - 1) begin
                                                // 进入排空阶段，保证最后一拍结果从流水线推出
                                                state      <= ST_DRAIN;
                                                // 额外多排空一拍，用于把 mac_last 后续的 0 推出，避免 out_valid 卡高
                                                cntr_drain <= P_ICH + 1;
                                            end else begin
                                                cntr_oh <= cntr_oh + 1;
                                            end
                                        end else begin
                                            cntr_ow <= cntr_ow + 1;
                                        end
                                    end else begin
                                        cntr_fo <= cntr_fo + 1;
                                    end
                                end else begin
                                    cntr_kh <= cntr_kh - 1;
                                end
                            end else begin
                                cntr_kw <= cntr_kw - 1;
                            end
                        end else begin
                            cntr_fi <= cntr_fi + 1;
                        end
                    end
                end

                ST_DRAIN: begin
                    if (proc_step_en) begin
                        if (cntr_drain == 1) begin
                            state        <= ST_INIT;
                            cntr_init_h  <= 0;
                            cntr_init_w  <= 0;
                            cntr_init_fi <= 0;
                            cntr_oh      <= 0;
                            cntr_ow      <= 0;
                            cntr_fo      <= 0;
                            cntr_kh      <= K - 1;
                            cntr_kw      <= K - 1;
                            cntr_fi      <= 0;
                            cntr_drain   <= 0;
                        end else begin
                            cntr_drain <= cntr_drain - 1;
                        end
                    end
                end
            endcase
        end
    end

    always_comb begin
        for (int o = 0; o < P_OCH; o++) begin
            out_data_r[o*B_BIT+:B_BIT] <= mac_acc[o];
        end
    end

    assign out_data = out_data_r;
    assign out_valid = mac_last;

endmodule
