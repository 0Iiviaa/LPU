// ---------------------------------------------------
// 修改：
// 1. 第一个fo也从ram读数据，减少分支
// 2. 统一代码形式，方便对齐
// 3. 数据和权重斜化放进mac内部
// 4. 流水线驱动简化，逻辑：
//    · mac使能只看out_ready，out_ready = 1 时更新输出，out_ready = 0 时输出保持，防止丢弃有效数据
//    · in_vaild无效要让流水线停住，但是不能让mac的计算停住，防止正在计算的有效数据卡住
//    · mac使能，mac输入数据和相应的控制信号需要对齐，所以rom和ram的读使能以及控制信号的delayline使能和mac一致
//    · flush机制通过pipe_en和pipe_en_out分离实现，只需要在in_valid=0时拉低is_lst_kk_fi_d0，out_ready会在pipe_en_out驱动的延迟后拉低
// ---------------------------------------------------
// ---------------------------------------------------
// 学习成果检验：
// 取消减少分支的设计，in_data打一拍和rom/ram的输出对齐
// · ram不存在连续读写，读地址不用打拍，为d0
// · 增加in_buf选择输入分支
// · 所有信号对齐位置为d1
// ---------------------------------------------------

module conv #(
    parameter int unsigned P_ICH      = 4,
    parameter int unsigned P_OCH      = 4,
    parameter int unsigned N_ICH      = 16,
    parameter int unsigned N_OCH      = 16,
    parameter int unsigned K          = 3,
    parameter int unsigned A_BIT      = 8,
    parameter int unsigned W_BIT      = 8,
    parameter int unsigned B_BIT      = 32,
    parameter int unsigned N_HW       = 64,
    parameter string       W_FILE     = "",
    parameter              W_ROM_TYPE = "block"
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic [P_ICH*A_BIT-1:0] in_data,
    input  logic                   in_valid,
    output logic                   in_ready,
    output logic [P_OCH*B_BIT-1:0] out_data,
    output logic                   out_valid,
    input  logic                   out_ready
);

    localparam int unsigned FOLD_I = N_ICH / P_ICH;
    localparam int unsigned FOLD_O = N_OCH / P_OCH;
    localparam int unsigned KK = K * K;
    localparam int unsigned WEIGHT_DEPTH = FOLD_O * FOLD_I * KK;
    localparam int unsigned LB_DEPTH = FOLD_I * KK;
    localparam int unsigned LB_AWIDTH = $clog2(LB_DEPTH);

    logic signed [               B_BIT-1:0] acc                     [  P_OCH];
    logic        [      $clog2(N_HW+1)-1:0] cntr_hw;
    logic        [    $clog2(FOLD_O+1)-1:0] cntr_fo;
    logic        [    $clog2(FOLD_I+1)-1:0] cntr_fi;
    logic        [        $clog2(KK+1)-1:0] cntr_kk;
    logic                                   pipe_en;
    logic                                   pipe_en_in;
    logic                                   pipe_en_out;
    logic        [         P_ICH*A_BIT-1:0] in_buf;
    logic                                   is_fst_fo_d0;
    logic                                   is_fst_fo_d1;
    logic        [         P_ICH*A_BIT-1:0] in_data_d1;
    logic                                   is_fst_kk_fi_d0;
    logic                                   is_fst_kk_fi_d1;
    logic                                   is_fst_kk_fi_delayed;
    logic                                   is_lst_kk_fi_d0;
    logic                                   is_lst_kk_fi_d1;
    logic                                   is_lst_kk_fi_delayed;
    logic                                   line_buffer_we;
    logic        [           LB_AWIDTH-1:0] line_buffer_waddr_d0;
    logic        [         P_ICH*A_BIT-1:0] line_buffer_wdata_d0;
    logic                                   line_buffer_re;
    logic        [           LB_AWIDTH-1:0] line_buffer_raddr_d0;
    logic        [           LB_AWIDTH-1:0] line_buffer_raddr_d1;
    logic        [         P_ICH*A_BIT-1:0] line_buffer_rdata_d1;
    logic        [$clog2(WEIGHT_DEPTH)-1:0] weight_addr_d0;
    logic        [   P_OCH*P_ICH*W_BIT-1:0] weight_data_d1;
    logic                                   mac_array_data_vld_d0;
    logic                                   mac_array_data_vld_d1;
    logic                                   mac_array_data_vld_delayed;



    assign is_fst_fo_d0             = (cntr_fo == 0);
    assign is_fst_kk_fi_d0          = (cntr_kk == 0) && (cntr_fi == 0);       
    assign is_lst_kk_fi_d0          = (cntr_kk == KK - 1) && (cntr_fi == FOLD_I - 1) && pipe_en_in;     
    assign pipe_en_in               = is_fst_fo_d0 ? in_valid : 1'b1;
    assign pipe_en_out              = (!out_valid || out_ready);        // 只有out_valid = 1 且 out_ready = 0时流水线停住
    assign pipe_en                  = pipe_en_in && (!out_valid || out_ready);
    assign in_ready                 = is_fst_fo_d0 && pipe_en_out;
    assign weight_addr_d0           = (cntr_fo * KK * FOLD_I) + cntr_fi * KK + cntr_kk;
    assign line_buffer_we           = is_fst_fo_d0 && in_valid;
    assign line_buffer_waddr_d0     = cntr_fi * KK + cntr_kk;
    assign line_buffer_wdata_d0     = in_data;
    assign line_buffer_re           = (!out_valid || out_ready);
    assign line_buffer_raddr_d0     = cntr_fi * KK + cntr_kk;
    assign mac_array_data_vld_d0    = (is_fst_fo_d0 ? in_valid : 1'b1);


    rom #(
        .DWIDTH(P_OCH * P_ICH * W_BIT),
        .AWIDTH($clog2(WEIGHT_DEPTH)),
        .MEM_SIZE(WEIGHT_DEPTH),
        .INIT_FILE(W_FILE),
        .ROM_TYPE(W_ROM_TYPE)
    ) u_weight_rom (
        .clk  (clk),
        .ce0  (pipe_en_out),
        .addr0(weight_addr_d0),
        .q0   (weight_data_d1)
    );

    ram #(
        .DWIDTH(P_ICH * A_BIT),
        .AWIDTH(LB_AWIDTH),
        .MEM_SIZE(LB_DEPTH)
    ) u_line_buffer (
        .clk  (clk),
        .we   (line_buffer_we),
        .waddr(line_buffer_waddr_d0),
        .wdata(line_buffer_wdata_d0),
        .re   (line_buffer_re),
        .raddr(line_buffer_raddr_d0),
        .rdata(line_buffer_rdata_d1)
    );


    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cntr_hw <= 0;
            cntr_fo <= 0;
            cntr_fi <= 0;
            cntr_kk <= 0;
        end else if (pipe_en) begin     
            if (cntr_kk == KK - 1) begin  
                cntr_kk <= 0;
                if (cntr_fi == FOLD_I - 1) begin
                    cntr_fi <= 0;
                    if (cntr_fo == FOLD_O - 1) begin  
                        cntr_fo <= 0;
                        if (cntr_hw == N_HW - 1) begin
                            cntr_hw <= 0;
                        end else begin
                            cntr_hw <= cntr_hw + 1;
                        end
                    end else begin
                        cntr_fo <= cntr_fo + 1;
                    end
                end else begin
                    cntr_fi <= cntr_fi + 1;
                end
            end else begin
                cntr_kk <= cntr_kk + 1;
            end
        end
    end


    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_data_d1              <= '0;
            is_fst_fo_d1            <= '0;
            is_fst_kk_fi_d1         <= '0;
            is_lst_kk_fi_d1         <= '0;
            mac_array_data_vld_d1   <= '0;
        end else if (pipe_en_out) begin
            in_data_d1              <= in_data;
            is_fst_fo_d1            <= is_fst_fo_d0;
            is_fst_kk_fi_d1         <= is_fst_kk_fi_d0;
            is_lst_kk_fi_d1         <= is_lst_kk_fi_d0;
            mac_array_data_vld_d1   <= mac_array_data_vld_d0;
        end
    end


    assign in_buf = is_fst_fo_d1 ? in_data_d1 : line_buffer_rdata_d1;

    logic        [A_BIT-1:0] x_vec[P_ICH];
    logic signed [W_BIT-1:0] w_vec[P_OCH] [P_ICH];

    always_comb begin
        for (int i = 0; i < P_ICH; i++) begin
            x_vec[i] = in_buf[i*A_BIT+:A_BIT];
        end
    end

    always_comb begin
        for (int o = 0; o < P_OCH; o++) begin
            for (int i = 0; i < P_ICH; i++) begin
                w_vec[o][i] = weight_data_d1[(P_ICH*o+i)*W_BIT+:W_BIT]; 
            end
        end
    end

    // mac数据有效，控制mac_tail，延时I-1个周期
    delayline #(
        .WIDTH(1), 
        .DEPTH(P_ICH - 1)
    ) u_mac_vld (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (pipe_en_out),
        .data_in (mac_array_data_vld_d1),
        .data_out(mac_array_data_vld_delayed)
    );

    // mac累加清零，控制mac_tail，延时I-1个周期
    delayline #(
        .WIDTH(1), 
        .DEPTH(P_ICH - 1)
    ) u_clr_skew (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (pipe_en_out),
        .data_in (is_fst_kk_fi_d1),
        .data_out(is_fst_kk_fi_delayed)
    );

    // mac输出有效，与cascade[P_ICH]对齐，延迟I个周期
    delayline #(
        .WIDTH(1), 
        .DEPTH(P_ICH)
    ) u_out_vld (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (pipe_en_out),
        .data_in (is_lst_kk_fi_d1),
        .data_out(is_lst_kk_fi_delayed)
    );


    // 实例化 MAC 阵列，挂载到 pipe_en_backend
    generate
        for (genvar o = 0; o < P_OCH; o++) begin : gen_mac_array
            conv_mac_array #(
                .P_ICH(P_ICH), 
                .A_BIT(A_BIT), 
                .W_BIT(W_BIT), 
                .B_BIT(B_BIT)
            ) u_mac_array (
                .clk    (clk),
                .rst_n  (rst_n),
                .en     (pipe_en_out),
                .dat_vld(mac_array_data_vld_delayed),   
                .clr    (is_fst_kk_fi_delayed),   
                .x_vec  (x_vec),
                .w_vec  (w_vec[o]),
                .acc    (acc[o])
            );
        end
    endgenerate

    assign out_valid = is_lst_kk_fi_delayed;

    always_comb begin
        for (int o = 0; o < P_OCH; o++) begin
            out_data[o*B_BIT+:B_BIT] = acc[o];
        end
    end

endmodule