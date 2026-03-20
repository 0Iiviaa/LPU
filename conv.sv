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
    
    logic                                   pipe_en_in;
    logic                                   pipe_en_out;
    // --- 流水线前后端分离使能信号 ---
    logic                                   pipe_en_frontend;
    logic                                   pipe_en_backend;
    logic        [               P_ICH-1:0] out_vld_sr;
    logic                                   is_flushing;
    logic                                   out_vld_pre;
    // ------------------------------------
    logic                                   is_fst_fo;
    logic                                   mac_array_data_vld;
    logic        [         P_ICH*A_BIT-1:0] in_buf;
    logic                                   is_fst_kk_fi;
    logic                                   is_lst_kk_fi;
    logic                                   line_buffer_we;
    logic        [           LB_AWIDTH-1:0] line_buffer_waddr;
    logic        [         P_ICH*A_BIT-1:0] line_buffer_wdata;
    logic                                   line_buffer_re;
    logic        [           LB_AWIDTH-1:0] line_buffer_raddr;
    logic        [         P_ICH*A_BIT-1:0] line_buffer_rdata;
    logic        [$clog2(WEIGHT_DEPTH)-1:0] weight_addr;
    logic        [   P_OCH*P_ICH*W_BIT-1:0] weight_data;

    rom #(
        .DWIDTH(P_OCH * P_ICH * W_BIT),
        .AWIDTH($clog2(WEIGHT_DEPTH)),
        .MEM_SIZE(WEIGHT_DEPTH),
        .INIT_FILE(W_FILE),
        .ROM_TYPE(W_ROM_TYPE)
    ) u_weight_rom (
        .clk  (clk),
        .ce0  (pipe_en_frontend), // 前端使能控制
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

    assign is_fst_fo            = (cntr_fo == 0);
    assign is_fst_kk_fi         = (cntr_kk == 0) && (cntr_fi == 0);       
    assign is_lst_kk_fi         = (cntr_kk == KK - 1) && (cntr_fi == FOLD_I - 1) && pipe_en_in;     
    assign pipe_en_in           = is_fst_fo ? in_valid : 1'b1;

    // --- 流水线使能解耦 ---
    assign pipe_en_frontend     = pipe_en_in && pipe_en_out;
    assign pipe_en_backend      = (pipe_en_in || is_flushing) && pipe_en_out;
    // ----------------------------
    
    assign in_ready             = is_fst_fo && pipe_en_out;

    assign weight_addr          = (cntr_fo * KK * FOLD_I) + cntr_fi * KK + cntr_kk;
    assign line_buffer_we       = is_fst_fo && pipe_en_frontend; // 前端使能控制
    assign line_buffer_waddr    = cntr_fi * KK + cntr_kk;
    assign line_buffer_wdata    = in_data;
    assign line_buffer_re       = !is_fst_fo && pipe_en_frontend; // 前端使能控制
    assign line_buffer_raddr    = cntr_fi * KK + cntr_kk;

    // 计数器挂载到 pipe_en_frontend
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cntr_hw <= 0;
            cntr_fo <= 0;
            cntr_fi <= 0;
            cntr_kk <= 0;
        end else if (pipe_en_frontend) begin     
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

    logic [P_ICH*A_BIT-1:0] in_data_d1;
    logic                   is_fst_fo_d1;
    logic                   in_valid_d1;
    logic                   is_fst_kk_fi_d1;
    logic                   is_lst_kk_fi_d1;
    logic                   mac_array_data_vld_d1;

    // 数据打拍挂载到 pipe_en_backend，保证 Flush 阶段能送入无效数据/清零信号
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_data_d1       <= '0;
            is_fst_fo_d1     <= '0;
            in_valid_d1      <= '0;
            is_fst_kk_fi_d1  <= '0;
            is_lst_kk_fi_d1  <= '0;
        end else if (pipe_en_backend) begin         
            in_data_d1       <= in_data;
            is_fst_fo_d1     <= is_fst_fo;
            in_valid_d1      <= in_valid;
            is_fst_kk_fi_d1  <= is_fst_kk_fi;
            is_lst_kk_fi_d1  <= is_lst_kk_fi;
        end
    end

    assign mac_array_data_vld_d1 = (is_fst_fo_d1 ? in_valid_d1 : 1'b1);
    assign in_buf = is_fst_fo_d1 ? in_data_d1 : line_buffer_rdata;

    // 延迟全部挂载到 pipe_en_backend
    delayline #(
        .WIDTH(1), 
        .DEPTH(P_ICH - 1)
    ) u_mac_vld (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (pipe_en_backend), // 修改为后端使能
        .data_in (mac_array_data_vld_d1),
        .data_out(mac_array_data_vld)
    );

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
                w_vec[o][i] = weight_data[(P_ICH*o+i)*W_BIT+:W_BIT]; 
            end
        end
    end

    // 数据与权重斜化 (Data Skewing) 挂载到 pipe_en_backend
    logic [A_BIT-1:0] x_vec_delayed [P_ICH];
    logic [W_BIT-1:0] w_vec_delayed [P_OCH][P_ICH];

    generate
        for (genvar i = 0; i < P_ICH; i++) begin : gen_skew
            delayline #(
                .WIDTH(A_BIT), 
                .DEPTH(i)
            ) u_dx (
                .clk(clk), 
                .rst_n(rst_n), 
                .en(pipe_en_backend),  // 修改为后端使能
                .data_in(x_vec[i]), 
                .data_out(x_vec_delayed[i])
            );
            
            for (genvar o = 0; o < P_OCH; o++) begin : gen_w_skew
                delayline #(
                    .WIDTH(W_BIT), 
                    .DEPTH(i)
                ) u_dw (
                    .clk(clk), 
                    .rst_n(rst_n), 
                    .en(pipe_en_backend),  // 修改为后端使能
                    .data_in(w_vec[o][i]), 
                    .data_out(w_vec_delayed[o][i])
                );
            end
        end
    endgenerate

    // 每一个新的输出位置清零信号延迟挂载到 pipe_en_backend
    logic clr_delayed;
    delayline #(
        .WIDTH(1), 
        .DEPTH(P_ICH - 1)
    ) u_clr_skew (
        .clk     (clk),
        .rst_n   (rst_n),
        .en      (pipe_en_backend), // 修改为后端使能
        .data_in (is_fst_kk_fi_d1),
        .data_out(clr_delayed)
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
                .en     (pipe_en_backend), // 修改为后端使能
                .dat_vld(mac_array_data_vld),   
                .clr    (clr_delayed),   
                .x_vec  (x_vec_delayed),
                .w_vec  (w_vec_delayed[o]),
                .acc    (acc[o])
            );
        end
    endgenerate

    // --- 使用移位寄存器管理 Flush 状态及取代原本的 u_out_vld_delay ---
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_vld_sr <= '0;
        end else if (pipe_en_backend) begin
            out_vld_sr <= {out_vld_sr[P_ICH-2:0], is_lst_kk_fi_d1};
        end
    end

    assign out_vld_pre = out_vld_sr[P_ICH-1];
    assign is_flushing = is_lst_kk_fi_d1 | (|out_vld_sr);
    // ------------------------------------------------------------------------

    // 记录当前输出是否在流水线 stall 期间已经被读走
    logic out_consumed;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_consumed <= 1'b0;
        // 使用 pipe_en_backend 作为推进标准
        end else if (pipe_en_backend) begin
            out_consumed <= 1'b0; 
        end else if (out_valid && out_ready) begin
            out_consumed <= 1'b1; 
        end
    end

    // 如果数据已经被读走了，就不再拉高 valid，防止重复读取
    assign out_valid   = out_vld_pre && !out_consumed;
    
    // 如果数据被读走了，即使后级 out_ready 拉低也要允许流水线前进
    assign pipe_en_out = out_ready || out_consumed; 

    always_comb begin
        for (int o = 0; o < P_OCH; o++) begin
            out_data[o*B_BIT+:B_BIT] = acc[o];
        end
    end

endmodule