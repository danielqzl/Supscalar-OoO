module riscv 
    import riscv_pkg ::*;
#(
    parameter INSTR_QUEUE_SIZE = 32,
    parameter NUM_PHY_REGS = 64
) (
    input  logic clk, rst_n, valid, 
    // Instruction writes 
    input  logic instr_wr_en, 
    input  logic [31:0] wr_instr,
    // Excpetion 
    input  logic resume, 
    input  logic [31:0] resume_pc,
    output logic exception,
    // output logic rb_done,
    output logic [31:0] cmt_pc,
    output logic completed,   

    mem_if.fu mem,
    
    sim_if.dut sim
);
    localparam N_RS = 4;
    localparam N_FU = 4;

    // ------------------------------------------------------------------------
    // Signals
    // ------------------------------------------------------------------------
    // instr queue <-> dispatch
    uop_s uop_deq    [2];   // uop from uop queue to dispatch 
    logic [0:1] dispatch_valid;
    logic [0:1] uop_q_valid;

    // Rename Output 
    rename_s rename_res [2];
    logic    ren_valid  [2];
    logic [0:1] rs_accept [N_RS];

    // RS <-> ALU
    // Logic 
    rs_fu_s rs_fu_sig_logic [2];
    logic   fu_free_logic   [2];
    // multiply 
    rs_fu_s rs_fu_sig_mul [1];
    logic   fu_free_mul   [1];
    // Load/Store
    rs_fu_s rs_fu_sig_ld [1], rs_fu_sig_st [1];
    logic   fu_free_ld [1], fu_free_st [1];

    // Commit Results from ROB 
    logic in_rollback;
    logic exc_flush;
    cmt_res_s cmt_res [2];

    // Harzad control signals 
    logic [0:N_RS-1] rs_stalled_full;
    logic [0:1] rename_stalled, rob_credit;
    logic stall;


    // Common Data Bus interface
    cdb_if #(.N(N_FU),  .TAG_WIDTH(TAG_WIDTH)) i_cdb (
        .clk(clk), .rst_n(rst_n)
    );


    // ------------------------------------------------------------------------
    // Module Instantiations
    // ------------------------------------------------------------------------
    // 1. Instruction Queue
    instruction_queue #(
        .SIZE(INSTR_QUEUE_SIZE)
    ) u_instr_queue (
        .clk(clk), .rst_n(rst_n),  
        .wr_en(instr_wr_en), .wr_instr(wr_instr),
        .dispatch_valid(dispatch_valid),
        .uop_q_valid(uop_q_valid),
        .uop_out(uop_deq),
        .stall(stall), // temp 
        .flush(exc_flush),
        .cmt(cmt_res),
    
        .resume(resume),
        .resume_pc(resume_pc),
        .cmt_pc_o(cmt_pc),
        .completed(completed)
    );


    //-----------------------------------------------------
    // 2. Dispatch 
    //-----------------------------------------------------
    dispatch_unit u_disp (
        // .uop(uop_deq),
        .uop_q_valid(uop_q_valid),
        .dispatch_valid(dispatch_valid),
        .stall(stall)
    );


    //-----------------------------------------------------
    // 3. Register File & Rename 
    //-----------------------------------------------------
    regFile #(.PHY_REGS(NUM_PHY_REGS)) u_regFile(
        .clk(clk), .rst_n(rst_n),
        // Dispathch Input  
        .dispatch_valid(dispatch_valid),
        .uop_in(uop_deq),
        // Rename Output
        .ren_o(rename_res),
        .ren_valid(ren_valid),
        // hardzard unit 
        .rename_stalled(rename_stalled),
        .stall(stall),  //.stall_o(stall_RO),
        .flush(exc_flush),
        // ROB Side
        .cmt(cmt_res),
        .sim(sim)
    );


    //-----------------------------------------------------
    // 4. Logic Reservation Station(scheduler) & FU
    //-----------------------------------------------------
    reservation_station #(
        .N_ENTRY(4), .N_FU(2),  .OP_MASK(4'b1000)
    ) u_rs_logic (
        .clk(clk), .rst_n(rst_n),
        .rs_accept(rs_accept[0]), 
        .ren(rename_res),        .ren_valid(ren_valid),
        .fu_in(rs_fu_sig_logic), .fu_free(fu_free_logic),  
        
        .flush(exc_flush),       .stall(stall),  .stall_q('0),
        .rs_stalled_full(rs_stalled_full[0]),

        .cmt(cmt_res),           .cdb(i_cdb.consumer)
    );

    generate
    for (genvar i = 0; i < 2; i++) begin : g_alu
        alu_logic #(.ID(i)) u_alu_logic (
            .clk(clk), .rst_n(rst_n),
            .alu_in(rs_fu_sig_logic[i]),
            .free(fu_free_logic[i]),
            .cdb(i_cdb.producer) 
        );
    end
    endgenerate


    //-----------------------------------------------------
    // 5. Multiply Reservation Station(scheduler) & FU
    //-----------------------------------------------------
    reservation_station #(
        .N_ENTRY(4), .N_FU(1),  .OP_MASK(4'b0100)
    ) u_rs_mul (
        .clk(clk), .rst_n(rst_n),
        .rs_accept(rs_accept[1]), 
        .ren(rename_res),        .ren_valid(ren_valid),
        .fu_in(rs_fu_sig_mul), .fu_free(fu_free_mul),  

        .flush(exc_flush),       .stall(stall),  .stall_q('0),
        .rs_stalled_full(rs_stalled_full[1]),
        
        .cmt(cmt_res),           .cdb(i_cdb.consumer)
    );

    alu_mul #(.ID(2)) u_alu_mul (
        .clk(clk), .rst_n(rst_n),
        .alu_in(rs_fu_sig_mul[0]),
        .free(fu_free_mul[0]),
        .cdb(i_cdb.producer) 
    );


    //-----------------------------------------------------
    // 6. Memory Unit 
    //-----------------------------------------------------
    logic [0:1] ldq_full, stq_full;
    logic ld_valid, st_valid;
    logic [31:0] ld_addr, st_addr;
    logic [TAG_WIDTH-1:0] ld_tag, st_tag;
    logic [31:0] st_data, st_imm [1];

    reservation_station #(
        .N_ENTRY(4), .N_FU(1),  .OP_MASK(4'b0010)
    ) u_rs_load (
        .clk(clk), .rst_n(rst_n),
        .rs_accept(rs_accept[2]),
        .ren(rename_res),        .ren_valid(ren_valid),
        .fu_in(rs_fu_sig_ld),    .fu_free(fu_free_ld),  

        .flush(exc_flush),       .stall(stall),  .stall_q(ldq_full),
        .rs_stalled_full(rs_stalled_full[2]),
        
        .cmt(cmt_res),           .cdb(i_cdb.consumer)
    );

    LD_FU u_ld_fu (
        .clk(clk), .rst_n(rst_n),
        .alu_in(rs_fu_sig_ld[0]),
        .free(fu_free_ld[0]),
        .queue_full(ldq_full),
        .valid(ld_valid),
        .rd_tag(ld_tag),
        .addr(ld_addr)
    );

    reservation_station #(
        .N_ENTRY(4), .N_FU(1),  .OP_MASK(4'b0001), .IS_ST(1'b1)
    ) u_rs_store (
        .clk(clk), .rst_n(rst_n),
        .rs_accept(rs_accept[3]),
        .ren(rename_res),        .ren_valid(ren_valid),
        .fu_in(rs_fu_sig_st),    .fu_free(fu_free_st),  .st_imm(st_imm),
        .flush(exc_flush),       .stall(stall),  .stall_q(stq_full),
        .rs_stalled_full(rs_stalled_full[3]),
        .cmt(cmt_res),           .cdb(i_cdb.consumer)
    );

    ST_FU u_st_fu (
        .clk(clk), .rst_n(rst_n),
        .alu_in(rs_fu_sig_st[0]),
        .imm(st_imm[0]),
        .free(fu_free_st[0]),
        .queue_full(stq_full),
        .valid(st_valid),
        .tag(st_tag),
        .addr(st_addr),
        .data(st_data)
    );

    memory_unit #(.ID(3)) u_mem_unit ( 
        .clk(clk), .rst_n(rst_n), 
        .ren(rename_res),
        .ldq_full(ldq_full), .stq_full(stq_full),
        .ld_rs_accept(rs_accept[2]), .st_rs_accept(rs_accept[3]),
        // From Address Unit
        .ld_valid(ld_valid), 
        .ld_addr(ld_addr),
        .ld_tag(ld_tag),
        .st_valid(st_valid),
        .st_addr(st_addr),
        .st_tag(st_tag),
        .st_data(st_data), 

        .flush(exc_flush), .stall(stall),
        .cmt(cmt_res),
        .cdb(i_cdb.producer),
        .mem(mem)
    );


    //-----------------------------------------------------
    // 7. Reorder Buffer 
    //-----------------------------------------------------
    rob #(8) u_rob (
        .clk(clk), .rst_n(rst_n),
        .stall(stall),
        .ren_valid_i(ren_valid),
        .ren_i(rename_res),
        .rob_free(rob_credit),
        //.in_rollback(in_rollback),
        // .rollback_done(rb_done),   
        .exc_flush(exc_flush),    
        .cmt(cmt_res),
        .cdb(i_cdb.consumer)
    );


    //-----------------------------------------------------
    // 8. Harzard Unit
    //-----------------------------------------------------
    hazard_unit #(.N_RS(N_RS)) u_hazard_unit (
        .clk(clk), .rst_n(rst_n),
        .rename_stalled(rename_stalled),
        .rob_credit(rob_credit),
        .ren_valid(ren_valid),
        .rs_stalled_full(rs_stalled_full),
        .cmt(cmt_res),
        .valid(valid),
        .resume(resume),
        .exc_flush(exc_flush),   
        .stall(stall),
        .proc_excepion(exception) 
    );


    //-----------------------------------------------------
    // 9. Common Data Bus
    //-----------------------------------------------------
    cdb_arbiter #(4) u_cdb_arbiter (
        .clk(clk), .rst_n(rst_n),
        .cdb(i_cdb.arbiter)
    );

endmodule
