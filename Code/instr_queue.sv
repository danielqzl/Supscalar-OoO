module instruction_queue 
    import riscv_pkg::*;
#(
    parameter SIZE = 256
)(
    input  logic clk, rst_n, 

    // --- Write Port (Load Instructions) ---
    input  logic  wr_en,
    input  logic [31:0] wr_instr,

    // --- Read Port (Dispatch Instructions) --- 
    input  logic [0:1] dispatch_valid,
    output logic [0:1] uop_q_valid,    // uop queue is not empty
    output var uop_s uop_out [2],

    // --- Excpetion Port ---
    input  logic stall,
    input  logic flush,
    input  var cmt_res_s cmt [2],
    input  logic resume,
    input  logic [31:0] resume_pc,
    output logic [31:0] cmt_pc_o,
    output logic completed  // signal complete / idle 
);
    // PC 
    logic [31:0] program_pc;
    logic [31:0] pc [2];
    logic [31:0] pc_next [2]; 
    logic [31:0] cmt_pc, cmt_pc_next; 
    logic [31:0] instr [2];
    uop_s uop_dec [2];

    logic [0:1] fetch_instr;
    // logic [0:1] fetch_ok;
    logic [0:1] uop_q_free;
    logic uop_queue_clr;

    logic stalled;
    logic stall_fetch;

    //-----------------------------------------------------
    // Program Counter 
    // ----------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin : r_prog_pc
        if (!rst_n) 
            program_pc <= '0;
        else if(wr_en)
            program_pc <= program_pc + 4;
    end

    always_ff @(posedge clk or negedge rst_n) begin : r_pc
        if (!rst_n) begin
            pc[0] <= '0;
            pc[1] <= '0;
            cmt_pc <= '0;
        end else begin
            pc[0] <= pc_next[0];
            pc[1] <= pc_next[1];
            cmt_pc <= cmt_pc_next;
        end
    end

    always_comb begin : u_pc_next
        if (resume) begin
            pc_next[0] = resume_pc;
        end else if (flush) begin
            pc_next[0] = cmt_pc_next;
        end else if (fetch_instr == 2'b11) begin
            pc_next[0] = pc[0] + 8;
        end else if (fetch_instr == 2'b10) begin
            pc_next[0] = pc[0] + 4;
        end else begin
            pc_next[0] = pc[0];
        end
        pc_next[1] = pc_next[0] + 4;
    end

    always_comb begin : u_cmt_pc_logic
        if (cmt[0].commit_valid && cmt[1].commit_valid) 
            cmt_pc_next = cmt_pc + 8;
        else if(cmt[0].commit_valid || cmt[1].commit_valid)
            cmt_pc_next = cmt_pc + 4;
        else if (resume)
            cmt_pc_next = resume_pc;
        else   
            cmt_pc_next = cmt_pc;
    end



    //-----------------------------------------------------
    // Instruction Memory
    // ----------------------------------------------------
    instr_mem #(
        .SIZE(SIZE)
    ) u_instr_mem (
        .clk(clk), .rst_n(rst_n),
        .wr_en(wr_en),
        .waddr(program_pc),
        .wdata(wr_instr),
        .raddr1(pc[0]),
        .rdata1(instr[0]),
        .raddr2(pc[1]),
        .rdata2(instr[1])
    );


    //-----------------------------------------------------
    // Instruction Decoder (2 wide)
    // ----------------------------------------------------
    generate 
    for (genvar i = 0; i < 2; i++) begin : gen_dec
        instruction_decoder i_dec(
            .instr(instr[i]),
            .uop(uop_dec[i])
        );
    end
    endgenerate


    //-----------------------------------------------------
    // Micro-op Queue 
    // ----------------------------------------------------
    banked_fifo_seq #(     
        .WIDTH($bits(uop_s)),
        .DEPTH(4),   // depth of each bank
        .N(2),
        .SHOW_AHEAD(1'b1)
    ) uop_queue (
        .clk(clk), .rst_n(rst_n),
        .clr(flush),
        .wr_data(uop_dec),
        .wr_en(fetch_instr), 
        .wr_ok(uop_q_free),
    
        .rd_data(uop_out),
        .rd_en(dispatch_valid),
        .rd_ok(uop_q_valid)  
    );


    //-----------------------------------------------------
    // Fetch Logic 
    // ----------------------------------------------------
    always_comb begin : u_fetch_instr
        // stall or reach end of program, stop fetch  
        if (stall || (pc[0] >= program_pc) || (uop_q_free == 2'b00)) 
            fetch_instr = 2'b00;
        else if ((pc[1] >= program_pc) || ((uop_q_free != '1))) 
            fetch_instr = 2'b10;
        else 
            fetch_instr = 2'b11;
    end

    // output signals 
    assign completed = (program_pc == cmt_pc);
    assign cmt_pc_o  = cmt_pc;

endmodule


//-----------------------------------------------------------------------------
// A simple memory block to hold instructions 
// Assume the entire program is pre-loaded
// ----------------------------------------------------------------------------
module instr_mem #(
    parameter int SIZE = 256
)(
    input  logic clk, rst_n,
    input  logic wr_en,
    input  logic [31:0] waddr,
    input  logic [31:0] wdata,
    // Read port 1
    input  logic [31:0] raddr1,
    output logic [31:0] rdata1,
    // Read port 2
    input  logic [31:0] raddr2,
    output logic [31:0] rdata2
);

    // Memory array
    logic [31:0] mem [SIZE];

    // Synchronous write
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[waddr[31:2]] <= wdata;
    end

    // Asynchronous read ports
    assign rdata1 = mem[raddr1[31:2]];
    assign rdata2 = mem[raddr2[31:2]];


endmodule