package riscv_pkg;

    // Op type Section 
    typedef enum logic [2:0] {
        OP_NOP = 0,
        OP_LOGIC, 
        OP_MUL, 
        OP_SW, 
        OP_LW, 
        OP_B, 
        OP_U,
        OP_EXC
    } opcode_t;

    // ALU Control Section 
    typedef logic [3:0] aluop_t;
    localparam aluop_t ALU_PASS = 0;
    localparam aluop_t ALU_ADD  = 1;
    localparam aluop_t ALU_SUB  = 2;
    localparam aluop_t ALU_XOR  = 3;
    localparam aluop_t ALU_OR   = 4;
    localparam aluop_t ALU_AND  = 5;
    localparam aluop_t ALU_SLL  = 6;
    localparam aluop_t ALU_SRL  = 7;
    localparam aluop_t ALU_SRA  = 8;
    localparam aluop_t ALU_SLT  = 9;
    localparam aluop_t ALU_SLTU = 10;

    localparam aluop_t ALU_BEQ = 11;
    localparam aluop_t ALU_BNE = 12;
    localparam aluop_t ALU_BLT = 13;
    localparam aluop_t ALU_BGE = 14;

    // Multiply 
    localparam aluop_t ALU_MUL  = 1;
    localparam aluop_t ALU_MULH = 2;
    
    // Exception 
    localparam aluop_t ALU_EXC  = 15;
    
    parameter TAG_WIDTH = 7;

    typedef struct packed {
        // opcode_t     opcode;
        logic [3:0]  rs_mask;
        aluop_t      alu_ctrl; 
        logic [4:0]  rs1, rs2, rd; 
        logic        srcB;
        logic [31:0] imm;
    } uop_s;

    typedef struct packed {
        logic        valid;
        aluop_t      alu_ctrl;
        logic [31:0] operand_a;
        logic [31:0] operand_b;
        logic [TAG_WIDTH-1:0] dest_tag;
    } rs_fu_s; 

    typedef struct packed {
        logic        valid;
        logic [31:0] rs1;
        logic [31:0] rs2;
        logic [31:0] imm;
        logic [TAG_WIDTH-1:0] id_tag;
    } rs_st_s; 

    typedef struct packed {
        // logic valid;
        logic [4:0] rd;
        logic [TAG_WIDTH-1:0] rs1_tag;
        logic [TAG_WIDTH-1:0] rs2_tag;
        logic [TAG_WIDTH-1:0] rd_tag;
        logic [31:0] rs1_data;
        logic [31:0] rs2_data;
        aluop_t      alu_ctrl;
        logic [3:0]  rs_mask;
        logic [31:0] st_imm;
    } rename_s; 

    typedef struct packed {
        logic commit_valid;
        logic [31:0] data;
        logic [TAG_WIDTH-1:0] Rd;
        logic [TAG_WIDTH-1:0] PRd;
    } cmt_res_s;

    parameter RS_LOGIC_MASK = 4'b1000;
    parameter RS_MUL_MASK = 4'b0100;
    parameter RS_LD_MASK = 4'b0010;
    parameter RS_ST_MASK = 4'b0001;

endpackage : riscv_pkg



interface mem_if 
    import riscv_pkg:: *;
(
    input logic clk, rst_n
);
    
    // Load signals
    logic ld_ok; 
    logic ld_req;
    logic [31:0] ld_addr;
    // Store signals
    logic st_ok;
    logic st_req;
    logic st_cpl;
    logic [31:0] st_addr;
    logic [31:0] st_data;
    // Memory supply data
    logic mem_ready;
    logic [31:0] mem_data;

    // Functional Unit side
    modport fu (
        input  ld_ok,
        output ld_req,
        output ld_addr,
        
        input  st_ok,
        input  st_cpl,
        output st_req,
        output st_addr,
        output st_data,

        input  mem_ready,
        input  mem_data
    );

    // Memory/Cache side
    modport mem (
        input  ld_req,
        input  ld_addr,
        output ld_ok,

        input  st_req,
        input  st_addr,
        input  st_data,
        output st_ok,
        output st_cpl,

        output mem_ready,
        output mem_data
    );

endinterface