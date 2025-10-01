module instruction_decoder
    import riscv_pkg:: *;
(
    input  logic [31:0] instr,
    output uop_s uop
);

    logic [6:0] op;
    logic [2:0] funct3;
    logic [6:0] funct7;
    opcode_t    opcode;

    // RISC-V decode logic
    always_comb begin : maindec
        op      = instr[6:0];
        funct3  = instr[14:12];
        funct7  = instr[31:25];
        uop.rd  = instr[11:7];    
        uop.rs1 = instr[19:15];
        uop.rs2 = instr[24:20];

        case(op)
            7'b0000011: begin // load type
                opcode = OP_LW;
                uop.srcB = 1'b1; 
                uop.rs2  = '0; 
                uop.imm  = {{20{instr[31]}}, instr[31:20]};
            end
            7'b0100011: begin // S type
                opcode = OP_SW;
                uop.srcB = 1'b0;
                uop.rd = '0;
                uop.imm  = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            end
            7'b0110011: begin // R-type
                uop.srcB = 1'b0; 
                uop.imm  = '0; 
                case(funct7)
                    7'h00: opcode = OP_LOGIC;
                    7'h20: opcode = OP_LOGIC;
                    7'h01: opcode = OP_MUL;
                    default: opcode = OP_LOGIC;
                endcase
            end
            7'b0010011: begin // I-type
                uop.srcB = 1'b1; 
                uop.rs2  = '0; 
                uop.imm  = {{20{instr[31]}}, instr[31:20]};
                if (funct3 == 3'h1 || funct3 == 3'h5) 
                    uop.imm  = {{27{1'b0}}, instr[24:20]};
                case(funct7)
                    7'h00: opcode = OP_LOGIC;
                    7'h20: opcode = OP_LOGIC;
                    default: opcode = OP_LOGIC;
                endcase
            end
            7'b1100011: begin  // B type
                uop.rd = '0;
                uop.srcB = 1'b0;
                uop.imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
                opcode = OP_B;
            end
            7'b1110011: begin // Exception 
                uop.rd = '0;
                uop.srcB = 1'b1;
                uop.imm  = {{20{instr[31]}}, instr[31:20]};
                opcode = OP_EXC;
            end
            default: begin
                opcode = OP_LOGIC;
                uop.srcB = 1'b0;
                uop.imm  = '0; 
            end
        endcase
    end : maindec

    always_comb begin : alu_dec
        case(opcode)
            OP_LW:  uop.alu_ctrl = ALU_ADD; 
            OP_SW:  uop.alu_ctrl = ALU_ADD; 
            OP_B:   uop.alu_ctrl = ALU_SUB; 
            OP_U:   uop.alu_ctrl = ALU_PASS; // pass through SrcB 
            OP_EXC: uop.alu_ctrl = ALU_EXC;
            OP_LOGIC: begin
                case(funct3) // R-type or I-type ALU
                    3'b000: if (funct7[5] == 1'b1 && op == 7'b0110011 ) 
                                uop.alu_ctrl = ALU_SUB;
                            else        
                                uop.alu_ctrl = ALU_ADD;
                    3'b001:     uop.alu_ctrl = ALU_SLL; 
                    3'b010:     uop.alu_ctrl = ALU_SLT;
                    3'b011:     uop.alu_ctrl = ALU_SLTU;
                    3'b100:     uop.alu_ctrl = ALU_XOR;
                    3'b101: if (funct7[5] == 1'b1)
                                uop.alu_ctrl = ALU_SRA; 
                            else           
                                uop.alu_ctrl = ALU_SRL;  
                    3'b110:     uop.alu_ctrl = ALU_OR; 
                    3'b111:     uop.alu_ctrl = ALU_AND; 
                    default:    uop.alu_ctrl = ALU_ADD; 
                endcase
            end
            OP_MUL: begin
                case (funct3)
                    3'b000:     uop.alu_ctrl = ALU_MUL;
                    3'b001:     uop.alu_ctrl = ALU_MULH;
                    default:    uop.alu_ctrl = ALU_MUL;
                endcase
            end
        endcase
    end : alu_dec

    scheduler_decoder_int u_rs_mask_dec (
        .opcode(opcode),
        .rs_accept_mask (uop.rs_mask)
    );

endmodule



//-----------------------------------------------------------------------------
// Dispatch Opcode Decoder 
// Hard coded 
// Encodes which RS can take what type of instructions 
// ----------------------------------------------------------------------------
module scheduler_decoder_int
    import riscv_pkg::*;
(
    input  opcode_t    opcode,
    output logic [3:0] rs_accept_mask 
); 

    always_comb begin
        case(opcode)
            // logic_mul_load/store
            OP_LOGIC:   rs_accept_mask  = 4'b1_0_00;
            OP_MUL:     rs_accept_mask  = 4'b0_1_00;
            OP_LW:      rs_accept_mask  = 4'b0_0_10;  
            OP_SW:      rs_accept_mask  = 4'b0_0_01;   
            OP_B:       rs_accept_mask  = 4'b1_0_00;
            OP_U:       rs_accept_mask  = 4'b1_0_00; 
            OP_EXC:     rs_accept_mask  = 4'b1_0_00;
            default:    rs_accept_mask  = 4'b0_0_00; 
        endcase    
    end    
endmodule