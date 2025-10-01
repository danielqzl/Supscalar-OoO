module alu_logic 
    import riscv_pkg:: *;
#(
    parameter ID = 0
) (
    input  logic clk, rst_n,
    // RS side signals  
    input  rs_fu_s alu_in,
    output logic free,

    // cdb side interface
    cdb_if.producer cdb
);

    // internal signals 
    logic [31:0] a, b;
    logic valid_i;
    always_comb begin : u_alias_block
        a = alu_in.operand_a;
        b = alu_in.operand_b;
        valid_i = alu_in.valid;
    end
    // output signals 
    logic [31:0] result, result_reg; 
    logic [TAG_WIDTH-1:0] tag_out;
    logic excp, excp_out;
    logic valid_out;

    // status signals
    logic forward;

    always_comb begin : u_alu_logic
        excp = (alu_in.alu_ctrl == ALU_EXC);
        case (alu_in.alu_ctrl)
            ALU_ADD:  result = a + b;   
            ALU_SUB:  result = a - b; 
            ALU_AND:  result = a & b;   
            ALU_OR:   result = a | b;                 
            ALU_XOR:  result = a ^ b;              
            ALU_SLL:  result = a << b;
            ALU_SRL:  result = a >> b;
            ALU_SLT:  result = (a < b) ? 32'b1 : 32'b0; 
            ALU_SRA:  result = $signed(a) >>> (b[4:0]);  
            default:  result = 32'b0;
        endcase
    end

    // result register 
    always_ff @(posedge clk) begin : r_result_out
        if (!rst_n) begin
            result_reg <= '0;
            tag_out <= '0;
            excp_out <= '0; 
            valid_out <= '0;
        end else if (forward) begin
            valid_out <= valid_i;
            result_reg <= result;
            tag_out <= alu_in.dest_tag;
            excp_out <= excp;
        end
    end 


    // status register 
    assign forward = cdb.grant[ID] | ~valid_out;
    assign free = forward;

    always_comb begin : u_cdb_input
        cdb.tag_in[ID] = tag_out;
        cdb.data_in[ID] = result_reg;
        cdb.exception_in[ID] = excp_out;
        cdb.req[ID] = valid_out;
    end
    
endmodule : alu_logic


