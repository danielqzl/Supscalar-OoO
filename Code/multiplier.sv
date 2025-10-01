module sequential_multiplier #(
    parameter WIDTH = 32
) (
    input  logic clk, rst_n,
    input  logic [WIDTH-1:0] op_a, op_b,
    input  logic en,
    output logic ready,
    output logic [2*WIDTH-1:0] res
);
    logic load, add, shift, decrement;

    logic [WIDTH-1:0] M, Q; // multiplier, multiplicand 
    logic C; // carry 
    logic [WIDTH-1:0] A;    // accumulator
    logic [$clog2(WIDTH)-1:0] P; // counter 

    always_ff @ (posedge clk) begin
        if (load) begin
            P <= WIDTH;
            A <= 0;
            C <= 0;
            M <= op_a;
            Q <= op_b;
        end
        if (add) {C, A} <= A + M;
        if (shift) {C, A, Q} <= {C, A, Q} >> 1;
        if (decrement) P <= P-1;
    end

    assign res = {A, Q};

    // Control Logic 
    typedef enum logic [1:0] { S_IDLE, S_ADD, S_SHIFT } state_t;
    state_t [2: 0] state, next_state;
    always_ff @(posedge clk) begin
        if (!rst_n) 
            state <= S_IDLE;
        else        
            state <= next_state;
    end

    always_comb begin
        next_state = S_IDLE;
        load = '0;
        add = '0;
        shift = '0;
        decrement = '0;
        ready = '0;
        case (state)
            S_IDLE: begin
                ready = 1'b1;
                if (en) begin
                    load = 1'b1;
                    next_state = S_ADD;
                end else 
                    next_state = S_IDLE;
            end
            
            S_ADD: begin
                decrement = 1'b1;
                if (Q[0]) add = 1'b1;
                next_state = S_SHIFT;
            end

            S_SHIFT: begin
                shift = 1'b1;
                if (P == 0) 
                    next_state = S_IDLE;
                else 
                    next_state = S_ADD;
            end
            default: next_state = S_IDLE;
        endcase
    end

endmodule


module alu_mul 
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
    logic valid_out;

    // status signals
    logic start, load, mul_rdy, busy;


    logic [63:0] product;
    sequential_multiplier #( .WIDTH(32)) u_multiplier (
        .clk(clk), .rst_n(rst_n),
        .op_a(a), .op_b(b),
        .en(start),
        .ready(mul_rdy),
        .res(product)
    );


    always_comb begin : u_alu_mul
        case (alu_in.alu_ctrl)
            ALU_MUL:  result = product[31:0];   
            default:  result = 32'b0;
        endcase
    end

    // result register 
    always_ff @(posedge clk) begin : r_result_out
        if (!rst_n) begin
            result_reg <= '0;
            tag_out <= '0;
        end else if (load) begin
            result_reg <= result;
            tag_out <= alu_in.dest_tag;
        end
    end 


    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= '0;
        end else if (cdb.grant[ID]) begin
            valid_out <= '0;
        end else if (load) begin
            valid_out <= '1;
        end
    end

    // Control Logic 
    typedef enum logic [1:0] { S_IDLE, S_EXEC, S_LOAD } state_t;
    state_t [2: 0] state, next_state;
    always_ff @(posedge clk) begin
        if (!rst_n) 
            state <= S_IDLE;
        else        
            state <= next_state;
    end

    always_comb begin
        next_state = S_IDLE;
        load = 1'b0;
        free = 1'b0;
        start = 1'b0;
        case (state)
            S_IDLE: begin
                free = 1'b1;
                if (valid_i) begin
                    start = 1'b1;
                    next_state = S_EXEC;
                end else begin 
                    next_state = S_IDLE;
                end
            end
            S_EXEC: begin
                if (mul_rdy && valid_out == 1'b0) begin
                    next_state = S_LOAD;
                end else begin 
                    next_state = S_EXEC;
                end
            end
            S_LOAD: begin
                load = 1'b1;
                next_state = S_IDLE;
            end
            default: next_state = S_IDLE;
        endcase
    end

    always_comb begin : u_cdb_input
        cdb.tag_in[ID] = tag_out;
        cdb.data_in[ID] = result_reg;
        cdb.exception_in[ID] = 1'b0;
        cdb.req[ID] = valid_out;
    end
    
endmodule : alu_mul


