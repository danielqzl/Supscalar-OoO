
module memory_unit 
    import riscv_pkg::*;
#(
    parameter ID = 2,
    parameter LQ_SIZE = 4,
    parameter SQ_SIZE = 4,
    parameter LD_MASK = 4'b0010,
    parameter ST_MASK = 4'b0001
)(
    input  logic clk, rst_n,

    input  var rename_s ren [2],  // From rename 
    // <-> RS 
    output logic [0:1] ldq_full, stq_full,
    input  logic [0:1] ld_rs_accept, st_rs_accept,
    // From Address Unit 
    input  logic ld_valid, st_valid,
    input  logic [31:0] ld_addr, st_addr,
    input  logic [TAG_WIDTH-1:0] ld_tag, st_tag,
    input  logic [31:0] st_data,
    // Hazard unit 
    input  logic flush,
    input  logic stall,
    // Broadcast
    input  var cmt_res_s cmt [2],
    cdb_if.producer cdb,
    mem_if.fu mem
);    
    localparam LQ_IDX_W = $clog2(LQ_SIZE);
    localparam SQ_IDX_W = $clog2(SQ_SIZE);

    logic [TAG_WIDTH-1:0] ren_tag [2];

    // Search <-> STQ (SQ entries) 
    logic [31:0] sq_addr [SQ_SIZE];
    logic sq_valid [SQ_SIZE];
    logic sq_resolved[SQ_SIZE];

    // Search <-> LDQ
    logic search_en;
    logic [31:0] search_addr;
    logic [SQ_IDX_W-1:0] search_start_idx;
    logic search_done;
    logic search_match;
    // logic unresolved_match;
    logic [SQ_IDX_W-1:0] last_match_idx;

    // STQ <-> LDQ
    logic sq_empty;
    logic [SQ_IDX_W-1:0] sq_head, sq_tail;
    logic [SQ_IDX_W-1:0] sq_rd_idx;
    logic sq_rd_resolved;
    logic [31:0] sq_rd_data;

    always_comb begin
        ren_tag[0] = ren[0].rd_tag;
        ren_tag[1] = ren[1].rd_tag;
    end

    // ------------------------------------------------------------------------
    // Load Queue 
    // ------------------------------------------------------------------------
    LDQ #(
        .ID(ID),
        .SIZE(LQ_SIZE),
        .IDX_W(LQ_IDX_W),
        .SQ_IDX_W(SQ_IDX_W)
    ) u_load_queue (
        .clk(clk), .rst_n(rst_n), .flush(flush),
        // From dispatch 
        .accept(ld_rs_accept),
        .is_store(st_rs_accept),
        .rd_tag_in(ren_tag),
        // From Address unit 
        .addr_valid(ld_valid),
        .addr_tag(ld_tag),
        .addr_in(ld_addr),
        // From Store Queue 
        .sq_empty(sq_empty),
        .sq_head(sq_head),
        .sq_tail(sq_tail),
        .sq_rd_idx(sq_rd_idx),
        .sq_resolved(sq_rd_resolved),
        .sq_rd_data(sq_rd_data),
        // Between Store Queue Search 
        .search_en_o(search_en),  
        .search_addr(search_addr),
        .search_start_idx(search_start_idx),
        .search_done(search_done),
        .match(search_match), // .unresolved_match(unresolved_match),
        .last_match_idx(last_match_idx), 

        .full(ldq_full),
        .cdb(cdb),
        .mem(mem)
    );

    sq_search #(
        .IDX_W(SQ_IDX_W), .SIZE(SQ_SIZE) 
    ) u_sq_searcher (
        .clk(clk), .rst_n(rst_n),
        .flush(flush),
        // store queue entries 
        .addr(sq_addr),
        .valid(sq_valid),
        .resolved(sq_resolved),
        
        // Load Queue
        .en(search_en),
        .search_addr(search_addr),
        .start_idx(search_start_idx), .end_idx(sq_tail),
        .done(search_done),
        .match(search_match), // .unresolved_match(unresolved_match),
        .last_match_idx(last_match_idx)
    );

    STQ #( .SIZE(SQ_SIZE), .IDX_W(SQ_IDX_W)) u_store_queue (
        .clk(clk), .rst_n(rst_n),
        .flush(flush),
        .accept(st_rs_accept),
        .tag_in(ren_tag),
        // From Address unit 
        .addr_valid(st_valid),
        .addr_tag(st_tag),
        .addr_in(st_addr),
        .data_in(st_data),
        // Between Load Queue 
        .head_o(sq_head), .tail_o(sq_tail),
        .rd_idx(sq_rd_idx),
        .rd_resolved(sq_rd_resolved),
        .rd_data(sq_rd_data),
        .empty(sq_empty),
        // Entry Output  
        .addr_o(sq_addr),
        .valid_o(sq_valid),
        .ready_o(sq_resolved),

        .full(stq_full),
        .cmt(cmt),
        .cdb(cdb),
        .mem(mem)
    );

endmodule


module sq_search #(
    parameter IDX_W = 3,
    parameter SIZE = 7 
)(
    input  logic clk, rst_n, 
    input  logic flush,
    input  logic en,
    input  logic [31:0] search_addr,
    input  logic [31:0] addr [SIZE],
    input  logic valid [SIZE],
    input  logic resolved [SIZE],
    input  logic [IDX_W-1:0] start_idx, end_idx,
    output logic done,
    output logic match,
    // output logic unresolved_match,
    output logic [IDX_W-1:0] last_match_idx
);
    // Internal signals 
    logic [IDX_W-1:0] s_idx;  // search index 
    logic hit, cpl;
    assign cpl = (s_idx == end_idx);
    assign hit = (valid[s_idx] & (search_addr == addr[s_idx]));
    logic load, step; // datapath control 

    // Search Pointer 
    always_ff @(posedge clk) begin
        if (!rst_n)
            s_idx <= '0;
        else if (load) 
            s_idx <= start_idx;
        else if (step) 
            s_idx <= s_idx - 1;
    end

    always_ff @(posedge clk) begin
        if (match && done) last_match_idx <= s_idx; 
    end

    // Control Logic 
    typedef enum logic [1:0] {
        IDLE, CHECK, COMPARE
    } state_t;

    state_t state, next_state;

    always_ff @(posedge clk) begin
        if (!rst_n || flush) 
            state <= IDLE;
        else        
            state <= next_state;
    end

    always_comb begin : u_fsm_logic
        load = '0;
        step = '0;
        done = '0;
        match = '0;
        case (state)
            IDLE: begin
                if (en) begin
                    load = 1'b1;
                    next_state = CHECK;
                end else begin
                    next_state = IDLE;
                end
            end
            CHECK: begin
                if (~resolved[s_idx] && valid[s_idx]) begin
                    next_state = CHECK; // wait store queue to resolve 
                end else begin
                    step = 1'b1;
                    next_state = COMPARE;
                end 
            end
            COMPARE: begin
                if (hit) begin 
                    done = 1'b1;
                    match = 1'b1;
                    next_state = IDLE;
                end else if (cpl) begin
                    done = 1'b1;
                    match = 1'b0;
                    next_state = IDLE;
                end else begin
                    next_state = CHECK;
                end
            end
            default: next_state = IDLE;
        endcase
    end
endmodule




// ----------------------------------------------------------------------------
// Load Instruction Address Unit
// ----------------------------------------------------------------------------
module LD_FU
    import riscv_pkg:: *;
(
    input  logic clk, rst_n,
    // RS side signals  
    input  rs_fu_s alu_in,
    input  logic [0:1] queue_full,
    
    output logic free,
    output logic valid,
    output logic [TAG_WIDTH-1:0] rd_tag,
    output logic [31:0] addr
);

    always_ff @(posedge clk) begin
        valid  <= alu_in.valid & rst_n; 
        addr   <= alu_in.operand_a + alu_in.operand_b;
        rd_tag <= alu_in.dest_tag;
    end

    // assign free = ~queue_full[0];
    assign free = 1'b1;
endmodule


// ----------------------------------------------------------------------------
// Load Instruction Address Unit
// ----------------------------------------------------------------------------
module ST_FU
    import riscv_pkg:: *;
(
    input  logic clk, rst_n,
    // RS side signals  
    input  rs_fu_s alu_in,
    input  [31:0] imm,  
    input  logic [0:1] queue_full,
    
    output logic free,
    output logic valid,
    output logic [TAG_WIDTH-1:0] tag,
    output logic [31:0] addr,
    output logic [31:0] data
);

    always_ff @(posedge clk) begin
        valid <= alu_in.valid & rst_n; 
        addr  <= alu_in.operand_a + imm;
        data  <= alu_in.operand_b;
        tag   <= alu_in.dest_tag;
    end

    // assign free = ~queue_full[0];
    assign free = 1'b1;
    
endmodule
