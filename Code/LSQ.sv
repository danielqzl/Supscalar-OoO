module LDQ 
    import riscv_pkg ::*;
#(
    parameter ID = 0,   // cdb ID
    parameter SIZE = 4, // must be 2^N
    parameter IDX_W = 3,
    parameter SQ_IDX_W = 3
) (
    input  logic clk, rst_n, flush,
    // From dispatch 
    input  logic [0:1] accept,
    input  logic [0:1] is_store,
    input  logic [TAG_WIDTH-1:0] rd_tag_in [2],
    // From Address unit 
    input  logic addr_valid,
    input  logic [TAG_WIDTH-1:0] addr_tag,
    input  logic [31:0] addr_in,
    // From Store Queue 
    input  logic sq_empty,
    input  logic [IDX_W-1:0] sq_head,
    input  logic [IDX_W-1:0] sq_tail,
    output logic [IDX_W-1:0] sq_rd_idx,
    input  logic sq_resolved,
    input  logic [31:0] sq_rd_data,
    // Between Store Queue Search 
    output logic search_en_o,
    output logic [31:0] search_addr,
    output logic [SQ_IDX_W-1:0] search_start_idx, 
    input  logic search_done,
    input  logic match, 
    input  logic [SQ_IDX_W-1:0] last_match_idx, 

    output logic [0:1] full,
    cdb_if.producer cdb,
    mem_if.fu mem
);
    // Queue Pointer 
    logic [IDX_W:0] head [2], tail; 
    logic [IDX_W-1:0] exe_ptr;
    logic empty;

    // Load Queue Entry
    logic valid [SIZE]; 
    logic ready [SIZE];
    logic [TAG_WIDTH-1:0] rd_tag [SIZE];
    logic [31:0]          addr   [SIZE];
    logic [SQ_IDX_W-1:0]  sq_idx [SIZE]; // store queue index at the time of enq
    logic no_older_st [SIZE];

    // load result (to CDB)
    logic [31:0] res_data;
    logic [TAG_WIDTH-1:0] res_rd;

    // CDB Buffer 
    logic cdb_can_req; // if the previous req has been granted  
    logic [31:0] cdb_data;
    logic cdb_valid, cdb_busy;
    logic [TAG_WIDTH-1:0] cdb_tag;

    // FSM Control signals 
    logic search_en;
    logic req_mem_rd, load_from_sq, load_from_mem;
    logic cdb_forward;

    // -------------------------------- ----------------------------------------
    // Enqueue New Entry  
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin : u_entry_enq    
        if (!rst_n | flush) begin
            valid <= '{default: '0};
            ready <= '{default: '0};
            rd_tag <= '{default: '0};
            no_older_st <= '{default: '0};
            sq_idx <= '{default: '0}; 
            addr   <= '{default: '0};
        end else begin
            if (accept != '0) begin
                valid[head[0][IDX_W-1:0]] <= 1'b1; 
                ready[head[0][IDX_W-1:0]] <= 1'b0;
                rd_tag[head[0][IDX_W-1:0]] <= (accept == 2'b01) ? rd_tag_in[1] : rd_tag_in[0];
                no_older_st[head[0][IDX_W-1:0]] <= 1'b0; 
                // store,load dispatched at the same cycle 
                sq_idx[head[0][IDX_W-1:0]] <= (is_store == 2'b10) ? sq_head + 1 : sq_head;
            end

            if (accept == 2'b11) begin
                valid[head[1][IDX_W-1:0]] <= 1'b1; 
                ready[head[1][IDX_W-1:0]] <= 1'b0;
                rd_tag[head[1][IDX_W-1:0]] <= rd_tag_in[1];
                no_older_st[head[1][IDX_W-1:0]] <= 1'b0; 
                // store,load dispatched at the same cycle 
                sq_idx[head[1][IDX_W-1:0]] <= sq_head;
            end
        end
    end

    // head Pointer
    assign head[1] = head[0] + 1'b1; 
    always_ff @(posedge clk) begin : r_head_ptr
        if (!rst_n || flush) begin
            head[0] <= '0; 
        end else if (accept == 2'b10 || accept == 2'b01) begin
            head[0] <= head[0] + 1;
        end else if (accept == 2'b11) begin
            head[0] <= head[0] + 2;
        end
    end

    // update resolved address 
    always_ff @(posedge clk) begin
        for (int i = 0; i < SIZE; i++) begin
            if (valid[i] && addr_valid && (rd_tag[i] == addr_tag)) begin
                addr[i] <= addr_in;
                ready[i] <= 1'b1; 
            end
        end
    end 

    // Search 
    always_ff @(posedge clk) begin : r_search_output
        if(!rst_n) begin // make simulation clean, not necessary 
            search_en_o <= 1'b0;
            search_addr <= '0;
            search_start_idx <= '0;
        end else if (search_en) begin
            search_en_o <= 1'b1;
            search_addr <= addr[exe_ptr];
            search_start_idx   <= sq_idx[exe_ptr];
        end else begin
            search_en_o <= 1'b0;
        end
    end


    // ------------------------------------------------------------------------
    // Execution 
    // ------------------------------------------------------------------------
    // Detect whether all older store instructions are executed 
    always_ff @(posedge clk) begin
        for (int i = 0; i < SIZE; i++) begin
            if (valid[i] && (sq_empty || sq_tail == sq_idx[i])) begin
                no_older_st[i] <= 1'b1;
            end
        end
    end 

    // send to memory for execution 
    always_ff @(posedge clk or negedge rst_n) begin
        if(req_mem_rd) begin
            mem.ld_req <= 1'b1;
            mem.ld_addr <= addr[exe_ptr]; 
        end else begin
            mem.ld_req <= 1'b0;
        end
    end

    // Load Result Datapath 
    assign sq_rd_idx = last_match_idx;
    always_ff @(posedge clk) begin : r_load_data
        if (!rst_n) begin
            res_data <= '0;
            res_rd   <= '0;
        end else if(load_from_sq) begin
            res_data <= sq_rd_data;
            res_rd   <= rd_tag[exe_ptr]; 
        end else if (load_from_mem) begin
            res_data <= mem.mem_data; 
        end
    end
    always_ff @(posedge clk) begin : r_load_rd_tag
        if (!rst_n || flush) begin
            res_rd <= '0;
        end else if(req_mem_rd || load_from_sq) begin
            res_rd <= rd_tag[exe_ptr]; 
        end
    end

    // execution pointer
    always_ff @(posedge clk) begin
        if (!rst_n || flush) begin
            exe_ptr <= '0;
        end else if (req_mem_rd || load_from_sq) begin
            valid [exe_ptr] <= 1'b0;
            exe_ptr <= exe_ptr + 1;
        end
    end

    // ------------------------------------------------------------------------
    // BroadCast
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin : r_cdb_buffer
        if (!rst_n) begin
            cdb_valid <= '0;
            cdb_data  <= '0;
            cdb_tag   <= '0;
            cdb_busy  <= '0;
        end else if (cdb_forward) begin
            cdb_busy  <= 1'b1;
            cdb_valid <= 1'b1;
            cdb_data  <= res_data;
            cdb_tag   <= res_rd;  
        end else if (cdb.grant[ID]) begin
            cdb_busy  <= 1'b0;
            cdb_valid <= 1'b0;
        end 
    end

    always_comb begin : u_cdb_output
        cdb.tag_in[ID] = cdb_tag;
        cdb.data_in[ID] = cdb_data;
        cdb.req[ID] = cdb_valid;
        cdb.exception_in[ID] = 1'b0;
    end

    always_comb begin : u_status_flag
        full[0] = ((head[0][IDX_W] != tail[IDX_W]) && (head[0][IDX_W-1:0] == tail[IDX_W-1:0]));
        full[1] = ((head[1][IDX_W] != tail[IDX_W]) && (head[1][IDX_W-1:0] == tail[IDX_W-1:0]));
        empty   = ((head[0][IDX_W] == tail[IDX_W]) && (head[0][IDX_W-1:0] == tail[IDX_W-1:0]));
    end

    // simplified case - only 1 memory request at a time 
    // confirmed execute ptr = tail ptr
    assign tail = exe_ptr;

    // ------------------------------------------------------------------------
    // FSM
    // ------------------------------------------------------------------------
    typedef enum logic [2:0] {
        IDLE, SEARCH, BYPASS, EXEC, WAIT_CDB, BROADCAST
    } state_t;

    state_t state, next_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush) 
            state <= IDLE;
        else        
            state <= next_state;
    end

    always_comb begin : u_fsm_logic
        search_en = 1'b0;
        req_mem_rd = 1'b0; 
        load_from_mem = 1'b0;
        load_from_sq = 1'b0;
        cdb_forward = 1'b0;
        case (state)
            IDLE: begin
                if (mem.ld_ok && valid[exe_ptr] && ready[exe_ptr]) begin
                    if (no_older_st[exe_ptr]) begin
                        req_mem_rd = 1'b1;
                        next_state = EXEC;
                    end else begin
                        search_en = 1'b1;
                        next_state = SEARCH;
                    end
                end else begin
                    next_state = IDLE;
                end
            end
            SEARCH: begin
                if (search_done && match) 
                    next_state = BYPASS;
                else if (search_done && !match) begin// no match in store queue 
                    req_mem_rd = 1'b1;
                    next_state = EXEC;
                end else 
                    next_state = SEARCH; 
            end
            EXEC: begin
                if (mem.mem_ready && cdb_busy) begin 
                    load_from_mem <= 1'b1;
                    next_state = WAIT_CDB;
                end else if (mem.mem_ready && !cdb_busy) begin
                    load_from_mem <= 1'b1;
                    next_state = BROADCAST;
                end else begin
                    next_state = EXEC;
                end
            end

            BYPASS: begin
                load_from_sq <= 1'b1;
                if (cdb_busy) 
                    next_state = WAIT_CDB;
                else 
                    next_state = BROADCAST;
            end

            WAIT_CDB: begin
                if (!cdb_busy) 
                    next_state = BROADCAST;
                else 
                    next_state = WAIT_CDB;
            end

            BROADCAST: begin
                cdb_forward = 1'b1;
                next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end
endmodule


// ----------------------------------------------------------------------------
// Store Queue 
// 2 accept/cycle, 2 commit/cycle, 1 execute/cycle 
// ---------------------------------------------------------------------------- 
module STQ 
    import riscv_pkg ::*;
#(
    parameter SIZE = 7, // must be 2^N-1
    parameter IDX_W = 3
) (
    input  logic clk, rst_n,
    input  logic flush,
    // From dispatch 
    input  logic [0:1] accept,
    input  logic [TAG_WIDTH-1:0] tag_in [2],
    // From Address unit 
    input  logic addr_valid,
    input  logic [TAG_WIDTH-1:0] addr_tag,
    input  logic [31:0] addr_in,
    input  logic [31:0] data_in,
    // Between Load Queue 
    output logic [IDX_W-1:0] head_o,
    output logic [IDX_W-1:0] tail_o,
    input  logic [IDX_W-1:0] rd_idx,
    output logic rd_resolved,
    output logic [31:0] rd_data,
    output logic empty,
    // Entry Output  
    output logic [31:0] addr_o [SIZE],
    output logic valid_o [SIZE],
    output logic ready_o [SIZE],

    output logic [0:1] full,
    input  var cmt_res_s cmt [2],
    cdb_if.producer cdb,
    mem_if.fu mem
);
    // Queue Pointer 
    logic [IDX_W:0] head [2], tail; 
    logic [IDX_W-1:0] exe_ptr;
    logic [IDX_W-1:0] cmt_ptr;


    // Store Queue Entry
    logic valid [SIZE]; 
    logic ready [SIZE];
    logic committed [SIZE]; 
    logic [TAG_WIDTH-1:0] tag [SIZE];
    logic [31:0] addr [SIZE];
    logic [31:0] data [SIZE];           


    // -------------------------------------------------------------------------
    // Enqueue New Entry  
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin : r_entry_enq
        if (!rst_n) begin
            valid <= '{default: '0};
            ready <= '{default: '0};
            committed <= '{default: '0};
            tag <= '{default: '0};
        end else if (flush) begin
            // flush speculative stores 
            for (int i = 0; i < SIZE; i++) begin
                if (!committed[i]) begin
                    valid[i] <= 1'b0; 
                end
            end
        end else begin
            if (accept != '0) begin
                valid[head[0][IDX_W-1:0]] <= 1'b1; 
                ready[head[0][IDX_W-1:0]] <= 1'b0;
                committed[head[0][IDX_W-1:0]] <= 1'b0;
                tag[head[0][IDX_W-1:0]] <= (accept == 2'b01) ? tag_in[1] : tag_in [0];
            end
            if (accept == 2'b11) begin
                valid[head[1][IDX_W-1:0]] <= 1'b1; 
                ready[head[1][IDX_W-1:0]] <= 1'b0;
                committed[head[1][IDX_W-1:0]] <= 1'b0;
                tag[head[1][IDX_W-1:0]] <= tag_in[1];
            end
        end
    end
    
    // head Pointer
    assign head[1] = head[0] + 1'b1; 
    always_ff @(posedge clk) begin : r_head_ptr
        if (!rst_n) begin
            head[0] <= '0; 
        end else if (accept == 2'b10 || accept == 2'b01) begin
            head[0] <= head[0] + 1;
        end else if (accept == 2'b11) begin
            head[0] <= head[0] + 2;
        end
    end

    // update resolved address
    always_ff @(posedge clk) begin : u_update
        for (int i = 0; i < SIZE; i++) begin
            if (valid[i] && addr_valid && (tag[i] == addr_tag)) begin
                addr[i] <= addr_in;
                data[i] <= data_in;
                ready[i] <= 1'b1; 
            end
        end
    end 

    // ----------------------------------------------------
    // Commit from ROB
    // ----------------------------------------------------
    logic [0:1] cmt_en;
    always_comb begin : u_cmt_en
        for (int i = 0; i < 2; i++) begin
            cmt_en[i] = ( valid[cmt_ptr[i]] && 
                    ((cmt[0].commit_valid && (tag[cmt_ptr[i]] == cmt[0].PRd)) || 
                    (cmt[1].commit_valid && (tag[cmt_ptr[i]] == cmt[1].PRd))));
        end
    end

    assign cmt_ptr[1] = cmt_ptr[0] + 1;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            cmt_ptr[0] <= '0;
        end else if (cmt_en == '1) begin
            cmt_ptr[0] <= cmt_ptr[0] + 2;
        end else if (^cmt_en) begin
            cmt_ptr[0] <= cmt_ptr[0] + 1;
        end else begin
            cmt_ptr[0] <= cmt_ptr[0];
        end
    end

    always_ff @(posedge clk) begin
        if (cmt_en[0]) committed[cmt_ptr[0]] <= 1'b1;
        if (cmt_en[1]) committed[cmt_ptr[1]] <= 1'b1;
    end

    // ----------------------------------------------------
    // Execution 
    // ----------------------------------------------------
    // send to memory for execution 
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            exe_ptr <= '0;
            mem.st_req <= '0;
        end else if (committed[exe_ptr] && ready[exe_ptr] && mem.st_ok && valid[exe_ptr]) begin
            exe_ptr <= exe_ptr + 1;
            mem.st_req <= 1'b1;
            mem.st_addr <= addr[exe_ptr]; 
            mem.st_data <= data[exe_ptr];
        end else begin
            exe_ptr <= exe_ptr;
            mem.st_req <= '0;
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n)
            tail <= '0; 
        else if (mem.st_cpl) begin 
            valid[tail[IDX_W-1:0]] <= '0;
            tail <= tail + 1;
        end
    end


    // ----------------------------------------------------
    // Output Signal 
    // ----------------------------------------------------
    always_comb begin : u_output_wire
        for (int i = 0; i < SIZE; i++) begin
            addr_o[i] = addr[i];
            valid_o[i] = valid[i];
            ready_o[i] = ready[i];
        end
    end

    always_comb begin : u_ldq_output
        head_o = head[0][IDX_W-1:0];
        tail_o = tail[IDX_W-1:0];
        rd_resolved = ready[rd_idx];
        rd_data = data[rd_idx];
    end

    always_comb begin : u_status_flag
        full[0] = ((head[0][IDX_W] != tail[IDX_W]) && (head[0][IDX_W-1:0] == tail[IDX_W-1:0]));
        full[1] = ((head[1][IDX_W] != tail[IDX_W]) && (head[1][IDX_W-1:0] == tail[IDX_W-1:0]));
        empty   = ((head[0][IDX_W] == tail[IDX_W]) && (head[0][IDX_W-1:0] == tail[IDX_W-1:0]));
    end

endmodule