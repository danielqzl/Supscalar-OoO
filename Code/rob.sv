module rob 
    import riscv_pkg ::*;
#(
    parameter SIZE = 8
) (
    input  logic clk, rst_n,

    // Enqueue interface
    input  var logic    ren_valid_i [2],
    input  var rename_s ren_i [2], 
    
    input  logic stall,

    // Status signals 
    output logic [0:1] rob_free,
    //output logic in_rollback,
    //output logic rollback_done,
    
    // Commit interface
    output logic exc_flush,
    output cmt_res_s cmt [2],
    cdb_if.consumer cdb
);
    //-----------------------------------------------------
    // Internal Signals and Wires
    //-----------------------------------------------------
    logic [0:1] commit_en;


    logic [4:0] Rd   [2]; 
    logic [TAG_WIDTH-1:0] PRd  [2];
    logic [31:0] data [2];


    //---------------------------------
    // Input pointer logic 
    //---------------------------------
    logic [0:1] enq_en;
    logic enq_ptr_0, enq_ptr_1;
    logic enq_ptr_next;

    always_comb begin : u_input_ptr
        enq_en[0] = ren_valid_i[0] & ~stall;
        enq_en[1] = ren_valid_i[1] & ~stall;

        enq_ptr_1 = enq_ptr_0 + 1;
        if (enq_en == 2'b10) begin
            enq_ptr_next = enq_ptr_0 + 1;
        end else begin
            enq_ptr_next = enq_ptr_0;
        end
    end

    always_ff @(posedge clk) begin : r_enq_pointer
        if (!rst_n)
            enq_ptr_0 <= '0;
        else
            enq_ptr_0 <= enq_ptr_next;
    end


    //---------------------------------
    // ROB Bank
    //---------------------------------
    logic [0:1] bank_full, bank_empty;
    logic [0:1] completed, exception, entry_valid;
    logic [0:1] bank_commit_en;
    generate 
    for (genvar i = 0; i < 2; i++) begin : g_bank
        rob_bank #( 
            .SIZE(SIZE), .TAG_WIDTH(TAG_WIDTH), .ID(i)
        ) u_bank (
            .clk(clk), .rst_n(rst_n),
            .enq_en_in(enq_en),  
            .ren(ren_i),
            .enq_ptr_0(enq_ptr_0),
            .enq_ptr_1(enq_ptr_1),
                
            .commit_en(commit_en[i]), 
            .flush(exc_flush),
            .empty(bank_empty[i]),
            .full(bank_full[i]),
            .valid_o(entry_valid[i]),
            .Rd_o(Rd[i]), 
            .PRd_o(PRd[i]),
            .data_o(data[i]),
            .cpl_o(completed[i]),
            .exc_o(exception[i]),

            .cdb(cdb)
        );
    end
    endgenerate


    //---------------------------------
    // Commit logic 
    //---------------------------------
    logic retire_ptr, retire_ptr_next;
    always_comb begin : u_commit_en
        // the current instr has completed and not an exception
        // if the other bank commit first, also check that bank can commit  
        if (retire_ptr == 1'b0) begin
            commit_en[0] = completed[0] & ~exception[0];
            commit_en[1] = completed[1] & ~exception[1] & commit_en[0];
        end else begin 
            commit_en[1] = completed[1] & ~exception[1];
            commit_en[0] = completed[0] & ~exception[0] & commit_en[1];
        end 
    end

    always_comb begin : u_retire_ptr_next
        // if only one bank committed, rotate priority pointer 
        if (^commit_en) 
            retire_ptr_next = retire_ptr + 1;
        else
            retire_ptr_next = retire_ptr;
    end

    always_ff @(posedge clk or negedge rst_n) begin : u_commit_pointer_reg
        if (!rst_n || exc_flush) 
            retire_ptr <= '0;
        else 
            retire_ptr <= retire_ptr_next;
    end
    

    //---------------------------------
    // Excepion logic 
    //---------------------------------
    logic [0:1] start_rb;
    always_comb begin : u_exc_flush
        if (retire_ptr == 1'b0) begin
            start_rb[0] = completed[0] & exception[0];
            start_rb[1] = completed[1] & exception[1] & completed[0];
        end else begin 
            start_rb[0] = completed[0] & exception[0] & completed[1];
            start_rb[1] = completed[1] & exception[1];
        end 
    end

    always_ff @(posedge clk) begin
        // wait commit to finish, then trigger a flush pulse  
        if (|start_rb && (commit_en == '0))
            exc_flush <= 1'b1;
        else 
            exc_flush <= 1'b0;
    end
    
    //---------------------------------
    // Output Buffer 
    //---------------------------------
    logic [TAG_WIDTH-1:0] Rd_o   [2]; 
    logic [TAG_WIDTH-1:0] PRd_o  [2];
    logic [31:0] data_o [2];

    logic commit_valid_o [2];
    logic rollback_o [2];
    
    always_ff @(posedge clk) begin : r_retire_buf
        for (int i = 0; i < 2; i++) begin
            commit_valid_o[i] <= commit_en[i];
            //rollback_o[i] <= rollback_en[i];
            if (commit_en[i]) begin
                Rd_o[i]   <= Rd[i];
                PRd_o[i]  <= PRd[i];
                data_o[i] <= data[i];
            end
        end
    end

    always_comb begin : u_output_signal
        for (int i = 0; i < 2; i++) begin
            cmt[i].commit_valid = commit_valid_o[i];
            //cmt[i].rollback_en  = rollback_o[i]; 
            cmt[i].Rd   = Rd_o[i];
            cmt[i].PRd  = PRd_o[i];
            cmt[i].data = data_o[i];
        end
    end

    //---------------------------------
    // Signals to scheduler 
    //---------------------------------
    always_comb begin : u_rob_free
        //rob_free[0] = !in_rollback & (!bank_full[enq_ptr_0]);
        //rob_free[1] = !in_rollback & (!bank_full[enq_ptr_1]);
        rob_free[0] = ~bank_full[enq_ptr_0];
        rob_free[1] = ~bank_full[enq_ptr_1];
    end


endmodule


module rob_bank 
    import riscv_pkg ::*;
#(
    parameter int SIZE = 8, // must be a power of 2
    parameter int TAG_WIDTH = 6,
    parameter int ID = 0
) (
    input  logic clk, rst_n,

    // Control signal from top level
    input  logic [0:1] enq_en_in,   //accept new instr
    input  var rename_s ren [2],

    input  logic enq_ptr_0, enq_ptr_1,

    input  logic commit_en, 
    input  logic flush,
    output logic valid_o,
    output logic [4:0] Rd_o, 
    output logic [TAG_WIDTH-1:0] PRd_o,
    output logic [31:0] data_o,
    output logic empty,
    output logic full,
    output logic cpl_o,
    output logic exc_o,

    cdb_if.consumer cdb
);

    localparam PTR_W = $clog2(SIZE);

    logic deq_en;
    assign deq_en = commit_en;
    // ------------------------------
    // Input selection mux 
    // ------------------------------
    logic enq_en;  
    logic [4:0] Rd_i;
    logic [TAG_WIDTH-1:0] PRd_i; 
    logic [3:0] op_mask_i;
    logic accept_path_0, accept_path_1;
    logic enq_sel;
    always_comb begin : u_enq_mux
        accept_path_0 = enq_en_in[0] && (enq_ptr_0 == ID);
        accept_path_1 = enq_en_in[1] && (enq_ptr_1 == ID);
        enq_en = accept_path_0 | accept_path_1;
        enq_sel = accept_path_1;
        // 2-1 data mux
        Rd_i = ren[enq_sel].rd;
        PRd_i = ren[enq_sel].rd_tag;
        op_mask_i = ren[enq_sel].rs_mask;
    end

    // ------------------------------
    // ROB Entry 
    // ------------------------------
    logic valid [SIZE];
    // logic [2:0] op [SIZE];
    logic [4:0] Rd [SIZE];
    logic [TAG_WIDTH-1:0] PRd  [SIZE];  // Rd Tag
    logic [31:0] data [SIZE];
    logic completed [SIZE];
    logic exception [SIZE];

    logic [PTR_W:0] r_ptr, w_ptr;
    logic in_rollback;


    // ------------------------------
    // take results from CDB
    // ------------------------------
    always_comb begin : u_cdb_snooper
        for (int j = 0; j < SIZE; j++) begin
            if (!rst_n) begin
                data      [j] <= '0;
                exception [j] <= '0;
            end 
            else if (valid[j] && cdb.valid[0] && PRd[j] == cdb.tag[0]) begin
                data      [j] <= cdb.data[0];
                exception [j] <= cdb.exception[0];
                completed [j] <= 1'b1;
            end
            else if (valid[j] && cdb.valid[1] && PRd[j] == cdb.tag[1]) begin
                data      [j] <= cdb.data[1];
                exception [j] <= cdb.exception[1];
                completed [j] <= 1'b1;
            end  
        end
    end

    // enqueue path 
    always_ff @(posedge clk) begin :r_enq
        if (!rst_n || flush) begin  // clear all for simulation clarity  
            valid <= '{default: '0};
            completed <= '{default: '0};
            Rd  <= '{default: '0};
            PRd <= '{default: '0};
        end else if(enq_en && !full) begin
            valid[w_ptr[PTR_W-1:0]] <= 1'b1;
            completed[w_ptr[PTR_W-1:0]] <= (op_mask_i == RS_ST_MASK);
            Rd  [w_ptr[PTR_W-1:0]] <= Rd_i;
            PRd [w_ptr[PTR_W-1:0]] <= PRd_i;
        end else if(deq_en && !empty) begin
            valid[r_ptr[PTR_W-1:0]] <= 1'b0;
            completed[r_ptr[PTR_W-1:0]] <= 1'b0;
        end
    end


    always_comb begin : u_deq_signal 
        valid_o = valid[r_ptr[PTR_W-1:0]];
        data_o = data[r_ptr[PTR_W-1:0]];
        Rd_o   = Rd[r_ptr[PTR_W-1:0]]; 
        PRd_o  = PRd[r_ptr[PTR_W-1:0]]; 
        cpl_o  = completed[r_ptr[PTR_W-1:0]];
        exc_o  = exception[r_ptr[PTR_W-1:0]];
    end
    
    // pointer logic 
    always_ff @(posedge clk or negedge rst_n) begin : r_pointer
        if (!rst_n || flush) begin
            w_ptr  <= '0;
            r_ptr  <= '0;
        end else begin 
            if (enq_en && !full) begin
                w_ptr <= w_ptr + 1;
            end
            if (deq_en && !empty) begin
                r_ptr <= r_ptr + 1; 
            end
        end
    end

    assign full = ((w_ptr[PTR_W] != r_ptr[PTR_W]) & (w_ptr[PTR_W-1:0] == r_ptr[PTR_W-1:0]));
    assign empty =((w_ptr[PTR_W] == r_ptr[PTR_W]) & (w_ptr[PTR_W-1:0] == r_ptr[PTR_W-1:0]));

endmodule
