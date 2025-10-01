module reservation_station 
    import riscv_pkg ::*;
#(
    parameter N_ENTRY = 2,
    parameter N_FU    = 1,
    parameter OP_MASK = 4'b000,
    parameter bit IS_ST = 1'b0
)(
    input  logic clk, rst_n,

    // Scheduler Interface
    output logic [0:1] rs_accept,

    // From rename 
    input  var logic    ren_valid [2],
    input  var rename_s ren [2],

    // Functional Unit
    input  var logic   fu_free [N_FU],  // Indicate if an FU can accept a new operation
    output var rs_fu_s fu_in [N_FU],
    output var logic [31:0] st_imm [N_FU],

    // Hazard unit 
    input  logic flush,
    input  logic stall,
    input  logic [0:1] stall_q, // only for load/store queue 
    output logic rs_stalled_full,

    // Broadcast
    input  var cmt_res_s cmt [2],
    cdb_if.consumer cdb
);
    // ---------------------------------
    //  Reservation Station Entries
    // ---------------------------------
    // For each entry [0..N_ENTRY-1], store:
    //   busy:        1 if this entry is in use
    //   aluop:       alu control signal 
    //   Qj, Qk:      If operand j/k is waiting for a tag; 0 if it’s ready
    //   Vj, Vk:      The actual operand values if Qj/Qk=0
    //   rdTag:       The tag for the destination (phy register) 
    logic [0:N_ENTRY-1]    busy;
    aluop_t                aluop  [N_ENTRY];
    logic [TAG_WIDTH-1:0]  Qj     [N_ENTRY];
    logic [TAG_WIDTH-1:0]  Qk     [N_ENTRY];
    logic [31:0]           Vj     [N_ENTRY];
    logic [31:0]           Vk     [N_ENTRY];
    logic [TAG_WIDTH-1:0]  rdTag  [N_ENTRY];



    // -----------------------------------------
    // Find a Free Entry for incoming issues 
    // -----------------------------------------
    logic [0:1] rs_free;
    logic [0:N_ENTRY-1] free_sel [2];
    find_two_set_bits #(
        .WIDTH(N_ENTRY)
    ) u_find_free_entry (
        .in(~busy),
        .found_one(rs_free[0]),
        .found_two(rs_free[1]),
        .onehot1(free_sel[0]),
        .onehot2(free_sel[1]),
        .idx1(),  .idx2()
    );
    
    // Whether should this RS accpet 
    always_comb begin : u_rs_accept
        logic [0:1] mask_match; 
        logic [0:1] rs_should_accept, rs_full;
    
        for (int i = 0; i < 2; i++) begin
            mask_match[i] = (ren[i].rs_mask == OP_MASK);
            rs_should_accept[i] = mask_match[i] & ren_valid[i];
            rs_full[i] = ~rs_free[i] | stall_q[i]; 
            rs_accept[i] = rs_should_accept[i] & ~rs_full[i] & ~stall;
        end

        rs_stalled_full = (rs_should_accept[0] & rs_full[0]) | 
                            (rs_should_accept[1] & rs_full[1]);
    end

    // -----------------------------------------
    // Which disptach port to accept 
    // -----------------------------------------
    logic [0:N_ENTRY-1] accept;
    logic [0:N_ENTRY-1] accept_sel;  // wire 

    always_comb begin : u_accept 
        for(int i = 0; i < N_ENTRY; i++) begin
            accept[i] = (rs_accept[0] & free_sel[0][i]) | 
                        (rs_accept[1] & free_sel[1][i]);
            accept_sel[i] = free_sel[1][i];
        end
    end


    // -------------------------------------------------
    // Pick an RS entry to execute
    // -------------------------------------------------
    logic [$clog2(N_ENTRY)-1:0] exec_idx [N_FU];
    logic [0:N_ENTRY-1] sel_for_exec; // used to change busy bit of RS entry
    logic [0:N_FU-1] candidate_found; 
    logic [0:N_FU-1] issue_en;

    logic [0:N_ENTRY-1] entry_ready;
    always_comb begin : u_entry_ready
        for (int i = 0; i < N_ENTRY; i++) begin
            if ((busy[i] == 1'b1) && (Qj[i] == '0) && (Qk[i] == '0)) 
                entry_ready[i] = 1'b1;
            else 
                entry_ready[i] = 1'b0;
        end
    end

    generate 
    if (N_FU == 2) begin : g_dual_FU
        logic [0:N_ENTRY-1] sel_for_exec_0, sel_for_exec_1;
        find_two_set_bits_rev #(
            .WIDTH(N_ENTRY)
        ) u_exec_sel (
            .in(entry_ready),
            .found_one(candidate_found[0]),
            .onehot1(sel_for_exec_0),
            .idx1(exec_idx[0]),
            .found_two(candidate_found[1]),  
            .onehot2(sel_for_exec_1),  
            .idx2(exec_idx[1])
        );
        // onehot will be zero if not found so we can safely OR the output 
        assign sel_for_exec = sel_for_exec_0 | sel_for_exec_1;
    end 
    else begin : g_single_FU
        find_two_set_bits_rev #(
            .WIDTH(N_ENTRY)
        ) u_exec_sel (
            .in(entry_ready),
            .found_one(candidate_found[0]),
            .onehot1(sel_for_exec),
            .idx1(exec_idx[0]),
            .found_two(),  
            .onehot2(),  
            .idx2()  // not used 
        );
    end 
    endgenerate

    // Issue to FU
    always_comb begin : u_issue
        for (int i = 0; i < N_FU; i++) begin
            issue_en[i] = candidate_found[i] & fu_free[i];
        end
    end

    // -------------------------------------------------
    // Snoop on broadcast 
    // -------------------------------------------------
    logic retrieve_opA [N_ENTRY];
    logic retrieve_opB [N_ENTRY];
    logic [31:0] opA_data [N_ENTRY];
    logic [31:0] opB_data [N_ENTRY];
    generate
        for (genvar i = 0; i < N_ENTRY; i++) begin : g_snooper
            rs_snooper entry_snooper_j (
                .tag(Qj[i]),
                .hit(retrieve_opA[i]),
                .data(opA_data[i]),
                .cmt(cmt), .cdb(cdb)
            );
            rs_snooper entry_snooper_k (
                .tag(Qk[i]),
                .hit(retrieve_opB[i]),
                .data(opB_data[i]),
                .cmt(cmt), .cdb(cdb)
            );
        end
    endgenerate


    // ------------------------------------------------------------------------
    // Write Logic For Busy
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        for (int i = 0; i < N_ENTRY; i++) begin
            if (!rst_n || flush) begin
                busy[i] <= 1'b0;
            end 
            else if (accept[i]) begin // wr enable 
                busy[i] <= 1'b1;
            end
            // to be sent to FU for execution
            else if (|issue_en && sel_for_exec[i]) begin 
                busy[i] <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Write Logic For RS Entry 
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        for (int i = 0; i < N_ENTRY; i++) begin
            if (!rst_n) begin
                // Clear all for simulation cleaness. 
                aluop[i]  <= ALU_PASS; 
                rdTag[i]  <= '0;
                Qj[i]     <= '0;
                Qk[i]     <= '0;
                Vj[i]     <= '0;
                Vk[i]     <= '0;
            end 
            else if (accept[i]) begin // accept new instr
                aluop[i]  <= ren[accept_sel[i]].alu_ctrl;
                rdTag[i]  <= ren[accept_sel[i]].rd_tag;
                Qj[i] <= ren[accept_sel[i]].rs1_tag;
                Qk[i] <= ren[accept_sel[i]].rs2_tag;
                Vj[i] <= ren[accept_sel[i]].rs1_data;
                Vk[i] <= ren[accept_sel[i]].rs2_data;
            end else begin 
                if (busy[i] && retrieve_opA[i]) begin // wr enable 
                    Qj[i] <= '0;
                    Vj[i] <= opA_data[i];
                end
                if (busy[i] && retrieve_opB[i]) begin // wr enable 
                    Qk[i] <= '0;
                    Vk[i] <= opB_data[i];
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Issue to FU
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin : r_fu_in
        for (int i = 0; i < N_FU; i++) begin
            if (!rst_n) begin
                fu_in[i] <= '0;
            end else if (issue_en[i]) begin
                fu_in[i].valid     <= 1'b1;
                fu_in[i].alu_ctrl  <= aluop[exec_idx[i]];
                fu_in[i].operand_a <= Vj[exec_idx[i]];
                fu_in[i].operand_b <= Vk[exec_idx[i]];
                fu_in[i].dest_tag  <= rdTag[exec_idx[i]];
            end else begin
                fu_in[i].valid <= 1'b0;
            end
        end
    end

    // -----------------------------------------
    // Extra entry for Store Queue 
    // -----------------------------------------
    generate 
    if (IS_ST) begin
        logic [31:0] Imm [N_ENTRY];
        always_ff @(posedge clk) begin
            for (int i = 0; i < N_ENTRY; i++) begin
                if (!rst_n) begin
                    Imm[i] <= '0;
                end else if (accept[i]) begin // accept new instr
                    Imm[i]  <= ren[accept_sel[i]].st_imm;
                end
            end
            for (int i = 0; i < N_FU; i++) begin
                if (issue_en[i]) begin
                    st_imm[i]  <= Imm[exec_idx[i]];
                end
            end
        end
    end
    endgenerate
endmodule


/*
//-----------------------------------------------------------------------------
// Buffer to store in-flight instructions between rename and RS
// Need to snoop on CDB to retrieve broadcasted operand values
//-----------------------------------------------------------------------------
module uop_buffer 
    import riscv_pkg ::*;
(
    input  logic clk, rst_n,
    input  logic wr_en,

    input  logic    valid_i,
    input  rename_s ren_i,
    output logic    valid_o,
    output rename_s ren_o,

    input  logic flush,

    // result bus to snoop 
    input  var cmt_res_s cmt [2],
    cdb_if.consumer  cdb
);

    logic    valid;
    rename_s ren; //reg 

    // Watch cdb and commit result for operand values
    logic accept_opA, accept_opB;
    logic [31:0] opA_data, opB_data;
    rs_snooper u_opA_snoop (
        .tag(ren.rs1_tag),
        .hit(accept_opA),
        .data(opA_data),
        .cmt(cmt),
        .cdb(cdb)
    );
    rs_snooper u_opB_snoop (
        .tag(ren.rs2_tag),
        .hit(accept_opB),
        .data(opB_data),
        .cmt(cmt),
        .cdb(cdb)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || flush) begin
            valid <= '0;
            ren <= '0;
        end else begin
            if (wr_en) begin 
                valid <= valid_i;
                ren <= ren_i;
            end
            if (accept_opA)  ren.rs1_data <= opA_data;
            if (accept_opB)  ren.rs2_data <= opB_data;
        end
    end

    always_comb begin : u_output 
        valid_o  = valid;
        ren_o = ren;
    end

endmodule
*/


//-----------------------------------------------------------------------------
// A comb module that snoops on cdb and commit interface 
// ----------------------------------------------------------------------------
module rs_snooper
    import riscv_pkg ::*;
(
    input  logic [TAG_WIDTH-1:0] tag,
    output logic hit, 
    output logic [31:0] data,

    input  var cmt_res_s cmt [2],
    cdb_if.consumer cdb
);
    logic [3:0] match;
    logic [1:0] sel;
    
    always_comb begin : u_tag_compare
        match[0] = (cdb.valid[0] && (tag == cdb.tag[0]));
        match[1] = (cdb.valid[1] && (tag == cdb.tag[1]));
        match[2] = (cmt[0].commit_valid && (tag == cmt[0].PRd));
        match[3] = (cmt[1].commit_valid && (tag == cmt[1].PRd));
        hit = |match;
    end 

    always_comb begin : u_sel 
        unique0 case (1'b1)
            match[0]: sel = 2'b00;
            match[1]: sel = 2'b01;
            match[2]: sel = 2'b10;
            match[3]: sel = 2'b11;
        endcase
    end

    always_comb begin : u_data_mux
        case(sel)
            2'b00: data = cdb.data[0];
            2'b01: data = cdb.data[1];
            2'b10: data = cmt[0].data;
            2'b11: data = cmt[1].data;
            default: data = 32'bx;
        endcase
    end

endmodule
