module regFile 
    import riscv_pkg ::*;
#(
    parameter PHY_REGS = 64
)(
    input  logic clk, rst_n,

    // From dispatch 
    input  logic [0:1] dispatch_valid,
    input  uop_s uop_in [2],

    // hazard unit 
    input  logic stall,    
    input  logic flush,
    output logic [0:1] rename_stalled, // how many new instr can the free list alloc 

    // To RS
    output rename_s ren_o [2],
    output logic    ren_valid [2],

    // commit/rollback from ROB 
    input  cmt_res_s cmt [2],

    sim_if.dut sim
);

    //-----------------------------------------------------
    // Input Buffer 
    //-----------------------------------------------------
    logic     valid [2];
    uop_s     uop   [2];
    logic [4:0] rs1 [2]; 
    logic [4:0] rs2 [2];
    logic [4:0] rd  [2]; 

    generate
    for(genvar i = 0; i < 2; i++) begin : genloop_input_buf
        always_ff @(posedge clk or negedge rst_n) begin : r_input_buf
            if (flush || !rst_n) begin // clr 
                uop  [i] <= '0;
                valid[i] <= 1'b0;
            end else if (!stall) begin // En 
                valid[i] <= dispatch_valid[i];
                if (dispatch_valid[i]) begin
                    uop  [i] <= uop_in[i];
                end 
            end
        end

        always_comb begin : u_alias
            rs1[i] = uop[i].rs1;
            rs2[i] = uop[i].rs2;
            rd [i] = uop[i].rd;
        end
    end
    endgenerate


    //-----------------------------------------------------
    // Internal Signals 
    //-----------------------------------------------------
    logic [0:1] rf_free_en, sb_free_en;
    logic [0:1] rf_alloc_en, sb_alloc_en;

    logic [TAG_WIDTH-2:0] allocReg [2];  // phy reg to allocate 
    logic [TAG_WIDTH-2:0] allocTag [2];  // instr that does not write to reg
    logic [TAG_WIDTH-2:0] pReg_s1 [2], pReg_s2[2];

    logic [0:1] instr_wr_to_rf;
    logic [PHY_REGS-1:0] data_present;
    
    always_ff @(posedge clk) begin : u_data_present_logic 
        if (!rst_n) begin
            data_present = '0;
        end
        // update present state 
        else begin
            if (cmt[0].commit_valid) data_present[cmt[0].PRd] = 1'b1;
            if (cmt[1].commit_valid) data_present[cmt[1].PRd] = 1'b1;
        end 
    end


    //-----------------------------------------------------
    // Register Alias Table
    //-----------------------------------------------------
    logic [TAG_WIDTH-2:0] cmt_rat [32];
    // Speculative register alias table 
    srat #(.WIDTH(TAG_WIDTH-1)) u_srat (
        .clk(clk), .rst_n(rst_n),
        .aReg_s1(rs1), .aReg_s2(rs2), 
        .pReg_s1(pReg_s1), .pReg_s2(pReg_s2), 
        // Write         
        .wr_en_i(rf_alloc_en),
        .wr_aReg(rd),
        .wr_pReg(allocReg),
        // Roll back signals 
        .restore(flush),
        .rat_restore(cmt_rat),
        .sim(sim)
    );

    // Architectural register alias table 
    logic [0:1] rf_cmt_en;
    assign rf_cmt_en[0] = cmt[0].commit_valid & ~cmt[0].PRd[TAG_WIDTH-1];
    assign rf_cmt_en[1] = cmt[1].commit_valid & ~cmt[1].PRd[TAG_WIDTH-1];
    arat #(.WIDTH(TAG_WIDTH-1)) u_arat (
        .clk(clk), .rst_n(rst_n),        
        .wr_en_i(rf_cmt_en),
        .wr_aReg_0(cmt[0].Rd), 
        .wr_aReg_1(cmt[1].Rd),
        .wr_pReg_0(cmt[0].PRd[TAG_WIDTH-2:0]),
        .wr_pReg_1(cmt[1].PRd[TAG_WIDTH-2:0]),
        .rat_restore(cmt_rat),
        .sim(sim)
    );


    //-----------------------------------------------------
    // Physical Registers
    //-----------------------------------------------------
    logic [31:0] rdata_s1 [2], rdata_s2 [2];
    prf #(.NUM_PHYS_REGS(PHY_REGS), .TAG_WIDTH(TAG_WIDTH-1)) u_prf(
        .clk(clk), .rst_n(rst_n),
        // write ports 
        .write_en0(rf_cmt_en[0]), .wAddr0(cmt[0].PRd[TAG_WIDTH-2:0]), .wData0(cmt[0].data),
        .write_en1(rf_cmt_en[1]), .wAddr1(cmt[1].PRd[TAG_WIDTH-2:0]), .wData1(cmt[1].data),
        // read ports
        .rAddr0(pReg_s1[0]), .rAddr1(pReg_s2[0]), 
        .rData0(rdata_s1[0]), .rData1(rdata_s2[0]),
        .rAddr2(pReg_s1[1]), .rAddr3(pReg_s2[1]),
        .rData2(rdata_s1[1]), .rData3(rdata_s2[1]),

        .sim(sim)
    );


    //-----------------------------------------------------
    // Free List
    //-----------------------------------------------------
    logic [0:1] fl_credit_rf, fl_credit_sb;
    always_comb begin : u_free_list_ctrl
        for (int i = 0; i < 2; i++) begin
            instr_wr_to_rf[i] = (rd[i] != '0);
            rf_alloc_en[i] = valid[i] & instr_wr_to_rf[i]; 
            sb_alloc_en[i] = valid[i] & ~instr_wr_to_rf[i];

            rf_free_en[i] = cmt[i].commit_valid & ~cmt[i].PRd[TAG_WIDTH-1];
            sb_free_en[i] = cmt[i].commit_valid & cmt[i].PRd[TAG_WIDTH-1];
            
        end
    end

    freelist #(
        .DEPTH(PHY_REGS/2),  .WIDTH(TAG_WIDTH-1)
    ) i_freeList_rf (
        .clk(clk), .rst_n(rst_n),
        .stall(stall),
        .alloc_en_i(rf_alloc_en),
        .alloc_tag_o(allocReg),
        .alloc_ok(fl_credit_rf),
        .free_en_i(rf_free_en),
        .flush(flush)  
    );

    freelist #(
        .DEPTH(8),  .WIDTH(TAG_WIDTH-1)
    ) i_freeList_sb (
        .clk(clk), .rst_n(rst_n),
        .stall(stall),
        .alloc_en_i(sb_alloc_en),
        .alloc_tag_o(allocTag),
        .alloc_ok(fl_credit_sb),
        .free_en_i(sb_free_en),
        .flush(flush)  
    );

    // Stall logic : no free entry to allocate 
    always_comb begin : u_rename_stall
        rename_stalled[0] =((rf_alloc_en[0] & ~fl_credit_rf[0]) | 
                            (sb_alloc_en[0] & ~fl_credit_sb[0]));   
        rename_stalled[1] =(((rf_alloc_en == '1) & ~fl_credit_rf[1]) | 
                            ((sb_alloc_en == '1) & ~fl_credit_sb[1]) | 
                            (rf_alloc_en[1] & ~fl_credit_rf[0]) | 
                            (sb_alloc_en[1] & ~fl_credit_sb[0]) | 
                            rename_stalled[0]);  
    end

    //-----------------------------------------------------
    // Output Logic  
    //-----------------------------------------------------
    logic cmt_bypass_s1[2], cmt_bypass_s2[2];
    always_comb begin : u_cmt_bypass
        for (int i = 0; i < 2; i++) begin
            cmt_bypass_s1[i] = (cmt[0].commit_valid && ({1'b0, pReg_s1[i]} == cmt[0].PRd)) | 
                                (cmt[1].commit_valid && ({1'b0, pReg_s1[i]} == cmt[1].PRd));
            cmt_bypass_s2[i] = (cmt[0].commit_valid && ({1'b0, pReg_s2[i]} == cmt[0].PRd)) | 
                                (cmt[1].commit_valid && ({1'b0, pReg_s2[i]} == cmt[1].PRd));
        end
    end
    // if data is ready or reads R0, mark tag 0 (RS takes data output)
    always_comb begin : u_output_mux 
        for (int i = 0; i < 2; i++) begin
            ren_o[i].rd = rd[i];
            ren_o[i].rs_mask = uop[i].rs_mask; 
            ren_o[i].alu_ctrl = uop[i].alu_ctrl; 
            
            if (instr_wr_to_rf[i])  ren_o[i].rd_tag = {1'b0, allocReg[i]};
            else    ren_o[i].rd_tag = {1'b1, allocTag[i]};
            
            ren_valid[i] = valid[i];
            ren_o[i].st_imm = uop[i].imm;     // Store imm

            // tag mux
            if (data_present[pReg_s1[i]] || (rs1[i] == '0) || cmt_bypass_s1[i]) 
                ren_o[i].rs1_tag = '0;
            else
                ren_o[i].rs1_tag = pReg_s1[i];
            if (data_present[pReg_s2[i]] || (rs2[i] == '0) || uop[i].srcB || cmt_bypass_s2[i]) 
                ren_o[i].rs2_tag = '0;
            else
                ren_o[i].rs2_tag = pReg_s2[i];

            // Data Mux 
            // if the register read is the same as the commit, take the commit data
            if(rs1[i] == '0) 
                ren_o[i].rs1_data = '0;
            else if(cmt_bypass_s1[i])
                ren_o[i].rs1_data = cmt[i].data[i];
            else
                ren_o[i].rs1_data = rdata_s1[i];

            if (uop[i].srcB == 1'b1)
                ren_o[i].rs2_data = uop[i].imm;
            else if(rs2[i] == '0)
                ren_o[i].rs2_data = '0; 
            else if(cmt_bypass_s2[i])
                ren_o[i].rs2_data = cmt[i].data;
            else 
                ren_o[i].rs2_data = rdata_s2[i];
        end

    end
endmodule


// ----------------------------------------------------------------------------
// Speculative RAT 
// ----------------------------------------------------------------------------
module srat #(
    parameter WIDTH = 6
)(
    input  logic clk, rst_n,

    // Request to read the current mapping (for source registers)
    input  var logic [4:0] aReg_s1[2], 
    input  var logic [4:0] aReg_s2[2],
    output var logic [WIDTH-1:0] pReg_s1[2],
    output var logic [WIDTH-1:0] pReg_s2[2],

    // Request to update the mapping (for the destination register rename)
    input  var logic [0:1] wr_en_i,
    input  var logic [4:0] wr_aReg[2], // archi register to rename 
    input  var logic [WIDTH-1:0] wr_pReg[2],

    // Renaming Roll Back
    input logic restore,
    input logic [WIDTH-1:0] rat_restore [32], 

    sim_if.dut sim
);

    logic [0:1] wr_en;
    always_comb begin : u_write_guard
        wr_en = wr_en_i;
        if (wr_aReg[0] == wr_aReg[1])
            wr_en[0] = 1'b0;
    end

    // The RAT: for each architectural register, store the current physical reg
    logic [WIDTH-1:0] aliasTable [32];

    // Read
    always_comb begin : u_read_logic
        pReg_s1[0] = aliasTable[aReg_s1[0]];
        pReg_s2[0] = aliasTable[aReg_s2[0]];
        pReg_s1[1] = aliasTable[aReg_s1[1]];
        pReg_s2[1] = aliasTable[aReg_s2[1]];
        
        // if the second instr' rs reads the first instr's rd 
        if (wr_en_i[0] && (aReg_s1[1] == wr_aReg[0]))  pReg_s1[1] = wr_pReg[0];
        if (wr_en_i[0] && (aReg_s2[1] == wr_aReg[0]))  pReg_s2[1] = wr_pReg[0];
    end

    // Write (update) new mapping
    // P0 (which always holds 0) is not taken here. 
    always_ff @(posedge clk) begin : u_write_logic 
        if (!rst_n) begin
            // Initialize the RAT to default 
            aliasTable <= '{default: '0}; 
        end 
        else if (restore) begin
            for (int i = 0; i < 32; i++) begin
                aliasTable[i] <= rat_restore[i];
            end
        end else begin
            // new renaming 
            if (wr_en[0]) begin
                aliasTable[wr_aReg[0]] <= wr_pReg[0];
            end
            if (wr_en[1]) begin
                aliasTable[wr_aReg[1]] <= wr_pReg[1];
            end
        end
    end

    assign sim.srat = aliasTable;
endmodule


// ----------------------------------------------------------------------------
// Architectural RAT (Committed RAT)
// ----------------------------------------------------------------------------
module arat #(
    parameter WIDTH = 6
)(
    input  logic clk, rst_n,
    // No read requests 

    // update the mapping on commit 
    input  logic [0:1] wr_en_i,
    input  logic [4:0] wr_aReg_0, wr_aReg_1,
    input  logic [WIDTH-1:0] wr_pReg_0, wr_pReg_1,

    // Committed RAT To Restore
    output logic [WIDTH-1:0] rat_restore [32], 

    sim_if.dut sim
);

    logic [0:1] wr_en;
    always_comb begin : u_write_guard
        wr_en = wr_en_i;
        if (wr_aReg_0 == wr_aReg_1)
            wr_en[0] = 1'b0;
    end

    // The RAT: for each architectural register, store the current physical reg
    logic [WIDTH-1:0] aliasTable [32];

    // Commit speculative mapping
        always_ff @(posedge clk) begin : u_write_logic 
        if (!rst_n) begin
            aliasTable <= '{default: '0}; 
        end else begin
            if (wr_en[0]) begin
                aliasTable[wr_aReg_0] <= wr_pReg_0;
            end
            if (wr_en[1]) begin
                aliasTable[wr_aReg_1] <= wr_pReg_1;
            end
        end
    end

    assign sim.arat = aliasTable;
    assign rat_restore = aliasTable;
endmodule





//-----------------------------------------------------------------------------
// Physcial register file
// 4 read and 2 write ports
//-----------------------------------------------------------------------------
module prf #(
    parameter NUM_PHYS_REGS = 64,
    parameter TAG_WIDTH = 6
)(
    input  logic clk, rst_n,

    // Write port
    input  logic write_en0, write_en1,
    input  logic [TAG_WIDTH-1:0] wAddr0, wAddr1,
    input  logic [31:0] wData0, wData1,

    // Read port
    input  logic [TAG_WIDTH-1:0] rAddr0, rAddr1, rAddr2, rAddr3,
    output logic [31:0] rData0, rData1, rData2, rData3,

    sim_if.dut sim
);
    logic [31:0] rf [0:NUM_PHYS_REGS-1]; 

    // 2 read ports
    assign rData0 = rf[rAddr0];
    assign rData1 = rf[rAddr1];
    assign rData2 = rf[rAddr2];
    assign rData3 = rf[rAddr3];

    // One write port
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rf <= '{default: '0};
        end
        else begin
            if (write_en0)  rf[wAddr0] <= wData0;
            if (write_en1)  rf[wAddr1] <= wData1;
        end
    end

    assign sim.prf = rf;

endmodule


// ----------------------------------------------------------------------------
// Dual banked freelist FIFO
// ----------------------------------------------------------------------------
module freelist #(
    parameter int WIDTH = 6,    // tag with
    parameter int DEPTH = 8     // depth of each bank
) (
    input  logic clk, rst_n,
    input  logic stall,

    input  logic [0:1] free_en_i, 
    
    output var logic [WIDTH-1:0] alloc_tag_o [2],
    input  logic [0:1] alloc_en_i,
    output logic [0:1] alloc_ok,

    input  logic flush
);
    logic [0:1] full, empty;

    localparam PTR_W = $clog2(DEPTH);
    logic [PTR_W:0] free_ptr  [2];
    logic [PTR_W:0] free_ptr_next  [2];
    logic [PTR_W:0] alloc_ptr [2]; 
    logic [PTR_W:0] alloc_ptr_next [2]; 

    logic [0:1] alloc_en;
    logic [0:1] free_en;

    // Input Wiring  
    // reorder pointer, swithch order if set 
    logic rop_alloc, rop_free; 
    always_comb begin : u_alloc_en_reorder
        if (^alloc_en_i)
            alloc_en = rop_alloc ? 2'b01 : 2'b10;  
        else 
            alloc_en = alloc_en_i;
    end

    always_comb begin : u_alloc_ok_reorder
        alloc_ok[0] = rop_alloc ? ~full[1] : ~full[0];
        alloc_ok[1] = rop_alloc ? ~full[0] : ~full[1];
    end

    always_comb begin : u_free_en_reorder
        if (^free_en_i)
            free_en = rop_free ? 2'b01 : 2'b10;  
        else 
            free_en = free_en_i;
    end

    // Reorder Pointer Update
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rop_alloc <= '0;
        end else if(~stall) begin 
            if (^alloc_en) rop_alloc <= ~rop_alloc;
        end
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rop_free  <='0;
        end else begin 
            if (^free_en)  rop_free  <= ~rop_free;
        end
    end

    // Allocate Pointer Update
    always_ff @(posedge clk) begin
        for (int i = 0; i < 2; i++) begin
            if (!rst_n) begin
                alloc_ptr[i] <= 0;
            end else if (~stall) begin
                alloc_ptr[i] <= alloc_ptr_next[i];
            end
        end
    end

    always_comb begin : u_alloc_ptr_next
        for (int i = 0; i < 2; i++) begin
            if (alloc_en[i]) begin
                alloc_ptr_next[i] = alloc_ptr[i] + 1;
            end else begin 
                alloc_ptr_next[i] = alloc_ptr[i];
            end
        end
    end

    // Free Pointer Update 
    always_ff @(posedge clk) begin
        for (int i = 0; i < 2; i++) begin
            if (!rst_n) begin
                free_ptr[i] <= 0;
            end else begin
                free_ptr[i] <= free_ptr_next[i];
            end
        end
    end

    always_comb begin : u_free_ptr_next
        for (int i = 0; i < 2; i++) begin
            if (flush)
                free_ptr_next[i] = alloc_ptr[i];
            else if (free_en[i] && !empty) 
                free_ptr_next[i] = free_ptr[i] + 1;
            else 
                free_ptr_next[i] = free_ptr[i];
        end
    end

    // Status signals 
    always_comb begin
        for (int i = 0; i < 2; i++) begin
            full[i]  =  ((free_ptr[i][PTR_W] != alloc_ptr[i][PTR_W]) && 
                        (free_ptr[i][PTR_W-1:0] == alloc_ptr[i][PTR_W-1:0]));
            empty[i] =  ((free_ptr[i][PTR_W] == alloc_ptr[i][PTR_W]) && 
                        (free_ptr[i][PTR_W-1:0] == alloc_ptr[i][PTR_W-1:0]));

        end
    end

    // output mux 
    logic [WIDTH-1:0] alloc_tag [2];
    
    always_comb begin : u_output_encoder
        for (int i = 0; i < 2; i++) begin
            alloc_tag[i] = (alloc_ptr[i] * 2) + i + 1;
        end
    end
    
    always_comb begin : u_output_mux
        alloc_tag_o[0] = alloc_tag[rop_alloc];
        if (alloc_en_i == 2'b11)
            alloc_tag_o[1] = alloc_tag[~rop_alloc];
        else 
            alloc_tag_o[1] = alloc_tag[rop_alloc];    
    end
endmodule 


