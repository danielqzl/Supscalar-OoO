`timescale 1ns / 1ps

interface sim_if #(
    parameter WIDTH = 6,
    parameter NUM_PHY_REGS = 64
);
    logic [WIDTH-1:0] srat [32];
    logic [WIDTH-1:0] arat [32];
    logic [31:0] prf [NUM_PHY_REGS];

    modport dut (
        output srat,
        output arat,
        output prf
    );

    modport tb(
        input  srat,
        input  arat,
        input  prf
    );

endinterface


module tb_tomasulo ();
    import riscv_pkg ::*;
    parameter int DMEM_SIZE = 32;
    parameter int IMEM_SIZE = 32;
    parameter int MEM_RD_CYCLE_RANGE = 4;
    parameter int NUM_PHY_REGS = 64;

    // global simualtion signals 
    logic sim_rst;
    int total_num_runs = 1;
    int sim_run_no = 0;   // current set   

    // ----------------------------------------------------
    // Internal Signals 
    // ----------------------------------------------------
    logic clk, rst_n;    

    // Instruction Load signals 
    logic [31:0] instr_mem [IMEM_SIZE];
    logic instr_wr_en;
    logic [31:0] wr_instr;
    int instr_count = 0;
    
    // processor status signals 
    logic valid, proc_idle, proc_exc, proc_resume;
    logic [31:0] cmt_pc, cmt_pc_d1;
    logic [31:0] resume_pc;

    // dump signals 
    logic [0:31][31:0] rf_hist [IMEM_SIZE];
    logic [0:DMEM_SIZE-1][31:0] mem_hist [IMEM_SIZE];
    logic [31:0] cmt_pc_hist [IMEM_SIZE];
    int hist_len = 0;
        
    // clk generation
    always begin
        clk <= 1; # 5; clk <= 0; # 5;
    end

    parameter string DIR = "../../../../test_vector/";
    
    string imem_filename;
    

    // ----------------------------------------------------
    // Main Simulation Process 
    // ----------------------------------------------------
    initial begin
        int f_st, f_scan;
        // Read Total number of runs  
        f_st = $fopen({DIR, "test_settings.txt"}, "r");
        f_scan = $fscanf(f_st, "%d", total_num_runs);
        $fclose(f_st);

        while(sim_run_no < total_num_runs) begin
            // 1. Reset 
            sim_rst <= 1'b1;
            @(posedge clk) 
            sim_rst <= 1'b0;

            // 2. Load Instruction 
            load_instr (sim_run_no);

            // 3. Wait completion 
            @(posedge proc_idle);
            repeat(2) @(posedge clk);

            // 4. Dump Result
            dump_result (sim_run_no); 

            sim_run_no++;
        end
    end


    // ----------------------------------------------------
    // Task - Load Instructions 
    // ----------------------------------------------------
    task automatic load_instr (input seq_no); 
        valid = 1'b0;
        // 1. load instructions from test file 
        foreach(instr_mem[i]) instr_mem[i] = 'hx;   // fill with X
        imem_filename = {DIR, $sformatf("testvector_imem_%0d.txt", seq_no)};
        $readmemh(imem_filename, instr_mem);
        foreach(instr_mem[i]) if (instr_mem[i] !== 'hx) instr_count++; 
        $display("Read %0d instructions", instr_count);

        // 2. Load to test unit 
        rst_n = 1'b0; #20; rst_n = 1'b1;
        @(posedge clk);
        instr_wr_en = 1'b1;
        for(int i = 0; i < instr_count; i++) begin
            wr_instr = instr_mem[i];
            @(posedge clk); 
        end
        instr_wr_en = 1'b0;
        
        @(posedge clk)
        valid = 1'b1;
    endtask


    // ----------------------------------------------------
    // Module Instantiation 
    // ----------------------------------------------------
    sim_if #(.WIDTH(TAG_WIDTH-1), .NUM_PHY_REGS(NUM_PHY_REGS)) sim();

    mem_if i_mem (.clk(clk), .rst_n(rst_n)); 
     
    riscv #(
        .NUM_PHY_REGS(NUM_PHY_REGS),
        .INSTR_QUEUE_SIZE(IMEM_SIZE)
    ) dut (
        .clk(clk), .rst_n(rst_n), .valid(valid), 
    
        .instr_wr_en(instr_wr_en), 
        .wr_instr(wr_instr),

        .mem(i_mem.fu),

        .resume(proc_resume),
        .resume_pc(resume_pc),
        .exception(proc_exc),
        .cmt_pc(cmt_pc),
        .completed(proc_idle),
        
        .sim(sim)
    );

    logic [0:DMEM_SIZE-1][31:0] dmem;
    data_mem #(
        .SIZE(DMEM_SIZE),
        .RD_DELAY_A(2), 
        .RD_DELAY_B(MEM_RD_CYCLE_RANGE),
        .DIR(DIR)
    ) u_dmem (
        .clk(clk), .rst_n(rst_n),
        .sim_rst(sim_rst), 
        .seq_no(sim_run_no), 
        .dmem_o(dmem),
        .mem(i_mem.mem)
    );


    // ----------------------------------------------------
    // handle exception
    // ----------------------------------------------------
    initial begin
    forever begin
        proc_resume <= '0;
        @(posedge proc_exc);
        repeat(2) @(posedge clk);
        proc_resume <= 1'b1;
        resume_pc <= cmt_pc + 12;
        @(posedge clk);
    end
    end


    // ----------------------------------------------------
    // Record results 
    // ----------------------------------------------------
    // Intermediate signals 
    logic [0:31][31:0] arf_frame;
    always_ff @(posedge clk) begin
        if (sim_rst) begin
            cmt_pc_hist <= '{default: '0};
            rf_hist <= '{default: '0};
            mem_hist <= '{default: '0};
            hist_len = 0;
        end else begin
            cmt_pc_d1 <= cmt_pc;
            if (valid && cmt_pc_d1 != cmt_pc) begin
                // get ARF 
                for (int i = 0; i < 32; i++) begin
                    arf_frame[i] = sim.prf[sim.arat[i]];
                end
                // record  
                cmt_pc_hist[hist_len] <= cmt_pc;
                rf_hist[hist_len] <= arf_frame;
                mem_hist[hist_len] <= dmem;
                hist_len <= hist_len + 1;
            end
        end
    end
    
    task automatic dump_result (input seq_no);
        string res_filename;
        int f_rf, f_mem;
        res_filename = {DIR, $sformatf("sim_result/res_rf_%0d.txt", seq_no)};
        f_rf = $fopen(res_filename, "w");
        for (int i = 0; i < hist_len; i++) begin
            $fwrite(f_rf, "%d ", $unsigned(cmt_pc_hist[i]));
            for (int j = 0; j < 32; j++)  $fwrite(f_rf, "%d ", $signed(rf_hist[i][j]));
            $fwrite(f_rf, "\n");
        end
        $fclose(f_rf);

        res_filename = {DIR, $sformatf("sim_result/res_mem_%0d.txt", seq_no)};
        f_mem = $fopen(res_filename, "w");
        for (int i = 0; i < hist_len; i++) begin
            $fwrite(f_mem, "%d ", $unsigned(cmt_pc_hist[i]));
            for (int j = 0; j < 32; j++)  $fwrite(f_mem, "%d ", $signed(mem_hist[i][j]));
            $fwrite(f_mem, "\n");
        end
        $fclose(f_mem);
    endtask

endmodule


module data_mem #(
    parameter SIZE = 32,
    parameter RD_DELAY_A = 2,
    parameter RD_DELAY_B = 10,
    parameter string DIR
)(
    input  logic clk, rst_n,
    input  logic sim_rst, 
    input  int   seq_no, 
    output logic [0:SIZE-1][31:0] dmem_o,
    mem_if.mem mem
);

    // Signals 
    logic ld_busy;
    logic [31:0] ld_addr; 
    string dmem_filename;

    // Memory definition 
    logic [31:0] dmem [SIZE];
    initial begin : u_dmem
        foreach(dmem[i]) dmem[i] = 'hx;
        dmem_filename = {DIR, $sformatf("testvector_dmem_%0d.txt", seq_no)};
        $readmemh(dmem_filename, dmem);
        mem.mem_ready = 1'b0;
        mem.st_cpl = 0; 
        mem.st_ok = 1'b1;
        ld_busy = 0;
    end

    // Wait a random number of cycle to supply memory data to load requests  
    task automatic supply_data();
        int delay_cycles;
        ld_busy = 1; // make unit busy, block processor from requesting  
        delay_cycles = $urandom_range(RD_DELAY_A, RD_DELAY_B);
        repeat (delay_cycles) @(posedge clk);
        mem.mem_ready = 1'b1;
        mem.mem_data  = dmem[ld_addr[31:2]];
        @(posedge clk);
        mem.mem_ready = 1'b0;
        ld_busy = 1'b0; // release unit
    endtask

    // watching load req
    always @(posedge clk) begin
        if (mem.ld_req && !ld_busy) begin
            ld_addr <= mem.ld_addr;
            fork
                supply_data();
            join_none
        end
    end

    // watching store req
    always @(posedge clk) begin
        if (mem.st_req) begin
            dmem[mem.st_addr[31:2]] = mem.st_data; 
            fork
                set_st_cpl();
            join_none
        end
    end

    task automatic set_st_cpl();
        repeat (4) @(posedge clk);
        mem.st_cpl = 1; 
        @(posedge clk);
        mem.st_cpl = 0; 
    endtask
    

    // signals to processor 
    always_comb begin
        mem.ld_ok = ~ld_busy;
    end
    
    always_comb begin
        for (int i = 0; i < SIZE; i++)
            dmem_o[i] = dmem[i];
    end
endmodule

