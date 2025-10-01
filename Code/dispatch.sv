module dispatch_unit 
    import riscv_pkg ::*;
(
    input  logic [0:1] uop_q_valid, 
    output logic [0:1] dispatch_valid,
    input  logic stall
);

    always_comb begin
        if (stall || (uop_q_valid == 2'b00)) begin
            dispatch_valid = 2'b00;
        end else begin
            dispatch_valid = uop_q_valid;
        end 
    end
    
endmodule


module hazard_unit
    import riscv_pkg ::*;
#(
    parameter N_RS = 2
)(   
    input  logic clk, rst_n,
    input  logic [0:1] rename_stalled,
    input  logic [0:1] rob_credit,
    input  var logic ren_valid [2],
    input  logic [0:N_RS-1] rs_stalled_full,
    input  var cmt_res_s cmt [2],
    input  logic valid,
    input  logic exc_flush,
    input  logic resume,

    output logic stall,
    output logic proc_excepion 
);

    logic exc_stall; 
    logic pl_stall;
    logic scheduler_stalled;

    always_comb begin : u_scheduler_stalled
        // scheduler trigger stall if any RS stall due to fullness 
        scheduler_stalled = |rs_stalled_full;  
    end

    always_comb begin : u_stall
        pl_stall = (|rename_stalled | (rob_credit != '1) | scheduler_stalled);
        
        stall =  ~valid | pl_stall | exc_stall;
    end

    // Processor Status 
    always_ff @(posedge clk) begin
        if (!rst_n)
            exc_stall <= 1'b0;    
        else if (resume)
            exc_stall <= 1'b0;
        else if (exc_flush)
            exc_stall <= 1'b1;
    end

    assign proc_excepion = exc_flush;

    

endmodule
