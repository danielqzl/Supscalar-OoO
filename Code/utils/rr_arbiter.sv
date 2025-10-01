// =============================================================================
// Round‑robin arbiter 
// Muxed design, fast but occupy more area 
// =============================================================================
module rr_arbiter_3 (
    input  logic       clk,
    input  logic       rst_n,
    input  logic [2:0] req,
    input  logic       req_valid,
    output logic [2:0] grant,
    output logic       grant_valid
);
    assign grant_valid = (req_valid && (|req != 0));

    logic [2:0] pri_out [3];

    fixed_priority_arbiter #(3) u_p0 (.req(req),    .grant(pri_out_0));
    fixed_priority_arbiter #(3) u_p1 (.req(req>>1), .grant(pri_out_1));
    fixed_priority_arbiter #(3) u_p2 (.req(req>>2), .grant(pri_out_2));

    //Select line pointer calculation
    logic [1:0] ptr, next_ptr;

    always @(posedge clk) begin
        if (!rst_n) 
            ptr <= '0;
        else if(req_valid)	 
            ptr <= next_ptr;
    end

    always_comb begin
        next_ptr = 2'b00;
        casez (grant)
            3'b001: next_ptr = 2'b01;
            3'b010: next_ptr = 2'b10;
            3'b100: next_ptr = 2'b00;
        endcase
    end

    always_comb begin
        case (ptr)
            2'b00: grant = pri_out_0;
            2'b01: grant = pri_out_1;
            2'b10: grant = pri_out_2;
            2'b11: grant = '0;
        endcase
    end
  
endmodule


module rr_arbiter_4 #(
    parameter logic [1:0] PRI_OS = 2'b00
) (
    input  logic       clk, rst_n,
    input  logic       rotate,
    input  logic [3:0] req,
    input  logic       req_valid,
    output logic [3:0] grant,
    output logic       grant_valid
);
    assign grant_valid = (req_valid && (|req != 0));

    logic [3:0] pri_out [4];

    generate 
        for(genvar i = 0; i < 4; i++) begin
            fixed_priority_arbiter #(4) u_p (
                .req(req>>i),    
                .grant(pri_out[i])
            );
        end
    endgenerate


    //Select line pointer calculation
    logic [1:0] ptr, next_ptr;

    always @(posedge clk) begin
        if (!rst_n) 
            ptr <= PRI_OS;
        else if(req_valid || rotate)	 
            ptr <= next_ptr;
    end

    always_comb begin
        next_ptr = 2'b00;
        casez (grant)
            4'b0001: next_ptr = (PRI_OS + 2'b01);
            4'b0010: next_ptr = (PRI_OS + 2'b10);
            4'b0100: next_ptr = (PRI_OS + 2'b11);
            4'b1000: next_ptr = (PRI_OS + 2'b00);
        endcase
    end

    always_comb begin
        case (ptr)
            2'b00: grant = pri_out[0];
            2'b01: grant = pri_out[1];
            2'b10: grant = pri_out[2];
            2'b11: grant = pri_out[3];
        endcase
    end
  
endmodule


module fixed_priority_arbiter #(
    parameter N = 4
) (
    input  logic [N-1:0] req,
    output logic [N-1:0] grant
);

    logic [N-1:0] higher_pri_req;
    assign higher_pri_req[0] = 1'b0; //LSB has the highest priority 
    
    generate
    for (genvar i = 0; i < N - 1; i++) begin
        assign higher_pri_req[i+1] = higher_pri_req[i] | req[i];
    end
    endgenerate

    assign grant[N-1:0] = req[N-1:0] & ~higher_pri_req[N-1:0];

endmodule

