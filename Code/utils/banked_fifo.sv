//-----------------------------------------------------------------------------
// N way banked fifo 
// Support N concurrent reads and write at one cycle 
// N must be a power of 2  
// Sequential Write/Read: must be asserted in order, cannot skip lower ports 
// Random Write/Read: no need to assert in order, can skip lower ports  
//-----------------------------------------------------------------------------
/*
module banked_fifo #(
    parameter int WIDTH = 32,
    parameter int DEPTH = 8,   // depth of each bank
    parameter int N = 4,
    parameter bit SHOW_AHEAD = 1'b1
) (
    input  logic clk, rst_n,
    input  var logic [WIDTH-1:0] wr_data [N],
    input  logic [0:N-1] wr_en, 
    output logic [0:N-1] wr_ok,
    
    output var logic [WIDTH-1:0] rd_data [N],
    input  logic [0:N-1] rd_en,
    output logic [0:N-1] rd_ok  
);
    // -------------------------
    // Queue signals 
    // -------------------------
    logic [WIDTH-1:0] q_wr_data [N];
    logic [WIDTH-1:0] q_rd_data [N];
    logic [0:N-1] q_wr_en, q_rd_en;
    logic [0:N-1] q_full;
    logic [0:N-1] q_empty;
    
    generate 
    for (genvar i = 0; i < N; i++) begin : g_queue
        sync_fifo #(
            .DEPTH(DEPTH),  .WIDTH(WIDTH),
            .SHOW_AHEAD(1'b1)
        ) u_queue (
            .clk(clk),   .rst_n(rst_n),
            .wr_en(q_wr_en[i]), 
            .wr_data(q_wr_data[i]),
            .full(q_full[i]),
            .rd_en(q_rd_en[i]),
            .rd_data(q_rd_data[i]),
            .empty(q_empty[i])
        );
    end
    endgenerate

    // ------------------------------
    // Write (Enqueue) Logic 
    // ------------------------------
    logic [$clog2(N)-1:0] wr_ptr, wr_ptr_next;
    logic [$clog2(N):0] wr_count; //Number of reads in this cycle

    integer i, wr_idx;
    always_comb begin : write_logic
        // default outputs
        for (i = 0; i < N; i++) begin
            q_wr_en[i] = 1'b0;
            q_wr_data[i] = '0;
        end

        wr_count = 0;
        // Scan inputs from port 0 → 3
        for (i = 0; i < N; i++) begin
            if (wr_en[i]) begin
                // physical output index with wrap-around
                wr_idx = (wr_ptr + wr_count) % N;
                q_wr_en[wr_idx] = 1'b1; 
                q_wr_data[wr_idx] = wr_data[i];
                wr_count = wr_count + 1;
            end
        end
        // pointer advance for next cycle
        wr_ptr_next = (wr_ptr + wr_count) % N;
    end
    // pointer update 
    always_ff @(posedge clk or negedge rst_n) begin : wr_ptr_update
        if (!rst_n)
            wr_ptr <= '0;
        else
            wr_ptr <= wr_ptr_next;
    end
    assign wr_ok = rotl(~q_full, wr_ptr);


    // ------------------------------
    // Read (Dequeue) Logic 
    // ------------------------------
    logic [$clog2(N)-1:0] rd_ptr, rd_ptr_next;
    logic [$clog2(N):0] rd_count; //Number of reads in this cycle

    integer j, rd_idx;
    always_comb begin : read_logic
        // default outputs
        for (j = 0; j < N; j++) begin
            q_rd_en[j] = 1'b0;
            rd_data[j] = '0;
        end

        rd_count = 0;
        // Scan inputs from port 0 -> 3
        for (j = 0; j < N; j++) begin
            if (rd_en[j]) begin
                // physical output index with wrap-around
                rd_idx = (rd_ptr + rd_count) % N;
                q_rd_en[rd_idx] = 1'b1; 
                rd_data [j] = q_rd_data[rd_idx];
                rd_count = rd_count + 1;
            end
        end
        // pointer advance for next cycle
        rd_ptr_next = (rd_ptr + rd_count) % N;
    end
    // pointer update 
    always_ff @(posedge clk or negedge rst_n) begin : rd_ptr_update
        if (!rst_n)
            rd_ptr <= '0;
        else
            rd_ptr <= rd_ptr_next;
    end
    assign rd_ok = rotl(~q_empty, rd_ptr);
    
    function automatic logic [N-1:0] rotl;
        input  logic [N-1:0] data;           // input vector
        input  int unsigned n;                   // rotation amount
        begin
            // ensure n fits within WIDTH
            rotl = (data << n) | (data >> (WIDTH - n));
        end
    endfunction


endmodule : banked_fifo
*/


//-----------------------------------------------------------------------------
// N way banked fifo 
// Support N concurrent reads and write at one cycle 
// N must be a power of 2  
// Sequential Write/Read: must be asserted in order, cannot skip lower ports 
//-----------------------------------------------------------------------------

module banked_fifo_seq #(
    parameter int WIDTH = 32,
    parameter int DEPTH = 8,   // depth of each bank
    parameter int N = 4,
    parameter bit SHOW_AHEAD = 1'b1
) (
    input  logic clk, rst_n, 
    input  logic clr,
    input  var logic [WIDTH-1:0] wr_data [N],
    input  logic [0:N-1] wr_en, 
    output logic [0:N-1] wr_ok,
    
    output var logic [WIDTH-1:0] rd_data [N],
    input  logic [0:N-1] rd_en,
    output logic [0:N-1] rd_ok  
);
    // -------------------------
    // Queue signals 
    // -------------------------
    logic [WIDTH-1:0] q_wr_data [N];
    logic [WIDTH-1:0] q_rd_data [N];
    logic [0:N-1] q_wr_en, q_rd_en;
    logic [0:N-1] q_full;
    logic [0:N-1] q_empty;
    
    generate 
    for (genvar i = 0; i < N; i++) begin : g_queue
        sync_fifo #(
            .DEPTH(DEPTH),  .WIDTH(WIDTH),
            .SHOW_AHEAD(1'b1)
        ) u_queue (
            .clk(clk),   .rst_n(rst_n),
            .clr(clr),
            .wr_en(q_wr_en[i]), 
            .wr_data(q_wr_data[i]),
            .full(q_full[i]),
            .rd_en(q_rd_en[i]),
            .rd_data(q_rd_data[i]),
            .empty(q_empty[i])
        );
    end
    endgenerate

    // ------------------------------
    // Write (Enqueue) Logic 
    // ------------------------------
    logic [$clog2(N)-1:0] wr_ptr, wr_ptr_next;
    logic [$clog2(N):0] wr_count; //Number of reads in this cycle
    assign wr_count = $countones(wr_en);

    integer i, wr_idx;
    always_comb begin : write_logic
        // Scan inputs from port 0 -> 3
        for (i = 0; i < N; i++) begin
            wr_idx   = (wr_ptr + i) % N;
            wr_ok[i] = ~q_full[wr_idx]; 
            if (wr_en[i]) begin
                // physical output index with wrap-around
                q_wr_en[wr_idx] = 1'b1; 
                q_wr_data[wr_idx] = wr_data[i];
            end else begin 
                q_wr_en[wr_idx] = 1'b0;
                q_wr_data[wr_idx] = '0;
            end
        end
        // pointer advance for next cycle
        wr_ptr_next = (wr_ptr + wr_count) % N;
    end
    // pointer update 
    always_ff @(posedge clk or negedge rst_n) begin : wr_ptr_update
        if (!rst_n)
            wr_ptr <= '0;
        else
            wr_ptr <= wr_ptr_next;
    end

    // ------------------------------
    // Read (Dequeue) Logic 
    // ------------------------------
    logic [$clog2(N)-1:0] rd_ptr, rd_ptr_next;
    logic [$clog2(N):0] rd_count; //Number of reads in this cycle
    assign rd_count = $countones(rd_en);

    integer j, rd_idx;
    always_comb begin : read_logic
        // Scan inputs from port 0 -> 3
        for (j = 0; j < N; j++) begin
            rd_idx   = (rd_ptr + j) % N;
            rd_ok[j] = ~q_empty[rd_idx];
            if (rd_en[j]) begin
                q_rd_en[rd_idx] = 1'b1; 
                rd_data [j] = q_rd_data[rd_idx];
            end else begin
                q_rd_en[rd_idx] = 1'b0;
                rd_data[j] = '0;
            end
        end
        // pointer advance for next cycle
        rd_ptr_next = (rd_ptr + rd_count) % N;
    end

    // pointer update 
    always_ff @(posedge clk or negedge rst_n) begin : rd_ptr_update
        if (!rst_n)
            rd_ptr <= '0;
        else
            rd_ptr <= rd_ptr_next;
    end
endmodule 