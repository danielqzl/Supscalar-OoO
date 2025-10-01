module sync_fifo #(
    parameter int DEPTH = 8, // must be a power of 2
    parameter int WIDTH = 64,
    parameter bit SHOW_AHEAD = 1'b1
) (
    input  logic clk, rst_n,
    input  logic clr,

    input  logic wr_en, 
    input  logic [WIDTH-1:0] wr_data,
    output logic full,

    input  logic rd_en,
    output logic [WIDTH-1:0] rd_data,
    output logic empty
);

    localparam PTR_W = $clog2(DEPTH);
    logic [PTR_W:0] w_ptr, r_ptr;

    logic [WIDTH-1:0] fifo [DEPTH];

    // To write data to FIFO
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            w_ptr <= '0;
            for (int i = 0; i < DEPTH; i++) fifo[i] <= '0; 
        end else if (clr) begin
            w_ptr <= '0;
        end else if(wr_en & !full)begin
            fifo[w_ptr[PTR_W-1:0]] <= wr_data;
            w_ptr <= w_ptr + 1;
        end
    end

    // Read pointer 
    always_ff @(posedge clk) begin
        if (!rst_n | clr) begin
            r_ptr <= 0;
        end else if(rd_en & !empty) begin
            r_ptr <= r_ptr + 1;
        end
    end

    // To read data from FIFO
    generate 
        if (SHOW_AHEAD) begin : g_showahead  // combinational read path
            assign rd_data = fifo[r_ptr[PTR_W-1:0]];
        end 
        else begin : g_registered       // registered read path
            logic [WIDTH-1:0] rd_data_r;
            always_ff @(posedge clk) begin
                if(rd_en & !empty) begin
                    rd_data_r <= fifo[r_ptr[PTR_W-1:0]];
                end
            end
            assign rd_data = rd_data_r;
        end
    endgenerate

    assign full  = ((w_ptr[PTR_W] != r_ptr[PTR_W]) && (w_ptr[PTR_W-1:0] == r_ptr[PTR_W-1:0]));
    assign empty = ((w_ptr[PTR_W] == r_ptr[PTR_W]) && (w_ptr[PTR_W-1:0] == r_ptr[PTR_W-1:0]));
endmodule









/*
// ============================================================================
// Simple, flexible synchronous queue (FIFO)
// ---------------------------------------------------------------------------
//  Simultaneous push and pop is allowed.  
//  Depth must be a power-of-two ≥1 
//  One clock domain   
//  Optional comb read, data available one cycle earlier
// ============================================================================
module sync_fifo #(
    parameter int WIDTH  = 64, // Data width       
    parameter int DEPTH  = 8,  // queue depth  (Must be power-of-two)
    parameter bit SHOW_AHEAD = 1'b1    // 1 = combinational read ("look-ahead")
)(
    input  logic clk,
    input  logic rst_n, 
    // ----------------  Enqueue side  ----------------
    input  logic             wr_en,  // enqueue request
    input  logic [WIDTH-1:0] wr_data,   // data in
    output logic             full,  // asserted when queue cannot accept data
    // ----------------  Dequeue side  ----------------
    input  logic             rd_en,   // dequeue request
    output logic [WIDTH-1:0] rd_data,  // data out
    output logic             empty  // asserted when queue has no valid data
);
    // ------------------------------------------------------------------------
    // Local parameters / sanity checks
    // ------------------------------------------------------------------------
    localparam int ADDR_W = $clog2(DEPTH);            // address bits
    initial begin
        if (DEPTH < 1 || (DEPTH & (DEPTH-1)) != 0) begin
            $fatal(1, "DEPTH (%0d) must be a power-of-two ≥1", DEPTH);
        end
    end

    // ------------------------------------------------------------------------
    // Storage array 
    // ------------------------------------------------------------------------
    logic [WIDTH-1:0] mem [DEPTH];                    // the queue RAM

    // ------------------------------------------------------------------------
    // Read / write pointers (ADDR_W + 1 bits: MSB is the wrap (phase) bit)
    // ------------------------------------------------------------------------
    logic [ADDR_W:0] wr_ptr, rd_ptr;

    // Empty / full decode
    assign empty = (wr_ptr == rd_ptr);
    assign full  = (wr_ptr[ADDR_W-1:0] == rd_ptr[ADDR_W-1:0]) &&
                   (wr_ptr[ADDR_W]     != rd_ptr[ADDR_W]);

    // ------------------------------------------------------------------------
    // Pointer Logic
    // ------------------------------------------------------------------------
    logic push_ok = wr_en & ~full;      // enqueue accepted
    logic pop_ok  = rd_en & ~empty;     // dequeue accepted

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
        end else begin
            if (push_ok)
                wr_ptr <= wr_ptr + 1'b1;
            if (pop_ok)
                rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // ------------------------------------------------------------------------
    // Write
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (push_ok)
            mem[wr_ptr[ADDR_W-1:0]] <= wr_data;
    end

    // ------------------------------------------------------------------------
    // Read  (two styles)
    // ------------------------------------------------------------------------
    generate
        if (SHOW_AHEAD) begin : g_showahead  // combinational read path
            assign rd_data = mem[rd_ptr[ADDR_W-1:0]];
        end else begin : g_registered       // registered read path
            logic [WIDTH-1:0] rd_data_r;
            always_ff @(posedge clk) begin
                if (pop_ok)
                    rd_data_r <= mem[rd_ptr[ADDR_W-1:0]];
            end
            assign rd_data = rd_data_r;
        end
    endgenerate

endmodule

*/