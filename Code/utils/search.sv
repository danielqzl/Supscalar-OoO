/*
module find_first_set_bit #(
    parameter N = 32,
    parameter IDX_WIDTH = $clog2(N)
) (
    input  logic [0:N-1] in,              
    output logic [IDX_WIDTH-1:0] index,     
    output logic found
);
    logic [IDX_WIDTH-1:0] result;

    always_comb begin
        found  = '0;
        result = '0;
        for (int i = 0; i < N; i++) begin
            if (!found && in[i]) begin
                result = i;
                found = 1'b1;
            end
        end
        index = result;
    end
endmodule
*/


module find_two_set_bits_rev #(
    parameter int WIDTH  = 8,
    parameter int IDX_W  = $clog2(WIDTH)
) (
    input  logic [0:WIDTH-1] in,
    output logic [0:WIDTH-1] onehot1,
    output logic [0:WIDTH-1] onehot2,
    output logic [IDX_W-1:0] idx1,   // index of lowest set bit
    output logic [IDX_W-1:0] idx2,   // index of next-lowest set bit
    output logic found_one,
    output logic found_two
);
    // Check if the vector has >=1 ones: x != 0
    assign found_one = |in;
    // Check if the vector has >=2 ones: (x & (x-1)) != 0
    assign found_two = |(in & (in - 1));

    // Isolate lowest set bit, then isolate next
    logic [0:WIDTH-1] masked;
    assign masked  = in & ~ onehot1;   // clear the lowest one

    always_comb begin : u_oh
        if (found_one) onehot1 = in & (~in + 1); // x & -x
        else onehot1 = '0;
        if (found_two) onehot2 = masked & (~masked + 1);
        else onehot2 = '0;
    end

    // Onehot to binary converter 
    always_comb begin : u_bin_1
        idx1 = '0;
        for (int i = 0; i < WIDTH; i++) begin
            if (onehot1[i])
                idx1 = IDX_W'(i);
        end
    end

    always_comb begin : u_bin_2
        idx2 = '0;
        for (int i = 0; i < WIDTH; i++) begin
            if (onehot2[i])
                idx2 = IDX_W'(i);
        end
    end

endmodule



module find_two_set_bits #(
    parameter int WIDTH  = 8,
    parameter int IDX_W  = $clog2(WIDTH)
) (
    input  logic [WIDTH-1:0] in,
    output logic [WIDTH-1:0] onehot1,
    output logic [WIDTH-1:0] onehot2,
    output logic [IDX_W-1:0] idx1,   // index of lowest set bit
    output logic [IDX_W-1:0] idx2,   // index of next-lowest set bit
    output logic found_one,
    output logic found_two
);
    // Check if the vector has >=1 ones: x != 0
    assign found_one = |in;
    // Check if the vector has >=2 ones: (x & (x-1)) != 0
    assign found_two = |(in & (in - 1));

    // Isolate lowest set bit, then isolate next
    logic [WIDTH-1:0] masked;
    assign masked  = in & ~ onehot1;   // clear the lowest one

    always_comb begin : u_oh
        if (found_one) onehot1 = in & (~in + 1); // x & -x
        else onehot1 = '0;
        if (found_two) onehot2 = masked & (~masked + 1);
        else onehot2 = '0;
    end

    // Onehot to binary converter 
    always_comb begin : u_bin_1
        idx1 = '0;
        for (int i = 0; i < WIDTH; i++) begin
            if (onehot1[i])
                idx1 = IDX_W'(i);
        end
    end

    always_comb begin : u_bin_2
        idx2 = '0;
        for (int i = 0; i < WIDTH; i++) begin
            if (onehot2[i])
                idx2 = IDX_W'(i);
        end
    end

endmodule
