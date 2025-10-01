//-----------------------------------------------------------------------------
// Dual channel Common Data Bus (CDB) Interface
//-----------------------------------------------------------------------------
interface cdb_if #(
    parameter N = 3, // number of clients(Funtional Units) 
    parameter TAG_WIDTH = 5
)(
    input logic clk, rst_n
);
    
    // Producer signals
    logic [31:0] data_in [N];          // Data being broadcast
    logic [TAG_WIDTH-1:0] tag_in [N];  // Tag (dest register number)
    logic exception_in [N];            // functional unit assert excpetion  
    logic [0:N-1] req;    // Request from Functional Units
    logic [0:N-1] grant;


    // Output signals 
    logic [0:1]  valid;   // Indicates valid data on the bus
    logic [0:1][31:0] data;
    logic [0:1][TAG_WIDTH-1:0] tag;
    logic [0:1] exception;

    // internal signals 
    logic [0:1] grant_valid;
    logic [$clog2(N)-1:0] grant_bin [2];
    logic [0:N-1] grant_oh [2];  // Grant signal (one-hot)

    assign grant_bin[0] = one_hot_to_binary(grant_oh[0]);
    assign grant_bin[1] = one_hot_to_binary(grant_oh[1]);

    
    always_comb begin : u_grant_rewire
        for (int i = 0; i < N; i++) begin
            grant[i] = grant_oh[0][i] | grant_oh [1][i]; 
        end
    end
    
    always_ff @(posedge clk) begin
        for (int i = 0; i < 2; i++) begin
            if (!rst_n) begin
                valid[i] <= 1'b0;
                data[i]  <= '0;
                tag[i]   <= '0;
                exception[i] <= '0;
            end else if (grant_valid[i]) begin
                valid[i] <= 1'b1;
                data[i]  <= data_in[grant_bin[i]];
                tag[i]   <= tag_in[grant_bin[i]];
                exception[i] <= exception_in[grant_bin[i]];
            end else begin
                valid[i] <= 1'b0;
            end
        end
    end

    // Modports for different views of the bus
    // Functional Unit (Producer) side
    modport producer (
        output data_in,
        output tag_in,
        output exception_in,
        output req,
        input  grant
    );

    // Reservation Station / ROB (Consumer) side
    modport consumer (
        input valid,
        input data,
        input tag,
        input exception
    );

    modport arbiter (
        input  req,
        output grant_oh,
        output grant_valid
    );

    function automatic logic [$clog2(N)-1:0] one_hot_to_binary (
        input logic [0:N-1] vector_oh
    );
        logic [$clog2(N)-1:0] binary_out = '0;
        for (int i = 0; i < N; i++) begin
            if (vector_oh[i]) begin
                binary_out = i;
                break; // Exit once the set bit is found
            end
        end
        return binary_out;
    endfunction

endinterface



module cdb_arbiter #(
    parameter int N = 4,  // must be >= 2 
    localparam int PTR_W =  $clog2(N)
) (
    input  logic clk, rst_n,
    cdb_if.arbiter cdb
);
    // Internal Signals 
    logic [N-1:0] req;     // request lines
    logic grant0_valid;
    logic [N-1:0] grant0;  // first grant (one-hot)
    logic grant1_valid;
    logic [N-1:0] grant1; 

    // Round-robin pointer
    logic [PTR_W-1:0] ptr, ptr_next;

    // Rotate requests so ptr maps to bit 0
    logic [N-1:0] rot_req;
    logic [N-1:0] g0_rot, g1_rot;

    // Rotate requests right by ptr: rot_req = ROR(req, ptr)
    always_comb begin : u_req_rotate
        rot_req = ({req, req} >> ptr);
    end

    logic [N-1:0] rot_req_2;
    always_comb begin
        g0_rot = rot_req & (~rot_req + 1); // Pick first winner
        rot_req_2 = rot_req & ~g0_rot;     // remove first winner
        g1_rot = rot_req_2 & (~rot_req_2 + 1); // pick the second winner
        
    end

    // Un-rotate grants back: grant = ROL(g_rot, ptr)
    always_comb begin
        logic [2*N-1:0] temp0, temp1;
        grant0_valid = |rot_req;
        grant1_valid = |rot_req_2;
        temp0 = ({g0_rot, g0_rot} << ptr);
        grant0 = temp0[2*N-1:N];
        temp1 = ({g1_rot, g1_rot} << ptr);
        grant1 = temp1[2*N-1:N];
    end

    // Advance pointer by number of grants issued (0,1,2) 
    logic [1:0] grant_cnt;
    always_comb begin
        logic [PTR_W:0] tmp;
        grant_cnt = (|g0_rot) + (|g1_rot);
        // modulo-N increment
        tmp = ptr + grant_cnt;
        if (tmp >= N) 
            ptr_next = tmp - N;
        else          
            ptr_next = tmp[PTR_W-1:0];
    end

    // --- Pointer register ---
    always_ff @(posedge clk) begin
        if (!rst_n) ptr <= '0;
        else        ptr <= ptr_next;
    end

    // interface signal 
    always_comb begin : u_interface_alias 
        req = cdb.req;
        cdb.grant_oh[0] = grant0;
        cdb.grant_oh[1] = grant1;
        cdb.grant_valid[0] = grant0_valid;
        cdb.grant_valid[1] = grant1_valid;
    end

endmodule