`timescale 1ns / 100ps

module sync_fifo_32 (
    input  logic        clk,
    input  logic        rst,
    input  logic        read_req,
    input  logic [31:0] write_data,
    input  logic        write_enable,
    output logic [31:0] read_data,
    output logic        fifo_empty,
    output logic        rdata_valid
);

    logic [4:0]  read_ptr;
    logic [4:0]  write_ptr;
    logic [31:0] mem [0:15];

    localparam FIFO_DEPTH = 16;
    localparam ADDR_WIDTH = $clog2(FIFO_DEPTH); // Automatically calculates 4 for 16

    logic [ADDR_WIDTH-1:0] write_addr;
    logic [ADDR_WIDTH-1:0] read_addr;

    assign write_addr = write_ptr[ADDR_WIDTH-1:0];
    assign read_addr  = read_ptr[ADDR_WIDTH-1:0];

    logic read_enable_int; // Internal signal for read_enable to avoid self-referencing issues
    assign read_enable_int = read_req && !fifo_empty; // Reordered for clarity: calculate fifo_empty first

    assign fifo_empty = (read_ptr == write_ptr); // Combinational check for empty

    // Write Pointer Logic
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            write_ptr <= '0; // SystemVerilog '0' initializes all bits to 0
        else if (write_enable)
            write_ptr <= write_ptr + 1; // Increment pointer
    end

    // Read Pointer Logic
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            read_ptr <= '0;
        else if (read_enable_int)
            read_ptr <= read_ptr + 1; // Increment pointer
    end

    // Memory Write
    always_ff @(posedge clk) begin
        if (write_enable)
            mem[write_addr] <= write_data;
    end

    // Memory Read & Read Data Valid
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            read_data   <= '0;
            rdata_valid <= 1'b0;
        end else begin
            if (read_enable_int) begin
                read_data   <= mem[read_addr];
                rdata_valid <= 1'b1;
            end else begin
                // If not reading, rdata_valid should typically go low
                // and read_data should hold its last value or go to X/0
                // Based on original, it goes to 0 if not enabled.
                rdata_valid <= 1'b0;
                // read_data should ideally hold its value until the next valid read.
                // However, the original sets it to 0 if read_enable is off.
                // Keeping original behavior:
                // read_data <= '0; // This makes it combinational, but the original was seq.
                // The original implicitly kept `read_data` as a 'reg' retaining value
                // until a new read_enable or reset.
                // To precisely match the original's sequential `read_data` without combinational behavior for 'else',
                // and assuming `read_data` only updates on `read_enable`, this is better:
                // No 'else' branch for read_data if you want it to hold, but original had it in the same block.
                // The most faithful translation of the original is the combined block below for `read_data`
                // if it *only* updates on `read_enable_int` and then stays.
                // Let's refine based on the original's multiple `always` blocks:
            end
        end
    end

    // Re-combining read_data and rdata_valid based on original structure
    // The original Verilog had separate always blocks for rdata_valid and read_data.
    // This is valid in SystemVerilog as well, often preferred for clarity.

    // rdata_valid logic (original logic was fine, slightly more concise now)
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            rdata_valid <= 1'b0;
        else
            rdata_valid <= read_enable_int; // If read_enable, then valid, else not.
    end

    // read_data logic (original logic was fine)
    always_ff @(posedge clk) begin // No rst in this original block
        if (read_enable_int)
            read_data <= mem[read_addr];
    end

endmodule
