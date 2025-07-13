/*
--------------------------------------------------------------------------------
Module: sync_fifo_ff (SystemVerilog)
Description:
  - Synchronous FIFO with 91-bit wide data for JPEG bitstream stages.
  - Stores encoded blocks and outputs them sequentially for FF checking.
  - Supports special "rollover_write" case to add delay after FF escaping.
  - FIFO depth: 16 entries.
  - Outputs valid flag and FIFO empty signal.

Inputs:
  - clk, rst              : Clock and active-high reset.
  - read_req              : Read request from consumer.
  - write_data [90:0]     : Data to be written into FIFO.
  - write_enable          : Write enable signal.
  - rollover_write        : Special condition to delay writing by 1 cycle.

Outputs:
  - read_data [90:0]      : Data read from FIFO.
  - fifo_empty            : Indicates FIFO is empty.
  - rdata_valid           : Output data is valid and can be used.
--------------------------------------------------------------------------------
*/

`timescale 1ns / 100ps

module sync_fifo_ff (
    input  logic        clk,
    input  logic        rst,
    input  logic        read_req,
    input  logic [90:0] write_data,
    input  logic        write_enable,
    input  logic        rollover_write,
    output logic [90:0] read_data,
    output logic        fifo_empty,
    output logic        rdata_valid
);

// --------------------------------------------------
// FIFO memory and pointers
// --------------------------------------------------
logic [90:0] mem [15:0];        // FIFO memory: 16 entries of 91 bits
logic [4:0] write_ptr, read_ptr; // Full pointer for comparison
logic [3:0] write_addr, read_addr;

assign write_addr = write_ptr[3:0];
assign read_addr  = read_ptr[3:0];

// --------------------------------------------------
// FIFO empty logic
// --------------------------------------------------
assign fifo_empty = (write_ptr == read_ptr);

// --------------------------------------------------
// Read enable: only allowed if FIFO is not empty
// --------------------------------------------------
logic read_enable;
assign read_enable = read_req && !fifo_empty;

// --------------------------------------------------
// Write pointer logic with rollover handling
// --------------------------------------------------
always_ff @(posedge clk or posedge rst) begin
    if (rst)
        write_ptr <= 5'd0;
    else if (write_enable) begin
        if (rollover_write)
            write_ptr <= write_ptr + 5'd2;  // Skip one entry to give time for FF insert
        else
            write_ptr <= write_ptr + 5'd1;
    end
end

// --------------------------------------------------
// FIFO write operation
// --------------------------------------------------
always_ff @(posedge clk) begin
    if (write_enable)
        mem[write_addr] <= write_data;
end

// --------------------------------------------------
// Read pointer logic
// --------------------------------------------------
always_ff @(posedge clk or posedge rst) begin
    if (rst)
        read_ptr <= 5'd0;
    else if (read_enable)
        read_ptr <= read_ptr + 5'd1;
end

// --------------------------------------------------
// FIFO read operation
// --------------------------------------------------
always_ff @(posedge clk) begin
    if (read_enable)
        read_data <= mem[read_addr];
end

// --------------------------------------------------
// Valid data flag generation
// --------------------------------------------------
always_ff @(posedge clk or posedge rst) begin
    if (rst)
        rdata_valid <= 1'b0;
    else
        rdata_valid <= read_enable;
end

endmodule
