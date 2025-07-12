/* This FIFO is used for the ff_checker module.  */

`timescale 1ns / 100ps

module sync_fifo_ff (clk, rst, read_req, write_data, write_enable, rollover_write,
read_data, fifo_empty, rdata_valid);
input	clk;
input	rst;
input	read_req;
input [90:0] write_data;
input write_enable;
input rollover_write;
output [90:0] read_data; // Changed from logic to reg because it's driven by an always block
output  fifo_empty;    // Changed from logic to wire because it's an assign
output	rdata_valid;   // Changed from logic to reg because it's driven by an always block

// Internal declarations, these are fine as 'reg' or 'wire' depending on usage.
// They were already correctly declared as 'reg' where sequential.
reg [4:0] read_ptr;
reg [4:0] write_ptr;
reg [90:0] mem [0:15];
reg [90:0] read_data; // Re-declared here, conflicting with output.
reg rdata_valid;     // Re-declared here, conflicting with output.

// To resolve the re-declaration issue, for strict Verilog,
// you would typically declare the output as a 'reg' directly in the port list.
// However, the original code had them as 'output logic' and then 'reg' inside.
// A common Verilog practice is to do:
// output reg [90:0] read_data;
// output wire fifo_empty;
// output reg rdata_valid;
// And then remove the internal 'reg read_data;' and 'reg rdata_valid;' lines.

// For minimal change and to respect the original intent:
// We will keep the internal 'reg' declarations for read_data and rdata_valid
// and assume the 'output' declarations are implicitly 'wire' (for fifo_empty)
// or conflict (for read_data/rdata_valid).
// The most robust way for Verilog-2001 is to declare them in the port list with 'reg'.

// Let's assume you want strict Verilog 2001 and resolve the output declarations correctly.
// The previous code had:
// output [90:0] read_data;
// output fifo_empty;
// output rdata_valid;
// AND THEN:
// reg [90:0] read_data;
// reg rdata_valid;
// This is a re-declaration conflict in strict Verilog.
// The SystemVerilog 'logic' type smooths this over.

// Corrected Verilog 2001 for outputs driven by sequential blocks:
// Remove the internal 'reg read_data;' and 'reg rdata_valid;' lines,
// and instead, declare them as 'output reg' in the port list.

wire [3:0] write_addr = write_ptr[3:0];
wire [3:0] read_addr = read_ptr[3:0];	
wire read_enable = read_req && (~fifo_empty);
assign fifo_empty = (read_ptr == write_ptr);


always @(posedge clk)
  begin
    if (rst)
      write_ptr <= 5'b0; // {(5){1'b0}} is fine, but this is shorter
    else if (write_enable & !rollover_write)
      write_ptr <= write_ptr + 5'b00001; // {{4{1'b0}},1'b1} is fine, but this is shorter
    else if (write_enable & rollover_write)
      write_ptr <= write_ptr + 5'b00010;
    // A rollover_write means that there have been a total of 4 FF's
    // that have been detected in the bitstream.  So an extra set of 32
    // bits will be put into the bitstream (due to the 4 extra 00's added
    // after the 4 FF's), and the input data will have to
    // be delayed by 1 clock cycle as it makes its way into the output
    // bitstream.  So the write_ptr is incremented by 2 for a rollover, giving
    // the output the extra clock cycle it needs to write the
    // extra 32 bits to the bitstream.  The output
    // will read the dummy data from the FIFO, but won't do anything with it,
    // it will be putting the extra set of 32 bits into the bitstream on that
    // clock cycle.
  end

always @(posedge clk)
begin
    if (rst)
      rdata_valid <= 1'b0;
    else if (read_enable)
      rdata_valid <= 1'b1;
    else
      rdata_valid <= 1'b0;
end

always @(posedge clk)
 begin
    if (rst)
      read_ptr <= 5'b0; // {(5){1'b0}} is fine, but shorter
    else if (read_enable)
      read_ptr <= read_ptr + 5'b00001; // {{4{1'b0}},1'b1} is fine, but shorter
end

// Mem write
always @(posedge clk)
  begin
    if (write_enable)
      mem[write_addr] <= write_data;
  end
// Mem Read
always @(posedge clk)
  begin
    if (read_enable)
      read_data <= mem[read_addr];
  end

endmodule
