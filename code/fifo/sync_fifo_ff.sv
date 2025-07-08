/* This FIFO is used for the ff_checker module.  */

timescale 1ns / 100ps

module sync_fifo_ff (
  input  logic        clk,
  input  logic        rst,
  input  logic        read_req,
  input  logic [90:0] write_data,
  input  logic        write_enable,
  input  logic        rollover_write, // Added this input
  output logic [90:0] read_data,
  output logic        fifo_empty,
  output logic        rdata_valid
);

  logic [4:0] read_ptr;
  logic [4:0] write_ptr;
  logic [90:0] mem [0:15];
  
  logic [3:0] write_addr;
  logic [3:0] read_addr;
  logic read_enable;

  // Assign combinational logic using continuous assignments
  assign write_addr = write_ptr[3:0];
  assign read_addr = read_ptr[3:0];
  assign fifo_empty = (read_ptr == write_ptr);
  assign read_enable = read_req && (~fifo_empty);

  // Write pointer logic
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      write_ptr <= '0; // SystemVerilog equivalent of all zeros
    end else if (write_enable) begin
      if (!rollover_write) begin
        write_ptr <= write_ptr + 1'b1;
      end else begin // rollover_write is true
        write_ptr <= write_ptr + 5'b00010; // Increment by 2
      end
    end
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

  // Read data valid logic
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      rdata_valid <= 1'b0;
    end else if (read_enable) begin
      rdata_valid <= 1'b1;
    end else begin
      rdata_valid <= 1'b0;
    end
  end
  
  // Read pointer logic
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      read_ptr <= '0; // SystemVerilog equivalent of all zeros
    end else if (read_enable) begin
      read_ptr <= read_ptr + 1'b1;
    end
  end

  // Memory write
  always_ff @(posedge clk) begin
    if (write_enable) begin
      mem[write_addr] <= write_data;
    end
  end

  // Memory Read
  always_ff @(posedge clk) begin
    if (read_enable) begin
      read_data <= mem[read_addr];
    end
  end

endmodule