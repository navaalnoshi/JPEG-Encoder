timescale 1ns / 100ps

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

  logic [4:0] read_ptr;
  logic [4:0] write_ptr;
  logic [31:0] mem [0:15];
  
  logic [3:0] write_addr;
  logic [3:0] read_addr;
  logic read_enable;

  // Assign combinational logic using always_comb or continuous assignments
  assign write_addr = write_ptr[3:0];
  assign read_addr = read_ptr[3:0];
  assign fifo_empty = (read_ptr == write_ptr);
  assign read_enable = read_req && (~fifo_empty);

  // Write pointer logic
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      write_ptr <= '0; // SystemVerilog equivalent of all zeros
    end else if (write_enable) begin
      write_ptr <= write_ptr + 1'b1;
    end
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