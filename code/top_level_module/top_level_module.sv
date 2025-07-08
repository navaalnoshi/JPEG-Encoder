/* This is the top level module of the JPEG Encoder Core.
This module takes the output from the fifo_out module and sends it 
to the ff_checker module to check for FF's in the bitstream.  When it finds an
FF, it puts a 00 after it, and then continues with the rest of the bitstream.
At the end of the file, if there is not a full 32 bit set of JPEG data, then the
signal "eof_data_partial_ready" will go high, to indicate there
are less than 32 valid JPEG bits in the bitstream.  The number of valid bits in the
last JPEG_bitstream value is written to the signal "end_of_file_bitstream_count".
*/
`timescale 1ns / 100ps

module jpeg_top (
    input  logic         clk,
    input  logic         rst,
    input  logic         end_of_file_signal,
    input  logic         enable,
    input  logic [23:0]  data_in,
    output logic [31:0]  JPEG_bitstream,
    output logic         data_ready,
    output logic [4:0]   end_of_file_bitstream_count,
    output logic         eof_data_partial_ready
);

    // Internal wires
    logic [31:0] JPEG_FF;
    logic        data_ready_FF;
    logic [4:0]  orc_reg_in;

    // Instantiate fifo_out module
    fifo_out u19 (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .data_in(data_in),
        .JPEG_bitstream(JPEG_FF),
        .data_ready(data_ready_FF),
        .orc_reg(orc_reg_in)
    );

    // Instantiate ff_checker module
    ff_checker u20 (
        .clk(clk),
        .rst(rst),
        .end_of_file_signal(end_of_file_signal),
        .JPEG_in(JPEG_FF),
        .data_ready_in(data_ready_FF),
        .orc_reg_in(orc_reg_in),
        .JPEG_bitstream_1(JPEG_bitstream),
        .data_ready_1(data_ready),
        .orc_reg(end_of_file_bitstream_count),
        .eof_data_partial_ready(eof_data_partial_ready)
    );

    `ifdef TRACE
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, jpeg_top);
    end
    `endif

endmodule