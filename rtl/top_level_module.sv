// -----------------------------------------------------------------------------
// Module: jpeg_top
//
// Description:
//   This is the top-level module for the JPEG Encoder Core.
//   It connects the `fifo_out` and `ff_checker` submodules to create the final
//   JPEG output stream. It performs the following tasks:
//
//   1. Receives 24-bit encoded input data from `fifo_out`.
//   2. Passes this data to `ff_checker`, which inserts `0x00` after any `0xFF`
//      in the bitstream (as required by the JPEG format).
//   3. At end-of-file, if the bitstream contains fewer than 32 valid bits,
//      it signals this with `eof_data_partial_ready` and outputs the valid bit
//      count using `end_of_file_bitstream_count`.
// -----------------------------------------------------------------------------

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

    // -------------------------------------------------------------------------
    // Internal Signals
    // -------------------------------------------------------------------------
    logic [31:0] JPEG_FF;                  // Output from fifo_out to ff_checker
    logic        data_ready_FF;            // Data ready from fifo_out
    logic [4:0]  orc_reg_in;               // Output register count from fifo_out

    // -------------------------------------------------------------------------
    // Submodule: fifo_out
    // Description:
    //   This module packs and outputs the encoded JPEG bits into a 32-bit word.
    // -------------------------------------------------------------------------
    fifo_out u19 (
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .data_in(data_in),
        .JPEG_bitstream(JPEG_FF),
        .data_ready(data_ready_FF),
        .orc_reg(orc_reg_in)
    );

    // -------------------------------------------------------------------------
    // Submodule: ff_checker
    // Description:
    //   Inserts a 0x00 byte after every 0xFF in the JPEG stream and handles EOF.
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Optional waveform generation for simulation (enabled with +define+TRACE)
    // -------------------------------------------------------------------------
    `ifdef TRACE
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, jpeg_top);
    end
    `endif

endmodule
