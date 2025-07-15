/*
--------------------------------------------------------------------------------
Module: pre_fifo
Description:
  - Top-level pre-processing module that:
      • Converts incoming 24-bit RGB data to Y, Cb, Cr format using RGB2YCBCR.
      • Sends Y, Cb, Cr values to their respective quantization and DCT modules.
      • Outputs bitstreams and control signals for each color channel.
  - Works in JPEG encoding pipeline after RGB input but before FIFO muxing.
  - Enables pipelined architecture for Y/Cb/Cr encoding.
--------------------------------------------------------------------------------
*/

`timescale 1ns / 100ps

module pre_fifo (
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [23:0] data_in,

    output logic [31:0] cr_JPEG_bitstream,
    output logic        cr_data_ready,
    output logic [4:0]  cr_orc,

    output logic [31:0] cb_JPEG_bitstream,
    output logic        cb_data_ready,
    output logic [4:0]  cb_orc,

    output logic [31:0] y_JPEG_bitstream,
    output logic        y_data_ready,
    output logic [4:0]  y_orc,

    output logic        y_eob_output,
    output logic        y_eob_empty,
    output logic        cb_eob_empty,
    output logic        cr_eob_empty
);

    // ------------------------------------------------------------------------
    // Internal Signals
    // ------------------------------------------------------------------------

    logic [23:0] dct_data_in;   // Output from RGB2YCBCR: packed YCbCr
    logic        rgb_enable;    // Enable from RGB2YCBCR to control downstream DCT blocks

    // ------------------------------------------------------------------------
    // RGB to YCbCr Conversion Module
    // Converts RGB input to 8-bit Y, Cb, and Cr components.
    // Output is packed as: [23:16] = Cr, [15:8] = Cb, [7:0] = Y
    // ------------------------------------------------------------------------

    rgb2ycrcb u4 (
        .clk        (clk),
        .rst        (rst),
        .enable     (enable),
        .data_in    (data_in),
        .data_out   (dct_data_in),
        .enable_out (rgb_enable)
    );

    // ------------------------------------------------------------------------
    // Cr Component Quantization, DCT, Huffman Encoding Module
    // Processes Cr byte and outputs its JPEG bitstream
    // ------------------------------------------------------------------------

    crd_q_h u11 (
        .clk               (clk),
        .rst               (rst),
        .enable            (rgb_enable),
        .data_in           (dct_data_in[23:16]),     // Cr component
        .JPEG_bitstream    (cr_JPEG_bitstream),
        .data_ready        (cr_data_ready),
        .cr_orc            (cr_orc),
        .end_of_block_empty(cr_eob_empty)
    );

    // ------------------------------------------------------------------------
    // Cb Component Quantization, DCT, Huffman Encoding Module
    // Processes Cb byte and outputs its JPEG bitstream
    // ------------------------------------------------------------------------

    cbd_q_h u12 (
        .clk               (clk),
        .rst               (rst),
        .enable            (rgb_enable),
        .data_in           (dct_data_in[15:8]),      // Cb component
        .JPEG_bitstream    (cb_JPEG_bitstream),
        .data_ready        (cb_data_ready),
        .cb_orc            (cb_orc),
        .end_of_block_empty(cb_eob_empty)
    );

    // ------------------------------------------------------------------------
    // Y Component Quantization, DCT, Huffman Encoding Module
    // Processes Y byte and outputs its JPEG bitstream
    // Also provides end-of-block (EOB) signals for synchronization
    // ------------------------------------------------------------------------

    yd_q_h u13 (
        .clk               (clk),
        .rst               (rst),
        .enable            (rgb_enable),
        .data_in           (dct_data_in[7:0]),       // Y component
        .JPEG_bitstream    (y_JPEG_bitstream),
        .data_ready        (y_data_ready),
        .y_orc             (y_orc),
        .end_of_block_output(y_eob_output),
        .end_of_block_empty(y_eob_empty)
    );

endmodule
