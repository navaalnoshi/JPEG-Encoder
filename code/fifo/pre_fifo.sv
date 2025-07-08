/* This module combines the Y, Cb, and Cr blocks, and the RGB to Y, Cb, and Cr
converter. */
`timescale 1ns / 100ps

module pre_fifo (
    input  logic         clk,
    input  logic         rst,
    input  logic         enable,
    input  logic [23:0]  data_in,

    output logic [31:0]  cr_JPEG_bitstream,
    output logic         cr_data_ready,
    output logic [4:0]   cr_orc,

    output logic [31:0]  cb_JPEG_bitstream,
    output logic         cb_data_ready,
    output logic [4:0]   cb_orc,

    output logic [31:0]  y_JPEG_bitstream,
    output logic         y_data_ready,
    output logic [4:0]   y_orc,

    output logic         y_eob_output,
    output logic         y_eob_empty,
    output logic         cb_eob_empty,
    output logic         cr_eob_empty
);

// Internal signals
logic [23:0] dct_data_in;
logic        rgb_enable;

// Module instantiations
RGB2YCBCR u4 (
    .clk(clk),
    .rst(rst),
    .enable(enable),
    .data_in(data_in),
    .data_out(dct_data_in),
    .enable_out(rgb_enable)
);

crd_q_h u11 (
    .clk(clk),
    .rst(rst),
    .enable(rgb_enable),
    .data_in(dct_data_in[23:16]),
    .JPEG_bitstream(cr_JPEG_bitstream),
    .data_ready(cr_data_ready),
    .cr_orc(cr_orc),
    .end_of_block_empty(cr_eob_empty)
);

cbd_q_h u12 (
    .clk(clk),
    .rst(rst),
    .enable(rgb_enable),
    .data_in(dct_data_in[15:8]),
    .JPEG_bitstream(cb_JPEG_bitstream),
    .data_ready(cb_data_ready),
    .cb_orc(cb_orc),
    .end_of_block_empty(cb_eob_empty)
);

yd_q_h u13 (
    .clk(clk),
    .rst(rst),
    .enable(rgb_enable),
    .data_in(dct_data_in[7:0]),
    .JPEG_bitstream(y_JPEG_bitstream),
    .data_ready(y_data_ready),
    .y_orc(y_orc),
    .end_of_block_output(y_eob_output),
    .end_of_block_empty(y_eob_empty)
);

endmodule
