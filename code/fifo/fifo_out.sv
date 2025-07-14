/* ---------------------------------------------------------------------------
Module: fifo_out

Description:
    This module combines Y, Cb, and Cr JPEG encoded bitstreams into a single 
    32-bit JPEG output stream. It uses three FIFOs—one each for Y, Cb, and Cr—
    to buffer and multiplex encoded data. It also aligns data according to the
    number of remaining output register count (ORC) bits and applies necessary
    shifting and padding. The output of this module connects to the ff_checker
    module for byte stuffing (e.g., inserting 0x00 after 0xFF).

    The logic includes multiple pipelining stages and multiplexing to handle
    overlapping bitstreams and end-of-block signals. The module operates on a 
    strict clocked pipeline to ensure timing alignment for JPEG formatting.
--------------------------------------------------------------------------- */
`timescale 1ns / 100ps

module fifo_out(
    input   logic         clk,          // Clock signal
    input   logic         rst,          // Asynchronous reset signal (active high)
    input   logic         enable,       // Master enable for the module
    input   logic [23:0]  data_in,      // Input data from the pre_fifo module
    output  logic [31:0]  JPEG_bitstream, // Output JPEG bitstream
    output  logic         data_ready,   // Indicates when a new 32-bit JPEG_bitstream is ready
    output  logic [4:0]   orc_reg       // Output register for the 'offset run count'
);

    // -------------------------------------------------------------------------------
    // Internal Signal Declarations
     // -------------------------------------------------------------------------------

    // Data and ORC (Offset Run Count) signals from the pre_fifo module
    
    logic [31:0]  cb_JPEG_bitstream, cr_JPEG_bitstream, y_JPEG_bitstream; // 32-bit JPEG data from pre_fifo for Cb, Cr, Y
    logic [4:0]   cr_orc, cb_orc, y_orc; // 5-bit ORC values from pre_fifo for Cr, Cb, Y

    // Output data and enable signals from the Y FIFO
    logic [31:0]  y_bits_out;     // 32-bit data read from the Y FIFO
    logic         y_out_enable;   // Indicates valid data read from Y FIFO

    // Data ready and FIFO empty signals for FIFOs
    logic         cb_data_ready, cr_data_ready, y_data_ready; // Indicates when data is ready from pre_fifo for writing to respective FIFOs
    logic         end_of_block_output; // Indicates end of a block from pre_fifo (typically 8x8 block)
    logic         y_eob_empty;         // Y FIFO End-of-Block (EOB) status (empty implies no EOB marker received)
    logic         cb_eob_empty;        // Cb FIFO EOB status
    logic         cr_eob_empty;        // Cr FIFO EOB status
    logic         y_fifo_empty;        // Y FIFO empty status

    // Internal ORC registers and pipeline stages for ORC calculation and tracking
    logic [4:0]   orc;          // ORC for Y component (calculated based on previous components)
    logic [4:0]   orc_cb;       // ORC for Cb component
    logic [4:0]   orc_cr;       // ORC for Cr component
    logic [4:0]   old_orc_reg;  // Stores a previous ORC value for rollover calculations
    logic [4:0]   sorc_reg;     // Selected ORC for current active component
    logic [4:0]   roll_orc_reg; // ORC used for rollover comparison

    // Pipelined ORC values for shifting/processing the bitstream
    logic [4:0]   orc_1, orc_2, orc_3, orc_4, orc_5;
    logic [4:0]   orc_reg_delay; // Delayed version of orc_reg for bit manipulation

    // Pipelined static ORC values (likely for output bit enabling)
    logic [4:0]   static_orc_1, static_orc_2, static_orc_3, static_orc_4, static_orc_5;
    logic [4:0]   static_orc_6;

    // Pipelined edge run-off values (for bit manipulation during shifting)
    logic [4:0]   edge_ro_1, edge_ro_2, edge_ro_3, edge_ro_4, edge_ro_5;

    // Pipelined JPEG data for bitstream construction and shifting
    logic [31:0]  jpeg_ro_1, jpeg_ro_2, jpeg_ro_3, jpeg_ro_4, jpeg_ro_5, jpeg_delay;

    // Main JPEG data registers in the pipeline
    logic [31:0]  jpeg;     // Current JPEG data selected from Y, Cb, Cr FIFOs
    logic [31:0]  jpeg_1, jpeg_2, jpeg_3, jpeg_4, jpeg_5, jpeg_6; // Pipelined stages of JPEG data

    // Delayed ORC and enable signals from the FIFOs
    logic [4:0]   cr_orc_1, cb_orc_1, y_orc_1; // Delayed ORC values from pre_fifo
    logic         cr_out_enable_1, cb_out_enable_1, y_out_enable_1; // Delayed FIFO read enables

    // End-of-Block (EOB) pipeline stages
    logic         eob_1, eob_2, eob_3, eob_4;

    // Generic enable pipeline stages (used for controlling muxes and ORC calculations)
    logic         enable_1, enable_2, enable_3, enable_4, enable_5;
    logic         enable_6, enable_7, enable_8, enable_9, enable_10;
    logic         enable_11, enable_12, enable_13, enable_14, enable_15;
    logic         enable_16, enable_17, enable_18, enable_19, enable_20;
    logic         enable_21, enable_22, enable_23, enable_24, enable_25;
    logic         enable_26, enable_27, enable_28, enable_29, enable_30;
    logic         enable_31, enable_32, enable_33, enable_34, enable_35;

    // Mux select signals for controlling data flow based on Y/Cb/Cr components
    logic [2:0]   bits_mux;     // Selects which component's data (Y, Cb, Cr) is currently active
    logic [2:0]   old_orc_mux;  // Selects which component's ORC is considered 'old' for rollover
    logic [2:0]   read_mux;     // Selects which FIFO to send a read request to

    // Data ready and rollover control signals
    logic         bits_ready;   // Indicates when current component's bits are valid
    logic         br_1, br_2, br_3, br_4, br_5, br_6, br_7, br_8; // Pipelined 'bits_ready'
    logic         rollover;     // Indicates a bitstream rollover condition
    logic         rollover_1, rollover_2, rollover_3, rollover_eob; // Pipelined rollover
    logic         rollover_4, rollover_5, rollover_6, rollover_7;
    logic         eobe_1;       // Delayed Y EOB empty (End Of Block Empty)
    logic         cb_read_req, cr_read_req, y_read_req; // FIFO read request signals
    logic         eob_early_out_enable; // Early EOB detection for enabling output
    logic         fifo_mux;     // Mux for switching between two sets of Cb/Cr FIFOs (likely for double buffering)

    // Signals for routing data to the correct Cb/Cr FIFO instance based on fifo_mux
    logic [31:0] cr_bits_out1, cr_bits_out2; // Outputs from Cr FIFO instances
    logic [31:0] cb_bits_out1, cb_bits_out2; // Outputs from Cb FIFO instances
    logic        cr_fifo_empty1, cr_fifo_empty2; // Empty flags for Cr FIFO instances
    logic        cb_fifo_empty1, cb_fifo_empty2; // Empty flags for Cb FIFO instances
    logic        cr_out_enable1, cr_out_enable2; // Read enable flags for Cr FIFO instances
    logic        cb_out_enable1, cb_out_enable2; // Read enable flags for Cb FIFO instances

     // -------------------------------------------------------------------------------
    // Muxed FIFO read requests for Cb/Cr FIFOs.
    // These signals determine which of the two FIFO instances (1 or 2) for Cb/Cr
    // should receive the read request. Only one is active at a time based on `fifo_mux`.
     // -------------------------------------------------------------------------------
    
    logic cr_read_req_out1; // Read request for Cr FIFO instance 1
    logic cr_read_req_out2; // Read request for Cr FIFO instance 2
    logic cb_read_req_out1; // Read request for Cb FIFO instance 1
    logic cb_read_req_out2; // Read request for Cb FIFO instance 2

     // -------------------------------------------------------------------------------
    // FIFO write enable signals. A FIFO is written to if data is ready from pre_fifo
    // and its corresponding EOB (End-of-Block) buffer is not empty (meaning there's
    // still EOB information to process for that component).
     // -------------------------------------------------------------------------------
    
    assign cb_write_enable = cb_data_ready && !cb_eob_empty;
    assign cr_write_enable = cr_data_ready && !cr_eob_empty;
    assign y_write_enable = y_data_ready && !y_eob_empty;

     // -------------------------------------------------------------------------------
    // FIFO read request routing based on `fifo_mux`. This effectively directs
    // the single `cr_read_req` or `cb_read_req` to one of two FIFO instances.
     // -------------------------------------------------------------------------------
    
    assign cr_read_req_out1 = fifo_mux ? 1'b0 : cr_read_req; // If fifo_mux is 1, cr_read_req1 is 0, else cr_read_req
    assign cr_read_req_out2 = fifo_mux ? cr_read_req : 1'b0; // If fifo_mux is 1, cr_read_req2 is cr_read_req, else 0

     // -------------------------------------------------------------------------------
    // FIFO write enable routing based on `fifo_mux`. This ensures data is written
    // to the currently selected FIFO instance.
     // -------------------------------------------------------------------------------
    
    assign cr_write_enable1 = fifo_mux && cr_write_enable;     // If fifo_mux is 1, enable write to FIFO1
    assign cr_write_enable2 = !fifo_mux && cr_write_enable;    // If fifo_mux is 0, enable write to FIFO2

    assign cb_read_req_out1 = fifo_mux ? 1'b0 : cb_read_req;
    assign cb_read_req_out2 = fifo_mux ? cb_read_req : 1'b0;

    assign cb_write_enable1 = fifo_mux && cb_write_enable;
    assign cb_write_enable2 = !fifo_mux && cb_write_enable;
    
     // -------------------------------------------------------------------------------
    // Muxed `write_data` for Cb/Cr FIFOs. When `fifo_mux` is 1, `cb_JPEG_bitstream`
    // (from pre_fifo) is routed to FIFO2's `write_data` through `cb_JPEG_bitstream_sel`,
    // and FIFO1's `write_data` gets 0 (don't care). This is a common pattern for
    // double-buffered writes.
    // -------------------------------------------------------------------------------
    logic [31:0] cr_JPEG_bitstream_sel;
    logic [31:0] cb_JPEG_bitstream_sel;

    assign cr_JPEG_bitstream_sel = fifo_mux ? cr_JPEG_bitstream : 32'b0;
    assign cb_JPEG_bitstream_sel = fifo_mux ? cb_JPEG_bitstream : 32'b0;

    // Muxed outputs from Cb/Cr FIFOs. The `fifo_mux` selects which FIFO's output
    // (read_data, fifo_empty, rdata_valid) is passed to the downstream logic.
    logic [31:0] cr_bits_out_mux;
    logic [31:0] cb_bits_out_mux;
    logic        cr_fifo_empty_mux;
    logic        cb_fifo_empty_mux;
    logic        cr_out_enable_mux;
    logic        cb_out_enable_mux;

    assign cr_bits_out_mux = fifo_mux ? cr_bits_out2 : cr_bits_out1;
    assign cr_fifo_empty_mux = fifo_mux ? cr_fifo_empty2 : cr_fifo_empty1;
    assign cr_out_enable_mux = fifo_mux ? cr_out_enable2 : cr_out_enable1;

    assign cb_bits_out_mux = fifo_mux ? cb_bits_out2 : cb_bits_out1;
    assign cb_fifo_empty_mux = fifo_mux ? cb_fifo_empty2 : cb_fifo_empty1;
    assign cb_out_enable_mux = fifo_mux ? cb_out_enable2 : cb_out_enable1;


    // Instance of the pre_fifo module (generates Y, Cb, Cr data and ORC)
    pre_fifo u14(
        .clk(clk),
        .rst(rst),
        .enable(enable),
        .data_in(data_in),
        .cr_JPEG_bitstream(cr_JPEG_bitstream),
        .cr_data_ready(cr_data_ready),
        .cr_orc(cr_orc),
        .cb_JPEG_bitstream(cb_JPEG_bitstream),
        .cb_data_ready(cb_data_ready),
        .cb_orc(cb_orc),
        .y_JPEG_bitstream(y_JPEG_bitstream),
        .y_data_ready(y_data_ready),
        .y_orc(y_orc),
        .y_eob_output(end_of_block_output),
        .y_eob_empty(y_eob_empty),
        .cb_eob_empty(cb_eob_empty),
        .cr_eob_empty(cr_eob_empty)
    );

    // Two instances of 32-bit synchronous FIFOs for Cb data (for double buffering)
    sync_fifo_32 u15( // Cb FIFO Instance 1
        .clk(clk),
        .rst(rst),
        .read_req(cb_read_req_out1),      // Muxed read request
        .write_data(cb_JPEG_bitstream_sel), // Muxed write data
        .write_enable(cb_write_enable1), // Muxed write enable
        .read_data(cb_bits_out1),
        .fifo_empty(cb_fifo_empty1),
        .rdata_valid(cb_out_enable1)
    );

    sync_fifo_32 u25( // Cb FIFO Instance 2
        .clk(clk),
        .rst(rst),
        .read_req(cb_read_req_out2),
        .write_data(cb_JPEG_bitstream_sel),
        .write_enable(cb_write_enable2),
        .read_data(cb_bits_out2),
        .fifo_empty(cb_fifo_empty2),
        .rdata_valid(cb_out_enable2)
    );

    // Two instances of 32-bit synchronous FIFOs for Cr data (for double buffering)
    sync_fifo_32 u16( // Cr FIFO Instance 1
        .clk(clk),
        .rst(rst),
        .read_req(cr_read_req_out1),
        .write_data(cr_JPEG_bitstream_sel),
        .write_enable(cr_write_enable1),
        .read_data(cr_bits_out1),
        .fifo_empty(cr_fifo_empty1),
        .rdata_valid(cr_out_enable1)
    );

    sync_fifo_32 u24( // Cr FIFO Instance 2
        .clk(clk),
        .rst(rst),
        .read_req(cr_read_req_out2),
        .write_data(cr_JPEG_bitstream_sel),
        .write_enable(cr_write_enable2),
        .read_data(cr_bits_out2),
        .fifo_empty(cr_fifo_empty2),
        .rdata_valid(cr_out_enable2)
    );

    // Single instance of 32-bit synchronous FIFO for Y data
    sync_fifo_32 u17(
        .clk(clk),
        .rst(rst),
        .read_req(y_read_req),
        .write_data(y_JPEG_bitstream),
        .write_enable(y_write_enable),
        .read_data(y_bits_out),
        .fifo_empty(y_fifo_empty),
        .rdata_valid(y_out_enable)
    );

    
