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
    input         clk,         // Clock input
    input         rst,         // Reset input
    input         enable,      // Enable signal
    input  [23:0] data_in,     // 24-bit input data
    output [31:0] JPEG_bitstream, // 32-bit JPEG bitstream output
    output        data_ready,  // Data ready signal
    output [4:0]  orc_reg      // Output register for orc value
);

// Wires for JPEG bitstream and orc values from pre_fifo
wire  [31:0]  cb_JPEG_bitstream, cr_JPEG_bitstream, y_JPEG_bitstream;
wire  [4:0]   cr_orc, cb_orc, y_orc;

// Wires for Y component output from FIFO and enable signal
wire  [31:0]  y_bits_out;
wire          y_out_enable;

// Wires for data ready signals from pre_fifo and FIFO empty status
wire          cb_data_ready, cr_data_ready, y_data_ready;
wire          end_of_block_output, y_eob_empty;
wire          cb_eob_empty, cr_eob_empty;
wire          y_fifo_empty;

// Registers for orc values and their delayed versions
reg  [4:0]    orc, orc_reg, orc_cb, orc_cr, old_orc_reg, sorc_reg, roll_orc_reg;
reg  [4:0]    orc_1, orc_2, orc_3, orc_4, orc_5, orc_reg_delay;
reg  [4:0]    static_orc_1, static_orc_2, static_orc_3, static_orc_4, static_orc_5;
reg  [4:0]    static_orc_6;

// Registers for edge rollover and JPEG bitstream values and their delayed versions
reg  [4:0]    edge_ro_1, edge_ro_2, edge_ro_3, edge_ro_4, edge_ro_5;
reg  [31:0]   jpeg_ro_1, jpeg_ro_2, jpeg_ro_3, jpeg_ro_4, jpeg_ro_5, jpeg_delay;

// Registers for JPEG bitstream and its delayed versions
reg  [31:0]   jpeg, jpeg_1, jpeg_2, jpeg_3, jpeg_4, jpeg_5, jpeg_6, JPEG_bitstream;

// Registers for delayed orc values and enable signals
reg  [4:0]    cr_orc_1, cb_orc_1, y_orc_1;
reg           cr_out_enable_1, cb_out_enable_1, y_out_enable_1, eob_1;
reg           eob_2, eob_3, eob_4;

// Registers for various enable signals
reg           enable_1, enable_2, enable_3, enable_4, enable_5;
reg           enable_6, enable_7, enable_8, enable_9, enable_10;
reg           enable_11, enable_12, enable_13, enable_14, enable_15;
reg           enable_16, enable_17, enable_18, enable_19, enable_20;
reg           enable_21, enable_22, enable_23, enable_24, enable_25;
reg           enable_26, enable_27, enable_28, enable_29, enable_30;
reg           enable_31, enable_32, enable_33, enable_34, enable_35;

// Registers for mux control and bit ready signals
reg  [2:0]    bits_mux, old_orc_mux, read_mux;
reg           bits_ready, br_1, br_2, br_3, br_4, br_5, br_6, br_7, br_8;

// Registers for rollover conditions and EOB signals
reg           rollover, rollover_1, rollover_2, rollover_3, rollover_eob;
reg           rollover_4, rollover_5, rollover_6, rollover_7;
reg           data_ready, eobe_1, cb_read_req, cr_read_req, y_read_req;
reg           eob_early_out_enable, fifo_mux;

// Wires for CR and CB FIFO outputs and empty/enable signals (for dual FIFOs)
wire [31:0]   cr_bits_out1, cr_bits_out2, cb_bits_out1, cb_bits_out2;
wire          cr_fifo_empty1, cr_fifo_empty2, cb_fifo_empty1, cb_fifo_empty2;
wire          cr_out_enable1, cr_out_enable2, cb_out_enable1, cb_out_enable2;

// Write enable signals for FIFOs, based on data ready and EOB empty
wire cb_write_enable = cb_data_ready && !cb_eob_empty;
wire cr_write_enable = cr_data_ready && !cr_eob_empty;
wire y_write_enable = y_data_ready && !y_eob_empty;

// FIFO muxing logic for CR component (distributing read/write to two FIFOs)
wire cr_read_req1 = fifo_mux ? 1'b0 : cr_read_req;
wire cr_read_req2 = fifo_mux ? cr_read_req : 1'b0;
wire [31:0] cr_JPEG_bitstream1 = fifo_mux ? cr_JPEG_bitstream : 32'b0;
wire [31:0] cr_JPEG_bitstream2 = fifo_mux ? 32'b0 : cr_JPEG_bitstream;
wire cr_write_enable1 = fifo_mux && cr_write_enable;
wire cr_write_enable2 = !fifo_mux && cr_write_enable;
wire [31:0] cr_bits_out = fifo_mux ? cr_bits_out2 : cr_bits_out1;
wire cr_fifo_empty = fifo_mux ? cr_fifo_empty2 : cr_fifo_empty1;
wire cr_out_enable = fifo_mux ? cr_out_enable2 : cr_out_enable1;

// FIFO muxing logic for CB component (distributing read/write to two FIFOs)
wire cb_read_req1 = fifo_mux ? 1'b0 : cb_read_req;
wire cb_read_req2 = fifo_mux ? cb_read_req : 1'b0;
wire [31:0] cb_JPEG_bitstream1 = fifo_mux ? cb_JPEG_bitstream : 32'b0;
wire [31:0] cb_JPEG_bitstream2 = fifo_mux ? 32'b0 : cb_JPEG_bitstream;
wire cb_write_enable1 = fifo_mux && cb_write_enable;
wire cb_write_enable2 = !fifo_mux && cb_write_enable;
wire [31:0] cb_bits_out = fifo_mux ? cb_bits_out2 : cb_bits_out1;
wire cb_fifo_empty = fifo_mux ? cb_fifo_empty2 : cb_fifo_empty1;
wire cb_out_enable = fifo_mux ? cb_out_enable2 : cb_out_enable1;

// Instance of pre_fifo module to get Y, Cb, Cr data and EOB signals
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

// Instances of sync_fifo_32 for CB component (two FIFOs for muxing)
sync_fifo_32 u15(
    .clk(clk),
    .rst(rst),
    .read_req(cb_read_req1),
    .write_data(cb_JPEG_bitstream1),
    .write_enable(cb_write_enable1),
    .read_data(cb_bits_out1),
    .fifo_empty(cb_fifo_empty1),
    .rdata_valid(cb_out_enable1)
);

sync_fifo_32 u25(
    .clk(clk),
    .rst(rst),
    .read_req(cb_read_req2),
    .write_data(cb_JPEG_bitstream2),
    .write_enable(cb_write_enable2),
    .read_data(cb_bits_out2),
    .fifo_empty(cb_fifo_empty2),
    .rdata_valid(cb_out_enable2)
);

// Instances of sync_fifo_32 for CR component (two FIFOs for muxing)
sync_fifo_32 u16(
    .clk(clk),
    .rst(rst),
    .read_req(cr_read_req1),
    .write_data(cr_JPEG_bitstream1),
    .write_enable(cr_write_enable1),
    .read_data(cr_bits_out1),
    .fifo_empty(cr_fifo_empty1),
    .rdata_valid(cr_out_enable1)
);

sync_fifo_32 u24(
    .clk(clk),
    .rst(rst),
    .read_req(cr_read_req2),
    .write_data(cr_JPEG_bitstream2),
    .write_enable(cr_write_enable2),
    .read_data(cr_bits_out2),
    .fifo_empty(cr_fifo_empty2),
    .rdata_valid(cr_out_enable2)
);

// Instance of sync_fifo_32 for Y component
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

// Sequential logic for fifo_mux: toggles on end_of_block_output
always @(posedge clk) begin
    if (rst)
        fifo_mux <= 1'b0;
    else if (end_of_block_output)
        fifo_mux <= ~fifo_mux; // Toggles between 0 and 1
end

// Sequential logic for y_read_req: asserts when Y FIFO is not empty and read_mux selects Y
always @(posedge clk) begin
    if (y_fifo_empty || read_mux != 3'b001)
        y_read_req <= 1'b0;
    else if (!y_fifo_empty && read_mux == 3'b001)
        y_read_req <= 1'b1;
end

// Sequential logic for cb_read_req: asserts when CB FIFO is not empty and read_mux selects Cb
always @(posedge clk) begin
    if (cb_fifo_empty || read_mux != 3'b010)
        cb_read_req <= 1'b0;
    else if (!cb_fifo_empty && read_mux == 3'b010)
        cb_read_req <= 1'b1;
end

// Sequential logic for cr_read_req: asserts when CR FIFO is not empty and read_mux selects Cr
always @(posedge clk) begin
    if (cr_fifo_empty || read_mux != 3'b100)
        cr_read_req <= 1'b0;
    else if (!cr_fifo_empty && read_mux == 3'b100)
        cr_read_req <= 1'b1;
end

// Sequential logic for bit ready (br) signals and static orc values, and data_ready
always @(posedge clk) begin
    if (rst) begin
        br_1 <= 1'b0; br_2 <= 1'b0; br_3 <= 1'b0; br_4 <= 1'b0; br_5 <= 1'b0; br_6 <= 1'b0;
        br_7 <= 1'b0; br_8 <= 1'b0;
        static_orc_1 <= 5'b0; static_orc_2 <= 5'b0; static_orc_3 <= 5'b0;
        static_orc_4 <= 5'b0; static_orc_5 <= 5'b0; static_orc_6 <= 5'b0;
        data_ready <= 1'b0; eobe_1 <= 1'b0;
    end else begin
        br_1 <= bits_ready & !eobe_1; // br_1 is bits_ready unless eob_early_out is enabled
        br_2 <= br_1; br_3 <= br_2; // Delayed versions of br_1
        br_4 <= br_3; br_5 <= br_4; br_6 <= br_5;
        br_7 <= br_6; br_8 <= br_7;
        static_orc_1 <= sorc_reg; static_orc_2 <= static_orc_1; // Delayed versions of sorc_reg
        static_orc_3 <= static_orc_2; static_orc_4 <= static_orc_3;
        static_orc_5 <= static_orc_4; static_orc_6 <= static_orc_5;
        data_ready <= br_6 & rollover_5; // Data ready when br_6 and rollover_5 are high
        eobe_1 <= y_eob_empty; // eobe_1 is high when Y EOB FIFO is empty
    end
end

// Sequential logic for rollover_eob: indicates rollover based on old_orc_reg and roll_orc_reg
always @(posedge clk) begin
    if (rst)
        rollover_eob <= 1'b0;
    else if (br_3)
        rollover_eob <= old_orc_reg >= roll_orc_reg;
end

// Sequential logic for rollover signals and EOB signals (delayed versions)
always @(posedge clk) begin
    if (rst) begin
        rollover_1 <= 1'b0; rollover_2 <= 1'b0; rollover_3 <= 1'b0;
        rollover_4 <= 1'b0; rollover_5 <= 1'b0; rollover_6 <= 1'b0;
        rollover_7 <= 1'b0; eob_1 <= 1'b0; eob_2 <= 1'b0;
        eob_3 <= 1'b0; eob_4 <= 1'b0;
        eob_early_out_enable <= 1'b0;
    end else begin
        rollover_1 <= rollover; rollover_2 <= rollover_1; // Delayed versions of rollover
        rollover_3 <= rollover_2;
        rollover_4 <= rollover_3 | rollover_eob; // rollover_4 is high if rollover_3 or rollover_eob is high
        rollover_5 <= rollover_4; rollover_6 <= rollover_5; // Delayed versions of rollover_4
        rollover_7 <= rollover_6; eob_1 <= end_of_block_output; // eob_1 from end_of_block_output
        eob_2 <= eob_1; eob_3 <= eob_2; eob_4 <= eob_3; // Delayed versions of eob_1
        eob_early_out_enable <= y_out_enable & y_out_enable_1 & eob_2; // Enables early EOB output
    end
end

// Sequential logic for rollover based on bits_mux selection and enable signals
always @(posedge clk) begin
    case (bits_mux)
        3'b001:  rollover <= y_out_enable_1 & !eob_4 & !eob_early_out_enable; // Y component
        3'b010:  rollover <= cb_out_enable_1 & cb_out_enable; // Cb component
        3'b100:  rollover <= cr_out_enable_1 & cr_out_enable; // Cr component
        default: rollover <= y_out_enable_1 & !eob_4; // Default to Y component
    endcase
end

// Sequential logic for orc (Y component's orc)
always @(posedge clk) begin
    if (rst)
        orc <= 5'b0;
    else if (enable_20)
        orc <= orc_cr + cr_orc_1;
end

// Sequential logic for orc_cb (Cb component's orc)
always @(posedge clk) begin
    if (rst)
        orc_cb <= 5'b0;
    else if (eob_1)
        orc_cb <= orc + y_orc_1;
end

// Sequential logic for orc_cr (Cr component's orc)
always @(posedge clk) begin
    if (rst)
        orc_cr <= 5'b0;
    else if (enable_5)
        orc_cr <= orc_cb + cb_orc_1;
end

// Sequential logic for delayed out_enable signals (cr, cb, y)
always @(posedge clk) begin
    if (rst) begin
        cr_out_enable_1 <= 1'b0; cb_out_enable_1 <= 1'b0; y_out_enable_1 <= 1'b0;
    end else begin
        cr_out_enable_1 <= cr_out_enable;
        cb_out_enable_1 <= cb_out_enable;
        y_out_enable_1 <= y_out_enable;
    end
end

// Sequential logic for jpeg: selects the appropriate bitstream based on bits_mux
always @(posedge clk) begin
    case (bits_mux)
        3'b001:  jpeg <= y_bits_out; // Y component bitstream
        3'b010:  jpeg <= cb_bits_out; // Cb component bitstream
        3'b100:  jpeg <= cr_bits_out; // Cr component bitstream
        default: jpeg <= y_bits_out; // Default to Y component bitstream
    endcase
end

// Sequential logic for bits_ready: indicates if the selected component's bits are ready
always @(posedge clk) begin
    case (bits_mux)
        3'b001:  bits_ready <= y_out_enable; // Y component
        3'b010:  bits_ready <= cb_out_enable; // Cb component
        3'b100:  bits_ready <= cr_out_enable; // Cr component
        default: bits_ready <= y_out_enable; // Default to Y component
    endcase
end

// Sequential logic for sorc_reg: selects the appropriate orc value based on bits_mux
always @(posedge clk) begin
    case (bits_mux)
        3'b001:  sorc_reg <= orc; // Y component orc
        3'b010:  sorc_reg <= orc_cb; // Cb component orc
        3'b100:  sorc_reg <= orc_cr; // Cr component orc
        default: sorc_reg <= orc; // Default to Y component orc
    endcase
end

// Sequential logic for roll_orc_reg: selects the appropriate orc value for rollover based on old_orc_mux
always @(posedge clk) begin
    case (old_orc_mux)
        3'b001:  roll_orc_reg <= orc; // Y component orc
        3'b010:  roll_orc_reg <= orc_cb; // Cb component orc
        3'b100:  roll_orc_reg <= orc_cr; // Cr component orc
        default: roll_orc_reg <= orc; // Default to Y component orc
    endcase
end

// Sequential logic for orc_reg: selects the appropriate orc value based on bits_mux
always @(posedge clk) begin
    case (bits_mux)
        3'b001:  orc_reg <= orc; // Y component orc
        3'b010:  orc_reg <= orc_cb; // Cb component orc
        3'b100:  orc_reg <= orc_cr; // Cr component orc
        default: orc_reg <= orc; // Default to Y component orc
    endcase
end

// Sequential logic for old_orc_reg: selects the appropriate orc value based on old_orc_mux
always @(posedge clk) begin
    case (old_orc_mux)
        3'b001:  old_orc_reg <= orc_cr; // Cr component orc
        3'b010:  old_orc_reg <= orc;    // Y component orc
        3'b100:  old_orc_reg <= orc_cb; // Cb component orc
        default: old_orc_reg <= orc_cr; // Default to Cr component orc
    endcase
end

// Sequential logic for bits_mux: controls which component (Y, Cb, Cr) is currently being processed
always @(posedge clk) begin
    if (rst)
        bits_mux <= 3'b001; // Y component (initial state)
    else if (enable_3)
        bits_mux <= 3'b010; // Cb component
    else if (enable_19)
        bits_mux <= 3'b100; // Cr component
    else if (enable_35)
        bits_mux <= 3'b001; // Y component (wrap around)
end

// Sequential logic for old_orc_mux: controls which old orc value is used for rollover
always @(posedge clk) begin
    if (rst)
        old_orc_mux <= 3'b001; // Y component (initial state)
    else if (enable_1)
        old_orc_mux <= 3'b010; // Cb component
    else if (enable_6)
        old_orc_mux <= 3'b100; // Cr component
    else if (enable_22)
        old_orc_mux <= 3'b001; // Y component (wrap around)
end

// Sequential logic for read_mux: controls which FIFO (Y, Cb, Cr) is being read from
always @(posedge clk) begin
    if (rst)
        read_mux <= 3'b001; // Y component (initial state)
    else if (enable_1)
        read_mux <= 3'b010; // Cb component
    else if (enable_17)
        read_mux <= 3'b100; // Cr component
    else if (enable_33)
        read_mux <= 3'b001; // Y component (wrap around)
end

// Sequential logic for delayed cr_orc_1, cb_orc_1, y_orc_1: capture orc values at EOB
always @(posedge clk) begin
    if (rst) begin
        cr_orc_1 <= 5'b0; cb_orc_1 <= 5'b0; y_orc_1 <= 5'b0;
    end else if (end_of_block_output) begin
        cr_orc_1 <= cr_orc;
        cb_orc_1 <= cb_orc;
        y_orc_1 <= y_orc;
    end
end

// Sequential logic for jpeg_ro_5 and edge_ro_5: shifting and decrementing for rollover
always @(posedge clk) begin
    if (rst) begin
        jpeg_ro_5 <= 32'b0; edge_ro_5 <= 5'b0;
    end else if (br_5) begin
        jpeg_ro_5 <= (edge_ro_4 <= 5'd1) ? jpeg_ro_4 << 1 : jpeg_ro_4; // Shift left by 1 if edge_ro_4 <= 1
        edge_ro_5 <= (edge_ro_4 <= 5'd1) ? edge_ro_4 : edge_ro_4 - 5'd1; // Decrement edge_ro_4
    end
end

// Sequential logic for jpeg_5, orc_5, jpeg_ro_4, edge_ro_4: shifting and decrementing
always @(posedge clk) begin
    if (rst) begin
        jpeg_5 <= 32'b0; orc_5 <= 5'b0; jpeg_ro_4 <= 32'b0; edge_ro_4 <= 5'b0;
    end else if (br_4) begin
        jpeg_5 <= (orc_4 >= 5'd1) ? jpeg_4 >> 1 : jpeg_4; // Shift right by 1 if orc_4 >= 1
        orc_5 <= (orc_4 >= 5'd1) ? orc_4 - 5'd1 : orc_4; // Decrement orc_4
        jpeg_ro_4 <= (edge_ro_3 <= 5'd2) ? jpeg_ro_3 << 2 : jpeg_ro_3; // Shift left by 2 if edge_ro_3 <= 2
        edge_ro_4 <= (edge_ro_3 <= 5'd2) ? edge_ro_3 : edge_ro_3 - 5'd2; // Decrement edge_ro_3 by 2
    end
end

// Sequential logic for jpeg_4, orc_4, jpeg_ro_3, edge_ro_3: shifting and decrementing
always @(posedge clk) begin
    if (rst) begin
        jpeg_4 <= 32'b0; orc_4 <= 5'b0; jpeg_ro_3 <= 32'b0; edge_ro_3 <= 5'b0;
    end else if (br_3) begin
        jpeg_4 <= (orc_3 >= 5'd2) ? jpeg_3 >> 2 : jpeg_3; // Shift right by 2 if orc_3 >= 2
        orc_4 <= (orc_3 >= 5'd2) ? orc_3 - 5'd2 : orc_3; // Decrement orc_3 by 2
        jpeg_ro_3 <= (edge_ro_2 <= 5'd4) ? jpeg_ro_2 << 4 : jpeg_ro_2; // Shift left by 4 if edge_ro_2 <= 4
        edge_ro_3 <= (edge_ro_2 <= 5'd4) ? edge_ro_2 : edge_ro_2 - 5'd4; // Decrement edge_ro_2 by 4
    end
end

// Sequential logic for jpeg_3, orc_3, jpeg_ro_2, edge_ro_2: shifting and decrementing
always @(posedge clk) begin
    if (rst) begin
        jpeg_3 <= 32'b0; orc_3 <= 5'b0; jpeg_ro_2 <= 32'b0; edge_ro_2 <= 5'b0;
    end else if (br_2) begin
        jpeg_3 <= (orc_2 >= 5'd4) ? jpeg_2 >> 4 : jpeg_2; // Shift right by 4 if orc_2 >= 4
        orc_3 <= (orc_2 >= 5'd4) ? orc_2 - 5'd4 : orc_2; // Decrement orc_2 by 4
        jpeg_ro_2 <= (edge_ro_1 <= 5'd8) ? jpeg_ro_1 << 8 : jpeg_ro_1; // Shift left by 8 if edge_ro_1 <= 8
        edge_ro_2 <= (edge_ro_1 <= 5'd8) ? edge_ro_1 : edge_ro_1 - 5'd8; // Decrement edge_ro_1 by 8
    end
end

// Sequential logic for jpeg_2, orc_2, jpeg_ro_1, edge_ro_1: shifting and decrementing
always @(posedge clk) begin
    if (rst) begin
        jpeg_2 <= 32'b0; orc_2 <= 5'b0; jpeg_ro_1 <= 32'b0; edge_ro_1 <= 5'b0;
    end else if (br_1) begin
        jpeg_2 <= (orc_1 >= 5'd8) ? jpeg_1 >> 8 : jpeg_1; // Shift right by 8 if orc_1 >= 8
        orc_2 <= (orc_1 >= 5'd8) ? orc_1 - 5'd8 : orc_1; // Decrement orc_1 by 8
        jpeg_ro_1 <= (orc_reg_delay <= 5'd16) ? jpeg_delay << 16 : jpeg_delay; // Shift left by 16 if orc_reg_delay <= 16
        edge_ro_1 <= (orc_reg_delay <= 5'd16) ? orc_reg_delay : orc_reg_delay - 5'd16; // Decrement orc_reg_delay by 16
    end
end

// Sequential logic for jpeg_1, orc_1, jpeg_delay, orc_reg_delay: shifting and delaying
always @(posedge clk) begin
    if (rst) begin
        jpeg_1 <= 32'b0; orc_1 <= 5'b0; jpeg_delay <= 32'b0; orc_reg_delay <= 5'b0;
    end else if (bits_ready) begin
        jpeg_1 <= (orc_reg >= 5'd16) ? jpeg >> 16 : jpeg; // Shift right by 16 if orc_reg >= 16
        orc_1 <= (orc_reg >= 5'd16) ? orc_reg - 5'd16 : orc_reg; // Decrement orc_reg by 16
        jpeg_delay <= jpeg; // Delay jpeg
        orc_reg_delay <= orc_reg; // Delay orc_reg
    end
end

// Sequential logic for various enable signals (delayed versions of end_of_block_output)
always @(posedge clk) begin
    if (rst) begin
        enable_1 <= 1'b0; enable_2 <= 1'b0; enable_3 <= 1'b0; enable_4 <= 1'b0; enable_5 <= 1'b0;
        enable_6 <= 1'b0; enable_7 <= 1'b0; enable_8 <= 1'b0; enable_9 <= 1'b0; enable_10 <= 1'b0;
        enable_11 <= 1'b0; enable_12 <= 1'b0; enable_13 <= 1'b0; enable_14 <= 1'b0; enable_15 <= 1'b0;
        enable_16 <= 1'b0; enable_17 <= 1'b0; enable_18 <= 1'b0; enable_19 <= 1'b0; enable_20 <= 1'b0;
        enable_21 <= 1'b0; enable_22 <= 1'b0; enable_23 <= 1'b0; enable_24 <= 1'b0; enable_25 <= 1'b0;
        enable_26 <= 1'b0; enable_27 <= 1'b0; enable_28 <= 1'b0; enable_29 <= 1'b0; enable_30 <= 1'b0;
        enable_31 <= 1'b0; enable_32 <= 1'b0; enable_33 <= 1'b0; enable_34 <= 1'b0; enable_35 <= 1'b0;
    end else begin
        enable_1 <= end_of_block_output; enable_2 <= enable_1; // Delayed versions of end_of_block_output
        enable_3 <= enable_2; enable_4 <= enable_3; enable_5 <= enable_4;
        enable_6 <= enable_5; enable_7 <= enable_6; enable_8 <= enable_7;
        enable_9 <= enable_8; enable_10 <= enable_9; enable_11 <= enable_10;
        enable_12 <= enable_11; enable_13 <= enable_12; enable_14 <= enable_13;
        enable_15 <= enable_14; enable_16 <= enable_15; enable_17 <= enable_16;
        enable_18 <= enable_17; enable_19 <= enable_18; enable_20 <= enable_19;
        enable_21 <= enable_20;
        enable_22 <= enable_21; enable_23 <= enable_22; enable_24 <= enable_23;
        enable_25 <= enable_24; enable_26 <= enable_25; enable_27 <= enable_26;
        enable_28 <= enable_27; enable_29 <= enable_28; enable_30 <= enable_29;
        enable_31 <= enable_30;
        enable_32 <= enable_31; enable_33 <= enable_32; enable_34 <= enable_33;
        enable_35 <= enable_34;
    end
end

// Sequential logic for JPEG_bitstream bits [31:0]
// Each bit is set based on br_7 & rollover_6 or br_6 & static_orc_6 condition
always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[31] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[31] <= jpeg_6[31];
    else if (br_6 && static_orc_6 == 5'd0)
        JPEG_bitstream[31] <= jpeg_6[31];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[30] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[30] <= jpeg_6[30];
    else if (br_6 && static_orc_6 <= 5'd1)
        JPEG_bitstream[30] <= jpeg_6[30];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[29] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[29] <= jpeg_6[29];
    else if (br_6 && static_orc_6 <= 5'd2)
        JPEG_bitstream[29] <= jpeg_6[29];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[28] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[28] <= jpeg_6[28];
    else if (br_6 && static_orc_6 <= 5'd3)
        JPEG_bitstream[28] <= jpeg_6[28];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[27] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[27] <= jpeg_6[27];
    else if (br_6 && static_orc_6 <= 5'd4)
        JPEG_bitstream[27] <= jpeg_6[27];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[26] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[26] <= jpeg_6[26];
    else if (br_6 && static_orc_6 <= 5'd5)
        JPEG_bitstream[26] <= jpeg_6[26];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[25] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[25] <= jpeg_6[25];
    else if (br_6 && static_orc_6 <= 5'd6)
        JPEG_bitstream[25] <= jpeg_6[25];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[24] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[24] <= jpeg_6[24];
    else if (br_6 && static_orc_6 <= 5'd7)
        JPEG_bitstream[24] <= jpeg_6[24];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[23] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[23] <= jpeg_6[23];
    else if (br_6 && static_orc_6 <= 5'd8)
        JPEG_bitstream[23] <= jpeg_6[23];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[22] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[22] <= jpeg_6[22];
    else if (br_6 && static_orc_6 <= 5'd9)
        JPEG_bitstream[22] <= jpeg_6[22];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[21] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[21] <= jpeg_6[21];
    else if (br_6 && static_orc_6 <= 5'd10)
        JPEG_bitstream[21] <= jpeg_6[21];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[20] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[20] <= jpeg_6[20];
    else if (br_6 && static_orc_6 <= 5'd11)
        JPEG_bitstream[20] <= jpeg_6[20];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[19] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[19] <= jpeg_6[19];
    else if (br_6 && static_orc_6 <= 5'd12)
        JPEG_bitstream[19] <= jpeg_6[19];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[18] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[18] <= jpeg_6[18];
    else if (br_6 && static_orc_6 <= 5'd13)
        JPEG_bitstream[18] <= jpeg_6[18];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[17] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[17] <= jpeg_6[17];
    else if (br_6 && static_orc_6 <= 5'd14)
        JPEG_bitstream[17] <= jpeg_6[17];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[16] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[16] <= jpeg_6[16];
    else if (br_6 && static_orc_6 <= 5'd15)
        JPEG_bitstream[16] <= jpeg_6[16];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[15] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[15] <= jpeg_6[15];
    else if (br_6 && static_orc_6 <= 5'd16)
        JPEG_bitstream[15] <= jpeg_6[15];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[14] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[14] <= jpeg_6[14];
    else if (br_6 && static_orc_6 <= 5'd17)
        JPEG_bitstream[14] <= jpeg_6[14];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[13] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[13] <= jpeg_6[13];
    else if (br_6 && static_orc_6 <= 5'd18)
        JPEG_bitstream[13] <= jpeg_6[13];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[12] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[12] <= jpeg_6[12];
    else if (br_6 && static_orc_6 <= 5'd19)
        JPEG_bitstream[12] <= jpeg_6[12];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[11] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[11] <= jpeg_6[11];
    else if (br_6 && static_orc_6 <= 5'd20)
        JPEG_bitstream[11] <= jpeg_6[11];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[10] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[10] <= jpeg_6[10];
    else if (br_6 && static_orc_6 <= 5'd21)
        JPEG_bitstream[10] <= jpeg_6[10];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[9] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[9] <= jpeg_6[9];
    else if (br_6 && static_orc_6 <= 5'd22)
        JPEG_bitstream[9] <= jpeg_6[9];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[8] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[8] <= jpeg_6[8];
    else if (br_6 && static_orc_6 <= 5'd23)
        JPEG_bitstream[8] <= jpeg_6[8];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[7] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[7] <= jpeg_6[7];
    else if (br_6 && static_orc_6 <= 5'd24)
        JPEG_bitstream[7] <= jpeg_6[7];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[6] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[6] <= jpeg_6[6];
    else if (br_6 && static_orc_6 <= 5'd25)
        JPEG_bitstream[6] <= jpeg_6[6];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[5] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[5] <= jpeg_6[5];
    else if (br_6 && static_orc_6 <= 5'd26)
        JPEG_bitstream[5] <= jpeg_6[5];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[4] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[4] <= jpeg_6[4];
    else if (br_6 && static_orc_6 <= 5'd27)
        JPEG_bitstream[4] <= jpeg_6[4];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[3] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[3] <= jpeg_6[3];
    else if (br_6 && static_orc_6 <= 5'd28)
        JPEG_bitstream[3] <= jpeg_6[3];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[2] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[2] <= jpeg_6[2];
    else if (br_6 && static_orc_6 <= 5'd29)
        JPEG_bitstream[2] <= jpeg_6[2];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[1] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[1] <= jpeg_6[1];
    else if (br_6 && static_orc_6 <= 5'd30)
        JPEG_bitstream[1] <= jpeg_6[1];
end

always @(posedge clk) begin
    if (rst)
        JPEG_bitstream[0] <= 1'b0;
    else if (br_7 & rollover_6)
        JPEG_bitstream[0] <= jpeg_6[0];
    else if (br_6 && static_orc_6 <= 5'd31)
        JPEG_bitstream[0] <= jpeg_6[0];
end

// Sequential logic for jpeg_6: combines jpeg_ro_5 and jpeg_5 based on rollover_5 and static_orc_5
always @(posedge clk) begin
    if (rst) begin
        jpeg_6 <= 32'b0;
    end else if (br_5 | br_6) begin
        // Each bit of jpeg_6 is set based on a conditional logic involving rollover_5, static_orc_5, jpeg_ro_5, and jpeg_5
        jpeg_6[31] <= (rollover_5 & static_orc_5 > 5'd0) ? jpeg_ro_5[31] : jpeg_5[31];
        jpeg_6[30] <= (rollover_5 & static_orc_5 > 5'd1) ? jpeg_ro_5[30] : jpeg_5[30];
        jpeg_6[29] <= (rollover_5 & static_orc_5 > 5'd2) ? jpeg_ro_5[29] : jpeg_5[29];
        jpeg_6[28] <= (rollover_5 & static_orc_5 > 5'd3) ? jpeg_ro_5[28] : jpeg_5[28];
        jpeg_6[27] <= (rollover_5 & static_orc_5 > 5'd4) ? jpeg_ro_5[27] : jpeg_5[27];
        jpeg_6[26] <= (rollover_5 & static_orc_5 > 5'd5) ? jpeg_ro_5[26] : jpeg_5[26];
        jpeg_6[25] <= (rollover_5 & static_orc_5 > 5'd6) ? jpeg_ro_5[25] : jpeg_5[25];
        jpeg_6[24] <= (rollover_5 & static_orc_5 > 5'd7) ? jpeg_ro_5[24] : jpeg_5[24];
        jpeg_6[23] <= (rollover_5 & static_orc_5 > 5'd8) ? jpeg_ro_5[23] : jpeg_5[23];
        jpeg_6[22] <= (rollover_5 & static_orc_5 > 5'd9) ? jpeg_ro_5[22] : jpeg_5[22];
        jpeg_6[21] <= (rollover_5 & static_orc_5 > 5'd10) ? jpeg_ro_5[21] : jpeg_5[21];
        jpeg_6[20] <= (rollover_5 & static_orc_5 > 5'd11) ? jpeg_ro_5[20] : jpeg_5[20];
        jpeg_6[19] <= (rollover_5 & static_orc_5 > 5'd12) ? jpeg_ro_5[19] : jpeg_5[19];
        jpeg_6[18] <= (rollover_5 & static_orc_5 > 5'd13) ? jpeg_ro_5[18] : jpeg_5[18];
        jpeg_6[17] <= (rollover_5 & static_orc_5 > 5'd14) ? jpeg_ro_5[17] : jpeg_5[17];
        jpeg_6[16] <= (rollover_5 & static_orc_5 > 5'd15) ? jpeg_ro_5[16] : jpeg_5[16];
        jpeg_6[15] <= (rollover_5 & static_orc_5 > 5'd16) ? jpeg_ro_5[15] : jpeg_5[15];
        jpeg_6[14] <= (rollover_5 & static_orc_5 > 5'd17) ? jpeg_ro_5[14] : jpeg_5[14];
        jpeg_6[13] <= (rollover_5 & static_orc_5 > 5'd18) ? jpeg_ro_5[13] : jpeg_5[13];
        jpeg_6[12] <= (rollover_5 & static_orc_5 > 5'd19) ? jpeg_ro_5[12] : jpeg_5[12];
        jpeg_6[11] <= (rollover_5 & static_orc_5 > 5'd20) ? jpeg_ro_5[11] : jpeg_5[11];
        jpeg_6[10] <= (rollover_5 & static_orc_5 > 5'd21) ? jpeg_ro_5[10] : jpeg_5[10];
        jpeg_6[9] <= (rollover_5 & static_orc_5 > 5'd22) ? jpeg_ro_5[9] : jpeg_5[9];
        jpeg_6[8] <= (rollover_5 & static_orc_5 > 5'd23) ? jpeg_ro_5[8] : jpeg_5[8];
        jpeg_6[7] <= (rollover_5 & static_orc_5 > 5'd24) ? jpeg_ro_5[7] : jpeg_5[7];
        jpeg_6[6] <= (rollover_5 & static_orc_5 > 5'd25) ? jpeg_ro_5[6] : jpeg_5[6];
        jpeg_6[5] <= (rollover_5 & static_orc_5 > 5'd26) ? jpeg_ro_5[5] : jpeg_5[5];
        jpeg_6[4] <= (rollover_5 & static_orc_5 > 5'd27) ? jpeg_ro_5[4] : jpeg_5[4];
        jpeg_6[3] <= (rollover_5 & static_orc_5 > 5'd28) ? jpeg_ro_5[3] : jpeg_5[3];
        jpeg_6[2] <= (rollover_5 & static_orc_5 > 5'd29) ? jpeg_ro_5[2] : jpeg_5[2];
        jpeg_6[1] <= (rollover_5 & static_orc_5 > 5'd30) ? jpeg_ro_5[1] : jpeg_5[1];
        jpeg_6[0] <= jpeg_5[0]; // Bit 0 is always from jpeg_5
    end
end

endmodule
