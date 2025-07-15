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
    input  logic [23:0] data_in,     // 24-bit input data
    output logic [31:0] JPEG_bitstream, // 32-bit JPEG bitstream output
    output logic        data_ready,  // Data ready signal
    output logic [4:0]  orc_reg      // Output register for orc value
);
    
//==============================================================declaration===============================================================================================
// logic for JPEG bitstream and orc values from pre_fifo
logic  [31:0]  cb_JPEG_bitstream, cr_JPEG_bitstream, y_JPEG_bitstream;
logic  [4:0]   cr_orc, cb_orc, y_orc;

// logic for Y component output from FIFO and enable signal
logic  [31:0]  y_bits_out;
logic          y_out_enable;

// logic for data ready signals from pre_fifo and FIFO empty status
logic          cb_data_ready, cr_data_ready, y_data_ready;
logic          end_of_block_output, y_eob_empty;
logic          cb_eob_empty, cr_eob_empty;
logic          y_fifo_empty;

// FIFO read request signals
logic y_read_req, cb_read_req, cr_read_req;

// logic for orc values and their delayed versions
logic  [4:0]    orc, orc_cb, orc_cr, old_orc_reg, sorc_reg, roll_orc_reg;
logic  [4:0]    orc_pipe[0:4]; // Pipelined ORC: orc_1 to orc_5
logic  [4:0]    static_orc_pipe[0:5]; // Pipelined static_orc: static_orc_1 to static_orc_6

// logic for edge rollover and JPEG bitstream values and their delayed versions
logic  [4:0]    edge_ro_pipe[0:4]; // Pipelined edge_ro: edge_ro_1 to edge_ro_5
logic  [31:0]   jpeg_ro_pipe[0:4], jpeg_delay; // Pipelined jpeg_ro: jpeg_ro_1 to jpeg_ro_5

// logic for JPEG bitstream and its delayed versions
logic  [31:0]   jpeg_pipe[0:5]; // Pipelined jpeg: jpeg_1 to jpeg_6
logic  [31:0]   jpeg_current; // Original 'jpeg' before pipelining

// logic for delayed orc values and enable signals
logic  [4:0]    cr_orc_delay, cb_orc_delay, y_orc_delay;
logic           cr_out_enable_delay, cb_out_enable_delay, y_out_enable_delay, eob_pipe[0:3]; // eob_1 to eob_4

// logic for various enable signals (using an array for conciseness)
logic           enable_pipe[0:34]; // enable_1 to enable_35

// logic for mux control and bit ready signals
logic  [2:0]    bits_mux, old_orc_mux, read_mux;
logic           bits_ready_current, br_pipe[0:7]; // br_1 to br_8

// logic for rollover conditions and EOB signals
logic           rollover_current, rollover_pipe[0:6]; // rollover_1 to rollover_7
logic           eobe_delay; // Original eobe_1
logic           rollover_eob_current;
logic           eob_early_out_enable_reg, fifo_mux;
logic [4:0]     orc_reg_delay; // Added for pipeline stage logic

// logic for CR and CB FIFO outputs and empty/enable signals (for dual FIFOs)
logic [31:0]   cr_bits_out1, cr_bits_out2, cb_bits_out1, cb_bits_out2;
logic          cr_fifo_empty1, cr_fifo_empty2, cb_fifo_empty1, cb_fifo_empty2;
logic          cr_out_enable1, cr_out_enable2, cb_out_enable1, cb_out_enable2;

// Write enable signals for FIFOs, based on data ready and EOB empty
logic cb_write_enable = cb_data_ready && !cb_eob_empty;
logic cr_write_enable = cr_data_ready && !cr_eob_empty;
logic y_write_enable = y_data_ready && !y_eob_empty;

// FIFO muxing logic for CR component (distributing read/write to two FIFOs)
logic cr_read_req1 = fifo_mux ? 1'b0 : cr_read_req;
logic cr_read_req2 = fifo_mux ? cr_read_req : 1'b0;
logic [31:0] cr_JPEG_bitstream1 = fifo_mux ? cr_JPEG_bitstream : 32'b0;
logic [31:0] cr_JPEG_bitstream2 = fifo_mux ? 32'b0 : cr_JPEG_bitstream;
logic cr_write_enable1 = fifo_mux && cr_write_enable;
logic cr_write_enable2 = !fifo_mux && cr_write_enable;
logic [31:0] cr_bits_out = fifo_mux ? cr_bits_out2 : cr_bits_out1;
logic cr_fifo_empty = fifo_mux ? cr_fifo_empty2 : cr_fifo_empty1;
logic cr_out_enable = fifo_mux ? cr_out_enable2 : cr_out_enable1;

// FIFO muxing logic for CB component (distributing read/write to two FIFOs)
logic cb_read_req1 = fifo_mux ? 1'b0 : cb_read_req;
logic cb_read_req2 = fifo_mux ? cb_read_req : 1'b0;
logic [31:0] cb_JPEG_bitstream1 = fifo_mux ? cb_JPEG_bitstream : 32'b0;
logic [31:0] cb_JPEG_bitstream2 = fifo_mux ? 32'b0 : cb_JPEG_bitstream;
logic cb_write_enable1 = fifo_mux && cb_write_enable;
logic cb_write_enable2 = !fifo_mux && cb_write_enable;
logic [31:0] cb_bits_out = fifo_mux ? cb_bits_out2 : cb_bits_out1;
logic cb_fifo_empty = fifo_mux ? cb_fifo_empty2 : cb_fifo_empty1;
logic cb_out_enable = fifo_mux ? cb_out_enable2 : cb_out_enable1;
//=============================================================================================================================================================


//=====================================================================instance calll===========================================================================
    
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
//=============================================================================================================================================================

// Consolidated sequential logic for FIFO read requests, mux controls, and FIFO mux
always_ff @(posedge clk) begin
    if (rst) begin
        fifo_mux <= 1'b0;
        y_read_req <= 1'b0;
        cb_read_req <= 1'b0;
        cr_read_req <= 1'b0;
        bits_mux <= 3'b001; // Y component (initial state)
        old_orc_mux <= 3'b001; // Y component (initial state)
        read_mux <= 3'b001; // Y component (initial state)
    end else begin
        // fifo_mux toggle
        if (end_of_block_output)
            fifo_mux <= ~fifo_mux;

        // y_read_req logic
        if (y_fifo_empty || read_mux != 3'b001)
            y_read_req <= 1'b0;
        else if (!y_fifo_empty && read_mux == 3'b001)
            y_read_req <= 1'b1;

        // cb_read_req logic
        if (cb_fifo_empty || read_mux != 3'b010)
            cb_read_req <= 1'b0;
        else if (!cb_fifo_empty && read_mux == 3'b010)
            cb_read_req <= 1'b1;

        // cr_read_req logic
        if (cr_fifo_empty || read_mux != 3'b100)
            cr_read_req <= 1'b0;
        else if (!cr_fifo_empty && read_mux == 3'b100)
            cr_read_req <= 1'b1;

        // bits_mux logic
        if (enable_pipe[2]) // enable_3
            bits_mux <= 3'b010; // Cb component
        else if (enable_pipe[18]) // enable_19
            bits_mux <= 3'b100; // Cr component
        else if (enable_pipe[34]) // enable_35
            bits_mux <= 3'b001; // Y component (wrap around)

        // old_orc_mux logic
        if (enable_pipe[0]) // enable_1
            old_orc_mux <= 3'b010; // Cb component
        else if (enable_pipe[5]) // enable_6
            old_orc_mux <= 3'b100; // Cr component
        else if (enable_pipe[21]) // enable_22
            old_orc_mux <= 3'b001; // Y component (wrap around)

        // read_mux logic
        if (enable_pipe[0]) // enable_1
            read_mux <= 3'b010; // Cb component
        else if (enable_pipe[16]) // enable_17
            read_mux <= 3'b100; // Cr component
        else if (enable_pipe[32]) // enable_33
            read_mux <= 3'b001; // Y component (wrap around)
    end
end

// Consolidated sequential logic for ORC values and out_enable delays
always_ff @(posedge clk) begin
    if (rst) begin
        orc <= 5'b0;
        orc_cb <= 5'b0;
        orc_cr <= 5'b0;
        cr_out_enable_delay <= 1'b0; cb_out_enable_delay <= 1'b0; y_out_enable_delay <= 1'b0;
        cr_orc_delay <= 5'b0; cb_orc_delay <= 5'b0; y_orc_delay <= 5'b0;
    end else begin
        // orc (Y component's orc)
        if (enable_pipe[19]) // enable_20
            orc <= orc_cr + cr_orc_delay;

        // orc_cb (Cb component's orc)
        if (eob_pipe[0]) // eob_1
            orc_cb <= orc + y_orc_delay;

        // orc_cr (Cr component's orc)
        if (enable_pipe[4]) // enable_5
            orc_cr <= orc_cb + cb_orc_delay;

        // Delayed out_enable signals
        cr_out_enable_delay <= cr_out_enable;
        cb_out_enable_delay <= cb_out_enable;
        y_out_enable_delay <= y_out_enable;

        // Capture orc values at EOB
        if (end_of_block_output) begin
            cr_orc_delay <= cr_orc;
            cb_orc_delay <= cb_orc;
            y_orc_delay <= y_orc;
        end
    end
end

// Consolidated sequential logic for current JPEG/bits_ready, sorc, roll_orc, old_orc
always_ff @(posedge clk) begin
    case (bits_mux)
        3'b001:  begin jpeg_current <= y_bits_out; bits_ready_current <= y_out_enable; sorc_reg <= orc; end // Y
        3'b010:  begin jpeg_current <= cb_bits_out; bits_ready_current <= cb_out_enable; sorc_reg <= orc_cb; end // Cb
        3'b100:  begin jpeg_current <= cr_bits_out; bits_ready_current <= cr_out_enable; sorc_reg <= orc_cr; end // Cr
        default: begin jpeg_current <= y_bits_out; bits_ready_current <= y_out_enable; sorc_reg <= orc; end // Default Y
    endcase

    case (old_orc_mux)
        3'b001:  begin roll_orc_reg <= orc; old_orc_reg <= orc_cr; end // Y for roll_orc, Cr for old_orc
        3'b010:  begin roll_orc_reg <= orc_cb; old_orc_reg <= orc; end // Cb for roll_orc, Y for old_orc
        3'b100:  begin roll_orc_reg <= orc_cr; old_orc_reg <= orc_cb; end // Cr for roll_orc, Cb for old_orc
        default: begin roll_orc_reg <= orc; old_orc_reg <= orc_cr; end // Default
    endcase
end

// Consolidated sequential logic for br_pipe, static_orc_pipe, rollover_eob, rollover_pipe, eob_pipe, enable_pipe
always_ff @(posedge clk) begin
    if (rst) begin
        for (integer i = 0; i < 8; i = i + 1) br_pipe[i] <= 1'b0;
        for (integer i = 0; i < 6; i = i + 1) static_orc_pipe[i] <= 5'b0;
        eobe_delay <= 1'b0;
        rollover_eob_current <= 1'b0;
        for (integer i = 0; i < 7; i = i + 1) rollover_pipe[i] <= 1'b0;
        for (integer i = 0; i < 4; i = i + 1) eob_pipe[i] <= 1'b0;
        eob_early_out_enable_reg <= 1'b0;
        for (integer i = 0; i < 35; i = i + 1) enable_pipe[i] <= 1'b0;
    end else begin
        // br_pipe logic
        br_pipe[0] <= bits_ready_current & !eobe_delay; // br_1 is bits_ready unless eob_early_out is enabled
        for (integer i = 1; i < 8; i = i + 1) br_pipe[i] <= br_pipe[i-1];

        // static_orc_pipe logic
        static_orc_pipe[0] <= sorc_reg;
        for (integer i = 1; i < 6; i = i + 1) static_orc_pipe[i] <= static_orc_pipe[i-1];

        // eobe_delay
        eobe_delay <= y_eob_empty;

        // rollover_eob_current
        if (br_pipe[2])
            rollover_eob_current <= old_orc_reg >= roll_orc_reg;

        // rollover_pipe logic
        rollover_pipe[0] <= rollover_current; // rollover_1
        for (integer i = 1; i < 4; i = i + 1) rollover_pipe[i] <= rollover_pipe[i-1];
        rollover_pipe[3] <= rollover_pipe[2] | rollover_eob_current; // rollover_4 is high if rollover_3 or rollover_eob is high
        for (integer i = 4; i < 7; i = i + 1) rollover_pipe[i] <= rollover_pipe[i-1];

        // eob_pipe logic
        eob_pipe[0] <= end_of_block_output; // eob_1 from end_of_block_output
        for (integer i = 1; i < 4; i = i + 1) eob_pipe[i] <= eob_pipe[i-1];

        // eob_early_out_enable_reg
        eob_early_out_enable_reg <= y_out_enable & y_out_enable_delay & eob_pipe[1];

        // enable_pipe logic
        enable_pipe[0] <= end_of_block_output; // enable_1
        for (integer i = 1; i < 35; i = i + 1) enable_pipe[i] <= enable_pipe[i-1];
    end
end

// Sequential logic for rollover based on bits_mux selection and enable signals
always_ff @(posedge clk) begin
    case (bits_mux)
        3'b001:  rollover_current <= y_out_enable_delay & !eob_pipe[3] & !eob_early_out_enable_reg; // Y component
        3'b010:  rollover_current <= cb_out_enable_delay & cb_out_enable; // Cb component
        3'b100:  rollover_current <= cr_out_enable_delay & cr_out_enable; // Cr component
        default: rollover_current <= y_out_enable_delay & !eob_pipe[3]; // Default to Y component
    endcase
end

// Consolidated sequential logic for pipeline stages (jpeg_X, orc_X, jpeg_ro_X, edge_ro_X)
always_ff @(posedge cllk) begin
    if (rst) begin
        jpeg_pipe[4] <= 32'b0; orc_pipe[4] <= 5'b0; jpeg_ro_pipe[3] <= 32'b0; edge_ro_pipe[3] <= 5'b0; // Stage 5
        jpeg_pipe[3] <= 32'b0; orc_pipe[3] <= 5'b0; jpeg_ro_pipe[2] <= 32'b0; edge_ro_pipe[2] <= 5'b0; // Stage 4
        jpeg_pipe[2] <= 32'b0; orc_pipe[2] <= 5'b0; jpeg_ro_pipe[1] <= 32'b0; edge_ro_pipe[1] <= 5'b0; // Stage 3
        jpeg_pipe[1] <= 32'b0; orc_pipe[1] <= 5'b0; jpeg_ro_pipe[0] <= 32'b0; edge_ro_pipe[0] <= 5'b0; // Stage 2
        jpeg_pipe[0] <= 32'b0; orc_pipe[0] <= 5'b0; jpeg_delay <= 32'b0; orc_reg_delay <= 5'b0; // Stage 1 (input to pipeline)
    end else begin
        // Stage 5 logic (from br_5)
        if (br_pipe[4]) begin
            jpeg_ro_pipe[4] <= (edge_ro_pipe[3] <= 5'd1) ? jpeg_ro_pipe[3] << 1 : jpeg_ro_pipe[3];
            edge_ro_pipe[4] <= (edge_ro_pipe[3] <= 5'd1) ? edge_ro_pipe[3] : edge_ro_pipe[3] - 5'd1;
        end

        // Stage 4 logic (from br_4)
        if (br_pipe[3]) begin
            jpeg_pipe[4] <= (orc_pipe[3] >= 5'd1) ? jpeg_pipe[3] >> 1 : jpeg_pipe[3];
            orc_pipe[4] <= (orc_pipe[3] >= 5'd1) ? orc_pipe[3] - 5'd1 : orc_pipe[3];
            jpeg_ro_pipe[3] <= (edge_ro_pipe[2] <= 5'd2) ? jpeg_ro_pipe[2] << 2 : jpeg_ro_pipe[2];
            edge_ro_pipe[3] <= (edge_ro_pipe[2] <= 5'd2) ? edge_ro_pipe[2] : edge_ro_pipe[2] - 5'd2;
        end

        // Stage 3 logic (from br_3)
        if (br_pipe[2]) begin
            jpeg_pipe[3] <= (orc_pipe[2] >= 5'd2) ? jpeg_pipe[2] >> 2 : jpeg_pipe[2];
            orc_pipe[3] <= (orc_pipe[2] >= 5'd2) ? orc_pipe[2] - 5'd2 : orc_pipe[2];
            jpeg_ro_pipe[2] <= (edge_ro_pipe[1] <= 5'd4) ? jpeg_ro_pipe[1] << 4 : jpeg_ro_pipe[1];
            edge_ro_pipe[2] <= (edge_ro_pipe[1] <= 5'd4) ? edge_ro_pipe[1] : edge_ro_pipe[1] - 5'd4;
        end

        // Stage 2 logic (from br_2)
        if (br_pipe[1]) begin
            jpeg_pipe[2] <= (orc_pipe[1] >= 5'd4) ? jpeg_pipe[1] >> 4 : jpeg_pipe[1];
            orc_pipe[2] <= (orc_pipe[1] >= 5'd4) ? orc_pipe[1] - 5'd4 : orc_pipe[1];
            jpeg_ro_pipe[1] <= (edge_ro_pipe[0] <= 5'd8) ? jpeg_ro_pipe[0] << 8 : jpeg_ro_pipe[0];
            edge_ro_pipe[1] <= (edge_ro_pipe[0] <= 5'd8) ? edge_ro_pipe[0] : edge_ro_pipe[0] - 5'd8;
        end

        // Stage 1 logic (from br_1)
        if (br_pipe[0]) begin
            jpeg_pipe[1] <= (orc_pipe[0] >= 5'd8) ? jpeg_pipe[0] >> 8 : jpeg_pipe[0];
            orc_pipe[1] <= (orc_pipe[0] >= 5'd8) ? orc_pipe[0] - 5'd8 : orc_pipe[0];
            jpeg_ro_pipe[0] <= (orc_reg_delay <= 5'd16) ? jpeg_delay << 16 : jpeg_delay;
            edge_ro_pipe[0] <= (orc_reg_delay <= 5'd16) ? orc_reg_delay : orc_reg_delay - 5'd16;
        end

        // Input to pipeline logic (from bits_ready_current)
        if (bits_ready_current) begin
            jpeg_pipe[0] <= (orc_reg >= 5'd16) ? jpeg_current >> 16 : jpeg_current;
            orc_pipe[0] <= (orc_reg >= 5'd16) ? orc_reg - 5'd16 : orc_reg;
            jpeg_delay <= jpeg_current;
            orc_reg_delay <= orc_reg;
        end
    end
end

// Direct assignment for data_ready output
assign data_ready = br_pipe[5] & rollover_pipe[4]; // Data ready when br_6 and rollover_5 are high

// Direct assignment for orc_reg output
assign orc_reg = (bits_mux == 3'b001) ? orc :
                 (bits_mux == 3'b010) ? orc_cb :
                 (bits_mux == 3'b100) ? orc_cr : orc; // Default to Y component orc

// Sequential logic for jpeg_6: combines jpeg_ro_5 and jpeg_5 based on rollover_5 and static_orc_5
always_ff @(posedge clk) begin
    if (rst) begin
        jpeg_pipe[5] <= 32'b0;
    end else if (br_pipe[4] | br_pipe[5]) begin // br_5 | br_6
        // Each bit of jpeg_6 is set based on a conditional logic involving rollover_5, static_orc_5, jpeg_ro_5, and jpeg_5
        for (integer i = 0; i < 31; i = i + 1) begin
            jpeg_pipe[5][i] <= (rollover_pipe[4] & static_orc_pipe[4] > (31 - i)) ? jpeg_ro_pipe[4][i] : jpeg_pipe[4][i];
        end
        jpeg_pipe[5][0] <= jpeg_pipe[4][0]; // Bit 0 is always from jpeg_5
    end
end

// Sequential logic for JPEG_bitstream bits [31:0]
// Each bit is set based on br_7 & rollover_6 or br_6 & static_orc_6 condition
generate
    genvar i;
    for (i = 0; i < 32; i = i + 1) begin : gen_jpeg_bitstream
        always_ff @(posedge clk) begin
            if (rst)
                JPEG_bitstream[i] <= 1'b0;
            else if (br_pipe[6] & rollover_pipe[5]) // br_7 & rollover_6
                JPEG_bitstream[i] <= jpeg_pipe[5][i];
            else if (br_pipe[5] && static_orc_pipe[5] <= (31 - i)) // br_6 && static_orc_6 <= (31 - i)
                JPEG_bitstream[i] <= jpeg_pipe[5][i];
        end
    end
endgenerate

endmodule
