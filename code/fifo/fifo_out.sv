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

Inputs:
    clk           : Clock signal
    rst           : Asynchronous active-high reset
    enable        : Enable signal to initiate processing
    data_in       : Input data from upstream module (e.g., Huffman-encoded block)

Outputs:
    JPEG_bitstream : Final 32-bit aligned output stream
    data_ready      : High when JPEG_bitstream contains valid data
    orc_reg         : ORC value corresponding to JPEG_bitstream
--------------------------------------------------------------------------- */

`timescale 1ns / 100ps

module fifo_out(
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [23:0] data_in,
    output logic [31:0] JPEG_bitstream,
    output logic        data_ready,
    output logic [4:0]  orc_reg
);

    // ------------------------------------------------------------------------
    // Internal Wires and Registers
    // ------------------------------------------------------------------------
    // Wires for individual JPEG bitstreams and control signals for Y, Cb, Cr
    logic [31:0] cb_JPEG_bitstream, cr_JPEG_bitstream, y_JPEG_bitstream;
    logic [4:0]  cr_orc, cb_orc, y_orc;
    logic        cb_data_ready, cr_data_ready, y_data_ready;
    logic        cb_eob_empty, cr_eob_empty, y_eob_empty;
    logic        end_of_block_output;
    
    // FIFOs data out, enable, empty
    logic [31:0] y_bits_out;
    logic        y_out_enable;
    logic        y_fifo_empty;

    // ORC pipelining and muxing control signals
    logic [4:0] orc, orc_cb, orc_cr, old_orc_reg, sorc_reg, roll_orc_reg;
    logic [4:0] orc_reg_delay;
    logic [4:0] cr_orc_1, cb_orc_1, y_orc_1;
    logic       cr_out_enable_1, cb_out_enable_1, y_out_enable_1;

    // Mux selectors and internal control logic
    logic [2:0] bits_mux, old_orc_mux, read_mux;
    logic       bits_ready;
    logic       rollover, rollover_eob;
    logic       eobe_1;
    logic       cb_read_req, cr_read_req, y_read_req;
    logic       eob_early_out_enable, fifo_mux;

    // Pipelined control logic
    logic [7:1] br_pipe;               // Valid bits pipeline stages (br_1 to br_7)
    logic [5:1][4:0] static_orc_pipe;  // ORC static values across stages
    logic [35:1] enable_pipe;          // Enable pulse tracking
    logic [3:1] eob_pipe;              // End-of-block pipelining
    logic [6:1] rollover_pipe;         // Rollover signal pipeline

    // JPEG data pipelines
    logic [31:0] jpeg_pipe[0:6];       // JPEG data per stage
    logic [4:0]  orc_pipe[0:5];        // ORC tracking pipeline
    logic [31:0] jpeg_ro_pipe[0:5];    // JPEG rotated outputs
    logic [4:0]  edge_ro_pipe[0:5];    // Edge ORC values per stage
    logic [31:0] jpeg_delay;

    // FIFO muxed output selection
    logic [31:0] cr_bits_out1, cr_bits_out2, cb_bits_out1, cb_bits_out2;
    logic        cr_fifo_empty1, cr_fifo_empty2, cb_fifo_empty1, cb_fifo_empty2;
    logic        cr_out_enable1, cr_out_enable2, cb_out_enable1, cb_out_enable2;

    // FIFO write enables
    logic        cr_write_enable_comb = cr_data_ready && !cr_eob_empty;
    logic        cb_write_enable_comb = cb_data_ready && !cb_eob_empty;
    logic        y_write_enable_comb = y_data_ready && !y_eob_empty;

    // ------------------------------------------------------------------------
    // FIFO Control and Preprocessing Modules
    // ------------------------------------------------------------------------

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

    sync_fifo_32 u17(
        .clk(clk),
        .rst(rst),
        .read_req(y_read_req),
        .write_data(y_JPEG_bitstream),
        .write_enable(y_write_enable_comb),
        .read_data(y_bits_out),
        .fifo_empty(y_fifo_empty),
        .rdata_valid(y_out_enable)
    );

    // ---
    // Pipelined Registers
    // ---

    // Pipelined enables (enable_1 to enable_35)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_pipe <= '0;
        end else begin
            enable_pipe <= {enable_pipe[34:1], end_of_block_output};
        end
    end

    // Pipelined bits_ready (br_1 to br_8)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            br_pipe <= '0;
        end else begin
            br_pipe <= {br_pipe[6:1], bits_ready && !eobe_1}; // Adjusted for 8 stages total
        end
    end

    // Pipelined static_orc (static_orc_1 to static_orc_6)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            static_orc_pipe <= '0;
        end else begin
            for (int i = 0; i < 5; i++) static_orc_pipe[i+1] <= static_orc_pipe[i]; // Shift values
            static_orc_pipe[0] <= sorc_reg; // New value into the first stage
        end
    end
    // Map individual static_orc_x signals to the array for clarity where used
    logic [4:0] static_orc_6; // Replaces original static_orc_6 reg
    assign static_orc_6 = static_orc_pipe[5]; // The 6th stage in the original code is pipe[5] (0-indexed)

    // Pipelined rollover signals (rollover_1 to rollover_7)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            rollover_pipe <= '0;
            eob_pipe <= '0;
            eob_early_out_enable <= 1'b0;
        end else begin
            rollover_pipe <= {rollover_pipe[5:1], rollover};
            rollover_pipe[3] <= rollover_pipe[2] | rollover_eob; // rollover_4 is rollover_pipe[3]
            eob_pipe <= {eob_pipe[2:1], end_of_block_output};
            eob_early_out_enable <= y_out_enable && y_out_enable_1 && eob_pipe[1]; // eob_2 is eob_pipe[1]
        end
    end

 // ------------------------------------------------------------------------
 // JPEG Pipeline Shifting and Bit Manipulation (6 stages)
 // ------------------------------------------------------------------------

    // Stage 0: Input Latch for JPEG data
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            jpeg_pipe[0] <= 32'b0; // This is 'jpeg' in original code
            orc_pipe[0] <= 5'b0;   // This is 'orc_reg' in original code
            jpeg_delay <= 32'b0;
            orc_reg_delay <= 5'b0;
        end else if (bits_ready) begin
            jpeg_pipe[0] <= (orc_reg >= 16) ? jpeg_pipe[0] >> 16 : jpeg_pipe[0];
            orc_pipe[0] <= (orc_reg >= 16) ? orc_reg - 16 : orc_reg;
            jpeg_delay <= jpeg_pipe[0]; // This 'jpeg' is now the input to the stage
            orc_reg_delay <= orc_reg;
        end
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 1; i <= 6; i++) jpeg_pipe[i] <= 32'b0;
            for (int i = 1; i <= 5; i++) orc_pipe[i] <= 5'b0;
            for (int i = 1; i <= 5; i++) jpeg_ro_pipe[i] <= 32'b0;
            for (int i = 1; i <= 5; i++) edge_ro_pipe[i] <= 5'b0;
        end else begin
            // Stage 1 (jpeg_2, orc_2, jpeg_ro_1, edge_ro_1)
            if (br_pipe[0]) begin // br_1
                jpeg_pipe[1] <= (orc_pipe[0] >= 8) ? jpeg_pipe[0] >> 8 : jpeg_pipe[0];
                orc_pipe[1] <= (orc_pipe[0] >= 8) ? orc_pipe[0] - 8 : orc_pipe[0];
                jpeg_ro_pipe[1] <= (orc_reg_delay <= 16) ? jpeg_delay << 16 : jpeg_delay;
                edge_ro_pipe[1] <= (orc_reg_delay <= 16) ? orc_reg_delay : orc_reg_delay - 16;
            end

            // Stage 2 (jpeg_3, orc_3, jpeg_ro_2, edge_ro_2)
            if (br_pipe[1]) begin // br_2
                jpeg_pipe[2] <= (orc_pipe[1] >= 4) ? jpeg_pipe[1] >> 4 : jpeg_pipe[1];
                orc_pipe[2] <= (orc_pipe[1] >= 4) ? orc_pipe[1] - 4 : orc_pipe[1];
                jpeg_ro_pipe[2] <= (edge_ro_pipe[1] <= 8) ? jpeg_ro_pipe[1] << 8 : jpeg_ro_pipe[1];
                edge_ro_pipe[2] <= (edge_ro_pipe[1] <= 8) ? edge_ro_pipe[1] : edge_ro_pipe[1] - 8;
            end

            // Stage 3 (jpeg_4, orc_4, jpeg_ro_3, edge_ro_3)
            if (br_pipe[2]) begin // br_3
                jpeg_pipe[3] <= (orc_pipe[2] >= 2) ? jpeg_pipe[2] >> 2 : jpeg_pipe[2];
                orc_pipe[3] <= (orc_pipe[2] >= 2) ? orc_pipe[2] - 2 : orc_pipe[2];
                jpeg_ro_pipe[3] <= (edge_ro_pipe[2] <= 4) ? jpeg_ro_pipe[2] << 4 : jpeg_ro_pipe[2];
                edge_ro_pipe[3] <= (edge_ro_pipe[2] <= 4) ? edge_ro_pipe[2] : edge_ro_pipe[2] - 4;
            end

            // Stage 4 (jpeg_5, orc_5, jpeg_ro_4, edge_ro_4)
            if (br_pipe[3]) begin // br_4
                jpeg_pipe[4] <= (orc_pipe[3] >= 1) ? jpeg_pipe[3] >> 1 : jpeg_pipe[3];
                orc_pipe[4] <= (orc_pipe[3] >= 1) ? orc_pipe[3] - 1 : orc_pipe[3];
                jpeg_ro_pipe[4] <= (edge_ro_pipe[3] <= 2) ? jpeg_ro_pipe[3] << 2 : jpeg_ro_pipe[3];
                edge_ro_pipe[4] <= (edge_ro_pipe[3] <= 2) ? edge_ro_pipe[3] : edge_ro_pipe[3] - 2;
            end

            // Stage 5 (jpeg_ro_5, edge_ro_5)
            if (br_pipe[4]) begin // br_5
                jpeg_ro_pipe[5] <= (edge_ro_pipe[4] <= 1) ? jpeg_ro_pipe[4] << 1 : jpeg_ro_pipe[4];
                edge_ro_pipe[5] <= (edge_ro_pipe[4] <= 1) ? edge_ro_pipe[4] : edge_ro_pipe[4] - 1;
            end

            // Stage 6 (jpeg_6)
            if (br_pipe[4] || br_pipe[5]) begin // br_5 | br_6 (this is the original logic)
                for (int i = 0; i < 31; i++) begin
                    jpeg_pipe[6][i] <= (rollover_pipe[4] && static_orc_pipe[4] > (31 - i)) ? jpeg_ro_pipe[5][i] : jpeg_pipe[4][i];
                end
                jpeg_pipe[6][0] <= jpeg_pipe[4][0]; // Special case for bit 0
            end
        end
    end

     // ------------------------------------------------------------------------
    // FIFO Mux and Read Request Management
    // ------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst)
            fifo_mux <= 1'b0;
        else if (end_of_block_output)
            fifo_mux <= !fifo_mux; // Toggles between 0 and 1
    end

    // Read Request Logic
    always_ff @(posedge clk) begin
        if (rst) y_read_req <= 1'b0;
        else y_read_req <= (!y_fifo_empty && read_mux == 3'b001);
    end

    always_ff @(posedge clk) begin
        if (rst) cb_read_req <= 1'b0;
        else cb_read_req <= (!cb_fifo_empty && read_mux == 3'b010);
    end

    always_ff @(posedge clk) begin
        if (rst) cr_read_req <= 1'b0;
        else cr_read_req <= (!cr_fifo_empty && read_mux == 3'b100);
    end

    // Data Ready Pipelining
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            data_ready <= 1'b0;
            eobe_1 <= 1'b0;
        end else begin
            data_ready <= br_pipe[5] && rollover_pipe[4]; // br_6 is br_pipe[5], rollover_5 is rollover_pipe[4]
            eobe_1 <= y_eob_empty;
        end
    end

    // Rollover End of Block Logic
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            rollover_eob <= 1'b0;
        else if (br_pipe[2]) // br_3
            rollover_eob <= old_orc_reg >= roll_orc_reg;
    end

    // Pipelining of out_enable signals
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            cr_out_enable_1 <= 1'b0;
            cb_out_enable_1 <= 1'b0;
            y_out_enable_1 <= 1'b0;
        end else begin
            cr_out_enable_1 <= cr_out_enable;
            cb_out_enable_1 <= cb_out_enable;
            y_out_enable_1 <= y_out_enable;
        end
    end

    // Rollover determination
    always_comb begin
        case (bits_mux)
            3'b001: rollover = y_out_enable_1 && !eob_pipe[3] && !eob_early_out_enable; // eob_4 is eob_pipe[3]
            3'b010: rollover = cb_out_enable_1 && cb_out_enable;
            3'b100: rollover = cr_out_enable_1 && cr_out_enable;
            default: rollover = y_out_enable_1 && !eob_pipe[3]; // eob_4
        endcase
    end

    // ORC calculations
    always_ff @(posedge clk or posedge rst) begin
        if (rst) orc <= 5'b0;
        else if (enable_pipe[19]) // enable_20
            orc <= orc_cr + cr_orc_1;
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) orc_cb <= 5'b0;
        else if (eob_pipe[0]) // eob_1
            orc_cb <= orc + y_orc_1;
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst) orc_cr <= 5'b0;
        else if (enable_pipe[4]) // enable_5
            orc_cr <= orc_cb + cb_orc_1;
    end

    // Muxing logic for jpeg data, bits_ready, and orc registers
    always_comb begin
        case (bits_mux)
            3'b001: jpeg_pipe[0] = y_bits_out; // 'jpeg' in original code
            3'b010: jpeg_pipe[0] = cb_bits_out;
            3'b100: jpeg_pipe[0] = cr_bits_out;
            default: jpeg_pipe[0] = y_bits_out;
        endcase

        case (bits_mux)
            3'b001: bits_ready = y_out_enable;
            3'b010: bits_ready = cb_out_enable;
            3'b100: bits_ready = cr_out_enable;
            default: bits_ready = y_out_enable;
        endcase

        case (bits_mux)
            3'b001: sorc_reg = orc;
            3'b010: sorc_reg = orc_cb;
            3'b100: sorc_reg = orc_cr;
            default: sorc_reg = orc;
        endcase

        case (old_orc_mux)
            3'b001: roll_orc_reg = orc;
            3'b010: roll_orc_reg = orc_cb;
            3'b100: roll_orc_reg = orc_cr;
            default: roll_orc_reg = orc;
        endcase

        case (bits_mux)
            3'b001: orc_reg = orc;
            3'b010: orc_reg = orc_cb;
            3'b100: orc_reg = orc_cr;
            default: orc_reg = orc;
        endcase

        case (old_orc_mux)
            3'b001: old_orc_reg = orc_cr;
            3'b010: old_orc_reg = orc;
            3'b100: old_orc_reg = orc_cb;
            default: old_orc_reg = orc_cr;
        endcase
    end

    // Mux state machine
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            bits_mux <= 3'b001; // Y
        else if (enable_pipe[2]) // enable_3
            bits_mux <= 3'b010; // Cb
        else if (enable_pipe[18]) // enable_19
            bits_mux <= 3'b100; // Cr
        else if (enable_pipe[34]) // enable_35
            bits_mux <= 3'b001; // Y
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            old_orc_mux <= 3'b001; // Y
        else if (enable_pipe[0]) // enable_1
            old_orc_mux <= 3'b010; // Cb
        else if (enable_pipe[5]) // enable_6
            old_orc_mux <= 3'b100; // Cr
        else if (enable_pipe[21]) // enable_22
            old_orc_mux <= 3'b001; // Y
    end

    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            read_mux <= 3'b001; // Y
        else if (enable_pipe[0]) // enable_1
            read_mux <= 3'b010; // Cb
        else if (enable_pipe[16]) // enable_17
            read_mux <= 3'b100; // Cr
        else if (enable_pipe[32]) // enable_33
            read_mux <= 3'b001; // Y
    end

    // Latch for orc_1 values
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            cr_orc_1 <= 5'b0;
            cb_orc_1 <= 5'b0;
            y_orc_1 <= 5'b0;
        end else if (end_of_block_output) begin
            cr_orc_1 <= cr_orc;
            cb_orc_1 <= cb_orc;
            y_orc_1 <= y_orc;
        end
    end

     // ------------------------------------------------------------------------
    // Final JPEG Output and ORC Calculation
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            JPEG_bitstream <= 32'b0;
        end else if (br_pipe[6] && rollover_pipe[5]) begin
            JPEG_bitstream <= jpeg_pipe[6];
        end else if (br_pipe[5]) begin
            for (int i = 0; i < 32; i++) begin
                if (static_orc_pipe[5] <= (31 - i)) begin
                    JPEG_bitstream[i] <= jpeg_pipe[6][i];
                end else begin
                    JPEG_bitstream[i] <= 1'b0;
                end
            end
        end else begin
            JPEG_bitstream <= 32'b0;
        end
    end

endmodule

