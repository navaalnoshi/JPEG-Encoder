/* This module takes the y, cb, and cr inputs from the pre_fifo module,
and it combines the bits into the jpeg_bitstream.  It uses 3 FIFO's to 
write the y, cb, and cr data while it's processing the data.  The output
of this module goes to the input of the ff_checker module, to check for
any FF's in the bitstream.
*/
`timescale 1ns / 100ps


module fifo_out #( // Using #() for parameters, though none are present here
    // Parameters can be added here if needed, e.g., parameter DATA_WIDTH = 32;
) (
    input logic         clk,
    input logic         rst,
    input logic         enable,
    input logic [23:0]  data_in,
    output logic [31:0] JPEG_bitstream,
    output logic        data_ready,
    output logic [4:0]  orc_reg
);

    // --- Wire Declarations ---
    // Using 'logic' for wires instead of 'wire' is generally preferred in SystemVerilog
    // as 'logic' can be driven by a 'wire', 'reg', 'assign', or 'always' block,
    // providing more flexibility and consistency.
    logic [31:0]  cb_JPEG_bitstream, cr_JPEG_bitstream, y_JPEG_bitstream;
    logic [4:0]   cr_orc, cb_orc, y_orc;
    logic [31:0]  y_bits_out;
    logic         y_out_enable;
    logic         cb_data_ready, cr_data_ready, y_data_ready;
    logic         end_of_block_output, y_eob_empty;
    logic         cb_eob_empty, cr_eob_empty;
    logic         y_fifo_empty;

    // --- Register Declarations ---
    // All 'reg' declarations are replaced with 'logic'.
    logic [4:0]   orc, orc_cb, orc_cr, old_orc_reg, sorc_reg, roll_orc_reg;
    logic [4:0]   orc_1, orc_2, orc_3, orc_4, orc_5, orc_reg_delay;
    logic [4:0]   static_orc_1, static_orc_2, static_orc_3, static_orc_4, static_orc_5;
    logic [4:0]   static_orc_6;
    logic [4:0]   edge_ro_1, edge_ro_2, edge_ro_3, edge_ro_4, edge_ro_5;
    logic [31:0]  jpeg_ro_1, jpeg_ro_2, jpeg_ro_3, jpeg_ro_4, jpeg_ro_5, jpeg_delay;
    logic [31:0]  jpeg, jpeg_1, jpeg_2, jpeg_3, jpeg_4, jpeg_5, jpeg_6; // JPEG_bitstream is an output, no longer needs to be reg
    logic [4:0]   cr_orc_1, cb_orc_1, y_orc_1;
    logic         cr_out_enable_1, cb_out_enable_1, y_out_enable_1, eob_1;
    logic         eob_2, eob_3, eob_4;
    logic         enable_1, enable_2, enable_3, enable_4, enable_5;
    logic         enable_6, enable_7, enable_8, enable_9, enable_10;
    logic         enable_11, enable_12, enable_13, enable_14, enable_15;
    logic         enable_16, enable_17, enable_18, enable_19, enable_20;
    logic         enable_21, enable_22, enable_23, enable_24, enable_25;
    logic         enable_26, enable_27, enable_28, enable_29, enable_30;
    logic         enable_31, enable_32, enable_33, enable_34, enable_35;
    logic [2:0]   bits_mux, old_orc_mux, read_mux;
    logic         bits_ready, br_1, br_2, br_3, br_4, br_5, br_6, br_7, br_8;
    logic         rollover, rollover_1, rollover_2, rollover_3, rollover_eob;
    logic         rollover_4, rollover_5, rollover_6, rollover_7;
    // data_ready is an output, no longer needs to be reg
    logic         eobe_1, cb_read_req, cr_read_req, y_read_req;
    logic         eob_early_out_enable, fifo_mux;

    // --- Derived signals for dual FIFOs (unchanged logic, just 'wire' to 'logic') ---
    logic [31:0] cr_bits_out1, cr_bits_out2, cb_bits_out1, cb_bits_out2;
    logic cr_fifo_empty1, cr_fifo_empty2, cb_fifo_empty1, cb_fifo_empty2;
    logic cr_out_enable1, cr_out_enable2, cb_out_enable1, cb_out_enable2;

    // Using continuous assignments for combinational logic
    assign cb_write_enable = cb_data_ready && !cb_eob_empty;
    assign cr_write_enable = cr_data_ready && !cr_eob_empty;
    assign y_write_enable = y_data_ready && !y_eob_empty;

    // Type casting literals to match bit width where appropriate (e.g., 1'b0, 32'b0)
    assign cr_read_req1 = fifo_mux ? 1'b0 : cr_read_req;
    assign cr_read_req2 = fifo_mux ? cr_read_req : 1'b0;
    assign cr_JPEG_bitstream1 = fifo_mux ? cr_JPEG_bitstream : 32'b0;
    assign cr_JPEG_bitstream2 = fifo_mux ? 32'b0 : cr_JPEG_bitstream;
    assign cr_write_enable1 = fifo_mux && cr_write_enable;
    assign cr_write_enable2 = !fifo_mux && cr_write_enable;
    assign cr_bits_out = fifo_mux ? cr_bits_out2 : cr_bits_out1;
    assign cr_fifo_empty = fifo_mux ? cr_fifo_empty2 : cr_fifo_empty1;
    assign cr_out_enable = fifo_mux ? cr_out_enable2 : cr_out_enable1;

    assign cb_read_req1 = fifo_mux ? 1'b0 : cb_read_req;
    assign cb_read_req2 = fifo_mux ? cb_read_req : 1'b0;
    assign cb_JPEG_bitstream1 = fifo_mux ? cb_JPEG_bitstream : 32'b0;
    assign cb_JPEG_bitstream2 = fifo_mux ? 32'b0 : cb_JPEG_bitstream;
    assign cb_write_enable1 = fifo_mux && cb_write_enable;
    assign cb_write_enable2 = !fifo_mux && cb_write_enable;
    assign cb_bits_out = fifo_mux ? cb_bits_out2 : cb_bits_out1;
    assign cb_fifo_empty = fifo_mux ? cb_fifo_empty2 : cb_fifo_empty1;
    assign cb_out_enable = fifo_mux ? cb_out_enable2 : cb_out_enable1;

    // --- Module Instantiations ---
    // Instantiations remain largely the same, but with 'logic' types for connections
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
        .write_enable(y_write_enable),
        .read_data(y_bits_out),
        .fifo_empty(y_fifo_empty),
        .rdata_valid(y_out_enable)
    );

    // --- Always Blocks (Sequential Logic) ---
    // 'always @(posedge clk)' is replaced by 'always_ff @(posedge cl)' for clarity
    // and to explicitly indicate a flip-flop inference.

    always_ff @(posedge clk) begin
        if (rst)
            fifo_mux <= 1'b0; // Use 1'b0 for single-bit literal
        else if (end_of_block_output)
            fifo_mux <= ~fifo_mux; // Toggling boolean value
    end

    always_ff @(posedge clk) begin
        if (y_fifo_empty || read_mux != 3'b001)
            y_read_req <= 1'b0;
        else if (!y_fifo_empty && read_mux == 3'b001)
            y_read_req <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (cb_fifo_empty || read_mux != 3'b010)
            cb_read_req <= 1'b0;
        else if (!cb_fifo_empty && read_mux == 3'b010)
            cb_read_req <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (cr_fifo_empty || read_mux != 3'b100)
            cr_read_req <= 1'b0;
        else if (!cr_fifo_empty && read_mux == 3'b100)
            cr_read_req <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            br_1 <= 1'b0; br_2 <= 1'b0; br_3 <= 1'b0; br_4 <= 1'b0; br_5 <= 1'b0; br_6 <= 1'b0;
            br_7 <= 1'b0; br_8 <= 1'b0;
            static_orc_1 <= 5'b0; static_orc_2 <= 5'b0; static_orc_3 <= 5'b0;
            static_orc_4 <= 5'b0; static_orc_5 <= 5'b0; static_orc_6 <= 5'b0;
            data_ready <= 1'b0; eobe_1 <= 1'b0;
        end else begin
            br_1 <= bits_ready & !eobe_1; br_2 <= br_1; br_3 <= br_2;
            br_4 <= br_3; br_5 <= br_4; br_6 <= br_5;
            br_7 <= br_6; br_8 <= br_7;
            static_orc_1 <= sorc_reg; static_orc_2 <= static_orc_1;
            static_orc_3 <= static_orc_2; static_orc_4 <= static_orc_3;
            static_orc_5 <= static_orc_4; static_orc_6 <= static_orc_5;
            data_ready <= br_6 & rollover_5;
            eobe_1 <= y_eob_empty;
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            rollover_eob <= 1'b0;
        else if (br_3)
            rollover_eob <= old_orc_reg >= roll_orc_reg;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            rollover_1 <= 1'b0; rollover_2 <= 1'b0; rollover_3 <= 1'b0;
            rollover_4 <= 1'b0; rollover_5 <= 1'b0; rollover_6 <= 1'b0;
            rollover_7 <= 1'b0; eob_1 <= 1'b0; eob_2 <= 1'b0;
            eob_3 <= 1'b0; eob_4 <= 1'b0;
            eob_early_out_enable <= 1'b0;
        end else begin
            rollover_1 <= rollover; rollover_2 <= rollover_1;
            rollover_3 <= rollover_2;
            rollover_4 <= rollover_3 | rollover_eob;
            rollover_5 <= rollover_4; rollover_6 <= rollover_5;
            rollover_7 <= rollover_6; eob_1 <= end_of_block_output;
            eob_2 <= eob_1; eob_3 <= eob_2; eob_4 <= eob_3;
            eob_early_out_enable <= y_out_enable & y_out_enable_1 & eob_2;
        end
    end

    // Using 'always_comb' for combinational logic blocks previously in 'always @(posedge clk)' but were not sequential.
    // However, the original code implies these are sequential as they are in @(posedge clk) blocks.
    // I'll keep them as 'always_ff' with correct reset where applicable,
    // assuming they are meant to be registered values updated on clock edge.
    // If these are intended to be purely combinational, they should be 'always_comb'.

    always_ff @(posedge clk) begin
        case (bits_mux)
            3'b001:  rollover <= y_out_enable_1 & !eob_4 & !eob_early_out_enable;
            3'b010:  rollover <= cb_out_enable_1 & cb_out_enable;
            3'b100:  rollover <= cr_out_enable_1 & cr_out_enable;
            default: rollover <= y_out_enable_1 & !eob_4;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst)
            orc <= 5'b0;
        else if (enable_20)
            orc <= orc_cr + cr_orc_1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            orc_cb <= 5'b0;
        else if (eob_1)
            orc_cb <= orc + y_orc_1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            orc_cr <= 5'b0;
        else if (enable_5)
            orc_cr <= orc_cb + cb_orc_1;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            cr_out_enable_1 <= 1'b0; cb_out_enable_1 <= 1'b0; y_out_enable_1 <= 1'b0;
        end else begin
            cr_out_enable_1 <= cr_out_enable;
            cb_out_enable_1 <= cb_out_enable;
            y_out_enable_1 <= y_out_enable;
        end
    end

    always_ff @(posedge clk) begin
        case (bits_mux)
            3'b001:  jpeg <= y_bits_out;
            3'b010:  jpeg <= cb_bits_out;
            3'b100:  jpeg <= cr_bits_out;
            default: jpeg <= y_bits_out;
        endcase
    end

    always_ff @(posedge clk) begin
        case (bits_mux)
            3'b001:  bits_ready <= y_out_enable;
            3'b010:  bits_ready <= cb_out_enable;
            3'b100:  bits_ready <= cr_out_enable;
            default: bits_ready <= y_out_enable;
        endcase
    end

    always_ff @(posedge clk) begin
        case (bits_mux)
            3'b001:  sorc_reg <= orc;
            3'b010:  sorc_reg <= orc_cb;
            3'b100:  sorc_reg <= orc_cr;
            default: sorc_reg <= orc;
        endcase
    end

    always_ff @(posedge clk) begin
        case (old_orc_mux)
            3'b001:  roll_orc_reg <= orc;
            3'b010:  roll_orc_reg <= orc_cb;
            3'b100:  roll_orc_reg <= orc_cr;
            default: roll_orc_reg <= orc;
        endcase
    end

    always_ff @(posedge clk) begin
        case (bits_mux)
            3'b001:  orc_reg <= orc;
            3'b010:  orc_reg <= orc_cb;
            3'b100:  orc_reg <= orc_cr;
            default: orc_reg <= orc;
        endcase
    end

    always_ff @(posedge clk) begin
        case (old_orc_mux)
            3'b001:  old_orc_reg <= orc_cr;
            3'b010:  old_orc_reg <= orc;
            3'b100:  old_orc_reg <= orc_cb;
            default: old_orc_reg <= orc_cr;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst)
            bits_mux <= 3'b001; // Y
        else if (enable_3)
            bits_mux <= 3'b010; // Cb
        else if (enable_19)
            bits_mux <= 3'b100; // Cr
        else if (enable_35)
            bits_mux <= 3'b001; // Y
    end

    always_ff @(posedge clk) begin
        if (rst)
            old_orc_mux <= 3'b001; // Y
        else if (enable_1)
            old_orc_mux <= 3'b010; // Cb
        else if (enable_6)
            old_orc_mux <= 3'b100; // Cr
        else if (enable_22)
            old_orc_mux <= 3'b001; // Y
    end

    always_ff @(posedge clk) begin
        if (rst)
            read_mux <= 3'b001; // Y
        else if (enable_1)
            read_mux <= 3'b010; // Cb
        else if (enable_17)
            read_mux <= 3'b100; // Cr
        else if (enable_33)
            read_mux <= 3'b001; // Y
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            cr_orc_1 <= 5'b0; cb_orc_1 <= 5'b0; y_orc_1 <= 5'b0;
        end else if (end_of_block_output) begin
            cr_orc_1 <= cr_orc;
            cb_orc_1 <= cb_orc;
            y_orc_1 <= y_orc;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            jpeg_ro_5 <= 32'b0; edge_ro_5 <= 5'b0;
        end else if (br_5) begin
            jpeg_ro_5 <= (edge_ro_4 <= 1) ? jpeg_ro_4 << 1 : jpeg_ro_4;
            edge_ro_5 <= (edge_ro_4 <= 1) ? edge_ro_4 : edge_ro_4 - 1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            jpeg_5 <= 32'b0; orc_5 <= 5'b0; jpeg_ro_4 <= 32'b0; edge_ro_4 <= 5'b0;
        end else if (br_4) begin
            jpeg_5 <= (orc_4 >= 1) ? jpeg_4 >> 1 : jpeg_4;
            orc_5 <= (orc_4 >= 1) ? orc_4 - 1 : orc_4;
            jpeg_ro_4 <= (edge_ro_3 <= 2) ? jpeg_ro_3 << 2 : jpeg_ro_3;
            edge_ro_4 <= (edge_ro_3 <= 2) ? edge_ro_3 : edge_ro_3 - 2;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            jpeg_4 <= 32'b0; orc_4 <= 5'b0; jpeg_ro_3 <= 32'b0; edge_ro_3 <= 5'b0;
        end else if (br_3) begin
            jpeg_4 <= (orc_3 >= 2) ? jpeg_3 >> 2 : jpeg_3;
            orc_4 <= (orc_3 >= 2) ? orc_3 - 2 : orc_3;
            jpeg_ro_3 <= (edge_ro_2 <= 4) ? jpeg_ro_2 << 4 : jpeg_ro_2;
            edge_ro_3 <= (edge_ro_2 <= 4) ? edge_ro_2 : edge_ro_2 - 4;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            jpeg_3 <= 32'b0; orc_3 <= 5'b0; jpeg_ro_2 <= 32'b0; edge_ro_2 <= 5'b0;
        end else if (br_2) begin
            jpeg_3 <= (orc_2 >= 4) ? jpeg_2 >> 4 : jpeg_2;
            orc_3 <= (orc_2 >= 4) ? orc_2 - 4 : orc_2;
            jpeg_ro_2 <= (edge_ro_1 <= 8) ? jpeg_ro_1 << 8 : jpeg_ro_1;
            edge_ro_2 <= (edge_ro_1 <= 8) ? edge_ro_1 : edge_ro_1 - 8;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            jpeg_2 <= 32'b0; orc_2 <= 5'b0; jpeg_ro_1 <= 32'b0; edge_ro_1 <= 5'b0;
        end else if (br_1) begin
            jpeg_2 <= (orc_1 >= 8) ? jpeg_1 >> 8 : jpeg_1;
            orc_2 <= (orc_1 >= 8) ? orc_1 - 8 : orc_1;
            jpeg_ro_1 <= (orc_reg_delay <= 16) ? jpeg_delay << 16 : jpeg_delay;
            edge_ro_1 <= (orc_reg_delay <= 16) ? orc_reg_delay : orc_reg_delay - 16;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            jpeg_1 <= 32'b0; orc_1 <= 5'b0; jpeg_delay <= 32'b0; orc_reg_delay <= 5'b0;
        end else if (bits_ready) begin
            jpeg_1 <= (orc_reg >= 16) ? jpeg >> 16 : jpeg;
            orc_1 <= (orc_reg >= 16) ? orc_reg - 16 : orc_reg;
            jpeg_delay <= jpeg;
            orc_reg_delay <= orc_reg;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            enable_1 <= 1'b0; enable_2 <= 1'b0; enable_3 <= 1'b0; enable_4 <= 1'b0; enable_5 <= 1'b0;
            enable_6 <= 1'b0; enable_7 <= 1'b0; enable_8 <= 1'b0; enable_9 <= 1'b0; enable_10 <= 1'b0;
            enable_11 <= 1'b0; enable_12 <= 1'b0; enable_13 <= 1'b0; enable_14 <= 1'b0; enable_15 <= 1'b0;
            enable_16 <= 1'b0; enable_17 <= 1'b0; enable_18 <= 1'b0; enable_19 <= 1'b0; enable_20 <= 1'b0;
            enable_21 <= 1'b0;
            enable_22 <= 1'b0; enable_23 <= 1'b0; enable_24 <= 1'b0; enable_25 <= 1'b0; enable_26 <= 1'b0;
            enable_27 <= 1'b0; enable_28 <= 1'b0; enable_29 <= 1'b0; enable_30 <= 1'b0;
            enable_31 <= 1'b0;
            enable_32 <= 1'b0; enable_33 <= 1'b0; enable_34 <= 1'b0; enable_35 <= 1'b0;
        end else begin
            enable_1 <= end_of_block_output; enable_2 <= enable_1;
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

    // --- Generate Block for individual JPEG_bitstream bits ---
    // This uses a 'generate' block with a 'for' loop to condense repetitive assignments.
    // The indexing `(31 - i)` is used to match your original Verilog's counting logic
    // where `static_orc_6 <= 0` corresponded to `JPEG_bitstream[31]`.
    generate
        for (genvar i = 0; i < 32; i++) begin : assign_jpeg_bit
            always_ff @(posedge clk) begin
                if (rst)
                    JPEG_bitstream[i] <= 1'b0;
                else if (br_7 && rollover_6)
                    JPEG_bitstream[i] <= jpeg_6[i];
                else if (br_6 && static_orc_6 <= (31 - i)) // This condition is crucial for bit-by-bit selection
                    JPEG_bitstream[i] <= jpeg_6[i];
            end
        end
    endgenerate

    // --- JPEG_6 assignment block ---
    always_ff @(posedge clk) begin
        if (rst) begin
            jpeg_6 <= 32'b0;
        end else if (br_5 || br_6) begin
            // Use a 'for' loop for more concise code instead of repetitive lines
            for (int i = 0; i < 32; i++) begin
                if (rollover_5 && static_orc_5 > (31 - i)) begin // Adjusted condition for bit index
                    jpeg_6[i] <= jpeg_ro_5[i];
                end else begin
                    jpeg_6[i] <= jpeg_5[i];
                end
            end
        end
    end

endmodule