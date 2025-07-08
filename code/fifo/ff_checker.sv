/*
This module takes the JPEG_bitstream as its input, and checks for any FF values in
the bitstream.  When it finds an FF in the bitstream, this module puts a 00 after the FF,
and then continues with the rest of the bitstream after the 00, per the JPEG standard.
*/
`timescale 1ns / 100ps

module ff_checker (
    input logic         clk,
    input logic         rst,
    input logic         end_of_file_signal,
    input logic [31:0]  JPEG_in,
    input logic         data_ready_in,
    input logic [4:0]   orc_reg_in,
    output logic [31:0] JPEG_bitstream_1,
    output logic        data_ready_1,
    output logic [4:0]  orc_reg,
    output logic        eof_data_partial_ready
);

    // --- Local Signal Declarations (all 'reg' replaced with 'logic') ---
    logic first_2bytes, second_2bytes, third_2bytes, fourth_2bytes;
    logic first_2bytes_eof, second_2bytes_eof, third_2bytes_eof;
    logic fourth_2bytes_eof, fifth_2bytes_eof, s2b, t2b;
    logic [79:0] JPEG_eof_6, JPEG_eof_7;
    logic [63:0] JPEG_5, JPEG_eof_5_1, JPEG_6, JPEG_7;
    logic [55:0] JPEG_4, JPEG_eof_3, JPEG_eof_4, JPEG_eof_5;
    logic [47:0] JPEG_3, JPEG_eof_2;
    logic [39:0] JPEG_2, JPEG_eof_1;
    logic [31:0] JPEG_1, JPEG_ro, JPEG_bitstream; // JPEG_bitstream_1 is an output
    logic [31:0] JPEG_eof, JPEG_eof_ro;
    logic [31:0] JPEG_bitstream_eof;
    logic [15:0] JPEG_eof_ro_ro;
    logic [87:0] JPEG_out, JPEG_out_1, JPEG_pf;
    logic [23:0] JPEG_ro_ro;
    logic dr_in_1, dr_in_2, dr_in_3, dr_in_4, dr_in_5, dr_in_6;
    logic dr_in_7, dr_in_8;
    logic rollover, rollover_1, rollover_2, rollover_3, rollover_4, rollover_5;
    logic rollover_pf, rpf_1;
    logic [1:0] FF_count, FF_count_1, FF_eof_shift;
    logic [2:0] count_total, ct_1;
    logic [1:0] ffc_1, ffc_2, ffc_3, ffc_4, ffc_5, ffc_6, ffc_7;
    logic [1:0] ffc_postfifo, count_total_eof;
    logic [4:0] orc_input;
    // orc_reg is an output
    logic [6:0] extra_bits_eof, extra_bits_eof_1;
    logic data_ready; // data_ready_1 is an output
    logic write_enable, read_req, rdv_1;
    logic end_of_file_enable, eof_count_enable;
    // eof_data_partial_ready is an output
    logic eof_dpr_1, eof_dpr_2;
    logic end_of_file_enable_hold, eof_data_ready;
    logic eof_data_ready_1, eof_bits_1, eof_bits_2, eof_bits_3;
    logic [8:0] eof_count;

    // FIFO interface signals
    logic [90:0] read_data;
    logic [90:0] write_data; // Declared as logic for consistent driving
    logic fifo_empty, rdata_valid;

    // Continuous assignment for write_data
    assign write_data = { JPEG_out_1, ffc_7, rollover_5 };

    // --- Module Instantiation ---
    sync_fifo_ff u18 (
        .clk(clk),
        .rst(rst),
        .read_req(read_req),
        .write_data(write_data),
        .write_enable(write_enable),
        .rollover_write(rollover_5),
        .read_data(read_data),
        .fifo_empty(fifo_empty),
        .rdata_valid(rdata_valid)
    );

    // The following blocks are sequential logic, so `always_ff` is used.

    always_ff @(posedge clk) begin
        if (rst)
            eof_data_partial_ready <= 1'b0;
        else if (eof_bits_1)
            eof_data_partial_ready <= (extra_bits_eof_1 > 7'd0) && (extra_bits_eof_1 < 7'd32);
        else
            eof_data_partial_ready <= eof_dpr_1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            eof_dpr_1 <= 1'b0;
        else if (eof_bits_1)
            eof_dpr_1 <= (extra_bits_eof_1 > 7'd32) && (extra_bits_eof_1 < 7'd64);
        else
            eof_dpr_1 <= eof_dpr_2;
    end

    always_ff @(posedge clk) begin
        if (rst)
            eof_dpr_2 <= 1'b0;
        else if (eof_bits_1)
            eof_dpr_2 <= extra_bits_eof_1 > 7'd64;
        else
            eof_dpr_2 <= 1'b0;
    end

    always_ff @(posedge clk) begin
        if (rst)
            eof_data_ready_1 <= 1'b0;
        else if (end_of_file_enable)
            eof_data_ready_1 <= (extra_bits_eof_1 > 7'd31);
        else if (eof_bits_1 || eof_bits_2)
            eof_data_ready_1 <= eof_data_ready;
    end

    always_ff @(posedge clk) begin
        if (rst)
            eof_data_ready <= 1'b0;
        else if (end_of_file_enable)
            eof_data_ready <= (extra_bits_eof_1 > 7'd63);
        else if (eof_bits_1)
            eof_data_ready <= 1'b0;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            eof_bits_1 <= 1'b0; eof_bits_2 <= 1'b0; eof_bits_3 <= 1'b0;
        end else begin
            eof_bits_1 <= end_of_file_enable;
            eof_bits_2 <= eof_bits_1;
            eof_bits_3 <= eof_bits_2;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_bitstream_eof <= 32'b0; JPEG_eof_ro <= 32'b0;
        end else if (end_of_file_enable) begin
            JPEG_bitstream_eof <= JPEG_eof_7[79:48];
            JPEG_eof_ro <= JPEG_eof_7[47:16];
        end else if (eof_bits_1 || eof_bits_2) begin
            JPEG_bitstream_eof <= JPEG_eof_ro;
            JPEG_eof_ro <= { JPEG_eof_ro_ro, 16'b0 }; // Explicit 16'b0
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            JPEG_eof_ro_ro <= 16'b0;
        else if (end_of_file_enable)
            JPEG_eof_ro_ro <= JPEG_eof_7[15:0];
    end

    always_ff @(posedge clk) begin // These registers combine the previous leftover bits with the end of file bits
        if (rst) begin
            JPEG_eof_7 <= 80'b0; JPEG_eof_6 <= 80'b0; JPEG_eof_5_1 <= 64'b0;
            FF_eof_shift <= 2'b0; FF_count_1 <= 2'b0;
        end else begin
            JPEG_eof_7[79:72] <= (FF_count_1 > 2'd0) ? JPEG_ro[31:24] : JPEG_eof_6[79:72];
            JPEG_eof_7[71:64] <= (FF_count_1 > 2'd1) ? JPEG_ro[23:16] : JPEG_eof_6[71:64];
            JPEG_eof_7[63:56] <= (FF_count_1 > 2'd2) ? JPEG_ro[15:8] : JPEG_eof_6[63:56];
            JPEG_eof_7[55:0] <= JPEG_eof_6[55:0];
            JPEG_eof_6 <= JPEG_eof_5_1 << { FF_eof_shift[1], 4'b0 }; // Explicit 4'b0
            JPEG_eof_5_1 <= JPEG_eof_5 << { FF_eof_shift[0], 3'b0 }; // Explicit 3'b0
            FF_eof_shift <= 2'b11 - FF_count;
            FF_count_1 <= FF_count;
        end
    end

    always_ff @(posedge clk) begin // These registers generate the end of file bits
        if (rst) begin
            orc_reg <= 5'b0; extra_bits_eof <= 7'b0;
            extra_bits_eof_1 <= 7'b0; count_total_eof <= 2'b0;
            JPEG_eof_5 <= 56'b0; fifth_2bytes_eof <= 1'b0;
            JPEG_eof_4 <= 56'b0; fourth_2bytes_eof <= 1'b0;
            JPEG_eof_3 <= 56'b0; third_2bytes_eof <= 1'b0;
            JPEG_eof_2 <= 48'b0;
            second_2bytes_eof <= 1'b0; JPEG_eof_1 <= 40'b0;
            first_2bytes_eof <= 1'b0; s2b <= 1'b0;
            t2b <= 1'b0; orc_input <= 5'b0;
        end else begin
            orc_reg <= extra_bits_eof_1[4:0];
            extra_bits_eof <= { 2'b0, orc_input } + { 2'b0, FF_count, 3'b0 }; // Explicit zeros
            extra_bits_eof_1 <= extra_bits_eof + { 2'b0, count_total_eof, 3'b0 }; // Explicit zeros
            count_total_eof <= first_2bytes_eof + s2b + t2b;

            // Simplified bit manipulation using concatenation and shifts
            JPEG_eof_5[55:16] <= JPEG_eof_4[55:16];
            JPEG_eof_5[15:0] <= fifth_2bytes_eof ? {8'h00, JPEG_eof_4[15:8]} : JPEG_eof_4[15:0];
            fifth_2bytes_eof <= (JPEG_eof_4[23:16] == 8'hFF);

            JPEG_eof_4[55:24] <= JPEG_eof_3[55:24];
            JPEG_eof_4[23:0] <= fourth_2bytes_eof ? {8'h00, JPEG_eof_3[23:16], JPEG_eof_3[15:8]} : JPEG_eof_3[23:0];
            fourth_2bytes_eof <= (JPEG_eof_3[31:24] == 8'hFF);

            JPEG_eof_3[55:32] <= JPEG_eof_2[47:24];
            JPEG_eof_3[31:0] <= third_2bytes_eof ? {8'h00, JPEG_eof_2[23:16], JPEG_eof_2[15:8], JPEG_eof_2[7:0]} : JPEG_eof_2[31:0]; // Careful with `JPEG_eof_2[7:0]` being `8'b0` in original
            // Re-evaluating original logic for JPEG_eof_3[7:0]: `third_2bytes_eof ? JPEG_eof_2[7:0] : 8'b00000000;`
            // This suggests zeroing out the last byte if third_2bytes_eof is true.
            // My previous simplified logic was incorrect for this specific byte. Let's fix.
            if (third_2bytes_eof) begin
                JPEG_eof_3[31:24] <= 8'h00;
                JPEG_eof_3[23:16] <= JPEG_eof_2[23:16];
                JPEG_eof_3[15:8] <= JPEG_eof_2[15:8];
                JPEG_eof_3[7:0] <= 8'h00; // Original had 8'b0 for this specific condition
            end else begin
                JPEG_eof_3[31:24] <= JPEG_eof_2[23:16];
                JPEG_eof_3[23:16] <= JPEG_eof_2[15:8];
                JPEG_eof_3[15:8] <= JPEG_eof_2[7:0];
                JPEG_eof_3[7:0] <= 8'h00; // Original also had 8'b0 for this specific condition
            end
            third_2bytes_eof <= (JPEG_eof_2[31:24] == 8'hFF);

            JPEG_eof_2[47:32] <= JPEG_eof_1[39:24];
            if (second_2bytes_eof) begin
                JPEG_eof_2[31:24] <= 8'h00;
                JPEG_eof_2[23:16] <= JPEG_eof_1[23:16];
                JPEG_eof_2[15:8] <= JPEG_eof_1[15:8];
                JPEG_eof_2[7:0] <= 8'h00; // Original had 8'b0 for this
            end else begin
                JPEG_eof_2[31:24] <= JPEG_eof_1[23:16];
                JPEG_eof_2[23:16] <= JPEG_eof_1[15:8];
                JPEG_eof_2[15:8] <= JPEG_eof_1[7:0];
                JPEG_eof_2[7:0] <= 8'h00; // Original also had 8'b0 for this
            end
            second_2bytes_eof <= (JPEG_eof_1[31:24] == 8'hFF);

            JPEG_eof_1[39:32] <= JPEG_eof[31:24];
            if (first_2bytes_eof) begin
                JPEG_eof_1[31:24] <= 8'h00;
                JPEG_eof_1[23:16] <= JPEG_eof[23:16];
                JPEG_eof_1[15:8] <= JPEG_eof[15:8];
                JPEG_eof_1[7:0] <= 8'h00; // Original had 8'b0 for this
            end else begin
                JPEG_eof_1[31:24] <= JPEG_eof[23:16];
                JPEG_eof_1[23:16] <= JPEG_eof[15:8];
                JPEG_eof_1[15:8] <= JPEG_eof[7:0];
                JPEG_eof_1[7:0] <= 8'h00; // Original also had 8'b0 for this
            end
            first_2bytes_eof <= (JPEG_eof[31:24] == 8'hFF);

            s2b <= (JPEG_eof[23:16] == 8'hFF);
            t2b <= (JPEG_eof[15:8] == 8'hFF);
            orc_input <= orc_reg_in;
        end
    end

    // Using a generate block for the JPEG_eof assignments
    generate
        for (genvar i = 0; i < 31; i++) begin : assign_jpeg_eof_bits
            always_ff @(posedge clk) begin
                if (rst)
                    JPEG_eof[i] <= 1'b0;
                else
                    JPEG_eof[i] <= (orc_reg_in > i) ? JPEG_in[i] : 1'b0;
            end
        end
        // Special case for JPEG_eof[0] which is always 0 in original code
        always_ff @(posedge clk) begin : assign_jpeg_eof_bit_0
            if (rst)
                JPEG_eof[0] <= 1'b0;
            else
                JPEG_eof[0] <= 1'b0;
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (rst)
            eof_count_enable <= 1'b0;
        else if (end_of_file_enable_hold)
            eof_count_enable <= 1'b0;
        else if (end_of_file_signal)
            eof_count_enable <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            eof_count <= 9'b0;
        else if (!eof_count_enable)
            eof_count <= 9'b0;
        else if (eof_count_enable)
            eof_count <= eof_count + 1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            end_of_file_enable <= 1'b0;
        else if (eof_count != 9'b011110000) // Using explicit bit width
            end_of_file_enable <= 1'b0;
        else if (eof_count == 9'b011110000)
            end_of_file_enable <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            end_of_file_enable_hold <= 1'b0;
        else if (end_of_file_enable)
            end_of_file_enable_hold <= 1'b1;
    end

    // --- End of End of File Section ---

    always_ff @(posedge clk) begin
        if (rst) begin
            data_ready_1 <= 1'b0; JPEG_bitstream_1 <= 32'b0;
        end else begin
            data_ready_1 <= data_ready || eof_data_ready_1;
            JPEG_bitstream_1 <= (eof_bits_1 || eof_bits_2 || eof_bits_3) ?
                                JPEG_bitstream_eof : JPEG_bitstream;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            data_ready <= 1'b0; rdv_1 <= 1'b0; rpf_1 <= 1'b0;
        end else begin
            data_ready <= rdv_1 || rpf_1;
            rdv_1 <= rdata_valid;
            rpf_1 <= rollover_pf & !rpf_1; // there can't be 2 rollover's in a row
                                            // because after the first rollover, the next fifo entry is dummy data
        end
    end

    // Using a generate block for JPEG_bitstream assignments
    generate
        for (genvar i = 0; i < 4; i++) begin : assign_jpeg_bitstream_byte
            always_ff @(posedge clk) begin
                if (rst)
                    JPEG_bitstream[i*8 +: 8] <= 8'b0; // Selects bytes: [7:0], [15:8], [23:16], [31:24]
                else if (rdv_1 && ffc_postfifo < (4 - i) && !rpf_1)
                    JPEG_bitstream[i*8 +: 8] <= JPEG_pf[63 + i*8 +: 8];
                else if (rpf_1 || (rdv_1 && ffc_postfifo >= (4 - i))) // Adjusted condition
                    JPEG_bitstream[i*8 +: 8] <= JPEG_ro[i*8 +: 8];
            end
        end
    endgenerate


    always_ff @(posedge clk) begin
        if (rst)
            JPEG_ro <= 32'b0;
        else if (rdv_1 && !rpf_1)
            JPEG_ro <= JPEG_pf[55:24];
        else if (rpf_1)
            JPEG_ro[31:8] <= JPEG_ro_ro;
            // JPEG_ro[7:0] maintains its value from previous clock cycle or reset
            // This is implicitly handled by the default no-change behavior in always_ff
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_ro_ro <= 24'b0;
        end else if (rdv_1) begin
            JPEG_ro_ro <= JPEG_pf[23:0];
        end
    end

    // This block likely intended to be combinational (read_req based on fifo_empty)
    // but was placed in an always @(posedge clk) block.
    // Assuming it should be combinational (level-sensitive logic) as it doesn't store state.
    always_comb begin
        if (fifo_empty)
            read_req = 1'b0;
        else // !fifo_empty
            read_req = 1'b1;
    end

    always_ff @(posedge clk) begin
        if (rst)
            rollover_pf <= 1'b0;
        else if (!rdata_valid)
            rollover_pf <= 1'b0;
        else if (rdata_valid)
            rollover_pf <= read_data[0];
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_pf <= 88'b0; ffc_postfifo <= 2'b0;
        end else if (rdata_valid) begin
            JPEG_pf <= read_data[90:3];
            ffc_postfifo <= read_data[2:1];
        end
    end

    always_comb begin // write_enable is also combinational from the description
        if (!dr_in_8)
            write_enable = 1'b0;
        else // dr_in_8
            write_enable = 1'b1;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_out_1 <= 88'b0; ffc_7 <= 2'b0;
        end else if (dr_in_8) begin
            JPEG_out_1 <= ffc_6[0] ? JPEG_out : (JPEG_out << 8); // Explicit shift
            ffc_7 <= ffc_6;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_out <= 88'b0; ffc_6 <= 2'b0;
        end else if (dr_in_7) begin
            JPEG_out <= ffc_5[1] ? JPEG_7 : (JPEG_7 << 16); // Explicit shift
            ffc_6 <= ffc_5;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_7 <= 64'b0; ffc_5 <= 2'b0;
        end else if (dr_in_6) begin
            JPEG_7[63:16] <= JPEG_6[63:16];
            JPEG_7[15:0] <= (JPEG_6[23:16] == 8'hFF) ? {8'h00, JPEG_6[15:8]} : JPEG_6[15:0]; // Simplified concat
            ffc_5 <= ffc_4;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_6 <= 64'b0; ffc_4 <= 2'b0;
        end else if (dr_in_5) begin
            JPEG_6[63:24] <= JPEG_5[63:24];
            JPEG_6[23:0] <= (JPEG_5[31:24] == 8'hFF) ? {8'h00, JPEG_5[23:16], JPEG_5[15:8]} : JPEG_5[23:0]; // Simplified concat
            ffc_4 <= ffc_3;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_5 <= 64'b0; ffc_3 <= 2'b0; // JPEG_5 declared as 63:0
        end else if (dr_in_4) begin
            JPEG_5[63:32] <= JPEG_4[55:24];
            if (JPEG_4[31:24] == 8'hFF) begin
                JPEG_5[31:24] <= 8'h00;
                JPEG_5[23:16] <= JPEG_4[23:16];
                JPEG_5[15:8] <= JPEG_4[15:8];
                JPEG_5[7:0] <= JPEG_4[7:0];
            end else begin
                JPEG_5[31:24] <= JPEG_4[23:16];
                JPEG_5[23:16] <= JPEG_4[15:8];
                JPEG_5[15:8] <= JPEG_4[7:0];
                JPEG_5[7:0] <= 8'h00; // Original: 8'b00000000 for this byte
            end
            ffc_3 <= ffc_2;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_4 <= 56'b0; ffc_2 <= 2'b0;
        end else if (dr_in_3) begin
            JPEG_4[55:32] <= JPEG_3[47:24];
            if (JPEG_3[31:24] == 8'hFF) begin
                JPEG_4[31:24] <= 8'h00;
                JPEG_4[23:16] <= JPEG_3[23:16];
                JPEG_4[15:8] <= JPEG_3[15:8];
                JPEG_4[7:0] <= JPEG_3[7:0];
            end else begin
                JPEG_4[31:24] <= JPEG_3[23:16];
                JPEG_4[23:16] <= JPEG_3[15:8];
                JPEG_4[15:8] <= JPEG_3[7:0];
                JPEG_4[7:0] <= 8'h00; // Original: 8'b00000000 for this byte
            end
            ffc_2 <= ffc_1;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_3 <= 48'b0; ct_1 <= 3'b0; FF_count <= 2'b0;
            ffc_1 <= 2'b0;
        end else if (dr_in_2) begin
            JPEG_3[47:32] <= JPEG_2[39:24];
            if (JPEG_2[31:24] == 8'hFF) begin
                JPEG_3[31:24] <= 8'h00;
                JPEG_3[23:16] <= JPEG_2[23:16];
                JPEG_3[15:8] <= JPEG_2[15:8];
                JPEG_3[7:0] <= JPEG_2[7:0];
            end else begin
                JPEG_3[31:24] <= JPEG_2[23:16];
                JPEG_3[23:16] <= JPEG_2[15:8];
                JPEG_3[15:8] <= JPEG_2[7:0];
                JPEG_3[7:0] <= 8'h00; // Original: 8'b00000000 for this byte
            end
            ct_1 <= count_total;
            FF_count <= FF_count + count_total;
            ffc_1 <= FF_count;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_2 <= 40'b0; count_total <= 3'b0;
        end else if (dr_in_1) begin
            JPEG_2[39:32] <= JPEG_1[31:24];
            if (first_2bytes) begin
                JPEG_2[31:24] <= 8'h00;
                JPEG_2[23:16] <= JPEG_1[23:16];
                JPEG_2[15:8] <= JPEG_1[15:8];
                JPEG_2[7:0] <= JPEG_1[7:0];
            end else begin
                JPEG_2[31:24] <= JPEG_1[23:16];
                JPEG_2[23:16] <= JPEG_1[15:8];
                JPEG_2[15:8] <= JPEG_1[7:0];
                JPEG_2[7:0] <= 8'h00; // Original: 8'b00000000 for this byte
            end
            count_total <= first_2bytes + second_2bytes + third_2bytes + fourth_2bytes;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            first_2bytes <= 1'b0; second_2bytes <= 1'b0;
            third_2bytes <= 1'b0; fourth_2bytes <= 1'b0;
            JPEG_1 <= 32'b0;
        end else if (data_ready_in) begin
            first_2bytes <= JPEG_in[31:24] == 8'hFF;
            second_2bytes <= JPEG_in[23:16] == 8'hFF;
            third_2bytes <= JPEG_in[15:8] == 8'hFF;
            fourth_2bytes <= JPEG_in[7:0] == 8'hFF;
            JPEG_1 <= JPEG_in;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            rollover_1 <= 1'b0; rollover_2 <= 1'b0; rollover_3 <= 1'b0;
            rollover_4 <= 1'b0; rollover_5 <= 1'b0;
        end else begin
            rollover_1 <= rollover;
            rollover_2 <= rollover_1;
            rollover_3 <= rollover_2;
            rollover_4 <= rollover_3;
            rollover_5 <= rollover_4;
        end
    end

    always_ff @(posedge clk) begin
        if (rst)
            rollover <= 1'b0;
        else if (!dr_in_3)
            rollover <= 1'b0;
        else if (dr_in_3)
            rollover <= (FF_count < ffc_1) || (ct_1 == 3'b100);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            dr_in_1 <= 1'b0; dr_in_2 <= 1'b0; dr_in_3 <= 1'b0; dr_in_4 <= 1'b0;
            dr_in_5 <= 1'b0; dr_in_6 <= 1'b0; dr_in_7 <= 1'b0; dr_in_8 <= 1'b0;
        end else begin
            dr_in_1 <= data_ready_in;
            dr_in_2 <= dr_in_1;
            dr_in_3 <= dr_in_2;
            dr_in_4 <= dr_in_3;
            dr_in_5 <= dr_in_4;
            dr_in_6 <= dr_in_5;
            dr_in_7 <= dr_in_6;
            dr_in_8 <= dr_in_7;
        end
    end

endmodule