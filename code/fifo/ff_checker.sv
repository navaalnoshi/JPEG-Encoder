/* -----------------------------------------------------------------------------
 * Module: ff_checker
 * Description:
 *   SystemVerilog implementation of a JPEG bitstream post-processor.
 *   This module scans incoming 32-bit data chunks for any byte equal to 0xFF.
 *   In compliance with the JPEG standard, if such a byte is found, the module
 *   appends a 0x00 byte immediately after to escape the FF marker.
 *
 *   This logic ensures the JPEG stream does not get corrupted or misinterpreted
 *   as marker segments, especially during decoding or storage.
 *
 * Functional Highlights:
 *   - Multi-stage pipelining (8 stages) is used to stagger the check-insert logic.
 *   - Registers maintain partial bits, byte counts, rollover detection, and ORC.
 *   - Integration with a downstream FIFO allows dynamic delay buffering.
 *   - Handles end-of-file cases with shortened valid lengths.
 *   - Ensures 32-bit aligned and FF-clean output JPEG stream.
 --------------------------------------------------------------------------------*/

`timescale 1ns / 100ps

module ff_checker(
    input logic clk,
    input logic rst,
    input logic end_of_file_signal,
    input logic [31:0] JPEG_in,
    input logic data_ready_in,
    input logic [4:0] orc_reg_in,
    output logic [31:0] JPEG_bitstream_1,
    output logic data_ready_1,
    output logic [4:0] orc_reg,
    output logic eof_data_partial_ready
);

    // Internal registers for byte checking and EOF handling
    logic first_2bytes, second_2bytes, third_2bytes, fourth_2bytes;
    logic first_2bytes_eof, second_2bytes_eof, third_2bytes_eof;
    logic fourth_2bytes_eof, fifth_2bytes_eof, s2b, t2b;

    // Registers for JPEG bitstream processing and EOF data
    logic [79:0] JPEG_eof_6, JPEG_eof_7;
    logic [63:0] JPEG_5, JPEG_eof_5_1, JPEG_6, JPEG_7;
    logic [55:0] JPEG_4, JPEG_eof_3, JPEG_eof_4, JPEG_eof_5;
    logic [47:0] JPEG_3, JPEG_eof_2;
    logic [39:0] JPEG_2, JPEG_eof_1;
    logic [31:0] JPEG_1, JPEG_ro, JPEG_bitstream, JPEG_bitstream_1;
    logic [31:0] JPEG_eof, JPEG_eof_ro;
    logic [31:0] JPEG_bitstream_eof;
    logic [15:0] JPEG_eof_ro_ro;
    logic [87:0] JPEG_out, JPEG_out_1, JPEG_pf;
    logic [23:0] JPEG_ro_ro;

    // Data ready pipeline registers
    logic dr_in_1, dr_in_2, dr_in_3, dr_in_4, dr_in_5, dr_in_6;
    logic dr_in_7, dr_in_8;

    // Rollover and FF count registers
    logic rollover, rollover_1, rollover_2, rollover_3, rollover_4, rollover_5;
    logic rollover_pf, rpf_1;
    logic [1:0] FF_count, FF_count_1, FF_eof_shift;
    logic [2:0] count_total, ct_1;
    logic [1:0] ffc_1, ffc_2, ffc_3, ffc_4, ffc_5, ffc_6, ffc_7;
    logic [1:0] ffc_postfifo, count_total_eof;

    // Input/output control registers
    logic [4:0] orc_input;
    logic [6:0] extra_bits_eof, extra_bits_eof_1;

    // FIFO signals
    wire [90:0] read_data;
    wire [90:0] write_data = { JPEG_out_1, ffc_7, rollover_5 }; // Concatenation for FIFO write [cite: 607]
    logic data_ready, write_enable, read_req, rdv_1;
    logic end_of_file_enable, eof_count_enable;
    logic eof_dpr_1, eof_dpr_2;
    logic end_of_file_enable_hold, eof_data_ready;
    logic eof_data_ready_1, eof_bits_1, eof_bits_2, eof_bits_3;
    logic [8:0] eof_count;
    wire fifo_empty, rdata_valid;

    //----------------------------------------------------------------------------
    // Synchronous FIFO instance for bitstream buffering [cite: 619]
    // A FIFO is used to store the JPEG bits when extra '00' bytes are inserted,
    // which causes a rollover of the 32-bit JPEG input. [cite: 620, 621, 622, 623]
    //----------------------------------------------------------------------------
    
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

    // Logic for eof_data_partial_ready output
    always_ff @(posedge clk) begin
        if (rst)
            eof_data_partial_ready <= 1'b0;
        else if (eof_bits_1)
            eof_data_partial_ready <= (extra_bits_eof_1 > 7'b0000000) && (extra_bits_eof_1 < 7'b0010000); // Checks if partial data is ready based on extra bits [cite: 628, 629]
        else
            eof_data_partial_ready <= eof_dpr_1;
    end

    // Pipeline for eof_dpr_1
    always_ff @(posedge clk) begin
        if (rst)
            eof_dpr_1 <= 1'b0;
        else if (eof_bits_1)
            eof_dpr_1 <= (extra_bits_eof_1 > 7'b0010000) && (extra_bits_eof_1 < 7'b0100000); // Checks for the next 32-bit chunk readiness [cite: 637, 638]
        else
            eof_dpr_1 <= eof_dpr_2;
    end

    // Pipeline for eof_dpr_2
    always_ff @(posedge clk) begin
        if (rst)
            eof_dpr_2 <= 1'b0;
        else if (eof_bits_1)
            eof_dpr_2 <= extra_bits_eof_1 > 7'b0100000; // Checks if the last chunk is ready [cite: 646, 647]
        else
            eof_dpr_2 <= 1'b0;
    end

    // Logic for eof_data_ready_1
    always_ff @(posedge clk) begin
        if (rst)
            eof_data_ready_1 <= 1'b0;
        else if (end_of_file_enable)
            eof_data_ready_1 <= (extra_bits_eof_1 > 7'b00011111); // Indicates if more than 31 extra bits exist for EOF [cite: 655, 656]
        else if (eof_bits_1 || eof_bits_2)
            eof_data_ready_1 <= eof_data_ready;
    end

    // Logic for eof_data_ready
    always_ff @(posedge clk) begin
        if (rst)
            eof_data_ready <= 1'b0;
        else if (end_of_file_enable)
            eof_data_ready <= (extra_bits_eof_1 > 7'b0111111); // Indicates if more than 63 extra bits exist for EOF [cite: 664, 665]
        else if (eof_bits_1)
            eof_data_ready <= 1'b0;
    end

    // Pipeline for eof_bits signals
    always_ff @(posedge clk) begin
        if (rst) begin
            eof_bits_1 <= 1'b0;
            eof_bits_2 <= 1'b0;
            eof_bits_3 <= 1'b0;
        end else begin
            eof_bits_1 <= end_of_file_enable;
            eof_bits_2 <= eof_bits_1;
            eof_bits_3 <= eof_bits_2;
        end
    end

    // Registers for end-of-file bitstream and rollover data
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_bitstream_eof <= 32'b0;
            JPEG_eof_ro <= 32'b0;
        end else if (end_of_file_enable) begin
            JPEG_bitstream_eof <= JPEG_eof_7[79:48]; // Takes the relevant part of the EOF data [cite: 685, 686]
            JPEG_eof_ro <= JPEG_eof_7[47:16]; // Stores the remaining EOF data for rollover [cite: 687]
        end else if (eof_bits_1 || eof_bits_2) begin
            JPEG_bitstream_eof <= JPEG_eof_ro; // Uses the rollover data as the next bitstream part [cite: 690]
            JPEG_eof_ro <= { JPEG_eof_ro_ro, 16'b0 }; // Prepares for the next rollover [cite: 691]
        end
    end

    // Register for JPEG_eof_ro_ro
    always_ff @(posedge clk) begin
        if (rst)
            JPEG_eof_ro_ro <= 16'b0;
        else if (end_of_file_enable)
            JPEG_eof_ro_ro <= JPEG_eof_7[15:0]; // Stores the last 16 bits of EOF data [cite: 698, 699]
    end

    // These registers combine previous leftover bits with end-of-file bits [cite: 702, 703]
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_eof_7 <= 80'b0;
            JPEG_eof_6 <= 80'b0;
            JPEG_eof_5_1 <= 64'b0;
            FF_eof_shift <= 2'b0;
            FF_count_1 <= 2'b0;
        end else begin
            // Adjusts JPEG_eof_7 based on FF_count_1 for proper byte alignment [cite: 709, 710, 711]
            JPEG_eof_7[79:72] <= (FF_count_1 > 2'b00) ? JPEG_ro[31:24] : JPEG_eof_6[79:72];
            JPEG_eof_7[71:64] <= (FF_count_1 > 2'b01) ? JPEG_ro[23:16] : JPEG_eof_6[71:64];
            JPEG_eof_7[63:56] <= (FF_count_1 > 2'b10) ? JPEG_ro[15:8] : JPEG_eof_6[63:56];
            JPEG_eof_7[55:0] <= JPEG_eof_6[55:0];
            JPEG_eof_6 <= JPEG_eof_5_1 << { FF_eof_shift[1], 4'b0000 }; // Shifts based on FF count for byte alignment [cite: 713]
            JPEG_eof_5_1 <= JPEG_eof_5 << { FF_eof_shift[0], 3'b000 }; // Shifts based on FF count for byte alignment [cite: 714]
            FF_eof_shift <= 2'b11 - FF_count; // Calculates shift amount [cite: 715]
            FF_count_1 <= FF_count; // Latch FF_count [cite: 716]
        end
    end

    // These registers generate the end-of-file bits and related control signals [cite: 720]
    always_ff @(posedge clk) begin
        if (rst) begin
            orc_reg <= 5'b0;
            extra_bits_eof <= 7'b0;
            extra_bits_eof_1 <= 7'b0;
            count_total_eof <= 2'b0;
            JPEG_eof_5 <= 56'b0;
            fifth_2bytes_eof <= 1'b0;
            JPEG_eof_4 <= 56'b0;
            fourth_2bytes_eof <= 1'b0;
            JPEG_eof_3 <= 56'b0;
            third_2bytes_eof <= 1'b0;
            JPEG_eof_2 <= 48'b0;
            second_2bytes_eof <= 1'b0;
            JPEG_eof_1 <= 40'b0;
            first_2bytes_eof <= 1'b0;
            s2b <= 1'b0;
            t2b <= 1'b0;
            orc_input <= 5'b0;
        end else begin
            orc_reg <= extra_bits_eof_1[4:0]; // Output remaining bit count [cite: 733]
            extra_bits_eof <= { 2'b00, orc_input } + { 2'b00, FF_count, 3'b000 }; // Calculates total extra bits due to FF insertion [cite: 734]
            extra_bits_eof_1 <= extra_bits_eof + { 2'b00, count_total_eof, 3'b000 }; // Accumulates total extra bits [cite: 735]
            count_total_eof <= first_2bytes_eof + s2b + t2b; // Counts 'FF' bytes at EOF [cite: 736]

            // Logic to insert '00' after 'FF' for EOF bytes, similar to regular processing but for remaining bits
            JPEG_eof_5[55:16] <= JPEG_eof_4[55:16];
            JPEG_eof_5[15:8] <= fifth_2bytes_eof ? 8'h00 : JPEG_eof_4[15:8];
            JPEG_eof_5[7:0] <= fifth_2bytes_eof ? JPEG_eof_4[15:8] : JPEG_eof_4[7:0];
            fifth_2bytes_eof <= (JPEG_eof_4[23:16] == 8'hFF); // Check for 'FF' in the 5th byte [cite: 740]

            JPEG_eof_4[55:24] <= JPEG_eof_3[55:24];
            JPEG_eof_4[23:16] <= fourth_2bytes_eof ? 8'h00 : JPEG_eof_3[23:16];
            JPEG_eof_4[15:8] <= fourth_2bytes_eof ? JPEG_eof_3[23:16] : JPEG_eof_3[15:8];
            JPEG_eof_4[7:0] <= fourth_2bytes_eof ? JPEG_eof_3[15:8] : JPEG_eof_3[7:0];
            fourth_2bytes_eof <= (JPEG_eof_3[31:24] == 8'hFF); // Check for 'FF' in the 4th byte [cite: 745]

            JPEG_eof_3[55:32] <= JPEG_eof_2[47:24];
            JPEG_eof_3[31:24] <= third_2bytes_eof ? 8'h00 : JPEG_eof_2[23:16];
            JPEG_eof_3[23:16] <= third_2bytes_eof ? JPEG_eof_2[23:16] : JPEG_eof_2[15:8];
            JPEG_eof_3[15:8] <= third_2bytes_eof ? JPEG_eof_2[15:8] : JPEG_eof_2[7:0];
            JPEG_eof_3[7:0] <= third_2bytes_eof ? JPEG_eof_2[7:0] : 8'h00;
            third_2bytes_eof <= (JPEG_eof_2[31:24] == 8'hFF); // Check for 'FF' in the 3rd byte [cite: 751]

            JPEG_eof_2[47:32] <= JPEG_eof_1[39:24];
            JPEG_eof_2[31:24] <= second_2bytes_eof ? 8'h00 : JPEG_eof_1[23:16];
            JPEG_eof_2[23:16] <= second_2bytes_eof ? JPEG_eof_1[23:16] : JPEG_eof_1[15:8];
            JPEG_eof_2[15:8] <= second_2bytes_eof ? JPEG_eof_1[15:8] : JPEG_eof_1[7:0];
            JPEG_eof_2[7:0] <= second_2bytes_eof ? JPEG_eof_1[7:0] : 8'h00;
            second_2bytes_eof <= (JPEG_eof_1[31:24] == 8'hFF); // Check for 'FF' in the 2nd byte [cite: 757]

            JPEG_eof_1[39:32] <= JPEG_eof[31:24];
            JPEG_eof_1[31:24] <= first_2bytes_eof ? 8'h00 : JPEG_eof[23:16];
            JPEG_eof_1[23:16] <= first_2bytes_eof ? JPEG_eof[23:16] : JPEG_eof[15:8];
            JPEG_eof_1[15:8] <= first_2bytes_eof ? JPEG_eof[15:8] : JPEG_eof[7:0];
            JPEG_eof_1[7:0] <= first_2bytes_eof ? JPEG_eof[7:0] : 8'h00;
            first_2bytes_eof <= (JPEG_eof[31:24] == 8'hFF); // Check for 'FF' in the 1st byte [cite: 763]

            s2b <= (JPEG_eof[23:16] == 8'hFF); // Check for 'FF' in the next byte [cite: 764]
            t2b <= (JPEG_eof[15:8] == 8'hFF); // Check for 'FF' in the next byte [cite: 765]
            orc_input <= orc_reg_in; // Latch input orc_reg [cite: 766]
        end
    end

     //----------------------------------------------------------------------------
    // Registers for JPEG_eof, handling remaining bits at EOF
     //----------------------------------------------------------------------------
    
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_eof <= 32'b0;
        end else begin
            // Clears bits of JPEG_eof based on orc_reg_in, effectively masking off unused bits [cite: 775, 776, 777, 778, 779, 780, 781, 782, 783, 784, 785, 786, 787, 788, 789, 790, 791, 792, 793, 794, 795, 796, 797, 798, 799, 800, 801, 802, 803, 804, 805]
            JPEG_eof[31] <= (orc_reg_in > 5'b00000) ? JPEG_in[31] : 1'b0;
            JPEG_eof[30] <= (orc_reg_in > 5'b00001) ? JPEG_in[30] : 1'b0;
            JPEG_eof[29] <= (orc_reg_in > 5'b00010) ? JPEG_in[29] : 1'b0;
            JPEG_eof[28] <= (orc_reg_in > 5'b00011) ? JPEG_in[28] : 1'b0;
            JPEG_eof[27] <= (orc_reg_in > 5'b00100) ? JPEG_in[27] : 1'b0;
            JPEG_eof[26] <= (orc_reg_in > 5'b00101) ? JPEG_in[26] : 1'b0;
            JPEG_eof[25] <= (orc_reg_in > 5'b00110) ? JPEG_in[25] : 1'b0;
            JPEG_eof[24] <= (orc_reg_in > 5'b00111) ? JPEG_in[24] : 1'b0;
            JPEG_eof[23] <= (orc_reg_in > 5'b01000) ? JPEG_in[23] : 1'b0;
            JPEG_eof[22] <= (orc_reg_in > 5'b01001) ? JPEG_in[22] : 1'b0;
            JPEG_eof[21] <= (orc_reg_in > 5'b01010) ? JPEG_in[21] : 1'b0;
            JPEG_eof[20] <= (orc_reg_in > 5'b01011) ? JPEG_in[20] : 1'b0;
            JPEG_eof[19] <= (orc_reg_in > 5'b01100) ? JPEG_in[19] : 1'b0;
            JPEG_eof[18] <= (orc_reg_in > 5'b01101) ? JPEG_in[18] : 1'b0;
            JPEG_eof[17] <= (orc_reg_in > 5'b01110) ? JPEG_in[17] : 1'b0;
            JPEG_eof[16] <= (orc_reg_in > 5'b01111) ? JPEG_in[16] : 1'b0;
            JPEG_eof[15] <= (orc_reg_in > 5'b10000) ? JPEG_in[15] : 1'b0;
            JPEG_eof[14] <= (orc_reg_in > 5'b10001) ? JPEG_in[14] : 1'b0;
            JPEG_eof[13] <= (orc_reg_in > 5'b10010) ? JPEG_in[13] : 1'b0;
            JPEG_eof[12] <= (orc_reg_in > 5'b10011) ? JPEG_in[12] : 1'b0;
            JPEG_eof[11] <= (orc_reg_in > 5'b10100) ? JPEG_in[11] : 1'b0;
            JPEG_eof[10] <= (orc_reg_in > 5'b10101) ? JPEG_in[10] : 1'b0;
            JPEG_eof[9] <= (orc_reg_in > 5'b10110) ? JPEG_in[9] : 1'b0;
            JPEG_eof[8] <= (orc_reg_in > 5'b10111) ? JPEG_in[8] : 1'b0;
            JPEG_eof[7] <= (orc_reg_in > 5'b11000) ? JPEG_in[7] : 1'b0;
            JPEG_eof[6] <= (orc_reg_in > 5'b11001) ? JPEG_in[6] : 1'b0;
            JPEG_eof[5] <= (orc_reg_in > 5'b11010) ? JPEG_in[5] : 1'b0;
            JPEG_eof[4] <= (orc_reg_in > 5'b11011) ? JPEG_in[4] : 1'b0;
            JPEG_eof[3] <= (orc_reg_in > 5'b11100) ? JPEG_in[3] : 1'b0;
            JPEG_eof[2] <= (orc_reg_in > 5'b11101) ? JPEG_in[2] : 1'b0;
            JPEG_eof[1] <= (orc_reg_in > 5'b11110) ? JPEG_in[1] : 1'b0;
            JPEG_eof[0] <= 1'b0; // Last bit is always 0 [cite: 806]
        end
    end

    // Controls the enable signal for eof_count
    always_ff @(posedge clk) begin
        if (rst)
            eof_count_enable <= 1'b0;
        else if (end_of_file_enable_hold)
            eof_count_enable <= 1'b0;
        else if (end_of_file_signal)
            eof_count_enable <= 1'b1;
    end

    // Increments eof_count when enabled
    always_ff @(posedge clk) begin
        if (rst)
            eof_count <= 9'b0;
        else if (!eof_count_enable)
            eof_count <= 9'b0;
        else if (eof_count_enable)
            eof_count <= eof_count + 1'b1;
    end

    // Sets end_of_file_enable when eof_count reaches a specific value
    always_ff @(posedge clk) begin
        if (rst)
            end_of_file_enable <= 1'b0;
        else if (eof_count != 9'b011110000)
            end_of_file_enable <= 1'b0;
        else if (eof_count == 9'b011110000)
            end_of_file_enable <= 1'b1;
    end

    // Holds the end_of_file_enable signal
    always_ff @(posedge clk) begin
        if (rst)
            end_of_file_enable_hold <= 1'b0;
        else if (end_of_file_enable)
            end_of_file_enable_hold <= 1'b1;
    end
    // This ends the section dealing with the end of file. [cite: 843]

    // Main output data ready and bitstream selection
    always_ff @(posedge clk) begin
        if (rst) begin
            data_ready_1 <= 1'b0;
            JPEG_bitstream_1 <= 32'b0;
        end else begin
            data_ready_1 <= data_ready || eof_data_ready_1; // Output data ready if either regular data or EOF data is ready [cite: 850]
            JPEG_bitstream_1 <= (eof_bits_1 || eof_bits_2 || eof_bits_3) ? // Selects between EOF bitstream and regular bitstream [cite: 851, 852]
                                JPEG_bitstream_eof : JPEG_bitstream;
        end
    end

    // Logic for data_ready and rollover flags from FIFO
    always_ff @(posedge clk) begin
        if (rst) begin
            data_ready <= 1'b0;
            rdv_1 <= 1'b0;
            rpf_1 <= 1'b0;
        end else begin
            data_ready <= rdv_1 || rpf_1; // Data is ready if FIFO data is valid or a rollover occurred [cite: 861]
            rdv_1 <= rdata_valid; // Latch rdata_valid [cite: 862]
            rpf_1 <= rollover_pf & !rpf_1; // Rollover flag, ensures only one rollover per sequence [cite: 863, 864]
        end
    end

    // Assigns the first byte of JPEG_bitstream based on FIFO data and rollover
    always_ff @(posedge clk) begin
        if (rst)
            JPEG_bitstream[31:24] <= 8'b0;
        else if (rdv_1 && ffc_postfifo == 2'b00 && !rpf_1)
            JPEG_bitstream[31:24] <= JPEG_pf[87:80]; // Regular data from FIFO [cite: 871]
        else if (rpf_1 || (rdv_1 && ffc_postfifo > 2'b00))
            JPEG_bitstream[31:24] <= JPEG_ro[31:24]; // Rollover data or adjusted data due to FF insertion [cite: 873]
    end

    // Assigns the second byte of JPEG_bitstream
    always_ff @(posedge clk) begin
        if (rst)
            JPEG_bitstream[23:16] <= 8'b0;
        else if (rdv_1 && ffc_postfifo < 2'b10 && !rpf_1)
            JPEG_bitstream[23:16] <= JPEG_pf[79:72];
        else if (rpf_1 || (rdv_1 && ffc_postfifo > 2'b01))
            JPEG_bitstream[23:16] <= JPEG_ro[23:16];
    end

    // Assigns the third byte of JPEG_bitstream
    always_ff @(posedge clk) begin
        if (rst)
            JPEG_bitstream[15:8] <= 8'b0;
        else if (rdv_1 && ffc_postfifo < 2'b11 && !rpf_1)
            JPEG_bitstream[15:8] <= JPEG_pf[71:64];
        else if (rpf_1 || (rdv_1 && ffc_postfifo == 2'b11))
            JPEG_bitstream[15:8] <= JPEG_ro[15:8];
    end

    // Assigns the fourth byte of JPEG_bitstream
    always_ff @(posedge clk) begin
        if (rst)
            JPEG_bitstream[7:0] <= 8'b0;
        else if (rdv_1 && !rpf_1)
            JPEG_bitstream[7:0] <= JPEG_pf[63:56];
        else if (rpf_1)
            JPEG_bitstream[7:0] <= JPEG_ro[7:0];
    end

    // Handles JPEG_ro (rollover register)
    always_ff @(posedge clk) begin
        if (rst)
            JPEG_ro <= 32'b0;
        else if (rdv_1 && !rpf_1)
            JPEG_ro <= JPEG_pf[55:24]; // Stores remaining data if not a rollover [cite: 907, 908]
        else if (rpf_1)
            JPEG_ro[31:8] <= JPEG_ro_ro; // Uses the rollover data [cite: 910]
    end

    // Handles JPEG_ro_ro (rollover register for JPEG_ro)
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_ro_ro <= 24'b0;
        end else if (rdv_1) begin
            JPEG_ro_ro <= JPEG_pf[23:0]; // Stores part of the data from the FIFO for potential future rollover [cite: 918]
        end
    end

    // Controls read_req for the FIFO
    always_ff @(posedge clk) begin
        if (fifo_empty)
            read_req <= 1'b0;
        else if (!fifo_empty)
            read_req <= 1'b1;
    end

    // Handles rollover_pf flag from FIFO read data
    always_ff @(posedge clk) begin
        if (rst)
            rollover_pf <= 1'b0;
        else if (!rdata_valid)
            rollover_pf <= 1'b0;
        else if (rdata_valid)
            rollover_pf <= read_data[0]; // Extracts rollover flag from FIFO data [cite: 934, 935]
    end

    // Updates JPEG_pf and ffc_postfifo from FIFO read data
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_pf <= 88'b0;
            ffc_postfifo <= 2'b0;
        end else if (rdata_valid) begin
            JPEG_pf <= read_data[90:3]; // Extracts JPEG data from FIFO [cite: 943]
            ffc_postfifo <= read_data[2:1]; // Extracts FF count from FIFO [cite: 944, 945, 946, 947]
        end
    end

    // Controls write_enable for the FIFO
    always_ff @(posedge clk) begin
        if (!dr_in_8)
            write_enable <= 1'b0;
        else if (dr_in_8)
            write_enable <= 1'b1; // Enables FIFO write when data is ready [cite: 954, 955, 956]
    end

    // Handles JPEG_out_1 and ffc_7, preparing data for FIFO write
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_out_1 <= 88'b0;
            ffc_7 <= 2'b0;
        end else if (dr_in_8) begin
            // Adjusts JPEG_out_1 based on ffc_6 for byte alignment [cite: 964]
            JPEG_out_1 <= ffc_6[0] ? JPEG_out : (JPEG_out << 8);
            ffc_7 <= ffc_6; // Latch ffc_6 [cite: 965]
        end
    end

    // Handles JPEG_out and ffc_6
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_out <= 88'b0;
            ffc_6 <= 2'b0;
        end else if (dr_in_7) begin
            // Adjusts JPEG_out based on ffc_5 for byte alignment [cite: 974]
            JPEG_out <= ffc_5[1] ? JPEG_7 : (JPEG_7 << 16);
            ffc_6 <= ffc_5; // Latch ffc_5 [cite: 975]
        end
    end

    // Handles JPEG_7 and ffc_5, inserts '00' after 'FF'
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_7 <= 64'b0;
            ffc_5 <= 2'b0;
        end else if (dr_in_6) begin
            JPEG_7[63:16] <= JPEG_6[63:16];
            JPEG_7[15:8] <= (JPEG_6[23:16] == 8'hFF) ? 8'h00 : JPEG_6[15:8]; // If 'FF' is found, insert '00' [cite: 985]
            JPEG_7[7:0] <= (JPEG_6[23:16] == 8'hFF) ? JPEG_6[15:8] : JPEG_6[7:0]; // Shift remaining bytes [cite: 986]
            ffc_5 <= ffc_4; // Latch ffc_4 [cite: 987]
        end
    end

    // Handles JPEG_6 and ffc_4, inserts '00' after 'FF'
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_6 <= 64'b0;
            ffc_4 <= 2'b0;
        end else if (dr_in_5) begin
            JPEG_6[63:24] <= JPEG_5[63:24];
            JPEG_6[23:16] <= (JPEG_5[31:24] == 8'hFF) ? 8'h00 : JPEG_5[23:16]; // If 'FF' is found, insert '00' [cite: 997]
            JPEG_6[15:8] <= (JPEG_5[31:24] == 8'hFF) ? JPEG_5[23:16] : JPEG_5[15:8]; // Shift remaining bytes [cite: 998]
            JPEG_6[7:0] <= (JPEG_5[31:24] == 8'hFF) ? JPEG_5[15:8] : JPEG_5[7:0]; // Shift remaining bytes [cite: 999]
            ffc_4 <= ffc_3; // Latch ffc_3 [cite: 1000]
        end
    end

    // Handles JPEG_5 and ffc_3, inserts '00' after 'FF'
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_5 <= 64'b0;
            ffc_3 <= 2'b0;
        end else if (dr_in_4) begin
            JPEG_5[63:32] <= JPEG_4[55:24];
            JPEG_5[31:24] <= (JPEG_4[31:24] == 8'hFF) ? 8'h00 : JPEG_4[23:16]; // If 'FF' is found, insert '00' [cite: 1010]
            JPEG_5[23:16] <= (JPEG_4[31:24] == 8'hFF) ? JPEG_4[23:16] : JPEG_4[15:8]; // Shift remaining bytes [cite: 1011]
            JPEG_5[15:8] <= (JPEG_4[31:24] == 8'hFF) ? JPEG_4[15:8] : JPEG_4[7:0]; // Shift remaining bytes [cite: 1012]
            JPEG_5[7:0] <= (JPEG_4[31:24] == 8'hFF) ? JPEG_4[7:0] : 8'h00; // Shift remaining bytes [cite: 1013]
            ffc_3 <= ffc_2; // Latch ffc_2 [cite: 1014]
        end
    end

    // Handles JPEG_4 and ffc_2, inserts '00' after 'FF'
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_4 <= 56'b0;
            ffc_2 <= 2'b0;
        end else if (dr_in_3) begin
            JPEG_4[55:32] <= JPEG_3[47:24];
            JPEG_4[31:24] <= (JPEG_3[31:24] == 8'hFF) ? 8'h00 : JPEG_3[23:16]; // If 'FF' is found, insert '00' [cite: 1024]
            JPEG_4[23:16] <= (JPEG_3[31:24] == 8'hFF) ? JPEG_3[23:16] : JPEG_3[15:8]; // Shift remaining bytes [cite: 1025]
            JPEG_4[15:8] <= (JPEG_3[31:24] == 8'hFF) ? JPEG_3[15:8] : JPEG_3[7:0]; // Shift remaining bytes [cite: 1026]
            JPEG_4[7:0] <= (JPEG_3[31:24] == 8'hFF) ? JPEG_3[7:0] : 8'h00; // Shift remaining bytes [cite: 1027]
            ffc_2 <= ffc_1; // Latch ffc_1 [cite: 1028]
        end
    end

    // Handles JPEG_3, ct_1, FF_count, and ffc_1, inserts '00' after 'FF'
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_3 <= 48'b0;
            ct_1 <= 3'b0;
            FF_count <= 2'b0;
            ffc_1 <= 2'b0;
        end else if (dr_in_2) begin
            JPEG_3[47:32] <= JPEG_2[39:24];
            JPEG_3[31:24] <= (JPEG_2[31:24] == 8'hFF) ? 8'h00 : JPEG_2[23:16]; // If 'FF' is found, insert '00' [cite: 1039]
            JPEG_3[23:16] <= (JPEG_2[31:24] == 8'hFF) ? JPEG_2[23:16] : JPEG_2[15:8]; // Shift remaining bytes [cite: 1040]
            JPEG_3[15:8] <= (JPEG_2[31:24] == 8'hFF) ? JPEG_2[15:8] : JPEG_2[7:0]; // Shift remaining bytes [cite: 1041]
            JPEG_3[7:0] <= (JPEG_2[31:24] == 8'hFF) ? JPEG_2[7:0] : 8'h00; // Shift remaining bytes [cite: 1042]
            ct_1 <= count_total; // Latch count_total [cite: 1043]
            FF_count <= FF_count + count_total; // Accumulate FF count [cite: 1044]
            ffc_1 <= FF_count; // Latch FF_count [cite: 1045]
        end
    end

    // Handles JPEG_2 and count_total, inserts '00' after 'FF'
    always_ff @(posedge clk) begin
        if (rst) begin
            JPEG_2 <= 40'b0;
            count_total <= 3'b0;
        end else if (dr_in_1) begin
            JPEG_2[39:32] <= JPEG_1[31:24];
            JPEG_2[31:24] <= first_2bytes ? 8'h00 : JPEG_1[23:16]; // If 'FF' is found, insert '00' [cite: 1055]
            JPEG_2[23:16] <= first_2bytes ? JPEG_1[23:16] : JPEG_1[15:8]; // Shift remaining bytes [cite: 1056]
            JPEG_2[15:8] <= first_2bytes ? JPEG_1[15:8] : JPEG_1[7:0]; // Shift remaining bytes [cite: 1057]
            JPEG_2[7:0] <= first_2bytes ? JPEG_1[7:0] : 8'h00; // Shift remaining bytes [cite: 1058]
            // Counts the total number of 'FF' bytes in the current 32-bit input [cite: 1059]
            count_total <= first_2bytes + second_2bytes + third_2bytes + fourth_2bytes;
        end
    end

    // Detects 'FF' bytes in the JPEG_in and latches JPEG_in
    always_ff @(posedge clk) begin
        if (rst) begin
            first_2bytes <= 1'b0;
            second_2bytes <= 1'b0;
            third_2bytes <= 1'b0;
            fourth_2bytes <= 1'b0;
            JPEG_1 <= 32'b0;
        end else if (data_ready_in) begin
            first_2bytes <= (JPEG_in[31:24] == 8'hFF); // Checks if the first byte is 'FF' [cite: 1070]
            second_2bytes <= (JPEG_in[23:16] == 8'hFF); // Checks if the second byte is 'FF' [cite: 1071]
            third_2bytes <= (JPEG_in[15:8] == 8'hFF); // Checks if the third byte is 'FF' [cite: 1072]
            fourth_2bytes <= (JPEG_in[7:0] == 8'hFF); // Checks if the fourth byte is 'FF' [cite: 1073]
            JPEG_1 <= JPEG_in; // Latch input JPEG data [cite: 1074]
        end
    end

    // Pipeline for rollover signals
    always_ff @(posedge clk) begin
        if (rst) begin
            rollover_1 <= 1'b0;
            rollover_2 <= 1'b0;
            rollover_3 <= 1'b0;
            rollover_4 <= 1'b0;
            rollover_5 <= 1'b0;
        end else begin
            rollover_1 <= rollover;
            rollover_2 <= rollover_1;
            rollover_3 <= rollover_2;
            rollover_4 <= rollover_3;
            rollover_5 <= rollover_4;
        end
    end

    // Determines if a rollover condition has occurred
    always_ff @(posedge clk) begin
        if (rst)
            rollover <= 1'b0;
        else if (!dr_in_3)
            rollover <= 1'b0;
        else if (dr_in_3)
            // A rollover occurs if FF_count decreases (due to 4 FF's wrapping around)
            // or if all 32 bits are FF's (ct_1 == 4 FF's, which is 3'b100) [cite: 1097, 1098, 1099, 1100, 1101, 1102, 1103, 1104, 1105]
            rollover <= (FF_count < ffc_1) || (ct_1 == 3'b100);
    end
    
     //----------------------------------------------------------------------------
    // Pipeline for data_ready_in signal
     //----------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            dr_in_1 <= 1'b0;
            dr_in_2 <= 1'b0;
            dr_in_3 <= 1'b0;
            dr_in_4 <= 1'b0;
            dr_in_5 <= 1'b0;
            dr_in_6 <= 1'b0;
            dr_in_7 <= 1'b0;
            dr_in_8 <= 1'b0;
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
