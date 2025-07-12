/*
This module takes the JPEG_bitstream as its input, and checks for any FF values in
the bitstream. When it finds an FF in the bitstream, this module puts a 00 after the FF,
and then continues with the rest of the bitstream after the 00, per the JPEG standard.
*/

`timescale 1ns / 100ps

module ff_checker #(
    parameter int DATA_WIDTH = 32, // Width of the JPEG bitstream (bytes * 8)
    parameter int ORC_WIDTH = 5    // Width of orc_reg_in
) (
    input  logic                   clk,
    input  logic                   rst,
    input  logic                   end_of_file_signal,
    input  logic [DATA_WIDTH-1:0]  JPEG_in,
    input  logic                   data_ready_in,
    input  logic [ORC_WIDTH-1:0]   orc_reg_in,
    output logic [DATA_WIDTH-1:0]  JPEG_bitstream_1,
    output logic                   data_ready_1,
    output logic [ORC_WIDTH-1:0]   orc_reg,
    output logic                   eof_data_partial_ready
);

    // --- Pipeline Registers and FF Counts ---
    localparam int PIPELINE_STAGES = 8;
    localparam int BYTE_WIDTH = 8;

    logic [DATA_WIDTH-1:0] dr_in_q [0:PIPELINE_STAGES-1]; // data_ready_in pipeline
    logic [1:0] ffc_q [0:6]; // ffc_1 to ffc_7, length 7
    logic [2:0] ct_q; // ct_1 is essentially a delayed count_total
    logic [DATA_WIDTH-1:0] jpeg_pipeline_in_q; // JPEG_1 is data_ready_in fed directly
    logic [39:0] JPEG_2; // Specific width, cannot be easily arrayed without complex width calc
    logic [47:0] JPEG_3;
    logic [55:0] JPEG_4, JPEG_eof_3, JPEG_eof_4, JPEG_eof_5;
    logic [63:0] JPEG_5, JPEG_eof_5_1, JPEG_6, JPEG_7;
    logic [79:0] JPEG_eof_6, JPEG_eof_7;
    logic [87:0] JPEG_out, JPEG_out_1;

    logic   first_2bytes, second_2bytes, third_2bytes, fourth_2bytes; // input FF detection
    logic [2:0] count_total; // sum of initial FF detections

    logic [1:0] FF_count; // total FFs detected so far in main pipeline
    logic [1:0] FF_count_1; // delayed FF_count for EOF
    logic [1:0] FF_eof_shift; // shift amount for EOF data

    // --- Rollover Logic ---
    logic   rollover;
    logic   rollover_q [0:4]; // rollover_1 to rollover_5

    // --- FIFO Interface ---
    logic [87:0] JPEG_pf;
    logic [1:0]  ffc_postfifo;
    logic        rollover_pf;
    logic        read_req, write_enable, rdv_1;
    wire [90:0]  read_data;
    wire [90:0]  write_data = { JPEG_out_1, ffc_q[6], rollover_q[4] }; // ffc_7 is ffc_q[6], rollover_5 is rollover_q[4]
    wire         fifo_empty, rdata_valid;

    // --- FIFO Instantiation (assuming sync_fifo_ff is defined elsewhere) ---
    sync_fifo_ff u18 (
        .clk(clk),
        .rst(rst),
        .read_req(read_req),
        .write_data(write_data),
        .write_enable(write_enable),
        .rollover_write(rollover_q[4]), // rollover_5
        .read_data(read_data),
        .fifo_empty(fifo_empty),
        .rdata_valid(rdata_valid)
    );

    // --- Output and Rollover from FIFO ---
    logic [DATA_WIDTH-1:0] JPEG_bitstream;
    logic [DATA_WIDTH-1:0] JPEG_ro;
    logic [23:0] JPEG_ro_ro;
    logic        data_ready; // internal data_ready before EOF mux
    logic        rpf_1; // lagged rollover_pf

    // --- EOF Registers and Logic ---
    logic   end_of_file_enable_hold;
    logic   eof_count_enable;
    logic [8:0] eof_count; // 9'b011110000 = 240
    logic   end_of_file_enable;

    logic [DATA_WIDTH-1:0] JPEG_eof; // Initial EOF data, masked
    logic [39:0] JPEG_eof_1;
    logic [47:0] JPEG_eof_2;
    logic [7:0] first_2bytes_eof, second_2bytes_eof, third_2bytes_eof,
                fourth_2bytes_eof, fifth_2bytes_eof; // Byte FF detection for EOF
    logic s2b, t2b; // Shortened names for byte 2 and 3 FF detection
    logic [4:0] orc_input;
    logic [6:0] extra_bits_eof, extra_bits_eof_1;
    logic [1:0] count_total_eof;

    logic [DATA_WIDTH-1:0] JPEG_bitstream_eof;
    logic [DATA_WIDTH-1:0] JPEG_eof_ro;
    logic [15:0] JPEG_eof_ro_ro;

    logic eof_data_ready, eof_data_ready_1;
    logic eof_dpr_1, eof_dpr_2;
    logic eof_bits_1, eof_bits_2, eof_bits_3;


    // --- Pipelined Data Ready (dr_in_X) ---
    always_ff @(posedge clk) begin : p_dr_in
        if (rst) begin
            for (int i = 0; i < PIPELINE_STAGES; i++) dr_in_q[i] <= '0;
        end else begin
            dr_in_q[0] <= data_ready_in;
            for (int i = 1; i < PIPELINE_STAGES; i++) dr_in_q[i] <= dr_in_q[i-1];
        end
    end

    // --- Initial FF Detection and JPEG_1 ---
    always_ff @(posedge clk) begin : p_initial_ff_detect
        if (rst) begin
            first_2bytes <= '0; second_2bytes <= '0;
            third_2bytes <= '0; fourth_2bytes <= '0;
            jpeg_pipeline_in_q <= '0; // This is JPEG_1
            count_total <= '0;
        end else if (data_ready_in) begin
            first_2bytes <= JPEG_in[31:24] == 8'hFF;
            second_2bytes <= JPEG_in[23:16] == 8'hFF;
            third_2bytes <= JPEG_in[15:8] == 8'hFF;
            fourth_2bytes <= JPEG_in[7:0] == 8'hFF;
            jpeg_pipeline_in_q <= JPEG_in; // This is JPEG_1
            count_total <= first_2bytes + second_2bytes + third_2bytes + fourth_2bytes;
        end
    end

    // --- Main FF Insertion Pipeline (JPEG_2 to JPEG_7, JPEG_out, JPEG_out_1) ---
    // This part requires careful replication of the original's complex shifting.
    // We cannot simply use a generic shift as the original's shifts are conditional
    // on *which* byte was FF.

    // JPEG_2, ct_1, FF_count, ffc_1
    always_ff @(posedge clk) begin : p_jpeg2_ffc1
        if (rst) begin
            JPEG_2 <= '0; ct_q <= '0; FF_count <= '0; ffc_q[0] <= '0;
        end else if (dr_in_q[1]) begin // dr_in_2
            JPEG_2[39:32] <= jpeg_pipeline_in_q[31:24]; // JPEG_1[31:24]
            JPEG_2[31:24] <= first_2bytes ? 8'h00 : jpeg_pipeline_in_q[23:16];
            JPEG_2[23:16] <= first_2bytes ? jpeg_pipeline_in_q[23:16] : jpeg_pipeline_in_q[15:8];
            JPEG_2[15:8] <= first_2bytes ? jpeg_pipeline_in_q[15:8] : jpeg_pipeline_in_q[7:0];
            JPEG_2[7:0] <= first_2bytes ? jpeg_pipeline_in_q[7:0] : 8'h00;
            ct_q <= count_total; // ct_1
            FF_count <= FF_count + count_total;
            ffc_q[0] <= FF_count; // ffc_1
        end
    end

    // JPEG_3, ffc_2
    always_ff @(posedge clk) begin : p_jpeg3_ffc2
        if (rst) begin
            JPEG_3 <= '0; ffc_q[1] <= '0;
        end else if (dr_in_q[2]) begin // dr_in_3
            JPEG_3[47:32] <= JPEG_2[39:24];
            JPEG_3[31:24] <= (JPEG_2[31:24] == 8'hFF) ? 8'h00 : JPEG_2[23:16];
            JPEG_3[23:16] <= (JPEG_2[31:24] == 8'hFF) ? JPEG_2[23:16] : JPEG_2[15:8];
            JPEG_3[15:8] <= (JPEG_2[31:24] == 8'hFF) ? JPEG_2[15:8] : JPEG_2[7:0];
            JPEG_3[7:0] <= (JPEG_2[31:24] == 8'hFF) ? JPEG_2[7:0] : 8'h00;
            ffc_q[1] <= ffc_q[0]; // ffc_2 <= ffc_1
        end
    end

    // JPEG_4, ffc_3
    always_ff @(posedge clk) begin : p_jpeg4_ffc3
        if (rst) begin
            JPEG_4 <= '0; ffc_q[2] <= '0;
        end else if (dr_in_q[3]) begin // dr_in_4
            JPEG_4[55:32] <= JPEG_3[47:24];
            JPEG_4[31:24] <= (JPEG_3[31:24] == 8'hFF) ? 8'h00 : JPEG_3[23:16];
            JPEG_4[23:16] <= (JPEG_3[31:24] == 8'hFF) ? JPEG_3[23:16] : JPEG_3[15:8];
            JPEG_4[15:8] <= (JPEG_3[31:24] == 8'hFF) ? JPEG_3[15:8] : JPEG_3[7:0];
            JPEG_4[7:0] <= (JPEG_3[31:24] == 8'hFF) ? JPEG_3[7:0] : 8'h00;
            ffc_q[2] <= ffc_q[1]; // ffc_3 <= ffc_2
        end
    end

    // JPEG_5, ffc_4
    always_ff @(posedge clk) begin : p_jpeg5_ffc4
        if (rst) begin
            JPEG_5 <= '0; ffc_q[3] <= '0;
        end else if (dr_in_q[4]) begin // dr_in_5
            JPEG_5[63:32] <= JPEG_4[55:24];
            JPEG_5[31:24] <= (JPEG_4[31:24] == 8'hFF) ? 8'h00 : JPEG_4[23:16];
            JPEG_5[23:16] <= (JPEG_4[31:24] == 8'hFF) ? JPEG_4[23:16] : JPEG_4[15:8];
            JPEG_5[15:8] <= (JPEG_4[31:24] == 8'hFF) ? JPEG_4[15:8] : JPEG_4[7:0];
            JPEG_5[7:0] <= (JPEG_4[31:24] == 8'hFF) ? JPEG_4[7:0] : 8'h00;
            ffc_q[3] <= ffc_q[2]; // ffc_4 <= ffc_3
        end
    end

    // JPEG_6, ffc_5
    always_ff @(posedge clk) begin : p_jpeg6_ffc5
        if (rst) begin
            JPEG_6 <= '0; ffc_q[4] <= '0;
        end else if (dr_in_q[5]) begin // dr_in_6
            JPEG_6[63:24] <= JPEG_5[63:24];
            JPEG_6[23:16] <= (JPEG_5[31:24] == 8'hFF) ? 8'h00 : JPEG_5[23:16];
            JPEG_6[15:8] <= (JPEG_5[31:24] == 8'hFF) ? JPEG_5[23:16] : JPEG_5[15:8];
            JPEG_6[7:0] <= (JPEG_5[31:24] == 8'hFF) ? JPEG_5[15:8] : JPEG_5[7:0];
            ffc_q[4] <= ffc_q[3]; // ffc_5 <= ffc_4
        end
    end

    // JPEG_7, ffc_6
    always_ff @(posedge clk) begin : p_jpeg7_ffc6
        if (rst) begin
            JPEG_7 <= '0; ffc_q[5] <= '0;
        end else if (dr_in_q[6]) begin // dr_in_7
            JPEG_7[63:16] <= JPEG_6[63:16];
            JPEG_7[15:8] <= (JPEG_6[23:16] == 8'hFF) ? 8'h00 : JPEG_6[15:8];
            JPEG_7[7:0] <= (JPEG_6[23:16] == 8'hFF) ? JPEG_6[15:8] : JPEG_6[7:0];
            ffc_q[5] <= ffc_q[4]; // ffc_6 <= ffc_5
        end
    end

    // JPEG_out, ffc_7 (ffc_q[6])
    always_ff @(posedge clk) begin : p_jpeg_out_ffc7
        if (rst) begin
            JPEG_out <= '0; ffc_q[6] <= '0;
        end else if (dr_in_q[7]) begin // dr_in_8, note this is the last stage before FIFO
            // The original logic for JPEG_out:
            // JPEG_out <= ffc_5[1] ? JPEG_7 : JPEG_7 << 16;
            // This suggests a shift of 16 bits if ffc_5[1] is false.
            // ffc_5[1] corresponds to ffc_q[4][1]
            JPEG_out <= ffc_q[4][1] ? JPEG_7 : (JPEG_7 << 16);
            ffc_q[6] <= ffc_q[5]; // ffc_7 <= ffc_6
        end
    end

    // JPEG_out_1
    always_ff @(posedge clk) begin : p_jpeg_out_1
        if (rst) begin
            JPEG_out_1 <= '0;
        end else if (dr_in_q[7]) begin // dr_in_8
            // Original: JPEG_out_1 <= ffc_6[0] ? JPEG_out : JPEG_out << 8;
            // ffc_6[0] corresponds to ffc_q[5][0]
            JPEG_out_1 <= ffc_q[5][0] ? JPEG_out : (JPEG_out << 8);
        end
    end

    // --- write_enable to FIFO ---
    always_ff @(posedge clk) begin : p_write_enable
        if (rst) write_enable <= '0;
        else write_enable <= dr_in_q[7]; // write_enable is dr_in_8
    end

    // --- Rollover Pipeline ---
    always_ff @(posedge clk) begin : p_rollover_pipeline
        if (rst) begin
            rollover <= '0;
            for (int i = 0; i < 5; i++) rollover_q[i] <= '0;
        end else begin
            // Original rollover logic
            // rollover <= (FF_count < ffc_1) | (ct_1 == 3'b100); when dr_in_3
            if (!dr_in_q[2]) // !dr_in_3
                rollover <= '0;
            else if (dr_in_q[2]) // dr_in_3
                rollover <= (FF_count < ffc_q[0]) | (ct_q == 3'b100);

            // Pipeline stages
            rollover_q[0] <= rollover; // rollover_1
            for (int i = 1; i < 5; i++) rollover_q[i] <= rollover_q[i-1];
        end
    end

    // --- FIFO Read and Output Control ---
    always_ff @(posedge clk) begin : p_fifo_read_control
        if (rst) begin
            read_req <= '0;
            rdv_1 <= '0;
            rpf_1 <= '0;
            data_ready <= '0;
            JPEG_pf <= '0; ffc_postfifo <= '0; rollover_pf <= '0;
        end else begin
            // read_req
            if (fifo_empty) read_req <= '0;
            else read_req <= 1'b1;

            // rdv_1, rpf_1, data_ready
            data_ready <= rdv_1 || rpf_1;
            rdv_1 <= rdata_valid;
            rpf_1 <= rollover_pf & !rpf_1;

            // rollover_pf
            if (!rdata_valid) rollover_pf <= '0;
            else rollover_pf <= read_data[0];

            // JPEG_pf, ffc_postfifo
            if (rdata_valid) begin
                JPEG_pf <= read_data[90:3];
                ffc_postfifo <= read_data[2:1];
            end else begin
                JPEG_pf <= '0; // Retain current value if not valid read
                ffc_postfifo <= '0;
            end
        end
    end

    // --- Main Bitstream Output (JPEG_bitstream) ---
    // Combine the 4 always_ff blocks for JPEG_bitstream[31:0]
    always_ff @(posedge clk) begin : p_jpeg_bitstream
        if (rst) begin
            JPEG_bitstream <= '0;
            JPEG_ro <= '0;
            JPEG_ro_ro <= '0;
        end else begin
            // JPEG_bitstream[31:24]
            if (rdv_1 && ffc_postfifo == '0 && !rpf_1)
                JPEG_bitstream[31:24] <= JPEG_pf[87:80];
            else if (rpf_1 || (rdv_1 && ffc_postfifo > '0))
                JPEG_bitstream[31:24] <= JPEG_ro[31:24];
            else
                JPEG_bitstream[31:24] <= JPEG_bitstream[31:24]; // Latch previous value if no condition met

            // JPEG_bitstream[23:16]
            if (rdv_1 && ffc_postfifo < 2 && !rpf_1)
                JPEG_bitstream[23:16] <= JPEG_pf[79:72];
            else if (rpf_1 || (rdv_1 && ffc_postfifo > 1))
                JPEG_bitstream[23:16] <= JPEG_ro[23:16];
            else
                JPEG_bitstream[23:16] <= JPEG_bitstream[23:16];

            // JPEG_bitstream[15:8]
            if (rdv_1 && ffc_postfifo < 3 && !rpf_1)
                JPEG_bitstream[15:8] <= JPEG_pf[71:64];
            else if (rpf_1 || (rdv_1 && ffc_postfifo == 3))
                JPEG_bitstream[15:8] <= JPEG_ro[15:8];
            else
                JPEG_bitstream[15:8] <= JPEG_bitstream[15:8];

            // JPEG_bitstream[7:0]
            if (rdv_1 && !rpf_1)
                JPEG_bitstream[7:0] <= JPEG_pf[63:56];
            else if (rpf_1)
                JPEG_bitstream[7:0] <= JPEG_ro[7:0];
            else
                JPEG_bitstream[7:0] <= JPEG_bitstream[7:0];

            // JPEG_ro
            if (rdv_1 && !rpf_1)
                JPEG_ro <= JPEG_pf[55:24];
            else if (rpf_1)
                JPEG_ro[31:8] <= JPEG_ro_ro;
            else
                JPEG_ro <= JPEG_ro; // Latch previous value

            // JPEG_ro_ro
            if (rdv_1)
                JPEG_ro_ro <= JPEG_pf[23:0];
            else
                JPEG_ro_ro <= JPEG_ro_ro;
        end
    end


    // --- EOF Logic ---

    // End of file counting and enabling
    always_ff @(posedge clk) begin : p_eof_control
        if (rst) begin
            eof_count_enable <= '0;
            eof_count <= '0;
            end_of_file_enable <= '0;
            end_of_file_enable_hold <= '0;
        end else begin
            // eof_count_enable
            if (end_of_file_enable_hold) eof_count_enable <= '0;
            else if (end_of_file_signal) eof_count_enable <= 1'b1;
            else eof_count_enable <= eof_count_enable; // Latch if no condition met

            // eof_count
            if (!eof_count_enable) eof_count <= '0;
            else if (eof_count_enable) eof_count <= eof_count + 1;
            else eof_count <= eof_count;

            // end_of_file_enable
            if (eof_count != 9'b011110000) end_of_file_enable <= '0;
            else if (eof_count == 9'b011110000) end_of_file_enable <= 1'b1;
            else end_of_file_enable <= end_of_file_enable;

            // end_of_file_enable_hold
            if (end_of_file_enable) end_of_file_enable_hold <= 1'b1;
            else end_of_file_enable_hold <= end_of_file_enable_hold;
        end
    end

    // JPEG_eof - mask input based on orc_reg_in
    // Using a for loop and bit select with replication for conciseness
    always_ff @(posedge clk) begin : p_jpeg_eof_mask
        if (rst) begin
            JPEG_eof <= '0;
            orc_input <= '0;
        end else begin
            orc_input <= orc_reg_in; // orc_input assignment moved here for conciseness
            for (int i = 0; i < DATA_WIDTH; i++) begin
                JPEG_eof[i] <= (orc_reg_in > i) ? JPEG_in[i] : 1'b0;
            end
             JPEG_eof[0] <= 1'b0; // As per original, bit 0 is always 0
        end
    end

    // EOF FF detection and pipeline stages (JPEG_eof_1 to JPEG_eof_7)
    // Similar to main pipeline, but with 5 bytes detected and different bit manipulation
    always_ff @(posedge clk) begin : p_eof_ff_insertion
        if (rst) begin
            first_2bytes_eof <= '0; second_2bytes_eof <= '0; third_2bytes_eof <= '0;
            fourth_2bytes_eof <= '0; fifth_2bytes_eof <= '0; s2b <= '0; t2b <= '0;
            count_total_eof <= '0;
            extra_bits_eof <= '0; extra_bits_eof_1 <= '0;
            orc_reg <= '0;
            JPEG_eof_1 <= '0; JPEG_eof_2 <= '0; JPEG_eof_3 <= '0; JPEG_eof_4 <= '0;
            JPEG_eof_5 <= '0; JPEG_eof_5_1 <= '0; JPEG_eof_6 <= '0; JPEG_eof_7 <= '0;
            FF_eof_shift <= '0; FF_count_1 <= '0;
            JPEG_eof_ro <= '0; JPEG_eof_ro_ro <= '0; JPEG_bitstream_eof <= '0;
        end else begin
            // FF detections for EOF (combinational within this sequential block)
            first_2bytes_eof <= (JPEG_eof[31:24] == 8'hFF);
            s2b <= (JPEG_eof[23:16] == 8'hFF); // s2b is original's second_2bytes_eof for count_total_eof
            t2b <= (JPEG_eof[15:8] == 8'hFF); // t2b is original's third_2bytes_eof for count_total_eof
            second_2bytes_eof <= (JPEG_eof_1[31:24] == 8'hFF); // For next stage of JPEG_eof_2
            third_2bytes_eof <= (JPEG_eof_2[31:24] == 8'hFF); // For next stage of JPEG_eof_3
            fourth_2bytes_eof <= (JPEG_eof_3[31:24] == 8'hFF); // For next stage of JPEG_eof_4
            fifth_2bytes_eof <= (JPEG_eof_4[23:16] == 8'hFF); // For next stage of JPEG_eof_5

            // EOF FF counts and extra bits
            count_total_eof <= first_2bytes_eof + s2b + t2b;
            extra_bits_eof <= { 2'b00, orc_input } + { 2'b00, FF_count, 3'b000 };
            extra_bits_eof_1 <= extra_bits_eof + { 2'b00, count_total_eof, 3'b000 };
            orc_reg <= extra_bits_eof_1[4:0];

            // JPEG_eof_1
            JPEG_eof_1[39:32] <= JPEG_eof[31:24];
            JPEG_eof_1[31:24] <= first_2bytes_eof ? 8'h00 : JPEG_eof[23:16];
            JPEG_eof_1[23:16] <= first_2bytes_eof ? JPEG_eof[23:16] : JPEG_eof[15:8];
            JPEG_eof_1[15:8] <= first_2bytes_eof ? JPEG_eof[15:8] : JPEG_eof[7:0];
            JPEG_eof_1[7:0] <= first_2bytes_eof ? JPEG_eof[7:0] : 8'h00;

            // JPEG_eof_2
            JPEG_eof_2[47:32] <= JPEG_eof_1[39:24];
            JPEG_eof_2[31:24] <= second_2bytes_eof ? 8'h00 : JPEG_eof_1[23:16];
            JPEG_eof_2[23:16] <= second_2bytes_eof ? JPEG_eof_1[23:16] : JPEG_eof_1[15:8];
            JPEG_eof_2[15:8] <= second_2bytes_eof ? JPEG_eof_1[15:8] : JPEG_eof_1[7:0];
            JPEG_eof_2[7:0] <= second_2bytes_eof ? JPEG_eof_1[7:0] : 8'h00;

            // JPEG_eof_3
            JPEG_eof_3[55:32] <= JPEG_eof_2[47:24];
            JPEG_eof_3[31:24] <= third_2bytes_eof ? 8'h00 : JPEG_eof_2[23:16];
            JPEG_eof_3[23:16] <= third_2bytes_eof ? JPEG_eof_2[23:16] : JPEG_eof_2[15:8];
            JPEG_eof_3[15:8] <= third_2bytes_eof ? JPEG_eof_2[15:8] : JPEG_eof_2[7:0];
            JPEG_eof_3[7:0] <= third_2bytes_eof ? JPEG_eof_2[7:0] : 8'h00;

            // JPEG_eof_4
            JPEG_eof_4[55:24] <= JPEG_eof_3[55:24];
            JPEG_eof_4[23:16] <= fourth_2bytes_eof ? 8'h00 : JPEG_eof_3[23:16];
            JPEG_eof_4[15:8] <= fourth_2bytes_eof ? JPEG_eof_3[23:16] : JPEG_eof_3[15:8];
            JPEG_eof_4[7:0] <= fourth_2bytes_eof ? JPEG_eof_3[15:8] : JPEG_eof_3[7:0];

            // JPEG_eof_5
            JPEG_eof_5[55:16] <= JPEG_eof_4[55:16];
            JPEG_eof_5[15:8] <= fifth_2bytes_eof ? 8'h00 : JPEG_eof_4[15:8];
            JPEG_eof_5[7:0] <= fifth_2bytes_eof ? JPEG_eof_4[15:8] : JPEG_eof_4[7:0];

            // JPEG_eof_5_1, JPEG_eof_6, JPEG_eof_7
            FF_eof_shift <= 2'b11 - FF_count; // Equivalent to 4 - FF_count, then map 00->32, 01->16, 10->8, 11->0 (original 2'b11 -> 0 shift)
            FF_count_1 <= FF_count;

            JPEG_eof_5_1 <= JPEG_eof_5 << { FF_eof_shift[0], 3'b000 };
            JPEG_eof_6 <= JPEG_eof_5_1 << { FF_eof_shift[1], 4'b0000 };

            JPEG_eof_7[79:72] <= (FF_count_1 > 0) ? JPEG_ro[31:24] : JPEG_eof_6[79:72];
            JPEG_eof_7[71:64] <= (FF_count_1 > 1) ? JPEG_ro[23:16] : JPEG_eof_6[71:64];
            JPEG_eof_7[63:56] <= (FF_count_1 > 2) ? JPEG_ro[15:8] : JPEG_eof_6[63:56];
            JPEG_eof_7[55:0] <= JPEG_eof_6[55:0];

            // EOF output data streaming
            if (end_of_file_enable) begin
                JPEG_bitstream_eof <= JPEG_eof_7[79:48];
                JPEG_eof_ro <= JPEG_eof_7[47:16];
                JPEG_eof_ro_ro <= JPEG_eof_7[15:0];
            end else if (eof_bits_1 | eof_bits_2) begin
                JPEG_bitstream_eof <= JPEG_eof_ro;
                JPEG_eof_ro <= { JPEG_eof_ro_ro, {16{1'b0}} };
            end else begin
                // Latch if no condition met
                JPEG_bitstream_eof <= JPEG_bitstream_eof;
                JPEG_eof_ro <= JPEG_eof_ro;
                JPEG_eof_ro_ro <= JPEG_eof_ro_ro;
            end
        end
    end

    // EOF data ready signals
    always_ff @(posedge clk) begin : p_eof_data_ready_signals
        if (rst) begin
            eof_bits_1 <= '0; eof_bits_2 <= '0; eof_bits_3 <= '0;
            eof_data_ready_1 <= '0; eof_data_ready <= '0;
            eof_dpr_1 <= '0; eof_dpr_2 <= '0;
            eof_data_partial_ready <= '0;
        end else begin
            // eof_bits_X
            eof_bits_1 <= end_of_file_enable;
            eof_bits_2 <= eof_bits_1;
            eof_bits_3 <= eof_bits_2;

            // eof_data_ready_1
            if (end_of_file_enable) eof_data_ready_1 <= (extra_bits_eof_1 > (DATA_WIDTH-1));
            else if (eof_bits_1 || eof_bits_2) eof_data_ready_1 <= eof_data_ready;
            else eof_data_ready_1 <= eof_data_ready_1;

            // eof_data_ready
            if (end_of_file_enable) eof_data_ready <= (extra_bits_eof_1 > (DATA_WIDTH*2 -1));
            else if (eof_bits_1) eof_data_ready <= '0;
            else eof_data_ready <= eof_data_ready;

            // eof_data_partial_ready, eof_dpr_1, eof_dpr_2
            if (eof_bits_1) begin
                eof_data_partial_ready <= (extra_bits_eof_1 > 0) && (extra_bits_eof_1 < DATA_WIDTH);
                eof_dpr_1 <= (extra_bits_eof_1 > DATA_WIDTH) && (extra_bits_eof_1 < DATA_WIDTH * 2);
                eof_dpr_2 <= extra_bits_eof_1 > DATA_WIDTH * 2;
            end else begin
                eof_data_partial_ready <= eof_dpr_1;
                eof_dpr_1 <= eof_dpr_2;
                eof_dpr_2 <= '0; // Original resets eof_dpr_2 to 0 here
            end
        end
    end

    // Final output mux
    always_ff @(posedge clk) begin : p_final_output
        if (rst) begin
            data_ready_1 <= '0; JPEG_bitstream_1 <= '0;
        end else begin
            data_ready_1 <= data_ready || eof_data_ready_1;
            JPEG_bitstream_1 <= (eof_bits_1 || eof_bits_2 || eof_bits_3) ?
                                JPEG_bitstream_eof : JPEG_bitstream;
        end
    end

endmodule
