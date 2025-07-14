/*----------------------------------------------------------------------------------
Module Name : y_huff
Description : This module performs Huffman encoding on the quantized 8x8 DCT coefficients 
of the Y (luminance) component in a JPEG image. It takes in DC and AC coefficients, 
computes the differences for DC values, determines the category and value for both 
DC and AC components, and generates corresponding Huffman codes based on 
standard JPEG Huffman tables. The module handles zero-run length encoding for AC 
coefficients and appends end-of-block (EOB) markers as needed. 

The output is a variable-length Huffman-encoded bitstream for the Y component,
ready to be packed and written to the JPEG bitstream by subsequent modules.
The module is pipelined and synchronized using control signals for efficient encoding.
--------------------------------------------------------------------------------*/
`timescale 1ns / 100ps

module y_huff (
    input  logic         clk,                  // Clock signal
    input  logic         rst,                  // Active-high synchronous reset
    input  logic         enable,               // Enable signal

    // 8x8 block of 11-bit Y components
    input  logic [10:0]  Y11, Y12, Y13, Y14, Y15, Y16, Y17, Y18,
    input  logic [10:0]  Y21, Y22, Y23, Y24, Y25, Y26, Y27, Y28,
    input  logic [10:0]  Y31, Y32, Y33, Y34, Y35, Y36, Y37, Y38,
    input  logic [10:0]  Y41, Y42, Y43, Y44, Y45, Y46, Y47, Y48,
    input  logic [10:0]  Y51, Y52, Y53, Y54, Y55, Y56, Y57, Y58,
    input  logic [10:0]  Y61, Y62, Y63, Y64, Y65, Y66, Y67, Y68,
    input  logic [10:0]  Y71, Y72, Y73, Y74, Y75, Y76, Y77, Y78,
    input  logic [10:0]  Y81, Y82, Y83, Y84, Y85, Y86, Y87, Y88,

    // Outputs
    output logic [31:0]  JPEG_bitstream,       // Output compressed bitstream
    output logic         data_ready,           // Signals when bitstream is ready
    output logic [4:0]   output_reg_count,     // Count of output registers used
    output logic         end_of_block_output,  // End-of-block marker
    output logic         end_of_block_empty    // Empty status of EOB marker
);


// Synthesizable SystemVerilog conversion of Verilog reg declarations
logic [7:0] block_counter;
logic [11:0] Y11_amp, Y11_1_pos, Y11_1_neg, Y11_diff;
logic [11:0] Y11_previous, Y11_1;
logic [10:0] Y12_amp, Y12_pos, Y12_neg;
logic [10:0] Y21_pos, Y21_neg, Y31_pos, Y31_neg, Y22_pos, Y22_neg;
logic [10:0] Y13_pos, Y13_neg, Y14_pos, Y14_neg, Y15_pos, Y15_neg;
logic [10:0] Y16_pos, Y16_neg, Y17_pos, Y17_neg, Y18_pos, Y18_neg;
logic [10:0] Y23_pos, Y23_neg, Y24_pos, Y24_neg, Y25_pos, Y25_neg;
logic [10:0] Y26_pos, Y26_neg, Y27_pos, Y27_neg, Y28_pos, Y28_neg;
logic [10:0] Y32_pos, Y32_neg;
logic [10:0] Y33_pos, Y33_neg, Y34_pos, Y34_neg, Y35_pos, Y35_neg;
logic [10:0] Y36_pos, Y36_neg, Y37_pos, Y37_neg, Y38_pos, Y38_neg;
logic [10:0] Y41_pos, Y41_neg, Y42_pos, Y42_neg;
logic [10:0] Y43_pos, Y43_neg, Y44_pos, Y44_neg, Y45_pos, Y45_neg;
logic [10:0] Y46_pos, Y46_neg, Y47_pos, Y47_neg, Y48_pos, Y48_neg;
logic [10:0] Y51_pos, Y51_neg, Y52_pos, Y52_neg;
logic [10:0] Y53_pos, Y53_neg, Y54_pos, Y54_neg, Y55_pos, Y55_neg;
logic [10:0] Y56_pos, Y56_neg, Y57_pos, Y57_neg, Y58_pos, Y58_neg;
logic [10:0] Y61_pos, Y61_neg, Y62_pos, Y62_neg;
logic [10:0] Y63_pos, Y63_neg, Y64_pos, Y64_neg, Y65_pos, Y65_neg;
logic [10:0] Y66_pos, Y66_neg, Y67_pos, Y67_neg, Y68_pos, Y68_neg;
logic [10:0] Y71_pos, Y71_neg, Y72_pos, Y72_neg;
logic [10:0] Y73_pos, Y73_neg, Y74_pos, Y74_neg, Y75_pos, Y75_neg;
logic [10:0] Y76_pos, Y76_neg, Y77_pos, Y77_neg, Y78_pos, Y78_neg;
logic [10:0] Y81_pos, Y81_neg, Y82_pos, Y82_neg;
logic [10:0] Y83_pos, Y83_neg, Y84_pos, Y84_neg, Y85_pos, Y85_neg;
logic [10:0] Y86_pos, Y86_neg, Y87_pos, Y87_neg, Y88_pos, Y88_neg;
logic [3:0] Y11_bits_pos, Y11_bits_neg, Y11_bits, Y11_bits_1;
logic [3:0] Y12_bits_pos, Y12_bits_neg, Y12_bits, Y12_bits_1;
logic [3:0] Y12_bits_2, Y12_bits_3;
logic Y11_msb, Y12_msb, Y12_msb_1, data_ready;
logic enable_1, enable_2, enable_3, enable_4, enable_5, enable_6;
logic enable_7, enable_8, enable_9, enable_10, enable_11, enable_12;
logic enable_13, enable_module, enable_latch_7, enable_latch_8;
logic Y12_et_zero, rollover, rollover_1, rollover_2, rollover_3;
logic rollover_4, rollover_5, rollover_6, rollover_7;
logic Y21_et_zero, Y21_msb, Y31_et_zero, Y31_msb;
logic Y22_et_zero, Y22_msb, Y13_et_zero, Y13_msb;
logic Y14_et_zero, Y14_msb, Y15_et_zero, Y15_msb;
logic Y16_et_zero, Y16_msb, Y17_et_zero, Y17_msb;
logic Y18_et_zero, Y18_msb;
logic Y23_et_zero, Y23_msb, Y24_et_zero, Y24_msb;
logic Y25_et_zero, Y25_msb, Y26_et_zero, Y26_msb;
logic Y27_et_zero, Y27_msb, Y28_et_zero, Y28_msb;
logic Y32_et_zero, Y32_msb, Y33_et_zero, Y33_msb;
logic Y34_et_zero, Y34_msb, Y35_et_zero, Y35_msb;
logic Y36_et_zero, Y36_msb, Y37_et_zero, Y37_msb;
logic Y38_et_zero, Y38_msb;
logic Y41_et_zero, Y41_msb, Y42_et_zero, Y42_msb;
logic Y43_et_zero, Y43_msb, Y44_et_zero, Y44_msb;
logic Y45_et_zero, Y45_msb, Y46_et_zero, Y46_msb;
logic Y47_et_zero, Y47_msb, Y48_et_zero, Y48_msb;
logic Y51_et_zero, Y51_msb, Y52_et_zero, Y52_msb;
logic Y53_et_zero, Y53_msb, Y54_et_zero, Y54_msb;
logic Y55_et_zero, Y55_msb, Y56_et_zero, Y56_msb;
logic Y57_et_zero, Y57_msb, Y58_et_zero, Y58_msb;
logic Y61_et_zero, Y61_msb, Y62_et_zero, Y62_msb;
logic Y63_et_zero, Y63_msb, Y64_et_zero, Y64_msb;
logic Y65_et_zero, Y65_msb, Y66_et_zero, Y66_msb;
logic Y67_et_zero, Y67_msb, Y68_et_zero, Y68_msb;
logic Y71_et_zero, Y71_msb, Y72_et_zero, Y72_msb;
logic Y73_et_zero, Y73_msb, Y74_et_zero, Y74_msb;
logic Y75_et_zero, Y75_msb, Y76_et_zero, Y76_msb;
logic Y77_et_zero, Y77_msb, Y78_et_zero, Y78_msb;
logic Y81_et_zero, Y81_msb, Y82_et_zero, Y82_msb;
logic Y83_et_zero, Y83_msb, Y84_et_zero, Y84_msb;
logic Y85_et_zero, Y85_msb, Y86_et_zero, Y86_msb;
logic Y87_et_zero, Y87_msb, Y88_et_zero, Y88_msb;
logic Y12_et_zero_1, Y12_et_zero_2, Y12_et_zero_3, Y12_et_zero_4, Y12_et_zero_5;
logic [10:0] Y_DC [0:11];
logic [3:0] Y_DC_code_length [0:11];
logic [15:0] Y_AC [0:161];
logic [4:0] Y_AC_code_length [0:161];
logic [7:0] Y_AC_run_code [0:250];
logic [10:0] Y11_Huff, Y11_Huff_1, Y11_Huff_2;
logic [15:0] Y12_Huff, Y12_Huff_1, Y12_Huff_2;
logic [3:0] Y11_Huff_count, Y11_Huff_shift, Y11_Huff_shift_1, Y11_amp_shift, Y12_amp_shift;
logic [3:0] Y12_Huff_shift, Y12_Huff_shift_1, zero_run_length, zrl_1, zrl_2, zrl_3;
logic [4:0] Y12_Huff_count, Y12_Huff_count_1;
logic [4:0] output_reg_count, Y11_output_count, old_orc_1, old_orc_2;
logic [4:0] old_orc_3, old_orc_4, old_orc_5, old_orc_6, Y12_oc_1;
logic [4:0] orc_3, orc_4, orc_5, orc_6, orc_7, orc_8;
logic [4:0] Y12_output_count;
logic [4:0] Y12_edge, Y12_edge_1, Y12_edge_2, Y12_edge_3, Y12_edge_4;
logic [31:0] JPEG_bitstream, JPEG_bs, JPEG_bs_1, JPEG_bs_2, JPEG_bs_3, JPEG_bs_4, JPEG_bs_5;
logic [31:0] JPEG_Y12_bs, JPEG_Y12_bs_1, JPEG_Y12_bs_2, JPEG_Y12_bs_3, JPEG_Y12_bs_4;
logic [31:0] JPEG_ro_bs, JPEG_ro_bs_1, JPEG_ro_bs_2, JPEG_ro_bs_3, JPEG_ro_bs_4;
logic [21:0] Y11_JPEG_LSBs_3;
logic [10:0] Y11_JPEG_LSBs, Y11_JPEG_LSBs_1, Y11_JPEG_LSBs_2;
logic [9:0] Y12_JPEG_LSBs, Y12_JPEG_LSBs_1, Y12_JPEG_LSBs_2, Y12_JPEG_LSBs_3;
logic [25:0] Y11_JPEG_bits, Y11_JPEG_bits_1, Y12_JPEG_bits, Y12_JPEG_LSBs_4;
logic [7:0] Y12_code_entry;
logic third_8_all_0s, fourth_8_all_0s, fifth_8_all_0s, sixth_8_all_0s, seventh_8_all_0s;
logic eighth_8_all_0s, end_of_block, code_15_0, zrl_et_15, end_of_block_output;
logic end_of_block_empty;
logic [7:0] code_index = {zrl_2, Y12_bits};

// Synthesizable SystemVerilog conversion of always block
always_ff @(posedge clk) begin
	if (rst) begin
		third_8_all_0s    <= 0;
		fourth_8_all_0s   <= 0;
		fifth_8_all_0s    <= 0;
		sixth_8_all_0s    <= 0;
		seventh_8_all_0s  <= 0;
		eighth_8_all_0s   <= 0;
	end
	else if (enable_1) begin
		third_8_all_0s   <= Y25_et_zero & Y34_et_zero & Y43_et_zero & Y52_et_zero
		                    & Y61_et_zero & Y71_et_zero & Y62_et_zero & Y53_et_zero;
		fourth_8_all_0s  <= Y44_et_zero & Y35_et_zero & Y26_et_zero & Y17_et_zero
		                    & Y18_et_zero & Y27_et_zero & Y36_et_zero & Y45_et_zero;
		fifth_8_all_0s   <= Y54_et_zero & Y63_et_zero & Y72_et_zero & Y81_et_zero
		                    & Y82_et_zero & Y73_et_zero & Y64_et_zero & Y55_et_zero;
		sixth_8_all_0s   <= Y46_et_zero & Y37_et_zero & Y28_et_zero & Y38_et_zero
		                    & Y47_et_zero & Y56_et_zero & Y65_et_zero & Y74_et_zero;
		seventh_8_all_0s <= Y83_et_zero & Y84_et_zero & Y75_et_zero & Y66_et_zero
		                    & Y57_et_zero & Y48_et_zero & Y58_et_zero & Y67_et_zero;
		eighth_8_all_0s  <= Y76_et_zero & Y85_et_zero & Y86_et_zero & Y77_et_zero
		                    & Y68_et_zero & Y78_et_zero & Y87_et_zero & Y88_et_zero;
	end
end

/*
end_of_block checks to see if there are any nonzero elements left in the block.
If there aren't any nonzero elements left, then the last bits in the JPEG stream
will be the end-of-block code.

The purpose of this register is to determine if the zero run length code 15-0 
should be used or not. It should be used if there are 15 or more zeros in a row, 
followed by a nonzero value. If there are only zeros left in the block, then 
end_of_block will be 1. If there are any nonzero values left in the block, 
end_of_block will be 0.
*/

// SystemVerilog - Synthesizable conversion

// end_of_block logic based on block_counter and enable flags
always_ff @(posedge clk) begin
	if (rst)
		end_of_block <= 0;
	else if (enable)
		end_of_block <= 0;
	else if (enable_module && block_counter < 32)
		end_of_block <= third_8_all_0s & fourth_8_all_0s & fifth_8_all_0s
		              & sixth_8_all_0s & seventh_8_all_0s & eighth_8_all_0s;
	else if (enable_module && block_counter < 48)
		end_of_block <= fifth_8_all_0s & sixth_8_all_0s & seventh_8_all_0s 
		              & eighth_8_all_0s;
	else if (enable_module && block_counter <= 64)
		end_of_block <= seventh_8_all_0s & eighth_8_all_0s;
	else if (enable_module && block_counter > 64)
		end_of_block <= 1;
end

// block_counter update logic
always_ff @(posedge clk) begin
	if (rst)
		block_counter <= 0;
	else if (enable)
		block_counter <= 0;
	else if (enable_module)
		block_counter <= block_counter + 1;
end

// output_reg_count update logic
always_ff @(posedge clk) begin
	if (rst)
		output_reg_count <= 0;
	else if (end_of_block_output)
		output_reg_count <= 0;
	else if (enable_6)
		output_reg_count <= output_reg_count + Y11_output_count;
	else if (enable_latch_7)
		output_reg_count <= output_reg_count + Y12_oc_1;
end

// old_orc_1 latch
always_ff @(posedge clk) begin
	if (rst)
		old_orc_1 <= 0;
	else if (end_of_block_output)
		old_orc_1 <= 0;
	else if (enable_module)
		old_orc_1 <= output_reg_count;
end

// SystemVerilog - Synthesizable code with detailed comments
always_ff @(posedge clk) begin
	if (rst) begin
		// Reset all rollover detection and output tracking signals
		rollover       <= 0; rollover_1 <= 0; rollover_2 <= 0;
		rollover_3     <= 0; rollover_4 <= 0; rollover_5 <= 0;
		rollover_6     <= 0; rollover_7 <= 0;

		old_orc_2      <= 0;          // Holds delayed output_reg_count value
		orc_3          <= 0; orc_4 <= 0; orc_5 <= 0;
		orc_6          <= 0; orc_7 <= 0; orc_8 <= 0;

		data_ready          <= 0;     // Indicates when JPEG bitstream is ready
		end_of_block_output <= 0;     // Signals that end of current 8x8 block is reached
		end_of_block_empty  <= 0;     // Indicates EOB with no data (all-zero case)
	end
	else if (enable_module) begin
		// Detect if rollover has occurred in the output register count
		rollover   <= (old_orc_1 > output_reg_count);

		// Shift the rollover signal through pipeline registers
		rollover_1 <= rollover;
		rollover_2 <= rollover_1;
		rollover_3 <= rollover_2;
		rollover_4 <= rollover_3;
		rollover_5 <= rollover_4;
		rollover_6 <= rollover_5;
		rollover_7 <= rollover_6;

		// Delay output_reg_count for comparison
		old_orc_2 <= old_orc_1;
		orc_3     <= old_orc_2;
		orc_4     <= orc_3;
		orc_5     <= orc_4;
		orc_6     <= orc_5;
		orc_7     <= orc_6;
		orc_8     <= orc_7;

		// Mark data as ready either on rollover or when block_counter hits 77
		data_ready          <= rollover_6 | (block_counter == 77);

		// Signal end of block when counter reaches 77
		end_of_block_output <= (block_counter == 77);

		// Assert empty end-of-block if output was rolled over and no data was emitted
		end_of_block_empty  <= rollover_7 & (block_counter == 77) & (output_reg_count == 0);
	end
end

// On each positive clock edge, update the JPEG bitstream register
always_ff @(posedge clk) begin
	if (rst) begin
		// Clear the final bitstream register on reset
		JPEG_bs_5 <= 0; 
	end
	else if (enable_module) begin 
		// Conditionally assign each bit of JPEG_bs_5:
		// If rollover_6 is high and orc_7 > index, use rollover (ro) buffer value
		// Otherwise, use the normal JPEG bitstream value from JPEG_bs_4
		// This mechanism ensures the correct bits are selected in rollover scenarios

		JPEG_bs_5[31] <= (rollover_6 && orc_7 > 0)  ? JPEG_ro_bs_4[31] : JPEG_bs_4[31];
		JPEG_bs_5[30] <= (rollover_6 && orc_7 > 1)  ? JPEG_ro_bs_4[30] : JPEG_bs_4[30];
		JPEG_bs_5[29] <= (rollover_6 && orc_7 > 2)  ? JPEG_ro_bs_4[29] : JPEG_bs_4[29];
		JPEG_bs_5[28] <= (rollover_6 && orc_7 > 3)  ? JPEG_ro_bs_4[28] : JPEG_bs_4[28];
		JPEG_bs_5[27] <= (rollover_6 && orc_7 > 4)  ? JPEG_ro_bs_4[27] : JPEG_bs_4[27];
		JPEG_bs_5[26] <= (rollover_6 && orc_7 > 5)  ? JPEG_ro_bs_4[26] : JPEG_bs_4[26];
		JPEG_bs_5[25] <= (rollover_6 && orc_7 > 6)  ? JPEG_ro_bs_4[25] : JPEG_bs_4[25];
		JPEG_bs_5[24] <= (rollover_6 && orc_7 > 7)  ? JPEG_ro_bs_4[24] : JPEG_bs_4[24];
		JPEG_bs_5[23] <= (rollover_6 && orc_7 > 8)  ? JPEG_ro_bs_4[23] : JPEG_bs_4[23];
		JPEG_bs_5[22] <= (rollover_6 && orc_7 > 9)  ? JPEG_ro_bs_4[22] : JPEG_bs_4[22];
		JPEG_bs_5[21] <= (rollover_6 && orc_7 > 10) ? JPEG_ro_bs_4[21] : JPEG_bs_4[21];
		JPEG_bs_5[20] <= (rollover_6 && orc_7 > 11) ? JPEG_ro_bs_4[20] : JPEG_bs_4[20];
		JPEG_bs_5[19] <= (rollover_6 && orc_7 > 12) ? JPEG_ro_bs_4[19] : JPEG_bs_4[19];
		JPEG_bs_5[18] <= (rollover_6 && orc_7 > 13) ? JPEG_ro_bs_4[18] : JPEG_bs_4[18];
		JPEG_bs_5[17] <= (rollover_6 && orc_7 > 14) ? JPEG_ro_bs_4[17] : JPEG_bs_4[17];
		JPEG_bs_5[16] <= (rollover_6 && orc_7 > 15) ? JPEG_ro_bs_4[16] : JPEG_bs_4[16];
		JPEG_bs_5[15] <= (rollover_6 && orc_7 > 16) ? JPEG_ro_bs_4[15] : JPEG_bs_4[15];
		JPEG_bs_5[14] <= (rollover_6 && orc_7 > 17) ? JPEG_ro_bs_4[14] : JPEG_bs_4[14];
		JPEG_bs_5[13] <= (rollover_6 && orc_7 > 18) ? JPEG_ro_bs_4[13] : JPEG_bs_4[13];
		JPEG_bs_5[12] <= (rollover_6 && orc_7 > 19) ? JPEG_ro_bs_4[12] : JPEG_bs_4[12];
		JPEG_bs_5[11] <= (rollover_6 && orc_7 > 20) ? JPEG_ro_bs_4[11] : JPEG_bs_4[11];
		JPEG_bs_5[10] <= (rollover_6 && orc_7 > 21) ? JPEG_ro_bs_4[10] : JPEG_bs_4[10];
		JPEG_bs_5[9]  <= (rollover_6 && orc_7 > 22) ? JPEG_ro_bs_4[9]  : JPEG_bs_4[9];
		JPEG_bs_5[8]  <= (rollover_6 && orc_7 > 23) ? JPEG_ro_bs_4[8]  : JPEG_bs_4[8];
		JPEG_bs_5[7]  <= (rollover_6 && orc_7 > 24) ? JPEG_ro_bs_4[7]  : JPEG_bs_4[7];
		JPEG_bs_5[6]  <= (rollover_6 && orc_7 > 25) ? JPEG_ro_bs_4[6]  : JPEG_bs_4[6];
		JPEG_bs_5[5]  <= (rollover_6 && orc_7 > 26) ? JPEG_ro_bs_4[5]  : JPEG_bs_4[5];
		JPEG_bs_5[4]  <= (rollover_6 && orc_7 > 27) ? JPEG_ro_bs_4[4]  : JPEG_bs_4[4];
		JPEG_bs_5[3]  <= (rollover_6 && orc_7 > 28) ? JPEG_ro_bs_4[3]  : JPEG_bs_4[3];
		JPEG_bs_5[2]  <= (rollover_6 && orc_7 > 29) ? JPEG_ro_bs_4[2]  : JPEG_bs_4[2];
		JPEG_bs_5[1]  <= (rollover_6 && orc_7 > 30) ? JPEG_ro_bs_4[1]  : JPEG_bs_4[1];

		// Bit 0 is always taken from JPEG_bs_4 unconditionally
		JPEG_bs_5[0]  <= JPEG_bs_4[0];
	end
end

// Stage 1: Final 32-bit bitstream preparation
always_ff @(posedge clk) begin
	if (rst) begin
		JPEG_bs_4 <= 0;         // Clear final shifted bitstream
		JPEG_ro_bs_4 <= 0;      // Clear final rollover bitstream
	end
	else if (enable_module) begin 
		// Right shift if only 1 bit remains to be output
		JPEG_bs_4 <= (old_orc_6 == 1) ? JPEG_bs_3 >> 1 : JPEG_bs_3;

		// Left shift if Y12_edge_4 is 1 or less, for rollover buffer
		JPEG_ro_bs_4 <= (Y12_edge_4 <= 1) ? JPEG_ro_bs_3 << 1 : JPEG_ro_bs_3;
	end
end	

// Stage 2
always_ff @(posedge clk) begin
	if (rst) begin
		JPEG_bs_3 <= 0;
		old_orc_6 <= 0;
		JPEG_ro_bs_3 <= 0;
		Y12_edge_4 <= 0;
	end
	else if (enable_module) begin 
		// Right shift by 2 if more than 2 bits present, else keep as is
		JPEG_bs_3 <= (old_orc_5 >= 2) ? JPEG_bs_2 >> 2 : JPEG_bs_2;
		old_orc_6 <= (old_orc_5 >= 2) ? old_orc_5 - 2 : old_orc_5;

		// Left shift by 2 if Y12_edge_3 is 2 or less
		JPEG_ro_bs_3 <= (Y12_edge_3 <= 2) ? JPEG_ro_bs_2 << 2 : JPEG_ro_bs_2;
		Y12_edge_4 <= (Y12_edge_3 <= 2) ? Y12_edge_3 : Y12_edge_3 - 2;
	end
end	

// Stage 3
always_ff @(posedge clk) begin
	if (rst) begin
		JPEG_bs_2 <= 0;
		old_orc_5 <= 0;
		JPEG_ro_bs_2 <= 0;
		Y12_edge_3 <= 0;
	end
	else if (enable_module) begin 
		// Right shift by 4 if needed
		JPEG_bs_2 <= (old_orc_4 >= 4) ? JPEG_bs_1 >> 4 : JPEG_bs_1;
		old_orc_5 <= (old_orc_4 >= 4) ? old_orc_4 - 4 : old_orc_4;

		// Left shift rollover stream if edge is small
		JPEG_ro_bs_2 <= (Y12_edge_2 <= 4) ? JPEG_ro_bs_1 << 4 : JPEG_ro_bs_1;
		Y12_edge_3 <= (Y12_edge_2 <= 4) ? Y12_edge_2 : Y12_edge_2 - 4;
	end
end	

// Stage 4
always_ff @(posedge clk) begin
	if (rst) begin
		JPEG_bs_1 <= 0;
		old_orc_4 <= 0;
		JPEG_ro_bs_1 <= 0;
		Y12_edge_2 <= 0;
	end
	else if (enable_module) begin 
		// Right shift by 8 if enough bits are present
		JPEG_bs_1 <= (old_orc_3 >= 8) ? JPEG_bs >> 8 : JPEG_bs;
		old_orc_4 <= (old_orc_3 >= 8) ? old_orc_3 - 8 : old_orc_3;

		// Left shift for rollover stream
		JPEG_ro_bs_1 <= (Y12_edge_1 <= 8) ? JPEG_ro_bs << 8 : JPEG_ro_bs;
		Y12_edge_2 <= (Y12_edge_1 <= 8) ? Y12_edge_1 : Y12_edge_1 - 8;
	end
end	

// Stage 5: Initial bitstream construction from Huffman-encoded Y11 block
always_ff @(posedge clk) begin
	if (rst) begin
		JPEG_bs <= 0;
		old_orc_3 <= 0;
		JPEG_ro_bs <= 0;
		Y12_edge_1 <= 0;
		Y11_JPEG_bits_1 <= 0;
	end
	else if (enable_module) begin 
		// Initial JPEG bitstream: shift right by 10 if we have 16 or more bits; else shift left
		JPEG_bs <= (old_orc_2 >= 16) ? Y11_JPEG_bits >> 10 : Y11_JPEG_bits << 6;
		old_orc_3 <= (old_orc_2 >= 16) ? old_orc_2 - 16 : old_orc_2;

		// Prepare rollover stream with full left shift if edge ≤ 16
		JPEG_ro_bs <= (Y12_edge <= 16) ? Y11_JPEG_bits_1 << 16 : Y11_JPEG_bits_1;
		Y12_edge_1 <= (Y12_edge <= 16) ? Y12_edge : Y12_edge - 16;

		// Save the input bits for use in next stage
		Y11_JPEG_bits_1 <= Y11_JPEG_bits;
	end
end

// Construct the final JPEG bitstream for Y12 block
always_ff @(posedge clk) begin
	if (rst) begin
		Y12_JPEG_bits <= 0;   // Clear Y12 output bitstream
		Y12_edge <= 0;        // Clear bitstream width info
	end
	else if (enable_module) begin 
		// Conditional bit selection: if shift is large, use LSBs; else take from Huffman
		Y12_JPEG_bits[25] <= (Y12_Huff_shift_1 >= 16) ? Y12_JPEG_LSBs_4[25] : Y12_Huff_2[15];
		Y12_JPEG_bits[24] <= (Y12_Huff_shift_1 >= 15) ? Y12_JPEG_LSBs_4[24] : Y12_Huff_2[14];
		Y12_JPEG_bits[23] <= (Y12_Huff_shift_1 >= 14) ? Y12_JPEG_LSBs_4[23] : Y12_Huff_2[13];
		Y12_JPEG_bits[22] <= (Y12_Huff_shift_1 >= 13) ? Y12_JPEG_LSBs_4[22] : Y12_Huff_2[12];
		Y12_JPEG_bits[21] <= (Y12_Huff_shift_1 >= 12) ? Y12_JPEG_LSBs_4[21] : Y12_Huff_2[11];
		Y12_JPEG_bits[20] <= (Y12_Huff_shift_1 >= 11) ? Y12_JPEG_LSBs_4[20] : Y12_Huff_2[10];
		Y12_JPEG_bits[19] <= (Y12_Huff_shift_1 >= 10) ? Y12_JPEG_LSBs_4[19] : Y12_Huff_2[9];
		Y12_JPEG_bits[18] <= (Y12_Huff_shift_1 >= 9) ? Y12_JPEG_LSBs_4[18] : Y12_Huff_2[8];
		Y12_JPEG_bits[17] <= (Y12_Huff_shift_1 >= 8) ? Y12_JPEG_LSBs_4[17] : Y12_Huff_2[7];
		Y12_JPEG_bits[16] <= (Y12_Huff_shift_1 >= 7) ? Y12_JPEG_LSBs_4[16] : Y12_Huff_2[6];
		Y12_JPEG_bits[15] <= (Y12_Huff_shift_1 >= 6) ? Y12_JPEG_LSBs_4[15] : Y12_Huff_2[5];
		Y12_JPEG_bits[14] <= (Y12_Huff_shift_1 >= 5) ? Y12_JPEG_LSBs_4[14] : Y12_Huff_2[4];
		Y12_JPEG_bits[13] <= (Y12_Huff_shift_1 >= 4) ? Y12_JPEG_LSBs_4[13] : Y12_Huff_2[3];
		Y12_JPEG_bits[12] <= (Y12_Huff_shift_1 >= 3) ? Y12_JPEG_LSBs_4[12] : Y12_Huff_2[2];
		Y12_JPEG_bits[11] <= (Y12_Huff_shift_1 >= 2) ? Y12_JPEG_LSBs_4[11] : Y12_Huff_2[1];
		Y12_JPEG_bits[10] <= (Y12_Huff_shift_1 >= 1) ? Y12_JPEG_LSBs_4[10] : Y12_Huff_2[0];

		// Lower 10 bits always come from the LSBs (no Huffman code here)
		Y12_JPEG_bits[9:0] <= Y12_JPEG_LSBs_4[9:0];

		// Track total width of the output (used for shifting in later stages)
		Y12_edge <= old_orc_2 + 26;
	end
end

// Construct the final JPEG bitstream for Y11 block
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_JPEG_bits <= 0;  // Clear Y11 JPEG output stream
	end
	else if (enable_7) begin 
		// Similar bit selection logic as above for Y11
		Y11_JPEG_bits[25] <= (Y11_Huff_shift_1 >= 11) ? Y11_JPEG_LSBs_3[21] : Y11_Huff_2[10];
		Y11_JPEG_bits[24] <= (Y11_Huff_shift_1 >= 10) ? Y11_JPEG_LSBs_3[20] : Y11_Huff_2[9];
		Y11_JPEG_bits[23] <= (Y11_Huff_shift_1 >= 9) ? Y11_JPEG_LSBs_3[19] : Y11_Huff_2[8];
		Y11_JPEG_bits[22] <= (Y11_Huff_shift_1 >= 8) ? Y11_JPEG_LSBs_3[18] : Y11_Huff_2[7];
		Y11_JPEG_bits[21] <= (Y11_Huff_shift_1 >= 7) ? Y11_JPEG_LSBs_3[17] : Y11_Huff_2[6];
		Y11_JPEG_bits[20] <= (Y11_Huff_shift_1 >= 6) ? Y11_JPEG_LSBs_3[16] : Y11_Huff_2[5];
		Y11_JPEG_bits[19] <= (Y11_Huff_shift_1 >= 5) ? Y11_JPEG_LSBs_3[15] : Y11_Huff_2[4];
		Y11_JPEG_bits[18] <= (Y11_Huff_shift_1 >= 4) ? Y11_JPEG_LSBs_3[14] : Y11_Huff_2[3];
		Y11_JPEG_bits[17] <= (Y11_Huff_shift_1 >= 3) ? Y11_JPEG_LSBs_3[13] : Y11_Huff_2[2];
		Y11_JPEG_bits[16] <= (Y11_Huff_shift_1 >= 2) ? Y11_JPEG_LSBs_3[12] : Y11_Huff_2[1];
		Y11_JPEG_bits[15] <= (Y11_Huff_shift_1 >= 1) ? Y11_JPEG_LSBs_3[11] : Y11_Huff_2[0];

		// Lower 11 bits: always from LSBs
		Y11_JPEG_bits[14:4] <= Y11_JPEG_LSBs_3[10:0];
	end
	else if (enable_latch_8) begin
		// On latch signal, copy Y12 bits into Y11 register
		Y11_JPEG_bits <= Y12_JPEG_bits;
	end
end

// Stage 1: Generate final output count and align JPEG LSBs for Y12 block
always_ff @(posedge clk) begin
	if (rst) begin
		Y12_oc_1 <= 0;
		Y12_JPEG_LSBs_4 <= 0;
		Y12_Huff_2 <= 0;
		Y12_Huff_shift_1 <= 0;
	end
	else if (enable_module) begin
		// Output count is 0 if:
		//  - All ACs are zero AND
		//  - Not a ZRL (15-0) code AND
		//  - Not the DC block (block 67)
		Y12_oc_1 <= (Y12_et_zero_5 && !code_15_0 && block_counter != 67)
		           ? 0 : (Y12_bits_3 + Y12_Huff_count_1);

		// Align LSBs by shifting left (Huffman shift tells how much space Huffman code used)
		Y12_JPEG_LSBs_4 <= Y12_JPEG_LSBs_3 << Y12_Huff_shift;

		// Latch Huffman code and its shift for next stage
		Y12_Huff_2 <= Y12_Huff_1;
		Y12_Huff_shift_1 <= Y12_Huff_shift;
	end
end

// Stage 2: Align JPEG LSBs for Y11 block
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_JPEG_LSBs_3 <= 0;
		Y11_Huff_2 <= 0;
		Y11_Huff_shift_1 <= 0;
	end
	else if (enable_6) begin
		// Shift LSBs left to align them after Huffman prefix
		Y11_JPEG_LSBs_3 <= Y11_JPEG_LSBs_2 << Y11_Huff_shift;

		// Store Huffman code and shift amount for final assembly
		Y11_Huff_2 <= Y11_Huff_1;
		Y11_Huff_shift_1 <= Y11_Huff_shift;
	end
end

// Stage 3: Prepare Huffman data for Y12 block
always_ff @(posedge clk) begin
	if (rst) begin
		Y12_Huff_shift <= 0;
		Y12_Huff_1 <= 0;
		Y12_JPEG_LSBs_3 <= 0;
		Y12_bits_3 <= 0;
		Y12_Huff_count_1 <= 0;
		Y12_et_zero_5 <= 0;
		code_15_0 <= 0;
	end
	else if (enable_module) begin
		// Determine how much to shift LSBs based on Huffman code length
		Y12_Huff_shift <= 16 - Y12_Huff_count;

		// Latch Huffman code
		Y12_Huff_1 <= Y12_Huff;

		// Carry forward LSBs and bit counts
		Y12_JPEG_LSBs_3 <= Y12_JPEG_LSBs_2;
		Y12_bits_3 <= Y12_bits_2;
		Y12_Huff_count_1 <= Y12_Huff_count;
		Y12_et_zero_5 <= Y12_et_zero_4;

		// Detect if we should use 15-0 ZRL (Zero Run Length) code
		code_15_0 <= zrl_et_15 && !end_of_block;
	end
end

// Stage 1: Final output count and aligned JPEG LSBs for Y11 block
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_output_count <= 0;
		Y11_JPEG_LSBs_2 <= 0;
		Y11_Huff_shift <= 0;
		Y11_Huff_1 <= 0;
	end
	else if (enable_5) begin
		// Total bits to output = Huffman bits + amplitude bits
		Y11_output_count <= Y11_bits_1 + Y11_Huff_count;

		// Align amplitude bits with Huffman code
		Y11_JPEG_LSBs_2 <= Y11_JPEG_LSBs_1 << Y11_amp_shift;

		// Shift amount required to align LSBs after Huffman prefix
		Y11_Huff_shift <= 11 - Y11_Huff_count;

		// Store Huffman code for next pipeline stage
		Y11_Huff_1 <= Y11_Huff;
	end
end

// Stage 2: Huffman & LSB pipeline for Y12 AC block
always_ff @(posedge clk) begin
	if (rst) begin
		Y12_JPEG_LSBs_2 <= 0;
		Y12_Huff <= 0;
		Y12_Huff_count <= 0;
		Y12_bits_2 <= 0;
		Y12_et_zero_4 <= 0;
		zrl_et_15 <= 0;
		zrl_3 <= 0;
	end
	else if (enable_module) begin
		// Align amplitude bits for Y12 block
		Y12_JPEG_LSBs_2 <= Y12_JPEG_LSBs_1 << Y12_amp_shift;

		// Fetch Huffman code and length from AC lookup table
		Y12_Huff <= Y_AC[Y12_code_entry];
		Y12_Huff_count <= Y_AC_code_length[Y12_code_entry];

		// Pass down the number of amplitude bits
		Y12_bits_2 <= Y12_bits_1;

		// Carry zero detection forward
		Y12_et_zero_4 <= Y12_et_zero_3;

		// Detect ZRL (Zero Run Length = 15 zeros in a row)
		zrl_et_15 <= (zrl_3 == 15);
		zrl_3 <= zrl_2;
	end
end

// Stage 3: Huffman generation for Y11 DC block
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_Huff <= 0;
		Y11_Huff_count <= 0;
		Y11_amp_shift <= 0;
		Y11_JPEG_LSBs_1 <= 0;
		Y11_bits_1 <= 0;
	end
	else if (enable_4) begin
		// Lookup Huffman code from DC table using category (Y11_bits)
		Y11_Huff[10:0] <= Y_DC[Y11_bits];

		// Get the Huffman code length
		Y11_Huff_count <= Y_DC_code_length[Y11_bits];

		// Shift amount = total bits (11) - amplitude bits
		Y11_amp_shift <= 11 - Y11_bits;

		// Pass LSBs for JPEG output
		Y11_JPEG_LSBs_1 <= Y11_JPEG_LSBs;

		// Pass number of bits used in DC amplitude
		Y11_bits_1 <= Y11_bits;
	end
end

// Stage 1: Generate code entry and prepare amplitude data for Y12 block
always_ff @(posedge clk) begin
	if (rst) begin
		Y12_code_entry   <= 0;
		Y12_JPEG_LSBs_1  <= 0;
		Y12_amp_shift    <= 0;
		Y12_bits_1       <= 0;
		Y12_et_zero_3    <= 0;
		zrl_2            <= 0;
	end
	else if (enable_module) begin
		// Look up Huffman code entry using run-length code
		Y12_code_entry <= Y_AC_run_code[code_index];

		// Store amplitude LSBs and shift info for next stage
		Y12_JPEG_LSBs_1 <= Y12_JPEG_LSBs;
		Y12_amp_shift   <= 10 - Y12_bits;

		// Preserve number of amplitude bits
		Y12_bits_1      <= Y12_bits;

		// Pass zero flag and ZRL counter
		Y12_et_zero_3   <= Y12_et_zero_2;
		zrl_2           <= zrl_1;
	end
end

// Stage 2: Select final Y11 bits and amplitude LSBs
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_bits      <= 0;
		Y11_JPEG_LSBs <= 0;
	end
	else if (enable_3) begin
		// Select between positive/negative bit-length depending on MSB sign
		Y11_bits      <= Y11_msb ? Y11_bits_neg : Y11_bits_pos;

		// Store lower 11 bits of amplitude (actual encoded data)
		Y11_JPEG_LSBs <= Y11_amp[10:0];
	end
end

// Stage 3: Select final Y12 bits and amplitude, handle ZRL and zero-run logic
always_ff @(posedge clk) begin
	if (rst) begin
		Y12_bits       <= 0;
		Y12_JPEG_LSBs  <= 0;
		zrl_1          <= 0;
		Y12_et_zero_2  <= 0;
	end
	else if (enable_module) begin
		// Choose bit length based on sign
		Y12_bits      <= Y12_msb_1 ? Y12_bits_neg : Y12_bits_pos;

		// Store lower 10 bits of amplitude
		Y12_JPEG_LSBs <= Y12_amp[9:0];

		// Update zero-run logic, reset at specific counter condition
		zrl_1         <= (block_counter == 62 && Y12_et_zero) ? 0 : zero_run_length;

		// Pass zero flag
		Y12_et_zero_2 <= Y12_et_zero_1;
	end
end

// Stage 4: Final amplitude selection for Y11 block (positive or negative)
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_amp <= 0;
	end
	else if (enable_2) begin
		// Choose amplitude based on sign bit
		Y11_amp <= Y11_msb ? Y11_1_neg : Y11_1_pos;
	end
end

// Zero Run Length Counter: Counts consecutive zero AC coefficients in Y12
always_ff @(posedge clk) begin
	if (rst) begin
		zero_run_length <= 0; // Reset zero run length
	end
	else if (enable) begin
		zero_run_length <= 0; // Reset when new block starts
	end
	else if (enable_module) begin
		// If current coefficient is zero, increment run; else reset
		zero_run_length <= Y12_et_zero ? zero_run_length + 1 : 0;
	end
end

// Y12 Amplitude Selection and MSB Tracking
always_ff @(posedge clk) begin
	if (rst) begin
		Y12_amp        <= 0;
		Y12_et_zero_1  <= 0;
		Y12_msb_1      <= 0;
	end
	else if (enable_module) begin
		// Choose between positive and negative amplitude representation
		Y12_amp       <= Y12_msb ? Y12_neg : Y12_pos;

		// Store if current value is zero and save sign bit
		Y12_et_zero_1 <= Y12_et_zero;
		Y12_msb_1     <= Y12_msb;
	end
end

// Y11 Amplitude Difference Handling and MSB Flag
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_1_pos    <= 0;
		Y11_1_neg    <= 0;
		Y11_msb      <= 0;
		Y11_previous <= 0;
	end
	else if (enable_1) begin
		// Store positive and negative versions of the DC difference
		Y11_1_pos    <= Y11_diff;
		Y11_1_neg    <= Y11_diff - 1;

		// Set MSB flag (1 if negative, 0 if positive)
		Y11_msb      <= Y11_diff[11];

		// Store previous DC value for next frame/block
		Y11_previous <= Y11_1;
	end
end

// Sequential reset block triggered on positive edge of clock
always_ff @(posedge clk) begin
    if (rst) begin
        // Reset all Yij_* signals to 0
        // Yij_pos, Yij_neg, Yij_msb, and Yij_et_zero are assumed to be registers
        
        // Iterate over rows (i = 1 to 8) and columns (j = 2 to 8)
        // For each (i,j), reset all 4 components
        Y12_pos <= 0; Y12_neg <= 0; Y12_msb <= 0; Y12_et_zero <= 0; 
        Y13_pos <= 0; Y13_neg <= 0; Y13_msb <= 0; Y13_et_zero <= 0;
        Y14_pos <= 0; Y14_neg <= 0; Y14_msb <= 0; Y14_et_zero <= 0; 
        Y15_pos <= 0; Y15_neg <= 0; Y15_msb <= 0; Y15_et_zero <= 0;
        Y16_pos <= 0; Y16_neg <= 0; Y16_msb <= 0; Y16_et_zero <= 0; 
        Y17_pos <= 0; Y17_neg <= 0; Y17_msb <= 0; Y17_et_zero <= 0;
        Y18_pos <= 0; Y18_neg <= 0; Y18_msb <= 0; Y18_et_zero <= 0; 
        Y21_pos <= 0; Y21_neg <= 0; Y21_msb <= 0; Y21_et_zero <= 0;
        Y22_pos <= 0; Y22_neg <= 0; Y22_msb <= 0; Y22_et_zero <= 0; 
        Y23_pos <= 0; Y23_neg <= 0; Y23_msb <= 0; Y23_et_zero <= 0;
        Y24_pos <= 0; Y24_neg <= 0; Y24_msb <= 0; Y24_et_zero <= 0; 
        Y25_pos <= 0; Y25_neg <= 0; Y25_msb <= 0; Y25_et_zero <= 0;
        Y26_pos <= 0; Y26_neg <= 0; Y26_msb <= 0; Y26_et_zero <= 0; 
        Y27_pos <= 0; Y27_neg <= 0; Y27_msb <= 0; Y27_et_zero <= 0;
        Y28_pos <= 0; Y28_neg <= 0; Y28_msb <= 0; Y28_et_zero <= 0; 
        Y31_pos <= 0; Y31_neg <= 0; Y31_msb <= 0; Y31_et_zero <= 0;  
        Y32_pos <= 0; Y32_neg <= 0; Y32_msb <= 0; Y32_et_zero <= 0; 
        Y33_pos <= 0; Y33_neg <= 0; Y33_msb <= 0; Y33_et_zero <= 0;
        Y34_pos <= 0; Y34_neg <= 0; Y34_msb <= 0; Y34_et_zero <= 0; 
        Y35_pos <= 0; Y35_neg <= 0; Y35_msb <= 0; Y35_et_zero <= 0;
        Y36_pos <= 0; Y36_neg <= 0; Y36_msb <= 0; Y36_et_zero <= 0; 
        Y37_pos <= 0; Y37_neg <= 0; Y37_msb <= 0; Y37_et_zero <= 0;
        Y38_pos <= 0; Y38_neg <= 0; Y38_msb <= 0; Y38_et_zero <= 0;
        Y41_pos <= 0; Y41_neg <= 0; Y41_msb <= 0; Y41_et_zero <= 0;  
        Y42_pos <= 0; Y42_neg <= 0; Y42_msb <= 0; Y42_et_zero <= 0; 
        Y43_pos <= 0; Y43_neg <= 0; Y43_msb <= 0; Y43_et_zero <= 0;
        Y44_pos <= 0; Y44_neg <= 0; Y44_msb <= 0; Y44_et_zero <= 0; 
        Y45_pos <= 0; Y45_neg <= 0; Y45_msb <= 0; Y45_et_zero <= 0;
        Y46_pos <= 0; Y46_neg <= 0; Y46_msb <= 0; Y46_et_zero <= 0; 
        Y47_pos <= 0; Y47_neg <= 0; Y47_msb <= 0; Y47_et_zero <= 0;
        Y48_pos <= 0; Y48_neg <= 0; Y48_msb <= 0; Y48_et_zero <= 0;
        Y51_pos <= 0; Y51_neg <= 0; Y51_msb <= 0; Y51_et_zero <= 0;  
        Y52_pos <= 0; Y52_neg <= 0; Y52_msb <= 0; Y52_et_zero <= 0; 
        Y53_pos <= 0; Y53_neg <= 0; Y53_msb <= 0; Y53_et_zero <= 0;
        Y54_pos <= 0; Y54_neg <= 0; Y54_msb <= 0; Y54_et_zero <= 0; 
        Y55_pos <= 0; Y55_neg <= 0; Y55_msb <= 0; Y55_et_zero <= 0;
        Y56_pos <= 0; Y56_neg <= 0; Y56_msb <= 0; Y56_et_zero <= 0; 
        Y57_pos <= 0; Y57_neg <= 0; Y57_msb <= 0; Y57_et_zero <= 0;
        Y58_pos <= 0; Y58_neg <= 0; Y58_msb <= 0; Y58_et_zero <= 0;
        Y61_pos <= 0; Y61_neg <= 0; Y61_msb <= 0; Y61_et_zero <= 0;  
        Y62_pos <= 0; Y62_neg <= 0; Y62_msb <= 0; Y62_et_zero <= 0; 
        Y63_pos <= 0; Y63_neg <= 0; Y63_msb <= 0; Y63_et_zero <= 0;
        Y64_pos <= 0; Y64_neg <= 0; Y64_msb <= 0; Y64_et_zero <= 0; 
        Y65_pos <= 0; Y65_neg <= 0; Y65_msb <= 0; Y65_et_zero <= 0;
        Y66_pos <= 0; Y66_neg <= 0; Y66_msb <= 0; Y66_et_zero <= 0; 
        Y67_pos <= 0; Y67_neg <= 0; Y67_msb <= 0; Y67_et_zero <= 0;
        Y68_pos <= 0; Y68_neg <= 0; Y68_msb <= 0; Y68_et_zero <= 0;
        Y71_pos <= 0; Y71_neg <= 0; Y71_msb <= 0; Y71_et_zero <= 0;  
        Y72_pos <= 0; Y72_neg <= 0; Y72_msb <= 0; Y72_et_zero <= 0; 
        Y73_pos <= 0; Y73_neg <= 0; Y73_msb <= 0; Y73_et_zero <= 0;
        Y74_pos <= 0; Y74_neg <= 0; Y74_msb <= 0; Y74_et_zero <= 0; 
        Y75_pos <= 0; Y75_neg <= 0; Y75_msb <= 0; Y75_et_zero <= 0;
        Y76_pos <= 0; Y76_neg <= 0; Y76_msb <= 0; Y76_et_zero <= 0; 
        Y77_pos <= 0; Y77_neg <= 0; Y77_msb <= 0; Y77_et_zero <= 0;
        Y78_pos <= 0; Y78_neg <= 0; Y78_msb <= 0; Y78_et_zero <= 0;
        Y81_pos <= 0; Y81_neg <= 0; Y81_msb <= 0; Y81_et_zero <= 0;  
        Y82_pos <= 0; Y82_neg <= 0; Y82_msb <= 0; Y82_et_zero <= 0; 
        Y83_pos <= 0; Y83_neg <= 0; Y83_msb <= 0; Y83_et_zero <= 0;
        Y84_pos <= 0; Y84_neg <= 0; Y84_msb <= 0; Y84_et_zero <= 0; 
        Y85_pos <= 0; Y85_neg <= 0; Y85_msb <= 0; Y85_et_zero <= 0;
        Y86_pos <= 0; Y86_neg <= 0; Y86_msb <= 0; Y86_et_zero <= 0; 
        Y87_pos <= 0; Y87_neg <= 0; Y87_msb <= 0; Y87_et_zero <= 0;
        Y88_pos <= 0; Y88_neg <= 0; Y88_msb <= 0; Y88_et_zero <= 0; 
    end
// Perform operations when 'enable' is high
// Each Yxx_pos stores the original value
// Each Yxx_neg stores value minus one
// Each Yxx_msb stores the MSB (bit 10) of the value
// Each Yxx_et_zero is high if the value is zero

else if (enable) begin
    Y12_pos <= Y12;	
    Y12_neg <= Y12 - 1;
    Y12_msb <= Y12[10];
    Y12_et_zero <= !(|Y12);

    Y13_pos <= Y13;	
    Y13_neg <= Y13 - 1;
    Y13_msb <= Y13[10];
    Y13_et_zero <= !(|Y13);

    Y14_pos <= Y14;	
    Y14_neg <= Y14 - 1;
    Y14_msb <= Y14[10];
    Y14_et_zero <= !(|Y14);

    Y15_pos <= Y15;	
    Y15_neg <= Y15 - 1;
    Y15_msb <= Y15[10];
    Y15_et_zero <= !(|Y15);

    Y16_pos <= Y16;	
    Y16_neg <= Y16 - 1;
    Y16_msb <= Y16[10];
    Y16_et_zero <= !(|Y16);

    Y17_pos <= Y17;	
    Y17_neg <= Y17 - 1;
    Y17_msb <= Y17[10];
    Y17_et_zero <= !(|Y17);

    Y18_pos <= Y18;	
    Y18_neg <= Y18 - 1;
    Y18_msb <= Y18[10];
    Y18_et_zero <= !(|Y18);

    Y21_pos <= Y21;	
    Y21_neg <= Y21 - 1;
    Y21_msb <= Y21[10];
    Y21_et_zero <= !(|Y21);

    Y22_pos <= Y22;	
    Y22_neg <= Y22 - 1;
    Y22_msb <= Y22[10];
    Y22_et_zero <= !(|Y22);

    Y23_pos <= Y23;	
    Y23_neg <= Y23 - 1;
    Y23_msb <= Y23[10];
    Y23_et_zero <= !(|Y23);

    Y24_pos <= Y24;	
    Y24_neg <= Y24 - 1;
    Y24_msb <= Y24[10];
    Y24_et_zero <= !(|Y24);

    Y25_pos <= Y25;	
    Y25_neg <= Y25 - 1;
    Y25_msb <= Y25[10];
    Y25_et_zero <= !(|Y25);

Y25_et_zero <= !(|Y25);   // Check if Y25 is zero
        Y26_pos <= Y26;           Y26_neg <= Y26 - 1;           Y26_msb <= Y26[10];           Y26_et_zero <= !(|Y26);
        Y27_pos <= Y27;           Y27_neg <= Y27 - 1;           Y27_msb <= Y27[10];           Y27_et_zero <= !(|Y27);
        Y28_pos <= Y28;           Y28_neg <= Y28 - 1;           Y28_msb <= Y28[10];           Y28_et_zero <= !(|Y28);
        Y31_pos <= Y31;           Y31_neg <= Y31 - 1;           Y31_msb <= Y31[10];           Y31_et_zero <= !(|Y31);
        Y32_pos <= Y32;           Y32_neg <= Y32 - 1;           Y32_msb <= Y32[10];           Y32_et_zero <= !(|Y32);
        Y33_pos <= Y33;           Y33_neg <= Y33 - 1;           Y33_msb <= Y33[10];           Y33_et_zero <= !(|Y33);
        Y34_pos <= Y34;           Y34_neg <= Y34 - 1;           Y34_msb <= Y34[10];           Y34_et_zero <= !(|Y34);
        Y35_pos <= Y35;           Y35_neg <= Y35 - 1;           Y35_msb <= Y35[10];           Y35_et_zero <= !(|Y35);
        Y36_pos <= Y36;           Y36_neg <= Y36 - 1;           Y36_msb <= Y36[10];           Y36_et_zero <= !(|Y36);
        Y37_pos <= Y37;           Y37_neg <= Y37 - 1;           Y37_msb <= Y37[10];           Y37_et_zero <= !(|Y37);
        Y38_pos <= Y38;           Y38_neg <= Y38 - 1;           Y38_msb <= Y38[10];           Y38_et_zero <= !(|Y38);
        Y41_pos <= Y41;           Y41_neg <= Y41 - 1;           Y41_msb <= Y41[10];           Y41_et_zero <= !(|Y41);
        Y42_pos <= Y42;
        
        		Y71_et_zero <= !(|Y71);
		Y72_pos <= Y72;	   
		Y72_neg <= Y72 - 1;
		Y72_msb <= Y72[10];
		Y72_et_zero <= !(|Y72);
		Y73_pos <= Y73;	   
		Y73_neg <= Y73 - 1;
		Y73_msb <= Y73[10];
		Y73_et_zero <= !(|Y73);
		Y74_pos <= Y74;	   
		Y74_neg <= Y74 - 1;
		Y74_msb <= Y74[10];
		Y74_et_zero <= !(|Y74);
		Y75_pos <= Y75;	   
		Y75_neg <= Y75 - 1;
		Y75_msb <= Y75[10];
		Y75_et_zero <= !(|Y75);
		Y76_pos <= Y76;	   
		Y76_neg <= Y76 - 1;
		Y76_msb <= Y76[10];
		Y76_et_zero <= !(|Y76);
		Y77_pos <= Y77;	   
		Y77_neg <= Y77 - 1;
		Y77_msb <= Y77[10];
		Y77_et_zero <= !(|Y77);
		Y78_pos <= Y78;	   
		Y78_neg <= Y78 - 1;
		Y78_msb <= Y78[10];
		Y78_et_zero <= !(|Y78);
		Y81_pos <= Y81;	   
		Y81_neg <= Y81 - 1;
		Y81_msb <= Y81[10];
		Y81_et_zero <= !(|Y81);
		Y82_pos <= Y82;	   
		Y82_neg <= Y82 - 1;
		Y82_msb <= Y82[10];
		Y82_et_zero <= !(|Y82);
		Y83_pos <= Y83;	   
		Y83_neg <= Y83 - 1;
		Y83_msb <= Y83[10];
		Y83_et_zero <= !(|Y83);
		Y84_pos <= Y84;	   
		Y84_neg <= Y84 - 1;
		Y84_msb <= Y84[10];
		Y84_et_zero <= !(|Y84);
		Y85_pos <= Y85;	   
		Y85_neg <= Y85 - 1;
		Y85_msb <= Y85[10];
		Y85_et_zero <= !(|Y85);
		Y86_pos <= Y86;	   
		Y86_neg <= Y86 - 1;
		Y86_msb <= Y86[10];
		Y86_et_zero <= !(|Y86);
		Y87_pos <= Y87;	   
		Y87_neg <= Y87 - 1;
		Y87_msb <= Y87[10];
		Y87_et_zero <= !(|Y87);
		Y88_pos <= Y88;	   
		Y88_neg <= Y88 - 1;
		Y88_msb <= Y88[10];
		Y88_et_zero <= !(|Y88);
		end
        
   else if (enable_module) begin 
		Y12_pos <= Y21_pos;	   
		Y12_neg <= Y21_neg;
		Y12_msb <= Y21_msb;
		Y12_et_zero <= Y21_et_zero;
		Y21_pos <= Y31_pos;	   
		Y21_neg <= Y31_neg;
		Y21_msb <= Y31_msb;
		Y21_et_zero <= Y31_et_zero;
		Y31_pos <= Y22_pos;	   
		Y31_neg <= Y22_neg;
		Y31_msb <= Y22_msb;
		Y31_et_zero <= Y22_et_zero;
		Y22_pos <= Y13_pos;	   
		Y22_neg <= Y13_neg;
		Y22_msb <= Y13_msb;
		Y22_et_zero <= Y13_et_zero;
		Y13_pos <= Y14_pos;	   
		Y13_neg <= Y14_neg;
		Y13_msb <= Y14_msb;
		Y13_et_zero <= Y14_et_zero;
		Y14_pos <= Y23_pos;	   
		Y14_neg <= Y23_neg;
		Y14_msb <= Y23_msb;
		Y14_et_zero <= Y23_et_zero;
		Y23_pos <= Y32_pos;	   
		Y23_neg <= Y32_neg;
		Y23_msb <= Y32_msb;
		Y23_et_zero <= Y32_et_zero;
		Y32_pos <= Y41_pos;	   
		Y32_neg <= Y41_neg;
		Y32_msb <= Y41_msb;
		Y32_et_zero <= Y41_et_zero;
		Y41_pos <= Y51_pos;	   
		Y41_neg <= Y51_neg;
		Y41_msb <= Y51_msb;
		Y41_et_zero <= Y51_et_zero;
		Y51_pos <= Y42_pos;	   
		Y51_neg <= Y42_neg;
		Y51_msb <= Y42_msb;
		Y51_et_zero <= Y42_et_zero;
		Y42_pos <= Y33_pos;	   
		Y42_neg <= Y33_neg;
		Y42_msb <= Y33_msb;
		Y42_et_zero <= Y33_et_zero;
		Y33_pos <= Y24_pos;	   
		Y33_neg <= Y24_neg;
		Y33_msb <= Y24_msb;
		Y33_et_zero <= Y24_et_zero;
		Y24_pos <= Y15_pos;	   
		Y24_neg <= Y15_neg;
		Y24_msb <= Y15_msb;
		Y24_et_zero <= Y15_et_zero;
		Y15_pos <= Y16_pos;	   
		Y15_neg <= Y16_neg;
		Y15_msb <= Y16_msb;
		Y15_et_zero <= Y16_et_zero;
		Y16_pos <= Y25_pos;	   
		Y16_neg <= Y25_neg;
		Y16_msb <= Y25_msb;
		Y16_et_zero <= Y25_et_zero;
		Y25_pos <= Y34_pos;	   
		Y25_neg <= Y34_neg;
		Y25_msb <= Y34_msb;
		Y25_et_zero <= Y34_et_zero;
		Y34_pos <= Y43_pos;	   
		Y34_neg <= Y43_neg;
		Y34_msb <= Y43_msb;
		Y34_et_zero <= Y43_et_zero;
		Y43_pos <= Y52_pos;	   
		Y43_neg <= Y52_neg;
		Y43_msb <= Y52_msb;
		Y43_et_zero <= Y52_et_zero;
		Y52_pos <= Y61_pos;	   
		Y52_neg <= Y61_neg;
		Y52_msb <= Y61_msb;
		Y52_et_zero <= Y61_et_zero;
		Y61_pos <= Y71_pos;	   
		Y61_neg <= Y71_neg;
		Y61_msb <= Y71_msb;
		Y61_et_zero <= Y71_et_zero;
		Y71_pos <= Y62_pos;	   
		Y71_neg <= Y62_neg;
		Y71_msb <= Y62_msb;
		Y71_et_zero <= Y62_et_zero;
		Y62_pos <= Y53_pos;	   
		Y62_neg <= Y53_neg;
		Y62_msb <= Y53_msb;
		Y62_et_zero <= Y53_et_zero;
		Y53_pos <= Y44_pos;	   
		Y53_neg <= Y44_neg;
		Y53_msb <= Y44_msb;
		Y53_et_zero <= Y44_et_zero;
		Y44_pos <= Y35_pos;	   
		Y44_neg <= Y35_neg;
		Y44_msb <= Y35_msb;
		Y44_et_zero <= Y35_et_zero;
		Y35_pos <= Y26_pos;	   
		Y35_neg <= Y26_neg;
		Y35_msb <= Y26_msb;
		Y35_et_zero <= Y26_et_zero;
		Y26_pos <= Y17_pos;	   
		Y26_neg <= Y17_neg;
		Y26_msb <= Y17_msb;
		Y26_et_zero <= Y17_et_zero;
		Y17_pos <= Y18_pos;	   
		Y17_neg <= Y18_neg;
		Y17_msb <= Y18_msb;
		Y17_et_zero <= Y18_et_zero;
		Y18_pos <= Y27_pos;	   
		Y18_neg <= Y27_neg;
		Y18_msb <= Y27_msb;
		Y18_et_zero <= Y27_et_zero;
		Y27_pos <= Y36_pos;	   
		Y27_neg <= Y36_neg;
		Y27_msb <= Y36_msb;
		Y27_et_zero <= Y36_et_zero;
		Y36_pos <= Y45_pos;	   
		Y36_neg <= Y45_neg;
		Y36_msb <= Y45_msb;
		Y36_et_zero <= Y45_et_zero;
		Y45_pos <= Y54_pos;	   
		Y45_neg <= Y54_neg;
		Y45_msb <= Y54_msb;
		Y45_et_zero <= Y54_et_zero;
		Y54_pos <= Y63_pos;	   
		Y54_neg <= Y63_neg;
		Y54_msb <= Y63_msb;
		Y54_et_zero <= Y63_et_zero;
		Y63_pos <= Y72_pos;	   
		Y63_neg <= Y72_neg;
		Y63_msb <= Y72_msb;
		Y63_et_zero <= Y72_et_zero;
		Y72_pos <= Y81_pos;	   
		Y72_neg <= Y81_neg;
		Y72_msb <= Y81_msb;
		Y72_et_zero <= Y81_et_zero;
		Y81_pos <= Y82_pos;	   
		Y81_neg <= Y82_neg;
		Y81_msb <= Y82_msb;
		Y81_et_zero <= Y82_et_zero;
		Y82_pos <= Y73_pos;	   
		Y82_neg <= Y73_neg;
		Y82_msb <= Y73_msb;
		Y82_et_zero <= Y73_et_zero;
		Y73_pos <= Y64_pos;	   
		Y73_neg <= Y64_neg;
		Y73_msb <= Y64_msb;
		Y73_et_zero <= Y64_et_zero;
		Y64_pos <= Y55_pos;	   
		Y64_neg <= Y55_neg;
		Y64_msb <= Y55_msb;
		Y64_et_zero <= Y55_et_zero;
		Y55_pos <= Y46_pos;	   
		Y55_neg <= Y46_neg;
		Y55_msb <= Y46_msb;
		Y55_et_zero <= Y46_et_zero;
		Y46_pos <= Y37_pos;	   
		Y46_neg <= Y37_neg;
		Y46_msb <= Y37_msb;
		Y46_et_zero <= Y37_et_zero;
		Y37_pos <= Y28_pos;	   
		Y37_neg <= Y28_neg;
		Y37_msb <= Y28_msb;
		Y37_et_zero <= Y28_et_zero;
		Y28_pos <= Y38_pos;	   
		Y28_neg <= Y38_neg;
		Y28_msb <= Y38_msb;
		Y28_et_zero <= Y38_et_zero;
		Y38_pos <= Y47_pos;	   
		Y38_neg <= Y47_neg;
		Y38_msb <= Y47_msb;
		Y38_et_zero <= Y47_et_zero;
		Y47_pos <= Y56_pos;	   
		Y47_neg <= Y56_neg;
		Y47_msb <= Y56_msb;
		Y47_et_zero <= Y56_et_zero;
		Y56_pos <= Y65_pos;	   
		Y56_neg <= Y65_neg;
		Y56_msb <= Y65_msb;
		Y56_et_zero <= Y65_et_zero;
		Y65_pos <= Y74_pos;	   
		Y65_neg <= Y74_neg;
		Y65_msb <= Y74_msb;
		Y65_et_zero <= Y74_et_zero;
		Y74_pos <= Y83_pos;	   
		Y74_neg <= Y83_neg;
		Y74_msb <= Y83_msb;
		Y74_et_zero <= Y83_et_zero;
		Y83_pos <= Y84_pos;	   
		Y83_neg <= Y84_neg;
		Y83_msb <= Y84_msb;
		Y83_et_zero <= Y84_et_zero;
		Y84_pos <= Y75_pos;	   
		Y84_neg <= Y75_neg;
		Y84_msb <= Y75_msb;
		Y84_et_zero <= Y75_et_zero;
		Y75_pos <= Y66_pos;	   
		Y75_neg <= Y66_neg;
		Y75_msb <= Y66_msb;
		Y75_et_zero <= Y66_et_zero;
		Y66_pos <= Y57_pos;	   
		Y66_neg <= Y57_neg;
		Y66_msb <= Y57_msb;
		Y66_et_zero <= Y57_et_zero;
		Y57_pos <= Y48_pos;	   
		Y57_neg <= Y48_neg;
		Y57_msb <= Y48_msb;
		Y57_et_zero <= Y48_et_zero;
		Y48_pos <= Y58_pos;	   
		Y48_neg <= Y58_neg;
		Y48_msb <= Y58_msb;
		Y48_et_zero <= Y58_et_zero;
		Y58_pos <= Y67_pos;	   
		Y58_neg <= Y67_neg;
		Y58_msb <= Y67_msb;
		Y58_et_zero <= Y67_et_zero;
		Y67_pos <= Y76_pos;	   
		Y67_neg <= Y76_neg;
		Y67_msb <= Y76_msb;
		Y67_et_zero <= Y76_et_zero;
		Y76_pos <= Y85_pos;	   
		Y76_neg <= Y85_neg;
		Y76_msb <= Y85_msb;
		Y76_et_zero <= Y85_et_zero;
		Y85_pos <= Y86_pos;	   
		Y85_neg <= Y86_neg;
		Y85_msb <= Y86_msb;
		Y85_et_zero <= Y86_et_zero;
		Y86_pos <= Y77_pos;	   
		Y86_neg <= Y77_neg;
		Y86_msb <= Y77_msb;
		Y86_et_zero <= Y77_et_zero;
		Y77_pos <= Y68_pos;	   
		Y77_neg <= Y68_neg;
		Y77_msb <= Y68_msb;
		Y77_et_zero <= Y68_et_zero;
		Y68_pos <= Y78_pos;	   
		Y68_neg <= Y78_neg;
		Y68_msb <= Y78_msb;
		Y68_et_zero <= Y78_et_zero;
		Y78_pos <= Y87_pos;	   
		Y78_neg <= Y87_neg;
		Y78_msb <= Y87_msb;
		Y78_et_zero <= Y87_et_zero;
		Y87_pos <= Y88_pos;	   
		Y87_neg <= Y88_neg;
		Y87_msb <= Y88_msb;
		Y87_et_zero <= Y88_et_zero;
		Y88_pos <= 0;	   
		Y88_neg <= 0;
		Y88_msb <= 0;
		Y88_et_zero <= 1;
		end
end	 
// This block computes the difference of Y11 (after sign extension) from the previous value.
// It also performs sign extension of Y11 to 12 bits for further processing.
always_ff @(posedge clk) begin
    if (rst) begin
        Y11_diff <= 0;            // Reset difference
        Y11_1    <= 0;            // Reset sign-extended Y11
    end else if (enable) begin
        // Sign extend Y11 to 12 bits and compute difference from Y11_previous
        Y11_diff <= {Y11[10], Y11} - Y11_previous;

        // Sign extension based on MSB (bit 10) of Y11
        Y11_1 <= Y11[10] ? {1'b1, Y11} : {1'b0, Y11};
    end
end

// This block calculates the number of bits needed to represent the positive value of Y11_1.
// It scans from MSB to LSB and assigns Y11_bits_pos accordingly.
always_ff @(posedge clk) begin
    if (rst) begin
        Y11_bits_pos <= 0;        // Reset bit count
    end else if (Y11_1_pos[10]) begin
        Y11_bits_pos <= 11;
    end else if (Y11_1_pos[9]) begin
        Y11_bits_pos <= 10;
    end else if (Y11_1_pos[8]) begin
        Y11_bits_pos <= 9;
    end else if (Y11_1_pos[7]) begin
        Y11_bits_pos <= 8;
    end else if (Y11_1_pos[6]) begin
        Y11_bits_pos <= 7;
    end else if (Y11_1_pos[5]) begin
        Y11_bits_pos <= 6;
    end else if (Y11_1_pos[4]) begin
        Y11_bits_pos <= 5;
    end else if (Y11_1_pos[3]) begin
        Y11_bits_pos <= 4;
    end else if (Y11_1_pos[2]) begin
        Y11_bits_pos <= 3;
    end else if (Y11_1_pos[1]) begin
        Y11_bits_pos <= 2;
    end else if (Y11_1_pos[0]) begin
        Y11_bits_pos <= 1;
    end else begin
        Y11_bits_pos <= 0;
    end
end


 		// This block calculates the number of bits required to represent the negative value of Y11_1
// It checks from MSB to LSB for the first 0 bit, which determines the bit-width
// Output: Y11_bits_neg

always_ff @(posedge clk) begin
    if (rst) begin
        Y11_bits_neg <= 0;  // Reset on rst
    end else if (Y11_1_neg[10] == 0) begin
        Y11_bits_neg <= 11;
    end else if (Y11_1_neg[9] == 0) begin
        Y11_bits_neg <= 10;
    end else if (Y11_1_neg[8] == 0) begin
        Y11_bits_neg <= 9;
    end else if (Y11_1_neg[7] == 0) begin
        Y11_bits_neg <= 8;
    end else if (Y11_1_neg[6] == 0) begin
        Y11_bits_neg <= 7;
    end else if (Y11_1_neg[5] == 0) begin
        Y11_bits_neg <= 6;
    end else if (Y11_1_neg[4] == 0) begin
        Y11_bits_neg <= 5;
    end else if (Y11_1_neg[3] == 0) begin
        Y11_bits_neg <= 4;
    end else if (Y11_1_neg[2] == 0) begin
        Y11_bits_neg <= 3;
    end else if (Y11_1_neg[1] == 0) begin
        Y11_bits_neg <= 2;
    end else if (Y11_1_neg[0] == 0) begin
        Y11_bits_neg <= 1;
    end else begin
        Y11_bits_neg <= 0;  // If no 0 found, default to 0
    end
end

// Calculate number of bits required to represent Y12_pos (positive value)
// Scan from MSB to LSB. Assign highest index with bit '1' as the bit-width.
always_ff @(posedge clk) begin
    if (rst)
        Y12_bits_pos <= 0;
    else if (Y12_pos[9])
        Y12_bits_pos <= 10;
    else if (Y12_pos[8])
        Y12_bits_pos <= 9;
    else if (Y12_pos[7])
        Y12_bits_pos <= 8;
    else if (Y12_pos[6])
        Y12_bits_pos <= 7;
    else if (Y12_pos[5])
        Y12_bits_pos <= 6;
    else if (Y12_pos[4])
        Y12_bits_pos <= 5;
    else if (Y12_pos[3])
        Y12_bits_pos <= 4;
    else if (Y12_pos[2])
        Y12_bits_pos <= 3;
    else if (Y12_pos[1])
        Y12_bits_pos <= 2;
    else if (Y12_pos[0])
        Y12_bits_pos <= 1;
    else
        Y12_bits_pos <= 0;
end

// Calculate number of bits required to represent Y12_neg (negative value)
// Scan from MSB to LSB. Assign highest index with bit '0' as the bit-width.
always_ff @(posedge clk) begin
    if (rst)
        Y12_bits_neg <= 0;
    else if (!Y12_neg[9])
        Y12_bits_neg <= 10;
    else if (!Y12_neg[8])
        Y12_bits_neg <= 9;
    else if (!Y12_neg[7])
        Y12_bits_neg <= 8;
    else if (!Y12_neg[6])
        Y12_bits_neg <= 7;
    else if (!Y12_neg[5])
        Y12_bits_neg <= 6;
    else if (!Y12_neg[4])
        Y12_bits_neg <= 5;
    else if (!Y12_neg[3])
        Y12_bits_neg <= 4;
    else if (!Y12_neg[2])
        Y12_bits_neg <= 3;
    else if (!Y12_neg[1])
        Y12_bits_neg <= 2;
    else if (!Y12_neg[0])
        Y12_bits_neg <= 1;
    else
        Y12_bits_neg <= 0;
end

// Latches the main 'enable' signal into 'enable_module' after 1 clock cycle
always_ff @(posedge clk) begin
    if (rst) begin
        enable_module <= 0;  // Reset the module enable
    end else if (enable) begin
        enable_module <= 1;  // Latch enable on rising edge
    end
end	 

// Controls 'enable_latch_7': gets set when 'enable_6' is high,
// and cleared when block_counter reaches 68 or when reset
always_ff @(posedge clk) begin
    if (rst) begin
        enable_latch_7 <= 0;  // Reset the latch
    end else if (block_counter == 68) begin
        enable_latch_7 <= 0;  // Disable latch when counter reaches 68
    end else if (enable_6) begin
        enable_latch_7 <= 1;  // Enable latch when enable_6 is active
    end
end	

// Controls 'enable_latch_8': gets set when 'enable_7' is high
always_ff @(posedge clk) begin
    if (rst) begin
        enable_latch_8 <= 0;  // Reset the latch
    end else if (enable_7) begin
        enable_latch_8 <= 1;  // Enable latch when enable_7 is active
    end
end	

// Shifts the enable signal through 13 delay registers (enable_1 to enable_13)
// This creates a delayed version of the original enable signal over 13 clock cycles
always_ff @(posedge clk) begin
    if (rst) begin
        enable_1  <= 0; enable_2  <= 0; enable_3  <= 0;
        enable_4  <= 0; enable_5  <= 0; enable_6  <= 0;
        enable_7  <= 0; enable_8  <= 0; enable_9  <= 0;
        enable_10 <= 0; enable_11 <= 0; enable_12 <= 0;
        enable_13 <= 0;
    end else begin
        enable_1  <= enable;     enable_2  <= enable_1;  enable_3  <= enable_2;
        enable_4  <= enable_3;   enable_5  <= enable_4;  enable_6  <= enable_5;
        enable_7  <= enable_6;   enable_8  <= enable_7;  enable_9  <= enable_8;
        enable_10 <= enable_9;   enable_11 <= enable_10; enable_12 <= enable_11;
        enable_13 <= enable_12;
    end
end


always @(posedge clk)
begin
	Y_DC_code_length[0] <= 2;
Y_DC_code_length[1] <= 2;
Y_DC_code_length[2] <= 2;
Y_DC_code_length[3] <= 3;
Y_DC_code_length[4] <= 4;
Y_DC_code_length[5] <= 5;
Y_DC_code_length[6] <= 6;
Y_DC_code_length[7] <= 7;
Y_DC_code_length[8] <= 8;
Y_DC_code_length[9] <= 9;
Y_DC_code_length[10] <= 10;
Y_DC_code_length[11] <= 11;
Y_DC[0] <= 11'b00000000000;
Y_DC[1] <= 11'b01000000000;
Y_DC[2] <= 11'b10000000000;
Y_DC[3] <= 11'b11000000000;
Y_DC[4] <= 11'b11100000000;
Y_DC[5] <= 11'b11110000000;
Y_DC[6] <= 11'b11111000000;
Y_DC[7] <= 11'b11111100000;
Y_DC[8] <= 11'b11111110000;
Y_DC[9] <= 11'b11111111000;
Y_DC[10] <= 11'b11111111100;
Y_DC[11] <= 11'b11111111110;
Y_AC_code_length[0] <= 2;
Y_AC_code_length[1] <= 2;
Y_AC_code_length[2] <= 3;
Y_AC_code_length[3] <= 4;
Y_AC_code_length[4] <= 4;
Y_AC_code_length[5] <= 4;
Y_AC_code_length[6] <= 5;
Y_AC_code_length[7] <= 5;
Y_AC_code_length[8] <= 5;
Y_AC_code_length[9] <= 6;
Y_AC_code_length[10] <= 6;
Y_AC_code_length[11] <= 7;
Y_AC_code_length[12] <= 7;
Y_AC_code_length[13] <= 7;
Y_AC_code_length[14] <= 7;
Y_AC_code_length[15] <= 8;
Y_AC_code_length[16] <= 8;
Y_AC_code_length[17] <= 8;
Y_AC_code_length[18] <= 9;
Y_AC_code_length[19] <= 9;
Y_AC_code_length[20] <= 9;
Y_AC_code_length[21] <= 9;
Y_AC_code_length[22] <= 9;
Y_AC_code_length[23] <= 10;
Y_AC_code_length[24] <= 10;
Y_AC_code_length[25] <= 10;
Y_AC_code_length[26] <= 10;
Y_AC_code_length[27] <= 10;
Y_AC_code_length[28] <= 11;
Y_AC_code_length[29] <= 11;
Y_AC_code_length[30] <= 11;
Y_AC_code_length[31] <= 11;
Y_AC_code_length[32] <= 12;
Y_AC_code_length[33] <= 12;
Y_AC_code_length[34] <= 12;
Y_AC_code_length[35] <= 12;
Y_AC_code_length[36] <= 15;
Y_AC_code_length[37] <= 16;
Y_AC_code_length[38] <= 16;
Y_AC_code_length[39] <= 16;
Y_AC_code_length[40] <= 16;
Y_AC_code_length[41] <= 16;
Y_AC_code_length[42] <= 16;
Y_AC_code_length[43] <= 16;
Y_AC_code_length[44] <= 16;
Y_AC_code_length[45] <= 16;
Y_AC_code_length[46] <= 16;
Y_AC_code_length[47] <= 16;
Y_AC_code_length[48] <= 16;
Y_AC_code_length[49] <= 16;
Y_AC_code_length[50] <= 16;
Y_AC_code_length[51] <= 16;
Y_AC_code_length[52] <= 16;
Y_AC_code_length[53] <= 16;
Y_AC_code_length[54] <= 16;
Y_AC_code_length[55] <= 16;
Y_AC_code_length[56] <= 16;
Y_AC_code_length[57] <= 16;
Y_AC_code_length[58] <= 16;
Y_AC_code_length[59] <= 16;
Y_AC_code_length[60] <= 16;
Y_AC_code_length[61] <= 16;
Y_AC_code_length[62] <= 16;
Y_AC_code_length[63] <= 16;
Y_AC_code_length[64] <= 16;
Y_AC_code_length[65] <= 16;
Y_AC_code_length[66] <= 16;
Y_AC_code_length[67] <= 16;
Y_AC_code_length[68] <= 16;
Y_AC_code_length[69] <= 16;
Y_AC_code_length[70] <= 16;
Y_AC_code_length[71] <= 16;
Y_AC_code_length[72] <= 16;
Y_AC_code_length[73] <= 16;
Y_AC_code_length[74] <= 16;
Y_AC_code_length[75] <= 16;
Y_AC_code_length[76] <= 16;
Y_AC_code_length[77] <= 16;
Y_AC_code_length[78] <= 16;
Y_AC_code_length[79] <= 16;
Y_AC_code_length[80] <= 16;
Y_AC_code_length[81] <= 16;
Y_AC_code_length[82] <= 16;
Y_AC_code_length[83] <= 16;
Y_AC_code_length[84] <= 16;
Y_AC_code_length[85] <= 16;
Y_AC_code_length[86] <= 16;
Y_AC_code_length[87] <= 16;
Y_AC_code_length[88] <= 16;
Y_AC_code_length[89] <= 16;
Y_AC_code_length[90] <= 16;
Y_AC_code_length[91] <= 16;
Y_AC_code_length[92] <= 16;
Y_AC_code_length[93] <= 16;
Y_AC_code_length[94] <= 16;
Y_AC_code_length[95] <= 16;
Y_AC_code_length[96] <= 16;
Y_AC_code_length[97] <= 16;
Y_AC_code_length[98] <= 16;
Y_AC_code_length[99] <= 16;
Y_AC_code_length[100] <= 16;
Y_AC_code_length[101] <= 16;
Y_AC_code_length[102] <= 16;
Y_AC_code_length[103] <= 16;
Y_AC_code_length[104] <= 16;
Y_AC_code_length[105] <= 16;
Y_AC_code_length[106] <= 16;
Y_AC_code_length[107] <= 16;
Y_AC_code_length[108] <= 16;
Y_AC_code_length[109] <= 16;
Y_AC_code_length[110] <= 16;
Y_AC_code_length[111] <= 16;
Y_AC_code_length[112] <= 16;
Y_AC_code_length[113] <= 16;
Y_AC_code_length[114] <= 16;
Y_AC_code_length[115] <= 16;
Y_AC_code_length[116] <= 16;
Y_AC_code_length[117] <= 16;
Y_AC_code_length[118] <= 16;
Y_AC_code_length[119] <= 16;
Y_AC_code_length[120] <= 16;
Y_AC_code_length[121] <= 16;
Y_AC_code_length[122] <= 16;
Y_AC_code_length[123] <= 16;
Y_AC_code_length[124] <= 16;
Y_AC_code_length[125] <= 16;
Y_AC_code_length[126] <= 16;
Y_AC_code_length[127] <= 16;
Y_AC_code_length[128] <= 16;
Y_AC_code_length[129] <= 16;
Y_AC_code_length[130] <= 16;
Y_AC_code_length[131] <= 16;
Y_AC_code_length[132] <= 16;
Y_AC_code_length[133] <= 16;
Y_AC_code_length[134] <= 16;
Y_AC_code_length[135] <= 16;
Y_AC_code_length[136] <= 16;
Y_AC_code_length[137] <= 16;
Y_AC_code_length[138] <= 16;
Y_AC_code_length[139] <= 16;
Y_AC_code_length[140] <= 16;
Y_AC_code_length[141] <= 16;
Y_AC_code_length[142] <= 16;
Y_AC_code_length[143] <= 16;
Y_AC_code_length[144] <= 16;
Y_AC_code_length[145] <= 16;
Y_AC_code_length[146] <= 16;
Y_AC_code_length[147] <= 16;
Y_AC_code_length[148] <= 16;
Y_AC_code_length[149] <= 16;
Y_AC_code_length[150] <= 16;
Y_AC_code_length[151] <= 16;
Y_AC_code_length[152] <= 16;
Y_AC_code_length[153] <= 16;
Y_AC_code_length[154] <= 16;
Y_AC_code_length[155] <= 16;
Y_AC_code_length[156] <= 16;
Y_AC_code_length[157] <= 16;
Y_AC_code_length[158] <= 16;
Y_AC_code_length[159] <= 16;
Y_AC_code_length[160] <= 16;
Y_AC_code_length[161] <= 16;

Y_AC[0] <= 16'b0000000000000000;
Y_AC[1] <= 16'b0100000000000000;
Y_AC[2] <= 16'b1000000000000000;
Y_AC[3] <= 16'b1010000000000000;
Y_AC[4] <= 16'b1011000000000000;
Y_AC[5] <= 16'b1100000000000000;
Y_AC[6] <= 16'b1101000000000000;
Y_AC[7] <= 16'b1101100000000000;
Y_AC[8] <= 16'b1110000000000000;
Y_AC[9] <= 16'b1110100000000000;
Y_AC[10] <= 16'b1110110000000000;
Y_AC[11] <= 16'b1111000000000000;
Y_AC[12] <= 16'b1111001000000000;
Y_AC[13] <= 16'b1111010000000000;
Y_AC[14] <= 16'b1111011000000000;
Y_AC[15] <= 16'b1111100000000000;
Y_AC[16] <= 16'b1111100100000000;
Y_AC[17] <= 16'b1111101000000000;
Y_AC[18] <= 16'b1111101100000000;
Y_AC[19] <= 16'b1111101110000000;
Y_AC[20] <= 16'b1111110000000000;
Y_AC[21] <= 16'b1111110010000000;
Y_AC[22] <= 16'b1111110100000000;
Y_AC[23] <= 16'b1111110110000000;
Y_AC[24] <= 16'b1111110111000000;
Y_AC[25] <= 16'b1111111000000000;
Y_AC[26] <= 16'b1111111001000000;
Y_AC[27] <= 16'b1111111010000000;
Y_AC[28] <= 16'b1111111011000000;
Y_AC[29] <= 16'b1111111011100000;
Y_AC[30] <= 16'b1111111100000000;
Y_AC[31] <= 16'b1111111100100000;
Y_AC[32] <= 16'b1111111101000000;
Y_AC[33] <= 16'b1111111101010000;
Y_AC[34] <= 16'b1111111101100000;
Y_AC[35] <= 16'b1111111101110000;
Y_AC[36] <= 16'b1111111110000000;
Y_AC[37] <= 16'b1111111110000010;
Y_AC[38] <= 16'b1111111110000011;
Y_AC[39] <= 16'b1111111110000100;
Y_AC[40] <= 16'b1111111110000101;
Y_AC[41] <= 16'b1111111110000110;
Y_AC[42] <= 16'b1111111110000111;
Y_AC[43] <= 16'b1111111110001000;
Y_AC[44] <= 16'b1111111110001001;
Y_AC[45] <= 16'b1111111110001010;
Y_AC[46] <= 16'b1111111110001011;
Y_AC[47] <= 16'b1111111110001100;
Y_AC[48] <= 16'b1111111110001101;
Y_AC[49] <= 16'b1111111110001110;
Y_AC[50] <= 16'b1111111110001111;
Y_AC[51] <= 16'b1111111110010000;
Y_AC[52] <= 16'b1111111110010001;
Y_AC[53] <= 16'b1111111110010010;
Y_AC[54] <= 16'b1111111110010011;
Y_AC[55] <= 16'b1111111110010100;
Y_AC[56] <= 16'b1111111110010101;
Y_AC[57] <= 16'b1111111110010110;
Y_AC[58] <= 16'b1111111110010111;
Y_AC[59] <= 16'b1111111110011000;
Y_AC[60] <= 16'b1111111110011001;
Y_AC[61] <= 16'b1111111110011010;
Y_AC[62] <= 16'b1111111110011011;
Y_AC[63] <= 16'b1111111110011100;
Y_AC[64] <= 16'b1111111110011101;
Y_AC[65] <= 16'b1111111110011110;
Y_AC[66] <= 16'b1111111110011111;
Y_AC[67] <= 16'b1111111110100000;
Y_AC[68] <= 16'b1111111110100001;
Y_AC[69] <= 16'b1111111110100010;
Y_AC[70] <= 16'b1111111110100011;
Y_AC[71] <= 16'b1111111110100100;
Y_AC[72] <= 16'b1111111110100101;
Y_AC[73] <= 16'b1111111110100110;
Y_AC[74] <= 16'b1111111110100111;
Y_AC[75] <= 16'b1111111110101000;
Y_AC[76] <= 16'b1111111110101001;
Y_AC[77] <= 16'b1111111110101010;
Y_AC[78] <= 16'b1111111110101011;
Y_AC[79] <= 16'b1111111110101100;
Y_AC[80] <= 16'b1111111110101101;
Y_AC[81] <= 16'b1111111110101110;
Y_AC[82] <= 16'b1111111110101111;
Y_AC[83] <= 16'b1111111110110000;
Y_AC[84] <= 16'b1111111110110001;
Y_AC[85] <= 16'b1111111110110010;
Y_AC[86] <= 16'b1111111110110011;
Y_AC[87] <= 16'b1111111110110100;
Y_AC[88] <= 16'b1111111110110101;
Y_AC[89] <= 16'b1111111110110110;
Y_AC[90] <= 16'b1111111110110111;
Y_AC[91] <= 16'b1111111110111000;
Y_AC[92] <= 16'b1111111110111001;
Y_AC[93] <= 16'b1111111110111010;
Y_AC[94] <= 16'b1111111110111011;
Y_AC[95] <= 16'b1111111110111100;
Y_AC[96] <= 16'b1111111110111101;
Y_AC[97] <= 16'b1111111110111110;
Y_AC[98] <= 16'b1111111110111111;
Y_AC[99] <= 16'b1111111111000000;
Y_AC[100] <= 16'b1111111111000001;
Y_AC[101] <= 16'b1111111111000010;
Y_AC[102] <= 16'b1111111111000011;
Y_AC[103] <= 16'b1111111111000100;
Y_AC[104] <= 16'b1111111111000101;
Y_AC[105] <= 16'b1111111111000110;
Y_AC[106] <= 16'b1111111111000111;
Y_AC[107] <= 16'b1111111111001000;
Y_AC[108] <= 16'b1111111111001001;
Y_AC[109] <= 16'b1111111111001010;
Y_AC[110] <= 16'b1111111111001011;
Y_AC[111] <= 16'b1111111111001100;
Y_AC[112] <= 16'b1111111111001101;
Y_AC[113] <= 16'b1111111111001110;
Y_AC[114] <= 16'b1111111111001111;
Y_AC[115] <= 16'b1111111111010000;
Y_AC[116] <= 16'b1111111111010001;
Y_AC[117] <= 16'b1111111111010010;
Y_AC[118] <= 16'b1111111111010011;
Y_AC[119] <= 16'b1111111111010100;
Y_AC[120] <= 16'b1111111111010101;
Y_AC[121] <= 16'b1111111111010110;
Y_AC[122] <= 16'b1111111111010111;
Y_AC[123] <= 16'b1111111111011000;
Y_AC[124] <= 16'b1111111111011001;
Y_AC[125] <= 16'b1111111111011010;
Y_AC[126] <= 16'b1111111111011011;
Y_AC[127] <= 16'b1111111111011100;
Y_AC[128] <= 16'b1111111111011101;
Y_AC[129] <= 16'b1111111111011110;
Y_AC[130] <= 16'b1111111111011111;
Y_AC[131] <= 16'b1111111111100000;
Y_AC[132] <= 16'b1111111111100001;
Y_AC[133] <= 16'b1111111111100010;
Y_AC[134] <= 16'b1111111111100011;
Y_AC[135] <= 16'b1111111111100100;
Y_AC[136] <= 16'b1111111111100101;
Y_AC[137] <= 16'b1111111111100110;
Y_AC[138] <= 16'b1111111111100111;
Y_AC[139] <= 16'b1111111111101000;
Y_AC[140] <= 16'b1111111111101001;
Y_AC[141] <= 16'b1111111111101010;
Y_AC[142] <= 16'b1111111111101011;
Y_AC[143] <= 16'b1111111111101100;
Y_AC[144] <= 16'b1111111111101101;
Y_AC[145] <= 16'b1111111111101110;
Y_AC[146] <= 16'b1111111111101111;
Y_AC[147] <= 16'b1111111111110000;
Y_AC[148] <= 16'b1111111111110001;
Y_AC[149] <= 16'b1111111111110010;
Y_AC[150] <= 16'b1111111111110011;
Y_AC[151] <= 16'b1111111111110100;
Y_AC[152] <= 16'b1111111111110101;
Y_AC[153] <= 16'b1111111111110110;
Y_AC[154] <= 16'b1111111111110111;
Y_AC[155] <= 16'b1111111111111000;
Y_AC[156] <= 16'b1111111111111001;
Y_AC[157] <= 16'b1111111111111010;
Y_AC[158] <= 16'b1111111111111011;
Y_AC[159] <= 16'b1111111111111100;
Y_AC[160] <= 16'b1111111111111101;
Y_AC[161] <= 16'b1111111111111110;
Y_AC_run_code[1] <= 0;
Y_AC_run_code[2] <= 1;
Y_AC_run_code[3] <= 2;
Y_AC_run_code[0] <= 3;
Y_AC_run_code[4] <= 4;
Y_AC_run_code[17] <= 5;
Y_AC_run_code[5] <= 6;
Y_AC_run_code[18] <= 7;
Y_AC_run_code[33] <= 8;
Y_AC_run_code[49] <= 9;
Y_AC_run_code[65] <= 10;
Y_AC_run_code[6] <= 11;
Y_AC_run_code[19] <= 12;
Y_AC_run_code[81] <= 13;
Y_AC_run_code[97] <= 14;
Y_AC_run_code[7] <= 15;
Y_AC_run_code[34] <= 16;
Y_AC_run_code[113] <= 17;
Y_AC_run_code[20] <= 18;
Y_AC_run_code[50] <= 19;
Y_AC_run_code[129] <= 20;
Y_AC_run_code[145] <= 21;
Y_AC_run_code[161] <= 22;
Y_AC_run_code[8] <= 23;
Y_AC_run_code[35] <= 24;
Y_AC_run_code[66] <= 25;
Y_AC_run_code[177] <= 26;
Y_AC_run_code[193] <= 27;
Y_AC_run_code[21] <= 28;
Y_AC_run_code[82] <= 29;
Y_AC_run_code[209] <= 30;
Y_AC_run_code[240] <= 31;
Y_AC_run_code[36] <= 32;
Y_AC_run_code[51] <= 33;
Y_AC_run_code[98] <= 34;
Y_AC_run_code[114] <= 35;
Y_AC_run_code[130] <= 36;
Y_AC_run_code[9] <= 37;
Y_AC_run_code[10] <= 38;
Y_AC_run_code[22] <= 39;
Y_AC_run_code[23] <= 40;
Y_AC_run_code[24] <= 41;
Y_AC_run_code[25] <= 42;
Y_AC_run_code[26] <= 43;
Y_AC_run_code[37] <= 44;
Y_AC_run_code[38] <= 45;
Y_AC_run_code[39] <= 46;
Y_AC_run_code[40] <= 47;
Y_AC_run_code[41] <= 48;
Y_AC_run_code[42] <= 49;
Y_AC_run_code[52] <= 50;
Y_AC_run_code[53] <= 51;
Y_AC_run_code[54] <= 52;
Y_AC_run_code[55] <= 53;
Y_AC_run_code[56] <= 54;
Y_AC_run_code[57] <= 55;
Y_AC_run_code[58] <= 56;
Y_AC_run_code[67] <= 57;
Y_AC_run_code[68] <= 58;
Y_AC_run_code[69] <= 59;
Y_AC_run_code[70] <= 60;
Y_AC_run_code[71] <= 61;
Y_AC_run_code[72] <= 62;
Y_AC_run_code[73] <= 63;
Y_AC_run_code[74] <= 64;
Y_AC_run_code[83] <= 65;
Y_AC_run_code[84] <= 66;
Y_AC_run_code[85] <= 67;
Y_AC_run_code[86] <= 68;
Y_AC_run_code[87] <= 69;
Y_AC_run_code[88] <= 70;
Y_AC_run_code[89] <= 71;
Y_AC_run_code[90] <= 72;
Y_AC_run_code[99] <= 73;
Y_AC_run_code[100] <= 74;
Y_AC_run_code[101] <= 75;
Y_AC_run_code[102] <= 76;
Y_AC_run_code[103] <= 77;
Y_AC_run_code[104] <= 78;
Y_AC_run_code[105] <= 79;
Y_AC_run_code[106] <= 80;
Y_AC_run_code[115] <= 81;
Y_AC_run_code[116] <= 82;
Y_AC_run_code[117] <= 83;
Y_AC_run_code[118] <= 84;
Y_AC_run_code[119] <= 85;
Y_AC_run_code[120] <= 86;
Y_AC_run_code[121] <= 87;
Y_AC_run_code[122] <= 88;
Y_AC_run_code[131] <= 89;
Y_AC_run_code[132] <= 90;
Y_AC_run_code[133] <= 91;
Y_AC_run_code[134] <= 92;
Y_AC_run_code[135] <= 93;
Y_AC_run_code[136] <= 94;
Y_AC_run_code[137] <= 95;
Y_AC_run_code[138] <= 96;
Y_AC_run_code[146] <= 97;
Y_AC_run_code[147] <= 98;
Y_AC_run_code[148] <= 99;
Y_AC_run_code[149] <= 100;
Y_AC_run_code[150] <= 101;
Y_AC_run_code[151] <= 102;
Y_AC_run_code[152] <= 103;
Y_AC_run_code[153] <= 104;
Y_AC_run_code[154] <= 105;
Y_AC_run_code[162] <= 106;
Y_AC_run_code[163] <= 107;
Y_AC_run_code[164] <= 108;
Y_AC_run_code[165] <= 109;
Y_AC_run_code[166] <= 110;
Y_AC_run_code[167] <= 111;
Y_AC_run_code[168] <= 112;
Y_AC_run_code[169] <= 113;
Y_AC_run_code[170] <= 114;
Y_AC_run_code[178] <= 115;
Y_AC_run_code[179] <= 116;
Y_AC_run_code[180] <= 117;
Y_AC_run_code[181] <= 118;
Y_AC_run_code[182] <= 119;
Y_AC_run_code[183] <= 120;
Y_AC_run_code[184] <= 121;
Y_AC_run_code[185] <= 122;
Y_AC_run_code[186] <= 123;
Y_AC_run_code[194] <= 124;
Y_AC_run_code[195] <= 125;
Y_AC_run_code[196] <= 126;
Y_AC_run_code[197] <= 127;
Y_AC_run_code[198] <= 128;
Y_AC_run_code[199] <= 129;
Y_AC_run_code[200] <= 130;
Y_AC_run_code[201] <= 131;
Y_AC_run_code[202] <= 132;
Y_AC_run_code[210] <= 133;
Y_AC_run_code[211] <= 134;
Y_AC_run_code[212] <= 135;
Y_AC_run_code[213] <= 136;
Y_AC_run_code[214] <= 137;
Y_AC_run_code[215] <= 138;
Y_AC_run_code[216] <= 139;
Y_AC_run_code[217] <= 140;
Y_AC_run_code[218] <= 141;
Y_AC_run_code[225] <= 142;
Y_AC_run_code[226] <= 143;
Y_AC_run_code[227] <= 144;
Y_AC_run_code[228] <= 145;
Y_AC_run_code[229] <= 146;
Y_AC_run_code[230] <= 147;
Y_AC_run_code[231] <= 148;
Y_AC_run_code[232] <= 149;
Y_AC_run_code[233] <= 150;
Y_AC_run_code[234] <= 151;
Y_AC_run_code[241] <= 152;
Y_AC_run_code[242] <= 153;
Y_AC_run_code[243] <= 154;
Y_AC_run_code[244] <= 155;
Y_AC_run_code[245] <= 156;
Y_AC_run_code[246] <= 157;
Y_AC_run_code[247] <= 158;
Y_AC_run_code[248] <= 159;
Y_AC_run_code[249] <= 160;
Y_AC_run_code[250] <= 161;
	Y_AC_run_code[16] <= 0;
	Y_AC_run_code[32] <= 0;
	Y_AC_run_code[48] <= 0;
	Y_AC_run_code[64] <= 0;
	Y_AC_run_code[80] <= 0;
	Y_AC_run_code[96] <= 0;
	Y_AC_run_code[112] <= 0;
	Y_AC_run_code[128] <= 0;
	Y_AC_run_code[144] <= 0;
	Y_AC_run_code[160] <= 0;
	Y_AC_run_code[176] <= 0;
	Y_AC_run_code[192] <= 0;
	Y_AC_run_code[208] <= 0;
	Y_AC_run_code[224] <= 0;
end	

// Each block below sets individual bits [31:27] of the JPEG_bitstream register
// based on the enable_module signal and specific timing/control conditions.
// On reset, all bits are cleared to 0.

// Bit 31 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[31] <= 0;                             // Reset to 0
	else if (enable_module && rollover_7)                   // Update if rollover_7
		JPEG_bitstream[31] <= JPEG_bs_5[31];
	else if (enable_module && orc_8 == 0)                   // Update if orc_8 is 0
		JPEG_bitstream[31] <= JPEG_bs_5[31];
end

// Bit 30 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[30] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[30] <= JPEG_bs_5[30];
	else if (enable_module && orc_8 <= 1)
		JPEG_bitstream[30] <= JPEG_bs_5[30];
end

// Bit 29 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[29] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[29] <= JPEG_bs_5[29];
	else if (enable_module && orc_8 <= 2)
		JPEG_bitstream[29] <= JPEG_bs_5[29];
end

// Bit 28 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[28] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[28] <= JPEG_bs_5[28];
	else if (enable_module && orc_8 <= 3)
		JPEG_bitstream[28] <= JPEG_bs_5[28];
end

// Bit 27 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[27] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[27] <= JPEG_bs_5[27];
	else if (enable_module && orc_8 <= 4)
		JPEG_bitstream[27] <= JPEG_bs_5[27];
end


// Bit 26 update logic
// Writes bit 26 from JPEG_bs_5 to JPEG_bitstream based on control signals
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[26] <= 0;                    // Reset bit 26
	else if (enable_module && rollover_7)
		JPEG_bitstream[26] <= JPEG_bs_5[26];        // Write on rollover_7
	else if (enable_module && orc_8 <= 5)
		JPEG_bitstream[26] <= JPEG_bs_5[26];        // Write if orc_8 ≤ 5
end

// Bit 25 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[25] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[25] <= JPEG_bs_5[25];
	else if (enable_module && orc_8 <= 6)
		JPEG_bitstream[25] <= JPEG_bs_5[25];
end

// Bit 24 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[24] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[24] <= JPEG_bs_5[24];
	else if (enable_module && orc_8 <= 7)
		JPEG_bitstream[24] <= JPEG_bs_5[24];
end

// Bit 23 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[23] <= 0;                      // Reset bit 23
	else if (enable_module && rollover_7)
		JPEG_bitstream[23] <= JPEG_bs_5[23];          // Update on rollover_7
	else if (enable_module && orc_8 <= 8)
		JPEG_bitstream[23] <= JPEG_bs_5[23];          // Update if orc_8 ≤ 8
end

// Bit 22 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[22] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[22] <= JPEG_bs_5[22];
	else if (enable_module && orc_8 <= 9)
		JPEG_bitstream[22] <= JPEG_bs_5[22];
end

// Bit 21 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[21] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[21] <= JPEG_bs_5[21];
	else if (enable_module && orc_8 <= 10)
		JPEG_bitstream[21] <= JPEG_bs_5[21];
end

// Bit 20 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[20] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[20] <= JPEG_bs_5[20];
	else if (enable_module && orc_8 <= 11)
		JPEG_bitstream[20] <= JPEG_bs_5[20];
end

// Bit 19 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[19] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[19] <= JPEG_bs_5[19];
	else if (enable_module && orc_8 <= 12)
		JPEG_bitstream[19] <= JPEG_bs_5[19];
end
// Bit 18 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[18] <= 0;                      // Reset bit 18
	else if (enable_module && rollover_7)
		JPEG_bitstream[18] <= JPEG_bs_5[18];          // Load bit on rollover
	else if (enable_module && orc_8 <= 13)
		JPEG_bitstream[18] <= JPEG_bs_5[18];          // Load if orc_8 <= 13
end

// Bit 17 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[17] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[17] <= JPEG_bs_5[17];
	else if (enable_module && orc_8 <= 14)
		JPEG_bitstream[17] <= JPEG_bs_5[17];
end

// Bit 16 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[16] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[16] <= JPEG_bs_5[16];
	else if (enable_module && orc_8 <= 15)
		JPEG_bitstream[16] <= JPEG_bs_5[16];
end

// Bit 15 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[15] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[15] <= JPEG_bs_5[15];
	else if (enable_module && orc_8 <= 16)
		JPEG_bitstream[15] <= JPEG_bs_5[15];
end

// Bit 14 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[14] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[14] <= JPEG_bs_5[14];
	else if (enable_module && orc_8 <= 17)
		JPEG_bitstream[14] <= JPEG_bs_5[14];
end

// Bit 13 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[13] <= 0;                    // Reset bit 13
	else if (enable_module && rollover_7)
		JPEG_bitstream[13] <= JPEG_bs_5[13];        // Load on rollover
	else if (enable_module && orc_8 <= 18)
		JPEG_bitstream[13] <= JPEG_bs_5[13];        // Load if orc_8 <= 18
end

// Bit 12 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[12] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[12] <= JPEG_bs_5[12];
	else if (enable_module && orc_8 <= 19)
		JPEG_bitstream[12] <= JPEG_bs_5[12];
end

// Bit 11 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[11] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[11] <= JPEG_bs_5[11];
	else if (enable_module && orc_8 <= 20)
		JPEG_bitstream[11] <= JPEG_bs_5[11];
end

// Bit 10 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[10] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[10] <= JPEG_bs_5[10];
	else if (enable_module && orc_8 <= 21)
		JPEG_bitstream[10] <= JPEG_bs_5[10];
end

// Bit 9 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[9] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[9] <= JPEG_bs_5[9];
	else if (enable_module && orc_8 <= 22)
		JPEG_bitstream[9] <= JPEG_bs_5[9];
end

// Bit 8 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[8] <= 0;                       // Reset bit 8
	else if (enable_module && rollover_7)
		JPEG_bitstream[8] <= JPEG_bs_5[8];            // Load on rollover
	else if (enable_module && orc_8 <= 23)
		JPEG_bitstream[8] <= JPEG_bs_5[8];            // Load if orc_8 <= 23
end

// Bit 7 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[7] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[7] <= JPEG_bs_5[7];
	else if (enable_module && orc_8 <= 24)
		JPEG_bitstream[7] <= JPEG_bs_5[7];
end

// Bit 6 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[6] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[6] <= JPEG_bs_5[6];
	else if (enable_module && orc_8 <= 25)
		JPEG_bitstream[6] <= JPEG_bs_5[6];
end

// Bit 5 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[5] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[5] <= JPEG_bs_5[5];
	else if (enable_module && orc_8 <= 26)
		JPEG_bitstream[5] <= JPEG_bs_5[5];
end

// Bit 4 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[4] <= 0;                  // Reset bit 4
	else if (enable_module && rollover_7)
		JPEG_bitstream[4] <= JPEG_bs_5[4];       // Load on rollover
	else if (enable_module && orc_8 <= 27)
		JPEG_bitstream[4] <= JPEG_bs_5[4];       // Load if orc_8 <= 27
end

// Bit 3 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[3] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[3] <= JPEG_bs_5[3];
	else if (enable_module && orc_8 <= 28)
		JPEG_bitstream[3] <= JPEG_bs_5[3];
end

// Bit 2 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[2] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[2] <= JPEG_bs_5[2];
	else if (enable_module && orc_8 <= 29)
		JPEG_bitstream[2] <= JPEG_bs_5[2];
end

// Bit 1 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[1] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[1] <= JPEG_bs_5[1];
	else if (enable_module && orc_8 <= 30)
		JPEG_bitstream[1] <= JPEG_bs_5[1];
end

// Bit 0 update logic
always_ff @(posedge clk) begin
	if (rst)
		JPEG_bitstream[0] <= 0;
	else if (enable_module && rollover_7)
		JPEG_bitstream[0] <= JPEG_bs_5[0];
	else if (enable_module && orc_8 <= 31)
		JPEG_bitstream[0] <= JPEG_bs_5[0];
end

endmodule
