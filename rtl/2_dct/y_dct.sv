/*----------------------------------------------------------------------------------
Module Name : y_dct
 Description : This module performs an 8x8 Discrete Cosine Transform (DCT) on 8-bit pixel
data for the Y (luminance) component of a JPEG image. It receives a stream 
of input pixel values and applies the 2D DCT operation using a separable row-column transformation method. 
The output consists of 64 frequency-domain coefficients labeled Z11_final through Z88_final,
representing the transformed matrix in zig-zag order or raster-scan layout depending on post-processing.
 The DCT is implemented using fixed-point arithmetic and pipelined processing
to improve throughput. The module supports enable and reset signals for 
control and synchronization with upstream modules.
----------------------------------------------------------------------------------------*/
`timescale 1ns / 100ps

module y_dct (
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [7:0]  data_in,

    output logic [10:0] Z11_final, Z12_final, Z13_final, Z14_final, Z15_final, Z16_final, Z17_final, Z18_final,
    output logic [10:0] Z21_final, Z22_final, Z23_final, Z24_final, Z25_final, Z26_final, Z27_final, Z28_final,
    output logic [10:0] Z31_final, Z32_final, Z33_final, Z34_final, Z35_final, Z36_final, Z37_final, Z38_final,
    output logic [10:0] Z41_final, Z42_final, Z43_final, Z44_final, Z45_final, Z46_final, Z47_final, Z48_final,
    output logic [10:0] Z51_final, Z52_final, Z53_final, Z54_final, Z55_final, Z56_final, Z57_final, Z58_final,
    output logic [10:0] Z61_final, Z62_final, Z63_final, Z64_final, Z65_final, Z66_final, Z67_final, Z68_final,
    output logic [10:0] Z71_final, Z72_final, Z73_final, Z74_final, Z75_final, Z76_final, Z77_final, Z78_final,
    output logic [10:0] Z81_final, Z82_final, Z83_final, Z84_final, Z85_final, Z86_final, Z87_final, Z88_final,

    output logic        output_enable
);

// -----------------------------------------------------------------------------
// Internal Constants and Variables
// -----------------------------------------------------------------------------
int T1, T21, T22, T23, T24, T25, T26, T27, T28, T31, T32, T33, T34, T52;
int Ti1, Ti21, Ti22, Ti23, Ti24, Ti25, Ti26, Ti27, Ti28, Ti31, Ti32, Ti33, Ti34, Ti52;
int Y2_mul_input, Y3_mul_input, Y4_mul_input, Y5_mul_input, Y6_mul_input, Y7_mul_input, Y8_mul_input;
int Ti2_mul_input, Ti3_mul_input, Ti4_mul_input, Ti5_mul_input, Ti6_mul_input, Ti7_mul_input, Ti8_mul_input;

// -----------------------------------------------------------------------------
// Registers and Temporary Storage
// -----------------------------------------------------------------------------
logic [24:0] Y11, Y21, Y31, Y41, Y51, Y61, Y71, Y81;
logic [24:0] Y21_final, Y31_final, Y41_final, Y51_final, Y61_final, Y71_final, Y81_final;
logic [24:0] Y21_final_prev, Y31_final_prev, Y41_final_prev, Y51_final_prev, Y61_final_prev, Y71_final_prev, Y81_final_prev;
logic [24:0] Y21_final_diff, Y31_final_diff, Y41_final_diff, Y51_final_diff, Y61_final_diff, Y71_final_diff, Y81_final_diff;

logic [31:0] Y11_final_2, Y21_final_2, Y11_final_3, Y11_final_4, Y31_final_2, Y41_final_2, Y51_final_2, Y61_final_2, Y71_final_2, Y81_final_2;
logic [12:0] Y11_final_1, Y21_final_1, Y31_final_1, Y41_final_1, Y51_final_1, Y61_final_1, Y71_final_1, Y81_final_1;

logic [31:0] Z_temp[1:8][1:8];  // Replaces Z_temp_11 ... Z_temp_88
logic [26:0] Z     [1:8][1:8];  // Replaces Z11 ... Z88

// -----------------------------------------------------------------------------
// Control Logic
// -----------------------------------------------------------------------------
logic [2:0] count, count_of, count_of_copy;
logic       count_1, count_3, count_4, count_5, count_6, count_7, count_8, count_9, count_10;
logic       enable_1;
logic [7:0] data_1;


// -----------------------------------------------------------------------------
// Forward DCT Matrix Constants (scaled fixed-point representation)
// -----------------------------------------------------------------------------
localparam int T1  = 5793;   // ≈ 0.3536
localparam int T21 = 8035;   // ≈ 0.4904
localparam int T22 = 6811;   // ≈ 0.4157
localparam int T23 = 4551;   // ≈ 0.2778
localparam int T24 = 1598;   // ≈ 0.0975
localparam int T25 = -1598;  // ≈ -0.0975
localparam int T26 = -4551;  // ≈ -0.2778
localparam int T27 = -6811;  // ≈ -0.4157
localparam int T28 = -8035;  // ≈ -0.4904
localparam int T31 = 7568;   // ≈ 0.4619
localparam int T32 = 3135;   // ≈ 0.1913
localparam int T33 = -3135;  // ≈ -0.1913
localparam int T34 = -7568;  // ≈ -0.4619
localparam int T52 = -5793;  // ≈ -0.3536

// -----------------------------------------------------------------------------
// Inverse DCT Matrix Constants (same values reused)
// -----------------------------------------------------------------------------
localparam int Ti1  = T1;
localparam int Ti21 = T21;
localparam int Ti22 = T22;
localparam int Ti23 = T23;
localparam int Ti24 = T24;
localparam int Ti25 = T25;
localparam int Ti26 = T26;
localparam int Ti27 = T27;
localparam int Ti28 = T28;
localparam int Ti31 = T31;
localparam int Ti32 = T32;
localparam int Ti33 = T33;
localparam int Ti34 = T34;
localparam int Ti52 = T52;

// -----------------------------------------------------------------------------
// Signal Declarations (outside the always_ff block)
// -----------------------------------------------------------------------------
logic [31:0] Z_temp[1:8][1:8];     // replaces Z_temp_11 to Z_temp_88
logic [31:0] Y_final[1:8];         // replaces Y11_final_4 to Y81_final_2
int          Ti_mul_input[1:8];    // replaces Ti1 to Ti8_mul_input

// -----------------------------------------------------------------------------
// Combinational Assignment (before the always_ff block)
// -----------------------------------------------------------------------------
always_comb begin
    Y_final[1] = Y11_final_4;
    Y_final[2] = Y21_final_2;
    Y_final[3] = Y31_final_2;
    Y_final[4] = Y41_final_2;
    Y_final[5] = Y51_final_2;
    Y_final[6] = Y61_final_2;
    Y_final[7] = Y71_final_2;
    Y_final[8] = Y81_final_2;

    Ti_mul_input[1] = Ti1;
    Ti_mul_input[2] = Ti2_mul_input;
    Ti_mul_input[3] = Ti3_mul_input;
    Ti_mul_input[4] = Ti4_mul_input;
    Ti_mul_input[5] = Ti5_mul_input;
    Ti_mul_input[6] = Ti6_mul_input;
    Ti_mul_input[7] = Ti7_mul_input;
    Ti_mul_input[8] = Ti8_mul_input;
end

// -----------------------------------------------------------------------------
// Sequential DCT Output Computation Block
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst) begin
        foreach (Z_temp[i, j])
            Z_temp[i][j] <= 32'd0;
    end else if (enable_1 && count_8) begin
        foreach (Z_temp[i, j])
            Z_temp[i][j] <= Y_final[i] * Ti_mul_input[j];
    end
end



// -----------------------------------------------------------------------------
// Sequential Block for Reset, Clear, and Accumulate
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst) begin
        foreach (Z[i, j])
            Z[i][j] <= '0;
    end 
    else if (count_8 && count_of == 3'd1) begin
        foreach (Z[i, j])
            Z[i][j] <= '0;
    end 
    else if (enable && count_9) begin
        foreach (Z[i, j])
            Z[i][j] <= Z[i][j] + Z_temp[i][j];
    end
end

// -----------------------------------------------------------------------------
// Final Output Computation with Reset and Rounding
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst) begin
        // Reset all final DCT outputs
        foreach (Z_final[i, j]) begin
            Z_final[i][j] <= '0;
        end
    end
    else if (count_10 && count_of == 0) begin
        // Rounding: if bit 15 is 1 (fraction ≥ 0.5), round up; else truncate
        foreach (Z[i, j]) begin
            Z_final[i][j] <= Z[i][j][15] ? Z[i][j][26:16] + 1 : Z[i][j][26:16];
        end
    end
end

// -----------------------------------------------------------------------------
// Module: Y Row Accumulation and Multiplier Stage
// Description: 
//   - Computes partial Y matrix entries during 1D DCT
//   - Handles multiplication and accumulation per clock cycle
//   - Signals when final output is ready to quantizer
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Control Signal: output_enable
//  - Asserted only when valid 8x8 block output is available (1st pass, count_10)
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst)
        output_enable <= 0;
    else if (!enable_1)
        output_enable <= 0;
    else if ((count_10 == 0) || count_of)
        output_enable <= 0;
    else if (count_10 && (count_of == 0))
        output_enable <= 1;  // Valid output condition
end

// -----------------------------------------------------------------------------
// Y_temp_11 Computation
//  - Multiply input data by transform coefficient T1
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst)
        Y_temp_11 <= 0;
    else if (enable)
        Y_temp_11 <= data_in * T1;
end

// -----------------------------------------------------------------------------
// Y11 Accumulation
//  - Load first term when count == 1
//  - Otherwise accumulate into Y11
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst)
        Y11 <= 0;
    else if ((count == 1) && enable)
        Y11 <= Y_temp_11;           // Initial load
    else if (enable)
        Y11 <= Y_temp_11 + Y11;     // Accumulate result
end

// -----------------------------------------------------------------------------
// Y_temp Computation for Rows 2 to 8
//  - Multiply input with corresponding transform coefficients
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst) begin
        Y_temp_21 <= 0;
        Y_temp_31 <= 0;
        Y_temp_41 <= 0;
        Y_temp_51 <= 0;
        Y_temp_61 <= 0;
        Y_temp_71 <= 0;
        Y_temp_81 <= 0;
    end 
    else if (!enable_1) begin
        Y_temp_21 <= 0;
        Y_temp_31 <= 0;
        Y_temp_41 <= 0;
        Y_temp_51 <= 0;
        Y_temp_61 <= 0;
        Y_temp_71 <= 0;
        Y_temp_81 <= 0;
    end 
    else if (enable_1) begin
        Y_temp_21 <= data_1 * Y2_mul_input;
        Y_temp_31 <= data_1 * Y3_mul_input;
        Y_temp_41 <= data_1 * Y4_mul_input;
        Y_temp_51 <= data_1 * Y5_mul_input;
        Y_temp_61 <= data_1 * Y6_mul_input;
        Y_temp_71 <= data_1 * Y7_mul_input;
        Y_temp_81 <= data_1 * Y8_mul_input;
    end
end
// -----------------------------------------------------------------------------
// Module: Y Row Accumulator and Clocked Counter Pipeline
// Description:
//   - Accumulates intermediate multiplication results (Y_temp_XX) row-wise
//   - Tracks DCT processing steps using a 10-stage pipeline of count flags
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Accumulation Logic for Y21 to Y81
//  - Accumulates Y_temp_xx values into corresponding Yxx registers
//  - Only active when enable_1 is high
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (rst) begin
		Y21 <= 0; Y31 <= 0; Y41 <= 0; Y51 <= 0;
		Y61 <= 0; Y71 <= 0; Y81 <= 0;
	end 
	else if (!enable_1) begin
		Y21 <= 0; Y31 <= 0; Y41 <= 0; Y51 <= 0;
		Y61 <= 0; Y71 <= 0; Y81 <= 0;
	end 
	else if (enable_1) begin
		Y21 <= Y21 + Y_temp_21;
		Y31 <= Y31 + Y_temp_31;
		Y41 <= Y41 + Y_temp_41;
		Y51 <= Y51 + Y_temp_51;
		Y61 <= Y61 + Y_temp_61;
		Y71 <= Y71 + Y_temp_71;
		Y81 <= Y81 + Y_temp_81;
	end
end

// -----------------------------------------------------------------------------
// Counter Pipeline Logic (count_1 to count_10)
//  - Used for pipelining processing steps (e.g., DCT stages)
//  - count_1 is assumed to be derived from count internally or elsewhere
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (rst) begin
		count     <= 0;
		count_3   <= 0;
		count_4   <= 0;
		count_5   <= 0;
		count_6   <= 0;
		count_7   <= 0;
		count_8   <= 0;
		count_9   <= 0;
		count_10  <= 0;
	end 
	else if (!enable) begin
		count     <= 0;
		count_3   <= 0;
		count_4   <= 0;
		count_5   <= 0;
		count_6   <= 0;
		count_7   <= 0;
		count_8   <= 0;
		count_9   <= 0;
		count_10  <= 0;
	end 
	else if (enable) begin
		count     <= count + 1;
		count_3   <= count_1;
		count_4   <= count_3;
		count_5   <= count_4;
		count_6   <= count_5;
		count_7   <= count_6;
		count_8   <= count_7;
		count_9   <= count_8;
		count_10  <= count_9;
	end
end


// -----------------------------------------------------------------------------
// count_1 goes high for one cycle when count == 7 and enable is active
// Used to signal end of an 8-cycle DCT row/column input sequence
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (rst) begin
		count_1 <= 0;
	end 
	else if (count != 7 || !enable) begin
		count_1 <= 0;
	end 
	else if (count == 7) begin
		count_1 <= 1;
	end
end

// -----------------------------------------------------------------------------
// count_of and count_of_copy increment when one 8-sample DCT pass is complete
// These act as DCT row (or column) counters for 8x8 matrix input
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (rst) begin
		count_of <= 0;
		count_of_copy <= 0;
	end 
	else if (!enable) begin
		count_of <= 0;
		count_of_copy <= 0;
	end 
	else if (count_1) begin
		count_of <= count_of + 1;
		count_of_copy <= count_of_copy + 1;
	end
end

// -----------------------------------------------------------------------------
// Center Y11 value by subtracting the DC bias from the first row's sum
// Only applied to first row (Y11) on count_3 (delayed pipeline stage)
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (rst) begin
		Y11_final <= 0;
	end 
	else if (count_3 && enable_1) begin
		// Subtract average (DC component): 128 * 8 * T1 = 128*8*5793 = 5932032
		Y11_final <= Y11 - 25'd5932032;

		/*
			Why subtract 5932032?
			- Each input Y pixel = 128 baseline (DC bias)
			- Block has 8 pixels: 128 * 8 = 1024
			- Coefficient T1 ≈ 5793 (scaled fixed-point)
			- Final DC bias = 1024 * 5793 = 5932032
			- This is the mean value that must be centered to 0
		*/
	end
end
// -----------------------------------------------------------------------------
// Module: Final Y Row Latching (Rows 2–8)
// Description:
//   - On reset: clear all final and previous values.
//   - When not enabled: clear all values to avoid stale data propagation.
//   - On count_4 (aligned to DCT pipeline): 
//     - Capture final row-wise accumulated DCT inputs.
//     - Also store previous values for potential differential processing.
// -----------------------------------------------------------------------------

always_ff @(posedge clk) begin
	if (rst) begin
		// Reset all outputs to 0
		Y21_final <= 0; Y21_final_prev <= 0;
		Y31_final <= 0; Y31_final_prev <= 0;
		Y41_final <= 0; Y41_final_prev <= 0;
		Y51_final <= 0; Y51_final_prev <= 0;
		Y61_final <= 0; Y61_final_prev <= 0;
		Y71_final <= 0; Y71_final_prev <= 0;
		Y81_final <= 0; Y81_final_prev <= 0;
	end 
	else if (!enable_1) begin
		// Clear values when disabled (to prevent invalid results)
		Y21_final <= 0; Y21_final_prev <= 0;
		Y31_final <= 0; Y31_final_prev <= 0;
		Y41_final <= 0; Y41_final_prev <= 0;
		Y51_final <= 0; Y51_final_prev <= 0;
		Y61_final <= 0; Y61_final_prev <= 0;
		Y71_final <= 0; Y71_final_prev <= 0;
		Y81_final <= 0; Y81_final_prev <= 0;
	end 
	else if (count_4 && enable_1) begin
		// Capture computed Y values on 4th pipeline cycle
		Y21_final <= Y21; Y21_final_prev <= Y21_final;
		Y31_final <= Y31; Y31_final_prev <= Y31_final;
		Y41_final <= Y41; Y41_final_prev <= Y41_final;
		Y51_final <= Y51; Y51_final_prev <= Y51_final;
		Y61_final <= Y61; Y61_final_prev <= Y61_final;
		Y71_final <= Y71; Y71_final_prev <= Y71_final;
		Y81_final <= Y81; Y81_final_prev <= Y81_final;
	end
end

// -----------------------------------------------------------------------------
// Module: Final Y Row Difference Calculator (Rows 2–8)
// Description:
//   - Computes how much each row’s output has changed from the previous cycle.
//   - Useful for differential processing (e.g., delta encoding).
//   - Active on the 5th pipeline cycle after accumulation.
// -----------------------------------------------------------------------------

always_ff @(posedge clk) begin
	if (rst) begin
		// Clear all differences on reset
		Y21_final_diff <= 0; Y31_final_diff <= 0;
		Y41_final_diff <= 0; Y51_final_diff <= 0;
		Y61_final_diff <= 0; Y71_final_diff <= 0;
		Y81_final_diff <= 0;
	end
	else if (count_5 && enable_1) begin
		// Calculate deltas between current and previous values
		Y21_final_diff <= Y21_final - Y21_final_prev;
		Y31_final_diff <= Y31_final - Y31_final_prev;
		Y41_final_diff <= Y41_final - Y41_final_prev;
		Y51_final_diff <= Y51_final - Y51_final_prev;
		Y61_final_diff <= Y61_final - Y61_final_prev;
		Y71_final_diff <= Y71_final - Y71_final_prev;
		Y81_final_diff <= Y81_final - Y81_final_prev;
	end
end


// Assign appropriate T2* values to Y2_mul_input based on the 3-bit count
always_ff @(posedge clk) begin
	unique case (count)
		3'b000: Y2_mul_input <= T21;
		3'b001: Y2_mul_input <= T22;
		3'b010: Y2_mul_input <= T23;
		3'b011: Y2_mul_input <= T24;
		3'b100: Y2_mul_input <= T25;
		3'b101: Y2_mul_input <= T26;
		3'b110: Y2_mul_input <= T27;
		3'b111: Y2_mul_input <= T28;
	endcase
end

// Assign appropriate T3* values to Y3_mul_input for mirrored DCT symmetry
always_ff @(posedge clk) begin
	unique case (count)
		3'b000: Y3_mul_input <= T31;
		3'b001: Y3_mul_input <= T32;
		3'b010: Y3_mul_input <= T33;
		3'b011: Y3_mul_input <= T34;
		3'b100: Y3_mul_input <= T34;
		3'b101: Y3_mul_input <= T33;
		3'b110: Y3_mul_input <= T32;
		3'b111: Y3_mul_input <= T31;
	endcase
end

// Assign specific permutation of T2* to Y4_mul_input for DCT transform
always_ff @(posedge clk) begin
	unique case (count)
		3'b000: Y4_mul_input <= T22;
		3'b001: Y4_mul_input <= T25;
		3'b010: Y4_mul_input <= T28;
		3'b011: Y4_mul_input <= T26;
		3'b100: Y4_mul_input <= T23;
		3'b101: Y4_mul_input <= T21;
		3'b110: Y4_mul_input <= T24;
		3'b111: Y4_mul_input <= T27;
	endcase
end

// Assign specific T values to Y5_mul_input for this row's DCT multiplication
always_ff @(posedge clk) begin
	unique case (count)
		3'b000: Y5_mul_input <= T1;
		3'b001: Y5_mul_input <= T52;
		3'b010: Y5_mul_input <= T52;
		3'b011: Y5_mul_input <= T1;
		3'b100: Y5_mul_input <= T1;
		3'b101: Y5_mul_input <= T52;
		3'b110: Y5_mul_input <= T52;
		3'b111: Y5_mul_input <= T1;
	endcase
end

// Assign specific T2* values to Y6_mul_input based on the DCT logic
always_ff @(posedge clk) begin
	unique case (count)
		3'b000: Y6_mul_input <= T23;
		3'b001: Y6_mul_input <= T28;
		3'b010: Y6_mul_input <= T24;
		3'b011: Y6_mul_input <= T22;
		3'b100: Y6_mul_input <= T27;
		3'b101: Y6_mul_input <= T25;
		3'b110: Y6_mul_input <= T21;
		3'b111: Y6_mul_input <= T26;
	endcase
end

// Assign mirrored T3* values to Y7_mul_input for DCT coefficient symmetry
always_ff @(posedge clk) begin
	unique case (count)
		3'b000: Y7_mul_input <= T32;
		3'b001: Y7_mul_input <= T34;
		3'b010: Y7_mul_input <= T31;
		3'b011: Y7_mul_input <= T33;
		3'b100: Y7_mul_input <= T33;
		3'b101: Y7_mul_input <= T31;
		3'b110: Y7_mul_input <= T34;
		3'b111: Y7_mul_input <= T32;
	endcase
end

// Assigning values to Y8_mul_input based on current count (forward DCT row logic)
always_ff @(posedge clk) begin
	unique case (count)
		3'b000: Y8_mul_input <= T24;
		3'b001: Y8_mul_input <= T26;
		3'b010: Y8_mul_input <= T22;
		3'b011: Y8_mul_input <= T28;
		3'b100: Y8_mul_input <= T21;
		3'b101: Y8_mul_input <= T27;
		3'b110: Y8_mul_input <= T23;
		3'b111: Y8_mul_input <= T25;
	endcase
end

// Assigning inverse DCT matrix row 2 inputs for column-wise iDCT
always_ff @(posedge clk) begin
	unique case (count_of_copy)
		3'b000: Ti2_mul_input <= Ti28;
		3'b001: Ti2_mul_input <= Ti21;
		3'b010: Ti2_mul_input <= Ti22;
		3'b011: Ti2_mul_input <= Ti23;
		3'b100: Ti2_mul_input <= Ti24;
		3'b101: Ti2_mul_input <= Ti25;
		3'b110: Ti2_mul_input <= Ti26;
		3'b111: Ti2_mul_input <= Ti27;
	endcase
end

// Assigning inverse DCT matrix row 3 inputs (symmetry logic applied)
always_ff @(posedge clk) begin
	unique case (count_of_copy)
		3'b000: Ti3_mul_input <= Ti31;
		3'b001: Ti3_mul_input <= Ti31;
		3'b010: Ti3_mul_input <= Ti32;
		3'b011: Ti3_mul_input <= Ti33;
		3'b100: Ti3_mul_input <= Ti34;
		3'b101: Ti3_mul_input <= Ti34;
		3'b110: Ti3_mul_input <= Ti33;
		3'b111: Ti3_mul_input <= Ti32;
	endcase
end

// Assigning inverse DCT matrix row 4 inputs for iDCT computation
always_ff @(posedge clk) begin
	unique case (count_of_copy)
		3'b000: Ti4_mul_input <= Ti27;
		3'b001: Ti4_mul_input <= Ti22;
		3'b010: Ti4_mul_input <= Ti25;
		3'b011: Ti4_mul_input <= Ti28;
		3'b100: Ti4_mul_input <= Ti26;
		3'b101: Ti4_mul_input <= Ti23;
		3'b110: Ti4_mul_input <= Ti21;
		3'b111: Ti4_mul_input <= Ti24;
	endcase
end

// Assign inverse DCT matrix row 5 inputs based on count_of_copy
always_ff @(posedge clk) begin
	unique case (count_of_copy)
		3'b000: Ti5_mul_input <= Ti1;
		3'b001: Ti5_mul_input <= Ti1;
		3'b010: Ti5_mul_input <= Ti52;
		3'b011: Ti5_mul_input <= Ti52;
		3'b100: Ti5_mul_input <= Ti1;
		3'b101: Ti5_mul_input <= Ti1;
		3'b110: Ti5_mul_input <= Ti52;
		3'b111: Ti5_mul_input <= Ti52;
	endcase
end

// Assign inverse DCT matrix row 6 inputs based on count_of_copy
always_ff @(posedge clk) begin
	unique case (count_of_copy)
		3'b000: Ti6_mul_input <= Ti26;
		3'b001: Ti6_mul_input <= Ti23;
		3'b010: Ti6_mul_input <= Ti28;
		3'b011: Ti6_mul_input <= Ti24;
		3'b100: Ti6_mul_input <= Ti22;
		3'b101: Ti6_mul_input <= Ti27;
		3'b110: Ti6_mul_input <= Ti25;
		3'b111: Ti6_mul_input <= Ti21;
	endcase
end

// Assign inverse DCT matrix row 7 inputs based on count_of_copy
always_ff @(posedge clk) begin
	unique case (count_of_copy)
		3'b000: Ti7_mul_input <= Ti32;
		3'b001: Ti7_mul_input <= Ti32;
		3'b010: Ti7_mul_input <= Ti34;
		3'b011: Ti7_mul_input <= Ti31;
		3'b100: Ti7_mul_input <= Ti33;
		3'b101: Ti7_mul_input <= Ti33;
		3'b110: Ti7_mul_input <= Ti31;
		3'b111: Ti7_mul_input <= Ti34;
	endcase
end

// Assign inverse DCT matrix row 8 inputs based on count_of_copy
always_ff @(posedge clk) begin
	unique case (count_of_copy)
		3'b000: Ti8_mul_input <= Ti25;
		3'b001: Ti8_mul_input <= Ti24;
		3'b010: Ti8_mul_input <= Ti26;
		3'b011: Ti8_mul_input <= Ti22;
		3'b100: Ti8_mul_input <= Ti28;
		3'b101: Ti8_mul_input <= Ti21;
		3'b110: Ti8_mul_input <= Ti27;
		3'b111: Ti8_mul_input <= Ti23;
	endcase
end

// -----------------------------------------------------------------------------
// Rounding and sign-extension stage for DCT output values (vectorized)
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (rst) begin
		data_1 <= 0;
		for (int i = 1; i <= 8; i++) begin
			Y_final_1[i] <= 0;
			Y_final_2[i] <= 0;
		end
		Y11_final_3 <= 0;
		Y11_final_4 <= 0;
	end else if (enable) begin
		data_1 <= data_in;

		// ---------- Y11 final (DC term) ----------
		Y_final_1[1] <= Y11_final[11] ? Y11_final[24:12] + 1 : Y11_final[24:12];
		Y_final_2[1][31:13] <= Y_final_1[1][12] ? 21'h1FFFFF : 21'h000000;
		Y_final_2[1][12:0]  <= Y_final_1[1];
		Y11_final_3 <= Y_final_2[1];
		Y11_final_4 <= Y11_final_3;

		// ---------- Y2 to Y8 AC values ----------
		for (int i = 2; i <= 8; i++) begin
			logic signed [24:0] diff;
			case (i)
				2: diff = Y21_final_diff;
				3: diff = Y31_final_diff;
				4: diff = Y41_final_diff;
				5: diff = Y51_final_diff;
				6: diff = Y61_final_diff;
				7: diff = Y71_final_diff;
				8: diff = Y81_final_diff;
			endcase

			Y_final_1[i] <= diff[11] ? diff[24:12] + 1 : diff[24:12];
			Y_final_2[i][31:13] <= Y_final_1[i][12] ? 21'h1FFFFF : 21'h000000;
			Y_final_2[i][12:0]  <= Y_final_1[i];
		end

	end
end

// -----------------------------------------------------------------------------
// Pipeline enable signal stage
// -----------------------------------------------------------------------------
always_ff @(posedge clk) begin
	if (rst)
		enable_1 <= 1'b0;
	else
		enable_1 <= enable;
end

