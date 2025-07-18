/*----------------------------------------------------------------------------------
Module Name : cb_dct
Description : This module performs an 8x8 Discrete Cosine Transform (DCT) on 8-bit pixel
data for the Cb (chrominance-blue) component of a JPEG image. It receives a stream 
of input pixel values and applies the 2D DCT operation using a separable row-column transformation method. 
The output consists of 64 frequency-domain coefficients labeled Z11_final through Z88_final,
representing the transformed matrix in zig-zag order or raster-scan layout depending on post-processing.
The DCT is implemented using fixed-point arithmetic and pipelined processing
to improve throughput. The module supports enable and reset signals for 
control and synchronization with upstream modules.
----------------------------------------------------------------------------------------*/
`timescale 1ns / 100ps

module cb_dct (
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [7:0]  data_in,

    output logic [10:0] Z11_final, Z12_final, Z13_final, Z14_final,
                        Z15_final, Z16_final, Z17_final, Z18_final,
                        Z21_final, Z22_final, Z23_final, Z24_final,
                        Z25_final, Z26_final, Z27_final, Z28_final,
                        Z31_final, Z32_final, Z33_final, Z34_final,
                        Z35_final, Z36_final, Z37_final, Z38_final,
                        Z41_final, Z42_final, Z43_final, Z44_final,
                        Z45_final, Z46_final, Z47_final, Z48_final,
                        Z51_final, Z52_final, Z53_final, Z54_final,
                        Z55_final, Z56_final, Z57_final, Z58_final,
                        Z61_final, Z62_final, Z63_final, Z64_final,
                        Z65_final, Z66_final, Z67_final, Z68_final,
                        Z71_final, Z72_final, Z73_final, Z74_final,
                        Z75_final, Z76_final, Z77_final, Z78_final,
                        Z81_final, Z82_final, Z83_final, Z84_final,
                        Z85_final, Z86_final, Z87_final, Z88_final,

    output logic        output_enable
);

// -----------------------------------------------------------------------------
// DCT and IDCT Constants - declared as signed integers for multiplication
// -----------------------------------------------------------------------------
integer T1, T21, T22, T23, T24, T25, T26, T27, T28;
integer T31, T32, T33, T34, T52;
integer Ti1, Ti21, Ti22, Ti23, Ti24, Ti25, Ti26, Ti27, Ti28;
integer Ti31, Ti32, Ti33, Ti34, Ti52;

// Temporary variables for intermediate Cb calculations
logic [24:0] Cb_temp_11;
logic [24:0] Cb11, Cb21, Cb31, Cb41, Cb51, Cb61, Cb71, Cb81, Cb11_final;
logic [31:0] Cb_temp_21, Cb_temp_31, Cb_temp_41, Cb_temp_51;
logic [31:0] Cb_temp_61, Cb_temp_71, Cb_temp_81;

// Z temporary matrix (8x8) for intermediate operations
logic [31:0] Z_temp_11, Z_temp_12, Z_temp_13, Z_temp_14;
logic [31:0] Z_temp_15, Z_temp_16, Z_temp_17, Z_temp_18;
logic [31:0] Z_temp_21, Z_temp_22, Z_temp_23, Z_temp_24;
logic [31:0] Z_temp_25, Z_temp_26, Z_temp_27, Z_temp_28;
logic [31:0] Z_temp_31, Z_temp_32, Z_temp_33, Z_temp_34;
logic [31:0] Z_temp_35, Z_temp_36, Z_temp_37, Z_temp_38;
logic [31:0] Z_temp_41, Z_temp_42, Z_temp_43, Z_temp_44;
logic [31:0] Z_temp_45, Z_temp_46, Z_temp_47, Z_temp_48;
logic [31:0] Z_temp_51, Z_temp_52, Z_temp_53, Z_temp_54;
logic [31:0] Z_temp_55, Z_temp_56, Z_temp_57, Z_temp_58;
logic [31:0] Z_temp_61, Z_temp_62, Z_temp_63, Z_temp_64;
logic [31:0] Z_temp_65, Z_temp_66, Z_temp_67, Z_temp_68;
logic [31:0] Z_temp_71, Z_temp_72, Z_temp_73, Z_temp_74;
logic [31:0] Z_temp_75, Z_temp_76, Z_temp_77, Z_temp_78;
logic [31:0] Z_temp_81, Z_temp_82, Z_temp_83, Z_temp_84;
logic [31:0] Z_temp_85, Z_temp_86, Z_temp_87, Z_temp_88;

// Z final values (8x8 matrix)
logic [24:0] Z11, Z12, Z13, Z14, Z15, Z16, Z17, Z18;
logic [24:0] Z21, Z22, Z23, Z24, Z25, Z26, Z27, Z28;
logic [24:0] Z31, Z32, Z33, Z34, Z35, Z36, Z37, Z38;
logic [24:0] Z41, Z42, Z43, Z44, Z45, Z46, Z47, Z48;
logic [24:0] Z51, Z52, Z53, Z54, Z55, Z56, Z57, Z58;
logic [24:0] Z61, Z62, Z63, Z64, Z65, Z66, Z67, Z68;
logic [24:0] Z71, Z72, Z73, Z74, Z75, Z76, Z77, Z78;
logic [24:0] Z81, Z82, Z83, Z84, Z85, Z86, Z87, Z88;

// Final Cb values and intermediate steps
logic [31:0] Cb11_final_2, Cb21_final_2, Cb11_final_3, Cb11_final_4, Cb31_final_2, Cb41_final_2;
logic [31:0] Cb51_final_2, Cb61_final_2, Cb71_final_2, Cb81_final_2;
logic [10:0] Cb11_final_1, Cb21_final_1, Cb31_final_1, Cb41_final_1;
logic [10:0] Cb51_final_1, Cb61_final_1, Cb71_final_1, Cb81_final_1;

logic [24:0] Cb21_final, Cb31_final, Cb41_final, Cb51_final;
logic [24:0] Cb61_final, Cb71_final, Cb81_final;

// Difference tracking for final Cb values
logic [24:0] Cb21_final_prev, Cb21_final_diff;
logic [24:0] Cb31_final_prev, Cb31_final_diff;
logic [24:0] Cb41_final_prev, Cb41_final_diff;
logic [24:0] Cb51_final_prev, Cb51_final_diff;
logic [24:0] Cb61_final_prev, Cb61_final_diff;
logic [24:0] Cb71_final_prev, Cb71_final_diff;
logic [24:0] Cb81_final_prev, Cb81_final_diff;

// Final Z values for all positions (8x8 matrix)
logic [10:0] Z11_final, Z12_final, Z13_final, Z14_final;
logic [10:0] Z15_final, Z16_final, Z17_final, Z18_final;
logic [10:0] Z21_final, Z22_final, Z23_final, Z24_final;
logic [10:0] Z25_final, Z26_final, Z27_final, Z28_final;
logic [10:0] Z31_final, Z32_final, Z33_final, Z34_final;
logic [10:0] Z35_final, Z36_final, Z37_final, Z38_final;
logic [10:0] Z41_final, Z42_final, Z43_final, Z44_final;
logic [10:0] Z45_final, Z46_final, Z47_final, Z48_final;
logic [10:0] Z51_final, Z52_final, Z53_final, Z54_final;
logic [10:0] Z55_final, Z56_final, Z57_final, Z58_final;
logic [10:0] Z61_final, Z62_final, Z63_final, Z64_final;
logic [10:0] Z65_final, Z66_final, Z67_final, Z68_final;
logic [10:0] Z71_final, Z72_final, Z73_final, Z74_final;
logic [10:0] Z75_final, Z76_final, Z77_final, Z78_final;
logic [10:0] Z81_final, Z82_final, Z83_final, Z84_final;
logic [10:0] Z85_final, Z86_final, Z87_final, Z88_final;

// Control signals and counters
logic [2:0] count;
logic [2:0] count_of, count_of_copy;
logic count_1, count_3, count_4, count_5, count_6, count_7, count_8;
logic count_9, count_10;
logic enable_1, output_enable;

// Input/output data
logic [7:0] data_1;

// Synthesis-friendly signed logic for multiplier inputs instead of integer
logic signed [31:0] Cb2_mul_input, Cb3_mul_input, Cb4_mul_input, Cb5_mul_input;
logic signed [31:0] Cb6_mul_input, Cb7_mul_input, Cb8_mul_input;
logic signed [31:0] Ti2_mul_input, Ti3_mul_input, Ti4_mul_input, Ti5_mul_input;
logic signed [31:0] Ti6_mul_input, Ti7_mul_input, Ti8_mul_input;

// Declare constants as signed 16-bit logic (suitable for DCT coefficients)
logic signed [15:0] T1, T21, T22, T23, T24, T25, T26, T27, T28;
logic signed [15:0] T31, T32, T33, T34, T52;

logic signed [15:0] Ti1, Ti21, Ti22, Ti23, Ti24, Ti25, Ti26, Ti27, Ti28;
logic signed [15:0] Ti31, Ti32, Ti33, Ti34, Ti52;

always_ff @(posedge clk) begin
    // DCT matrix constants (scaled values)
    T1  <= 16'sd5793;   // ~0.3536
    T21 <= 16'sd8035;   // ~0.4904
    T22 <= 16'sd6811;   // ~0.4157
    T23 <= 16'sd4551;   // ~0.2778
    T24 <= 16'sd1598;   // ~0.0975
    T25 <= -16'sd1598;  // ~-0.0975
    T26 <= -16'sd4551;  // ~-0.2778
    T27 <= -16'sd6811;  // ~-0.4157
    T28 <= -16'sd8035;  // ~-0.4904
    T31 <= 16'sd7568;   // ~0.4619
    T32 <= 16'sd3135;   // ~0.1913
    T33 <= -16'sd3135;  // ~-0.1913
    T34 <= -16'sd7568;  // ~-0.4619
    T52 <= -16'sd5793;  // ~-0.3536
end

always_ff @(posedge clk) begin
    // Inverse DCT matrix constants (same values)
    Ti1  <= 16'sd5793;
    Ti21 <= 16'sd8035;
    Ti22 <= 16'sd6811;
    Ti23 <= 16'sd4551;
    Ti24 <= 16'sd1598;
    Ti25 <= -16'sd1598;
    Ti26 <= -16'sd4551;
    Ti27 <= -16'sd6811;
    Ti28 <= -16'sd8035;
    Ti31 <= 16'sd7568;
    Ti32 <= 16'sd3135;
    Ti33 <= -16'sd3135;
    Ti34 <= -16'sd7568;
    Ti52 <= -16'sd5793;
end

// Converted from Verilog to SystemVerilog
always_ff @(posedge clk)
begin
	if (rst) begin
 		Z_temp_11 <= 0; Z_temp_12 <= 0; Z_temp_13 <= 0; Z_temp_14 <= 0;
		Z_temp_15 <= 0; Z_temp_16 <= 0; Z_temp_17 <= 0; Z_temp_18 <= 0;
		Z_temp_21 <= 0; Z_temp_22 <= 0; Z_temp_23 <= 0; Z_temp_24 <= 0;
		Z_temp_25 <= 0; Z_temp_26 <= 0; Z_temp_27 <= 0; Z_temp_28 <= 0;
		Z_temp_31 <= 0; Z_temp_32 <= 0; Z_temp_33 <= 0; Z_temp_34 <= 0;
		Z_temp_35 <= 0; Z_temp_36 <= 0; Z_temp_37 <= 0; Z_temp_38 <= 0;
		Z_temp_41 <= 0; Z_temp_42 <= 0; Z_temp_43 <= 0; Z_temp_44 <= 0;
		Z_temp_45 <= 0; Z_temp_46 <= 0; Z_temp_47 <= 0; Z_temp_48 <= 0;
		Z_temp_51 <= 0; Z_temp_52 <= 0; Z_temp_53 <= 0; Z_temp_54 <= 0;
		Z_temp_55 <= 0; Z_temp_56 <= 0; Z_temp_57 <= 0; Z_temp_58 <= 0;
		Z_temp_61 <= 0; Z_temp_62 <= 0; Z_temp_63 <= 0; Z_temp_64 <= 0;
		Z_temp_65 <= 0; Z_temp_66 <= 0; Z_temp_67 <= 0; Z_temp_68 <= 0;
		Z_temp_71 <= 0; Z_temp_72 <= 0; Z_temp_73 <= 0; Z_temp_74 <= 0;
		Z_temp_75 <= 0; Z_temp_76 <= 0; Z_temp_77 <= 0; Z_temp_78 <= 0;
		Z_temp_81 <= 0; Z_temp_82 <= 0; Z_temp_83 <= 0; Z_temp_84 <= 0;
		Z_temp_85 <= 0; Z_temp_86 <= 0; Z_temp_87 <= 0; Z_temp_88 <= 0;
	end
	else if (enable_1 & count_8) begin
		Z_temp_11 <= Cb11_final_4 * Ti1; Z_temp_12 <= Cb11_final_4 * Ti2_mul_input;
		Z_temp_13 <= Cb11_final_4 * Ti3_mul_input; Z_temp_14 <= Cb11_final_4 * Ti4_mul_input;
		Z_temp_15 <= Cb11_final_4 * Ti5_mul_input; Z_temp_16 <= Cb11_final_4 * Ti6_mul_input;
		Z_temp_17 <= Cb11_final_4 * Ti7_mul_input; Z_temp_18 <= Cb11_final_4 * Ti8_mul_input;
		Z_temp_21 <= Cb21_final_2 * Ti1; Z_temp_22 <= Cb21_final_2 * Ti2_mul_input;
		Z_temp_23 <= Cb21_final_2 * Ti3_mul_input; Z_temp_24 <= Cb21_final_2 * Ti4_mul_input;
		Z_temp_25 <= Cb21_final_2 * Ti5_mul_input; Z_temp_26 <= Cb21_final_2 * Ti6_mul_input;
		Z_temp_27 <= Cb21_final_2 * Ti7_mul_input; Z_temp_28 <= Cb21_final_2 * Ti8_mul_input;
		Z_temp_31 <= Cb31_final_2 * Ti1; Z_temp_32 <= Cb31_final_2 * Ti2_mul_input;
		Z_temp_33 <= Cb31_final_2 * Ti3_mul_input; Z_temp_34 <= Cb31_final_2 * Ti4_mul_input;
		Z_temp_35 <= Cb31_final_2 * Ti5_mul_input; Z_temp_36 <= Cb31_final_2 * Ti6_mul_input;
		Z_temp_37 <= Cb31_final_2 * Ti7_mul_input; Z_temp_38 <= Cb31_final_2 * Ti8_mul_input;
		Z_temp_41 <= Cb41_final_2 * Ti1; Z_temp_42 <= Cb41_final_2 * Ti2_mul_input;
		Z_temp_43 <= Cb41_final_2 * Ti3_mul_input; Z_temp_44 <= Cb41_final_2 * Ti4_mul_input;
		Z_temp_45 <= Cb41_final_2 * Ti5_mul_input; Z_temp_46 <= Cb41_final_2 * Ti6_mul_input;
		Z_temp_47 <= Cb41_final_2 * Ti7_mul_input; Z_temp_48 <= Cb41_final_2 * Ti8_mul_input;
		Z_temp_51 <= Cb51_final_2 * Ti1; Z_temp_52 <= Cb51_final_2 * Ti2_mul_input;
		Z_temp_53 <= Cb51_final_2 * Ti3_mul_input; Z_temp_54 <= Cb51_final_2 * Ti4_mul_input;
		Z_temp_55 <= Cb51_final_2 * Ti5_mul_input; Z_temp_56 <= Cb51_final_2 * Ti6_mul_input;
		Z_temp_57 <= Cb51_final_2 * Ti7_mul_input; Z_temp_58 <= Cb51_final_2 * Ti8_mul_input;
		Z_temp_61 <= Cb61_final_2 * Ti1; Z_temp_62 <= Cb61_final_2 * Ti2_mul_input;
		Z_temp_63 <= Cb61_final_2 * Ti3_mul_input; Z_temp_64 <= Cb61_final_2 * Ti4_mul_input;
		Z_temp_65 <= Cb61_final_2 * Ti5_mul_input; Z_temp_66 <= Cb61_final_2 * Ti6_mul_input;
		Z_temp_67 <= Cb61_final_2 * Ti7_mul_input; Z_temp_68 <= Cb61_final_2 * Ti8_mul_input;
		Z_temp_71 <= Cb71_final_2 * Ti1; Z_temp_72 <= Cb71_final_2 * Ti2_mul_input;
		Z_temp_73 <= Cb71_final_2 * Ti3_mul_input; Z_temp_74 <= Cb71_final_2 * Ti4_mul_input;
		Z_temp_75 <= Cb71_final_2 * Ti5_mul_input; Z_temp_76 <= Cb71_final_2 * Ti6_mul_input;
		Z_temp_77 <= Cb71_final_2 * Ti7_mul_input; Z_temp_78 <= Cb71_final_2 * Ti8_mul_input;
		Z_temp_81 <= Cb81_final_2 * Ti1; Z_temp_82 <= Cb81_final_2 * Ti2_mul_input;
		Z_temp_83 <= Cb81_final_2 * Ti3_mul_input; Z_temp_84 <= Cb81_final_2 * Ti4_mul_input;
		Z_temp_85 <= Cb81_final_2 * Ti5_mul_input; Z_temp_86 <= Cb81_final_2 * Ti6_mul_input;
		Z_temp_87 <= Cb81_final_2 * Ti7_mul_input; Z_temp_88 <= Cb81_final_2 * Ti8_mul_input;
	end
end


always_ff @(posedge clk)
begin
	if (rst) begin
		Z11 <= 0; Z12 <= 0; Z13 <= 0; Z14 <= 0; Z15 <= 0; Z16 <= 0; Z17 <= 0; Z18 <= 0;
		Z21 <= 0; Z22 <= 0; Z23 <= 0; Z24 <= 0; Z25 <= 0; Z26 <= 0; Z27 <= 0; Z28 <= 0;
		Z31 <= 0; Z32 <= 0; Z33 <= 0; Z34 <= 0; Z35 <= 0; Z36 <= 0; Z37 <= 0; Z38 <= 0;
		Z41 <= 0; Z42 <= 0; Z43 <= 0; Z44 <= 0; Z45 <= 0; Z46 <= 0; Z47 <= 0; Z48 <= 0;
		Z51 <= 0; Z52 <= 0; Z53 <= 0; Z54 <= 0; Z55 <= 0; Z56 <= 0; Z57 <= 0; Z58 <= 0;
		Z61 <= 0; Z62 <= 0; Z63 <= 0; Z64 <= 0; Z65 <= 0; Z66 <= 0; Z67 <= 0; Z68 <= 0;
		Z71 <= 0; Z72 <= 0; Z73 <= 0; Z74 <= 0; Z75 <= 0; Z76 <= 0; Z77 <= 0; Z78 <= 0;
		Z81 <= 0; Z82 <= 0; Z83 <= 0; Z84 <= 0; Z85 <= 0; Z86 <= 0; Z87 <= 0; Z88 <= 0;
	end
	else if (count_8 & count_of == 1) begin
		Z11 <= 0; Z12 <= 0; Z13 <= 0; Z14 <= 0;
		Z15 <= 0; Z16 <= 0; Z17 <= 0; Z18 <= 0;
		Z21 <= 0; Z22 <= 0; Z23 <= 0; Z24 <= 0;
		Z25 <= 0; Z26 <= 0; Z27 <= 0; Z28 <= 0;
		Z31 <= 0; Z32 <= 0; Z33 <= 0; Z34 <= 0;
		Z35 <= 0; Z36 <= 0; Z37 <= 0; Z38 <= 0;
		Z41 <= 0; Z42 <= 0; Z43 <= 0; Z44 <= 0;
		Z45 <= 0; Z46 <= 0; Z47 <= 0; Z48 <= 0;
		Z51 <= 0; Z52 <= 0; Z53 <= 0; Z54 <= 0;
		Z55 <= 0; Z56 <= 0; Z57 <= 0; Z58 <= 0;
		Z61 <= 0; Z62 <= 0; Z63 <= 0; Z64 <= 0;
		Z65 <= 0; Z66 <= 0; Z67 <= 0; Z68 <= 0;
		Z71 <= 0; Z72 <= 0; Z73 <= 0; Z74 <= 0;
		Z75 <= 0; Z76 <= 0; Z77 <= 0; Z78 <= 0;
		Z81 <= 0; Z82 <= 0; Z83 <= 0; Z84 <= 0;
		Z85 <= 0; Z86 <= 0; Z87 <= 0; Z88 <= 0;
	end
	else if (enable & count_9) begin
		Z11 <= Z_temp_11 + Z11; Z12 <= Z_temp_12 + Z12; Z13 <= Z_temp_13 + Z13; Z14 <= Z_temp_14 + Z14;
		Z15 <= Z_temp_15 + Z15; Z16 <= Z_temp_16 + Z16; Z17 <= Z_temp_17 + Z17; Z18 <= Z_temp_18 + Z18;
		Z21 <= Z_temp_21 + Z21; Z22 <= Z_temp_22 + Z22; Z23 <= Z_temp_23 + Z23; Z24 <= Z_temp_24 + Z24;
		Z25 <= Z_temp_25 + Z25; Z26 <= Z_temp_26 + Z26; Z27 <= Z_temp_27 + Z27; Z28 <= Z_temp_28 + Z28;
		Z31 <= Z_temp_31 + Z31; Z32 <= Z_temp_32 + Z32; Z33 <= Z_temp_33 + Z33; Z34 <= Z_temp_34 + Z34;
		Z35 <= Z_temp_35 + Z35; Z36 <= Z_temp_36 + Z36; Z37 <= Z_temp_37 + Z37; Z38 <= Z_temp_38 + Z38;
		Z41 <= Z_temp_41 + Z41; Z42 <= Z_temp_42 + Z42; Z43 <= Z_temp_43 + Z43; Z44 <= Z_temp_44 + Z44;
		Z45 <= Z_temp_45 + Z45; Z46 <= Z_temp_46 + Z46; Z47 <= Z_temp_47 + Z47; Z48 <= Z_temp_48 + Z48;
		Z51 <= Z_temp_51 + Z51; Z52 <= Z_temp_52 + Z52; Z53 <= Z_temp_53 + Z53; Z54 <= Z_temp_54 + Z54;
		Z55 <= Z_temp_55 + Z55; Z56 <= Z_temp_56 + Z56; Z57 <= Z_temp_57 + Z57; Z58 <= Z_temp_58 + Z58;
		Z61 <= Z_temp_61 + Z61; Z62 <= Z_temp_62 + Z62; Z63 <= Z_temp_63 + Z63; Z64 <= Z_temp_64 + Z64;
		Z65 <= Z_temp_65 + Z65; Z66 <= Z_temp_66 + Z66; Z67 <= Z_temp_67 + Z67; Z68 <= Z_temp_68 + Z68;
		Z71 <= Z_temp_71 + Z71; Z72 <= Z_temp_72 + Z72; Z73 <= Z_temp_73 + Z73; Z74 <= Z_temp_74 + Z74;
		Z75 <= Z_temp_75 + Z75; Z76 <= Z_temp_76 + Z76; Z77 <= Z_temp_77 + Z77; Z78 <= Z_temp_78 + Z78;
		Z81 <= Z_temp_81 + Z81; Z82 <= Z_temp_82 + Z82; Z83 <= Z_temp_83 + Z83; Z84 <= Z_temp_84 + Z84;
		Z85 <= Z_temp_85 + Z85; Z86 <= Z_temp_86 + Z86; Z87 <= Z_temp_87 + Z87; Z88 <= Z_temp_88 + Z88;
	end	
end

// Converted from Verilog to SystemVerilog
// Rounds and stores final 8x8 IDCT matrix result after normalization
always_ff @(posedge clk)
begin
	if (rst) begin
		// Reset all Z*_final values to zero
		Z11_final <= 0; Z12_final <= 0; Z13_final <= 0; Z14_final <= 0;
		Z15_final <= 0; Z16_final <= 0; Z17_final <= 0; Z18_final <= 0;
		Z21_final <= 0; Z22_final <= 0; Z23_final <= 0; Z24_final <= 0;
		Z25_final <= 0; Z26_final <= 0; Z27_final <= 0; Z28_final <= 0;
		Z31_final <= 0; Z32_final <= 0; Z33_final <= 0; Z34_final <= 0;
		Z35_final <= 0; Z36_final <= 0; Z37_final <= 0; Z38_final <= 0;
		Z41_final <= 0; Z42_final <= 0; Z43_final <= 0; Z44_final <= 0;
		Z45_final <= 0; Z46_final <= 0; Z47_final <= 0; Z48_final <= 0;
		Z51_final <= 0; Z52_final <= 0; Z53_final <= 0; Z54_final <= 0;
		Z55_final <= 0; Z56_final <= 0; Z57_final <= 0; Z58_final <= 0;
		Z61_final <= 0; Z62_final <= 0; Z63_final <= 0; Z64_final <= 0;
		Z65_final <= 0; Z66_final <= 0; Z67_final <= 0; Z68_final <= 0;
		Z71_final <= 0; Z72_final <= 0; Z73_final <= 0; Z74_final <= 0;
		Z75_final <= 0; Z76_final <= 0; Z77_final <= 0; Z78_final <= 0;
		Z81_final <= 0; Z82_final <= 0; Z83_final <= 0; Z84_final <= 0;
		Z85_final <= 0; Z86_final <= 0; Z87_final <= 0; Z88_final <= 0;
	end
	else if (count_10 & count_of == 0) begin
		// Round and normalize 25-bit values to final output by taking bits [24:14]
		// If bit 13 is 1, add 1 to apply rounding
		Z11_final <= Z11[13] ? Z11[24:14] + 1 : Z11[24:14];
		Z12_final <= Z12[13] ? Z12[24:14] + 1 : Z12[24:14];
		Z13_final <= Z13[13] ? Z13[24:14] + 1 : Z13[24:14];
		Z14_final <= Z14[13] ? Z14[24:14] + 1 : Z14[24:14];
		Z15_final <= Z15[13] ? Z15[24:14] + 1 : Z15[24:14];
		Z16_final <= Z16[13] ? Z16[24:14] + 1 : Z16[24:14];
		Z17_final <= Z17[13] ? Z17[24:14] + 1 : Z17[24:14];
		Z18_final <= Z18[13] ? Z18[24:14] + 1 : Z18[24:14]; 
		Z21_final <= Z21[13] ? Z21[24:14] + 1 : Z21[24:14];
		Z22_final <= Z22[13] ? Z22[24:14] + 1 : Z22[24:14];
		Z23_final <= Z23[13] ? Z23[24:14] + 1 : Z23[24:14];
		Z24_final <= Z24[13] ? Z24[24:14] + 1 : Z24[24:14];
		Z25_final <= Z25[13] ? Z25[24:14] + 1 : Z25[24:14];
		Z26_final <= Z26[13] ? Z26[24:14] + 1 : Z26[24:14];
		Z27_final <= Z27[13] ? Z27[24:14] + 1 : Z27[24:14];
		Z28_final <= Z28[13] ? Z28[24:14] + 1 : Z28[24:14]; 
		Z31_final <= Z31[13] ? Z31[24:14] + 1 : Z31[24:14];
		Z32_final <= Z32[13] ? Z32[24:14] + 1 : Z32[24:14];
		Z33_final <= Z33[13] ? Z33[24:14] + 1 : Z33[24:14];
		Z34_final <= Z34[13] ? Z34[24:14] + 1 : Z34[24:14];
		Z35_final <= Z35[13] ? Z35[24:14] + 1 : Z35[24:14];
		Z36_final <= Z36[13] ? Z36[24:14] + 1 : Z36[24:14];
		Z37_final <= Z37[13] ? Z37[24:14] + 1 : Z37[24:14];
		Z38_final <= Z38[13] ? Z38[24:14] + 1 : Z38[24:14]; 
		Z41_final <= Z41[13] ? Z41[24:14] + 1 : Z41[24:14];
		Z42_final <= Z42[13] ? Z42[24:14] + 1 : Z42[24:14];
		Z43_final <= Z43[13] ? Z43[24:14] + 1 : Z43[24:14];
		Z44_final <= Z44[13] ? Z44[24:14] + 1 : Z44[24:14];
		Z45_final <= Z45[13] ? Z45[24:14] + 1 : Z45[24:14];
		Z46_final <= Z46[13] ? Z46[24:14] + 1 : Z46[24:14];
		Z47_final <= Z47[13] ? Z47[24:14] + 1 : Z47[24:14];
		Z48_final <= Z48[13] ? Z48[24:14] + 1 : Z48[24:14]; 
		Z51_final <= Z51[13] ? Z51[24:14] + 1 : Z51[24:14];
		Z52_final <= Z52[13] ? Z52[24:14] + 1 : Z52[24:14];
		Z53_final <= Z53[13] ? Z53[24:14] + 1 : Z53[24:14];
		Z54_final <= Z54[13] ? Z54[24:14] + 1 : Z54[24:14];
		Z55_final <= Z55[13] ? Z55[24:14] + 1 : Z55[24:14];
		Z56_final <= Z56[13] ? Z56[24:14] + 1 : Z56[24:14];
		Z57_final <= Z57[13] ? Z57[24:14] + 1 : Z57[24:14];
		Z58_final <= Z58[13] ? Z58[24:14] + 1 : Z58[24:14]; 
		Z61_final <= Z61[13] ? Z61[24:14] + 1 : Z61[24:14];
		Z62_final <= Z62[13] ? Z62[24:14] + 1 : Z62[24:14];
		Z63_final <= Z63[13] ? Z63[24:14] + 1 : Z63[24:14];
		Z64_final <= Z64[13] ? Z64[24:14] + 1 : Z64[24:14];
		Z65_final <= Z65[13] ? Z65[24:14] + 1 : Z65[24:14];
		Z66_final <= Z66[13] ? Z66[24:14] + 1 : Z66[24:14];
		Z67_final <= Z67[13] ? Z67[24:14] + 1 : Z67[24:14];
		Z68_final <= Z68[13] ? Z68[24:14] + 1 : Z68[24:14]; 
		Z71_final <= Z71[13] ? Z71[24:14] + 1 : Z71[24:14];
		Z72_final <= Z72[13] ? Z72[24:14] + 1 : Z72[24:14];
		Z73_final <= Z73[13] ? Z73[24:14] + 1 : Z73[24:14];
		Z74_final <= Z74[13] ? Z74[24:14] + 1 : Z74[24:14];
		Z75_final <= Z75[13] ? Z75[24:14] + 1 : Z75[24:14];
		Z76_final <= Z76[13] ? Z76[24:14] + 1 : Z76[24:14];
		Z77_final <= Z77[13] ? Z77[24:14] + 1 : Z77[24:14];
		Z78_final <= Z78[13] ? Z78[24:14] + 1 : Z78[24:14]; 
		Z81_final <= Z81[13] ? Z81[24:14] + 1 : Z81[24:14];
		Z82_final <= Z82[13] ? Z82[24:14] + 1 : Z82[24:14];
		Z83_final <= Z83[13] ? Z83[24:14] + 1 : Z83[24:14];
		Z84_final <= Z84[13] ? Z84[24:14] + 1 : Z84[24:14];
		Z85_final <= Z85[13] ? Z85[24:14] + 1 : Z85[24:14];
		Z86_final <= Z86[13] ? Z86[24:14] + 1 : Z86[24:14];
		Z87_final <= Z87[13] ? Z87[24:14] + 1 : Z87[24:14];
		Z88_final <= Z88[13] ? Z88[24:14] + 1 : Z88[24:14]; 
	end
end


// output_enable signals the next block, the quantizer, that the input data is ready
always_ff @(posedge clk) begin
	if (rst) 
		output_enable <= 0;
	else if (!enable_1)
		output_enable <= 0;
	else if (count_10 == 0 || count_of) // disable when not at proper count
		output_enable <= 0;
	else if (count_10 && count_of == 0) // enable when timing is aligned
		output_enable <= 1;
end

// Multiply incoming data by constant T1 on enable
always_ff @(posedge clk) begin
	if (rst)
		Cb_temp_11 <= 0;
	else if (enable)
		Cb_temp_11 <= data_in * T1;
end  

// Accumulate Cb_temp_11 values into Cb11
always_ff @(posedge clk) begin
	if (rst)
		Cb11 <= 0;
	else if (count == 1 && enable == 1)
		Cb11 <= Cb_temp_11; // initialize
	else if (enable)
		Cb11 <= Cb_temp_11 + Cb11; // accumulate
end

// Multiply data_1 with the respective CbX_mul_input values when enable_1 is active
always_ff @(posedge clk) begin
	if (rst) begin
		// Reset all temporary Cb multiplication results
		Cb_temp_21 <= 0;
		Cb_temp_31 <= 0;
		Cb_temp_41 <= 0;
		Cb_temp_51 <= 0;
		Cb_temp_61 <= 0;
		Cb_temp_71 <= 0;
		Cb_temp_81 <= 0;
	end
	else if (!enable_1) begin
		// Clear outputs when not enabled
		Cb_temp_21 <= 0;
		Cb_temp_31 <= 0;
		Cb_temp_41 <= 0;
		Cb_temp_51 <= 0;
		Cb_temp_61 <= 0;
		Cb_temp_71 <= 0;
		Cb_temp_81 <= 0;
	end
	else begin
		// Perform multiplication with corresponding coefficients
		Cb_temp_21 <= data_1 * Cb2_mul_input; 
		Cb_temp_31 <= data_1 * Cb3_mul_input; 
		Cb_temp_41 <= data_1 * Cb4_mul_input; 
		Cb_temp_51 <= data_1 * Cb5_mul_input; 
		Cb_temp_61 <= data_1 * Cb6_mul_input; 
		Cb_temp_71 <= data_1 * Cb7_mul_input; 
		Cb_temp_81 <= data_1 * Cb8_mul_input; 
	end
end

// Accumulate results from temporary Cb calculations when enabled
always_ff @(posedge clk) begin
	if (rst) begin
		// Reset all Cb accumulators
		Cb21 <= 0;
		Cb31 <= 0;
		Cb41 <= 0;
		Cb51 <= 0;
		Cb61 <= 0;
		Cb71 <= 0;
		Cb81 <= 0;
	end
	else if (!enable_1) begin
		// Clear if not enabled
		Cb21 <= 0;
		Cb31 <= 0;
		Cb41 <= 0;
		Cb51 <= 0;
		Cb61 <= 0;
		Cb71 <= 0;
		Cb81 <= 0;
	end
	else begin
		// Accumulate partial multiplication results
		Cb21 <= Cb_temp_21 + Cb21;
		Cb31 <= Cb_temp_31 + Cb31;
		Cb41 <= Cb_temp_41 + Cb41;
		Cb51 <= Cb_temp_51 + Cb51;
		Cb61 <= Cb_temp_61 + Cb61;
		Cb71 <= Cb_temp_71 + Cb71;
		Cb81 <= Cb_temp_81 + Cb81;
	end
end

// Main counters for pipeline stage delays and control
always_ff @(posedge clk) begin
	if (rst) begin
		// Reset all pipeline delay counters
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
		// Clear counters when not enabled
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
	else begin
		// Shift pipeline delay values
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

// Assert count_1 when count reaches 7 and enable is active
always_ff @(posedge clk) begin
	if (rst)
		count_1 <= 0;
	else if (count != 7 || !enable)
		count_1 <= 0;
	else
		count_1 <= 1;
end

// Overflow counter and backup for tracking block cycles
always_ff @(posedge clk) begin
	if (rst) begin
		count_of       <= 0;
		count_of_copy  <= 0;
	end
	else if (!enable) begin
		count_of       <= 0;
		count_of_copy  <= 0;
	end
	else if (count_1) begin
		count_of       <= count_of + 1;
		count_of_copy  <= count_of_copy + 1;
	end
end

// Final adjustment for Cb11 after subtracting DC bias (128 × 8 × T1 = 5932032)
always_ff @(posedge clk) begin
	if (rst) begin
		Cb11_final <= 0;
	end
	else if (count_3 && enable_1) begin
		Cb11_final <= Cb11 - 25'd5932032;
		// Centering pixel values around 0 by subtracting 128 offset
		// 128 × 8 × T1 = 5932032 is precomputed offset for DC component
	end
end

// Final values for Cb rows 2 to 8 and holding previous values for potential reuse
always_ff @(posedge clk) begin
	if (rst) begin
		Cb21_final <= 0; Cb21_final_prev <= 0;
		Cb31_final <= 0; Cb31_final_prev <= 0;
		Cb41_final <= 0; Cb41_final_prev <= 0;
		Cb51_final <= 0; Cb51_final_prev <= 0;
		Cb61_final <= 0; Cb61_final_prev <= 0;
		Cb71_final <= 0; Cb71_final_prev <= 0;
		Cb81_final <= 0; Cb81_final_prev <= 0;
	end
	else if (!enable_1) begin
		// Clear on disable
		Cb21_final <= 0; Cb21_final_prev <= 0;
		Cb31_final <= 0; Cb31_final_prev <= 0;
		Cb41_final <= 0; Cb41_final_prev <= 0;
		Cb51_final <= 0; Cb51_final_prev <= 0;
		Cb61_final <= 0; Cb61_final_prev <= 0;
		Cb71_final <= 0; Cb71_final_prev <= 0;
		Cb81_final <= 0; Cb81_final_prev <= 0;
	end
	else if (count_4 && enable_1) begin
		// Store current and previous values for rows 2–8
		Cb21_final <= Cb21; Cb21_final_prev <= Cb21_final;
		Cb31_final <= Cb31; Cb31_final_prev <= Cb31_final;
		Cb41_final <= Cb41; Cb41_final_prev <= Cb41_final;
		Cb51_final <= Cb51; Cb51_final_prev <= Cb51_final;
		Cb61_final <= Cb61; Cb61_final_prev <= Cb61_final;
		Cb71_final <= Cb71; Cb71_final_prev <= Cb71_final;
		Cb81_final <= Cb81; Cb81_final_prev <= Cb81_final;
	end
end


// Calculate difference between current and previous final values of rows 2 to 8
always_ff @(posedge clk) begin
	if (rst) begin
		Cb21_final_diff <= 0;
		Cb31_final_diff <= 0;
		Cb41_final_diff <= 0;
		Cb51_final_diff <= 0;
		Cb61_final_diff <= 0;
		Cb71_final_diff <= 0;
		Cb81_final_diff <= 0;
	end
	else if (count_5 && enable_1) begin
		// Difference computation: current - previous
		Cb21_final_diff <= Cb21_final - Cb21_final_prev;
		Cb31_final_diff <= Cb31_final - Cb31_final_prev;
		Cb41_final_diff <= Cb41_final - Cb41_final_prev;
		Cb51_final_diff <= Cb51_final - Cb51_final_prev;
		Cb61_final_diff <= Cb61_final - Cb61_final_prev;
		Cb71_final_diff <= Cb71_final - Cb71_final_prev;
		Cb81_final_diff <= Cb81_final - Cb81_final_prev;
	end
end

// Select T2* multiplier based on count for second row
always_ff @(posedge clk) begin
	case (count)
		3'b000: Cb2_mul_input <= T21;
		3'b001: Cb2_mul_input <= T22;
		3'b010: Cb2_mul_input <= T23;
		3'b011: Cb2_mul_input <= T24;
		3'b100: Cb2_mul_input <= T25;
		3'b101: Cb2_mul_input <= T26;
		3'b110: Cb2_mul_input <= T27;
		3'b111: Cb2_mul_input <= T28;
	endcase
end

// Select T3* multiplier based on count for third row (symmetric values)
always_ff @(posedge clk) begin
	case (count)
		3'b000: Cb3_mul_input <= T31;
		3'b001: Cb3_mul_input <= T32;
		3'b010: Cb3_mul_input <= T33;
		3'b011: Cb3_mul_input <= T34;
		3'b100: Cb3_mul_input <= T34;
		3'b101: Cb3_mul_input <= T33;
		3'b110: Cb3_mul_input <= T32;
		3'b111: Cb3_mul_input <= T31;
	endcase
end

// Select T4* multiplier based on count for fourth row (nonlinear pattern)
always_ff @(posedge clk) begin
	case (count)
		3'b000: Cb4_mul_input <= T22;
		3'b001: Cb4_mul_input <= T25;
		3'b010: Cb4_mul_input <= T28;
		3'b011: Cb4_mul_input <= T26;
		3'b100: Cb4_mul_input <= T23;
		3'b101: Cb4_mul_input <= T21;
		3'b110: Cb4_mul_input <= T24;
		3'b111: Cb4_mul_input <= T27;
	endcase
end

// Select T5* multiplier based on count for fifth row (symmetric repeated pattern)
always_ff @(posedge clk) begin
	case (count)
		3'b000: Cb5_mul_input <= T1;
		3'b001: Cb5_mul_input <= T52;
		3'b010: Cb5_mul_input <= T52;
		3'b011: Cb5_mul_input <= T1;
		3'b100: Cb5_mul_input <= T1;
		3'b101: Cb5_mul_input <= T52;
		3'b110: Cb5_mul_input <= T52;
		3'b111: Cb5_mul_input <= T1;
	endcase
end

// Select T6* multiplier based on count for sixth row (nonlinear mapping)
always_ff @(posedge clk) begin
	case (count)
		3'b000: Cb6_mul_input <= T23;
		3'b001: Cb6_mul_input <= T28;
		3'b010: Cb6_mul_input <= T24;
		3'b011: Cb6_mul_input <= T22;
		3'b100: Cb6_mul_input <= T27;
		3'b101: Cb6_mul_input <= T25;
		3'b110: Cb6_mul_input <= T21;
		3'b111: Cb6_mul_input <= T26;
	endcase
end

// Select T7* multiplier based on count for seventh row (symmetric pattern)
always_ff @(posedge clk) begin
	case (count)
		3'b000: Cb7_mul_input <= T32;
		3'b001: Cb7_mul_input <= T34;
		3'b010: Cb7_mul_input <= T31;
		3'b011: Cb7_mul_input <= T33;
		3'b100: Cb7_mul_input <= T33;
		3'b101: Cb7_mul_input <= T31;
		3'b110: Cb7_mul_input <= T34;
		3'b111: Cb7_mul_input <= T32;
	endcase
end


// Select T8* multiplier based on count for the eighth row
always_ff @(posedge clk) begin
	case (count)
		3'b000: Cb8_mul_input <= T24;
		3'b001: Cb8_mul_input <= T26;
		3'b010: Cb8_mul_input <= T22;
		3'b011: Cb8_mul_input <= T28;
		3'b100: Cb8_mul_input <= T21;
		3'b101: Cb8_mul_input <= T27;
		3'b110: Cb8_mul_input <= T23;
		3'b111: Cb8_mul_input <= T25;
	endcase
end

// Inverse DCT second row multipliers based on count_of_copy
always_ff @(posedge clk) begin
	case (count_of_copy)
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

// Inverse DCT third row multipliers with symmetry around center
always_ff @(posedge clk) begin
	case (count_of_copy)
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

// Inverse DCT fourth row multiplier selection based on count_of_copy
always_ff @(posedge clk) begin
	case (count_of_copy)
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

// Inverse DCT fifth row: repeated Ti1 and Ti52 pattern
always_ff @(posedge clk) begin
	case (count_of_copy)
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

// Inverse DCT sixth row multiplier selection
always_ff @(posedge clk) begin
	case (count_of_copy)
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


// Inverse DCT seventh row multiplier selection
always_ff @(posedge clk) begin
	case (count_of_copy)
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

// Inverse DCT eighth row multiplier selection
always_ff @(posedge clk) begin
	case (count_of_copy)
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

//-----------------------------
// Rounding Stage
//-----------------------------
always_ff @(posedge clk) begin
	if (rst) begin
		data_1 <= 0;
		Cb11_final_1 <= 0; Cb21_final_1 <= 0; Cb31_final_1 <= 0; Cb41_final_1 <= 0;
		Cb51_final_1 <= 0; Cb61_final_1 <= 0; Cb71_final_1 <= 0; Cb81_final_1 <= 0;
		Cb11_final_2 <= 0; Cb21_final_2 <= 0; Cb31_final_2 <= 0; Cb41_final_2 <= 0;
		Cb51_final_2 <= 0; Cb61_final_2 <= 0; Cb71_final_2 <= 0; Cb81_final_2 <= 0;
		Cb11_final_3 <= 0; Cb11_final_4 <= 0;
	end else if (enable) begin
		// Pass input to next stage
		data_1 <= data_in;

		// Round and sign-extend Cb11
		Cb11_final_1 <= Cb11_final[13] ? Cb11_final[24:14] + 1 : Cb11_final[24:14];
		Cb11_final_2[31:11] <= Cb11_final_1[10] ? 21'h1FFFFF : 21'd0;  // sign extension
		Cb11_final_2[10:0] <= Cb11_final_1;
		Cb11_final_3 <= Cb11_final_2;
		Cb11_final_4 <= Cb11_final_3;

		// Round and sign-extend Cb21
		Cb21_final_1 <= Cb21_final_diff[13] ? Cb21_final_diff[24:14] + 1 : Cb21_final_diff[24:14];
		Cb21_final_2[31:11] <= Cb21_final_1[10] ? 21'h1FFFFF : 21'd0;
		Cb21_final_2[10:0] <= Cb21_final_1;

		// Round and sign-extend Cb31
		Cb31_final_1 <= Cb31_final_diff[13] ? Cb31_final_diff[24:14] + 1 : Cb31_final_diff[24:14];
		Cb31_final_2[31:11] <= Cb31_final_1[10] ? 21'h1FFFFF : 21'd0;
		Cb31_final_2[10:0] <= Cb31_final_1;

		// Round and sign-extend Cb41
		Cb41_final_1 <= Cb41_final_diff[13] ? Cb41_final_diff[24:14] + 1 : Cb41_final_diff[24:14];
		Cb41_final_2[31:11] <= Cb41_final_1[10] ? 21'h1FFFFF : 21'd0;
		Cb41_final_2[10:0] <= Cb41_final_1;

		// Round and sign-extend Cb51
		Cb51_final_1 <= Cb51_final_diff[13] ? Cb51_final_diff[24:14] + 1 : Cb51_final_diff[24:14];
		Cb51_final_2[31:11] <= Cb51_final_1[10] ? 21'h1FFFFF : 21'd0;
		Cb51_final_2[10:0] <= Cb51_final_1;

		// Round and sign-extend Cb61
		Cb61_final_1 <= Cb61_final_diff[13] ? Cb61_final_diff[24:14] + 1 : Cb61_final_diff[24:14];
		Cb61_final_2[31:11] <= Cb61_final_1[10] ? 21'h1FFFFF : 21'd0;
		Cb61_final_2[10:0] <= Cb61_final_1;

		// Round and sign-extend Cb71
		Cb71_final_1 <= Cb71_final_diff[13] ? Cb71_final_diff[24:14] + 1 : Cb71_final_diff[24:14];
		Cb71_final_2[31:11] <= Cb71_final_1[10] ? 21'h1FFFFF : 21'd0;
		Cb71_final_2[10:0] <= Cb71_final_1;

		// Round and sign-extend Cb81
		Cb81_final_1 <= Cb81_final_diff[13] ? Cb81_final_diff[24:14] + 1 : Cb81_final_diff[24:14];
		Cb81_final_2[31:11] <= Cb81_final_1[10] ? 21'h1FFFFF : 21'd0;
		Cb81_final_2[10:0] <= Cb81_final_1;

		/* Notes:
		 - Bit 13 is the rounding bit (LSB of integer part)
		 - Bits 24:14 hold the integer result after fixed-point scaling
		 - Sign-extension preserves negative values in 2's complement form
		*/
	end
end

//-----------------------------
// Pipeline Control Enable Signal (1-cycle delay)
//-----------------------------
always_ff @(posedge clk) begin
	if (rst)
		enable_1 <= 1'b0;
	else
		enable_1 <= enable;
end

endmodule
