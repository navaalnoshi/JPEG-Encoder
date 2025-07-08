`timescale 1ns / 100ps

module y_dct(
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic  [7:0] data_in,
    output logic [10:0] Z11_final, Z12_final, Z13_final, Z14_final,
    output logic [10:0] Z15_final, Z16_final, Z17_final, Z18_final,
    output logic [10:0] Z21_final, Z22_final, Z23_final, Z24_final,
    output logic [10:0] Z25_final, Z26_final, Z27_final, Z28_final,
    output logic [10:0] Z31_final, Z32_final, Z33_final, Z34_final,
    output logic [10:0] Z35_final, Z36_final, Z37_final, Z38_final,
    output logic [10:0] Z41_final, Z42_final, Z43_final, Z44_final,
    output logic [10:0] Z45_final, Z46_final, Z47_final, Z48_final,
    output logic [10:0] Z51_final, Z52_final, Z53_final, Z54_final,
    output logic [10:0] Z55_final, Z56_final, Z57_final, Z58_final,
    output logic [10:0] Z61_final, Z62_final, Z63_final, Z64_final,
    output logic [10:0] Z65_final, Z66_final, Z67_final, Z68_final,
    output logic [10:0] Z71_final, Z72_final, Z73_final, Z74_final,
    output logic [10:0] Z75_final, Z76_final, Z77_final, Z78_final,
    output logic [10:0] Z81_final, Z82_final, Z83_final, Z84_final,
    output logic [10:0] Z85_final, Z86_final, Z87_final, Z88_final,
    output logic        output_enable
);

    // DCT matrix entries (fixed-point representation, scaled by 2^14 for .3536, .4904 etc.)
    localparam int T1  = 5793;  // .3536 * 2^14
    localparam int T21 = 8035;  // .4904 * 2^14
    localparam int T22 = 6811;  // .4157 * 2^14
    localparam int T23 = 4551;  // .2778 * 2^14
    localparam int T24 = 1598;  // .0975 * 2^14
    localparam int T25 = -1598; // -.0975 * 2^14
    localparam int T26 = -4551; // -.2778 * 2^14
    localparam int T27 = -6811; // -.4157 * 2^14
    localparam int T28 = -8035; // -.4904 * 2^14
    localparam int T31 = 7568;  // .4619 * 2^14
    localparam int T32 = 3135;  // .1913 * 2^14
    localparam int T33 = -3135; // -.1913 * 2^14
    localparam int T34 = -7568; // -.4619 * 2^14
    localparam int T52 = -5793; // -.3536 * 2^14

    // Inverse DCT matrix entries (same values as DCT matrix for this implementation)
    localparam int Ti1  = 5793;  // .3536 * 2^14
    localparam int Ti21 = 8035;  // .4904 * 2^14
    localparam int Ti22 = 6811;  // .4157 * 2^14
    localparam int Ti23 = 4551;  // .2778 * 2^14
    localparam int Ti24 = 1598;  // .0975 * 2^14
    localparam int Ti25 = -1598; // -.0975 * 2^14
    localparam int Ti26 = -4551; // -.2778 * 2^14
    localparam int Ti27 = -6811; // -.4157 * 2^14
    localparam int Ti28 = -8035; // -.4904 * 2^14
    localparam int Ti31 = 7568;  // .4619 * 2^14
    localparam int Ti32 = 3135;  // .1913 * 2^14
    localparam int Ti33 = -3135; // -.1913 * 2^14
    localparam int Ti34 = -7568; // -.4619 * 2^14
    localparam int Ti52 = -5793; // -.3536 * 2^14

    // Internal registers and wires
    logic signed [24:0] Y_temp_11;
    logic signed [24:0] Y11, Y21, Y31, Y41, Y51, Y61, Y71, Y81;
    logic signed [24:0] Y11_final;

    logic signed [31:0] Y_temp_21, Y_temp_31, Y_temp_41, Y_temp_51;
    logic signed [31:0] Y_temp_61, Y_temp_71, Y_temp_81;

    logic signed [31:0] Z_temp_11, Z_temp_12, Z_temp_13, Z_temp_14;
    logic signed [31:0] Z_temp_15, Z_temp_16, Z_temp_17, Z_temp_18;
    logic signed [31:0] Z_temp_21, Z_temp_22, Z_temp_23, Z_temp_24;
    logic signed [31:0] Z_temp_25, Z_temp_26, Z_temp_27, Z_temp_28;
    logic signed [31:0] Z_temp_31, Z_temp_32, Z_temp_33, Z_temp_34;
    logic signed [31:0] Z_temp_35, Z_temp_36, Z_temp_37, Z_temp_38;
    logic signed [31:0] Z_temp_41, Z_temp_42, Z_temp_43, Z_temp_44;
    logic signed [31:0] Z_temp_45, Z_temp_46, Z_temp_47, Z_temp_48;
    logic signed [31:0] Z_temp_51, Z_temp_52, Z_temp_53, Z_temp_54;
    logic signed [31:0] Z_temp_55, Z_temp_56, Z_temp_57, Z_temp_58;
    logic signed [31:0] Z_temp_61, Z_temp_62, Z_temp_63, Z_temp_64;
    logic signed [31:0] Z_temp_65, Z_temp_66, Z_temp_67, Z_temp_68;
    logic signed [31:0] Z_temp_71, Z_temp_72, Z_temp_73, Z_temp_74;
    logic signed [31:0] Z_temp_75, Z_temp_76, Z_temp_77, Z_temp_78;
    logic signed [31:0] Z_temp_81, Z_temp_82, Z_temp_83, Z_temp_84;
    logic signed [31:0] Z_temp_85, Z_temp_86, Z_temp_87, Z_temp_88;

    logic signed [26:0] Z11, Z12, Z13, Z14, Z15, Z16, Z17, Z18;
    logic signed [26:0] Z21, Z22, Z23, Z24, Z25, Z26, Z27, Z28;
    logic signed [26:0] Z31, Z32, Z33, Z34, Z35, Z36, Z37, Z38;
    logic signed [26:0] Z41, Z42, Z43, Z44, Z45, Z46, Z47, Z48;
    logic signed [26:0] Z51, Z52, Z53, Z54, Z55, Z56, Z57, Z58;
    logic signed [26:0] Z61, Z62, Z63, Z64, Z65, Z66, Z67, Z68;
    logic signed [26:0] Z71, Z72, Z73, Z74, Z75, Z76, Z77, Z78;
    logic signed [26:0] Z81, Z82, Z83, Z84, Z85, Z86, Z87, Z88;

    logic signed [31:0] Y11_final_2, Y21_final_2, Y11_final_3, Y11_final_4, Y31_final_2, Y41_final_2;
    logic signed [31:0] Y51_final_2, Y61_final_2, Y71_final_2, Y81_final_2;

    logic signed [12:0] Y11_final_1, Y21_final_1, Y31_final_1, Y41_final_1;
    logic signed [12:0] Y51_final_1, Y61_final_1, Y71_final_1, Y81_final_1;

    logic signed [24:0] Y21_final, Y31_final, Y41_final, Y51_final;
    logic signed [24:0] Y61_final, Y71_final, Y81_final;
    logic signed [24:0] Y21_final_prev, Y21_final_diff;
    logic signed [24:0] Y31_final_prev, Y31_final_diff;
    logic signed [24:0] Y41_final_prev, Y41_final_diff;
    logic signed [24:0] Y51_final_prev, Y51_final_diff;
    logic signed [24:0] Y61_final_prev, Y61_final_diff;
    logic signed [24:0] Y71_final_prev, Y71_final_diff;
    logic signed [24:0] Y81_final_prev, Y81_final_diff;

    logic [2:0] count;
    logic [2:0] count_of, count_of_copy;
    logic       count_1, count_3, count_4, count_5, count_6, count_7, count_8, enable_1;
    logic       count_9, count_10;
    logic [7:0] data_1;

    logic signed Y2_mul_input, Y3_mul_input, Y4_mul_input, Y5_mul_input;
    logic signed Y6_mul_input, Y7_mul_input, Y8_mul_input;
    logic signed Ti2_mul_input, Ti3_mul_input, Ti4_mul_input, Ti5_mul_input;
    logic signed Ti6_mul_input, Ti7_mul_input, Ti8_mul_input;

    // Registers for storing DCT matrix entries (already assigned as localparams, so these registers are redundant)
    // Removed to simplify. If these were meant to be changeable, they should be inputs.

    // Registers for storing Inverse DCT matrix entries (already assigned as localparams, so these registers are redundant)
    // Removed to simplify.

    // Z_temp_XX registers reset
    always_ff @(posedge clk or posedge rst) begin
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
        end else if (enable_1 && count_8) begin
            Z_temp_11 <= Y11_final_4 * Ti1; Z_temp_12 <= Y11_final_4 * Ti2_mul_input;
            Z_temp_13 <= Y11_final_4 * Ti3_mul_input; Z_temp_14 <= Y11_final_4 * Ti4_mul_input;
            Z_temp_15 <= Y11_final_4 * Ti5_mul_input; Z_temp_16 <= Y11_final_4 * Ti6_mul_input;
            Z_temp_17 <= Y11_final_4 * Ti7_mul_input; Z_temp_18 <= Y11_final_4 * Ti8_mul_input;
            Z_temp_21 <= Y21_final_2 * Ti1; Z_temp_22 <= Y21_final_2 * Ti2_mul_input;
            Z_temp_23 <= Y21_final_2 * Ti3_mul_input; Z_temp_24 <= Y21_final_2 * Ti4_mul_input;
            Z_temp_25 <= Y21_final_2 * Ti5_mul_input; Z_temp_26 <= Y21_final_2 * Ti6_mul_input;
            Z_temp_27 <= Y21_final_2 * Ti7_mul_input; Z_temp_28 <= Y21_final_2 * Ti8_mul_input;
            Z_temp_31 <= Y31_final_2 * Ti1; Z_temp_32 <= Y31_final_2 * Ti2_mul_input;
            Z_temp_33 <= Y31_final_2 * Ti3_mul_input; Z_temp_34 <= Y31_final_2 * Ti4_mul_input;
            Z_temp_35 <= Y31_final_2 * Ti5_mul_input; Z_temp_36 <= Y31_final_2 * Ti6_mul_input;
            Z_temp_37 <= Y31_final_2 * Ti7_mul_input; Z_temp_38 <= Y31_final_2 * Ti8_mul_input;
            Z_temp_41 <= Y41_final_2 * Ti1; Z_temp_42 <= Y41_final_2 * Ti2_mul_input;
            Z_temp_43 <= Y41_final_2 * Ti3_mul_input; Z_temp_44 <= Y41_final_2 * Ti4_mul_input;
            Z_temp_45 <= Y41_final_2 * Ti5_mul_input; Z_temp_46 <= Y41_final_2 * Ti6_mul_input;
            Z_temp_47 <= Y41_final_2 * Ti7_mul_input; Z_temp_48 <= Y41_final_2 * Ti8_mul_input;
            Z_temp_51 <= Y51_final_2 * Ti1; Z_temp_52 <= Y51_final_2 * Ti2_mul_input;
            Z_temp_53 <= Y51_final_2 * Ti3_mul_input; Z_temp_54 <= Y51_final_2 * Ti4_mul_input;
            Z_temp_55 <= Y51_final_2 * Ti5_mul_input; Z_temp_56 <= Y51_final_2 * Ti6_mul_input;
            Z_temp_57 <= Y51_final_2 * Ti7_mul_input; Z_temp_58 <= Y51_final_2 * Ti8_mul_input;
            Z_temp_61 <= Y61_final_2 * Ti1; Z_temp_62 <= Y61_final_2 * Ti2_mul_input;
            Z_temp_63 <= Y61_final_2 * Ti3_mul_input; Z_temp_64 <= Y61_final_2 * Ti4_mul_input;
            Z_temp_65 <= Y61_final_2 * Ti5_mul_input; Z_temp_66 <= Y61_final_2 * Ti6_mul_input;
            Z_temp_67 <= Y61_final_2 * Ti7_mul_input; Z_temp_68 <= Y61_final_2 * Ti8_mul_input;
            Z_temp_71 <= Y71_final_2 * Ti1; Z_temp_72 <= Y71_final_2 * Ti2_mul_input;
            Z_temp_73 <= Y71_final_2 * Ti3_mul_input; Z_temp_74 <= Y71_final_2 * Ti4_mul_input;
            Z_temp_75 <= Y71_final_2 * Ti5_mul_input; Z_temp_76 <= Y71_final_2 * Ti6_mul_input;
            Z_temp_77 <= Y71_final_2 * Ti7_mul_input; Z_temp_78 <= Y71_final_2 * Ti8_mul_input;
            Z_temp_81 <= Y81_final_2 * Ti1; Z_temp_82 <= Y81_final_2 * Ti2_mul_input;
            Z_temp_83 <= Y81_final_2 * Ti3_mul_input; Z_temp_84 <= Y81_final_2 * Ti4_mul_input;
            Z_temp_85 <= Y81_final_2 * Ti5_mul_input; Z_temp_86 <= Y81_final_2 * Ti6_mul_input;
            Z_temp_87 <= Y81_final_2 * Ti7_mul_input; Z_temp_88 <= Y81_final_2 * Ti8_mul_input;
        end
    end

    // Z_XX registers for accumulation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Z11 <= 0; Z12 <= 0; Z13 <= 0; Z14 <= 0; Z15 <= 0; Z16 <= 0; Z17 <= 0; Z18 <= 0;
            Z21 <= 0; Z22 <= 0; Z23 <= 0; Z24 <= 0; Z25 <= 0; Z26 <= 0; Z27 <= 0; Z28 <= 0;
            Z31 <= 0; Z32 <= 0; Z33 <= 0; Z34 <= 0; Z35 <= 0; Z36 <= 0; Z37 <= 0; Z38 <= 0;
            Z41 <= 0; Z42 <= 0; Z43 <= 0; Z44 <= 0; Z45 <= 0; Z46 <= 0; Z47 <= 0; Z48 <= 0;
            Z51 <= 0; Z52 <= 0; Z53 <= 0; Z54 <= 0; Z55 <= 0; Z56 <= 0; Z57 <= 0; Z58 <= 0;
            Z61 <= 0; Z62 <= 0; Z63 <= 0; Z64 <= 0; Z65 <= 0; Z66 <= 0; Z67 <= 0; Z68 <= 0;
            Z71 <= 0; Z72 <= 0; Z73 <= 0; Z74 <= 0; Z75 <= 0; Z76 <= 0; Z77 <= 0; Z78 <= 0;
            Z81 <= 0; Z82 <= 0; Z83 <= 0; Z84 <= 0; Z85 <= 0; Z86 <= 0; Z87 <= 0; Z88 <= 0;
        end else if (count_8 && count_of == 1) begin // Reset when a new block calculation starts
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
        end else if (enable && count_9) begin // Accumulate results
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

    // Final Z_XX output registers with rounding
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
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
        end else if (count_10 && count_of == 0) begin // Apply rounding when calculation is complete
            Z11_final <= Z11[15] ? Z11[26:16] + 1 : Z11[26:16];
            Z12_final <= Z12[15] ? Z12[26:16] + 1 : Z12[26:16];
            Z13_final <= Z13[15] ? Z13[26:16] + 1 : Z13[26:16];
            Z14_final <= Z14[15] ? Z14[26:16] + 1 : Z14[26:16];
            Z15_final <= Z15[15] ? Z15[26:16] + 1 : Z15[26:16];
            Z16_final <= Z16[15] ? Z16[26:16] + 1 : Z16[26:16];
            Z17_final <= Z17[15] ? Z17[26:16] + 1 : Z17[26:16];
            Z18_final <= Z18[15] ? Z18[26:16] + 1 : Z18[26:16];
            Z21_final <= Z21[15] ? Z21[26:16] + 1 : Z21[26:16];
            Z22_final <= Z22[15] ? Z22[26:16] + 1 : Z22[26:16];
            Z23_final <= Z23[15] ? Z23[26:16] + 1 : Z23[26:16];
            Z24_final <= Z24[15] ? Z24[26:16] + 1 : Z24[26:16];
            Z25_final <= Z25[15] ? Z25[26:16] + 1 : Z25[26:16];
            Z26_final <= Z26[15] ? Z26[26:16] + 1 : Z26[26:16];
            Z27_final <= Z27[15] ? Z27[26:16] + 1 : Z27[26:16];
            Z28_final <= Z28[15] ? Z28[26:16] + 1 : Z28[26:16];
            Z31_final <= Z31[15] ? Z31[26:16] + 1 : Z31[26:16];
            Z32_final <= Z32[15] ? Z32[26:16] + 1 : Z32[26:16];
            Z33_final <= Z33[15] ? Z33[26:16] + 1 : Z33[26:16];
            Z34_final <= Z34[15] ? Z34[26:16] + 1 : Z34[26:16];
            Z35_final <= Z35[15] ? Z35[26:16] + 1 : Z35[26:16];
            Z36_final <= Z36[15] ? Z36[26:16] + 1 : Z36[26:16];
            Z37_final <= Z37[15] ? Z37[26:16] + 1 : Z37[26:16];
            Z38_final <= Z38[15] ? Z38[26:16] + 1 : Z38[26:16];
            Z41_final <= Z41[15] ? Z41[26:16] + 1 : Z41[26:16];
            Z42_final <= Z42[15] ? Z42[26:16] + 1 : Z42[26:16];
            Z43_final <= Z43[15] ? Z43[26:16] + 1 : Z43[26:16];
            Z44_final <= Z44[15] ? Z44[26:16] + 1 : Z44[26:16];
            Z45_final <= Z45[15] ? Z45[26:16] + 1 : Z45[26:16];
            Z46_final <= Z46[15] ? Z46[26:16] + 1 : Z46[26:16];
            Z47_final <= Z47[15] ? Z47[26:16] + 1 : Z47[26:16];
            Z48_final <= Z48[15] ? Z48[26:16] + 1 : Z48[26:16];
            Z51_final <= Z51[15] ? Z51[26:16] + 1 : Z51[26:16];
            Z52_final <= Z52[15] ? Z52[26:16] + 1 : Z52[26:16];
            Z53_final <= Z53[15] ? Z53[26:16] + 1 : Z53[26:16];
            Z54_final <= Z54[15] ? Z54[26:16] + 1 : Z54[26:16];
            Z55_final <= Z55[15] ? Z55[26:16] + 1 : Z55[26:16];
            Z56_final <= Z56[15] ? Z56[26:16] + 1 : Z56[26:16];
            Z57_final <= Z57[15] ? Z57[26:16] + 1 : Z57[26:16];
            Z58_final <= Z58[15] ? Z58[26:16] + 1 : Z58[26:16];
            Z61_final <= Z61[15] ? Z61[26:16] + 1 : Z61[26:16];
            Z62_final <= Z62[15] ? Z62[26:16] + 1 : Z62[26:16];
            Z63_final <= Z63[15] ? Z63[26:16] + 1 : Z63[26:16];
            Z64_final <= Z64[15] ? Z64[26:16] + 1 : Z64[26:16];
            Z65_final <= Z65[15] ? Z65[26:16] + 1 : Z65[26:16];
            Z66_final <= Z66[15] ? Z66[26:16] + 1 : Z66[26:16];
            Z67_final <= Z67[15] ? Z67[26:16] + 1 : Z67[26:16];
            Z68_final <= Z68[15] ? Z68[26:16] + 1 : Z68[26:16];
            Z71_final <= Z71[15] ? Z71[26:16] + 1 : Z71[26:16];
            Z72_final <= Z72[15] ? Z72[26:16] + 1 : Z72[26:16];
            Z73_final <= Z73[15] ? Z73[26:16] + 1 : Z73[26:16];
            Z74_final <= Z74[15] ? Z74[26:16] + 1 : Z74[26:16];
            Z75_final <= Z75[15] ? Z75[26:16] + 1 : Z75[26:16];
            Z76_final <= Z76[15] ? Z76[26:16] + 1 : Z76[26:16];
            Z77_final <= Z77[15] ? Z77[26:16] + 1 : Z77[26:16];
            Z78_final <= Z78[15] ? Z78[26:16] + 1 : Z78[26:16];
            Z81_final <= Z81[15] ? Z81[26:16] + 1 : Z81[26:16];
            Z82_final <= Z82[15] ? Z82[26:16] + 1 : Z82[26:16];
            Z83_final <= Z83[15] ? Z83[26:16] + 1 : Z83[26:16];
            Z84_final <= Z84[15] ? Z84[26:16] + 1 : Z84[26:16];
            Z85_final <= Z85[15] ? Z85[26:16] + 1 : Z85[26:16];
            Z86_final <= Z86[15] ? Z86[26:16] + 1 : Z86[26:16];
            Z87_final <= Z87[15] ? Z87[26:16] + 1 : Z87[26:16];
            Z88_final <= Z88[15] ? Z88[26:16] + 1 : Z88[26:16];
        end
    end

    // output_enable signals the next block, the quantizer, that the input data is ready
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            output_enable <= 1'b0;
        else if (!enable_1)
            output_enable <= 1'b0;
        else if (count_10 == 0 || count_of != 0) // Using || for OR
            output_enable <= 1'b0;
        else if (count_10 && count_of == 0) // Using && for AND
            output_enable <= 1'b1;
    end

    // Y_temp_11 calculation
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            Y_temp_11 <= 0;
        else if (enable)
            Y_temp_11 <= $signed(data_in) * T1;
    end

    // Y11 accumulation
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            Y11 <= 0;
        else if (count == 1 && enable == 1) // Using && for AND
            Y11 <= Y_temp_11;
        else if (enable)
            Y11 <= Y_temp_11 + Y11;
    end

    // Y_temp_X1 calculations
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Y_temp_21 <= 0;
            Y_temp_31 <= 0;
            Y_temp_41 <= 0;
            Y_temp_51 <= 0;
            Y_temp_61 <= 0;
            Y_temp_71 <= 0;
            Y_temp_81 <= 0;
        end else if (!enable_1) begin
            Y_temp_21 <= 0;
            Y_temp_31 <= 0;
            Y_temp_41 <= 0;
            Y_temp_51 <= 0;
            Y_temp_61 <= 0;
            Y_temp_71 <= 0;
            Y_temp_81 <= 0;
        end else if (enable_1) begin
            Y_temp_21 <= $signed(data_1) * Y2_mul_input;
            Y_temp_31 <= $signed(data_1) * Y3_mul_input;
            Y_temp_41 <= $signed(data_1) * Y4_mul_input;
            Y_temp_51 <= $signed(data_1) * Y5_mul_input;
            Y_temp_61 <= $signed(data_1) * Y6_mul_input;
            Y_temp_71 <= $signed(data_1) * Y7_mul_input;
            Y_temp_81 <= $signed(data_1) * Y8_mul_input;
        end
    end

    // YX1 accumulation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Y21 <= 0;
            Y31 <= 0;
            Y41 <= 0;
            Y51 <= 0;
            Y61 <= 0;
            Y71 <= 0;
            Y81 <= 0;
        end else if (!enable_1) begin
            Y21 <= 0;
            Y31 <= 0;
            Y41 <= 0;
            Y51 <= 0;
            Y61 <= 0;
            Y71 <= 0;
            Y81 <= 0;
        end else if (enable_1) begin
            Y21 <= Y_temp_21 + Y21;
            Y31 <= Y_temp_31 + Y31;
            Y41 <= Y_temp_41 + Y41;
            Y51 <= Y_temp_51 + Y51;
            Y61 <= Y_temp_61 + Y61;
            Y71 <= Y_temp_71 + Y71;
            Y81 <= Y_temp_81 + Y81;
        end
    end

    // Counters for timing and control
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 0; count_3 <= 0; count_4 <= 0; count_5 <= 0;
            count_6 <= 0; count_7 <= 0; count_8 <= 0; count_9 <= 0;
            count_10 <= 0;
        end else if (!enable) begin
            count <= 0; count_3 <= 0; count_4 <= 0; count_5 <= 0;
            count_6 <= 0; count_7 <= 0; count_8 <= 0; count_9 <= 0;
            count_10 <= 0;
        end else if (enable) begin
            count <= count + 1;
            count_3 <= count_1;
            count_4 <= count_3;
            count_5 <= count_4;
            count_6 <= count_5;
            count_7 <= count_6;
            count_8 <= count_7;
            count_9 <= count_8;
            count_10 <= count_9;
        end
    end

    // count_1 generation
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            count_1 <= 1'b0;
        else if (count != 3'b111 || !enable) // Using || for OR, 3'b111 is 7
            count_1 <= 1'b0;
        else if (count == 3'b111)
            count_1 <= 1'b1;
    end

    // count_of and count_of_copy
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            count_of <= 0;
            count_of_copy <= 0;
        end else if (!enable) begin
            count_of <= 0;
            count_of_copy <= 0;
        end else if (count_1 == 1'b1) begin
            count_of <= count_of + 1;
            count_of_copy <= count_of_copy + 1;
        end
    end

    // Y11_final calculation with DC offset subtraction
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Y11_final <= 0;
        end else if (count_3 && enable_1) begin
            Y11_final <= Y11 - 25'd5932032;
        end
    end

    // YX1_final and YX1_final_prev updates
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Y21_final <= 0; Y21_final_prev <= 0;
            Y31_final <= 0; Y31_final_prev <= 0;
            Y41_final <= 0; Y41_final_prev <= 0;
            Y51_final <= 0; Y51_final_prev <= 0;
            Y61_final <= 0; Y61_final_prev <= 0;
            Y71_final <= 0; Y71_final_prev <= 0;
            Y81_final <= 0; Y81_final_prev <= 0;
        end else if (!enable_1) begin
            Y21_final <= 0; Y21_final_prev <= 0;
            Y31_final <= 0; Y31_final_prev <= 0;
            Y41_final <= 0; Y41_final_prev <= 0;
            Y51_final <= 0; Y51_final_prev <= 0;
            Y61_final <= 0; Y61_final_prev <= 0;
            Y71_final <= 0; Y71_final_prev <= 0;
            Y81_final <= 0; Y81_final_prev <= 0;
        end else if (count_4 && enable_1) begin
            Y21_final <= Y21; Y21_final_prev <= Y21_final;
            Y31_final <= Y31; Y31_final_prev <= Y31_final;
            Y41_final <= Y41; Y41_final_prev <= Y41_final;
            Y51_final <= Y51; Y51_final_prev <= Y51_final;
            Y61_final <= Y61; Y61_final_prev <= Y61_final;
            Y71_final <= Y71; Y71_final_prev <= Y71_final;
            Y81_final <= Y81; Y81_final_prev <= Y81_final;
        end
    end

    // YX1_final_diff calculations
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Y21_final_diff <= 0; Y31_final_diff <= 0;
            Y41_final_diff <= 0; Y51_final_diff <= 0;
            Y61_final_diff <= 0; Y71_final_diff <= 0;
            Y81_final_diff <= 0;
        end else if (count_5 && enable_1) begin
            Y21_final_diff <= Y21_final - Y21_final_prev;
            Y31_final_diff <= Y31_final - Y31_final_prev;
            Y41_final_diff <= Y41_final - Y41_final_prev;
            Y51_final_diff <= Y51_final - Y51_final_prev;
            Y61_final_diff <= Y61_final - Y61_final_prev;
            Y71_final_diff <= Y71_final - Y71_final_prev;
            Y81_final_diff <= Y81_final - Y81_final_prev;
        end
    end

    // Combinational logic for Y_mul_input based on count
    always_comb begin
        case (count)
            3'b000: Y2_mul_input = T21;
            3'b001: Y2_mul_input = T22;
            3'b010: Y2_mul_input = T23;
            3'b011: Y2_mul_input = T24;
            3'b100: Y2_mul_input = T25;
            3'b101: Y2_mul_input = T26;
            3'b110: Y2_mul_input = T27;
            3'b111: Y2_mul_input = T28;
            default: Y2_mul_input = 0; // Should not happen with 3-bit counter
        endcase
    end

    always_comb begin
        case (count)
            3'b000: Y3_mul_input = T31;
            3'b001: Y3_mul_input = T32;
            3'b010: Y3_mul_input = T33;
            3'b011: Y3_mul_input = T34;
            3'b100: Y3_mul_input = T34;
            3'b101: Y3_mul_input = T33;
            3'b110: Y3_mul_input = T32;
            3'b111: Y3_mul_input = T31;
            default: Y3_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count)
            3'b000: Y4_mul_input = T22;
            3'b001: Y4_mul_input = T25;
            3'b010: Y4_mul_input = T28;
            3'b011: Y4_mul_input = T26;
            3'b100: Y4_mul_input = T23;
            3'b101: Y4_mul_input = T21;
            3'b110: Y4_mul_input = T24;
            3'b111: Y4_mul_input = T27;
            default: Y4_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count)
            3'b000: Y5_mul_input = T1;
            3'b001: Y5_mul_input = T52;
            3'b010: Y5_mul_input = T52;
            3'b011: Y5_mul_input = T1;
            3'b100: Y5_mul_input = T1;
            3'b101: Y5_mul_input = T52;
            3'b110: Y5_mul_input = T52;
            3'b111: Y5_mul_input = T1;
            default: Y5_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count)
            3'b000: Y6_mul_input = T23;
            3'b001: Y6_mul_input = T28;
            3'b010: Y6_mul_input = T24;
            3'b011: Y6_mul_input = T22;
            3'b100: Y6_mul_input = T27;
            3'b101: Y6_mul_input = T25;
            3'b110: Y6_mul_input = T21;
            3'b111: Y6_mul_input = T26;
            default: Y6_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count)
            3'b000: Y7_mul_input = T32;
            3'b001: Y7_mul_input = T34;
            3'b010: Y7_mul_input = T31;
            3'b011: Y7_mul_input = T33;
            3'b100: Y7_mul_input = T33;
            3'b101: Y7_mul_input = T31;
            3'b110: Y7_mul_input = T34;
            3'b111: Y7_mul_input = T32;
            default: Y7_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count)
            3'b000: Y8_mul_input = T24;
            3'b001: Y8_mul_input = T26;
            3'b010: Y8_mul_input = T22;
            3'b011: Y8_mul_input = T28;
            3'b100: Y8_mul_input = T21;
            3'b101: Y8_mul_input = T27;
            3'b110: Y8_mul_input = T23;
            3'b111: Y8_mul_input = T25;
            default: Y8_mul_input = 0;
        endcase
    end

    // Combinational logic for Ti_mul_input based on count_of_copy (for inverse DCT coefficients)
    always_comb begin
        case (count_of_copy)
            3'b000: Ti2_mul_input = Ti28;
            3'b001: Ti2_mul_input = Ti21;
            3'b010: Ti2_mul_input = Ti22;
            3'b011: Ti2_mul_input = Ti23;
            3'b100: Ti2_mul_input = Ti24;
            3'b101: Ti2_mul_input = Ti25;
            3'b110: Ti2_mul_input = Ti26;
            3'b111: Ti2_mul_input = Ti27;
            default: Ti2_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count_of_copy)
            3'b000: Ti3_mul_input = Ti31;
            3'b001: Ti3_mul_input = Ti31;
            3'b010: Ti3_mul_input = Ti32;
            3'b011: Ti3_mul_input = Ti33;
            3'b100: Ti3_mul_input = Ti34;
            3'b101: Ti3_mul_input = Ti34;
            3'b110: Ti3_mul_input = Ti33;
            3'b111: Ti3_mul_input = Ti32;
            default: Ti3_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count_of_copy)
            3'b000: Ti4_mul_input = Ti27;
            3'b001: Ti4_mul_input = Ti22;
            3'b010: Ti4_mul_input = Ti25;
            3'b011: Ti4_mul_input = Ti28;
            3'b100: Ti4_mul_input = Ti26;
            3'b101: Ti4_mul_input = Ti23;
            3'b110: Ti4_mul_input = Ti21;
            3'b111: Ti4_mul_input = Ti24;
            default: Ti4_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count_of_copy)
            3'b000: Ti5_mul_input = Ti1;
            3'b001: Ti5_mul_input = Ti1;
            3'b010: Ti5_mul_input = Ti52;
            3'b011: Ti5_mul_input = Ti52;
            3'b100: Ti5_mul_input = Ti1;
            3'b101: Ti5_mul_input = Ti1;
            3'b110: Ti5_mul_input = Ti52;
            3'b111: Ti5_mul_input = Ti52;
            default: Ti5_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count_of_copy)
            3'b000: Ti6_mul_input = Ti26;
            3'b001: Ti6_mul_input = Ti23;
            3'b010: Ti6_mul_input = Ti28;
            3'b011: Ti6_mul_input = Ti24;
            3'b100: Ti6_mul_input = Ti22;
            3'b101: Ti6_mul_input = Ti27;
            3'b110: Ti6_mul_input = Ti25;
            3'b111: Ti6_mul_input = Ti21;
            default: Ti6_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count_of_copy)
            3'b000: Ti7_mul_input = Ti32;
            3'b001: Ti7_mul_input = Ti32;
            3'b010: Ti7_mul_input = Ti34;
            3'b011: Ti7_mul_input = Ti31;
            3'b100: Ti7_mul_input = Ti33;
            3'b101: Ti7_mul_input = Ti33;
            3'b110: Ti7_mul_input = Ti31;
            3'b111: Ti7_mul_input = Ti34;
            default: Ti7_mul_input = 0;
        endcase
    end

    always_comb begin
        case (count_of_copy)
            3'b000: Ti8_mul_input = Ti25;
            3'b001: Ti8_mul_input = Ti24;
            3'b010: Ti8_mul_input = Ti26;
            3'b011: Ti8_mul_input = Ti22;
            3'b100: Ti8_mul_input = Ti28;
            3'b101: Ti8_mul_input = Ti21;
            3'b110: Ti8_mul_input = Ti27;
            3'b111: Ti8_mul_input = Ti23;
            default: Ti8_mul_input = 0;
        endcase
    end

    // Rounding stage and sign extension
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            data_1 <= 0;
            Y11_final_1 <= 0; Y21_final_1 <= 0; Y31_final_1 <= 0; Y41_final_1 <= 0;
            Y51_final_1 <= 0; Y61_final_1 <= 0; Y71_final_1 <= 0; Y81_final_1 <= 0;
            Y11_final_2 <= 0; Y21_final_2 <= 0; Y31_final_2 <= 0; Y41_final_2 <= 0;
            Y51_final_2 <= 0; Y61_final_2 <= 0; Y71_final_2 <= 0; Y81_final_2 <= 0;
            Y11_final_3 <= 0; Y11_final_4 <= 0;
        end else if (enable) begin
            data_1 <= data_in;
            Y11_final_1 <= Y11_final[11] ? Y11_final[24:12] + 1 : Y11_final[24:12];
            Y11_final_2 <= {{19{Y11_final_1[12]}}, Y11_final_1}; // Sign extension to 32 bits
            Y11_final_3 <= Y11_final_2;
            Y11_final_4 <= Y11_final_3;

            Y21_final_1 <= Y21_final_diff[11] ? Y21_final_diff[24:12] + 1 : Y21_final_diff[24:12];
            Y21_final_2 <= {{19{Y21_final_1[12]}}, Y21_final_1};
            Y31_final_1 <= Y31_final_diff[11] ? Y31_final_diff[24:12] + 1 : Y31_final_diff[24:12];
            Y31_final_2 <= {{19{Y31_final_1[12]}}, Y31_final_1};
            Y41_final_1 <= Y41_final_diff[11] ? Y41_final_diff[24:12] + 1 : Y41_final_diff[24:12];
            Y41_final_2 <= {{19{Y41_final_1[12]}}, Y41_final_1};
            Y51_final_1 <= Y51_final_diff[11] ? Y51_final_diff[24:12] + 1 : Y51_final_diff[24:12];
            Y51_final_2 <= {{19{Y51_final_1[12]}}, Y51_final_1};
            Y61_final_1 <= Y61_final_diff[11] ? Y61_final_diff[24:12] + 1 : Y61_final_diff[24:12];
            Y61_final_2 <= {{19{Y61_final_1[12]}}, Y61_final_1};
            Y71_final_1 <= Y71_final_diff[11] ? Y71_final_diff[24:12] + 1 : Y71_final_diff[24:12];
            Y71_final_2 <= {{19{Y71_final_1[12]}}, Y71_final_1};
            Y81_final_1 <= Y81_final_diff[11] ? Y81_final_diff[24:12] + 1 : Y81_final_diff[24:12];
            Y81_final_2 <= {{19{Y81_final_1[12]}}, Y81_final_1};
        end
    end

    // enable_1 update
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            enable_1 <= 1'b0;
        else
            enable_1 <= enable;
    end

endmodule