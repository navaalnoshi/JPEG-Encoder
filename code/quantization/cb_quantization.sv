/* ---------------------------------------------------------------------------
Module: cb_quantizer

Description:
    This module performs quantization on an 8x8 matrix of Cb DCT coefficients.
    These coefficients are the result of applying 2D DCT to a chroma (Cb) block.
    Quantization reduces the precision of the DCT coefficients to achieve 
    lossy compression, which is a key part of JPEG encoding.

    The module replaces division with multiplication (4096 / Q), using pre-computed
    constants to reduce hardware cost. It implements a 3-stage pipeline:
    - Stage 1: Input registration and sign-extension
    - Stage 2: Multiplication with quantization factors
    - Stage 3: Output rounding and quantization result
--------------------------------------------------------------------------- */
`timescale 1ns / 100ps

module cb_quantizer(
    input  logic         clk,
    input  logic         rst,
    input  logic         enable,
    // Input Z signals (individual declarations)
    input  logic [10:0]  Z11, Z12, Z13, Z14, Z15, Z16, Z17, Z18,
    input  logic [10:0]  Z21, Z22, Z23, Z24, Z25, Z26, Z27, Z28,
    input  logic [10:0]  Z31, Z32, Z33, Z34, Z35, Z36, Z37, Z38,
    input  logic [10:0]  Z41, Z42, Z43, Z44, Z45, Z46, Z47, Z48,
    input  logic [10:0]  Z51, Z52, Z53, Z54, Z55, Z56, Z57, Z58,
    input  logic [10:0]  Z61, Z62, Z63, Z64, Z65, Z66, Z67, Z68,
    input  logic [10:0]  Z71, Z72, Z73, Z74, Z75, Z76, Z77, Z78,
    input  logic [10:0]  Z81, Z82, Z83, Z84, Z85, Z86, Z87, Z88,

    // Output Q signals (individual declarations)
    output logic [10:0]  Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18,
    output logic [10:0]  Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28,
    output logic [10:0]  Q31, Q32, Q33, Q34, Q35, Q36, Q37, Q38,
    output logic [10:0]  Q41, Q42, Q43, Q44, Q45, Q46, Q47, Q48,
    output logic [10:0]  Q51, Q52, Q53, Q54, Q55, Q56, Q57, Q58,
    output logic [10:0]  Q61, Q62, Q63, Q64, Q65, Q66, Q67, Q68,
    output logic [10:0]  Q71, Q72, Q73, Q74, Q75, Q76, Z77, Q78, // Corrected Z77 to Q77
    output logic [10:0]  Q81, Q82, Q83, Q84, Q85, Q86, Q87, Q88,

    output logic         out_enable
);
     /* Below are the quantization values, these can be changed for different
    quantization levels. */
    parameter Q1_1 = 1; parameter Q1_2 = 1; parameter Q1_3 = 1; parameter Q1_4 = 1;
    parameter Q1_5 = 1; parameter Q1_6 = 1; parameter Q1_7 = 1; parameter Q1_8 = 1;
    parameter Q2_1 = 1; parameter Q2_2 = 1; parameter Q2_3 = 1; parameter Q2_4 = 1;
    parameter Q2_5 = 1; parameter Q2_6 = 1; parameter Q2_7 = 1; parameter Q2_8 = 1;
    parameter Q3_1 = 1; parameter Q3_2 = 1; parameter Q3_3 = 1; parameter Q3_4 = 1;
    parameter Q3_5 = 1; parameter Q3_6 = 1; parameter Q3_7 = 1; parameter Q3_8 = 1;
    parameter Q4_1 = 1; parameter Q4_2 = 1; parameter Q4_3 = 1; parameter Q4_4 = 1;
    parameter Q4_5 = 1; parameter Q4_6 = 1; parameter Q4_7 = 1; parameter Q4_8 = 1;
    parameter Q5_1 = 1; parameter Q5_2 = 1; parameter Q5_3 = 1; parameter Q5_4 = 1;
    parameter Q5_5 = 1; parameter Q5_6 = 1; parameter Q5_7 = 1; parameter Q5_8 = 1;
    parameter Q6_1 = 1; parameter Q6_2 = 1; parameter Q6_3 = 1; parameter Q6_4 = 1;
    parameter Q6_5 = 1; parameter Q6_6 = 1; parameter Q6_7 = 1; parameter Q6_8 = 1;
    parameter Q7_1 = 1; parameter Q7_2 = 1; parameter Q7_3 = 1; parameter Q7_4 = 1;
    parameter Q7_5 = 1; parameter Q7_6 = 1; parameter Q7_7 = 1; parameter Q7_8 = 1;
    parameter Q8_1 = 1; parameter Q8_2 = 1; parameter Q8_3 = 1; parameter Q8_4 = 1;
    parameter Q8_5 = 1; parameter Q8_6 = 1; parameter Q8_7 = 1; parameter Q8_8 = 1;
    
     //---------------------------------------------------
    // Precomputed reciprocal values (scaled by 4096 = 2^12) to replace division
    // QQ[i][j] = floor(4096 / Q[i][j])
    // Final quantization = (Z[i][j] * QQ[i][j]) >>> 12
    //   /* The following parameters hold the values of 4096 divided by the corresponding
    //individual quantization values. These values are needed to get around
    //actually dividing the Y, Cb, and Cr values by the quantization values.
    //Instead, you can multiply by the values below, and then divide by 4096
    //to get the same result. And 4096 = 2^12, so instead of dividing, you can
    //just shift the bottom 12 bits out. This is a lossy process, as 4096/Q divided
    //by 4096 doesn't always equal the same value as if you were just dividing by Q. But
    //the quantization process is also lossy, so the additional loss of precision is
    //negligible. To decrease the loss, you could use a larger number than 4096 to
    //get around the actual use of division. Like take 8192/Q and then divide by 8192.*//
    // -------------------------------------------------------------------------

    parameter QQ1_1 = 4096 / Q1_1; parameter QQ1_2 = 4096 / Q1_2; parameter QQ1_3 = 4096 / Q1_3; parameter QQ1_4 = 4096 / Q1_4;
    parameter QQ1_5 = 4096 / Q1_5; parameter QQ1_6 = 4096 / Q1_6; parameter QQ1_7 = 4096 / Q1_7; parameter QQ1_8 = 4096 / Q1_8;
    parameter QQ2_1 = 4096 / Q2_1; parameter QQ2_2 = 4096 / Q2_2; parameter QQ2_3 = 4096 / Q2_3; parameter QQ2_4 = 4096 / Q2_4;
    parameter QQ2_5 = 4096 / Q2_5; parameter QQ2_6 = 4096 / Q2_6; parameter QQ2_7 = 4096 / Q2_7; parameter QQ2_8 = 4096 / Q2_8;
    parameter QQ3_1 = 4096 / Q3_1; parameter QQ3_2 = 4096 / Q3_2; parameter QQ3_3 = 4096 / Q3_3; parameter QQ3_4 = 4096 / Q3_4;
    parameter QQ3_5 = 4096 / Q3_5; parameter QQ3_6 = 4096 / Q3_6; parameter QQ3_7 = 4096 / Q3_7; parameter QQ3_8 = 4096 / Q3_8;
    parameter QQ4_1 = 4096 / Q4_1; parameter QQ4_2 = 4096 / Q4_2; parameter QQ4_3 = 4096 / Q4_3; parameter QQ4_4 = 4096 / Q4_4;
    parameter QQ4_5 = 4096 / Q4_5; parameter QQ4_6 = 4096 / Q4_6; parameter QQ4_7 = 4096 / Q4_7; parameter QQ4_8 = 4096 / Q4_8;
    parameter QQ5_1 = 4096 / Q5_1; parameter QQ5_2 = 4096 / Q5_2; parameter QQ5_3 = 4096 / Q5_3; parameter QQ5_4 = 4096 / Q5_4;
    parameter QQ5_5 = 4096 / Q5_5; parameter QQ5_6 = 4096 / Q5_6; parameter QQ5_7 = 4096 / Q5_7; parameter QQ5_8 = 4096 / Q5_8;
    parameter QQ6_1 = 4096 / Q6_1; parameter QQ6_2 = 4096 / Q6_2; parameter QQ6_3 = 4096 / Q6_3; parameter QQ6_4 = 4096 / Q6_4;
    parameter QQ6_5 = 4096 / Q6_5; parameter QQ6_6 = 4096 / Q6_6; parameter QQ6_7 = 4096 / Q6_7; parameter QQ6_8 = 4096 / Q6_8;
    parameter QQ7_1 = 4096 / Q7_1; parameter QQ7_2 = 4096 / Q7_2; parameter QQ7_3 = 4096 / Q7_3; parameter QQ7_4 = 4096 / Q7_4;
    parameter QQ7_5 = 4096 / Q7_5; parameter QQ7_6 = 4096 / Q7_6; parameter QQ7_7 = 4096 / Q7_7; parameter QQ7_8 = 4096 / Q7_8;
    parameter QQ8_1 = 4096 / Q8_1; parameter QQ8_2 = 4096 / Q8_2; parameter QQ8_3 = 4096 / Q8_3; parameter QQ8_4 = 4096 / Q8_4;
    parameter QQ8_5 = 4096 / Q8_5; parameter QQ8_6 = 4096 / Q8_6; parameter QQ8_7 = 4096 / Q8_7; parameter QQ8_8 = 4096 / Q8_8;

    
    logic [12:0] QM1_1 = QQ1_1; logic [12:0] QM1_2 = QQ1_2; logic [12:0] QM1_3 = QQ1_3; logic [12:0] QM1_4 = QQ1_4;
    logic [12:0] QM1_5 = QQ1_5; logic [12:0] QM1_6 = QQ1_6; logic [12:0] QM1_7 = QQ1_7; logic [12:0] QM1_8 = QQ1_8;
    logic [12:0] QM2_1 = QQ2_1; logic [12:0] QM2_2 = QQ2_2; logic [12:0] QM2_3 = QQ2_3; logic [12:0] QM2_4 = QQ2_4;
    logic [12:0] QM2_5 = QQ2_5; logic [12:0] QM2_6 = QQ2_6; logic [12:0] QM2_7 = QQ2_7; logic [12:0] QM2_8 = QQ2_8;
    logic [12:0] QM3_1 = QQ3_1; logic [12:0] QM3_2 = QQ3_2; logic [12:0] QM3_3 = QQ3_3; logic [12:0] QM3_4 = QQ3_4;
    logic [12:0] QM3_5 = QQ3_5; logic [12:0] QM3_6 = QQ3_6; logic [12:0] QM3_7 = QQ3_7; logic [12:0] QM3_8 = QQ3_8;
    logic [12:0] QM4_1 = QQ4_1; logic [12:0] QM4_2 = QQ4_2; logic [12:0] QM4_3 = QQ4_3; logic [12:0] QM4_4 = QQ4_4;
    logic [12:0] QM4_5 = QQ4_5; logic [12:0] QM4_6 = QQ4_6; logic [12:0] QM4_7 = QQ4_7; logic [12:0] QM4_8 = QQ4_8;
    logic [12:0] QM5_1 = QQ5_1; logic [12:0] QM5_2 = QQ5_2; logic [12:0] QM5_3 = QQ5_3; logic [12:0] QM5_4 = QQ5_4;
    logic [12:0] QM5_5 = QQ5_5; logic [12:0] QM5_6 = QQ5_6; logic [12:0] QM5_7 = QQ5_7; logic [12:0] QM5_8 = QQ5_8;
    logic [12:0] QM6_1 = QQ6_1; logic [12:0] QM6_2 = QQ6_2; logic [12:0] QM6_3 = QQ6_3; logic [12:0] QM6_4 = QQ6_4;
    logic [12:0] QM6_5 = QQ6_5; logic [12:0] QM6_6 = QQ6_6; logic [12:0] QM6_7 = QQ6_7; logic [12:0] QM6_8 = QQ6_8;
    logic [12:0] QM7_1 = QQ7_1; logic [12:0] QM7_2 = QQ7_2; logic [12:0] QM7_3 = QQ7_3; logic [12:0] QM7_4 = QQ7_4;
    logic [12:0] QM7_5 = QQ7_5; logic [12:0] QM7_6 = QQ7_6; logic [12:0] QM7_7 = QQ7_7; logic [12:0] QM7_8 = QQ7_8;
    logic [12:0] QM8_1 = QQ8_1; logic [12:0] QM8_2 = QQ8_2; logic [12:0] QM8_3 = QQ8_3; logic [12:0] QM8_4 = QQ8_4;
    logic [12:0] QM8_5 = QQ8_5; logic [12:0] QM8_6 = QQ8_6; logic [12:0] QM8_7 = QQ8_7; logic [12:0] QM8_8 = QQ8_8;

    // Internal pipeline registers using logic type
    logic [22:0] Z11_temp, Z12_temp, Z13_temp, Z14_temp, Z15_temp, Z16_temp, Z17_temp, Z18_temp;
    logic [22:0] Z21_temp, Z22_temp, Z23_temp, Z24_temp, Z25_temp, Z26_temp, Z27_temp, Z28_temp;
    logic [22:0] Z31_temp, Z32_temp, Z33_temp, Z34_temp, Z35_temp, Z36_temp, Z37_temp, Z38_temp;
    logic [22:0] Z41_temp, Z42_temp, Z43_temp, Z44_temp, Z45_temp, Z46_temp, Z47_temp, Z48_temp;
    logic [22:0] Z51_temp, Z52_temp, Z53_temp, Z54_temp, Z55_temp, Z56_temp, Z57_temp, Z58_temp;
    logic [22:0] Z61_temp, Z62_temp, Z63_temp, Z64_temp, Z65_temp, Z66_temp, Z67_temp, Z68_temp;
    logic [22:0] Z71_temp, Z72_temp, Z73_temp, Z74_temp, Z75_temp, Z76_temp, Z77_temp, Z78_temp;
    logic [22:0] Z81_temp, Z82_temp, Z83_temp, Z84_temp, Z85_temp, Z86_temp, Z87_temp, Z88_temp;

    logic [22:0] Z11_temp_1, Z12_temp_1, Z13_temp_1, Z14_temp_1, Z15_temp_1, Z16_temp_1, Z17_temp_1, Z18_temp_1;
    logic [22:0] Z21_temp_1, Z22_temp_1, Z23_temp_1, Z24_temp_1, Z25_temp_1, Z26_temp_1, Z27_temp_1, Z28_temp_1;
    logic [22:0] Z31_temp_1, Z32_temp_1, Z33_temp_1, Z34_temp_1, Z35_temp_1, Z36_temp_1, Z37_temp_1, Z38_temp_1;
    logic [22:0] Z41_temp_1, Z42_temp_1, Z43_temp_1, Z44_temp_1, Z45_temp_1, Z46_temp_1, Z47_temp_1, Z48_temp_1;
    logic [22:0] Z51_temp_1, Z52_temp_1, Z53_temp_1, Z54_temp_1, Z55_temp_1, Z56_temp_1, Z57_temp_1, Z58_temp_1;
    logic [22:0] Z61_temp_1, Z62_temp_1, Z63_temp_1, Z64_temp_1, Z65_temp_1, Z66_temp_1, Z67_temp_1, Z68_temp_1;
    logic [22:0] Z71_temp_1, Z72_temp_1, Z73_temp_1, Z74_temp_1, Z75_temp_1, Z76_temp_1, Z77_temp_1, Z78_temp_1;
    logic [22:0] Z81_temp_1, Z82_temp_1, Z83_temp_1, Z84_temp_1, Z85_temp_1, Z86_temp_1, Z87_temp_1, Z88_temp_1;

    // Signed intermediate values using logic type (will be automatically signed-extended if assigned from smaller signed values)
    logic signed [31:0] Z11_s, Z12_s, Z13_s, Z14_s, Z15_s, Z16_s, Z17_s, Z18_s;
    logic signed [31:0] Z21_s, Z22_s, Z23_s, Z24_s, Z25_s, Z26_s, Z27_s, Z28_s;
    logic signed [31:0] Z31_s, Z32_s, Z33_s, Z34_s, Z35_s, Z36_s, Z37_s, Z38_s;
    logic signed [31:0] Z41_s, Z42_s, Z43_s, Z44_s, Z45_s, Z46_s, Z47_s, Z48_s;
    logic signed [31:0] Z51_s, Z52_s, Z53_s, Z54_s, Z55_s, Z56_s, Z57_s, Z58_s;
    logic signed [31:0] Z61_s, Z62_s, Z63_s, Z64_s, Z65_s, Z66_s, Z67_s, Z68_s;
    logic signed [31:0] Z71_s, Z72_s, Z73_s, Z74_s, Z75_s, Z76_s, Z77_s, Z78_s;
    logic signed [31:0] Z81_s, Z82_s, Z83_s, Z84_s, Z85_s, Z86_s, Z87_s, Z88_s;

    logic enable_1, enable_2, enable_3;

    
    // -------------------------------------------------------------------------
    // Stage 1: Input latching and sign extension
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Z11_s <= 0; Z12_s <= 0; Z13_s <= 0; Z14_s <= 0;
            Z15_s <= 0; Z16_s <= 0; Z17_s <= 0; Z18_s <= 0;
            Z21_s <= 0; Z22_s <= 0; Z23_s <= 0; Z24_s <= 0;
            Z25_s <= 0; Z26_s <= 0; Z27_s <= 0; Z28_s <= 0;
            Z31_s <= 0; Z32_s <= 0; Z33_s <= 0; Z34_s <= 0;
            Z35_s <= 0; Z36_s <= 0; Z37_s <= 0; Z38_s <= 0;
            Z41_s <= 0; Z42_s <= 0; Z43_s <= 0; Z44_s <= 0;
            Z45_s <= 0; Z46_s <= 0; Z47_s <= 0; Z48_s <= 0;
            Z51_s <= 0; Z52_s <= 0; Z53_s <= 0; Z54_s <= 0;
            Z55_s <= 0; Z56_s <= 0; Z57_s <= 0; Z58_s <= 0;
            Z61_s <= 0; Z62_s <= 0; Z63_s <= 0; Z64_s <= 0;
            Z65_s <= 0; Z66_s <= 0; Z67_s <= 0; Z68_s <= 0;
            Z71_s <= 0; Z72_s <= 0; Z73_s <= 0; Z74_s <= 0;
            Z75_s <= 0; Z76_s <= 0; Z77_s <= 0; Z78_s <= 0;
            Z81_s <= 0; Z82_s <= 0; Z83_s <= 0; Z84_s <= 0;
            Z85_s <= 0; Z86_s <= 0; Z87_s <= 0; Z88_s <= 0;
        end else if (enable) begin
            // Automatic signed extension for Z_s due to 'signed' declaration
            Z11_s <= $signed(Z11); Z12_s <= $signed(Z12); Z13_s <= $signed(Z13); Z14_s <= $signed(Z14);
            Z15_s <= $signed(Z15); Z16_s <= $signed(Z16); Z17_s <= $signed(Z17); Z18_s <= $signed(Z18);
            Z21_s <= $signed(Z21); Z22_s <= $signed(Z22); Z23_s <= $signed(Z23); Z24_s <= $signed(Z24);
            Z25_s <= $signed(Z25); Z26_s <= $signed(Z26); Z27_s <= $signed(Z27); Z28_s <= $signed(Z28);
            Z31_s <= $signed(Z31); Z32_s <= $signed(Z32); Z33_s <= $signed(Z33); Z34_s <= $signed(Z34);
            Z35_s <= $signed(Z35); Z36_s <= $signed(Z36); Z37_s <= $signed(Z37); Z38_s <= $signed(Z38);
            Z41_s <= $signed(Z41); Z42_s <= $signed(Z42); Z43_s <= $signed(Z43); Z44_s <= $signed(Z44);
            Z45_s <= $signed(Z45); Z46_s <= $signed(Z46); Z47_s <= $signed(Z47); Z48_s <= $signed(Z48);
            Z51_s <= $signed(Z51); Z52_s <= $signed(Z52); Z53_s <= $signed(Z53); Z54_s <= $signed(Z54);
            Z55_s <= $signed(Z55); Z56_s <= $signed(Z56); Z57_s <= $signed(Z57); Z58_s <= $signed(Z58);
            Z61_s <= $signed(Z61); Z62_s <= $signed(Z62); Z63_s <= $signed(Z63); Z64_s <= $signed(Z64);
            Z65_s <= $signed(Z65); Z66_s <= $signed(Z66); Z67_s <= $signed(Z67); Z68_s <= $signed(Z68);
            Z71_s <= $signed(Z71); Z72_s <= $signed(Z72); Z73_s <= $signed(Z73); Z74_s <= $signed(Z74);
            Z75_s <= $signed(Z75); Z76_s <= $signed(Z76); Z77_s <= $signed(Z77); Z78_s <= $signed(Z78);
            Z81_s <= $signed(Z81); Z82_s <= $signed(Z82); Z83_s <= $signed(Z83); Z84_s <= $signed(Z84);
            Z85_s <= $signed(Z85); Z86_s <= $signed(Z86); Z87_s <= $signed(Z87); Z88_s <= $signed(Z88);
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Multiply with precomputed quantization factors
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Z11_temp <= 0; Z12_temp <= 0; Z13_temp <= 0; Z14_temp <= 0;
            Z15_temp <= 0; Z16_temp <= 0; Z17_temp <= 0; Z18_temp <= 0;
            Z21_temp <= 0; Z22_temp <= 0; Z23_temp <= 0; Z24_temp <= 0;
            Z25_temp <= 0; Z26_temp <= 0; Z27_temp <= 0; Z28_temp <= 0;
            Z31_temp <= 0; Z32_temp <= 0; Z33_temp <= 0; Z34_temp <= 0;
            Z35_temp <= 0; Z36_temp <= 0; Z37_temp <= 0; Z38_temp <= 0;
            Z41_temp <= 0; Z42_temp <= 0; Z43_temp <= 0; Z44_temp <= 0;
            Z45_temp <= 0; Z46_temp <= 0; Z47_temp <= 0; Z48_temp <= 0;
            Z51_temp <= 0; Z52_temp <= 0; Z53_temp <= 0; Z54_temp <= 0;
            Z55_temp <= 0; Z56_temp <= 0; Z57_temp <= 0; Z58_temp <= 0;
            Z61_temp <= 0; Z62_temp <= 0; Z63_temp <= 0; Z64_temp <= 0;
            Z65_temp <= 0; Z66_temp <= 0; Z67_temp <= 0; Z68_temp <= 0;
            Z71_temp <= 0; Z72_temp <= 0; Z73_temp <= 0; Z74_temp <= 0;
            Z75_temp <= 0; Z76_temp <= 0; Z77_temp <= 0; Z78_temp <= 0;
            Z81_temp <= 0; Z82_temp <= 0; Z83_temp <= 0; Z84_temp <= 0;
            Z85_temp <= 0; Z86_temp <= 0; Z87_temp <= 0; Z88_temp <= 0;
        end else if (enable_1) begin
            Z11_temp <= Z11_s * QM1_1; Z12_temp <= Z12_s * QM1_2;
            Z13_temp <= Z13_s * QM1_3; Z14_temp <= Z14_s * QM1_4;
            Z15_temp <= Z15_s * QM1_5; Z16_temp <= Z16_s * QM1_6;
            Z17_temp <= Z17_s * QM1_7; Z18_temp <= Z18_s * QM1_8;
            Z21_temp <= Z21_s * QM2_1; Z22_temp <= Z22_s * QM2_2;
            Z23_temp <= Z23_s * QM2_3; Z24_temp <= Z24_s * QM2_4;
            Z25_temp <= Z25_s * QM2_5; Z26_temp <= Z26_s * QM2_6;
            Z27_temp <= Z27_s * QM2_7; Z28_temp <= Z28_s * QM2_8;
            Z31_temp <= Z31_s * QM3_1; Z32_temp <= Z32_s * QM3_2;
            Z33_temp <= Z33_s * QM3_3; Z34_temp <= Z34_s * QM3_4;
            Z35_temp <= Z35_s * QM3_5; Z36_temp <= Z36_s * QM3_6;
            Z37_temp <= Z37_s * QM3_7; Z38_temp <= Z38_s * QM3_8;
            Z41_temp <= Z41_s * QM4_1; Z42_temp <= Z42_s * QM4_2;
            Z43_temp <= Z43_s * QM4_3; Z44_temp <= Z44_s * QM4_4;
            Z45_temp <= Z45_s * QM4_5; Z46_temp <= Z46_s * QM4_6;
            Z47_temp <= Z47_s * QM4_7; Z48_temp <= Z48_s * QM4_8;
            Z51_temp <= Z51_s * QM5_1; Z52_temp <= Z52_s * QM5_2;
            Z53_temp <= Z53_s * QM5_3; Z54_temp <= Z54_s * QM5_4;
            Z55_temp <= Z55_s * QM5_5; Z56_temp <= Z56_s * QM5_6;
            Z57_temp <= Z57_s * QM5_7; Z58_temp <= Z58_s * QM5_8;
            Z61_temp <= Z61_s * QM6_1; Z62_temp <= Z62_s * QM6_2;
            Z63_temp <= Z63_s * QM6_3; Z64_temp <= Z64_s * QM6_4;
            Z65_temp <= Z65_s * QM6_5; Z66_temp <= Z66_s * QM6_6;
            Z67_temp <= Z67_s * QM6_7; Z68_temp <= Z68_s * QM6_8;
            Z71_temp <= Z71_s * QM7_1; Z72_temp <= Z72_s * QM7_2;
            Z73_temp <= Z73_s * QM7_3; Z74_temp <= Z74_s * QM7_4;
            Z75_temp <= Z75_s * QM7_5; Z76_temp <= Z76_s * QM7_6;
            Z77_temp <= Z77_s * QM7_7; Z78_temp <= Z78_s * QM7_8;
            Z81_temp <= Z81_s * QM8_1; Z82_temp <= Z82_s * QM8_2;
            Z83_temp <= Z83_s * QM8_3; Z84_temp <= Z84_s * QM8_4;
            Z85_temp <= Z85_s * QM8_5; Z86_temp <= Z86_s * QM8_6;
            Z87_temp <= Z87_s * QM8_7; Z88_temp <= Z88_s * QM8_8;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Pipeline the intermediate result
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Z11_temp_1 <= 0; Z12_temp_1 <= 0; Z13_temp_1 <= 0; Z14_temp_1 <= 0;
            Z15_temp_1 <= 0; Z16_temp_1 <= 0; Z17_temp_1 <= 0; Z18_temp_1 <= 0;
            Z21_temp_1 <= 0; Z22_temp_1 <= 0; Z23_temp_1 <= 0; Z24_temp_1 <= 0;
            Z25_temp_1 <= 0; Z26_temp_1 <= 0; Z27_temp_1 <= 0; Z28_temp_1 <= 0;
            Z31_temp_1 <= 0; Z32_temp_1 <= 0; Z33_temp_1 <= 0; Z34_temp_1 <= 0;
            Z35_temp_1 <= 0; Z36_temp_1 <= 0; Z37_temp_1 <= 0; Z38_temp_1 <= 0;
            Z41_temp_1 <= 0; Z42_temp_1 <= 0; Z43_temp_1 <= 0; Z44_temp_1 <= 0;
            Z45_temp_1 <= 0; Z46_temp_1 <= 0; Z47_temp_1 <= 0; Z48_temp_1 <= 0;
            Z51_temp_1 <= 0; Z52_temp_1 <= 0; Z53_temp_1 <= 0; Z54_temp_1 <= 0;
            Z55_temp_1 <= 0; Z56_temp_1 <= 0; Z57_temp_1 <= 0; Z58_temp_1 <= 0;
            Z61_temp_1 <= 0; Z62_temp_1 <= 0; Z63_temp_1 <= 0; Z64_temp_1 <= 0;
            Z65_temp_1 <= 0; Z66_temp_1 <= 0; Z67_temp_1 <= 0; Z68_temp_1 <= 0;
            Z71_temp_1 <= 0; Z72_temp_1 <= 0; Z73_temp_1 <= 0; Z74_temp_1 <= 0;
            Z75_temp_1 <= 0; Z76_temp_1 <= 0; Z77_temp_1 <= 0; Z78_temp_1 <= 0;
            Z81_temp_1 <= 0; Z82_temp_1 <= 0; Z83_temp_1 <= 0; Z84_temp_1 <= 0;
            Z85_temp_1 <= 0; Z86_temp_1 <= 0; Z87_temp_1 <= 0; Z88_temp_1 <= 0;
        end else if (enable_2) begin
            Z11_temp_1 <= Z11_temp; Z12_temp_1 <= Z12_temp;
            Z13_temp_1 <= Z13_temp; Z14_temp_1 <= Z14_temp;
            Z15_temp_1 <= Z15_temp; Z16_temp_1 <= Z16_temp;
            Z17_temp_1 <= Z17_temp; Z18_temp_1 <= Z18_temp;
            Z21_temp_1 <= Z21_temp; Z22_temp_1 <= Z22_temp;
            Z23_temp_1 <= Z23_temp; Z24_temp_1 <= Z24_temp;
            Z25_temp_1 <= Z25_temp; Z26_temp_1 <= Z26_temp;
            Z27_temp_1 <= Z27_temp; Z28_temp_1 <= Z28_temp;
            Z31_temp_1 <= Z31_temp; Z32_temp_1 <= Z32_temp;
            Z33_temp_1 <= Z33_temp; Z34_temp_1 <= Z34_temp;
            Z35_temp_1 <= Z35_temp; Z36_temp_1 <= Z36_temp;
            Z37_temp_1 <= Z37_temp; Z38_temp_1 <= Z38_temp;
            Z41_temp_1 <= Z41_temp; Z42_temp_1 <= Z42_temp;
            Z43_temp_1 <= Z43_temp; Z44_temp_1 <= Z44_temp;
            Z45_temp_1 <= Z45_temp; Z46_temp_1 <= Z46_temp;
            Z47_temp_1 <= Z47_temp; Z48_temp_1 <= Z48_temp;
            Z51_temp_1 <= Z51_temp; Z52_temp_1 <= Z52_temp;
            Z53_temp_1 <= Z53_temp; Z54_temp_1 <= Z54_temp;
            Z55_temp_1 <= Z55_temp; Z56_temp_1 <= Z56_temp;
            Z57_temp_1 <= Z57_temp; Z58_temp_1 <= Z58_temp;
            Z61_temp_1 <= Z61_temp; Z62_temp_1 <= Z62_temp;
            Z63_temp_1 <= Z63_temp; Z64_temp_1 <= Z64_temp;
            Z65_temp_1 <= Z65_temp; Z66_temp_1 <= Z66_temp;
            Z67_temp_1 <= Z67_temp; Z68_temp_1 <= Z68_temp;
            Z71_temp_1 <= Z71_temp; Z72_temp_1 <= Z72_temp;
            Z73_temp_1 <= Z73_temp; Z74_temp_1 <= Z74_temp;
            Z75_temp_1 <= Z75_temp; Z76_temp_1 <= Z76_temp;
            Z77_temp_1 <= Z77_temp; Z78_temp_1 <= Z78_temp;
            Z81_temp_1 <= Z81_temp; Z82_temp_1 <= Z82_temp;
            Z83_temp_1 <= Z83_temp; Z84_temp_1 <= Z84_temp;
            Z85_temp_1 <= Z85_temp; Z86_temp_1 <= Z86_temp;
            Z87_temp_1 <= Z87_temp; Z88_temp_1 <= Z88_temp;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 4: Right shift with rounding to produce quantized output
    // -------------------------------------------------------------------------
   always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Q11 <= 0; Q12 <= 0; Q13 <= 0; Q14 <= 0; Q15 <= 0; Q16 <= 0; Q17 <= 0; Q18 <= 0;
            Q21 <= 0; Q22 <= 0; Q23 <= 0; Q24 <= 0; Q25 <= 0; Q26 <= 0; Q27 <= 0; Q28 <= 0;
            Q31 <= 0; Q32 <= 0; Q33 <= 0; Q34 <= 0; Q35 <= 0; Q36 <= 0; Q37 <= 0; Q38 <= 0;
            Q41 <= 0; Q42 <= 0; Q43 <= 0; Q44 <= 0; Q45 <= 0; Q46 <= 0; Q47 <= 0; Q48 <= 0;
            Q51 <= 0; Q52 <= 0; Q53 <= 0; Q54 <= 0; Q55 <= 0; Q56 <= 0; Q57 <= 0; Q58 <= 0;
            Q61 <= 0; Q62 <= 0; Q63 <= 0; Q64 <= 0; Q65 <= 0; Q66 <= 0; Q67 <= 0; Q68 <= 0;
            Q71 <= 0; Q72 <= 0; Q73 <= 0; Q74 <= 0; Q75 <= 0; Q76 <= 0; Q77 <= 0; Q78 <= 0;
            Q81 <= 0; Q82 <= 0; Q83 <= 0; Q84 <= 0; Q85 <= 0; Q86 <= 0; Q87 <= 0; Q88 <= 0;
        end else if (enable_3) begin
            // Rounding based on the bit in the 11th place of Zxx_temp_1
            Q11 <= Z11_temp_1[11] ? Z11_temp_1[22:12] + 1 : Z11_temp_1[22:12];
            Q12 <= Z12_temp_1[11] ? Z12_temp_1[22:12] + 1 : Z12_temp_1[22:12];
            Q13 <= Z13_temp_1[11] ? Z13_temp_1[22:12] + 1 : Z13_temp_1[22:12];
            Q14 <= Z14_temp_1[11] ? Z14_temp_1[22:12] + 1 : Z14_temp_1[22:12];
            Q15 <= Z15_temp_1[11] ? Z15_temp_1[22:12] + 1 : Z15_temp_1[22:12];
            Q16 <= Z16_temp_1[11] ? Z16_temp_1[22:12] + 1 : Z16_temp_1[22:12];
            Q17 <= Z17_temp_1[11] ? Z17_temp_1[22:12] + 1 : Z17_temp_1[22:12];
            Q18 <= Z18_temp_1[11] ? Z18_temp_1[22:12] + 1 : Z18_temp_1[22:12];
            Q21 <= Z21_temp_1[11] ? Z21_temp_1[22:12] + 1 : Z21_temp_1[22:12];
            Q22 <= Z22_temp_1[11] ? Z22_temp_1[22:12] + 1 : Z22_temp_1[22:12];
            Q23 <= Z23_temp_1[11] ? Z23_temp_1[22:12] + 1 : Z23_temp_1[22:12];
            Q24 <= Z24_temp_1[11] ? Z24_temp_1[22:12] + 1 : Z24_temp_1[22:12];
            Q25 <= Z25_temp_1[11] ? Z25_temp_1[22:12] + 1 : Z25_temp_1[22:12];
            Q26 <= Z26_temp_1[11] ? Z26_temp_1[22:12] + 1 : Z26_temp_1[22:12];
            Q27 <= Z27_temp_1[11] ? Z27_temp_1[22:12] + 1 : Z27_temp_1[22:12];
            Q28 <= Z28_temp_1[11] ? Z28_temp_1[22:12] + 1 : Z28_temp_1[22:12];
            Q31 <= Z31_temp_1[11] ? Z31_temp_1[22:12] + 1 : Z31_temp_1[22:12];
            Q32 <= Z32_temp_1[11] ? Z32_temp_1[22:12] + 1 : Z32_temp_1[22:12];
            Q33 <= Z33_temp_1[11] ? Z33_temp_1[22:12] + 1 : Z33_temp_1[22:12];
            Q34 <= Z34_temp_1[11] ? Z34_temp_1[22:12] + 1 : Z34_temp_1[22:12];
            Q35 <= Z35_temp_1[11] ? Z35_temp_1[22:12] + 1 : Z35_temp_1[22:12];
            Q36 <= Z36_temp_1[11] ? Z36_temp_1[22:12] + 1 : Z36_temp_1[22:12];
            Q37 <= Z37_temp_1[11] ? Z37_temp_1[22:12] + 1 : Z37_temp_1[22:12];
            Q38 <= Z38_temp_1[11] ? Z38_temp_1[22:12] + 1 : Z38_temp_1[22:12];
            Q41 <= Z41_temp_1[11] ? Z41_temp_1[22:12] + 1 : Z41_temp_1[22:12];
            Q42 <= Z42_temp_1[11] ? Z42_temp_1[22:12] + 1 : Z42_temp_1[22:12];
            Q43 <= Z43_temp_1[11] ? Z43_temp_1[22:12] + 1 : Z43_temp_1[22:12];
            Q44 <= Z44_temp_1[11] ? Z44_temp_1[22:12] + 1 : Z44_temp_1[22:12];
            Q45 <= Z45_temp_1[11] ? Z45_temp_1[22:12] + 1 : Z45_temp_1[22:12];
            Q46 <= Z46_temp_1[11] ? Z46_temp_1[22:12] + 1 : Z46_temp_1[22:12];
            Q47 <= Z47_temp_1[11] ? Z47_temp_1[22:12] + 1 : Z47_temp_1[22:12];
            Q48 <= Z48_temp_1[11] ? Z48_temp_1[22:12] + 1 : Z48_temp_1[22:12];
            Q51 <= Z51_temp_1[11] ? Z51_temp_1[22:12] + 1 : Z51_temp_1[22:12];
            Q52 <= Z52_temp_1[11] ? Z52_temp_1[22:12] + 1 : Z52_temp_1[22:12];
            Q53 <= Z53_temp_1[11] ? Z53_temp_1[22:12] + 1 : Z53_temp_1[22:12];
            Q54 <= Z54_temp_1[11] ? Z54_temp_1[22:12] + 1 : Z54_temp_1[22:12];
            Q55 <= Z55_temp_1[11] ? Z55_temp_1[22:12] + 1 : Z55_temp_1[22:12];
            Q56 <= Z56_temp_1[11] ? Z56_temp_1[22:12] + 1 : Z56_temp_1[22:12];
            Q57 <= Z57_temp_1[11] ? Z57_temp_1[22:12] + 1 : Z57_temp_1[22:12];
            Q58 <= Z58_temp_1[11] ? Z58_temp_1[22:12] + 1 : Z58_temp_1[22:12];
            Q61 <= Z61_temp_1[11] ? Z61_temp_1[22:12] + 1 : Z61_temp_1[22:12];
            Q62 <= Z62_temp_1[11] ? Z62_temp_1[22:12] + 1 : Z62_temp_1[22:12];
            Q63 <= Z63_temp_1[11] ? Z63_temp_1[22:12] + 1 : Z63_temp_1[22:12];
            Q64 <= Z64_temp_1[11] ? Z64_temp_1[22:12] + 1 : Z64_temp_1[22:12];
            Q65 <= Z65_temp_1[11] ? Z65_temp_1[22:12] + 1 : Z65_temp_1[22:12];
            Q66 <= Z66_temp_1[11] ? Z66_temp_1[22:12] + 1 : Z66_temp_1[22:12];
            Q67 <= Z67_temp_1[11] ? Z67_temp_1[22:12] + 1 : Z67_temp_1[22:12];
            Q68 <= Z68_temp_1[11] ? Z68_temp_1[22:12] + 1 : Z68_temp_1[22:12];
            Q71 <= Z71_temp_1[11] ? Z71_temp_1[22:12] + 1 : Z71_temp_1[22:12];
            Q72 <= Z72_temp_1[11] ? Z72_temp_1[22:12] + 1 : Z72_temp_1[22:12];
            Q73 <= Z73_temp_1[11] ? Z73_temp_1[22:12] + 1 : Z73_temp_1[22:12];
            Q74 <= Z74_temp_1[11] ? Z74_temp_1[22:12] + 1 : Z74_temp_1[22:12];
            Q75 <= Z75_temp_1[11] ? Z75_temp_1[22:12] + 1 : Z75_temp_1[22:12];
            Q76 <= Z76_temp_1[11] ? Z76_temp_1[22:12] + 1 : Z76_temp_1[22:12];
            Q77 <= Z77_temp_1[11] ? Z77_temp_1[22:12] + 1 : Z77_temp_1[22:12];
            Q78 <= Z78_temp_1[11] ? Z78_temp_1[22:12] + 1 : Z78_temp_1[22:12];
            Q81 <= Z81_temp_1[11] ? Z81_temp_1[22:12] + 1 : Z81_temp_1[22:12];
            Q82 <= Z82_temp_1[11] ? Z82_temp_1[22:12] + 1 : Z82_temp_1[22:12];
            Q83 <= Z83_temp_1[11] ? Z83_temp_1[22:12] + 1 : Z83_temp_1[22:12];
            Q84 <= Z84_temp_1[11] ? Z84_temp_1[22:12] + 1 : Z84_temp_1[22:12];
            Q85 <= Z85_temp_1[11] ? Z85_temp_1[22:12] + 1 : Z85_temp_1[22:12];
            Q86 <= Z86_temp_1[11] ? Z86_temp_1[22:12] + 1 : Z86_temp_1[22:12];
            Q87 <= Z87_temp_1[11] ? Z87_temp_1[22:12] + 1 : Z87_temp_1[22:12];
            Q88 <= Z88_temp_1[11] ? Z88_temp_1[22:12] + 1 : Z88_temp_1[22:12];
        end
    end

    // -------------------------------------------------------------------------
    // Enable signal pipelining: tracks progress through pipeline stages
    /* enable_1 is delayed one clock cycle from enable, and it's used to
    enable the logic that needs to execute on the clock cycle after enable goes high
    enable_2 is delayed two clock cycles, and out_enable signals the next module
    that its input data is ready*/
    // -------------------------------------------------------------------------
    
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_1 <= 1'b0;
            enable_2 <= 1'b0;
            enable_3 <= 1'b0;
            out_enable <= 1'b0;
        end else begin
            enable_1 <= enable;
            enable_2 <= enable_1;
            enable_3 <= enable_2;
            out_enable <= enable_3;
        end
    end

endmodule
