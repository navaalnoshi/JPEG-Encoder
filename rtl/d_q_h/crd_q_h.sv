/*----------------------------------------------------------------------------------
Module Name : crd_q_h
Description : This module integrates the Discrete Cosine Transform (DCT), quantizer, 
and Huffman encoder specifically for the Cr (chrominance-red) component of the JPEG 
encoding process. It processes an 8x8 block of input pixel data by:
  1. Performing 2D DCT using fixed-point arithmetic
  2. Quantizing the resulting DCT coefficients using a chrominance quantization matrix
  3. Compressing the quantized values using Huffman encoding

This pipeline is synchronized using `enable` and `rst`, and it produces a compressed
32-bit JPEG bitstream along with data-ready flags and end-of-block status.
----------------------------------------------------------------------------------*/

`timescale 1ns / 100ps

module crd_q_h (
    input  logic        clk,                  // System clock
    input  logic        rst,                  // Active-high synchronous reset
    input  logic        enable,               // Enable for input data processing
    input  logic [7:0]  data_in,              // 8-bit Cr component pixel input

    output logic [31:0] JPEG_bitstream,       // Huffman encoded JPEG bitstream (Cr channel)
    output logic        data_ready,           // Indicates valid data on JPEG_bitstream
    output logic [4:0]  cr_orc,               // Output register count for Cr encoding
    output logic        end_of_block_empty    // FIFO empty flag for EOB of Cr block
);

    // ---------------- Internal control wires ----------------
    logic dct_enable, quantizer_enable;

    // ---------------- Wires for DCT output ----------------
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

    // ---------------- Wires for Quantizer output ----------------
    logic [10:0] Q11, Q12, Q13, Q14, Q15, Q16, Q17, Q18;
    logic [10:0] Q21, Q22, Q23, Q24, Q25, Q26, Q27, Q28;
    logic [10:0] Q31, Q32, Q33, Q34, Q35, Q36, Q37, Q38;
    logic [10:0] Q41, Q42, Q43, Q44, Q45, Q46, Q47, Q48;
    logic [10:0] Q51, Q52, Q53, Q54, Q55, Q56, Q57, Q58;
    logic [10:0] Q61, Q62, Q63, Q64, Q65, Q66, Q67, Q68;
    logic [10:0] Q71, Q72, Q73, Q74, Q75, Q76, Q77, Q78;
    logic [10:0] Q81, Q82, Q83, Q84, Q85, Q86, Q87, Q88;

    // ---------------- DCT Module for Cr Component ----------------
    cr_dct u8 (
        .clk(clk), .rst(rst), .enable(enable), .data_in(data_in),
        .Z11_final(Z11_final), .Z12_final(Z12_final), .Z13_final(Z13_final), .Z14_final(Z14_final),
        .Z15_final(Z15_final), .Z16_final(Z16_final), .Z17_final(Z17_final), .Z18_final(Z18_final),
        .Z21_final(Z21_final), .Z22_final(Z22_final), .Z23_final(Z23_final), .Z24_final(Z24_final),
        .Z25_final(Z25_final), .Z26_final(Z26_final), .Z27_final(Z27_final), .Z28_final(Z28_final),
        .Z31_final(Z31_final), .Z32_final(Z32_final), .Z33_final(Z33_final), .Z34_final(Z34_final),
        .Z35_final(Z35_final), .Z36_final(Z36_final), .Z37_final(Z37_final), .Z38_final(Z38_final),
        .Z41_final(Z41_final), .Z42_final(Z42_final), .Z43_final(Z43_final), .Z44_final(Z44_final),
        .Z45_final(Z45_final), .Z46_final(Z46_final), .Z47_final(Z47_final), .Z48_final(Z48_final),
        .Z51_final(Z51_final), .Z52_final(Z52_final), .Z53_final(Z53_final), .Z54_final(Z54_final),
        .Z55_final(Z55_final), .Z56_final(Z56_final), .Z57_final(Z57_final), .Z58_final(Z58_final),
        .Z61_final(Z61_final), .Z62_final(Z62_final), .Z63_final(Z63_final), .Z64_final(Z64_final),
        .Z65_final(Z65_final), .Z66_final(Z66_final), .Z67_final(Z67_final), .Z68_final(Z68_final),
        .Z71_final(Z71_final), .Z72_final(Z72_final), .Z73_final(Z73_final), .Z74_final(Z74_final),
        .Z75_final(Z75_final), .Z76_final(Z76_final), .Z77_final(Z77_final), .Z78_final(Z78_final),
        .Z81_final(Z81_final), .Z82_final(Z82_final), .Z83_final(Z83_final), .Z84_final(Z84_final),
        .Z85_final(Z85_final), .Z86_final(Z86_final), .Z87_final(Z87_final), .Z88_final(Z88_final),
        .output_enable(dct_enable)
    );

    // ---------------- Quantizer for Cr DCT coefficients ----------------
    cr_quantizer u9 (
        .clk(clk), .rst(rst), .enable(dct_enable),
        .Z11(Z11_final), .Z12(Z12_final), .Z13(Z13_final), .Z14(Z14_final),
        .Z15(Z15_final), .Z16(Z16_final), .Z17(Z17_final), .Z18(Z18_final),
        .Z21(Z21_final), .Z22(Z22_final), .Z23(Z23_final), .Z24(Z24_final),
        .Z25(Z25_final), .Z26(Z26_final), .Z27(Z27_final), .Z28(Z28_final),
        .Z31(Z31_final), .Z32(Z32_final), .Z33(Z33_final), .Z34(Z34_final),
        .Z35(Z35_final), .Z36(Z36_final), .Z37(Z37_final), .Z38(Z38_final),
        .Z41(Z41_final), .Z42(Z42_final), .Z43(Z43_final), .Z44(Z44_final),
        .Z45(Z45_final), .Z46(Z46_final), .Z47(Z47_final), .Z48(Z48_final),
        .Z51(Z51_final), .Z52(Z52_final), .Z53(Z53_final), .Z54(Z54_final),
        .Z55(Z55_final), .Z56(Z56_final), .Z57(Z57_final), .Z58(Z58_final),
        .Z61(Z61_final), .Z62(Z62_final), .Z63(Z63_final), .Z64(Z64_final),
        .Z65(Z65_final), .Z66(Z66_final), .Z67(Z67_final), .Z68(Z68_final),
        .Z71(Z71_final), .Z72(Z72_final), .Z73(Z73_final), .Z74(Z74_final),
        .Z75(Z75_final), .Z76(Z76_final), .Z77(Z77_final), .Z78(Z78_final),
        .Z81(Z81_final), .Z82(Z82_final), .Z83(Z83_final), .Z84(Z84_final),
        .Z85(Z85_final), .Z86(Z86_final), .Z87(Z87_final), .Z88(Z88_final),
        .Q11(Q11), .Q12(Q12), .Q13(Q13), .Q14(Q14), .Q15(Q15), .Q16(Q16), .Q17(Q17), .Q18(Q18),
        .Q21(Q21), .Q22(Q22), .Q23(Q23), .Q24(Q24), .Q25(Q25), .Q26(Q26), .Q27(Q27), .Q28(Q28),
        .Q31(Q31), .Q32(Q32), .Q33(Q33), .Q34(Q34), .Q35(Q35), .Q36(Q36), .Q37(Q37), .Q38(Q38),
        .Q41(Q41), .Q42(Q42), .Q43(Q43), .Q44(Q44), .Q45(Q45), .Q46(Q46), .Q47(Q47), .Q48(Q48),
        .Q51(Q51), .Q52(Q52), .Q53(Q53), .Q54(Q54), .Q55(Q55), .Q56(Q56), .Q57(Q57), .Q58(Q58),
        .Q61(Q61), .Q62(Q62), .Q63(Q63), .Q64(Q64), .Q65(Q65), .Q66(Q66), .Q67(Q67), .Q68(Q68),
        .Q71(Q71), .Q72(Q72), .Q73(Q73), .Q74(Q74), .Q75(Q75), .Q76(Q76), .Q77(Q77), .Q78(Q78),
        .Q81(Q81), .Q82(Q82), .Q83(Q83), .Q84(Q84), .Q85(Q85), .Q86(Q86), .Q87(Q87), .Q88(Q88),
        .out_enable(quantizer_enable)
    );

    // ---------------- Huffman Encoder for Cr ----------------
    cr_huff u10 (
        .clk(clk), .rst(rst), .enable(quantizer_enable),
        .Cr11(Q11), .Cr12(Q21), .Cr13(Q31), .Cr14(Q41), .Cr15(Q51), .Cr16(Q61), .Cr17(Q71), .Cr18(Q81),
        .Cr21(Q12), .Cr22(Q22), .Cr23(Q32), .Cr24(Q42), .Cr25(Q52), .Cr26(Q62), .Cr27(Q72), .Cr28(Q82),
        .Cr31(Q13), .Cr32(Q23), .Cr33(Q33), .Cr34(Q43), .Cr35(Q53), .Cr36(Q63), .Cr37(Q73), .Cr38(Q83),
        .Cr41(Q14), .Cr42(Q24), .Cr43(Q34), .Cr44(Q44), .Cr45(Q54), .Cr46(Q64), .Cr47(Q74), .Cr48(Q84),
        .Cr51(Q15), .Cr52(Q25), .Cr53(Q35), .Cr54(Q45), .Cr55(Q55), .Cr56(Q65), .Cr57(Q75), .Cr58(Q85),
        .Cr61(Q16), .Cr62(Q26), .Cr63(Q36), .Cr64(Q46), .Cr65(Q56), .Cr66(Q66), .Cr67(Q76), .Cr68(Q86),
        .Cr71(Q17), .Cr72(Q27), .Cr73(Q37), .Cr74(Q47), .Cr75(Q57), .Cr76(Q67), .Cr77(Q77), .Cr78(Q87),
        .Cr81(Q18), .Cr82(Q28), .Cr83(Q38), .Cr84(Q48), .Cr85(Q58), .Cr86(Q68), .Cr87(Q78), .Cr88(Q88),
        .JPEG_bitstream(JPEG_bitstream),
        .data_ready(data_ready),
        .output_reg_count(cr_orc),
        .end_of_block_empty(end_of_block_empty)
    );

endmodule
