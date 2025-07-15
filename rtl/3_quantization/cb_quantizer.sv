// Copyright 2025 Maktab-e-Digital Systems Lahore.
// Licensed under the Apache License, Version 2.0, see LICENSE file for details.
// SPDX-License-Identifier: Apache-2.0
// Description:
//    This module performs quantization on an 8x8 matrix of Cb DCT coefficients.
//    These coefficients are the result of applying 2D DCT to a chroma (Cb) block.
//    Quantization reduces the precision of the DCT coefficients to achieve
//    lossy compression, which is a key part of JPEG encoding.The module replaces division 
//    with multiplication (4096 / Q), using pre-computed constants to reduce hardware cost. 
//    It implements a 4-stage pipeline:
//    - Stage 1: Input registration and sign-extension
//    - Stage 2: Multiplication with quantization factors
//    - Stage 3: Pipelining of intermediate result
//    - Stage 4: Output rounding and quantization result
// Author:Navaal Noshi
// Date:11th July,2025.
`timescale 1ns / 100ps

module cb_quantizer #(
    parameter logic [63:0] CB_Q_MATRIX [0:63] = '{default: 64'd1}
) (
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [10:0] Z [7:0][7:0],        // Input DCT Coefficients
    output logic [10:0] Q [7:0][7:0],        // Output Quantized Coefficients
    output logic        out_enable           // Output enable after 3-stage pipeline
);

    // Internal pipeline registers
    logic signed [31:0] Z_int     [7:0][7:0];  // Stage 1
    logic signed [22:0] Z_temp    [7:0][7:0];  // Stage 2
    logic signed [22:0] Z_temp_1  [7:0][7:0];  // Stage 3

    // Quantization multipliers (4096 / Q[i][j])
    logic [12:0] QM [7:0][7:0];

    // Pipeline control
    logic enable_1, enable_2;

    // -------------------------------------------------------------------------
    // Stage 0: Initialize Quantization Multiplier Matrix (4096 / Q[i][j])
    // -------------------------------------------------------------------------
    initial begin
        for (int i = 0; i < 8; i++) begin
            for (int j = 0; j < 8; j++) begin
                if (CB_Q_MATRIX[i*8 + j] == 0)
                    QM[i][j] = 13'd0;
                else
                    QM[i][j] = 4096 / CB_Q_MATRIX[i*8 + j];
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 1: Sign-extension and input register (Z_int)
    // -------------------------------------------------------------------------
    generate
        for (genvar i = 0; i < 8; i++) begin : gen_stage1_row
            for (genvar j = 0; j < 8; j++) begin : gen_stage1_col
                always_ff @(posedge clk or posedge rst) begin
                    if (rst)
                        Z_int[i][j] <= 32'sd0;
                    else if (enable)
                        Z_int[i][j] <= {{21{Z[i][j][10]}}, Z[i][j]};
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 2: Multiply Z_int with quantization multipliers (Z_temp)
    // -------------------------------------------------------------------------
    generate
        for (genvar i = 0; i < 8; i++) begin : gen_stage2_row
            for (genvar j = 0; j < 8; j++) begin : gen_stage2_col
                always_ff @(posedge clk or posedge rst) begin
                    if (rst)
                        Z_temp[i][j] <= 23'sd0;
                    else if (enable_1)
                        Z_temp[i][j] <= Z_int[i][j] * QM[i][j];
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 3: Pipeline Z_temp to Z_temp_1
    // -------------------------------------------------------------------------
    generate
        for (genvar i = 0; i < 8; i++) begin : gen_stage3_row
            for (genvar j = 0; j < 8; j++) begin : gen_stage3_col
                always_ff @(posedge clk or posedge rst) begin
                    if (rst)
                        Z_temp_1[i][j] <= 23'sd0;
                    else if (enable_2)
                        Z_temp_1[i][j] <= Z_temp[i][j];
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 4: Final rounding and shift to produce quantized output (Q)
    // -------------------------------------------------------------------------
    generate
        for (genvar i = 0; i < 8; i++) begin : gen_stage4_row
            for (genvar j = 0; j < 8; j++) begin : gen_stage4_col
                always_ff @(posedge clk or posedge rst) begin
                    if (rst)
                        Q[i][j] <= 11'sd0;
                    else if (out_enable)
                        Q[i][j] <= (Z_temp_1[i][j] + 2048) >>> 12;
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Pipeline control logic for enable signal propagation
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_1   <= 1'b0;
            enable_2   <= 1'b0;
            out_enable <= 1'b0;
        end else begin
            enable_1   <= enable;
            enable_2   <= enable_1;
            out_enable <= enable_2;
        end
    end

endmodule
