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

Inputs:
    clk        : Clock signal
    rst        : Active-high reset
    enable     : Enables loading of new input data
    Z[8][8]    : 8x8 matrix of 11-bit signed DCT coefficients

Outputs:
    Q[8][8]        : 8x8 matrix of 11-bit quantized output values
    out_enable     : Asserted when Q is valid (3 clock cycles after enable)
--------------------------------------------------------------------------- */

`timescale 1ns / 100ps

module cb_quantizer(
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [10:0] Z [8][8],     // 8x8 input block (signed DCT coefficients)
    output logic [10:0] Q [8][8],     // 8x8 output block (quantized)
    output logic        out_enable    // Valid output indicator (3-cycle latency)
);

    // -------------------------------------------------------------------------
    // Quantization matrix (can be configured)
    // JPEG typically uses higher values for high frequencies to discard details
    // -------------------------------------------------------------------------
    parameter int Q_MATRIX [8][8] = {
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1}
    };

    // -------------------------------------------------------------------------
    // Precomputed reciprocal values (scaled by 4096 = 2^12) to replace division
    // QQ[i][j] = floor(4096 / Q[i][j])
    // Final quantization = (Z[i][j] * QQ[i][j]) >>> 12
    // -------------------------------------------------------------------------
    localparam int QQ_MATRIX [8][8] = {
        {4096/Q_MATRIX[0][0], 4096/Q_MATRIX[0][1], 4096/Q_MATRIX[0][2], 4096/Q_MATRIX[0][3], 4096/Q_MATRIX[0][4], 4096/Q_MATRIX[0][5], 4096/Q_MATRIX[0][6], 4096/Q_MATRIX[0][7]},
        {4096/Q_MATRIX[1][0], 4096/Q_MATRIX[1][1], 4096/Q_MATRIX[1][2], 4096/Q_MATRIX[1][3], 4096/Q_MATRIX[1][4], 4096/Q_MATRIX[1][5], 4096/Q_MATRIX[1][6], 4096/Q_MATRIX[1][7]},
        {4096/Q_MATRIX[2][0], 4096/Q_MATRIX[2][1], 4096/Q_MATRIX[2][2], 4096/Q_MATRIX[2][3], 4096/Q_MATRIX[2][4], 4096/Q_MATRIX[2][5], 4096/Q_MATRIX[2][6], 4096/Q_MATRIX[2][7]},
        {4096/Q_MATRIX[3][0], 4096/Q_MATRIX[3][1], 4096/Q_MATRIX[3][2], 4096/Q_MATRIX[3][3], 4096/Q_MATRIX[3][4], 4096/Q_MATRIX[3][5], 4096/Q_MATRIX[3][6], 4096/Q_MATRIX[3][7]},
        {4096/Q_MATRIX[4][0], 4096/Q_MATRIX[4][1], 4096/Q_MATRIX[4][2], 4096/Q_MATRIX[4][3], 4096/Q_MATRIX[4][4], 4096/Q_MATRIX[4][5], 4096/Q_MATRIX[4][6], 4096/Q_MATRIX[4][7]},
        {4096/Q_MATRIX[5][0], 4096/Q_MATRIX[5][1], 4096/Q_MATRIX[5][2], 4096/Q_MATRIX[5][3], 4096/Q_MATRIX[5][4], 4096/Q_MATRIX[5][5], 4096/Q_MATRIX[5][6], 4096/Q_MATRIX[5][7]},
        {4096/Q_MATRIX[6][0], 4096/Q_MATRIX[6][1], 4096/Q_MATRIX[6][2], 4096/Q_MATRIX[6][3], 4096/Q_MATRIX[6][4], 4096/Q_MATRIX[6][5], 4096/Q_MATRIX[6][6], 4096/Q_MATRIX[6][7]},
        {4096/Q_MATRIX[7][0], 4096/Q_MATRIX[7][1], 4096/Q_MATRIX[7][2], 4096/Q_MATRIX[7][3], 4096/Q_MATRIX[7][4], 4096/Q_MATRIX[7][5], 4096/Q_MATRIX[7][6], 4096/Q_MATRIX[7][7]}
    };

    // Internal registers for pipelining
    logic [31:0] Z_int [8][8];     // Stage 1: Extended DCT values (signed 32-bit)
    logic [22:0] Z_temp [8][8];    // Stage 2: After multiplication
    logic [22:0] Z_temp_1 [8][8];  // Stage 3: Intermediate result pipelined

    // Pipeline control signals (3-cycle latency)
    logic enable_1, enable_2, enable_3;

    // -------------------------------------------------------------------------
    // Stage 1: Input latching and sign extension
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_int[i][j] <= 0;
        end else if (enable) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_int[i][j] <= $signed(Z[i][j]); // Sign-extend input
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Multiply with precomputed quantization factors
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp[i][j] <= 0;
        end else if (enable_1) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp[i][j] <= Z_int[i][j] * QQ_MATRIX[i][j];
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Pipeline the intermediate result
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp_1[i][j] <= 0;
        end else if (enable_2) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp_1[i][j] <= Z_temp[i][j];
        end
    end

    // -------------------------------------------------------------------------
    // Stage 4: Right shift with rounding to produce quantized output
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Q[i][j] <= 0;
        end else if (enable_3) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++) begin
                    // Bit 11 is used for rounding
                    Q[i][j] <= Z_temp_1[i][j][11] 
                        ? (Z_temp_1[i][j][22:12] + 1) 
                        :  Z_temp_1[i][j][22:12];
                end
        end
    end

    // -------------------------------------------------------------------------
    // Enable signal pipelining: tracks progress through pipeline stages
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_1 <= 0;
            enable_2 <= 0;
            enable_3 <= 0;
            out_enable <= 0;
        end else begin
            enable_1 <= enable;
            enable_2 <= enable_1;
            enable_3 <= enable_2;
            out_enable <= enable_3; // Final stage valid signal
        end
    end

endmodule
