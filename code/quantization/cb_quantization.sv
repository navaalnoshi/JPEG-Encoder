/* This module comes after the dct module.  The 64 matrix entries calculated after
performing the 2D DCT are inputs to this quantization module.  This module quantizes
the entire 8x8 block of Cb values.  The outputs from this module
are the quantized Cb values for one 8x8 block. */

`timescale 1ns / 100ps

module cb_quantizer(
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [10:0] Z [8][8], // 2D array for Z inputs
    output logic [10:0] Q [8][8], // 2D array for Q outputs
    output logic        out_enable
);

    // Quantization values (parameters can be changed)
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

    // Pre-calculated multiplication factors (4096 / Q)
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

    logic [31:0] Z_int [8][8]; // Use logic [31:0] for signed integers, SystemVerilog handles implicitly
    logic [22:0] Z_temp [8][8];
    logic [22:0] Z_temp_1 [8][8];

    logic enable_1, enable_2, enable_3;

    // Stage 1: Input Latching and Sign Extension
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_int[i][j] <= 0;
                end
            end
        end else if (enable) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    // Assign and implicitly sign-extend Z[i][j] to Z_int[i][j] (32-bit)
                    Z_int[i][j] <= Z[i][j];
                end
            end
        end
    end

    // Stage 2: Multiplication
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp[i][j] <= 0;
                end
            end
        end else if (enable_1) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp[i][j] <= Z_int[i][j] * QQ_MATRIX[i][j];
                end
            end
        end
    end

    // Stage 3: Pipeline Register for Multiplication Result
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp_1[i][j] <= 0;
                end
            end
        end else if (enable_2) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp_1[i][j] <= Z_temp[i][j];
                end
            end
        end
    end

    // Stage 4: Quantization and Rounding
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Q[i][j] <= 0;
                end
            end
        end else if (enable_3) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    // Rounding based on the bit in the 11th place
                    Q[i][j] <= Z_temp_1[i][j][11] ? (Z_temp_1[i][j][22:12] + 1) : Z_temp_1[i][j][22:12];
                end
            end
        end
    end

    // Enable Signal Pipelining
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
            out_enable <= enable_3;
        end
    end

endmodule