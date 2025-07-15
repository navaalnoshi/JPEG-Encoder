


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

`timescale 1ns / 100ps

module cb_quantizer #(
    // Quantization values (can be changed for different quantization levels)
    // JPEG standard Cb/Cr quantization matrix example (simplified for demonstration)
    // The default values here are '1' for maximum precision (minimal compression)
    parameter int Q_MATRIX [8][8] = {
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1}
    }
) (
    input  logic          clk,
    input  logic          rst,
    input  logic          enable,
    input  logic [10:0]   Z [8][8],     // 8x8 matrix of Cb DCT coefficients
    output logic [10:0]   Q [8][8],     // 8x8 matrix of quantized Cb coefficients
    output logic          out_enable    // Output enable signal
);

    // Precomputed reciprocal values (scaled by 4096 = 2^12) to replace division
    // QQ[i][j] = floor(4096 / Q_MATRIX[i][j])
    localparam int C_SHIFT_AMOUNT = 12; // Corresponding to 4096 (2^12)
    localparam int C_ROUND_ADD = 1 << (C_SHIFT_AMOUNT - 1); // 2048 for rounding

    // Precompute reciprocal quantization factors at compile time
    genvar i, j;
    generate
        for (i = 0; i < 8; i++) begin : gen_qq_factors
            for (j = 0; j < 8; j++) begin : gen_qq_elements
                // QM_MATRIX is 13 bits to hold values up to 4096 (when Q_MATRIX element is 1)
                localparam logic [C_SHIFT_AMOUNT:0] QM_MATRIX_VAL = (Q_MATRIX[i][j] == 0) ? 0 : (4096 / Q_MATRIX[i][j]);
                logic [C_SHIFT_AMOUNT:0] QM_matrix_reg; // Register for QM_MATRIX_VAL
                initial QM_matrix_reg = QM_MATRIX_VAL; // Initialize with parameter value
            end
        end
    endgenerate

    // Pipelined intermediate signals
    logic signed [10:0]   Z_s_reg [8][8];       // Stage 1: Signed input
    logic signed [23:0]   Z_mul_reg [8][8];     // Stage 2: Product of Z_s_reg and QM_matrix (11 bits + 13 bits = 24 bits)
    logic signed [23:0]   Z_pipe_reg [8][8];    // Stage 3: Pipelined multiplication result

    // Pipelined enable signals
    logic enable_1, enable_2, enable_3;

    // ---
    // Stage 1: Input latching and sign extension
    // ---
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    Z_s_reg[i][j] <= 0;
                end
            end
        end else if (enable) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    // $signed() casts Z to signed, then assigned to signed Z_s_reg, handled correctly
                    Z_s_reg[i][j] <= $signed(Z[i][j]);
                end
            end
        end
    end

    // ---
    // Stage 2: Multiply with precomputed quantization factors
    // ---
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    Z_mul_reg[i][j] <= 0;
                end
            end
        end else if (enable_1) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    Z_mul_reg[i][j] <= $signed(Z_s_reg[i][j]) * QM_MATRIX_VAL; // QM_MATRIX_VAL is implicitly positive
                end
            end
        end
    end

    // ---
    // Stage 3: Pipeline the intermediate result
    // ---
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    Z_pipe_reg[i][j] <= 0;
                end
            end
        end else if (enable_2) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    Z_pipe_reg[i][j] <= Z_mul_reg[i][j];
                end
            end
        end
    end

    // ---
    // Stage 4: Right shift with rounding to produce quantized output
    // ---
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    Q[i][j] <= 0;
                end
            end
        end else if (enable_3) begin
            for (i = 0; i < 8; i++) begin
                for (j = 0; j < 8; j++) begin
                    // Rounding: add 2048 (half of 4096) before arithmetic right shift
                    // This implements "round half up" for positive numbers and "round half down" for negative numbers
                    Q[i][j] <= ($signed(Z_pipe_reg[i][j]) + C_ROUND_ADD) >>> C_SHIFT_AMOUNT;
                end
            end
        end
    end

    // ---
    // Enable signal pipelining: tracks progress through pipeline stages
    // ---
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_1   <= 1'b0;
            enable_2   <= 1'b0;
            enable_3   <= 1'b0;
            out_enable <= 1'b0;
        end else begin
            enable_1   <= enable;
            enable_2   <= enable_1;
            enable_3   <= enable_2;
            out_enable <= enable_3; // out_enable asserts when Q outputs are valid
        end
    end

endmodule
