// -----------------------------------------------------------------------------
// Module: cr_quantizer
//
// Description:
//   This module performs quantization on an 8x8 block of Cr chrominance values,
//   typically obtained after applying a 2D Discrete Cosine Transform (DCT).
//   It replaces division with multiplication using precomputed scale factors
//   (4096 / Q[i][j]), followed by a right shift of 12 bits (i.e., division by 4096).
//
//   The module is pipelined in 3 stages:
//     Stage 1: Sign-extend and register the 11-bit DCT input
//     Stage 2: Multiply with quantization multipliers
//     Stage 3: Apply rounding and right shift to produce quantized output
// -----------------------------------------------------------------------------

`timescale 1ns / 100ps

module cr_quantizer #(
    // Use a single bit vector for the quantization matrix, like y_quantizer
    // This is an 8x8 matrix, with each value being 64'd1 in this default.
    // If you need actual JPEG quantization, these '1's should be replaced
    // with the appropriate Q_luminance or Q_chrominance values.
    // Each value occupies 64 bits to match the example, though a smaller width
    // (e.g., 8-16 bits) would be sufficient for typical Q values.
    parameter CR_Q_MATRIX = {
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,
        64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1,  64'd1
    }
) (
    input  logic              clk,
    input  logic              rst,
    input  logic              enable, // Overall module enable

    // Input Z values (8x8 block of 11-bit signed DCT coefficients)
    input  logic [10:0] Z [7:0][7:0],

    // Output Q values (8x8 block of 11-bit quantized coefficients)
    output logic [10:0] Q [7:0][7:0],
    output logic              out_enable
);

    // Hardcoded dimensions and widths as implied by the Y_Q_MATRIX and port declarations
    localparam Q_MATRIX_ROWS = 8;
    localparam Q_MATRIX_COLS = 8;
    localparam INPUT_WIDTH   = 11;
    localparam SHIFT_AMOUNT  = 12; // For dividing by 4096 (2^12) for quantization

    // Derived widths for internal signals
    localparam MULT_FACTOR_WIDTH = SHIFT_AMOUNT + 1; // 4096 needs 13 bits (0 to 4095)
    localparam MULT_RESULT_WIDTH = INPUT_WIDTH + SHIFT_AMOUNT; // 11 + 12 = 23 bits
    localparam Z_INT_WIDTH = 32; // Align with y_quantizer's explicit 32-bit width for Z_int

    // Pre-computed multipliers (4096 / Q_val[i][j])
    logic [MULT_FACTOR_WIDTH-1:0] QM [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];

    // Calculate QM values using a generate block, extracting from the bit vector
    genvar i_gen, j_gen;
    generate
        for (i_gen = 0; i_gen < Q_MATRIX_ROWS; i_gen++) begin : gen_qm_rows
            for (j_gen = 0; j_gen < Q_MATRIX_COLS; j_gen++) begin : gen_qm_cols
                // Extract 64-bit value from the flattened CR_Q_MATRIX
                localparam Q_val_raw = CR_Q_MATRIX[(i_gen * Q_MATRIX_COLS + j_gen) * 64 +: 64];
                // Take the lowest part of the 64-bit value, assuming Q values fit in int
                localparam Q_val_elem = Q_val_raw; // Implicit conversion or cast if needed, but for '1' it's fine
                // The original y_quantizer example used an initial block here,
                // which is unusual for synthesizable constants. 'assign' is better.
                assign QM[i_gen][j_gen] = (Q_val_elem == 0) ? 0 : (4096 / Q_val_elem);
            end
        end
    endgenerate

    // Pipelined Registers for data
    logic signed [Z_INT_WIDTH-1:0] Z_int [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];
    logic signed [MULT_RESULT_WIDTH-1:0] Z_temp [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];
    logic signed [MULT_RESULT_WIDTH-1:0] Z_temp_1 [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];

    // Pipelined Enable signals
    logic enable_s1, enable_s2; // enable_s3 is directly `out_enable`

    // --- Pipelined Stages ---
    genvar r, c; // Loop variables for always_ff blocks

    // Stage 1: Input registration and sign-extension
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Z_int[r][c] <= '0;
                end
            end
        end else if (enable) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Z_int[r][c] <= {{Z_INT_WIDTH - INPUT_WIDTH{Z[r][c][INPUT_WIDTH-1]}}, Z[r][c]};
                end
            end
        end
    end

    // Stage 2: Multiply input with quantization multipliers
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Z_temp[r][c] <= '0;
                end
            end
        end else if (enable_s1) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Z_temp[r][c] <= Z_int[r][c] * QM[r][c];
                end
            end
        end
    end

    // Stage 3: Pipeline intermediate result (multiplied value)
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Z_temp_1[r][c] <= '0;
                end
            end
        end else if (enable_s2) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Z_temp_1[r][c] <= Z_temp[r][c];
                end
            end
        end
    end

    // Stage 4: Final quantization by right shift and rounding
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Q[r][c] <= '0;
                end
            end
        end else if (out_enable) begin
            // Formula: Quantized_Value = (DCT_Coeff × [4096/Qij] + 2048) >> 12
            // The addition of 2048 (1 << 11) before the right shift by 12 implements
            // rounding to the nearest integer, with halves rounding away from zero,
            // aligning with the specified JPEG rounding method.
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Q[r][c] <= (Z_temp_1[r][c] + (1 << (SHIFT_AMOUNT - 1))) >>> SHIFT_AMOUNT;
                end
            end
        end
    end

    // --- Pipeline Control: Shift enable signal across stages ---
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_s1  <= 1'b0;
            enable_s2  <= 1'b0;
            out_enable <= 1'b0; // This is the enable for the final output stage
        end else begin
            enable_s1  <= enable;
            enable_s2  <= enable_s1;
            out_enable <= enable_s2; // out_enable signals that the Stage 4 output is valid
        end
    end

endmodule
