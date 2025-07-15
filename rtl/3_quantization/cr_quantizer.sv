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
    parameter Q_MATRIX_ROWS = 8,
    parameter Q_MATRIX_COLS = 8,
    parameter INPUT_WIDTH   = 11, // Width of input Z coefficients (e.g., 11 for signed -1024 to 1023)
    parameter SHIFT_AMOUNT  = 12  // For dividing by 4096 (2^12) for quantization
) (
    input  logic              clk,
    input  logic              rst,
    input  logic              enable, // Overall module enable

    // Input Z values (2D array of signed DCT coefficients)
    input  logic [INPUT_WIDTH-1:0] Z [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0],

    // Output Q values (2D array of quantized coefficients)
    output logic [INPUT_WIDTH-1:0] Q [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0],
    output logic              out_enable // Indicates when Q output is valid
);

    // Default Quantization values (can be changed via parameter override)
    // For Q=1, all quantization values are 1. This means multiplication by 4096.
    parameter integer Q_VALS [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0] = {
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1}
    };

    // Derived widths for internal signals
    // QM (Quantization Multiplier): Needs SHIFT_AMOUNT + 1 bits (e.g., 4096 is 2^12, needs 13 bits: [12:0])
    localparam MULT_FACTOR_WIDTH = SHIFT_AMOUNT + 1;
    // The multiplication result (Z * QM): Max value approx (2^INPUT_WIDTH-1) * (2^SHIFT_AMOUNT+1-1)
    // So, max bit width = INPUT_WIDTH + SHIFT_AMOUNT. (e.g., 11 + 12 = 23 bits)
    localparam MULT_RESULT_WIDTH = INPUT_WIDTH + SHIFT_AMOUNT;
    // Z_int width (Stage 1 register): Wider to accommodate sign extension for multiplication.
    // The y_quantizer example used 32 bits, which is a common DSP width.
    // Ensure it's wide enough for INPUT_WIDTH + SHIFT_AMOUNT and any potential intermediate overflow
    // The formula (Z_s3[i][j] + 2048) implies the intermediate result Z_s3 has room for +2048 (11 bits) before shift.
    // A 32-bit width for Z_int (as in y_quantizer) is generally safe for intermediate stages.
    localparam Z_INT_WIDTH = 32;

    // Pre-computed multipliers (4096 / Q_VALS[i][j])
    logic [MULT_FACTOR_WIDTH-1:0] QM [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];

    // Calculate QM values using a generate block (constant synthesis)
    genvar i_gen, j_gen; // Using unique genvar names to avoid potential conflicts
    generate
        for (i_gen = 0; i_gen < Q_MATRIX_ROWS; i_gen++) begin : gen_qm_rows
            for (j_gen = 0; j_gen < Q_MATRIX_COLS; j_gen++) begin : gen_qm_cols
                // Extract Q_val from the parameter array
                localparam Q_val_elem = Q_VALS[i_gen][j_gen];
                // Assign the precomputed multiplier. Handle division by zero for safety, though Q_VALS should be >= 1.
                assign QM[i_gen][j_gen] = (Q_val_elem == 0) ? 0 : (4096 / Q_val_elem);
            end
        end
    endgenerate

    // Pipelined Registers for data
    // Z_int: Stage 1 registered input, possibly sign-extended to a wider bus
    logic signed [Z_INT_WIDTH-1:0] Z_int [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];
    // Z_temp: Stage 2 result (Z * QM)
    logic signed [MULT_RESULT_WIDTH-1:0] Z_temp [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];
    // Z_temp_1: Stage 3 result (pipelined Z_temp)
    logic signed [MULT_RESULT_WIDTH-1:0] Z_temp_1 [Q_MATRIX_ROWS-1:0][Q_MATRIX_COLS-1:0];

    // Pipelined Enable signals
    logic enable_s1, enable_s2; // enable_s3 is directly `out_enable`

    // --- Pipelined Stages ---
    genvar r, c; // Loop variables for always_ff blocks

    // Stage 1: Input registration and sign-extension
    // The loop iterates over the 2D arrays for conciseness.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    Z_int[r][c] <= '0; // Reset all bits to 0
                end
            end
        end else if (enable) begin
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    // Sign-extension: Replicate MSB to fill Z_INT_WIDTH
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
        end else if (enable_s1) begin // Process only when Stage 1 data is valid
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
        end else if (enable_s2) begin // Process only when Stage 2 data is valid
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
        end else if (out_enable) begin // Process only when Stage 3 data is valid
            for (r = 0; r < Q_MATRIX_ROWS; r++) begin
                for (c = 0; c < Q_MATRIX_COLS; c++) begin
                    // Rounding: Add 2048 (which is 1 << (SHIFT_AMOUNT - 1)) before right shift by SHIFT_AMOUNT.
                    // This implements "rounding to nearest, halves away from zero" for signed numbers.
                    // Arithmetic right shift (>>>) preserves the sign.
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
            out_enable <= 1'b0;
        end else begin
            enable_s1  <= enable;      // Enable for Stage 2
            enable_s2  <= enable_s1;   // Enable for Stage 3
            out_enable <= enable_s2;   // Enable for Stage 4 (final output valid)
        end
    end

endmodule
