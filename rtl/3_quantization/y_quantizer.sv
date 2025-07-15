// -----------------------------------------------------------------------------
// Module: y_quantizer
//
// Description:
//   This module performs quantization on an 8x8 block of luminance (Y) values
//   after 2D Discrete Cosine Transform (DCT). Quantization reduces the precision
//   of DCT coefficients, enabling lossy compression for JPEG encoding.
//
//   Instead of dividing each DCT coefficient by the corresponding value in a 
//   quantization matrix (Q), this module precomputes reciprocals scaled by 4096,
//   and performs multiplication followed by right-shift to approximate division.
//
//   The module operates using a 3-stage pipeline:
//     Stage 1: Sign-extension and latching of input DCT coefficients
//     Stage 2: Multiplication with quantization multipliers
//     Stage 3: Final rounding and shift to produce quantized values
// -----------------------------------------------------------------------------

`timescale 1ns / 100ps

module y_quantizer #(
    // Default Quantization values (can be changed for different quantization levels)
    // An 8x8 matrix of quantization values.
    // Example for a typical JPEG luminance quantization matrix (scaled):
    // For Q=1, all multipliers are 4096.
    parameter Y_Q_MATRIX = {
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
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,

    // Input Z values (8x8 block of 11-bit signed DCT coefficients)
    input  logic [10:0] Z [7:0][7:0],

    // Output Q values (8x8 block of 11-bit quantized coefficients)
    output logic [10:0] Q [7:0][7:0],
    output logic        out_enable
);

    // Pre-computed multipliers (4096 / Q[i][j])
    // These are constant and will be synthesized as fixed values.
    logic [12:0] QM [7:0][7:0];

    generate
        for (genvar i = 0; i < 8; i++) begin : gen_qm_rows
            for (genvar j = 0; j < 8; j++) begin : gen_qm_cols
                localparam Q_val = Y_Q_MATRIX[i*8 + j];
                initial begin
                    if (Q_val == 0) QM[i][j] = 0; // Avoid division by zero, though Q values should be non-zero
                    else QM[i][j] = 4096 / Q_val;
                end
            end
        end
    endgenerate

    // Pipeline Stage Registers
    logic signed [22:0] Z_temp [7:0][7:0];     // Stage 2 result
    logic signed [22:0] Z_temp_1 [7:0][7:0];   // Stage 3 result
    logic signed [31:0] Z_int [7:0][7:0];      // Stage 1 registered and sign-extended

    logic enable_1, enable_2; // enable_3 is 'out_enable' for the final stage

    // -------------------------------------------------------------------------
    // Stage 1: Input registration and sign-extension
    // -------------------------------------------------------------------------
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
                    Z_int[i][j] <= {{21{Z[i][j][10]}}, Z[i][j]}; // Sign-extension
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Multiply input with quantization multipliers
    // -------------------------------------------------------------------------
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
                    Z_temp[i][j] <= Z_int[i][j] * QM[i][j];
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Pipeline intermediate result (multiplied value)
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    // Stage 4: Final quantization by right shift and rounding
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Q[i][j] <= 0;
                end
            end
        end else if (out_enable) begin // Using out_enable for this stage for clarity
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                 // Formula: Quantized_Value = (DCT_Coeff × [4096/Qij] + 2048) >> 12
                 // The addition of 2048 (1 << 11) before the right shift by 12 implements
                // rounding to the nearest integer, with halves rounding away from zero,
                // aligning with the specified JPEG rounding method.
                      Q[i][j] <= (Z_temp_1[i][j] + 2048) >>> 12;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline Control: Shift enable signal across 3 stages
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_1   <= 0;
            enable_2   <= 0;
            out_enable <= 0;
        end else begin
            enable_1   <= enable;
            enable_2   <= enable_1;
            out_enable <= enable_2; // out_enable now signals that the Stage 4 output is valid
        end
    end

endmodule
