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
//
// Inputs:
//   - clk        : Clock signal
//   - rst        : Active-high synchronous reset
//   - enable     : Enables data load into pipeline
//   - Z[8][8]    : 8x8 input matrix of 11-bit signed DCT values
//
// Outputs:
//   - Q_out[8][8]: 8x8 output matrix of quantized 11-bit values
//   - out_enable : Output valid signal asserted 3 cycles after 'enable'
// -----------------------------------------------------------------------------

`timescale 1ns / 100ps

module y_quantizer #(
    // Quantization matrix (modifiable for different compression quality)
    parameter integer Q[8][8] = '{
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1}
    }
) (
    input  logic        clk,              // Clock signal
    input  logic        rst,              // Synchronous reset (active high)
    input  logic        enable,           // Input load enable
    input  logic [10:0] Z[8][8],          // 8x8 block of DCT inputs
    output logic [10:0] Q_out[8][8],      // 8x8 block of quantized outputs
    output logic        out_enable        // Valid output signal (after 3 cycles)
);

    // -------------------------------------------------------------------------
    // Pre-computed multipliers (4096 / Q[i][j])
    // Replace costly division with multiplication + right-shift
    // -------------------------------------------------------------------------
    localparam integer QQ[8][8] = {
        {4096/Q[0][0], 4096/Q[0][1], 4096/Q[0][2], 4096/Q[0][3], 4096/Q[0][4], 4096/Q[0][5], 4096/Q[0][6], 4096/Q[0][7]},
        {4096/Q[1][0], 4096/Q[1][1], 4096/Q[1][2], 4096/Q[1][3], 4096/Q[1][4], 4096/Q[1][5], 4096/Q[1][6], 4096/Q[1][7]},
        {4096/Q[2][0], 4096/Q[2][1], 4096/Q[2][2], 4096/Q[2][3], 4096/Q[2][4], 4096/Q[2][5], 4096/Q[2][6], 4096/Q[2][7]},
        {4096/Q[3][0], 4096/Q[3][1], 4096/Q[3][2], 4096/Q[3][3], 4096/Q[3][4], 4096/Q[3][5], 4096/Q[3][6], 4096/Q[3][7]},
        {4096/Q[4][0], 4096/Q[4][1], 4096/Q[4][2], 4096/Q[4][3], 4096/Q[4][4], 4096/Q[4][5], 4096/Q[4][6], 4096/Q[4][7]},
        {4096/Q[5][0], 4096/Q[5][1], 4096/Q[5][2], 4096/Q[5][3], 4096/Q[5][4], 4096/Q[5][5], 4096/Q[5][6], 4096/Q[5][7]},
        {4096/Q[6][0], 4096/Q[6][1], 4096/Q[6][2], 4096/Q[6][3], 4096/Q[6][4], 4096/Q[6][5], 4096/Q[6][6], 4096/Q[6][7]},
        {4096/Q[7][0], 4096/Q[7][1], 4096/Q[7][2], 4096/Q[7][3], 4096/Q[7][4], 4096/Q[7][5], 4096/Q[7][6], 4096/Q[7][7]}
    };

    // Wire assignments for quantization multipliers
    logic [12:0] QM[8][8];
    for (genvar i = 0; i < 8; i++) begin : gen_qm_assign
        for (genvar j = 0; j < 8; j++) begin
            assign QM[i][j] = QQ[i][j];
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline Stage Registers
    // -------------------------------------------------------------------------
    logic signed [31:0] Z_int[8][8];     // Stage 1: Input sign-extension
    logic signed [22:0] Z_temp[8][8];    // Stage 2: Multiplication result
    logic signed [22:0] Z_temp_1[8][8];  // Stage 3: Pipelined result for rounding

    logic enable_dly[3];                 // Pipeline control signals

    // -------------------------------------------------------------------------
    // Pipeline Control: shift enable signal across 3 stages
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_dly <= '0;
            out_enable <= 1'b0;
        end else begin
            enable_dly[0] <= enable;
            enable_dly[1] <= enable_dly[0];
            enable_dly[2] <= enable_dly[1];
            out_enable    <= enable_dly[2]; // Output ready on 3rd cycle
        end
    end

    // -------------------------------------------------------------------------
    // Stage 1: Input registration and sign-extension
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_int[i][j] <= '0;
        end else if (enable) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_int[i][j] <= signed'(Z[i][j]); // Sign-extend from 11 to 32 bits
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Multiply input with quantization multipliers (no division)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp[i][j] <= '0;
        end else if (enable_dly[0]) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp[i][j] <= Z_int[i][j] * QM[i][j];
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Pipeline intermediate result to align with control
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp_1[i][j] <= '0;
        end else if (enable_dly[1]) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp_1[i][j] <= Z_temp[i][j];
        end
    end

    // -------------------------------------------------------------------------
    // Stage 4: Final quantization by right shift and rounding
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Q_out[i][j] <= '0;
        end else if (enable_dly[2]) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++) begin
                    // Round result by checking bit 11 of fractional part
                    Q_out[i][j] <= Z_temp_1[i][j][11] 
                        ? (Z_temp_1[i][j] >>> 12) + 1 
                        :  (Z_temp_1[i][j] >>> 12);
                end
        end
    end

endmodule
