// -----------------------------------------------------------------------------
// Module: y_quantizer
// Description:
//   This module performs quantization on an 8x8 block of input values (`Z`),
//   which are typically the result of a 2D Discrete Cosine Transform (DCT).
//   It uses a quantization matrix (`Q`) to scale the DCT coefficients
//   according to the JPEG compression standard.
//   The quantization process is pipelined over 3 clock cycles.
// -----------------------------------------------------------------------------
`timescale 1ns / 100ps
module y_quantizer #(
    // Quantization matrix parameter (can be modified for custom quantization)
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
    input  logic        rst,              // Active-high reset signal
    input  logic        enable,           // Enable signal for input capture
    input  logic [10:0] Z[8][8],          // 8x8 input matrix of DCT values
    output logic [10:0] Q_out[8][8],      // 8x8 output matrix of quantized values
    output logic        out_enable        // Indicates output is valid
);

    // -------------------------------------------------------------------------
    // Pre-computed multiplier matrix (4096 / Q[i][j])
    // Division is replaced by multiplication followed by right shift of 12 bits.
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

    // Quantization multipliers (assigned from QQ matrix)
    logic [12:0] QM[8][8];
    for (genvar i = 0; i < 8; i++) begin : gen_qm_assign
        for (genvar j = 0; j < 8; j++) begin
            assign QM[i][j] = QQ[i][j];
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline Registers
    // -------------------------------------------------------------------------
    logic signed [31:0] Z_int[8][8];     // Stage 1: Signed extension of input Z
    logic signed [22:0] Z_temp[8][8];    // Stage 2: Z_int * QM
    logic signed [22:0] Z_temp_1[8][8];  // Stage 3: Pipelined result of Z_temp

    // Enable signal pipeline (3-cycle delay)
    logic enable_dly[3];

    // -------------------------------------------------------------------------
    // Pipeline: Enable signal shift register
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_dly <= '0;
            out_enable <= 1'b0;
        end else begin
            enable_dly[0] <= enable;
            enable_dly[1] <= enable_dly[0];
            enable_dly[2] <= enable_dly[1];
            out_enable <= enable_dly[2]; // Output valid after 3 cycles
        end
    end

    // -------------------------------------------------------------------------
    // Stage 1: Sign-extend the 11-bit input values to 32-bit signed integers
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_int[i][j] <= '0;
                end
            end
        end else if (enable) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_int[i][j] <= signed'(Z[i][j]); // Automatic sign-extension
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Multiply Z_int by the precomputed multiplier QM
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp[i][j] <= '0;
                end
            end
        end else if (enable_dly[0]) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp[i][j] <= Z_int[i][j] * QM[i][j]; // Avoids division
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Pipeline the result of multiplication for 1 cycle
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp_1[i][j] <= '0;
                end
            end
        end else if (enable_dly[1]) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_temp_1[i][j] <= Z_temp[i][j]; // Store intermediate result
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 4: Final quantization - shift right by 12 bits with rounding
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Q_out[i][j] <= '0;
                end
            end
        end else if (enable_dly[2]) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    // Rounding: if the 12th bit is 1, add 1 after shift
                    Q_out[i][j] <= (Z_temp_1[i][j][11]) 
                        ? (Z_temp_1[i][j] >>> 12) + 1 
                        : (Z_temp_1[i][j] >>> 12);
                end
            end
        end
    end

endmodule
