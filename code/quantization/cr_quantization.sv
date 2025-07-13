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
//
// Inputs:
//   - clk        : Clock signal
//   - rst        : Active-high synchronous reset
//   - enable     : Enables new input into pipeline
//   - Z[8][8]    : 8x8 input matrix of 11-bit signed DCT values
//
// Outputs:
//   - Q[8][8]    : 8x8 matrix of quantized values (11-bit each)
//   - out_enable : Valid output signal (asserted 3 clock cycles after enable)
// -----------------------------------------------------------------------------

`timescale 1ns / 100ps

module cr_quantizer(
    input logic         clk,
    input logic         rst,
    input logic         enable,
    input logic [10:0]  Z [8][8],      // Input 8x8 Cr DCT coefficients
    output logic [10:0] Q [8][8],      // Output 8x8 quantized coefficients
    output logic        out_enable     // Indicates valid output
);

    // -------------------------------------------------------------------------
    // Parameter: Quantization matrix (can be adjusted for quality/compression)
    // -------------------------------------------------------------------------
    parameter int Q_VALS [8][8] = {
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1},
        '{1, 1, 1, 1, 1, 1, 1, 1}
    };

    // -------------------------------------------------------------------------
    // Precomputed multipliers (4096 / Q[i][j])
    // Replace division with multiplication followed by >> 12
    // -------------------------------------------------------------------------
    parameter int QQ_VALS [8][8] = {
        {4096/Q_VALS[0][0], 4096/Q_VALS[0][1], 4096/Q_VALS[0][2], 4096/Q_VALS[0][3], 4096/Q_VALS[0][4], 4096/Q_VALS[0][5], 4096/Q_VALS[0][6], 4096/Q_VALS[0][7]},
        {4096/Q_VALS[1][0], 4096/Q_VALS[1][1], 4096/Q_VALS[1][2], 4096/Q_VALS[1][3], 4096/Q_VALS[1][4], 4096/Q_VALS[1][5], 4096/Q_VALS[1][6], 4096/Q_VALS[1][7]},
        {4096/Q_VALS[2][0], 4096/Q_VALS[2][1], 4096/Q_VALS[2][2], 4096/Q_VALS[2][3], 4096/Q_VALS[2][4], 4096/Q_VALS[2][5], 4096/Q_VALS[2][6], 4096/Q_VALS[2][7]},
        {4096/Q_VALS[3][0], 4096/Q_VALS[3][1], 4096/Q_VALS[3][2], 4096/Q_VALS[3][3], 4096/Q_VALS[3][4], 4096/Q_VALS[3][5], 4096/Q_VALS[3][6], 4096/Q_VALS[3][7]},
        {4096/Q_VALS[4][0], 4096/Q_VALS[4][1], 4096/Q_VALS[4][2], 4096/Q_VALS[4][3], 4096/Q_VALS[4][4], 4096/Q_VALS[4][5], 4096/Q_VALS[4][6], 4096/Q_VALS[4][7]},
        {4096/Q_VALS[5][0], 4096/Q_VALS[5][1], 4096/Q_VALS[5][2], 4096/Q_VALS[5][3], 4096/Q_VALS[5][4], 4096/Q_VALS[5][5], 4096/Q_VALS[5][6], 4096/Q_VALS[5][7]},
        {4096/Q_VALS[6][0], 4096/Q_VALS[6][1], 4096/Q_VALS[6][2], 4096/Q_VALS[6][3], 4096/Q_VALS[6][4], 4096/Q_VALS[6][5], 4096/Q_VALS[6][6], 4096/Q_VALS[6][7]},
        {4096/Q_VALS[7][0], 4096/Q_VALS[7][1], 4096/Q_VALS[7][2], 4096/Q_VALS[7][3], 4096/Q_VALS[7][4], 4096/Q_VALS[7][5], 4096/Q_VALS[7][6], 4096/Q_VALS[7][7]}
    };

    logic [12:0] QM [8][8];              // Quantization multipliers
    logic [22:0] Z_temp [8][8];          // After multiplication
    logic [22:0] Z_temp_1 [8][8];        // Pipeline intermediate result
    logic [31:0] Z_int [8][8];           // Sign-extended DCT inputs

    logic enable_1, enable_2, enable_3;  // Pipeline control signals

    // -------------------------------------------------------------------------
    // Wire QQ_VALS into QM (logic array) using generate block
    // -------------------------------------------------------------------------
    generate
        for (genvar i = 0; i < 8; i++) begin : gen_qm_wire
            for (genvar j = 0; j < 8; j++) begin
                assign QM[i][j] = QQ_VALS[i][j];
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Stage 1: Sign-extend the 11-bit inputs to 32 bits
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_int[i][j] <= 0;
        end else if (enable) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_int[i][j] <= {{21{Z[i][j][10]}}, Z[i][j]}; // Manual sign extension
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Multiply with precomputed quantization factors
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp[i][j] <= 0;
        end else if (enable_1) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Z_temp[i][j] <= Z_int[i][j] * QM[i][j];
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Pipeline intermediate multiplication results
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
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
    // Stage 4: Final rounding and right shift to produce quantized output
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++)
                    Q[i][j] <= 0;
        end else if (enable_3) begin
            for (int i = 0; i < 8; i++)
                for (int j = 0; j < 8; j++) begin
                    // Bit 11 of Z_temp_1 is used for rounding before shift
                    Q[i][j] <= Z_temp_1[i][j][11] 
                             ? Z_temp_1[i][j][22:12] + 1 
                             : Z_temp_1[i][j][22:12];
                end
        end
    end

    // -------------------------------------------------------------------------
    // Enable signal pipelining (3-stage shift register)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            enable_1    <= 0;
            enable_2    <= 0;
            enable_3    <= 0;
            out_enable  <= 0;
        end else begin
            enable_1    <= enable;
            enable_2    <= enable_1;
            enable_3    <= enable_2;
            out_enable  <= enable_3;
        end
    end

endmodule
