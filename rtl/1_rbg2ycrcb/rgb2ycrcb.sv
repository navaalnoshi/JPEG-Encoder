// -----------------------------------------------------------------------------
// Module: RGB2YCBCR
//
// Description:
//   This module converts 8-bit RGB (Red, Green, Blue) pixel data into 8-bit
//   YCbCr format: 
//     - Y  (Luminance) 
//     - Cb (Chroma Blue difference)
//     - Cr (Chroma Red difference)
//
//   The transformation uses a fixed-point approximation of the standard ITU-R
//   BT.601 matrix, with scaling by 2^13 to avoid floating point arithmetic.
//
//   A 2-stage pipeline performs the following:
//     - Stage 1: Multiplication of RGB with fixed-point coefficients
//     - Stage 2: Rounding and clamping to [0, 255]
//
//   Outputs are registered and synchronized using delayed enable signals.
//
// Inputs:
//   - clk        : Clock signal
//   - rst        : Active-high synchronous reset
//   - enable     : Enable signal to process new input pixel
//   - data_in    : 24-bit RGB input {Blue[23:16], Green[15:8], Red[7:0]}
//
// Outputs:
//   - data_out   : 24-bit YCbCr output {Cr[23:16], Cb[15:8], Y[7:0]}
//   - enable_out : Indicates output data is valid (2-cycle latency)
// -----------------------------------------------------------------------------

`timescale 1ns / 100ps

module RGB2YCBCR (
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [23:0] data_in,    // RGB pixel: {B[23:16], G[15:8], R[7:0]}
    output logic [23:0] data_out,   // YCbCr output: {Cr[23:16], Cb[15:8], Y[7:0]}
    output logic        enable_out  // Valid output signal
);

    // -------------------------------------------------------------------------
    // Fixed-point coefficients (scaled by 2^13 for 13-bit precision)
    // -------------------------------------------------------------------------
    localparam logic [13:0] Y1  = 14'd4899;  // 0.299  * 8192
    localparam logic [13:0] Y2  = 14'd9617;  // 0.587  * 8192
    localparam logic [13:0] Y3  = 14'd1868;  // 0.114  * 8192

    localparam logic [13:0] CB1 = 14'd2764;  // 0.1687 * 8192
    localparam logic [13:0] CB2 = 14'd5428;  // 0.3313 * 8192
    localparam logic [13:0] CB3 = 14'd8192;  // 0.5000 * 8192

    localparam logic [13:0] CR1 = 14'd8192;  // 0.5000 * 8192
    localparam logic [13:0] CR2 = 14'd6860;  // 0.4187 * 8192
    localparam logic [13:0] CR3 = 14'd1332;  // 0.0813 * 8192

    // -------------------------------------------------------------------------
    // Internal signals
    // -------------------------------------------------------------------------
    logic [21:0] y_product_r, y_product_g, y_product_b;
    logic [21:0] cb_product_r, cb_product_g, cb_product_b;
    logic [21:0] cr_product_r, cr_product_g, cr_product_b;

    logic [21:0] y_sum, cb_sum, cr_sum; // Raw sums before normalization
    logic [7:0]  y_out, cb_out, cr_out; // Final outputs

    logic enable_d1, enable_d2;         // Enable pipeline stages

    // -------------------------------------------------------------------------
    // Output packing: YCbCr = {Cr, Cb, Y}
    // -------------------------------------------------------------------------
    assign data_out = {cr_out, cb_out, y_out};

    // -------------------------------------------------------------------------
    // Stage 1: Multiply RGB by corresponding coefficients and compute sums
    //          (Intermediate results are in fixed-point format)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            y_product_r  <= 0; y_product_g  <= 0; y_product_b  <= 0;
            cb_product_r <= 0; cb_product_g <= 0; cb_product_b <= 0;
            cr_product_r <= 0; cr_product_g <= 0; cr_product_b <= 0;
            y_sum        <= 0; cb_sum       <= 0; cr_sum       <= 0;
        end else if (enable) begin
            // Extract RGB values
            logic [7:0] R = data_in[7:0];
            logic [7:0] G = data_in[15:8];
            logic [7:0] B = data_in[23:16];

            // Y = 0.299R + 0.587G + 0.114B
            y_product_r  <= Y1  * R;
            y_product_g  <= Y2  * G;
            y_product_b  <= Y3  * B;

            // Cb = 128 - 0.1687R - 0.3313G + 0.5B
            cb_product_r <= CB1 * R;
            cb_product_g <= CB2 * G;
            cb_product_b <= CB3 * B;

            // Cr = 128 + 0.5R - 0.4187G - 0.0813B
            cr_product_r <= CR1 * R;
            cr_product_g <= CR2 * G;
            cr_product_b <= CR3 * B;

            // DC offset = 128 << 13 = 128 * 8192 = 2_097_152
            y_sum  <= y_product_r + y_product_g + y_product_b;
            cb_sum <= 22'd2097152 - cb_product_r - cb_product_g + cb_product_b;
            cr_sum <= 22'd2097152 + cr_product_r - cr_product_g - cr_product_b;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Normalize results (divide by 8192 = 2^13), round and clamp to 255
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            y_out  <= 0;
            cb_out <= 0;
            cr_out <= 0;
        end else if (enable_d1) begin
            // Round Y: check bit 13 (LSB of fractional part)
            y_out <= y_sum[13] ? y_sum[21:14] + 1 : y_sum[21:14];

            // Round and saturate Cb
            cb_out <= (cb_sum[13] && cb_sum[21:14] != 8'd255) 
                    ? cb_sum[21:14] + 1 
                    : cb_sum[21:14];

            // Round and saturate Cr
            cr_out <= (cr_sum[13] && cr_sum[21:14] != 8'd255) 
                    ? cr_sum[21:14] + 1 
                    : cr_sum[21:14];
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Enable signal pipeline to align with 2-stage data processing
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_d1  <= 0;
            enable_d2  <= 0;
            enable_out <= 0;
        end else begin
            enable_d1  <= enable;
            enable_d2  <= enable_d1;
            enable_out <= enable_d2;
        end
    end

endmodule
