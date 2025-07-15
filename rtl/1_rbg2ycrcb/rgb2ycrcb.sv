// Copyright 2025 Maktab-e-Digital Systems Lahore.
// Licensed under the Apache License, Version 2.0, see LICENSE file for details.
// SPDX-License-Identifier: Apache-2.0
// Description:
//   This module converts 8-bit RGB (Red, Green, Blue) pixel data into 8-bit
//   YCbCr format.The transformation uses a fixed-point approximation of the standard ITU-R
//   BT.601 matrix, with scaling by 2^13 to avoid floating point arithmetic.
//   A 2-stage pipeline performs the following:
//     - Stage 1: Multiplication of RGB with fixed-point coefficients
//     - Stage 2: Rounding and clamping to [0, 255]
//   Outputs are registered and synchronized using delayed enable signals.
// Author:Navaal Noshi
// Date:12th July,2025.

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
    logic [7:0] R, G, B;

    logic [21:0] y_product_r, y_product_g, y_product_b;
    logic [21:0] cb_product_r, cb_product_g, cb_product_b;
    logic [21:0] cr_product_r, cr_product_g, cr_product_b;

    logic [21:0] y_sum, cb_sum, cr_sum;
    logic [7:0]  y_out, cb_out, cr_out;

    logic [7:0]  y_rounded, cb_rounded, cr_rounded;

    logic enable_d1, enable_d2;

    // -------------------------------------------------------------------------
    // Output packing: YCbCr = {Cr, Cb, Y}
    // -------------------------------------------------------------------------
    assign data_out = {cr_out, cb_out, y_out};

    // -------------------------------------------------------------------------
    // Stage 1: Multiply and accumulate
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            R <= 0; G <= 0; B <= 0;
            y_product_r <= 0; y_product_g <= 0; y_product_b <= 0;
            cb_product_r <= 0; cb_product_g <= 0; cb_product_b <= 0;
            cr_product_r <= 0; cr_product_g <= 0; cr_product_b <= 0;
            y_sum <= 0; cb_sum <= 0; cr_sum <= 0;
        end else if (enable) begin
            R <= data_in[7:0];
            G <= data_in[15:8];
            B <= data_in[23:16];

            y_product_r  <= Y1  * R;
            y_product_g  <= Y2  * G;
            y_product_b  <= Y3  * B;

            cb_product_r <= CB1 * R;
            cb_product_g <= CB2 * G;
            cb_product_b <= CB3 * B;

            cr_product_r <= CR1 * R;
            cr_product_g <= CR2 * G;
            cr_product_b <= CR3 * B;

            y_sum  <= y_product_r + y_product_g + y_product_b;
            cb_sum <= 22'd2097152 - cb_product_r - cb_product_g + cb_product_b;
            cr_sum <= 22'd2097152 + cr_product_r - cr_product_g - cr_product_b;
        end
    end

    // -------------------------------------------------------------------------
    // Stage 2: Normalize, round, and clamp without ternary
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            y_out  <= 0;
            cb_out <= 0;
            cr_out <= 0;
        end else if (enable_d1) begin
            // Y: rounding
            y_rounded = y_sum[21:14];
            if (y_sum[13]) begin
                y_rounded = y_rounded + 1;
            end
            if (y_rounded > 8'd255) begin
                y_out <= 8'd255;
            end else begin
                y_out <= y_rounded;
            end

            // Cb: rounding
            cb_rounded = cb_sum[21:14];
            if (cb_sum[13]) begin
                cb_rounded = cb_rounded + 1;
            end
            if (cb_rounded > 8'd255) begin
                cb_out <= 8'd255;
            end else begin
                cb_out <= cb_rounded;
            end

            // Cr: rounding
            cr_rounded = cr_sum[21:14];
            if (cr_sum[13]) begin
                cr_rounded = cr_rounded + 1;
            end
            if (cr_rounded > 8'd255) begin
                cr_out <= 8'd255;
            end else begin
                cr_out <= cr_rounded;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Stage 3: Enable signal pipeline alignment
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
