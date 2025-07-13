// This module converts 8-bit Red, Green, and Blue pixel data into 8-bit Y, Cb, and Cr values.
// The output values are unsigned and range from 0 to 255.
//
// Input:
//   data_in[7:0]   : Red pixel value
//   data_in[15:8]  : Green pixel value
//   data_in[23:16] : Blue pixel value
//
// Output:
//   data_out[7:0]   : Y (Luminance) value
//   data_out[15:8]  : Cb (Chroma Blue) value
//   data_out[23:16] : Cr (Chroma Red) value

`timescale 1ns / 100ps

module RGB2YCBCR (
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [23:0] data_in,
    output logic [23:0] data_out,
    output logic        enable_out
);

    // Fixed-point coefficients (scaled by 2^13 for 8-bit input to 21-bit product)
    // These coefficients are derived from standard YCbCr conversion matrices.
    localparam logic [13:0] Y1  = 14'd4899;  // 0.299 * 2^13
    localparam logic [13:0] Y2  = 14'd9617;  // 0.587 * 2^13
    localparam logic [13:0] Y3  = 14'd1868;  // 0.114 * 2^13
    localparam logic [13:0] CB1 = 14'd2764;  // 0.1687 * 2^13
    localparam logic [13:0] CB2 = 14'd5428;  // 0.3313 * 2^13
    localparam logic [13:0] CB3 = 14'd8192;  // 0.5 * 2^13
    localparam logic [13:0] CR1 = 14'd8192;  // 0.5 * 2^13
    localparam logic [13:0] CR2 = 14'd6860;  // 0.4187 * 2^13
    localparam logic [13:0] CR3 = 14'd1332;  // 0.0813 * 2^13

    // Internal signals for pipelined stages
    logic [21:0] y_product_r, y_product_g, y_product_b;
    logic [21:0] cb_product_r, cb_product_g, cb_product_b;
    logic [21:0] cr_product_r, cr_product_g, cr_product_b;

    logic [21:0] y_sum, cb_sum, cr_sum;
    logic [7:0]  y_out, cb_out, cr_out;

    logic enable_d1, enable_d2; // Enable delay line for output synchronization

    // Assign output based on calculated Y, Cb, Cr values
    assign data_out = {cr_out, cb_out, y_out};

    // --- First Pipeline Stage: Multiplication and Summation ---
    // Calculates the intermediate Y, Cb, and Cr sums.
    // The constant 22'd2097152 is 128 * 2^13, used for the DC offset in Cb and Cr.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            y_product_r  <= '0; y_product_g  <= '0; y_product_b  <= '0;
            cb_product_r <= '0; cb_product_g <= '0; cb_product_b <= '0;
            cr_product_r <= '0; cr_product_g <= '0; cr_product_b <= '0;
            y_sum        <= '0; cb_sum       <= '0; cr_sum       <= '0;
        end else if (enable) begin
            // Perform multiplications for Y
            y_product_r  <= Y1  * data_in[7:0];   // R
            y_product_g  <= Y2  * data_in[15:8];  // G
            y_product_b  <= Y3  * data_in[23:16]; // B

            // Perform multiplications for Cb
            cb_product_r <= CB1 * data_in[7:0];
            cb_product_g <= CB2 * data_in[15:8];
            cb_product_b <= CB3 * data_in[23:16];

            // Perform multiplications for Cr
            cr_product_r <= CR1 * data_in[7:0];
            cr_product_g <= CR2 * data_in[15:8];
            cr_product_b <= CR3 * data_in[23:16];

            // Sum products, adding 128 (scaled by 2^13) for Cb and Cr offset
            y_sum  <= y_product_r + y_product_g + y_product_b;
            cb_sum <= 22'd2097152 - cb_product_r - cb_product_g + cb_product_b;
            cr_sum <= 22'd2097152 + cr_product_r - cr_product_g - cr_product_b;
        end
    end

    // --- Second Pipeline Stage: Rounding and Saturation ---
    // Shifts right by 13 (division by 2^13) to get the final value.
    // Performs rounding by checking the 13th bit (LSB of the fractional part).
    // Implements saturation to 255 for Cb/Cr.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            y_out  <= '0;
            cb_out <= '0;
            cr_out <= '0;
        end else if (enable_d1) begin // Use delayed enable for correct pipeline operation
            // Round Y value: add 1 if fractional part is 0.5 or more (bit 13 is set)
            y_out  <= y_sum[13] ? y_sum[21:14] + 1 : y_sum[21:14];

            // Round and saturate Cb: add 1 for rounding, saturate at 255
            cb_out <= (cb_sum[13] && cb_sum[21:14] != 8'd255) ? cb_sum[21:14] + 1 : cb_sum[21:14];
            // Round and saturate Cr: add 1 for rounding, saturate at 255
            cr_out <= (cr_sum[13] && cr_sum[21:14] != 8'd255) ? cr_sum[21:14] + 1 : cr_sum[21:14];
        end
    end

    // --- Third Pipeline Stage: Enable Signal Delay ---
    // Propagates the enable signal through the pipeline to synchronize with the output data.
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_d1  <= '0;
            enable_d2  <= '0;
            enable_out <= '0;
        end else begin
            enable_d1  <= enable;
            enable_d2  <= enable_d1;
            enable_out <= enable_d2; // Output enable synchronized with data_out
        end
    end

endmodule
