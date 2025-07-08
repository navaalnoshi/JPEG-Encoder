
/* This module converts the incoming Red, Green, and Blue 8-bit pixel data
into Y, Cb, and Cr 8-bit values.  The output values will be unsigned
in the range of 0 to 255. 
data_in contains the Red pixel value in bits [7:0], Green in bits [15:8],
and Blue in bits [23:16].
data_out contains the Y value in bits [7:0], Cb value in bits [15:8],
and Cr balue in bits [23:16].*/

`timescale 1ns / 100ps

module RGB2YCBCR (
    input  logic         clk,
    input  logic         rst,
    input  logic         enable,
    input  logic [23:0]  data_in,
    output logic [23:0]  data_out,
    output logic         enable_out
);

    // Fixed-point coefficients (scaled by 2^13)
    localparam logic [13:0] Y1  = 14'd4899;
    localparam logic [13:0] Y2  = 14'd9617;
    localparam logic [13:0] Y3  = 14'd1868;
    localparam logic [13:0] CB1 = 14'd2764;
    localparam logic [13:0] CB2 = 14'd5428;
    localparam logic [13:0] CB3 = 14'd8192;
    localparam logic [13:0] CR1 = 14'd8192;
    localparam logic [13:0] CR2 = 14'd6860;
    localparam logic [13:0] CR3 = 14'd1332;

    // Internal signals
    logic [21:0] Y1_product, Y2_product, Y3_product;
    logic [21:0] CB1_product, CB2_product, CB3_product;
    logic [21:0] CR1_product, CR2_product, CR3_product;

    logic [21:0] Y_temp, CB_temp, CR_temp;
    logic [7:0]  Y, CB, CR;

    logic enable_1, enable_2;

    assign data_out = {CR, CB, Y};

    // First pipeline stage: multiplication and accumulation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Y1_product  <= 0; Y2_product  <= 0; Y3_product  <= 0;
            CB1_product <= 0; CB2_product <= 0; CB3_product <= 0;
            CR1_product <= 0; CR2_product <= 0; CR3_product <= 0;
            Y_temp      <= 0; CB_temp     <= 0; CR_temp     <= 0;
        end else if (enable) begin
            Y1_product  <= Y1  * data_in[7:0];    // R
            Y2_product  <= Y2  * data_in[15:8];   // G
            Y3_product  <= Y3  * data_in[23:16];  // B

            CB1_product <= CB1 * data_in[7:0];
            CB2_product <= CB2 * data_in[15:8];
            CB3_product <= CB3 * data_in[23:16];

            CR1_product <= CR1 * data_in[7:0];
            CR2_product <= CR2 * data_in[15:8];
            CR3_product <= CR3 * data_in[23:16];

            Y_temp  <= Y1_product + Y2_product + Y3_product;
            CB_temp <= 22'd2097152 - CB1_product - CB2_product + CB3_product; // +128
            CR_temp <= 22'd2097152 + CR1_product - CR2_product - CR3_product; // +128
        end
    end

    // Second stage: rounding and saturation control
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Y  <= 8'd0;
            CB <= 8'd0;
            CR <= 8'd0;
        end else if (enable) begin
            // Rounding
            Y  <= Y_temp[13]  ? Y_temp[21:14]  + 1 : Y_temp[21:14];
            CB <= (CB_temp[13] && CB_temp[21:14] != 8'd255) ? CB_temp[21:14] + 1 : CB_temp[21:14];
            CR <= (CR_temp[13] && CR_temp[21:14] != 8'd255) ? CR_temp[21:14] + 1 : CR_temp[21:14];
        end
    end

    // Third stage: enable_out delay line
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_1    <= 0;
            enable_2    <= 0;
            enable_out  <= 0;
        end else begin
            enable_1    <= enable;
            enable_2    <= enable_1;
            enable_out  <= enable_2;
        end
    end

endmodule
