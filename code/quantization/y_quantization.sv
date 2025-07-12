`timescale 1ns / 100ps

module y_quantizer #(
    // Quantization values (can be changed for different levels)
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
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    input  logic [10:0] Z[8][8], // 2D array for input Z values
    output logic [10:0] Q_out[8][8], // 2D array for output Q values
    output logic        out_enable
);

    // Multipliers to avoid division (4096 = 2^12)
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

    logic [12:0] QM[8][8]; // Multipliers assigned from QQ
    for (genvar i = 0; i < 8; i++) begin : gen_qm_assign
        for (genvar j = 0; j < 8; j++) begin
            assign QM[i][j] = QQ[i][j];
        end
    end

    // Internal registers for pipelining
    // Use signed for intermediate calculations to handle negative numbers correctly
    logic signed [31:0] Z_int[8][8];      // Input Z values, sign-extended
    logic signed [22:0] Z_temp[8][8];     // Result of Z_int * QM
    logic signed [22:0] Z_temp_1[8][8];   // Pipelined Z_temp

    logic enable_dly[3]; // Pipelined enable signal

    // Pipelining enable signal
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            enable_dly <= '0;
            out_enable <= 1'b0;
        end else begin
            enable_dly[0] <= enable;
            enable_dly[1] <= enable_dly[0];
            enable_dly[2] <= enable_dly[1];
            out_enable <= enable_dly[2];
        end
    end

    // Stage 1: Register Z inputs and sign-extend
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
                    Z_int[i][j] <= signed'(Z[i][j]); // SystemVerilog handles sign extension automatically with signed'() cast
                end
            end
        end
    end

    // Stage 2: Perform multiplication (Z_int * QM)
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
                    Z_temp[i][j] <= Z_int[i][j] * QM[i][j];
                end
            end
        end
    end

    // Stage 3: Pipeline Z_temp
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
                    Z_temp_1[i][j] <= Z_temp[i][j];
                end
            end
        end
    end

    // Stage 4: Perform final quantization (right shift and rounding)
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
                    // Rounding based on the 11th bit (equivalent to bit 0 of the fractional part)
                    Q_out[i][j] <= (Z_temp_1[i][j][11]) ? (Z_temp_1[i][j] >>> 12) + 1 : (Z_temp_1[i][j] >>> 12);
                end
            end
        end
    end

endmodule