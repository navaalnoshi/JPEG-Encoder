timescale 1ns / 100ps

module cr_quantizer(
    input logic         clk,
    input logic         rst,
    input logic         enable,
    input logic [10:0]  Z [8][8],
    output logic [10:0] Q [8][8],
    output logic        out_enable
);

    // Quantization values (can be changed for different quantization levels)
    parameter int Q_VALS [8][8] = {
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1}
    };

    // Pre-calculated multiplication values (4096 / Q_VAL)
    // These values are needed to get around actual division.
    // Instead, you multiply by these values and then divide by 4096 (right shift by 12).
    // This is a lossy process, but the additional loss of precision is negligible.
    // To decrease the loss, you could use a larger number than 4096.
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

    logic [12:0] QM [8][8];
    logic [22:0] Z_temp [8][8];
    logic [22:0] Z_temp_1 [8][8];

    logic enable_1, enable_2, enable_3;

    // Convert parameters to wires
    generate
        for (genvar i = 0; i < 8; i++) begin : gen_qm_wire
            for (genvar j = 0; j < 8; j++) begin
                assign QM[i][j] = QQ_VALS[i][j];
            end
        end
    endgenerate

    // Stage 1: Register Z inputs and sign extend
    logic [31:0] Z_int [8][8];
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_int[i][j] <= 0;
                end
            end
        end else if (enable) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Z_int[i][j] <= {{21{Z[i][j][10]}}, Z[i][j]}; // Sign extend
                end
            end
        end
    end

    // Stage 2: Perform multiplication
    always_ff @(posedge clk) begin
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

    // Stage 3: Register intermediate results
    always_ff @(posedge clk) begin
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

    // Stage 4: Perform rounding and assign to Q outputs
    always_ff @(posedge clk) begin
        if (rst) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    Q[i][j] <= 0;
                end
            end
        end else if (enable_3) begin
            for (int i = 0; i < 8; i++) begin
                for (int j = 0; j < 8; j++) begin
                    // Rounding based on the bit in the 11th place
                    Q[i][j] <= Z_temp_1[i][j][11] ? Z_temp_1[i][j][22:12] + 1 : Z_temp_1[i][j][22:12];
                end
            end
        end
    end

    // Enable signal pipeline
    always_ff @(posedge clk) begin
        if (rst) begin
            enable_1 <= 0;
            enable_2 <= 0;
            enable_3 <= 0;
            out_enable <= 0;
        end else begin
            enable_1 <= enable;
            enable_2 <= enable_1;
            enable_3 <= enable_2;
            out_enable <= enable_3;
        end
    end

endmodule