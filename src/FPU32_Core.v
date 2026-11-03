`timescale 1ns/1ps

module FPU32_Core (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,
    input  wire        op_sub,   // 0: A+B, 1: A-B
    input  wire [31:0] a,
    input  wire [31:0] b,

    output reg  [31:0] result,
    output reg         busy,
    output reg         done,
    output reg         error
);

    // FSM
    localparam reg [3:0] IDLE      = 4'd0;
    localparam reg [3:0] UNPACK    = 4'd1;
    localparam reg [3:0] COMPARE   = 4'd2;
    localparam reg [3:0] ALIGN     = 4'd3;
    localparam reg [3:0] ADDSUB   = 4'd4;
    localparam reg [3:0] NORMALIZE = 4'd5;
    localparam reg [3:0] ROUND     = 4'd6;
    localparam reg [3:0] PACK      = 4'd7;
    localparam reg [3:0] DONE      = 4'd8;

    reg [3:0] state;
  
    // Operand registers
    reg [31:0] a_reg, b_reg;
    reg        sub_reg;

    reg        sign_a, sign_b;
    reg [7:0]  exp_a, exp_b;
    reg [23:0] sig_a, sig_b;

    reg        sign_large, sign_small;
    reg [7:0]  exp_large;
    reg [7:0]  exp_small;
    reg [7:0]  exp_diff;

    // Extended significands: 24 significant bits + GRS
    reg [27:0] sig_large_ext;
    reg [27:0] sig_small_ext;
    reg [27:0] sig_result;

    reg        result_sign;
    reg [7:0]  result_exp;
    reg [24:0] rounded_sig;

    reg        special_case;
    reg [31:0] special_result;

    integer i;

    // Shift right with sticky-bit preservation
    function automatic [27:0] shift_right_jam;
        input [27:0] value;
        input [7:0]  amount;

        reg sticky;
        integer k;
        begin
            sticky = 1'b0;

            if (amount == 0) begin
                shift_right_jam = value;
            end
            else if (amount >= 28) begin
                shift_right_jam = 28'b0;
                shift_right_jam[0] = |value;
            end
            else begin
                for (k = 0; k < 28; k = k + 1) begin
                    if (k < amount)
                        sticky = sticky | value[k];
                end

                shift_right_jam = value >> amount;
                shift_right_jam[0] =
                    shift_right_jam[0] | sticky;
            end
        end
    endfunction

    // Main sequential controller
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= IDLE;
            result         <= 32'b0;
            busy           <= 1'b0;
            done           <= 1'b0;
            error          <= 1'b0;

            a_reg          <= 32'b0;
            b_reg          <= 32'b0;
            sub_reg        <= 1'b0;

            sign_a         <= 1'b0;
            sign_b         <= 1'b0;
            exp_a          <= 8'b0;
            exp_b          <= 8'b0;
            sig_a          <= 24'b0;
            sig_b          <= 24'b0;

            sign_large     <= 1'b0;
            sign_small     <= 1'b0;
            exp_large      <= 8'b0;
            exp_small      <= 8'b0;
            exp_diff       <= 8'b0;

            sig_large_ext  <= 28'b0;
            sig_small_ext  <= 28'b0;
            sig_result     <= 28'b0;

            result_sign    <= 1'b0;
            result_exp     <= 8'b0;
            rounded_sig    <= 25'b0;

            special_case   <= 1'b0;
            special_result <= 32'b0;
        end
        else begin
            done <= 1'b0;

            case (state)

                IDLE: begin
                    busy <= 1'b0;

                    if (start) begin
                        a_reg   <= a;
                        b_reg   <= b;
                        sub_reg <= op_sub;

                        busy    <= 1'b1;
                        state   <= UNPACK;
                    end
                end

                // Decode normal and subnormal significands.
                // Effective exponent is 1 for exponent field 0.
                UNPACK: begin
                    sign_a <= a_reg[31];
                    sign_b <= b_reg[31] ^ sub_reg;

                    exp_a <= (a_reg[30:23] == 0)
                             ? 8'd1 : a_reg[30:23];

                    exp_b <= (b_reg[30:23] == 0)
                             ? 8'd1 : b_reg[30:23];

                    sig_a <= {
                        (a_reg[30:23] != 0),
                        a_reg[22:0]
                    };

                    sig_b <= {
                        (b_reg[30:23] != 0),
                        b_reg[22:0]
                    };

                    special_case <= 1'b0;
                    special_result <= 32'b0;

                    // NaN propagation: canonical quiet NaN.
                    if ((a_reg[30:23] == 8'hFF &&
                         a_reg[22:0] != 0) ||
                        (b_reg[30:23] == 8'hFF &&
                         b_reg[22:0] != 0)) begin

                        special_case   <= 1'b1;
                        special_result <= 32'h7FC00000;
                    end

                    // Infinity handling.
                    else if (a_reg[30:23] == 8'hFF ||
                             b_reg[30:23] == 8'hFF) begin

                        special_case <= 1'b1;

                        if (a_reg[30:23] == 8'hFF &&
                            b_reg[30:23] == 8'hFF &&
                            (a_reg[31] !=
                             (b_reg[31] ^ sub_reg))) begin
                            // Infinity plus opposite infinity.
                            special_result <= 32'h7FC00000;
                        end
                        else if (a_reg[30:23] == 8'hFF) begin
                            special_result <= {
                                a_reg[31], 8'hFF, 23'b0
                            };
                        end
                        else begin
                            special_result <= {
                                (b_reg[31] ^ sub_reg),
                                8'hFF, 23'b0
                            };
                        end
                    end

                    state <= COMPARE;
                end

                // Handle special values, then order magnitudes.
                COMPARE: begin
                    if (special_case) begin
                        result <= special_result;
                        state  <= DONE;
                    end
                    else if ({exp_a, sig_a} >=
                             {exp_b, sig_b}) begin

                        sign_large <= sign_a;
                        sign_small <= sign_b;

                        exp_large <= exp_a;
                        exp_small <= exp_b;

                        sig_large_ext <= {1'b0, sig_a, 3'b000};
                        sig_small_ext <= {1'b0, sig_b, 3'b000};

                        result_sign <= sign_a;
                        state <= ALIGN;
                    end
                    else begin
                        sign_large <= sign_b;
                        sign_small <= sign_a;

                        exp_large <= exp_b;
                        exp_small <= exp_a;

                        sig_large_ext <= {1'b0, sig_b, 3'b000};
                        sig_small_ext <= {1'b0, sig_a, 3'b000};

                        result_sign <= sign_b;
                        state <= ALIGN;
                    end
                end

                ALIGN: begin
                    exp_diff <= exp_large - exp_small;

                    sig_small_ext <= shift_right_jam(
                        sig_small_ext,
                        exp_large - exp_small
                    );

                    state <= ADDSUB;
                end

                // Shared arithmetic datapath.
                ADDSUB: begin
                    result_exp <= exp_large;

                    if (sign_large == sign_small) begin
                        sig_result <= sig_large_ext +
                                      sig_small_ext;
                        result_sign <= sign_large;
                    end
                    else begin
                        sig_result <= sig_large_ext -
                                      sig_small_ext;
                        result_sign <= sign_large;
                    end

                    state <= NORMALIZE;
                end

                // Iterative normalization: one shift per cycle.
                NORMALIZE: begin
                    if (sig_result == 0) begin
                        result <= 32'b0;
                        state  <= DONE;
                    end
                    else if (sig_result[27]) begin
                        // Carry from addition: shift right and jam.
                        sig_result <= shift_right_jam(
                            sig_result, 8'd1
                        );
                        result_exp <= result_exp + 1'b1;
                    end
                    else if (!sig_result[26] &&
                             result_exp > 8'd1) begin
                        sig_result <= sig_result << 1;
                        result_exp <= result_exp - 1'b1;
                    end
                    else begin
                        state <= ROUND;
                    end
                end

                // Round to nearest, ties to even.
                ROUND: begin
                    rounded_sig <= {
                        1'b0, sig_result[26:3]
                    } + (
                        sig_result[2] &
                        (sig_result[1] |
                         sig_result[0] |
                         sig_result[3])
                    );

                    state <= PACK;
                end

                PACK: begin
                    if (result_exp >= 8'hFF) begin
                        // Overflow -> infinity.
                        result <= {
                            result_sign, 8'hFF, 23'b0
                        };
                    end
                    else if (rounded_sig[24]) begin
                        // Rounding carry.
                        if (result_exp == 8'hFE) begin
                            result <= {
                                result_sign, 8'hFF, 23'b0
                            };
                        end
                        else begin
                            result <= {
                                result_sign,
                                result_exp + 1'b1,
                                rounded_sig[23:1]
                            };
                        end
                    end
                    else if (result_exp == 8'd1 &&
                             !rounded_sig[23]) begin
                        // Subnormal result or signed zero.
                        result <= {
                            result_sign,
                            8'b0,
                            rounded_sig[22:0]
                        };
                    end
                    else begin
                        result <= {
                            result_sign,
                            result_exp,
                            rounded_sig[22:0]
                        };
                    end

                    state <= DONE;
                end

                DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    error <= (result[30:23] == 8'hFF) && (result[22:0] != 0);
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                    busy  <= 1'b0;
                    error <= 1'b0;
                    done  <= 1'b0;
                end

            endcase
        end
    end

endmodule
