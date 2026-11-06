// ============================================================================
// Parameterized IEEE 754 Floating-Point Adder/Subtractor (Combinational)
// ============================================================================
// Mostly combinational approach: single-cycle alignment (barrel shift),
// add/subtract, and normalisation. 3-state FSM: IDLE → SPECIAL → OPERATION.
//
//   Format     | EXP_W | MAN_W | WIDTH
//   -----------|-------|-------|------
//   FP8  E4M3  |   4   |   3   |   8
//   FP8  E5M2  |   5   |   2   |   8
//   FP16       |   5   |  10   |  16    ← default
//   BF16       |   8   |   7   |  16
//   FP32       |   8   |  23   |  32
//   FP64       |  11   |  52   |  64
// ============================================================================
`default_nettype none

module fpu_core #(
    parameter EXP_W = 5,            // exponent width
    parameter MAN_W = 10,           // mantissa width (excluding hidden bit)
    localparam WIDTH = 1 + EXP_W + MAN_W // total bit-width
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,      // Assert when valid operands + opcode are ready
    input  wire               op,         // 0 = ADD, 1 = SUB
    input  wire [WIDTH-1:0]   op_a,
    input  wire [WIDTH-1:0]   op_b,
    output reg  [WIDTH-1:0]   result,
    output reg                done,       // Done flag
    output reg                err         // Error flag
);

    // ----------------------------------------------------------------
    // Derived parameters
    // ----------------------------------------------------------------
    localparam MAN_FULL  = MAN_W + 1;                   // mantissa + hidden bit
    localparam SUM_W     = MAN_W + 2;                   // carry + hidden + mantissa
    localparam LZ_W      = $clog2(MAN_FULL + 1) > 0    // leading-zero counter width
                         ? $clog2(MAN_FULL + 1) : 1;
    localparam [EXP_W-1:0] EXP_INF = {EXP_W{1'b1}};   // all-ones exponent (inf/nan)
    localparam [EXP_W-1:0] EXP_MAX = EXP_INF - 1'b1;   // max normal exponent

    // ----------------------------------------------------------------
    // FSM states
    // ----------------------------------------------------------------
    localparam S_IDLE      = 2'd0;
    localparam S_SPECIAL   = 2'd1;
    localparam S_OPERATION = 2'd2;

    // ----------------------------------------------------------------
    // Swap / sign handling
    // ----------------------------------------------------------------
    wire sign_eff = op_b[WIDTH-1] ^ op;                 // SUB = ADD with B's sign flipped
    wire a_bigger = (op_a[WIDTH-2:0] >= op_b[WIDTH-2:0]);

    wire sign_big   = a_bigger ? op_a[WIDTH-1] : sign_eff;
    wire sign_small = a_bigger ? sign_eff      : op_a[WIDTH-1];

    wire [EXP_W-1:0] exp_big   = a_bigger ? op_a[WIDTH-2:MAN_W] : op_b[WIDTH-2:MAN_W];
    wire [EXP_W-1:0] exp_small = a_bigger ? op_b[WIDTH-2:MAN_W] : op_a[WIDTH-2:MAN_W];
    wire [MAN_W-1:0] mant_big   = a_bigger ? op_a[MAN_W-1:0] : op_b[MAN_W-1:0];
    wire [MAN_W-1:0] mant_small = a_bigger ? op_b[MAN_W-1:0] : op_a[MAN_W-1:0];

    wire sub_operation = sign_big ^ sign_small;         // 1 = subtract magnitudes

    // ----------------------------------------------------------------
    // Mantissa with hidden bit
    // ----------------------------------------------------------------
    wire [MAN_FULL-1:0] m_big   = {1'b1, mant_big};
    wire [MAN_FULL-1:0] m_small = {1'b1, mant_small};

    // ----------------------------------------------------------------
    // Alignment: barrel-shift smaller mantissa right by exponent diff
    // (if diff >= MAN_FULL the shifted mantissa becomes zero — correct)
    // ----------------------------------------------------------------
    wire [EXP_W-1:0]    exp_diff        = exp_big - exp_small;
    wire [MAN_FULL-1:0] m_small_aligned = m_small >> exp_diff;

    // ----------------------------------------------------------------
    // Add / Subtract
    // ----------------------------------------------------------------
    wire [SUM_W-1:0] mant_sum = sub_operation
        ? {1'b0, m_big} - {1'b0, m_small_aligned}
        : {1'b0, m_big} + {1'b0, m_small_aligned};

    // ----------------------------------------------------------------
    // Leading-zero count (priority encoder, combinational)
    // Scans from LSB to MSB; last match wins → gives leading-zero count.
    // ----------------------------------------------------------------
    function [LZ_W-1:0] leading_zeroes;
        input [MAN_FULL-1:0] x;
        integer i;
        begin
            leading_zeroes = MAN_W[LZ_W-1:0];          // default: all zeros
            for (i = 0; i < MAN_FULL; i = i + 1)
                if (x[i]) leading_zeroes = MAN_W[LZ_W-1:0] - i[LZ_W-1:0];
        end
    endfunction

    wire [LZ_W-1:0]    lz        = leading_zeroes(mant_sum[MAN_FULL-1:0]);

    /* verilator lint_off UNUSEDSIGNAL */
    wire [MAN_FULL-1:0] mant_norm = mant_sum[MAN_FULL-1:0] << lz;
    /* verilator lint_on UNUSEDSIGNAL */

    // Extend lz to exponent width for safe comparison and subtraction
    wire [EXP_W-1:0] lz_ext = {{(EXP_W-LZ_W){1'b0}}, lz};

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
    reg [1:0] ps, ns;

    // Present-state register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) ps <= S_IDLE;
        else        ps <= ns;
    end

    // Next-state + output logic (combinational)
    always @(*) begin
        ns     = ps;
        result = {WIDTH{1'b0}};
        done   = 1'b0;
        err    = 1'b0;

        case (ps)
            S_IDLE: begin
                if (start) ns = S_SPECIAL;
            end

            S_SPECIAL: begin
                // --- Infinity / NaN (exponent = all ones) ---
                if ((exp_big == EXP_INF) || (exp_small == EXP_INF)) begin
                    err  = 1'b1;
                    done = 1'b1;
                    ns   = S_IDLE;
                end
                // --- Denormalized numbers → error ---
                else if (((exp_big   == {EXP_W{1'b0}}) && (mant_big   != {MAN_W{1'b0}})) ||
                         ((exp_small == {EXP_W{1'b0}}) && (mant_small != {MAN_W{1'b0}}))) begin
                    err  = 1'b1;
                    done = 1'b1;
                    ns   = S_IDLE;
                end
                // --- Smaller operand is zero ---
                else if ((exp_small == {EXP_W{1'b0}}) && (mant_small == {MAN_W{1'b0}})) begin
                    result = {sign_big, exp_big, mant_big};
                    done   = 1'b1;
                    ns     = S_IDLE;
                end
                else begin
                    ns = S_OPERATION;
                end
            end

            S_OPERATION: begin
                done = 1'b1;
                ns   = S_IDLE;

                // Carry-out from addition → shift right 1, exponent + 1
                if (mant_sum[SUM_W-1]) begin
                    if (exp_big == EXP_MAX)
                        result = {sign_big, EXP_INF, {MAN_W{1'b0}}};       // overflow → ±∞
                    else
                        result = {sign_big, exp_big + 1'b1, mant_sum[MAN_FULL-1:1]};
                end
                // Exact cancellation → +0
                else if (mant_sum[MAN_FULL-1:0] == {MAN_FULL{1'b0}})
                    result = {WIDTH{1'b0}};
                // Underflow — lz exceeds available exponent room
                else if (lz_ext >= exp_big)
                    result = {sign_big, {(WIDTH-1){1'b0}}};
                // Left-normalise
                else
                    result = {sign_big, exp_big - lz_ext, mant_norm[MAN_W-1:0]};
            end

            default: ns = S_IDLE;
        endcase
    end

endmodule
