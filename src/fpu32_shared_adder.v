module FPU_Core(
    input  wire        clk,
    input  wire        rstn,
    input  wire [31:0] a,           // operand 1
    input  wire [31:0] b,           // operand 2
    input  wire        op,          // 1: a + b, 0: a - b
    input  wire        start,

    output wire [31:0] result,
    output wire        valid_out,
    output reg         error
);

    localparam IDLE = 3'd0, ALIGN = 3'd1, EXEC = 3'd2, NORMAL = 3'd3, ROUND = 3'd4, DONE = 3'd5;

   
    reg [2:0]  state;
    reg        sign;                // sign of result
    reg        eff_sub;             // 1: magnitudes are subtracted
    reg [27:0] frac_big;            // {Carry, Hidden, Frac[22:0], G, R, S}  bigger operand / running result
    reg [26:0] frac_sml;            // {Hidden, Frac[22:0], G, R, S}          smaller operand (aligned)
    reg [7:0]  exp;                 // max pre-round value is 255 (254 + carry)
    reg [4:0]  cnt;                 // alignment shifts left


    wire [31:0] b_eff;
    wire        swap;
    wire [31:0] big, sml;
    wire        big_hid, sml_hid;
    wire [7:0]  big_e, sml_e, diff;
    wire        invalid_input;
    // shared adder
    wire        round_up;
    wire [27:0] alu_b;
    wire        alu_c;
    wire [27:0] sum;
    wire [24:0] frac_round;
    // shared exponent unit
    wire        exp_dec;
    wire [7:0]  exp_pm;
    wire [7:0]  e_round;
    wire        ovf;


    assign b_eff   = {b[31] ^ ~op, b[30:0]};                  // flip sign of b for subtraction
    assign swap    = (a[30:0] < b_eff[30:0]);
    assign big     = swap ? b_eff : a;
    assign sml     = swap ? a : b_eff;

    // hidden bit is 0 for zero/subnormal; their effective exponent is 1
    assign big_hid = |big[30:23];
    assign sml_hid = |sml[30:23];
    assign big_e   = big[30:23] | {7'b0, ~big_hid};
    assign sml_e   = sml[30:23] | {7'b0, ~sml_hid};
    assign diff    = big_e - sml_e;

    // exp == FF : Inf (frac==0) or NaN (frac!=0)
    assign invalid_input = (&a[30:23]) | (&b[30:23]);

    // rounding (RNE): round_up = G & (R | S | L)
    assign round_up   = frac_big[2] & (frac_big[1] | frac_big[0] | frac_big[3]);

    // shared adder: EXEC = big +/- sml ; ROUND = big + (round_up << 3)
    assign alu_b      = (state == ROUND) ? {24'b0, round_up, 3'b0} : ({1'b0, frac_sml} ^ {28{eff_sub}});
    assign alu_c      = (state != ROUND) & eff_sub;
    assign sum        = frac_big + alu_b + {27'b0, alu_c};
    assign frac_round = sum[27:3];                            // valid in ROUND; [24] = carry-out (then [22:0] == 0)

    // shared exponent unit: NORMAL left shift = -1 ; NORMAL carry and ROUND carry = +1
    assign exp_dec    = (state == NORMAL) & ~frac_big[27];
    assign exp_pm     = exp + {{7{exp_dec}}, 1'b1};
    assign e_round    = frac_round[24] ? exp_pm : (frac_round[23] ? exp : 8'd0);   // 0 => subnormal result
    assign ovf        = &exp;

    assign result     = {sign, exp, frac_big[25:3]};
    assign valid_out  = (state == DONE);

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state    <= IDLE;
            sign     <= 1'b0;
            eff_sub  <= 1'b0;
            frac_big <= 28'b0;
            frac_sml <= 27'b0;
            exp      <= 8'b0;
            cnt      <= 5'b0;
            error    <= 1'b0;
        end else begin
            error <= 1'b0;                                    // pulse: high only in the DONE cycle after an invalid start
            case (state)
                IDLE: begin
                    if (start) begin                          // wrapper ensures one start per transaction
                        if (invalid_input) begin
                            error <= 1'b1;
                            state <= DONE;
                        end else begin
                            sign     <= big[31];              // sign of bigger operand
                            eff_sub  <= big[31] ^ sml[31];    // different signs => subtract magnitudes
                            exp      <= big_e;
                            frac_big <= {1'b0, big_hid, big[22:0], 3'b000};   // {C, H, Frac, G, R, S}
                            frac_sml <= {sml_hid, sml[22:0], 3'b000};         // {H, Frac, G, R, S}
                            cnt      <= (diff > 8'd27) ? 5'd27 : diff[4:0];   // >=27 shifts: everything is sticky
                            state    <= ALIGN;
                        end
                    end
                end

                ALIGN: begin
                    if (|cnt) begin
                        frac_sml <= {1'b0, frac_sml[26:2], frac_sml[1] | frac_sml[0]};   // 1 bit/cycle, sticky OR
                        cnt      <= cnt - 5'd1;
                    end else state <= EXEC;
                end

                EXEC: begin
                    frac_big <= sum;
                    state    <= NORMAL;
                end

                NORMAL: begin
                    if (~|frac_big) begin                     // exact cancellation: +0 (RNE), or -0 for (-0)+(-0)
                        exp   <= 8'd0;
                        sign  <= sign & ~eff_sub;
                        state <= DONE;
                    end else if (frac_big[27]) begin          // carry: shift right, exp+1
                        frac_big <= {1'b0, frac_big[27:2], frac_big[1] | frac_big[0]};
                        exp      <= exp_pm;
                        state    <= ROUND;
                    end else if (!frac_big[26] && |exp[7:1]) begin   // left normalise one bit per cycle, exp-1
                        frac_big <= {frac_big[26:0], 1'b0};
                        exp      <= exp_pm;
                    end else begin
                        state <= ROUND;
                    end
                end

                ROUND: begin
                    if (ovf) begin
                        exp <= 8'd255;
                        frac_big[25:3] <= 23'd0;
                    end else begin
                        exp <= e_round;
                        frac_big[25:3] <= frac_round[22:0];   // carry-out => already 0
                    end
                    state <= DONE;
                end

                DONE: state <= IDLE;

                default: state <= IDLE;
            endcase
        end
    end

endmodule
