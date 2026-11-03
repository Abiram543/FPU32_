module FPU_Param_Core #(
    parameter EXP_W  = 5,
    parameter FRAC_W = 10
)(
    input  wire                  clk,
    input  wire                  rstn,
    input  wire [EXP_W+FRAC_W:0] a,        // Operand 1  
    input  wire [EXP_W+FRAC_W:0] b,        // Operand 2
    input  wire                  op,       // 1: Add, 0: Subtract
    input  wire                  start,

    output wire [EXP_W+FRAC_W:0] result,
    output wire                  valid_out,
    output reg                   error
);

    localparam DATA_W   = 1 + EXP_W + FRAC_W;     //Data Width
    localparam CNT_W    = $clog2(FRAC_W + 4);     //Count register Width for align
    localparam MAX_SHIFT = FRAC_W + 3; // Max shift for align

    localparam IDLE   = 3'd0,
               ALIGN  = 3'd1,
               EXEC   = 3'd2,
               NORMAL = 3'd3,
               ROUND  = 3'd4,
               DONE   = 3'd5;

    reg [2:0] state;
    reg sign;    //sign is the final sign of the result

     /* eff_sub decides the operation that needs to be performed.
    For subtraction, the sign of B is flipped in b_eff.
    If the sign of b_eff and a are different then it performs subtraction, else addition */
    reg eff_sub;

    //Expanded Mantissa - { Carry, Hidden, Fraction, G, R, S}. Smaller Mantissa doesn't      have carry     
    reg [FRAC_W+4:0] frac_big;
    reg [FRAC_W+3:0] frac_sml;

    //Stores exponent of bigger operand, extra bit for overflow
    reg [EXP_W:0] exp;

    //Counter for alignment
    reg [CNT_W-1:0] cnt;

    //Wires to store b_eff, bigger and smaller operand
    wire [DATA_W-1:0] b_eff;
    wire [DATA_W-1:0] big, sml;
    //Checks magnitude to see if a and b need to be swapped
    wire swap;

    //Hidden bit for both operands
    wire big_hid, sml_hid;

    //Stores bigger, smaller exponents and difference for alignment
    wire [EXP_W-1:0] big_e, sml_e, diff;

    //Error for invalid input
    wire in_err; 

    // Shared ALU
    wire round_up; // Indicates if rounding is needed
    wire [FRAC_W+4:0] alu_b; //Second ALU operand
    wire alu_c; //Carry in for sub
    wire [FRAC_W+4:0] alu; // ALU output
    
    // Rounding and exponent logic
    wire [FRAC_W+1:0] frac_round; //Rounded significand
    wire exp_dec; //Indicates exp increment or decrement
    wire [EXP_W:0] exp_upd; //Exponent after normalization
    wire [EXP_W:0] e_round; //Exponent after rounding
    wire ovf; // Checks exponent overflow
    
    
    //Inverts sign of B if subtraction
    assign b_eff = {b[DATA_W-1] ^ ~op, b[DATA_W-2:0]};
    
    //Compare magnitudes and swaps
    assign swap = (a[DATA_W-2:0] < b_eff[DATA_W-2:0]);
    assign big = swap ? b_eff : a;
    assign sml = swap ? a : b_eff;
    
    //hidden bit ( 1 for normal, 0 for subnormal)
    assign big_hid = |big[FRAC_W+EXP_W-1:FRAC_W];
    assign sml_hid = |sml[FRAC_W+EXP_W-1:FRAC_W];
    
    //Assigns effective exponents of operands
    //If one operand is subnormal it's stored exponent is 0 but it's effective exp is 1
    //difference of effective exponents is calculated for alignment
    assign big_e = big[FRAC_W+EXP_W-1:FRAC_W] | {{(EXP_W-1){1'b0}}, ~big_hid};
    assign sml_e = sml[FRAC_W+EXP_W-1:FRAC_W] | {{(EXP_W-1){1'b0}}, ~sml_hid};
    assign diff = big_e - sml_e;
    
    // Detect Inf and NaN inputs by checking whether the exponent is all ones.
    assign in_err = (&a[FRAC_W+EXP_W-1:FRAC_W]) | (&b[FRAC_W+EXP_W-1:FRAC_W]);
    
    //Round logic ( G & R | S | L)
    assign round_up = frac_big[2] & (frac_big[1] | frac_big[0] | frac_big[3]);
    
    //Select second operand for ALU
    // If state is Round, then round_up otherwise smaller operand or it's complement
    assign alu_b = (state == ROUND) ? {{(FRAC_W+1){1'b0}}, round_up, 3'b000} : ({1'b0, frac_sml} ^ {(FRAC_W+5){eff_sub}});
    
    // Generate the carry-in required for two's complement subtraction.
    assign alu_c = (state != ROUND) & eff_sub;
    
    
    // Shared ALU performs significand addition, subtraction or rounding.
    assign alu = frac_big + alu_b + {{(FRAC_W+4){1'b0}}, alu_c};

    //Removes GRS bits for rounding
    assign frac_round = alu[FRAC_W+4:3];
    
    
    // Determine whether the exponent must be decremented or incremented ( Left Normalization / Right Normalization)
    assign exp_dec = (state == NORMAL) & ~frac_big[FRAC_W+4];
    assign exp_upd = exp + {{EXP_W{exp_dec}}, 1'b1};
    
    
    // Select the exponent after rounding.
    // If rounding generates a carry, use the updated exponent.
    // Otherwise, retain the current exponent.
    assign e_round = frac_round[FRAC_W+1] ? exp_upd : (frac_round[FRAC_W] ? exp : {(EXP_W+1){1'b0}});
    
    // Detect exponent overflow or an all-ones exponent
    assign ovf = e_round[EXP_W] | (&e_round[EXP_W-1:0]);
    
    
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state    <= IDLE;
            sign     <= 1'b0;
            eff_sub  <= 1'b0;
            error    <= 1'b0;
    
            frac_big <= {(FRAC_W+5){1'b0}};
            frac_sml <= {(FRAC_W+4){1'b0}};
    
            exp      <= {(EXP_W+1){1'b0}};
            cnt      <= {CNT_W{1'b0}};
    
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        error <= in_err;
                        sign    <= big[DATA_W-1]; // stores sign of bigger operand
                        eff_sub <= big[DATA_W-1] ^ sml[DATA_W-1]; // if same sign = add, mismatch sign = sub
                        exp <= {1'b0, big_e}; //stores bigger exponent with extra bit for ovf
                        frac_big <= {1'b0,big_hid,big[FRAC_W-1:0],3'b000};
                        frac_sml <= {sml_hid,sml[FRAC_W-1:0],3'b000};
                        
                        cnt <= (diff > MAX_SHIFT) ? MAX_SHIFT : diff; // If difference greater than max shift, it is capped at max shift
                        if (in_err)
                            state <= DONE;
                        else
                            state <= ALIGN;
                    end
                end
    
    
                ALIGN: begin
                    if (|cnt) begin // If cnt != 0
                        // Right-shift the smaller significand to align
                        // its exponent with the larger operand.
                        // Preserve discarded information in the sticky bit.
                        frac_sml <= { 1'b0, frac_sml[FRAC_W+3:2], frac_sml[1] | frac_sml[0] };
                        cnt <= cnt - 1'b1;
                    end else begin
                        state <= EXEC;
                    end
                end
    
    
                EXEC: begin // Perform add/sub using shared alu
                    frac_big <= alu;
                    state <= NORMAL;
                end
    
    
                NORMAL: begin
                    // Exact cancellation, result is 0
                    if (~|frac_big) begin 
                        exp <= {(EXP_W+1){1'b0}};
                        sign <= sign & ~eff_sub;
                        state <= DONE;
                    end
    
                    // Right normalization when addition generates a carry.
                    else if (frac_big[FRAC_W+4]) begin
                        frac_big <= { 1'b0, frac_big[FRAC_W+4:2], frac_big[1] | frac_big[0]};
                        exp <= exp_upd;
                        state <= ROUND;
                    end
    
                    // Left normalization when the leading significand bit is zero.
                    else if (!frac_big[FRAC_W+3] && |exp[EXP_W:1]) begin
                        frac_big <= { frac_big[FRAC_W+3:0], 1'b0};
                        exp <= exp_upd;
                    end
                    else state <= ROUND;
                end

                ROUND: begin
                    if (ovf) begin // Gives result as infinity
                        exp <= {1'b0, {EXP_W{1'b1}}};
                        frac_big[FRAC_W+2:3] <= {FRAC_W{1'b0}};
                    end else begin
                        exp <= e_round;
                        frac_big[FRAC_W+2:3] <= frac_round[FRAC_W-1:0];
                    end
                    state <= DONE;
                end

                DONE: state <= IDLE;
    
                default: state <= IDLE;
    
            endcase
        end
    end
    
    
    // Combine sign, exponent and fraction to generate the FP result.
    assign result = {sign, exp[EXP_W-1:0], frac_big[FRAC_W+2:3]};
    assign valid_out = (state == DONE);
    
endmodule
