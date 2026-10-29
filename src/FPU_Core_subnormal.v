module FPU_Core(
    input wire clk,
    input wire rstn,
    input wire [31:0] a, //Operand 1
    input wire [31:0] b, //Operand 2
    input wire op, // 1 for addition and 0 for sub
    input wire start,
    
    output wire [31:0] result,
    output wire valid_out,
    output reg error
    );
    
    localparam IDLE = 3'd0, ALIGN = 3'd1, EXEC = 3'd2, NORMAL = 3'd3, ROUND = 3'd4, DONE = 3'd5;
    
    reg [2:0] state;
    reg sign; // sign is the final sign of result
    
     /* eff_sub decides the operation that needs to be performed.
    For subtraction, the sign of B is flipped in b_eff.
    If the sign of b_eff and a are different then it performs subtraction, else addition */
    reg eff_sub;
    //Mantissa of bigger operand - 28 bits: Carry, Hidden, [25:3]Fraction, G,R,S
    reg [27:0] frac_big;
    //Mantissa of smaller operand - doesn't have Carry
    reg [26:0] frac_sml;
    //Stores exponent of bigger operand, extra bit for overflow
    reg [8:0] exp;
    //Counter for alignment
    reg [4:0]cnt;
    
    //Wires to store b_eff, bigger and smaller operand
    wire [31:0] b_eff, big,sml;
    //Stores bigger, smaller exponents and difference for alignment
    wire [7:0] big_e, sml_e, diff;
    //Hidden bit for both operands
    wire big_hid, sml_hid;
    //Checks magnitude to see if a and b need to be swapped
    wire swap;
    wire a_nan,b_nan,a_inf, b_inf;
    
    //Rounding logic wires
    wire ovf, round_up;
    wire [24:0] frac_round;
    wire [8:0] e_round;
    wire a_zero,b_zero;
    
    
    //Rounding Logic 
    // round up = G & (R | S | L)
    assign round_up = frac_big[2] & (frac_big[1] | frac_big[0] | frac_big[3]);
    //Rounded Fraction, extra bit to check carry when rounding
    assign frac_round = {1'b0, frac_big[26:3]} + round_up;
    //Rounded exponent - if carry occurs when rounding, increment exponent
    //If no carry then check hidden bit - if 0, then subnormal, therefore exponent is 0; else retain exponent
    assign e_round = frac_round[24] ? (exp + 9'd1) : (frac_round[23] ? exp : 9'd0);
    //Checks if overflow occurs ( e_round >=255)
    assign ovf = (e_round[8] || (&e_round[7:0]));
    
    //Swaps sign of b if subtraction
    assign b_eff = {b[31] ^ ~op, b[30:0]};
    //Checks magnitude of both operands without sign and assigns accordingly
    //Result gets sign of bigger operand
    assign swap = (a[30:0] < b_eff[30:0]);
    assign big = swap ? b_eff : a;
    assign sml = swap ? a : b_eff;
    
    //hidden bit to signify subnormal numbers (for subnormal exponent shd be 0)
    assign big_hid = |big[30:23];
    assign sml_hid = |sml[30:23];
    
    //Assigns effective exponents of operands
    //If one operand is subnormal it's stored exponent is 0 but it's effective exp is 1
    //difference of effective exponents is calculated for alignment
    assign big_e = big[30:23] | {7'b0, ~big_hid};
    assign sml_e = sml[30:23] | {7'b0, ~sml_hid};
    assign diff = big_e - sml_e;
   
    assign a_nan = (&a[30:23]) & (|a[22:0]);
    assign b_nan = (&b[30:23]) & (|b[22:0]);
    assign a_inf = (&a[30:23]) & (&a[22:0]);
    assign b_inf = (&b[30:23]) & (&b[22:0]);
    //Checks if inputs are zero
    assign a_zero = ~|a[30:0];
    assign b_zero = ~|b[30:0];
    
    assign invalid_input = a_nan | b_nan | a_inf | b_inf;
    
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin // All registers are reset
            state <= IDLE;
            sign <= 1'b0;
            eff_sub <= 1'b0;
            frac_big <= 28'b0;
            frac_sml <= 27'b0;
            exp <= 9'b0;
            cnt <= 5'b0;
            error <= 1'b0;
        end else begin
            error <= 1'b0;
            case(state) 
                IDLE:begin
                    if(start) begin // Wrapper ensures only 1 start per transaction
                        if (invalid_input) begin
                            error <= 1'b1;
                            state <= DONE;
                        end else begin
                            state <= ALIGN;
                            sign <= big[31]; // stores sign of bigger operand
                            eff_sub <= big[31] ^ sml[31]; // if same sign = add, mismatch sign = sub
                            exp <= {1'b0, big_e}; //stores bigger exponent with extra bit for ovf
                            
                            frac_big <= {1'b0, big_hid,big[22:0], 3'b000}; // {C, H, Frac, G,R,S}
                            frac_sml <= {sml_hid, sml[22:0], 3'b000}; //{H,Frac,G,R,S)
                            
                            cnt <= (diff > 8'd27)? 5'd27 : diff[4:0]; //if difference greater than 27, counter capped at 27
                            if(a_zero || b_zero) begin
                                if(a_zero && b_zero) begin
                                    sign <= a[31]& b[31];
                                    exp <= {1'b0,big[30:23]};
                                end
                                state <= DONE;
                            end
                        end
                    end
                end
                
                ALIGN: begin
                    if(|cnt) begin // if count != 0
                        frac_sml <= { 1'b0, frac_sml[26:2], frac_sml[1] | frac_sml[0]}; //shifts 1 bit per cycle
                        cnt <= cnt - 5'd1; 
                    end else state <= EXEC;
                end
                
                EXEC: begin //performs sub or addition
                    frac_big <= eff_sub ? (frac_big - {1'b0, frac_sml}) : (frac_big + {1'b0, frac_sml});
                    state <= NORMAL;
                end
                
                NORMAL: begin
                    if (~|frac_big) begin // if cancellation then result is 0; directly goes to done
                        exp <= 9'd0;
                        sign <= sign & ~eff_sub;
                        state <= DONE;

                    end else if (frac_big[27]) begin //detects if carry and shifts, increments exponent
                        frac_big <= {1'b0,frac_big[27:2],frac_big[1] | frac_big[0]};
                        exp   <= exp + 9'd1;
                        state <= ROUND;

                    end else if (!frac_big[26] && |exp[8:1]) begin //left normalization - if hidden bit is 0 and exp is nonzero then shift
                        frac_big <= {frac_big[26:0], 1'b0};
                        exp <= exp - 9'd1;

                    end else begin
                        state <= ROUND;
                    end
                end
                
                ROUND: begin
                    if (ovf) begin //if exp >= 255 then overflow, result = infinity
                        exp <= 9'd255;
                        frac_big[25:3] <= 23'd0;
                    end else begin
                        exp <= e_round;
                        frac_big[25:3] <= frac_round[24] ? 23'd0 : frac_round[22:0];
                    end
                    state <= DONE;
                end
                
                DONE: state <= IDLE;

                default: state <= IDLE;
            endcase 
        end    
     end
     
    assign result = {sign, exp[7:0], frac_big[25:3]}; //final result
    assign valid_out = (state == DONE);    
                   
endmodule

