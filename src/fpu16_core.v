module FPU16_Core(
    input  wire        clk,
    input  wire        rstn,
    input  wire [15:0] a,
    input  wire [15:0] b,
    input  wire        op,          // 1: Add, 0: Subtract
    input  wire        start,

    output wire [15:0] result,
    output wire        valid_out,
    output reg         error
);

    localparam IDLE = 3'd0, ALIGN  = 3'd1, EXEC   = 3'd2, NORMAL = 3'd3, ROUND  = 3'd4, DONE   = 3'd5;

    reg [2:0] state;
    reg sign, eff_sub;

    reg [14:0] frac_big;
    reg [13:0] frac_sml;

    reg [5:0] exp;
    reg [3:0] cnt;

    // Operand ordering
    wire [15:0] b_eff;
    wire [15:0] big, sml;
    wire swap;

    wire big_hid, sml_hid;
    wire [4:0] big_e, sml_e, diff;

    wire in_err;

    // Shared ALU
    wire round_up;
    wire [14:0] alu_b;
    wire alu_c;
    wire [14:0] alu;

    // Rounding and exponent logic
    wire [11:0] frac_round;
    wire exp_dec;
    wire [5:0] exp_upd;
    wire [5:0] e_round;
    wire ovf;

    assign b_eff = {b[15] ^ ~op, b[14:0]};

    assign swap = (a[14:0] < b_eff[14:0]);

    assign big = swap ? b_eff : a;
    assign sml = swap ? a : b_eff;

    assign big_hid = |big[14:10];
    assign sml_hid = |sml[14:10];

    assign big_e = big[14:10] | {4'b0, ~big_hid};
    assign sml_e = sml[14:10] | {4'b0, ~sml_hid};

    assign diff = big_e - sml_e;

    // Inf / NaN detection
    assign in_err = (&a[14:10]) | (&b[14:10]);


    assign round_up = frac_big[2] & (frac_big[1] | frac_big[0] | frac_big[3]);

    assign alu_b = (state == ROUND) ? {11'b0, round_up, 3'b0} : ({1'b0, frac_sml} ^ {15{eff_sub}});

    assign alu_c = (state != ROUND) & eff_sub;

    assign alu = frac_big + alu_b + {14'b0, alu_c};

    assign frac_round = alu[14:3];


    assign exp_dec = (state == NORMAL) & ~frac_big[14];

    assign exp_upd = exp + {{5{exp_dec}}, 1'b1};

    assign e_round = frac_round[11] ? exp_upd : (frac_round[10] ? exp : 6'd0);

    assign ovf = e_round[5] | (&e_round[4:0]);
    
    /*wire a_zero, b_zero;

    assign a_zero = ~|a[14:0];
    assign b_zero = ~|b[14:0];*/

    always @(posedge clk or negedge rstn) begin

        if (!rstn) begin
            state <= IDLE;
            sign    <= 1'b0;
            eff_sub <= 1'b0;
            error   <= 1'b0;
            frac_big <= 15'd0;
            frac_sml <= 14'd0;
            exp <= 6'd0;
            cnt <= 4'd0;
        end else begin

            case (state)
                IDLE: begin
                    if (start) begin
                        error <= in_err;
                        sign <= big[15];
                        eff_sub <= big[15] ^ sml[15];
                        exp <= {1'b0, big_e};
                        frac_big <= {1'b0,big_hid,big[9:0],3'b000};
                        frac_sml <= {sml_hid,sml[9:0],3'b000};

                        cnt <= (diff > 5'd13) ?  4'd13 : diff[3:0];

                        if (in_err) state <= DONE;
                        /*else if (a_zero || b_zero) begin
                            exp <= 6'd0;
                            frac_big <= 15'd0;
                            state <= DONE;
                            if (a_zero && b_zero)
                                sign <= a[15] & b_eff[15];
                            else
                                sign <= a_zero ? b_eff[15] : a[15];
                        
                        end */ else state <= ALIGN;
                    end
                end
                ALIGN: begin
                    if (|cnt) begin
                        frac_sml <= {1'b0,frac_sml[13:2],frac_sml[1] | frac_sml[0]};
                        cnt <= cnt - 4'd1;
                    end
                    else state <= EXEC;
                end

                EXEC: begin
                    frac_big <= alu;
                    state <= NORMAL;
                end


                NORMAL: begin
                    // Exact cancellation
                    if (~|frac_big) begin
                        exp <= 6'd0;
                        sign <= sign & ~eff_sub;
                        state <= DONE;
                    end
                    // Right normalization after carry
                    else if (frac_big[14]) begin
                        frac_big <= {1'b0, frac_big[14:2],frac_big[1] | frac_big[0]};
                        exp <= exp_upd;
                        state <= ROUND;
                    end
                    // Left normalization
                    else if (!frac_big[13] && |exp[5:1]) begin
                        frac_big <= {frac_big[13:0],1'b0};
                        exp <= exp_upd;
                    end

                    else state <= ROUND;
                end

                ROUND: begin
                    if (ovf) begin // if ovf then infinity
                        exp <= 6'd31;
                        frac_big[12:3] <= 10'd0;
                    end
                    else begin
                        exp <= e_round;
                        frac_big[12:3] <= frac_round[9:0];
                    end
                    state <= DONE;
                end

                DONE: state <= IDLE;

                default: state <= IDLE;
            endcase
        end
    end

    assign result = {sign,exp[4:0],frac_big[12:3]};
    assign valid_out = (state == DONE);

endmodule
