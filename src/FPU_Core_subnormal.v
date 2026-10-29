// FPU_Core fix B: error output + no zero shortcut + area optimisations (shared add/sub, datapath flops without reset)
//   error : 1-cycle pulse coincident with valid_out when a or b is Inf/NaN (exp==FF). result is don't-care then.
//   zero  : not special-cased; a zero operand has frac_sml = 0, and the NORMAL cancellation branch yields the RNE zero sign.
module FPU_Core(
    input  wire clk, input wire rstn,
    input  wire [31:0] a, input wire [31:0] b,
    input  wire op,                // 1: a+b, 0: a-b
    input  wire start,             // 1-cycle pulse (wrapper guarantees)
    output wire [31:0] result,     // meaningless before first valid_out and when error=1
    output wire valid_out,
    output reg  error);
    localparam IDLE = 3'd0, ALIGN = 3'd1, EXEC = 3'd2, NORMAL = 3'd3, ROUND = 3'd4, DONE = 3'd5;
    reg [2:0]  state;
    reg        sign, eff_sub;
    reg [27:0] frac_big;
    reg [26:0] frac_sml;
    reg [8:0]  exp;
    reg [4:0]  cnt;

    wire [31:0] b_eff = {b[31] ^ ~op, b[30:0]};
    wire        swap  = (a[30:0] < b_eff[30:0]);
    wire [31:0] big   = swap ? b_eff : a;
    wire [31:0] sml   = swap ? a : b_eff;
    wire big_hid = |big[30:23], sml_hid = |sml[30:23];
    wire [7:0] big_e = big[30:23] | {7'b0, ~big_hid};
    wire [7:0] sml_e = sml[30:23] | {7'b0, ~sml_hid};
    wire [7:0] diff  = big_e - sml_e;
    wire invalid_input = (&a[30:23]) | (&b[30:23]);          // Inf or NaN

    wire        round_up   = frac_big[2] & (frac_big[1] | frac_big[0] | frac_big[3]);
    wire [24:0] frac_round = {1'b0, frac_big[26:3]} + round_up;
    wire [8:0]  e_round    = frac_round[24] ? (exp + 9'd1) : (frac_round[23] ? exp : 9'd0);
    wire        ovf        = e_round[8] | (&e_round[7:0]);
    wire [27:0] sum        = frac_big + ({1'b0, frac_sml} ^ {28{eff_sub}}) + {27'b0, eff_sub};

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin state <= IDLE; error <= 1'b0; end
        else begin
            error <= 1'b0;
            case (state)
                IDLE:   if (start) begin if (invalid_input) begin error <= 1'b1; state <= DONE; end else state <= ALIGN; end
                ALIGN:  if (!(|cnt)) state <= EXEC;
                EXEC:   state <= NORMAL;
                NORMAL: if      (~|frac_big)                    state <= DONE;
                        else if (frac_big[27])                  state <= ROUND;
                        else if (!frac_big[26] && |exp[8:1])    state <= NORMAL;
                        else                                    state <= ROUND;
                ROUND:  state <= DONE;
                default: state <= IDLE;
            endcase
        end
    end
    always @(posedge clk) begin
        case (state)
            IDLE: if (start) begin
                sign <= big[31]; eff_sub <= big[31] ^ sml[31]; exp <= {1'b0, big_e};
                frac_big <= {1'b0, big_hid, big[22:0], 3'b000};
                frac_sml <= {sml_hid, sml[22:0], 3'b000};
                cnt <= (diff > 8'd27) ? 5'd27 : diff[4:0];
            end
            ALIGN: if (|cnt) begin
                frac_sml <= {1'b0, frac_sml[26:2], frac_sml[1] | frac_sml[0]}; cnt <= cnt - 5'd1;
            end
            EXEC: frac_big <= sum;
            NORMAL: begin
                if (~|frac_big) begin exp <= 9'd0; sign <= sign & ~eff_sub; end
                else if (frac_big[27]) begin frac_big <= {1'b0, frac_big[27:2], frac_big[1] | frac_big[0]}; exp <= exp + 9'd1; end
                else if (!frac_big[26] && |exp[8:1]) begin frac_big <= {frac_big[26:0], 1'b0}; exp <= exp - 9'd1; end
            end
            ROUND: begin
                if (ovf) begin exp <= 9'd255; frac_big[25:3] <= 23'd0; end
                else begin exp <= e_round; frac_big[25:3] <= frac_round[22:0]; end   // carry-out => already 0
            end
            default: ;
        endcase
    end
    assign result    = {sign, exp[7:0], frac_big[25:3]};
    assign valid_out = (state == DONE);
endmodule
