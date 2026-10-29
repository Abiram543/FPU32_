`default_nettype none
module fpu_core(
    input wire clk,
    input wire rst_n,
    input wire start, // Will assert once the valid operands and opcode has arrived
    input wire op, // 0=ADD, 1=SUB
    input wire [31:0] op_a,
    input wire [31:0] op_b,
    output reg [31:0] result,
    output reg done, // Done flag
    output reg err // Error flag
    );

    localparam S_IDLE = 0;
    localparam S_SPECIAL = 1;
    localparam S_OPERATION = 2;
    
    // Swap/Sign handling
    wire sign_eff = op_b[31] ^ op; // SUB = ADD with B's sign flipped 
    wire a_bigger = (op_a[30:0] >= op_b[30:0]); // Comparison of magnitudes
    
    wire sign_big =  a_bigger ? op_a[31] : sign_eff; // Bigger number sign is retained in the result
    wire sign_small = a_bigger ? sign_eff : op_a[31];
    wire [7:0] exp_big = a_bigger ? op_a[30:23] : op_b[30:23];
    wire [7:0] exp_small = a_bigger ? op_b[30:23] : op_a[30:23];
    wire [22:0] mant_big = a_bigger ? op_a[22:0] : op_b[22:0];
    wire [22:0] mant_small = a_bigger ? op_b[22:0] : op_a[22:0];
    
    wire sub_operation = sign_big ^ sign_small; // 1 = Subtract magnitudes       
    
    // Updated mantissa with the hidden bit used by S_OPERATION state
    // Hidden bit is always 1 as all the numbers are normalized during S_OPERATION
    wire [23:0] m_big = {1'b1, mant_big};
    wire [23:0] m_small = {1'b1, mant_small};
                   
    // Align : Shift the smaller mantissa right by the exponent difference
    // If exp diff >= 24 then the smaller mantissa becomes zero. Clear the whole mantissa.
    wire [7:0] exp_diff = exp_big - exp_small;
    wire [23:0] m_small_aligned = m_small >> exp_diff;
    
    wire [24:0] mant_sum = sub_operation ? {1'b0, m_big} - {1'b0, m_small_aligned} : 
                                            {1'b0, m_big} + {1'b0, m_small_aligned};
    
    // Leading zeroes count for normalization 
    function [4:0] leading_zeroes; // returns a 5 bit number
        input [23:0] x; 
        integer i;
        begin
        leading_zeroes = 5'd23;
        for(i = 0; i < 24; i = i + 1) 
            if(x[i]) leading_zeroes = 23 - i;
        end
    endfunction
    
    wire [4:0] lz = leading_zeroes(mant_sum[23:0]);
    wire [23:0] mant_norm = mant_sum[23:0] << lz;
     
    reg [1:0] ps,ns;
    // Present state logic
    always@(posedge clk or negedge rst_n) begin
        if(!rst_n) ps <= S_IDLE;
        else ps <= ns;
    end
    
    // Next state logic
    always@(*) begin
        ns = ps;
        result = 32'd0;
        done = 1'b0;
        err = 1'b0;
        
        case(ps) 
            S_IDLE: begin
                if(start) ns = S_SPECIAL;
            end
            
            S_SPECIAL: begin
                // Infinity and NaN cases
                if((exp_big == 8'b11111111) || (exp_small == 8'b11111111)) begin 
                    err = 1'b1;
                    done = 1'b1;
                    ns = S_IDLE;
                end
                // Operand b = 0 special case
                else if((exp_small == 8'd0) && (mant_small == 22'd0)) begin
                    result = {sign_big, exp_big, mant_big};
                    done = 1'b1;
                    ns = S_IDLE;
                end
                // Denormalized numbers special case
                else if(((exp_big == 8'd0) && (mant_big != 22'd0)) || ((exp_small == 8'd0) && (mant_small != 22'd0))) begin
                    err = 1'b1;
                    done = 1'b1;
                    ns = S_IDLE;
                end
                
                else begin
                    ns = S_OPERATION;
                end
            end
            
            S_OPERATION: begin
                done = 1'b1;
                ns = S_IDLE;
                if(mant_sum[24]) begin // When carry bit is logic high, shift right 1 and add 1 to exponent  
                    if(exp_big == 8'd254) 
                        result = {sign_big, 8'hFF, 23'd0}; // Overflow to infinity
                    else 
                        result = {sign_big, exp_big + 8'b1, mant_sum[23:1]}; 
                end
                // If the magnitude of op_a and op_b are same and the operation is SUB
                // Then the result should be +0
                else if(mant_sum[23:0] == 24'd0)
                    result = 32'd0;
                // Underflow - When during normalization exp value falls outside the range of exp    
                else if({3'b000, lz} >= exp_big) 
                    result = {sign_big, 31'd0};
                // Left normalize
                else 
                    result = {sign_big, (exp_big - {3'b000, lz}), mant_norm[22:0]};
            end
            
            default: ns = S_IDLE;
        endcase
    end
endmodule
