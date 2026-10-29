/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module project (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] ui_in,    // [0]-write_en, [1]-read_en, [2]-start, [3]-opcode, [4]-clear, [5:7]-reserved 
    input  wire [7:0] uio_in,   // input when ui_in[0]==1
    output wire [7:0] uio_out,  // output when ui_in[1]==1
    output wire [7:0] uio_oe,   // Enable for inout 0--> Input, 1--> Output
    output wire [7:0] uo_out
);
// Synchronizers + edge detect
reg [2:0] wr_s, rd_s, st_s;
reg [1:0] clr_s;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_s <= 0; rd_s <= 0; st_s <= 0; clr_s <= 0;
    end else begin
        wr_s  <= {wr_s[1:0],  ui_in[0]};
        rd_s  <= {rd_s[1:0],  ui_in[1]};
        st_s  <= {st_s[1:0],  ui_in[2]};
        clr_s <= {clr_s[0],   ui_in[4]};
    end
end

wire wr_pulse    =  wr_s[1] & ~wr_s[2];
wire start_pulse =  st_s[1] & ~st_s[2];
wire rd_fall     = ~rd_s[1] &  rd_s[2];
wire rd_level    =  rd_s[1];
wire clr         =  clr_s[1];

// Registers
reg [31:0] a_reg, b_reg, res_reg;
reg [2:0]  wr_ptr;
reg [1:0]  rd_ptr;
reg        op_reg;
reg        busy, done_l, err_l, ready, core_start;

// FPU core 
wire [31:0] core_result;
wire        core_done, core_err;
fpu_core u_fpu (
    .clk    (clk),
    .rst_n  (rst_n),
    .start  (core_start),
    .op     (op_reg),
    .op_a   (a_reg),
    .op_b   (b_reg),
    .result (core_result),
    .done   (core_done),
    .err    (core_err)
);

// Control Logic
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        a_reg <= 0; b_reg <= 0; res_reg <= 0;
        wr_ptr <= 0; rd_ptr <= 0; op_reg <= 0;
        busy <= 0; done_l <= 0; err_l <= 0; ready <= 0; core_start <= 0;
    end else begin
        core_start <= 1'b0;  // default: 1-cycle pulse
        if (busy) begin
            // LOCKED: all user inputs ignored
            // Only the core can release.
            if (!core_start && (core_done || core_err)) begin
                res_reg <= core_result;
                busy    <= 1'b0;
                done_l  <= core_done;
                err_l   <= core_err;
            end
        end else begin
            // IDLE
            if (clr) begin
                wr_ptr <= 0;
                rd_ptr <= 0;
                ready  <= 0;
            end
            else if (start_pulse && ready) begin
                op_reg     <= ui_in[3];
                core_start <= 1'b1;
                busy       <= 1'b1;
                done_l     <= 1'b0;
                err_l      <= 1'b0;
                rd_ptr     <= 0;
            end
            else if (wr_pulse) begin
                if (!wr_ptr[2]) a_reg[wr_ptr[1:0]*8 +: 8] <= uio_in;
                else            b_reg[wr_ptr[1:0]*8 +: 8] <= uio_in;
                wr_ptr <= wr_ptr + 1'b1;
                ready  <= (wr_ptr == 3'd7);
            end
            else if (rd_fall) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end
end
// Outputs
assign uio_oe  = (rd_level && !busy) ? 8'hFF : 8'h00;
reg [7:0] rd_byte;
always @(*) begin
    case (rd_ptr)
        2'd0: rd_byte = res_reg[7:0];
        2'd1: rd_byte = res_reg[15:8];
        2'd2: rd_byte = res_reg[23:16];
        2'd3: rd_byte = res_reg[31:24];
        default: rd_byte = 0;
    endcase
end
assign uio_out = rd_byte;
assign uo_out[0]   = busy;
assign uo_out[1]   = done_l;
assign uo_out[2]   = err_l;
assign uo_out[3]   = ready;
assign uo_out[6:4] = wr_ptr;
assign uo_out[7]   = 1'b0;
endmodule
