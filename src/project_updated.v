/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_project_fpu (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] ui_in,    // [0]-write_en, [1]-read_en, [2]-start, [3]-opcode, [4]-clear, [5:7]-reserved 
    input  wire [7:0] uio_in,   // input when ui_in[0]==1
    output wire [7:0] uio_out,  // output when ui_in[1]==1
    output wire [7:0] uio_oe,   // Enable for inout 0--> Input, 1--> Output
    input  wire       ena,
    output wire [7:0] uo_out
);

// Edge detect — single-stage pipeline (TT mux is same clock domain, no CDC needed)
reg [1:0] wr_s, rd_s, st_s;
reg       clr_s;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_s <= 0; rd_s <= 0; st_s <= 0; clr_s <= 0;
    end else begin
        wr_s  <= {wr_s[0],  ui_in[0]};
        rd_s  <= {rd_s[0],  ui_in[1]};
        st_s  <= {st_s[0],  ui_in[2]};
        clr_s <= ui_in[4];
    end
end

wire wr_pulse    =  wr_s[0] & ~wr_s[1];
wire start_pulse =  st_s[0] & ~st_s[1];
wire rd_fall     = ~rd_s[0] &  rd_s[1];
wire rd_level    =  rd_s[0];
wire clr         =  clr_s;

// Registers — a_reg/b_reg intentionally have NO reset to save mux cells
reg [31:0] a_reg, b_reg;
reg [2:0]  wr_ptr;
reg [1:0]  rd_ptr;
reg        op_reg;
reg        busy, done_l, err_l, ready, core_start;

// FPU core (iterative, area-optimised — no barrel shifters)
wire [31:0] core_result;
wire        core_done, core_err;

FPU_Core u_fpu (
    .clk       (clk),
    .rstn      (rst_n),
    .a         (a_reg),
    .b         (b_reg),
    .op        (op_reg),
    .start     (core_start),
    .result    (core_result),
    .valid_out (core_done),
    .error     (core_err)
);

// Data register writes — separate always block, no reset, explicit case decode
always @(posedge clk) begin
    if (!busy && !clr && wr_pulse && !(start_pulse && ready)) begin
        case (wr_ptr)
            3'd0: a_reg[ 7: 0] <= uio_in;
            3'd1: a_reg[15: 8] <= uio_in;
            3'd2: a_reg[23:16] <= uio_in;
            3'd3: a_reg[31:24] <= uio_in;
            3'd4: b_reg[ 7: 0] <= uio_in;
            3'd5: b_reg[15: 8] <= uio_in;
            3'd6: b_reg[23:16] <= uio_in;
            3'd7: b_reg[31:24] <= uio_in;
        endcase
    end
end

// Control Logic
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_ptr <= 0; rd_ptr <= 0; op_reg <= 0;
        busy <= 0; done_l <= 0; err_l <= 0; ready <= 0; core_start <= 0;
    end else begin
        core_start <= 1'b0;  // default: 1-cycle pulse
        if (busy) begin
            // LOCKED: all user inputs ignored
            // Only the core can release.
            if (!core_start && (core_done || core_err)) begin
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
                wr_ptr <= wr_ptr + 1'b1;
                ready  <= (wr_ptr == 3'd7);
            end
            else if (rd_fall) begin
                rd_ptr <= rd_ptr + 1'b1;
            end
        end
    end
end

// Outputs — read core_result directly (no res_reg copy needed;
// FPU_Core holds result stable in its registers until the next start pulse)
assign uio_oe  = (rd_level && !busy) ? 8'hFF : 8'h00;

reg [7:0] rd_byte;
always @(*) begin
    case (rd_ptr)
        2'd0: rd_byte = core_result[ 7: 0];
        2'd1: rd_byte = core_result[15: 8];
        2'd2: rd_byte = core_result[23:16];
        2'd3: rd_byte = core_result[31:24];
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

wire _unused = &{ena, 1'b0};

endmodule
