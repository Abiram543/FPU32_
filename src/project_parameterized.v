// ============================================================================
// Parameterized Tiny Tapeout Wrapper for IEEE 754 FPU (Combinational Core)
// ============================================================================
// Adapts automatically to any FP width set by EXP_W / MAN_W.
// Default: FP16 (EXP_W=5, MAN_W=10) — fits comfortably in a 1x1 TT tile.
//
// Byte-loading protocol (little-endian, 8-bit uio port):
//   FP8:  2 writes (1 byte × 2 operands), 1 byte read
//   FP16: 4 writes (2 bytes × 2 operands), 2 byte reads
//   FP32: 8 writes (4 bytes × 2 operands), 4 byte reads
//   FP64: 16 writes (8 bytes × 2 operands), 8 byte reads
//
// Control pins (ui_in):
//   [0] write_en   — pulse to write one byte from uio_in
//   [1] read_en    — hold high to drive uio_out; falling edge advances byte
//   [2] start      — pulse to begin computation (after all bytes loaded)
//   [3] opcode     — sampled on start: 1=Add, 0=Subtract
//   [4] clear      — reset byte pointers and ready flag
//
// Status pins (uo_out):
//   [0] busy       — 1 while FPU is computing
//   [1] done       — 1 when result is ready
//   [2] error      — 1 if input was Inf/NaN
//   [3] ready      — 1 after all operand bytes have been loaded
//   [7:4] wr_ptr   — current write pointer (lower 4 bits)
// ============================================================================
/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_project_fpu_parameterized #(
    parameter EXP_W = 5,            // exponent width  (default: FP16)
    parameter MAN_W = 10            // mantissa width   (default: FP16)
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] ui_in,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    output wire [7:0] uo_out
);

// ====================================================================
// Derived constants
// ====================================================================
localparam WIDTH        = 1 + EXP_W + MAN_W;               // total FP width
localparam BYTES_PER_OP = WIDTH / 8;                        // bytes per operand
localparam TOTAL_BYTES  = 2 * BYTES_PER_OP;                 // a + b
localparam WR_PTR_W     = $clog2(TOTAL_BYTES)  > 0
                        ? $clog2(TOTAL_BYTES)  : 1;         // write-pointer width
localparam RD_PTR_W     = $clog2(BYTES_PER_OP) > 0
                        ? $clog2(BYTES_PER_OP) : 1;         // read-pointer width

// ====================================================================
// Edge detection — single-stage pipeline (TT mux, same clock domain)
// ====================================================================
reg [1:0] wr_s, rd_s, st_s;
reg       clr_s;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_s <= 2'b0;  rd_s <= 2'b0;  st_s <= 2'b0;  clr_s <= 1'b0;
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

// ====================================================================
// Operand registers — NO reset (saves mux cells)
// ====================================================================
reg [WIDTH-1:0]       a_reg, b_reg;
reg [WR_PTR_W-1:0]   wr_ptr;
reg [RD_PTR_W-1:0]   rd_ptr;
reg                   op_reg;
reg                   busy, done_l, err_l, ready, core_start;

// ====================================================================
// FPU Core instantiation (parameterized, combinational approach)
// ====================================================================
wire [WIDTH-1:0] core_result;
wire             core_done, core_err;

fpu_core #(
    .EXP_W (EXP_W),
    .MAN_W (MAN_W)
) u_fpu (
    .clk       (clk),
    .rst_n     (rst_n),
    .op_a      (a_reg),
    .op_b      (b_reg),
    .op        (op_reg),
    .start     (core_start),
    .result    (core_result),
    .done      (core_done),
    .err       (core_err)
);

// ====================================================================
// Byte-write logic — uses synthesisable variable part-select
// Each iteration of the loop is unrolled at elaboration (constant index).
// ====================================================================
integer wi;
always @(posedge clk) begin
    if (!busy && !clr && wr_pulse && !(start_pulse && ready)) begin
        for (wi = 0; wi < BYTES_PER_OP; wi = wi + 1) begin
            if (wr_ptr == wi[WR_PTR_W-1:0])
                a_reg[wi*8 +: 8] <= uio_in;
            if (wr_ptr == (BYTES_PER_OP + wi))
                b_reg[wi*8 +: 8] <= uio_in;
        end
    end
end

// ====================================================================
// Control FSM
// ====================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_ptr     <= {WR_PTR_W{1'b0}};
        rd_ptr     <= {RD_PTR_W{1'b0}};
        op_reg     <= 1'b0;
        busy       <= 1'b0;
        done_l     <= 1'b0;
        err_l      <= 1'b0;
        ready      <= 1'b0;
        core_start <= 1'b0;
    end else begin
        core_start <= 1'b0;                          // default: 1-cycle pulse

        if (busy) begin
            // LOCKED — ignore user inputs while core computes
            if (!core_start && (core_done || core_err)) begin
                busy   <= 1'b0;
                done_l <= core_done;
                err_l  <= core_err;
            end
        end else begin
            // IDLE
            if (clr) begin
                wr_ptr <= {WR_PTR_W{1'b0}};
                rd_ptr <= {RD_PTR_W{1'b0}};
                ready  <= 1'b0;
            end
            else if (start_pulse && ready) begin
                op_reg     <= ui_in[3];
                core_start <= 1'b1;
                busy       <= 1'b1;
                done_l     <= 1'b0;
                err_l      <= 1'b0;
                rd_ptr     <= {RD_PTR_W{1'b0}};
            end
            else if (wr_pulse) begin
                wr_ptr <= wr_ptr + {{(WR_PTR_W-1){1'b0}}, 1'b1};
                ready  <= (wr_ptr == TOTAL_BYTES - 1);  // ready after last byte
            end
            else if (rd_fall) begin
                rd_ptr <= rd_ptr + {{(RD_PTR_W-1){1'b0}}, 1'b1};
            end
        end
    end
end

// ====================================================================
// Byte-read mux (safe for all widths, including FP8 where only 1 byte)
// ====================================================================
reg [7:0] rd_byte;
integer ri;
always @(*) begin
    rd_byte = core_result[7:0];                       // default: lowest byte
    for (ri = 1; ri < BYTES_PER_OP; ri = ri + 1) begin
        if (rd_ptr == ri[RD_PTR_W-1:0])
            rd_byte = core_result[ri*8 +: 8];
    end
end

// ====================================================================
// Output assignments
// ====================================================================
assign uio_oe  = (rd_level && !busy) ? 8'hFF : 8'h00;
assign uio_out = rd_byte;

assign uo_out[0] = busy;
assign uo_out[1] = done_l;
assign uo_out[2] = err_l;
assign uo_out[3] = ready;

// Show write-pointer on uo_out[7:4] (up to 4 bits)
generate
    if (WR_PTR_W >= 4)
        assign uo_out[7:4] = wr_ptr[3:0];
    else
        assign uo_out[7:4] = {{(4-WR_PTR_W){1'b0}}, wr_ptr};
endgenerate

// Tie off unused inputs
wire _unused = &{ena, ui_in[7:5], 1'b0};

endmodule
