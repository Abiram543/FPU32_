`default_nettype none

module tt_um_project_fpu (
    input  wire [7:0] ui_in,    // write data byte
    output wire [7:0] uo_out,   // result byte (always driven)
    input  wire [7:0] uio_in,   // [0]-WR [1]-RD [2]-START [3]-OP [4]-CLR (inputs)
    output wire [7:0] uio_out,  // [5]-BUSY [6]-DONE [7]-ERR (outputs)
    output wire [7:0] uio_oe,   // fixed direction: 1 = output
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // Control inputs on the bidirectional pins
    wire wr_in    = uio_in[0];
    wire rd_in    = uio_in[1];
    wire start_in = uio_in[2];
    wire op_in    = uio_in[3];
    wire clr_in   = uio_in[4];

    // ---------- Synchronizers + edge detect ----------
    reg [2:0] wr_s, rd_s, st_s;
    reg [1:0] clr_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_s <= 0; rd_s <= 0; st_s <= 0; clr_s <= 0;
        end else begin
            wr_s  <= {wr_s[1:0],  wr_in};
            rd_s  <= {rd_s[1:0],  rd_in};
            st_s  <= {st_s[1:0],  start_in};
            clr_s <= {clr_s[0],   clr_in};
        end
    end

    wire wr_pulse    = wr_s[1] & ~wr_s[2];
    wire start_pulse = st_s[1] & ~st_s[2];
    wire rd_pulse    = rd_s[1] & ~rd_s[2];   // rising edge advances result byte
    wire clr         = clr_s[1];

    // ---------- Registers ----------
    reg [31:0] a_reg, b_reg, res_reg;
    reg [2:0]  wr_ptr;
    reg [1:0]  rd_ptr;
    reg        op_reg;
    reg        busy, done_l, err_l, ready, core_start;

    // ---------- FPU core ----------
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

    // ---------- Control ----------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_reg <= 0; b_reg <= 0; res_reg <= 0;
            wr_ptr <= 0; rd_ptr <= 0; op_reg <= 0;
            busy <= 0; done_l <= 0; err_l <= 0; ready <= 0; core_start <= 0;
        end else begin
            core_start <= 1'b0;                  // default: 1-cycle pulse
            if (busy) begin
                // LOCKED: all user inputs ignored, only the core releases
                if (!core_start && (core_done || core_err)) begin
                    res_reg <= core_result;
                    busy    <= 1'b0;
                    done_l  <= core_done;
                    err_l   <= core_err;
                end
            end else begin
                // IDLE: one action per cycle, fixed priority
                if (clr) begin
                    wr_ptr <= 0;
                    rd_ptr <= 0;
                    ready  <= 0;
                end
                else if (start_pulse && ready) begin
                    op_reg     <= op_in;
                    core_start <= 1'b1;
                    busy       <= 1'b1;
                    done_l     <= 1'b0;
                    err_l      <= 1'b0;
                    rd_ptr     <= 0;
                end
                else if (wr_pulse) begin
                    if (!wr_ptr[2]) a_reg[wr_ptr[1:0]*8 +: 8] <= ui_in;
                    else            b_reg[wr_ptr[1:0]*8 +: 8] <= ui_in;
                    wr_ptr <= wr_ptr + 1'b1;
                    ready  <= (wr_ptr == 3'd7);
                end
                else if (rd_pulse) begin
                    rd_ptr <= rd_ptr + 1'b1;     // wraps 3 -> 0
                end
            end
        end
    end

    // ---------- Outputs ----------
    reg [7:0] rd_byte;
    always @(*) begin
        case (rd_ptr)
            2'd0: rd_byte = res_reg[7:0];
            2'd1: rd_byte = res_reg[15:8];
            2'd2: rd_byte = res_reg[23:16];
            2'd3: rd_byte = res_reg[31:24];
            default: rd_byte = 8'd0;
        endcase
    end
    assign uo_out = rd_byte;

    // uio[7:5] are outputs (status), uio[4:0] are inputs (control)
    assign uio_out = {err_l, done_l, busy, 5'b00000};
    assign uio_oe  = 8'b1110_0000;

    wire _unused = &{ena, uio_in[7:5], 1'b0};

endmodule
