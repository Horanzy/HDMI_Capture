// -----------------------------------------------------------------------------
// ft601_master.v -- 128-bit video FIFO to FT601 32-bit 245-sync-FIFO writer
// Four low-first bus words are emitted per FIFO word.  A pop issued while bus
// phase 2 is emitted arrives while phase 3 is emitted, allowing a bubble-free
// direct reload.  s0 parks an arrival across a TXE stall.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module ft601_master (
    input  wire         clk_usb,
    input  wire         rstn,
    output reg          fifo_rd_en,
    input  wire         fifo_rd_valid,
    input  wire [127:0] fifo_rd_data,
    input  wire         fifo_rd_empty,
    input  wire         txe_n_pad,
    input  wire         cmd_inhibit,
    output reg  [31:0]  o_data,
    output reg  [3:0]   o_be,
    output reg          o_wr_n
);

    reg txe_n_q;
    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) txe_n_q <= 1'b1;
        else       txe_n_q <= txe_n_pad;
    end

    reg         spl_v;
    reg [1:0]   spl_sel;
    reg [127:0] spl_d;
    reg         s0_v;
    reg [127:0] s0_d;

    wire         s1_v = fifo_rd_valid;
    wire [127:0] s1_d = fifo_rd_data;
    wire out_fire = !txe_n_q && !cmd_inhibit && spl_v;
    wire last_emit = out_fire && (spl_sel == 2'd3);
    wire preload_emit = out_fire && (spl_sel == 2'd2);

    reg [31:0] out_word;
    always @(*) begin
        case (spl_sel)
            2'd0: out_word = spl_d[31:0];
            2'd1: out_word = spl_d[63:32];
            2'd2: out_word = spl_d[95:64];
            default: out_word = spl_d[127:96];
        endcase
    end

    wire direct_take = (last_emit && !s0_v) || (!spl_v && s1_v);
    wire s0_capture = s1_v && !direct_take;

    always @(*) begin
        fifo_rd_en = !fifo_rd_empty && !s0_v && (preload_emit || !spl_v);
    end

    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            spl_v <= 1'b0;
            spl_sel <= 2'd0;
            spl_d <= 128'h0;
            s0_v <= 1'b0;
            s0_d <= 128'h0;
        end else begin
            if (last_emit) begin
                spl_sel <= 2'd0;
                if (s0_v) begin
                    spl_d <= s0_d;
                    spl_v <= 1'b1;
                end else if (s1_v) begin
                    spl_d <= s1_d;
                    spl_v <= 1'b1;
                end else begin
                    spl_v <= 1'b0;
                end
            end else if (out_fire) begin
                spl_sel <= spl_sel + 2'd1;
            end else if (!spl_v && s1_v) begin
                spl_d <= s1_d;
                spl_v <= 1'b1;
                spl_sel <= 2'd0;
            end

            if (last_emit && s0_v)
                s0_v <= 1'b0;
            if (s0_capture) begin
                s0_v <= 1'b1;
                s0_d <= s1_d;
            end
        end
    end

    // Experiment: launch on posedge and use output IODELAY at the top level.
    // This gives a full-clock setup budget; the programmed output delay is
    // intended to satisfy FT601's 4.8 ns hold requirement.
    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            o_data <= 32'h0;
            o_be <= 4'hF;
            o_wr_n <= 1'b1;
        end else begin
            o_data <= out_word;
            o_be <= 4'hF;
            o_wr_n <= !out_fire;
        end
    end

endmodule
