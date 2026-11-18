// -----------------------------------------------------------------------------
// async_fifo_dc.v -- portable dual-clock FIFO (inferred BSRAM, gray pointers)
//
// Function
//   Independent-clock FIFO handing 128-bit raw line-packet words from the
//   video domain to the FT601/USB domain.  The writer produces at most one
//   word per PCLK; ft601_master splits each word into four 32-bit bus words.
//   True dual-port inferred BSRAM + Cummings-style gray-pointer CDC.
//
// Interface / handshake
//   Write: wr_en && !wr_full commits one word in wr_clk.
//   Read : rd_en && !rd_empty pops one word; rd_valid + rd_data are
//          presented ONE rd_clk LATER (registered BSRAM read).  The consumer
//          must keep a prefetch and only assert rd_en when it can accept.
//
// Parameters used by the top: DW=128, DEPTH_LOG2=10 -> 1024 x 128 b = 16 KiB.
//
// Reset: wr_rstn / rd_rstn async assert, sync release; hold both >= 3 cycles
// of each clock.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module async_fifo_dc #(
    parameter DW        = 64,
    parameter DEPTH_LOG2 = 11
) (
    // write port (video domain)
    input  wire              wr_clk,
    input  wire              wr_rstn,
    input  wire              wr_en,
    input  wire [DW-1:0]     wr_data,
    output wire              wr_full,
    output wire [DEPTH_LOG2:0] wr_free_words,   // write-domain view (stale ~3 clk)

    // read port (FT601/USB domain)
    input  wire              rd_clk,
    input  wire              rd_rstn,
    input  wire              rd_en,
    output wire [DW-1:0]     rd_data,
    output wire              rd_valid,
    output wire              rd_empty
);

    localparam AW = DEPTH_LOG2;          // address bits
    localparam PW = AW + 1;              // pointer width (with wrap bit)

    // ---------------------------------------------------------------- memory
    reg [DW-1:0] mem [0:(1<<AW)-1];

    // ------------------------------------------------------- gray conversion
    function [PW-1:0] bin2gray;
        input [PW-1:0] b;
        begin
            bin2gray = (b >> 1) ^ b;
        end
    endfunction

    function [PW-1:0] gray2bin;
        input [PW-1:0] g;
        reg [PW-1:0] b;
        integer i;
        begin
            b[PW-1] = g[PW-1];
            for (i = PW-2; i >= 0; i = i - 1)
                b[i] = b[i+1] ^ g[i];
            gray2bin = b;
        end
    endfunction

    // -------------------------------------------------------- write pointers
    reg  [PW-1:0] wbin, wgray;
    reg           wr_full_q;
    wire          wr_commit    = wr_en & ~wr_full_q;
    wire [PW-1:0] wbin_next    = wbin + {{(PW-1){1'b0}}, wr_commit};
    wire [PW-1:0] wgray_next   = bin2gray(wbin_next);

    // read pointer synchronized into write domain (2 FFs)
    reg [PW-1:0] rgray_wq1, rgray_wq2;
    always @(posedge wr_clk or negedge wr_rstn) begin
        if (!wr_rstn) begin
            rgray_wq1 <= {PW{1'b0}};
            rgray_wq2 <= {PW{1'b0}};
        end else begin
            rgray_wq1 <= rgray;
            rgray_wq2 <= rgray_wq1;
        end
    end

    wire [PW-1:0] wfull_cmp = {~rgray_wq2[PW-1:PW-2], rgray_wq2[PW-3:0]};
    always @(posedge wr_clk or negedge wr_rstn) begin
        if (!wr_rstn) wr_full_q <= 1'b0;     // starts empty
        else          wr_full_q <= (wgray_next == wfull_cmp);
    end
    assign wr_full = wr_full_q;

    // free-space view (write domain, stale by ~2 wr_clk + 2 rd_clk)
    wire [PW-1:0] rbin_w = gray2bin(rgray_wq2);
    wire [PW-1:0] occ_w  = wbin - rbin_w;
    localparam [PW-1:0] DEPTH_COUNT = {1'b1, {AW{1'b0}}};
    assign wr_free_words = DEPTH_COUNT - occ_w;

    // --------------------------------------------------------- read pointers
    reg [PW-1:0] rbin, rgray;
    wire rd_commit = rd_en & ~rd_empty_q;
    wire [PW-1:0] rbin_next  = rbin + {{(PW-1){1'b0}}, rd_commit};
    wire [PW-1:0] rgray_next = bin2gray(rbin_next);

    // write pointer synchronized into read domain (2 FFs)
    reg [PW-1:0] wgray_rq1, wgray_rq2;
    always @(posedge rd_clk or negedge rd_rstn) begin
        if (!rd_rstn) begin
            wgray_rq1 <= {PW{1'b0}};
            wgray_rq2 <= {PW{1'b0}};
        end else begin
            wgray_rq1 <= wgray;
            wgray_rq2 <= wgray_rq1;
        end
    end

    reg rd_empty_q;
    always @(posedge rd_clk or negedge rd_rstn) begin
        if (!rd_rstn) rd_empty_q <= 1'b1;
        else          rd_empty_q <= (rgray_next == wgray_rq2);
    end
    assign rd_empty = rd_empty_q;

    // ------------------------------------------------- registered read data
    reg [DW-1:0] rd_data_q;
    reg          rd_valid_q;
    always @(posedge rd_clk or negedge rd_rstn) begin
        if (!rd_rstn) begin
            rd_data_q  <= {DW{1'b0}};
            rd_valid_q <= 1'b0;
        end else begin
            rd_valid_q <= rd_commit;
            if (rd_commit) rd_data_q <= mem[rbin[AW-1:0]];
        end
    end
    assign rd_data  = rd_data_q;
    assign rd_valid = rd_valid_q;

    // --------------------------------------------------- pointer commitments
    always @(posedge wr_clk or negedge wr_rstn) begin
        if (!wr_rstn) begin
            wbin  <= {PW{1'b0}};
            wgray <= {PW{1'b0}};
        end else begin
            wbin  <= wbin_next;
            wgray <= wgray_next;
        end
    end

    // ------------------------------------------------------ memory write port
    // THE write into mem.  Without this statement wr_data has zero fanout and
    // synthesis prunes the ENTIRE upstream video path (60 QD pins reported
    // unused, CV0016 flood) while the FIFO degrades to an empty shell.
    // Standard synchronous write at the pre-increment write pointer: one
    // 64-bit word committed per wr_clk when wr_en && !wr_full.  Reads use the
    // registered BSRAM read port in the rd_clk domain (Cummings-style gray
    // pointer CDC, no read-during-write hazard: the read side only reaches a
    // synchronized address >= 2 rd_clk after the write pointer gray value).
    always @(posedge wr_clk) begin
        if (wr_commit) mem[wbin[AW-1:0]] <= wr_data;
    end

    always @(posedge rd_clk or negedge rd_rstn) begin
        if (!rd_rstn) begin
            rbin  <= {PW{1'b0}};
            rgray <= {PW{1'b0}};
        end else begin
            rbin  <= rbin_next;
            rgray <= rgray_next;
        end
    end

endmodule
