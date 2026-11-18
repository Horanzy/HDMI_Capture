// -----------------------------------------------------------------------------
// ddr3_line_buffer.v -- bounded-latency 128-bit stream buffer for Gowin DDR3 IP
//
// One UI command transports one 128-bit word (16 B) and advances the DDR3
// address by eight 16-bit locations.  The whole 2 Gbit device forms a ring, but
// backlog_throttle limits new capture packets to a small low-latency backlog.
// Existing FIFO contents are always drained; the ring never overwrites unread
// data.  Reads start at READ_WATERMARK words, or immediately for a tail when
// the input FIFO is empty.  Alternating read/write operations prevent starvation.
//
// This block deliberately does not parse packet headers.  video_stream uses
// backlog_throttle together with its whole-line reservation to drop new lines
// before either the small asynchronous FIFO or the DDR ring can overflow.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module ddr3_line_buffer #(
    parameter integer RING_WORDS_LOG2 = 24,
    parameter [24:0] READ_WATERMARK    = 25'd64,
    parameter [24:0] MAX_BACKLOG_WORDS = 25'd256
) (
    input  wire         clk_ui,
    input  wire         rst_n,
    input  wire         init_calib_complete,

    // Registered-read asynchronous input FIFO (video -> UI).
    output wire         src_rd_en,
    input  wire [127:0] src_rd_data,
    input  wire         src_rd_valid,
    input  wire         src_rd_empty,

    // Registered-write asynchronous output FIFO (UI -> USB).
    output wire         dst_wr_en,
    output wire [127:0] dst_wr_data,
    input  wire [10:0]  dst_wr_free_words,

    // Gowin DDR3 controller user interface.
    output reg  [2:0]   app_cmd,
    output reg          app_cmd_en,
    output reg  [27:0]  app_addr,
    input  wire         app_cmd_ready,
    output wire [127:0] app_wr_data,
    output wire         app_wr_data_en,
    output wire         app_wr_data_end,
    output wire [15:0]  app_wr_data_mask,
    input  wire         app_wr_data_rdy,
    input  wire [127:0] app_rd_data,
    input  wire         app_rd_data_valid,

    output wire         backlog_throttle,
    output wire [24:0]  debug_occupancy
);

    localparam [2:0] ST_IDLE  = 3'd0;
    localparam [2:0] ST_POP   = 3'd1;
    localparam [2:0] ST_WRITE = 3'd2;
    localparam [2:0] ST_READ  = 3'd3;
    localparam [2:0] ST_RWAIT = 3'd4;

    localparam [24:0] RING_WORDS = (25'd1 << RING_WORDS_LOG2);
    localparam [27:0] ADDR_LAST  = (28'd1 << (RING_WORDS_LOG2 + 3)) - 28'd8;

    reg [2:0]  state;
    reg [27:0] wr_addr, rd_addr;
    reg [24:0] occupancy;
    reg        last_was_write;

    wire write_want = !src_rd_empty && (occupancy < RING_WORDS);
    wire read_want = (occupancy >= READ_WATERMARK)
                   || (src_rd_empty && (occupancy != 25'd0));
    wire read_can = read_want && (dst_wr_free_words != 11'd0);

    // Next-state selection uses the post-operation occupancy.  Avoiding an
    // idle bubble between operations preserves enough throughput at 50 MHz.
    function [2:0] pick_next;
        input [24:0] occ_next;
        input        prefer_read;
        input        src_empty_next;
        input        dst_has_room;
        reg          wr_want_n;
        reg          rd_want_n;
        reg          rd_can_n;
        begin
            wr_want_n = !src_empty_next && (occ_next < RING_WORDS);
            rd_want_n = (occ_next >= READ_WATERMARK)
                      || (src_empty_next && (occ_next != 25'd0));
            rd_can_n = rd_want_n && dst_has_room;
            if (prefer_read && rd_can_n)
                pick_next = ST_READ;
            else if (!prefer_read && wr_want_n)
                pick_next = ST_POP;
            else if (wr_want_n)
                pick_next = ST_POP;
            else if (rd_can_n)
                pick_next = ST_READ;
            else
                pick_next = ST_IDLE;
        end
    endfunction

    wire write_fire = (state == ST_WRITE) && app_cmd_ready && app_wr_data_rdy;
    wire read_fire = (state == ST_READ) && app_cmd_ready;
    wire dst_fire = (state == ST_RWAIT) && app_rd_data_valid
                  && (dst_wr_free_words != 11'd0);

    assign src_rd_en = (state == ST_POP) && !src_rd_empty;
    assign dst_wr_en = dst_fire;
    assign dst_wr_data = app_rd_data;

    assign app_wr_data = src_rd_data;
    assign app_wr_data_en = (state == ST_WRITE) && app_cmd_ready && app_wr_data_rdy;
    assign app_wr_data_end = app_wr_data_en; // one UI word per BL8 command
    assign app_wr_data_mask = 16'h0000;

    assign backlog_throttle = (occupancy >= MAX_BACKLOG_WORDS);
    assign debug_occupancy = occupancy;

    // src_rd_valid is expected one cycle after ST_POP.  ST_WRITE necessarily
    // occurs after that cycle and the FIFO holds rd_data until the next pop.
    // Keep the signal visible to synthesis/assertion-based simulation without
    // adding another throughput-reducing wait state.
    wire unused_ok = &{1'b0, src_rd_valid, app_rd_data_valid};

    always @(*) begin
        app_cmd = 3'b000;
        app_cmd_en = 1'b0;
        app_addr = wr_addr;
        if (state == ST_WRITE) begin
            app_cmd = 3'b000;
            app_cmd_en = app_cmd_ready && app_wr_data_rdy;
            app_addr = wr_addr;
        end else if (state == ST_READ) begin
            app_cmd = 3'b001;
            app_cmd_en = app_cmd_ready;
            app_addr = rd_addr;
        end
    end

    always @(posedge clk_ui or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            wr_addr <= 28'd0;
            rd_addr <= 28'd0;
            occupancy <= 25'd0;
            last_was_write <= 1'b0;
        end else if (!init_calib_complete) begin
            state <= ST_IDLE;
            last_was_write <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (last_was_write && read_can)
                        state <= ST_READ;
                    else if (!last_was_write && write_want)
                        state <= ST_POP;
                    else if (write_want)
                        state <= ST_POP;
                    else if (read_can)
                        state <= ST_READ;
                end

                ST_POP: begin
                    if (!src_rd_empty)
                        state <= ST_WRITE;
                    else
                        state <= ST_IDLE;
                end

                ST_WRITE: begin
                    if (write_fire) begin
                        wr_addr <= (wr_addr == ADDR_LAST) ? 28'd0 : wr_addr + 28'd8;
                        occupancy <= occupancy + 25'd1;
                        last_was_write <= 1'b1;
                        state <= pick_next(occupancy + 25'd1, 1'b1,
                                           src_rd_empty,
                                           (dst_wr_free_words != 11'd0));
                    end
                end

                ST_READ: begin
                    if (read_fire)
                        state <= ST_RWAIT;
                end

                ST_RWAIT: begin
                    if (dst_fire) begin
                        rd_addr <= (rd_addr == ADDR_LAST) ? 28'd0 : rd_addr + 28'd8;
                        occupancy <= occupancy - 25'd1;
                        last_was_write <= 1'b0;
                        state <= pick_next(occupancy - 25'd1, 1'b0,
                                           src_rd_empty,
                                           (dst_wr_free_words > 11'd1));
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
