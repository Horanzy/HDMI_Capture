// -----------------------------------------------------------------------------
// ddr3_line_buffer.v -- bounded-latency 128-bit stream buffer for Gowin DDR3 IP
//
// One UI command transports one 128-bit word (16 B) and advances the DDR3
// address by eight 16-bit locations.  The source side uses a two-entry prefetch
// buffer so writes can issue on consecutive UI cycles.  Reads support multiple
// outstanding commands and reserve destination-FIFO space at command time.
// The command channel is still shared, so simultaneous traffic alternates reads
// and writes; unlike the old POP/WRITE + READ/RWAIT FSM, there is no per-word
// prefetch or read-latency bubble in the command schedule.
//
// The whole 2 Gbit device forms a ring, but backlog_throttle limits new capture
// packets to a small low-latency backlog.  Existing FIFO/ring contents drain in
// order and the write pointer never overwrites unread data.  Reads start at
// READ_WATERMARK words, or immediately for a tail when the input FIFO is empty.
// This block deliberately does not parse packet headers.
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

    output wire         src_rd_en,
    input  wire [127:0] src_rd_data,
    input  wire         src_rd_valid,
    input  wire         src_rd_empty,

    output wire         dst_wr_en,
    output wire [127:0] dst_wr_data,
    input  wire [10:0]  dst_wr_free_words,

    output wire [2:0]   app_cmd,
    output wire         app_cmd_en,
    output wire [27:0]  app_addr,
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

    localparam [24:0] RING_WORDS = (25'd1 << RING_WORDS_LOG2);
    localparam [27:0] ADDR_LAST  = (28'd1 << (RING_WORDS_LOG2 + 3)) - 28'd8;

    reg [27:0] wr_addr, rd_addr;
    reg [24:0] occupancy;
    reg [10:0] read_pending;
    reg        last_was_write;

    // Two entries plus one in-flight FIFO read cover the registered-read
    // async_fifo_dc latency while allowing one source word per UI cycle.
    reg [127:0] src_buf0, src_buf1;
    reg [1:0]   src_count;
    reg         src_pending;

    wire [2:0] src_total = {1'b0, src_count} + {2'b00, src_pending};
    wire [2:0] src_limit = 3'd2 + {2'b00, write_fire};

    assign src_rd_en = !src_rd_empty && (src_total < src_limit);

    wire write_available = (src_count != 2'd0) && (occupancy < RING_WORDS);
    wire write_ready = write_available && app_wr_data_rdy;

    wire read_want = (occupancy >= READ_WATERMARK)
                   || (src_rd_empty && (occupancy != 25'd0));
    wire [24:0] read_unissued = occupancy - {{14{1'b0}}, read_pending};
    wire read_available = read_want && (read_unissued != 25'd0)
                        && (dst_wr_free_words > read_pending);

    // If both directions have work, alternate commands.  At UI=100 MHz this
    // gives each direction ample margin over the default packed 2K144 rate;
    // if either direction is idle, the other can use every command slot.
    reg select_write;
    always @(*) begin
        if (write_ready && read_available)
            select_write = !last_was_write;
        else
            select_write = write_ready;
    end

    wire write_fire = select_write && write_ready && app_cmd_ready;
    wire read_fire  = !select_write && read_available && app_cmd_ready;
    wire read_return = app_rd_data_valid && init_calib_complete;

    assign app_cmd = select_write ? 3'b000 : 3'b001;
    assign app_cmd_en = write_fire || read_fire;
    assign app_addr = select_write ? wr_addr : rd_addr;
    assign app_wr_data = src_buf0;
    assign app_wr_data_en = write_fire;
    assign app_wr_data_end = write_fire; // 1:4 + BL8: one UI word per command
    assign app_wr_data_mask = 16'h0000;

    // Every issued read has reserved one destination-FIFO slot.  The generic
    // testbench asserts that a return never arrives with zero free slots.
    assign dst_wr_en = read_return;
    assign dst_wr_data = app_rd_data;

    assign backlog_throttle = (occupancy >= MAX_BACKLOG_WORDS);
    assign debug_occupancy = occupancy;

    always @(posedge clk_ui or negedge rst_n) begin
        if (!rst_n) begin
            wr_addr <= 28'd0;
            rd_addr <= 28'd0;
            occupancy <= 25'd0;
            read_pending <= 11'd0;
            last_was_write <= 1'b0;
            src_buf0 <= 128'd0;
            src_buf1 <= 128'd0;
            src_count <= 2'd0;
            src_pending <= 1'b0;
        end else if (!init_calib_complete) begin
            // A recalibration invalidates both DDR contents and outstanding
            // transactions; restart the ring instead of exposing stale data.
            wr_addr <= 28'd0;
            rd_addr <= 28'd0;
            occupancy <= 25'd0;
            read_pending <= 11'd0;
            last_was_write <= 1'b0;
            src_count <= 2'd0;
            src_pending <= 1'b0;
        end else begin
            src_pending <= src_rd_en;

            case ({src_rd_valid, write_fire})
                2'b10: begin
                    if (src_count == 2'd0)
                        src_buf0 <= src_rd_data;
                    else
                        src_buf1 <= src_rd_data;
                    src_count <= src_count + 2'd1;
                end
                2'b01: begin
                    src_buf0 <= src_buf1;
                    src_count <= src_count - 2'd1;
                end
                2'b11: begin
                    if (src_count == 2'd1) begin
                        src_buf0 <= src_rd_data;
                    end else begin
                        src_buf0 <= src_buf1;
                        src_buf1 <= src_rd_data;
                    end
                end
                default: begin end
            endcase

            if (write_fire) begin
                wr_addr <= (wr_addr == ADDR_LAST) ? 28'd0 : wr_addr + 28'd8;
                last_was_write <= 1'b1;
            end
            if (read_fire) begin
                rd_addr <= (rd_addr == ADDR_LAST) ? 28'd0 : rd_addr + 28'd8;
                last_was_write <= 1'b0;
            end

            case ({write_fire, read_return})
                2'b10: occupancy <= occupancy + 25'd1;
                2'b01: occupancy <= occupancy - 25'd1;
                default: begin end
            endcase

            case ({read_fire, read_return})
                2'b10: read_pending <= read_pending + 11'd1;
                2'b01: read_pending <= read_pending - 11'd1;
                default: begin end
            endcase
        end
    end

endmodule
