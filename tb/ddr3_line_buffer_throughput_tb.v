`timescale 1ns/1ps
module ddr3_line_buffer_throughput_tb;
    localparam integer WORDS = 20000;
    localparam integer AW = 10;
    localparam integer LAT = 6;

    reg clk_src = 0, clk_ui = 0, clk_usb = 0;
    always #3.367 clk_src = ~clk_src;
    always #5 clk_ui = ~clk_ui;
    always #5 clk_usb = ~clk_usb;

    reg rst_n = 0;
    initial #100 rst_n = 1;

    reg src_wr_en = 0;
    reg [127:0] src_wr_data = 0;
    wire src_full;
    wire [AW:0] src_free;
    wire src_rd_en, src_rd_valid, src_rd_empty;
    wire [127:0] src_rd_data;
    wire dst_wr_en;
    wire [127:0] dst_wr_data;
    wire [AW:0] dst_free;
    reg dst_rd_en = 0;
    wire [127:0] dst_rd_data;
    wire dst_rd_valid, dst_rd_empty;

    wire [2:0] app_cmd;
    wire app_cmd_en;
    wire [27:0] app_addr;
    wire [127:0] app_wr_data;
    wire app_wr_data_en, app_wr_data_end;
    wire [15:0] app_wr_data_mask;
    reg [127:0] app_rd_data = 0;
    reg app_rd_data_valid = 0;
    wire throttle;
    wire [24:0] occupancy;

    async_fifo_dc #(.DW(128), .DEPTH_LOG2(AW)) u_src_fifo (
        .wr_clk(clk_src), .wr_rstn(rst_n), .wr_en(src_wr_en),
        .wr_data(src_wr_data), .wr_full(src_full), .wr_free_words(src_free),
        .rd_clk(clk_ui), .rd_rstn(rst_n), .rd_en(src_rd_en),
        .rd_data(src_rd_data), .rd_valid(src_rd_valid), .rd_empty(src_rd_empty)
    );

    ddr3_line_buffer #(
        .RING_WORDS_LOG2(12), .READ_WATERMARK(25'd64),
        .MAX_BACKLOG_WORDS(25'd256)
    ) dut (
        .clk_ui(clk_ui), .rst_n(rst_n), .init_calib_complete(rst_n),
        .src_rd_en(src_rd_en), .src_rd_data(src_rd_data),
        .src_rd_valid(src_rd_valid), .src_rd_empty(src_rd_empty),
        .dst_wr_en(dst_wr_en), .dst_wr_data(dst_wr_data),
        .dst_wr_free_words({{(11-AW-1){1'b0}}, dst_free}),
        .app_cmd(app_cmd), .app_cmd_en(app_cmd_en), .app_addr(app_addr),
        .app_cmd_ready(1'b1), .app_wr_data(app_wr_data),
        .app_wr_data_en(app_wr_data_en), .app_wr_data_end(app_wr_data_end),
        .app_wr_data_mask(app_wr_data_mask), .app_wr_data_rdy(1'b1),
        .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid),
        .backlog_throttle(throttle), .debug_occupancy(occupancy)
    );

    async_fifo_dc #(.DW(128), .DEPTH_LOG2(AW)) u_dst_fifo (
        .wr_clk(clk_ui), .wr_rstn(rst_n), .wr_en(dst_wr_en),
        .wr_data(dst_wr_data), .wr_full(), .wr_free_words(dst_free),
        .rd_clk(clk_usb), .rd_rstn(rst_n), .rd_en(dst_rd_en),
        .rd_data(dst_rd_data), .rd_valid(dst_rd_valid), .rd_empty(dst_rd_empty)
    );

    function [127:0] pattern;
        input integer n;
        begin pattern = {32'hC2510000, n[31:0], 32'hA55A0000, ~n[31:0]}; end
    endfunction

    integer sent = 0, received = 0;
    integer write_commands = 0, read_commands = 0;
    integer max_occupancy = 0;
    reg throttle_seen = 0;

    // Deliberately offer data faster than 2K144 packed-line words.  The source
    // FIFO applies finite backpressure; the DUT itself must not hit throttle.
    always @(negedge clk_src) begin
        src_wr_en <= 1'b0;
        if (rst_n && sent < WORDS + 2000 && !src_full) begin
            src_wr_en <= 1'b1;
            src_wr_data <= pattern(sent);
            sent <= sent + 1;
        end
    end
    always @(negedge clk_usb) dst_rd_en <= rst_n && !dst_rd_empty;

    always @(posedge clk_usb) begin
        if (rst_n && dst_rd_valid) begin
            if (dst_rd_data !== pattern(received)) begin
                $display("FAIL data index=%0d", received);
                $fatal(1);
            end
            received <= received + 1;
            if (received == WORDS-1) begin
                if (throttle_seen) begin
                    $display("FAIL throttle under continuous full-rate traffic");
                    $fatal(1);
                end
                $display("PASS words=%0d writes=%0d reads=%0d max_occ=%0d throttle=0",
                         WORDS, write_commands, read_commands, max_occupancy);
                $finish;
            end
        end
    end

    reg [127:0] mem [0:4095];
    reg pipe_valid [0:LAT-1];
    reg [27:0] pipe_addr [0:LAT-1];
    integer i;
    always @(posedge clk_ui) begin
        app_rd_data_valid <= 1'b0;
        if (!rst_n) begin
            for (i = 0; i < LAT; i = i + 1) pipe_valid[i] <= 1'b0;
        end else begin
            if (occupancy > max_occupancy) max_occupancy <= occupancy;
            if (throttle) throttle_seen <= 1'b1;
            if (app_cmd_en && app_cmd == 3'b000) begin
                if (!app_wr_data_en || !app_wr_data_end || app_wr_data_mask != 0)
                    $fatal(1, "write handshake");
                mem[app_addr[14:3]] <= app_wr_data;
                write_commands <= write_commands + 1;
            end
            if (app_cmd_en && app_cmd == 3'b001)
                read_commands <= read_commands + 1;

            if (pipe_valid[LAT-1]) begin
                app_rd_data <= mem[pipe_addr[LAT-1][14:3]];
                app_rd_data_valid <= 1'b1;
                if (dst_free == 0) $fatal(1, "FIFO reservation");
            end
            for (i = LAT-1; i > 0; i = i - 1) begin
                pipe_valid[i] <= pipe_valid[i-1];
                pipe_addr[i] <= pipe_addr[i-1];
            end
            pipe_valid[0] <= (app_cmd_en && app_cmd == 3'b001);
            pipe_addr[0] <= app_addr;
        end
    end

    initial begin
        #10000000;
        $display("FAIL timeout sent=%0d received=%0d occ=%0d", sent, received, occupancy);
        $fatal(1);
    end
endmodule
