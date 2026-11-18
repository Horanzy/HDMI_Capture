`timescale 1ns/1ps
module ddr3_line_buffer_tb;
    localparam integer WORDS = 4000;
    localparam integer AW = 4;

    reg clk_src = 0, clk_ui = 0, clk_usb = 0;
    always #3.367 clk_src = ~clk_src;
    always #10 clk_ui = ~clk_ui;
    always #5 clk_usb = ~clk_usb;

    reg rst_n = 0;
    initial begin
        #100 rst_n = 1;
    end

    reg src_wr_en = 0;
    reg [127:0] src_wr_data = 0;
    wire src_full;
    wire [AW:0] src_free;
    wire src_rd_en, src_rd_valid, src_rd_empty;
    wire [127:0] src_rd_data;

    wire dst_wr_en;
    wire [127:0] dst_wr_data;
    wire [AW:0] dst_free_small;
    wire [10:0] dst_free = {{(11-AW-1){1'b0}}, dst_free_small};
    reg dst_rd_en = 0;
    wire [127:0] dst_rd_data;
    wire dst_rd_valid, dst_rd_empty;

    wire [2:0] app_cmd;
    wire app_cmd_en;
    wire [27:0] app_addr;
    reg app_cmd_ready = 0;
    wire [127:0] app_wr_data;
    wire app_wr_data_en, app_wr_data_end;
    wire [15:0] app_wr_data_mask;
    reg app_wr_data_rdy = 0;
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
        .RING_WORDS_LOG2(4),
        .READ_WATERMARK(25'd4),
        .MAX_BACKLOG_WORDS(25'd8)
    ) dut (
        .clk_ui(clk_ui), .rst_n(rst_n), .init_calib_complete(rst_n),
        .src_rd_en(src_rd_en), .src_rd_data(src_rd_data),
        .src_rd_valid(src_rd_valid), .src_rd_empty(src_rd_empty),
        .dst_wr_en(dst_wr_en), .dst_wr_data(dst_wr_data),
        .dst_wr_free_words(dst_free),
        .app_cmd(app_cmd), .app_cmd_en(app_cmd_en), .app_addr(app_addr),
        .app_cmd_ready(app_cmd_ready), .app_wr_data(app_wr_data),
        .app_wr_data_en(app_wr_data_en), .app_wr_data_end(app_wr_data_end),
        .app_wr_data_mask(app_wr_data_mask), .app_wr_data_rdy(app_wr_data_rdy),
        .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid),
        .backlog_throttle(throttle), .debug_occupancy(occupancy)
    );

    async_fifo_dc #(.DW(128), .DEPTH_LOG2(AW)) u_dst_fifo (
        .wr_clk(clk_ui), .wr_rstn(rst_n), .wr_en(dst_wr_en),
        .wr_data(dst_wr_data), .wr_full(), .wr_free_words(dst_free_small),
        .rd_clk(clk_usb), .rd_rstn(rst_n), .rd_en(dst_rd_en),
        .rd_data(dst_rd_data), .rd_valid(dst_rd_valid), .rd_empty(dst_rd_empty)
    );

    integer sent = 0;
    integer received = 0;
    integer seed = 32'h51a7c0de;
    reg dst_allow = 0;
    reg throttle_seen = 0, wrap_seen = 0;

    function [127:0] pattern;
        input integer n;
        begin
            pattern = {32'hC2510000, n[31:0], 32'hA55A0000, ~n[31:0]};
        end
    endfunction

    // Source bubbles and finite FIFO backpressure.
    always @(negedge clk_src) begin
        src_wr_en <= 1'b0;
        if (rst_n && sent < WORDS && !src_full && (($random(seed) & 3) != 0)) begin
            src_wr_en <= 1'b1;
            src_wr_data <= pattern(sent);
            sent <= sent + 1;
        end
    end

    // Hold the consumer initially to force backlog and throttle, then add stalls.
    initial begin
        #20000 dst_allow = 1;
    end
    always @(negedge clk_usb) begin
        dst_rd_en <= 1'b0;
        if (rst_n && dst_allow && !dst_rd_empty && (($random(seed) & 3) != 0))
            dst_rd_en <= 1'b1;
    end

    always @(posedge clk_usb) begin
        if (rst_n && dst_rd_valid) begin
            if (dst_rd_data !== pattern(received)) begin
                $display("FAIL data index=%0d got=%032h expected=%032h",
                         received, dst_rd_data, pattern(received));
                $fatal(1);
            end
            received <= received + 1;
            if (received == WORDS-1) begin
                if (!throttle_seen || !wrap_seen) begin
                    $display("FAIL coverage throttle=%0d wrap=%0d", throttle_seen, wrap_seen);
                    $fatal(1);
                end
                $display("PASS words=%0d throttle=%0d wrap=%0d", WORDS, throttle_seen, wrap_seen);
                $finish;
            end
        end
    end

    // Randomly backpressure the generic DDR command interface.
    always @(negedge clk_ui) begin
        app_cmd_ready <= (($random(seed) & 3) != 0);
        app_wr_data_rdy <= (($random(seed) & 7) != 0);
    end

    reg [127:0] mem [0:15];
    reg read_pending = 0;
    reg [27:0] read_addr = 0;
    integer read_delay = 0;

    always @(posedge clk_ui) begin
        app_rd_data_valid <= 1'b0;
        if (!rst_n) begin
            read_pending <= 1'b0;
        end else begin
            if (occupancy > 16) begin
                $display("FAIL occupancy=%0d", occupancy);
                $fatal(1);
            end
            if (throttle) throttle_seen <= 1'b1;
            if (app_cmd_en && app_cmd == 3'b000 && app_addr == 28'd120)
                wrap_seen <= 1'b1;
            if (app_cmd_en && app_cmd == 3'b000) begin
                if (!app_wr_data_en || !app_wr_data_end || app_wr_data_mask != 0) begin
                    $display("FAIL write handshake");
                    $fatal(1);
                end
                mem[app_addr[6:3]] <= app_wr_data;
            end
            if (app_cmd_en && app_cmd == 3'b001) begin
                if (read_pending) begin
                    $display("FAIL multiple outstanding reads");
                    $fatal(1);
                end
                read_pending <= 1'b1;
                read_addr <= app_addr;
                read_delay <= ($random(seed) & 7);
            end else if (read_pending) begin
                if (read_delay == 0) begin
                    app_rd_data <= mem[read_addr[6:3]];
                    app_rd_data_valid <= 1'b1;
                    read_pending <= 1'b0;
                end else begin
                    read_delay <= read_delay - 1;
                end
            end
        end
    end

    initial begin
        #10000000;
        $display("FAIL timeout sent=%0d received=%0d occ=%0d", sent, received, occupancy);
        $fatal(1);
    end
endmodule
