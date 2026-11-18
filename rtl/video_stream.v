// -----------------------------------------------------------------------------
// video_stream.v -- format-transparent FullBusDDR capture and line packetizer
//
// IT68051 supplies two 60-bit, two-pixel words per PCLK.  IDDR Q0/Q1 align
// both words at the same internal rising edge: word_r contains the earlier
// rising-edge Pixel0/1 and word_f the following falling-edge Pixel2/3.
// Within each word pixel A is earlier than pixel B.  pixel_fmt is detected
// automatically from IT68051 input state; IT68051 performs color conversion.
//
// AUTO_FORMAT mode transports each 30-bit channel slot in a 32-bit container.
// The default fixed-422 mode instead packs Cb/Y0/Cr/Y1 as 4x10 bits per pixel
// pair: no samples or low bits are discarded.  This reduces a 640-pixel line
// from 2560 B to 1600 B and is required because consecutive active lines can
// arrive faster than the FT601's theoretical 400 MB/s drain rate.
//   0: YUV42210 raw slots, 2560 B/line
//   1: RGB888 source mode, raw 10-bit slots, 2560 B/line
//   2: YUV444 source mode, raw 10-bit slots, 2560 B/line
//   3: RGB10 source mode, raw 10-bit slots, 2560 B/line
//   4: YUV44410 source mode, raw 10-bit slots, 2560 B/line
//   5: packed YUV42210, 1600 B/line (Linux emits UYVY8 or packed 10-bit)
//
// Packets have a 16-byte header with sync bytes C2 51 A5; STATUS is header-only.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module video_stream #(
    parameter CROP_W = 640,
    parameter PACK_422 = 0
) (
    input  wire         clk_vid,
    input  wire         rstn,
    input  wire [59:0]  word_r,
    input  wire [59:0]  word_f,
    input  wire         de_pre,  // one PCLK before IDDR-aligned pixel data
    input  wire         de_s,
    input  wire         hs_s,
    input  wire         vs_s,
    input  wire         stream_en,
    input  wire [2:0]   pixel_fmt,
    input  wire [11:0]  crop_x0,
    input  wire [11:0]  crop_y0,
    input  wire [15:0]  status_bits,
    output reg          hs_tick,
    output wire         htot_ok,
    output reg          stream_stable,
    output reg          wr_en,
    output reg  [127:0] wr_data,
    input  wire         wr_full,
    input  wire [10:0]  wr_free_words,
    input  wire         ddr_throttle
);

    localparam [2:0] FMT_UYVY8 = 3'd0;
    localparam [7:0] PAY128_RAW_M1 = PACK_422 ? 8'd99 : 8'd159;
    localparam [10:0] LINE_WORDS = PACK_422 ? 11'd101 : 11'd161;

    // ------------------------------------------------------------- syncs
    reg de_d1, pre_d1, vs_d1;
    wire de_rise = de_s & ~de_d1;
    wire pre_rise = de_pre & ~pre_d1;
    wire vs_rise = vs_s & ~vs_d1;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            de_d1 <= 1'b0;
            pre_d1 <= 1'b0;
            vs_d1 <= 1'b0;
        end else begin
            de_d1 <= de_s;
            pre_d1 <= de_pre;
            vs_d1 <= vs_s;
        end
    end

    // -------------------------------------------------- line/frame telemetry
    reg [31:0] frame_idx;
    reg [12:0] line_cnt, r_cnt, de_w, htot, de_w_q, htot_q;
    reg [1:0] good_hist;
    wire line_good = (de_w_q[0] == 1'b0) && (htot_q[0] == 1'b0)
                   && (de_w_q < htot_q)
                   && (htot_q >= 13'd100) && (htot_q <= 13'd4095);

    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            frame_idx <= 32'd0;
            line_cnt <= 13'd0;
            r_cnt <= 13'd0;
            de_w <= 13'd0;
            htot <= 13'd0;
            de_w_q <= 13'd0;
            htot_q <= 13'd0;
            good_hist <= 2'd0;
            hs_tick <= 1'b0;
            stream_stable <= 1'b0;
        end else begin
            hs_tick <= 1'b0;
            if (vs_rise) begin
                frame_idx <= frame_idx + 32'd1;
                line_cnt <= 13'd0;
            end
            if (de_rise) begin
                hs_tick <= 1'b1;
                de_w_q <= de_w;
                htot_q <= htot;
                good_hist <= {line_good, good_hist[1]};
                stream_stable <= (good_hist == 2'b11);
                line_cnt <= line_cnt + 13'd1;
                r_cnt <= 13'd1;  // the first four pixels are present now
                de_w <= 13'd1;
                htot <= 13'd1;
            end else begin
                r_cnt <= r_cnt + (de_s ? 13'd1 : 13'd0);
                de_w <= de_w + (de_s ? 13'd1 : 13'd0);
                htot <= htot + 13'd1;
            end
        end
    end
    assign htot_ok = (de_w_q[0] == 1'b0) && (htot_q[0] == 1'b0);

    // ------------------------------------------------ crop/format frame latch
    reg [11:0] x0_q, y0_q;
    reg [2:0] fmt_q;
    reg en_q;
    wire [11:0] x0_eff = {crop_x0[11:2], 2'b00};
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            x0_q <= 12'd960;
            y0_q <= 12'd400;
            fmt_q <= FMT_UYVY8;
            en_q <= 1'b0;
        end else if (vs_rise) begin
            x0_q <= x0_eff;
            y0_q <= crop_y0;
            fmt_q <= pixel_fmt;
            en_q <= stream_en;
        end
    end

    wire in_line = (line_cnt >= {1'b0, y0_q})
                 && (line_cnt < ({1'b0, y0_q} + 13'd640));
    wire [9:0] line_idx = line_cnt[9:0] - y0_q[9:0];
    wire eolf = (line_idx == 10'd639);
    reg eolf_q;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) eolf_q <= 1'b0;
        else if (de_rise) eolf_q <= eolf;
    end

    // ------------------------------------------------------- crop word gate
    // Each aligned PCLK contains four consecutive pixels: rising Pixel0/1
    // followed by falling Pixel2/3.  At de_rise, r_cnt has not yet been loaded
    // with one, so explicitly use pixel base zero for that first word.
    wire word_raw = de_s & en_q;
    wire [15:0] base_r = de_rise ? 16'd0 : {1'b0, r_cnt, 2'b00};
    wire [15:0] base_f = base_r + 16'd2;
    wire [15:0] x0_16 = {2'b0, x0_q, 2'b00};
    wire [15:0] crop_end = x0_16 + 16'd640;
    wire in_f = word_raw && (base_f >= x0_16) && ((base_f + 16'd2) <= crop_end);
    wire in_r = word_raw && (base_r >= x0_16) && ((base_r + 16'd2) <= crop_end);

    // AUTO_FORMAT transparent transport: p[29:20], p[19:10], p[9:0] retain
    // their original IT68051 pin/channel meanings.
    //
    // FPGA_IMAGE_PROCESSING_INSERTION_POINT:
    // If FPGA-side CSC or scaling is ever required, insert it here before the
    // packing below.  It is disabled for low-latency transparent firmware.
    wire [63:0] f_pack = {2'b00, word_f[59:30], 2'b00, word_f[29:0]};
    wire [63:0] r_pack = {2'b00, word_r[59:30], 2'b00, word_r[29:0]};
    wire [127:0] raw_quad = {f_pack, r_pack};

    // Lossless fixed-422 packing, using the IT68051 YUV422 pin map (not the
    // RGB channel interpretation): A_R=Cb, A_G=Y0, B_R=Cr, B_G=Y1.
    // QE[9:0] and QE[39:30] are unused in this mode.
    // pair[9:0]=Cb, [19:10]=Y0, [29:20]=Y1, [39:30]=Cr.
    // Two PCLK pairs form 80 bits; the gearbox fills complete 128-bit FIFO
    // words.  160 PCLKs therefore produce exactly 100 payload words per line.
    wire [39:0] pair40_r = {word_r[59:50], word_r[49:40],
                            word_r[19:10], word_r[29:20]};
    wire [39:0] pair40_f = {word_f[59:50], word_f[49:40],
                            word_f[19:10], word_f[29:20]};
    wire [79:0] raw80 = {pair40_f, pair40_r};
    reg  [255:0] pack_acc;
    reg  [7:0]   pack_count;
    wire [255:0] pack_sum = pack_acc | ({176'd0, raw80} << pack_count);
    wire         pack_emit = (pack_count >= 8'd48); // count + 80 >= 128

    // The IDDR has already aligned both edge words; no fabric shift pipeline
    // is needed.  With the four-pixel crop alignment, in_r and in_f are equal.
    wire pair_v = in_r & in_f;

    // -------------------------------------------------------- packet writer
    reg in_packet, in_status;
    reg [7:0] pay_cnt;
    reg drop_latch;
    reg [15:0] drop_cnt;

    wire line_start = pre_rise & en_q & in_line;
    wire [11:0] crop_r_end = {2'b0, x0_q[11:2]} + 12'd160;
    wire crop_fits = (de_w_q >= crop_r_end);
    wire [10:0] reserve_words = LINE_WORDS + 11'd4 + (eolf ? 11'd1 : 11'd0);
    wire fifo_room = (wr_free_words >= reserve_words) && !ddr_throttle;
    reg  fifo_room_q;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) fifo_room_q <= 1'b0;
        else       fifo_room_q <= fifo_room;
    end
    wire raw_last = pair_v && (pay_cnt == PAY128_RAW_M1) &&
                    (!PACK_422 || pack_emit);

    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            wr_en <= 1'b0;
            wr_data <= 128'h0;
            in_packet <= 1'b0;
            in_status <= 1'b0;
            pay_cnt <= 8'd0;
            drop_latch <= 1'b0;
            drop_cnt <= 16'd0;
            pack_acc <= 256'd0;
            pack_count <= 8'd0;
        end else begin
            wr_en <= 1'b0;
            if (!en_q) begin
                in_packet <= 1'b0;
                in_status <= 1'b0;
                drop_latch <= 1'b0;
            end else if (line_start && crop_fits
                         && !in_packet && !in_status) begin
                if (fifo_room_q) begin
                    // de_pre is exactly one PCLK ahead of the first IDDR-aligned
                    // pixel quad.  Enter in_packet immediately: the next cycle
                    // must accept pixel data, not wait in a separate header state.
                    in_packet <= 1'b1;
                    pay_cnt <= 8'd0;
                    drop_latch <= 1'b0;
                    pack_acc <= 256'd0;
                    pack_count <= 8'd0;
                    wr_en <= 1'b1;
                    wr_data <= {status_bits, drop_cnt,
                                24'hA551C2, 3'd0, fmt_q, drop_latch, eolf,
                                16'h0, 6'h0, line_idx, frame_idx};
                end else begin
                    drop_latch <= 1'b1;
                    drop_cnt <= drop_cnt + 16'd1;
                end
            end else if (in_packet && pair_v) begin
                if (!PACK_422) begin
                    wr_en <= 1'b1;
                    wr_data <= raw_quad;
                end else if (pack_emit) begin
                    wr_en <= 1'b1;
                    wr_data <= pack_sum[127:0];
                    pack_acc <= pack_sum >> 128;
                    pack_count <= pack_count - 8'd48;
                end else begin
                    pack_acc <= pack_sum;
                    pack_count <= pack_count + 8'd80;
                end
                if (raw_last) begin
                    in_packet <= 1'b0;
                    pay_cnt <= 8'd0;
                    pack_acc <= 256'd0;
                    pack_count <= 8'd0;
                    if (eolf_q) in_status <= 1'b1;
                end else if (!PACK_422 || pack_emit) begin
                    pay_cnt <= pay_cnt + 8'd1;
                end
            end else if (in_status) begin
                wr_en <= 1'b1;
                wr_data <= {status_bits, drop_cnt,
                            24'hA551C2, 3'd1, fmt_q, 1'b0, 1'b1,
                            16'h0, 16'hFFFF, frame_idx};
                in_status <= 1'b0;
            end
        end
    end

    // hs_s and wr_full are intentionally retained as bring-up/status ports.
    // The source cannot be back-pressured; fifo_room reserves whole packets.
    wire unused_ok = &{1'b0, hs_s, wr_full, drop_cnt[0]};

endmodule
