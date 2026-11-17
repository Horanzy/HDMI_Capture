// -----------------------------------------------------------------------------
// video_stream.v -- video-domain capture chain: crop window, RGB->YUV422,
//                   line packet assembly (16B header + 1280B payload)
//
// Clock domain: clk_vid = RX_PCLK (IT68051 dual-pixel DDR, 148.5 MHz @2K144).
//
// Per PCLK cycle the IT68051 presents TWO 2-pixel words:
//   word_f : captured at the falling edge  (older pixels, base = 4*r-2)
//   word_r : captured at the rising edge   (newer pixels, base = 4*r)
//   where r counts rising edges inside DE (r=0 at the first DE pixel).
//   Word layout (V1.4 netlist labels):
//     [29:0]  pixel A = {R[29:20], G[19:10], B[9:0]}
//     [59:30] pixel B = same order   (A is the earlier pixel of the word)
//
// FIFO write port: 64-bit, ONE write per PCLK cycle.  Payload pairing:
//   pair_k = {hi = f-word of cycle x0/4+k+1, lo = r-word of cycle x0/4+k}
//   - lo holds the earlier pixels (4r..4r+1), hi the next two (4r+2..4r+3);
//     the FT601 bus sends the low 32 bits first -> correct pixel order.
//   - the two YUV pipelines align naturally (f-path and r-path outputs both
//     valid from cycle x0/4+8), giving exactly 160 payload writes per line.
//
// Output stream (into the async FIFO), 162 64-bit words per packet.
// Byte order on the wire: each 64-bit FIFO word leaves the FT601 as lo[31:0]
// first (bytes D0..D3 = LSB first), then hi[63:32]; the host parses the
// stream as little-endian u32 words w0..w3 (sw/linux_rx.py Receiver.feed).
//   w64[0] = wire w0+w1 : lo = frame_idx[31:0], hi = {16'h0, line_idx[15:0]}
//   w64[1] = wire w2+w3 : lo = {24'h0, flags[7:0]}, hi = {status[15:0], 16'h0}
//          flags: bit0=EOLF bit1=OVERRUN bits[7:5]=type (0=LINE, 1=STATUS)
//   w64[2..161] payload = 160 UYVY pair words
// A STATUS packet (type=1, line_idx=0xFFFF, zero payload) follows the EOLF
// line of every frame (162 words: header pair + 160 zero pairs).
//
// Overrun policy: a line starts only if the FIFO reports
// LINE_W64+4 (+324/2... i.e. +162 for the STATUS packet on EOLF lines) free
// words; otherwise the line is dropped whole and the OVERRUN flag is stamped
// on the next successfully written line header.  The video path is never
// back-pressured.  Lines whose source DE is too narrow for the window are
// never started (would desync the host).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module video_stream #(
    parameter CROP_W    = 640,           // frozen capture width (pixels)
    parameter LINE_W64  = 162,           // 2 header + 160 payload 64-bit words
    parameter BT709     = 0
) (
    input  wire        clk_vid,
    input  wire        rstn,

    // from iddr_bus_in
    input  wire [59:0] word_r,
    input  wire [59:0] word_f,
    input  wire        de_s,
    input  wire        hs_s,            // sampled; UNUSED in v1 (no hsync-edge
                                        // logic; port kept so RX_HSYNC stays a
                                        // valid bring-up probe in the .cst)
    input  wire        vs_s,

    // control (quasi-static, synchronized, latched at vs_rise)
    input  wire        stream_en,       // init_ctrl: HPD asserted & stream stable
    input  wire [11:0] crop_x0,         // source x of window, %4 forced in RTL
    input  wire [11:0] crop_y0,
    input  wire [15:0] status_bits,     // device status word for packet word 3

    // video-domain telemetry
    output reg         hs_tick,         // one clk_vid pulse per source line
    output wire        htot_ok,         // even line period and even DE width
    output reg         stream_stable,   // two consecutive well-formed lines

    // async FIFO write port (64-bit)
    output reg         wr_en,
    output reg  [63:0] wr_data,
    input  wire        wr_full,         // unused by policy (never back-pressures)
    input  wire [11:0] wr_free_words
);

    localparam [7:0] PAY_WORDS_M1 = (CROP_W / 4) - 1;   // 159 (64-bit pairs)
    localparam [7:0] PAY_WORDS    = (CROP_W / 4);       // 160

    // ---------------------------------------------------------------- syncs
    // (hs_d1 removed: hs_s has no edge logic in v1 -- keeping the register
    // made RX_HSYNC a write-only net and invited "unused input" pruning.)
    reg  de_d1, vs_d1;
    wire de_rise = de_s & ~de_d1;
    wire vs_rise = vs_s & ~vs_d1;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            de_d1 <= 1'b0; vs_d1 <= 1'b0;
        end else begin
            de_d1 <= de_s; vs_d1 <= vs_s;
        end
    end

    // ------------------------------------------------- line/frame telemetry
    reg  [31:0] frame_idx;
    reg  [12:0] line_cnt;        // DE rises since VSYNC (0 during first line)
    reg  [12:0] r_cnt;           // rising-edge index inside current DE
    reg  [12:0] de_w;            // DE width counter (current line)
    reg  [12:0] htot;            // PCLK cycles since last DE rise
    reg  [12:0] de_w_q, htot_q;
    reg  [1:0]  good_hist;

    wire line_good = (de_w_q[0] == 1'b0) && (htot_q[0] == 1'b0)
                   && (de_w_q < htot_q)
                   && (htot_q >= 13'd100) && (htot_q <= 13'd4095);

    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            frame_idx <= 32'd0; line_cnt <= 13'd0;
            r_cnt <= 13'd0; de_w <= 13'd0; htot <= 13'd0;
            de_w_q <= 13'd0; htot_q <= 13'd0;
            good_hist <= 2'd0;
            hs_tick <= 1'b0;
            stream_stable <= 1'b0;
        end else begin
            hs_tick    <= 1'b0;
            if (vs_rise) begin
                frame_idx  <= frame_idx + 32'd1;
                line_cnt   <= 13'd0;
            end
            if (de_rise) begin
                hs_tick    <= 1'b1;
                de_w_q     <= de_w;                 // stats of the line that ended
                htot_q     <= htot;
                good_hist  <= {line_good, good_hist[1]};
                stream_stable <= (good_hist == 2'b11);
                line_cnt   <= line_cnt + 13'd1;
                r_cnt      <= 13'd0;
                de_w       <= 13'd1;
                htot       <= 13'd1;
            end else begin
                r_cnt  <= r_cnt + (de_s ? 13'd1 : 13'd0);
                de_w   <= de_w + (de_s ? 13'd1 : 13'd0);
                htot   <= htot + 13'd1;
            end
        end
    end

    // IT68051 datasheet: dual-pixel DDR needs source HTotal%4==0.  With
    // HActive%4==0 as well this shows up as even line period and even DE
    // width in PCLK cycles - checked here.
    assign htot_ok = (de_w_q[0] == 1'b0) && (htot_q[0] == 1'b0);

    // ------------------------------------------------------------ crop latch
    reg [11:0] x0_q, y0_q;
    reg        en_q;
    wire [11:0] x0_eff = {crop_x0[11:2], 2'b00};      // force x0 %4 == 0
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            x0_q <= 12'd960;                          // defaults: 2560x1440
            y0_q <= 12'd400;                          //   centered 640x640
            en_q <= 1'b0;
        end else if (vs_rise) begin
            x0_q <= x0_eff;
            y0_q <= crop_y0;
            en_q <= stream_en;
        end
    end

    wire        in_line  = (line_cnt >= {1'b0, y0_q}) &&
                           (line_cnt <  ({1'b0, y0_q} + 13'd640));
    wire [9:0]  line_idx = line_cnt[9:0] - y0_q[9:0];      // 0..639 in window
    wire        eolf     = (line_idx == 10'd639);

    // EOLF is registered at line start: during header/payload write the line
    // counter has already moved on to the next source line.  (The line index
    // itself is used combinationally in the line_start cycle -- li_q existed
    // but was never read, removed.)
    reg       eolf_q;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            eolf_q <= 1'b0;
        end else if (de_rise) begin
            eolf_q <= eolf;
        end
    end

    // -------------------------------------------------- word index / crop gate
    wire        word_f_raw = de_d1 & de_s & en_q;         // falling word valid
    wire        word_r_raw = de_s & en_q;                 // rising word valid
    wire [15:0] base_f     = {1'b0, r_cnt, 2'b00} - 16'd2;
    wire [15:0] base_r     = {1'b0, r_cnt, 2'b00};
    wire [15:0] x0_16      = {4'b0, x0_q, 2'b00};
    wire [15:0] crop_end   = x0_16 + 16'd640;

    wire in_f = word_f_raw && (base_f >= x0_16) && ((base_f + 16'd2) <= crop_end);
    wire in_r = word_r_raw && (base_r >= x0_16) && ((base_r + 16'd2) <= crop_end);

    // RGB8 extraction (top 8 of each 10-bit channel)
    wire [7:0] f_ar = word_f[29:22], f_ag = word_f[19:12], f_ab = word_f[9:2];
    wire [7:0] f_br = word_f[59:52], f_bg = word_f[49:42], f_bb = word_f[39:32];
    wire [7:0] r_ar = word_r[29:22], r_ag = word_r[19:12], r_ab = word_r[9:2];
    wire [7:0] r_br = word_r[59:52], r_bg = word_r[49:42], r_bb = word_r[39:32];

    // pipeline delays: both YUV outputs must be valid in the SAME cycle for
    // the 64-bit pairing (f input delayed 4, r input delayed 5; pipelines 3
    // deep -> both outputs from cycle x0/4+8).  Boundary cycles carry only
    // one valid word each and pair up symmetrically (first r with next f,
    // last f with previous r).
    reg        vf_d [0:3];
    reg [7:0]  vf_ar [0:3], vf_ag [0:3], vf_ab [0:3];
    reg [7:0]  vf_br [0:3], vf_bg [0:3], vf_bb [0:3];
    reg        vr_d [0:4];
    reg [7:0]  vr_ar [0:4], vr_ag [0:4], vr_ab [0:4];
    reg [7:0]  vr_br [0:4], vr_bg [0:4], vr_bb [0:4];
    integer k;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            for (k = 0; k <= 3; k = k + 1) begin
                vf_d[k] <= 1'b0;
                vf_ar[k] <= 8'h0; vf_ag[k] <= 8'h0; vf_ab[k] <= 8'h0;
                vf_br[k] <= 8'h0; vf_bg[k] <= 8'h0; vf_bb[k] <= 8'h0;
            end
            for (k = 0; k <= 4; k = k + 1) begin
                vr_d[k] <= 1'b0;
                vr_ar[k] <= 8'h0; vr_ag[k] <= 8'h0; vr_ab[k] <= 8'h0;
                vr_br[k] <= 8'h0; vr_bg[k] <= 8'h0; vr_bb[k] <= 8'h0;
            end
        end else begin
            vf_d[0] <= in_f;
            vf_ar[0] <= f_ar; vf_ag[0] <= f_ag; vf_ab[0] <= f_ab;
            vf_br[0] <= f_br; vf_bg[0] <= f_bg; vf_bb[0] <= f_bb;
            for (k = 1; k <= 3; k = k + 1) begin
                vf_d[k] <= vf_d[k-1];
                vf_ar[k] <= vf_ar[k-1]; vf_ag[k] <= vf_ag[k-1]; vf_ab[k] <= vf_ab[k-1];
                vf_br[k] <= vf_br[k-1]; vf_bg[k] <= vf_bg[k-1]; vf_bb[k] <= vf_bb[k-1];
            end
            vr_d[0] <= in_r;
            vr_ar[0] <= r_ar; vr_ag[0] <= r_ag; vr_ab[0] <= r_ab;
            vr_br[0] <= r_br; vr_bg[0] <= r_bg; vr_bb[0] <= r_bb;
            for (k = 1; k <= 4; k = k + 1) begin
                vr_d[k] <= vr_d[k-1];
                vr_ar[k] <= vr_ar[k-1]; vr_ag[k] <= vr_ag[k-1]; vr_ab[k] <= vr_ab[k-1];
                vr_br[k] <= vr_br[k-1]; vr_bg[k] <= vr_bg[k-1]; vr_bb[k] <= vr_bb[k-1];
            end
        end
    end

    wire        yuv_f_v, yuv_r_v;
    wire [31:0] yuv_f, yuv_r;
    rgb2yuv422 #(.BT709(BT709)) u_yuv_f (
        .clk(clk_vid), .rstn(rstn),
        .in_valid(vf_d[3]),
        .p0_r(vf_ar[3]), .p0_g(vf_ag[3]), .p0_b(vf_ab[3]),
        .p1_r(vf_br[3]), .p1_g(vf_bg[3]), .p1_b(vf_bb[3]),
        .out_valid(yuv_f_v), .out_uyvy(yuv_f));
    rgb2yuv422 #(.BT709(BT709)) u_yuv_r (
        .clk(clk_vid), .rstn(rstn),
        .in_valid(vr_d[4]),
        .p0_r(vr_ar[4]), .p0_g(vr_ag[4]), .p0_b(vr_ab[4]),
        .p1_r(vr_br[4]), .p1_g(vr_bg[4]), .p1_b(vr_bb[4]),
        .out_valid(yuv_r_v), .out_uyvy(yuv_r));

    // payload pair: both pipelines coincide; lo = earlier pixels (r-word),
    // hi = later pixels (f-word).  Exactly CROP_W/4 pairs per line.
    wire pair_v   = yuv_f_v & yuv_r_v;
    wire [63:0] pair_d = {yuv_f, yuv_r};       // lo = yuv_r (earlier)

    // -------------------------------------------------------- packet writer
    //  in_hdr    : writing the 2 header 64-bit words
    //  in_packet : payload streaming for a captured line
    //  in_status : writing the per-frame STATUS packet (162 words)
    reg        in_hdr, in_packet, in_status;
    reg [7:0]  pay_cnt;
    reg [7:0]  stat_cnt;
    reg        drop_latch;
    reg [15:0] drop_cnt;

    wire line_start = de_rise & en_q & in_line;
    // the source DE width (measured on the previous line) must span the whole
    // crop window, else the packet would end up short and desync the host
    wire [11:0] crop_r_end = {2'b0, x0_q[11:2]} + 12'd160;  // last rising edge
    wire crop_fits = (de_w_q >= crop_r_end);
    // reserve room for the line; an EOLF line additionally reserves the whole
    // STATUS packet that follows it (otherwise the STATUS words could hit
    // FIFO-full and be silently lost)
    wire fifo_room  = (wr_free_words >= (LINE_W64 + 4 +
                                         (eolf ? 8'd162 : 8'd0)));

    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            wr_en <= 1'b0; wr_data <= 64'h0;
            in_hdr <= 1'b0; in_packet <= 1'b0; in_status <= 1'b0;
            pay_cnt <= 8'd0; stat_cnt <= 8'd0;
            drop_latch <= 1'b0; drop_cnt <= 16'd0;
        end else begin
            wr_en <= 1'b0;
            if (!en_q) begin
                in_hdr <= 1'b0; in_packet <= 1'b0; in_status <= 1'b0;
                drop_latch <= 1'b0;
            end else if (line_start && crop_fits &&
                         !in_packet && !in_hdr && !in_status) begin
                if (fifo_room) begin
                    in_hdr    <= 1'b1;
                    pay_cnt   <= 8'd0;
                    wr_en     <= 1'b1;
                    // w64[0] -- wire bytes 0..7, little-endian 32-bit words
                    // (linux_rx.py: w0=frame_idx, w1=line_idx&0xFFFF):
                    //   lo = frame_idx[31:0]  hi = {rsv 16'h0, line_idx u16}
                    // -> line_idx sits at wr_data[47:32], frame_idx at [31:0].
                    wr_data   <= {16'h0000, 6'h0000, line_idx, frame_idx};
                end else begin
                    drop_latch <= 1'b1;                      // drop whole line
                    drop_cnt   <= drop_cnt + 16'd1;
                end
            end else if (in_hdr) begin
                wr_en   <= 1'b1;
                // w64[1] -- wire bytes 8..15 (linux_rx.py: flags = w2 & 0xFF,
                // typ = flags[7:5], status = w3[31:16]):
                //   lo = {rsv 24'h0, flags[7:0]}, hi = {status u16, rsv 16'h0}
                //   flags = {type=LINE(3'd0), 3'b000, OVERRUN, EOLF}
                wr_data <= {status_bits, 16'h0,             // hi = wire word 3
                            24'h0, 3'd0, 3'b000,            // lo = wire word 2
                            drop_latch, eolf_q};
                in_hdr    <= 1'b0;
                in_packet <= 1'b1;
                drop_latch<= 1'b0;
            end else if (in_packet && pair_v) begin
                wr_en   <= 1'b1;
                wr_data <= pair_d;
                if (pay_cnt == PAY_WORDS_M1) begin
                    in_packet <= 1'b0;
                    pay_cnt   <= 8'd0;
                    if (eolf_q) begin                        // frame done -> STATUS
                        in_status <= 1'b1;
                        stat_cnt  <= 8'd0;
                    end
                end else begin
                    pay_cnt <= pay_cnt + 8'd1;
                end
            end else if (in_status) begin
                wr_en <= 1'b1;
                // The STATUS packet is a full 162-word packet: the host parses
                // a 16-BYTE header (w0..w3) from EVERY packet, so it needs the
                // same two header words as a LINE packet.  (Writing only the
                // flags word here used to shift the whole packet up by one
                // word: the flags word landed on w0, w2 read back 0 ->
                // type=LINE, and the zero payload smeared over a frame row.)
                case (stat_cnt)
                    8'd0:    // w64[0]: lo = frame_idx, hi = {rsv 16'h0,
                             // line_idx = 16'hFFFF (STATUS marker)}
                        wr_data <= {16'h0000, 16'hFFFF, frame_idx};
                    8'd1: begin
                        // w64[1]: flags = {type=STATUS (3'd1), 3'b000,
                        // OVERRUN=0, EOLF=1} = 8'h21 (host ignores EOLF here)
                        wr_data <= {status_bits, 16'h0,      // hi = wire w3
                                    24'h0, 3'd1, 3'b000,     // lo = wire w2
                                    1'b0, 1'b1};
                    end
                    default: wr_data <= 64'h0;               // zero payload
                endcase
                if (stat_cnt == 8'd161) begin
                    in_status <= 1'b0;
                    stat_cnt  <= 8'd0;
                end else begin
                    stat_cnt <= stat_cnt + 8'd1;
                end
            end
        end
    end

endmodule
