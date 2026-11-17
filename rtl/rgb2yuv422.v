// -----------------------------------------------------------------------------
// rgb2yuv422.v -- RGB888 -> YUV422 (UYVY) pixel-pair converter, pipelined
//
// Function
//   Converts one horizontal pixel PAIR (two RGB 8-bit pixels, p0 = left,
//   p1 = right) into four UYVY bytes packed in one 32-bit word:
//       word[7:0]   = Cb        (word[7:0] leaves the FT601 bus first)
//       word[15:8]  = Y0
//       word[23:16] = Cr
//       word[31:24] = Y1
//   Chroma (Cb/Cr) is computed from the sum of the two pixels (the standard
//   4:2:2 co-sited average), i.e. scaled by 2 and shifted right by 9.
//
// Latency / throughput
//   3 clk cycles latency, 1 pair per clock sustained.  Two instances run in
//   parallel (one per DDR word) inside video_stream.
//
// Bit mapping
//   The IT68051 10-bit channels are truncated to the top 8 bits (RGB8 =
//   chan[9:2]) by the caller, so this module sees plain 8-bit full-range RGB.
//
// Coefficients (integer, same matrices as iTE6805_CSC_Table.h)
//   BT.601 (BT709=0):  Y = ( 66R+129G+ 25B+128)>>8 + 16
//                      Cb = (-38R- 74G+112B+256)>>9 + 128   (on R+G+B sums)
//                      Cr = (112R- 94G- 18B+256)>>9 + 128
//   BT.709 (BT709=1):  Y = ( 47R+157G+ 16B+128)>>8 + 16
//                      Cb = (-26R- 87G+113B+256)>>9 + 128
//                      Cr = (113R- 94G- 18B+256)>>9 + 128
//   With full-range 8-bit inputs the results are mathematically inside
//   Y[16,235] / CbCr[15,240]; a final saturating clamp is applied anyway.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module rgb2yuv422 #(
    parameter BT709 = 0                    // 0 = BT.601, 1 = BT.709
) (
    input  wire        clk,
    input  wire        rstn,
    input  wire        in_valid,
    input  wire [7:0]  p0_r, input wire [7:0] p0_g, input wire [7:0] p0_b,
    input  wire [7:0]  p1_r, input wire [7:0] p1_g, input wire [7:0] p1_b,
    output reg         out_valid,
    output reg  [31:0] out_uyvy
);

    // ---- coefficients (all positive here; signs applied in the sums)
    localparam signed [8:0] CY_R = BT709 ? 9'sd47  : 9'sd66;
    localparam signed [8:0] CY_G = BT709 ? 9'sd157 : 9'sd129;
    localparam signed [8:0] CY_B = BT709 ? 9'sd16  : 9'sd25;
    localparam signed [8:0] CB_R = BT709 ? 9'sd26  : 9'sd38;   // negated
    localparam signed [8:0] CB_G = BT709 ? 9'sd87  : 9'sd74;   // negated
    localparam signed [8:0] CB_B = BT709 ? 9'sd113 : 9'sd112;
    localparam signed [8:0] CR_R = BT709 ? 9'sd113 : 9'sd112;
    localparam signed [8:0] CR_G = BT709 ? 9'sd94  : 9'sd94;   // negated
    localparam signed [8:0] CR_B = BT709 ? 9'sd18  : 9'sd18;   // negated

    // ---- stage 0: latch inputs and pixel-pair sums
    reg        v1;
    reg [7:0]  a_r, a_g, a_b;          // pixel 0 (Y0)
    reg [7:0]  b_r, b_g, b_b;          // pixel 1 (Y1)
    reg [8:0]  s_r, s_g, s_b;          // p0+p1 sums (2x average)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            v1 <= 1'b0;
        end else begin
            v1  <= in_valid;
            a_r <= p0_r; a_g <= p0_g; a_b <= p0_b;
            b_r <= p1_r; b_g <= p1_g; b_b <= p1_b;
            s_r <= {1'b0, p0_r} + {1'b0, p1_r};
            s_g <= {1'b0, p0_g} + {1'b0, p1_g};
            s_b <= {1'b0, p0_b} + {1'b0, p1_b};
        end
    end

    // ---- stage 1: products (positive constants x unsigned data)
    function signed [18:0] m9x9;
        input signed [8:0]  k;         // constant 0..160
        input        [8:0]  d;         // unsigned 8 or 9 bit data, zero-extended
        begin
            m9x9 = k * $signed({1'b0, d});
        end
    endfunction

    reg        v2;
    reg signed [17:0] y0_p, y1_p;                       // Y sums (pre-shift)
    reg signed [19:0] cb_p, cr_p;                       // chroma sums (pre-shift)
    reg signed [17:0] y0_b, y1_b;                       // bias register (+128)
    reg signed [19:0] ch_b;                             // chroma bias (+256)
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            v2 <= 1'b0;
        end else begin
            v2 <= v1;
            y0_p <= m9x9(CY_R, {1'b0, a_r}) + m9x9(CY_G, {1'b0, a_g})
                  + m9x9(CY_B, {1'b0, a_b});
            y1_p <= m9x9(CY_R, {1'b0, b_r}) + m9x9(CY_G, {1'b0, b_g})
                  + m9x9(CY_B, {1'b0, b_b});
            cb_p <= m9x9(CB_B, {1'b0, s_b}) - m9x9(CB_R, {1'b0, s_r})
                  - m9x9(CB_G, {1'b0, s_g});
            cr_p <= m9x9(CR_R, {1'b0, s_r}) - m9x9(CR_G, {1'b0, s_g})
                  - m9x9(CR_B, {1'b0, s_b});
            y0_b <= 18'sd128;
            y1_b <= 18'sd128;
            ch_b <= 20'sd256;
        end
    end

    // ---- stage 2: shift, offset, clamp, pack
    function [7:0] clamp8;
        input signed [19:0] v;
        begin
            if      (v < 20'sd0)   clamp8 = 8'h00;
            else if (v > 20'sd255) clamp8 = 8'hFF;
            else                   clamp8 = v[7:0];
        end
    endfunction

    wire signed [19:0] y0s = (y0_p + y0_b) >>> 8;   // [0..219]
    wire signed [19:0] y1s = (y1_p + y1_b) >>> 8;
    wire signed [19:0] cbs = (cb_p + ch_b) >>> 9;
    wire signed [19:0] crs = (cr_p + ch_b) >>> 9;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            out_valid <= 1'b0;
            out_uyvy  <= 32'h80108010;
        end else begin
            out_valid <= v2;
            out_uyvy  <= {clamp8(y1s + 20'sd16), clamp8(crs + 20'sd128),
                          clamp8(y0s + 20'sd16), clamp8(cbs + 20'sd128)};
        end
    end

endmodule
