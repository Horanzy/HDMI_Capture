// -----------------------------------------------------------------------------
// iddr_bus_in.v -- 60-bit dual-pixel DDR video input capture for IT68051 QD bus
//
// Function
//   Captures the 60 QD bits as a rising-edge word and a falling-edge word and
//   the three single-edge sync signals with ordinary flops.  All capture
//   registers are clocked by clk_vid (buffered RX_PCLK, same edge the IT68051
//   uses for data).
//
//   DDR CAPTURE STYLE -- IOLOGIC IDDR PRIMITIVE.
//     UG304 4.2.1 documents direct IDDR instantiation and shows that Q0/Q1
//     are aligned at a common rising edge with two cycles of output latency.
//     The official DDR IP template in this Gowin installation instantiates
//     the same primitive.  The local one-bit iddr_probe project confirms that
//     Education V1.9.11.03 + GW5A-25A preserves and routes it as IOLOGIC.
//     Do not replace this with independent fabric posedge/negedge flops:
//     those were not packed into IOLOGIC and failed source-synchronous TA.
//
//   Post-synthesis/P&R check: resource usage must report 60 IDDR instances.
//   If it does not, stop and investigate rather than accepting a fabric-only
//   bitstream.  The 60 direct pad inputs must remain free of fabric logic.
//
// Port / timing assumptions
//   - qd_in[i] comes directly from an input pad (IBUF), as required by IDDR.
//   - clk_vid is RX_PCLK buffered through DCE (PLL-free v1 flow).
//   - IT68051 dual-pixel DDR: rising edge supplies Pixel0/1 and the following
//     falling edge supplies Pixel2/3.  word_r/word_f retain that order.
//   - de_in/hs_in/vs_in are single-edge aligned and are delayed to match IDDR.
//
// Reset: the DDR capture registers are deliberately NOT reset (a reset on a
//   negedge capture register prevents IOLOGIC placement, and the video path
//   is gated by stream_en/de anyway).  The sync flop trio de_r/hs_r/vs_r
//   keeps the async active-low reset.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
(* syn_useioff = 1 *)
module iddr_bus_in (
    input  wire        rstn,
    input  wire        clk_vid,      // DCE(RX_PCLK)
    // direct pad inputs (must NOT pass through fabric registers)
    input  wire [59:0] qd_in,        // {QD59..QD0}, netlist net order
    input  wire        de_in,
    input  wire        hs_in,
    input  wire        vs_in,
    // video-domain outputs, all valid in the same clk_vid cycle
    output wire [59:0] word_r,       // 2-pixel word captured at PCLK rising edge
    output wire [59:0] word_f,       // 2-pixel word captured at PCLK falling edge
    output wire        de_pre,
    output wire        de_s,         // syncs sampled at the rising edge
    output wire        hs_s,
    output wire        vs_s
);

    // IOLOGIC DDR capture.  UG304 4.2.1: Q0 is the rising-edge word and Q1
    // the falling-edge word; both are re-presented together at a rising edge
    // two PCLK cycles after the rising-edge sample.  A one-bit probe using
    // this exact Education version and GW5A-25A device passed synthesis/P&R
    // and reported IOLOGIC=1, IDDR=1 (see iddr_probe/ in this test folder).
    genvar i;
    generate
        for (i = 0; i < 60; i = i + 1) begin : g_iddr
            IDDR u_iddr (
                .D   (qd_in[i]),
                .CLK (clk_vid),
                .Q0  (word_r[i]),
                .Q1  (word_f[i])
            );
        end
    endgenerate

    // Match the two extra rising-edge cycles from the IDDR outputs.  The first
    // flop samples the pad; de_d3/hs_d3/vs_d3 are aligned with word_r/word_f.
    reg de_r, hs_r, vs_r;
    reg de_d2, hs_d2, vs_d2;
    reg de_d3, hs_d3, vs_d3;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            de_r <= 1'b0; hs_r <= 1'b0; vs_r <= 1'b0;
            de_d2 <= 1'b0; hs_d2 <= 1'b0; vs_d2 <= 1'b0;
            de_d3 <= 1'b0; hs_d3 <= 1'b0; vs_d3 <= 1'b0;
        end else begin
            de_r <= de_in; hs_r <= hs_in; vs_r <= vs_in;
            de_d2 <= de_r; hs_d2 <= hs_r; vs_d2 <= vs_r;
            de_d3 <= de_d2; hs_d3 <= hs_d2; vs_d3 <= vs_d2;
        end
    end

    assign de_pre = de_d2;
    assign de_s = de_d3;
    assign hs_s = hs_d3;
    assign vs_s = vs_d3;

endmodule
