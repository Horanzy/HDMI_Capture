// -----------------------------------------------------------------------------
// iddr_bus_in.v -- 60-bit dual-pixel DDR video input capture for IT68051 QD bus
//
// Function
//   Captures the 60 QD bits as a rising-edge word and a falling-edge word and
//   the three single-edge sync signals with ordinary flops.  All capture
//   registers are clocked by clk_vid (buffered RX_PCLK, same edge the IT68051
//   uses for data).
//
//   DDR CAPTURE STYLE -- RTL INFERENCE, NOT THE IDDR PRIMITIVE (read this
//   before "fixing" it back):
//     word_r (posedge) and word_f (negedge) are plain RTL register pairs on
//     pad-direct inputs; with (* syn_useioff = 1 *) / "Use IO Register" the
//     Gowin synthesizer places them in the IOLOGIC as DDR input registers
//     (the same hardware the IDDR primitive wraps).
//     HISTORY: an earlier version instantiated the bare IDDR primitive
//     directly (IDDR u_iddr (.Q0,.Q1,.D,.CLK)).  On the Education edition
//     V1.9.11 this primitive is SILENTLY DROPPED during synthesis: no error,
//     no warning beyond the collateral CV0016 flood ("Input A_B2 is unused"
//     x60), because word_r/word_f lose their drivers and the whole video
//     path is pruned backwards from the undriven nets.  Do NOT go back to
//     the bare primitive.  If the inferred form does not land in IOLOGIC
//     (see "post-synthesis check" below) or the 148.5 MHz input timing
//     fails, the sanctioned alternative is the IP Core Generator IDDR IP,
//     not the bare primitive.
//
//   Post-synthesis check: in the synthesis report the "I/O Register" usage
//   must jump (about +60 registers placed as IOLOGIC/IOB DDR input flops;
//   before the fix the project showed 43 "I/O Register as FF" and no IOLOGIC
//   input registers).  If word_r/word_f end up as ordinary fabric flops the
//   148.5 MHz source-synchronous capture margins will likely fail TA --
//   switch to the IDDR IP then.
//
// Port / timing assumptions
//   - qd_in[i] comes directly from an input pad (IBUF) - required for
//     IOLOGIC placement; do not insert fabric logic between pad and capture.
//   - clk_vid is the RX_PCLK ball buffered through DCE (PLL-free v1 flow).
//   - IT68051 dual-pixel DDR: each PCLK rising edge presents one 2-pixel word,
//     each falling edge the next 2-pixel word.  Datasheet setup/hold at the
//     rising edge: TPSRd=1.22ns / TPSFd=1.0ns, at the falling edge
//     TPHRd=1.42ns / TPHFd=0.84ns  -> margins are ~1ns, no phase shift used.
//   - word_r = word latched at the rising edge,  word_f = word latched at the
//     falling edge (both presented after the rising edge, per UG304 4.2.1
//     block diagram).  If bring-up shows swapped halves, exchange the two
//     always blocks below.
//   - de_in/hs_in/vs_in are single-edge (rising-edge aligned) IT68051 outputs,
//     setup 2.88ns / hold 1.88ns -> captured with one flop at the same edge.
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
    output wire        de_s,         // syncs sampled at the rising edge
    output wire        hs_s,
    output wire        vs_s
);

    // ---- DDR capture, inferred into IOLOGIC (see header before changing) ----
    reg [59:0] word_r_q;
    reg [59:0] word_f_q;
    always @(posedge clk_vid) word_r_q <= qd_in;   // rising-edge word
    always @(negedge clk_vid) word_f_q <= qd_in;   // falling-edge word
    assign word_r = word_r_q;
    assign word_f = word_f_q;

    // ---- single-edge syncs ----
    reg de_r, hs_r, vs_r;
    always @(posedge clk_vid or negedge rstn) begin
        if (!rstn) begin
            de_r <= 1'b0;
            hs_r <= 1'b0;
            vs_r <= 1'b0;
        end else begin
            de_r <= de_in;
            hs_r <= hs_in;
            vs_r <= vs_in;
        end
    end

    assign de_s = de_r;
    assign hs_s = hs_r;
    assign vs_s = vs_r;

endmodule
