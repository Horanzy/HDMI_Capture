// -----------------------------------------------------------------------------
// ft601_master.v -- FT601Q 245-sync-FIFO master write engine (USB domain)
//
// Bus protocol (FT600/FT601 datasheet v1.05, sec 4.3/4.4 + AN_421):
//   - CLK is the FT601 output (100 MHz), everything synchronous to it.
//   - TXE_N low = at least 1 byte of space.  Space is only consumed by the
//     master (USB side drains), so "TXE_N low sampled at edge N" guarantees
//     acceptance of the word sampled at edge N+1 (canonical FTDI handshake).
//   - Master write: WR_N low + DATA[31:0] + BE[3:0] (active-high byte enables,
//     all four valid here - every packet is 1296 bytes = 324 whole words).
//   - AC timing Table 4.2: T3 master setup 1 ns before the sampling edge,
//     T4 master hold 4.8 ns after it.
//
// Timing architecture (the subtle part, see docs/README.md):
//   - TXE_N is captured with an IOB flop at the rising edge (T1=3.0/T2=3.5
//     around that edge give ample setup/hold).
//   - DATA/BE/WR_N are launched from IOB flops clocked on the FALLING edge:
//       setup : 5 ns - tCO >= 1 ns   -> tCO <= 4 ns  (check the STA report!)
//       hold  : next launch a full period later -> always satisfied
//     This sustains one 32-bit bus word per clock = 400 MB/s burst.
//
// Datapath: the async FIFO is 64-bit wide (one video word = falling + rising
// YUV pair).  This engine converts 64-bit FIFO words into two bus words
// (lo half first = earlier pixels):
//
//   FIFO --(pop at lo-emission / spl-empty)--> splitter (lo,hi phases)
//          --(bypass on hi-emission)--> back-to-back bus words
//   s0: one-word holding slot for an arrival that lands while the splitter
//       is busy (only happens across a TXE stall) - pop is gated by !s0_v.
//
// Sustained rate with TXE_N low and data available: 1 bus word / clock, no
// bubbles (pop at lo-emission, arrival bypasses into the freeing splitter).
//
// cmd_inhibit: while the (optional) host->device command path performs a read
// transaction on the shared DATA bus, writes pause (bus ownership).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module ft601_master (
    input  wire        clk_usb,          // = FT601_CLK pad (DCE-buffered GCLK)
    input  wire        rstn,
    // FIFO read port (async_fifo_dc semantics, 64-bit)
    output reg         fifo_rd_en,       // combinational pop request
    input  wire        fifo_rd_valid,    // 1 cycle after rd_en
    input  wire [63:0] fifo_rd_data,
    input  wire        fifo_rd_empty,
    // FT601 signals
    input  wire        txe_n_pad,        // captured with IOB flop here
    input  wire        cmd_inhibit,      // pause writes (cmd path owns bus)
    // negedge-launched bus outputs (to pads, via top-level tri-state mux)
    output reg  [31:0] o_data,
    output reg  [3:0]  o_be,
    output reg         o_wr_n
);

    // ---------------------------------------------------- TXE_N capture (IOB)
    reg txe_n_q;
    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) txe_n_q <= 1'b1;
        else       txe_n_q <= txe_n_pad;
    end

    // ---------------------------------------------------- splitter + s0 slot
    reg        spl_v;                     // splitter holds a word
    reg        spl_hi;                    // 0: lo next, 1: hi next
    reg [63:0] spl_d;
    reg        s0_v;                      // one-word holding slot
    reg [63:0] s0_d;

    wire       s1_v = fifo_rd_valid;      // arrival (1 cycle after pop)
    wire [63:0] s1_d = fifo_rd_data;

    wire       out_fire = !txe_n_q && !cmd_inhibit && spl_v;
    wire       hi_emit  = spl_v &&  spl_hi && out_fire;   // spl frees
    wire       lo_emit  = spl_v && !spl_hi && out_fire;   // spl stays
    wire [31:0] out_word = spl_hi ? spl_d[63:32] : spl_d[31:0];

    // arrival consumed directly by the splitter?
    wire direct_take = (hi_emit && !s0_v) || (!spl_v && s1_v);
    // arrival must be parked in s0 (spl busy)
    wire s0_capture  = s1_v && !direct_take;

    // combinational pop: the arrival (next cycle) must find a home:
    //  - pop at lo-emission: splitter frees at the arrival cycle and takes
    //    the arrival directly (bypass);
    //  - pop when splitter empty: arrival loads it;
    //  - never pop while s0 holds a word (its home is the busy splitter).
    always @(*) begin
        fifo_rd_en = !fifo_rd_empty && !s0_v && (lo_emit || !spl_v);
    end

    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            spl_v <= 1'b0; spl_hi <= 1'b0; spl_d <= 64'h0;
            s0_v  <= 1'b0; s0_d  <= 64'h0;
        end else begin
            // ---- splitter
            if (hi_emit) begin
                if (s0_v) begin
                    spl_d <= s0_d; spl_v <= 1'b1; spl_hi <= 1'b0;
                end else if (s1_v) begin
                    spl_d <= s1_d; spl_v <= 1'b1; spl_hi <= 1'b0;
                end else begin
                    spl_v <= 1'b0; spl_hi <= 1'b0;
                end
            end else if (lo_emit) begin
                spl_hi <= 1'b1;                      // data unchanged
            end else if (!spl_v && s1_v) begin
                spl_d <= s1_d; spl_v <= 1'b1; spl_hi <= 1'b0;
            end
            // ---- s0 slot
            if (hi_emit && s0_v)
                s0_v <= 1'b0;                        // drained into splitter
            if (s0_capture) begin
                s0_v <= 1'b1;
                s0_d <= s1_d;
            end
        end
    end

    // ------------------------------------- negedge output stage (IOB, pad side)
    // FSM decisions complete at the posedge; pad drivers register them at the
    // following negedge so pad data changes half a period before the FT601
    // sampling edge (header: T3/T4 analysis).
    always @(negedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            o_data <= 32'h0;
            o_be   <= 4'hF;
            o_wr_n <= 1'b1;
        end else begin
            o_data <= out_word;
            o_be   <= 4'hF;          // every line packet is word aligned
            o_wr_n <= !out_fire;
        end
    end

endmodule
