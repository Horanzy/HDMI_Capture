// -----------------------------------------------------------------------------
// ft601_cmd_rx.v -- host->device in-band command receiver (FT601 OUT FIFO)
//
// Rationale (docs/README.md "control plane"):
//   The FT601 is a dumb FIFO; USB EP0 vendor requests terminate inside the
//   FT601 and cannot reach the FPGA.  Host->device control therefore uses the
//   FT601 OUT (USB OUT) FIFO: the host bulk-writes 16-byte command packets,
//   the FPGA reads them here.  Compile-time parameter EN_CMD_PATH=0 disables
//   the whole module (OE_N/RD_N then stay high forever, matching the frozen
//   "read direction unused" hardware note; crop stays at its defaults).
//
// Command packet (16 bytes, little-endian on the wire):
//   byte0..3  'B','F','C','P'   (0x42 0x46 0x43 0x50)
//   byte4     cmd   : 2=SOFT_RESET; other opcodes are reserved/ignored
//   byte5     len   : 8
//   byte6..15 reserved (crop is automatic from measured IT68051 timing)
//
// Timing experiment: OE_N/RD_N are launched from posedge registers and pass
// through top-level output IODELAY.  C_WAIT preserves one full valid RD_N
// cycle before data_q sampling.  DATA direction still changes on negedge, so
// OE is enabled only after the FPGA has released the shared bus.  DATA from
// FT601 is captured at rising edges.  Command bandwidth is intentionally low.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module ft601_cmd_rx #(
    parameter EN_CMD_PATH = 1
) (
    input  wire        clk_usb,
    input  wire        rstn,
    // FT601 pads
    input  wire        rxf_n_pad,          // low = OUT FIFO has data
    input  wire [31:0] data_in_pad,        // FT601-driven DATA (captured here)
    output reg         oe_n,               // posedge-launched, output IODELAY at top
    output reg         rd_n,               // posedge-launched, output IODELAY at top
    // decoded command
    output reg         cmd_soft_rst_req,
    output wire        cmd_active,         // high while a read owns the bus
    output wire        cmd_inhibit         // also blocks the last write before read entry
);

    localparam [31:0] MAGIC = 32'h50434642;  // bytes 'B','F','C','P' on wire

    // ------------------------------------------------------- pad captures
    reg rxf_n_q;
    reg [31:0] data_q;
    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            rxf_n_q <= 1'b1;
            data_q  <= 32'h0;
        end else begin
            rxf_n_q <= rxf_n_pad;
            data_q  <= data_in_pad;              // IOB capture
        end
    end

    // ------------------------------------------------------- read sequence
    localparam [2:0] C_IDLE = 3'd0, C_LEAD = 3'd1, C_REQ = 3'd2,
                     C_SAMP = 3'd3, C_GAP = 3'd4, C_DONE = 3'd5,
                     C_RELEASE = 3'd6, C_WAIT = 3'd7;

    reg [2:0]  cst;
    reg [1:0]  wcnt;                          // 0..3 words of the packet
    reg [1:0]  lead;
    reg [4:0]  junk;                          // junk-word flush budget
    reg [31:0] w0, w1;
    reg        oe_req, rd_req;                // posedge decisions

    // Keep bus-direction control on the original state-only path.  A separate
    // inhibit stops the final posedge-launched write when a read is pending;
    // this avoids adding the rxf combinational path to the negedge tri-state FFs.
    assign cmd_active  = (cst != C_IDLE) & EN_CMD_PATH;
    assign cmd_inhibit = ((cst != C_IDLE) || ((cst == C_IDLE) && !rxf_n_q)) & EN_CMD_PATH;

    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            cst <= C_IDLE; wcnt <= 2'd0; lead <= 2'd0; junk <= 5'd0;
            w0 <= 32'h0; w1 <= 32'h0;
            oe_req <= 1'b0; rd_req <= 1'b0;
            cmd_soft_rst_req <= 1'b0;
        end else begin
            cmd_soft_rst_req <= 1'b0;
            if (!EN_CMD_PATH) begin
                cst <= C_IDLE;
            end else begin
                case (cst)
                    C_IDLE: begin
                        wcnt <= 2'd0; junk <= 5'd0;
                        if (!rxf_n_q) begin
                            oe_req <= 1'b1;       // release the bus to FT601
                            lead   <= 2'd0;
                            cst    <= C_LEAD;
                        end
                    end
                    C_LEAD: begin                 // 2-cycle OE turn-around
                        oe_req <= 1'b1;
                        lead   <= lead + 2'd1;
                        if (lead == 2'd1) begin
                            rd_req<= 1'b1;        // strobe next word
                            cst   <= C_REQ;
                        end
                    end
                    C_REQ: begin                  // registered RD_N launches next posedge
                        oe_req <= 1'b1;
                        rd_req <= 1'b0;
                        cst    <= C_WAIT;
                    end
                    C_WAIT: begin                 // RD_N is low before this sampling edge
                        oe_req <= 1'b1;
                        rd_req <= 1'b0;
                        cst    <= C_SAMP;
                    end
                    C_SAMP: begin                 // word valid in data_q now
                        oe_req <= 1'b1;
                        case (wcnt)
                            2'd0: w0 <= data_q;
                            2'd1: w1 <= data_q;
                            default: ; // drain the remaining two words
                        endcase
                        if (wcnt == 2'd3) begin
                            wcnt <= 2'd0;
                            cst  <= C_DONE;
                        end else begin
                            wcnt  <= wcnt + 2'd1;
                            rd_req<= 1'b1;        // strobe the next word
                            cst   <= C_REQ;
                        end
                    end
                    C_DONE: begin
                        oe_req <= 1'b0;           // release bus
                        if (w0 == MAGIC && w1[15:8] == 8'd8) begin
                            if (w1[7:0] == 8'd2)
                                cmd_soft_rst_req <= 1'b1;
                            cst <= C_RELEASE;
                        end else if (junk != 5'h1F) begin
                            junk <= junk + 5'd1;
                            lead <= 2'd1;         // short lead for continuation
                            cst  <= C_LEAD;       // keep draining junk
                        end else begin
                            cst <= C_RELEASE;
                        end
                    end
                    C_RELEASE: begin
                        // OE_N rises after this posedge through output delay.
                        // Keep DATA tri-stated for the extra cycle so the FT601
                        // stops driving before the FPGA resumes write ownership.
                        oe_req <= 1'b0;
                        rd_req <= 1'b0;
                        cst    <= C_IDLE;
                    end
                    default: cst <= C_IDLE;
                endcase
            end
        end
    end

    // Posedge launch + top-level output IODELAY.  C_WAIT compensates for the
    // registered output so data_q still samples a word driven by an active RD_N.
    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            oe_n <= 1'b1;
            rd_n <= 1'b1;
        end else begin
            oe_n <= !oe_req;
            rd_n <= !rd_req;
        end
    end

endmodule
