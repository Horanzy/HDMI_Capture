// -----------------------------------------------------------------------------
// i2c_master_od.v -- open-drain I2C master (bit engine), FT601 clock domain
//
// Function
//   Byte-level command engine for the IT68051 PCSCL/PCSDA programming port.
//   Open-drain: outputs drive '0' or release (external 10k pullups R10/R11
//   provide the high level, V1.4 netlist).  Clock stretching is tolerated
//   (bit phases wait while SCL is held low); a stretch longer than ~2.6 ms
//   raises scl_stuck for the caller.
//
// Rate: CLK_DIV=125 at clk_usb=100 MHz -> 200 kHz.  Up to 400 kHz allowed by
// the IT68051 datasheet (F/S mode).
//
// Protocol constants: START = SDA falls while SCL high; STOP = SDA rises
// while SCL high; data changes only while SCL low; data sampled while SCL
// high.  Byte = 8 data bits (MSB first) + 1 ACK bit.
//
// Device addressing: the caller supplies the full 8-bit address byte (R/W bit
// included).  This board: write 0x90 / read 0x91 (7-bit 0x48; PCADR low -
// the R8 0R pull-up is DNP, R6 10k pulls PCADR to GND, see README).
//
// Commands (one at a time; pulse cmd_wr for one clk when !busy):
//   3'h1 START (or repeated START)   3'h2 TX tx_byte
//   3'h3 RX (master ACKs if ack_mst) 3'h4 STOP
// done pulses one clk on completion; ack_err latched on slave NACK of a TX.
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module i2c_master_od #(
    parameter CLK_DIV = 125              // clk cycles per I2C quarter bit
) (
    input  wire       clk,
    input  wire       rstn,
    output wire       scl_drv_n,         // 1 = release (Z), 0 = drive low
    output wire       sda_drv_n,
    input  wire       scl_in,            // pad feedback (synchronized upstream)
    input  wire       sda_in,
    input  wire [2:0] cmd,
    input  wire       ack_mst,
    input  wire [7:0] tx_byte,
    input  wire       cmd_wr,
    output wire       busy,
    output reg        done,
    output reg  [7:0] rx_byte,
    output reg        ack_err,
    output reg        scl_stuck
);

    localparam CMD_START = 3'h1, CMD_TX = 3'h2, CMD_RX = 3'h3, CMD_STOP = 3'h4;

    localparam S_IDLE   = 4'd0, S_START1 = 4'd1, S_START2 = 4'd2,
               S_BIT_A  = 4'd3, S_BIT_B  = 4'd4, S_BIT_C  = 4'd5, S_BIT_D = 4'd6,
               S_STOP1  = 4'd7, S_STOP2  = 4'd8, S_STOP3  = 4'd9;

    reg [3:0]  st;
    reg [2:0]  cmd_q;
    reg [7:0]  sh;
    reg [3:0]  bitn;                       // 0..7 data bits, 8 = ACK bit
    reg        sda_do, scl_do;
    reg [9:0]  phase;
    reg [17:0] stretch_cnt;

    assign busy      = (st != S_IDLE);
    assign scl_drv_n = scl_do;
    assign sda_drv_n = sda_do;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            st <= S_IDLE; cmd_q <= 3'h0; sh <= 8'h0; bitn <= 4'd0;
            sda_do <= 1'b1; scl_do <= 1'b1;
            done <= 1'b0; rx_byte <= 8'h0; ack_err <= 1'b0;
            phase <= 10'd0; stretch_cnt <= 18'd0; scl_stuck <= 1'b0;
        end else begin
            done <= 1'b0;

            case (st)
                S_IDLE: begin
                    scl_do <= 1'b1; sda_do <= 1'b1;
                    if (cmd_wr && cmd != 3'h0) begin
                        cmd_q   <= cmd;
                        sh      <= tx_byte;
                        bitn    <= 4'd0;
                        phase   <= 10'd0;
                        ack_err <= 1'b0;
                        if      (cmd == CMD_START) st <= S_START1;
                        else if (cmd == CMD_TX
                              || cmd == CMD_RX)  st <= S_BIT_A;
                        else if (cmd == CMD_STOP)  st <= S_STOP1;
                    end
                end

                // -------- START: SDA falls while SCL released high
                S_START1: begin
                    scl_do <= 1'b1; sda_do <= 1'b1;
                    if (phase == 2*CLK_DIV - 1) begin
                        phase <= 10'd0; st <= S_START2;
                    end else phase <= phase + 10'd1;
                end
                S_START2: begin
                    sda_do <= 1'b0;                     // SDA low, SCL high
                    if (phase == 2*CLK_DIV - 1) begin
                        phase <= 10'd0;
                        scl_do <= 1'b0;                 // then SCL low
                        done   <= 1'b1;
                        st     <= S_IDLE;
                    end else phase <= phase + 10'd1;
                end

                // -------- bit engine (9 clocks: 8 data + ACK)
                S_BIT_A: begin                          // SCL low: set data
                    scl_do <= 1'b0;
                    if (phase == CLK_DIV - 1) begin
                        phase <= 10'd0;
                        st    <= S_BIT_B;
                        if (cmd_q == CMD_RX)
                            sda_do <= (bitn == 4'd8) ? ~ack_mst : 1'b1;
                        else
                            sda_do <= (bitn == 4'd8) ? 1'b1 : sh[7];
                    end else phase <= phase + 10'd1;
                end
                S_BIT_B: begin                          // SCL high (slave samples)
                    scl_do <= 1'b1;
                    if (!scl_in) begin                  // tolerate stretching
                        if (stretch_cnt == 18'h3FFFF) scl_stuck <= 1'b1;
                        else stretch_cnt <= stretch_cnt + 18'd1;
                    end else begin
                        stretch_cnt <= 18'd0;
                        if (phase == 2*CLK_DIV - 1) begin
                            phase <= 10'd0; st <= S_BIT_C;
                        end else phase <= phase + 10'd1;
                    end
                end
                S_BIT_C: begin                          // SCL high: master samples
                    scl_do <= 1'b1;
                    if (phase == 3*CLK_DIV - 1) begin
                        phase <= 10'd0; st <= S_BIT_D;
                        if (cmd_q == CMD_RX && bitn < 4'd8)
                            sh <= {sh[6:0], sda_in};
                        if (cmd_q == CMD_TX && bitn == 4'd8 && sda_in)
                            ack_err <= 1'b1;            // slave NACKed
                    end else phase <= phase + 10'd1;
                end
                S_BIT_D: begin                          // SCL low again
                    scl_do <= 1'b0;
                    if (phase == 4*CLK_DIV - 1) begin
                        phase <= 10'd0;
                        if (bitn == 4'd8) begin
                            done  <= 1'b1;
                            st    <= S_IDLE;
                            sda_do <= 1'b1;             // release SDA
                            if (cmd_q == CMD_RX) rx_byte <= sh;
                        end else begin
                            bitn <= bitn + 4'd1;
                            st   <= S_BIT_A;
                            if (cmd_q == CMD_TX)
                                sh <= {sh[6:0], 1'b0};  // MSB-first shift
                        end
                    end else phase <= phase + 10'd1;
                end

                // -------- STOP: SDA rises while SCL released high
                S_STOP1: begin
                    scl_do <= 1'b0; sda_do <= 1'b0;
                    if (phase == 2*CLK_DIV - 1) begin
                        phase <= 10'd0; st <= S_STOP2;
                    end else phase <= phase + 10'd1;
                end
                S_STOP2: begin
                    scl_do <= 1'b1;
                    if (phase == 3*CLK_DIV - 1) begin
                        phase <= 10'd0; st <= S_STOP3;
                    end else phase <= phase + 10'd1;
                end
                S_STOP3: begin
                    sda_do <= 1'b1;
                    if (phase == 4*CLK_DIV - 1) begin
                        phase <= 10'd0;
                        done  <= 1'b1;
                        st    <= S_IDLE;
                    end else phase <= phase + 10'd1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
