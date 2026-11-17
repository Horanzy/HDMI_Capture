// -----------------------------------------------------------------------------
// it68051_capture_top.v -- USB HDMI capture card, Gowin GW5A-LV25UG324 (U1)
//
// Data flow
//   IT68051 (U5) dual-pixel DDR TTL out, PCLK=148.5 MHz @2560x1440 CVT-RB:
//     QD[59:0] via 60x inferred DDR input registers (IOLOGIC; iddr_bus_in.v
//     header documents why the bare IDDR primitive must not be used)
//     -> word_r/word_f (2 pixels each)
//     -> video_stream: crop 640x640 (x0/y0 default 960/400, in-band settable)
//        + RGB888->YUV422 (BT.601/709, UYVY) + 16B header + 1280B payload
//     -> async FIFO (2048x64 BSRAM, gray pointers, 148.5M <-> 100M)
//     -> ft601_master: 245-sync-FIFO bursts into FT601Q (1 word/clk, 400MB/s)
//   FT601 packs USB SuperSpeed Bulk IN (MPS 1024, bMaxBurst 15 per FT601
//   configuration); a 1296B line packet written as one continuous bus
//   transaction leaves as a 1024B + a 272B short USB packet.
//
// Control flow
//   init_ctrl (USB domain): reset IT68051 -> I2C replay of the EVB init table
//   (ROM) -> EDID upload (custom 2560x1440@144 EDID from edid_init.v written
//   into the IT68051 EDID SRAM at I2C slave 0xA8, mirroring the EVB
//   iTE6805_EDID.c flow; the whole phase completes before HPD ever asserts)
//   -> wait for stable video -> HPD assert (pad LOW).  Stream loss or host
//   SOFT_RESET -> HPD release + full re-init incl. fresh EDID.  Host->device
//   commands (SET_CROP / SOFT_RESET / GET_STATUS) arrive through the FT601
//   OUT FIFO (ft601_cmd_rx, parameter-gated; EN_CMD_PATH=0 restores the
//   frozen "read direction unused" behaviour).
//
// Port names follow the V1.4 netlist net names at U1 and match
// constr/capture.cst 1:1 (114 ports; JTAG balls are device-dedicated and
// intentionally absent).  The five digit-leading netlist nets 68051_* are
// renamed it68051_* in the RTL because Verilog identifiers cannot start
// with a digit; the CST comments carry the netlist net names.
//
// Pin notes (review/final_bank_table_full.csv, cross-checked vs V1.4.tel):
//   RX_PCLK   M10 = IOB26A/GCLKT_9B  (dedicated global clock input,
//                                     buffered to GCLK via DCE, UG306 3.1)
//   FT601_CLK H4  = IOT1A/GCLKT_15   (same, DCE)
//   HPD polarity: pad LOW -> Q3 off -> HDMI HPD high (source sees valid).
//   I2C device address 0x90/0x91, 7-bit 0x48 (PCADR low: R8 0R is DNP,
//   R6 10k pulls PCADR to GND).
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
(* syn_useioff = 1 *)
module it68051_capture_top #(
    parameter BT709       = 0,   // RGB->YUV matrix: 0=BT.601, 1=BT.709
    parameter EN_CMD_PATH = 1,   // 0 = frozen v1: OE_N/RD_N high, no commands
    parameter FIFO_LOG2   = 11   // line FIFO words = 2**FIFO_LOG2 (16 KiB ~ 12.6 line packets)
) (
    // ---- IT68051 video output (dual-pixel DDR) ---------------------------
    input  wire RX_PCLK,
    input  wire RX_VSYNC,
    input  wire RX_HSYNC,
    input  wire RX_DE,
    input  wire A_B2,  input wire A_B3,  input wire A_B4,  input wire A_B5,
    input  wire A_B6,  input wire A_B7,  input wire A_B8,  input wire A_B9,
    input  wire A_B10, input wire A_B11,
    input  wire A_G2,  input wire A_G3,  input wire A_G4,  input wire A_G5,
    input  wire A_G6,  input wire A_G7,  input wire A_G8,  input wire A_G9,
    input  wire A_G10, input wire A_G11,
    input  wire A_R2,  input wire A_R3,  input wire A_R4,  input wire A_R5,
    input  wire A_R6,  input wire A_R7,  input wire A_R8,  input wire A_R9,
    input  wire A_R10, input wire A_R11,
    input  wire B_B2,  input wire B_B3,  input wire B_B4,  input wire B_B5,
    input  wire B_B6,  input wire B_B7,  input wire B_B8,  input wire B_B9,
    input  wire B_B10, input wire B_B11,
    input  wire B_G2,  input wire B_G3,  input wire B_G4,  input wire B_G5,
    input  wire B_G6,  input wire B_G7,  input wire B_G8,  input wire B_G9,
    input  wire B_G10, input wire B_G11,
    input  wire B_R2,  input wire B_R3,  input wire B_R4,  input wire B_R5,
    input  wire B_R6,  input wire B_R7,  input wire B_R8,  input wire B_R9,
    input  wire B_R10, input wire B_R11,

    // ---- IT68051 control --------------------------------------------------
    output wire it68051_hpd,               // LOW = HDMI HPD asserted (valid)
    input  wire it68051_int,               // INT#/SCDT (unused in v1, kept for bring-up)
    inout  wire it68051_pcscl,             // I2C SCL, open-drain
    inout  wire it68051_pcsda,             // I2C SDA, open-drain
    output wire it68051_sysrstn,           // IT68051 reset (active low)

    // ---- FT601Q 245-sync-FIFO --------------------------------------------
    input  wire FT601_CLK,               // 100 MHz from FT601 (always on)
    inout  wire FT601_DATA_0,  inout wire FT601_DATA_1,
    inout  wire FT601_DATA_2,  inout wire FT601_DATA_3,
    inout  wire FT601_DATA_4,  inout wire FT601_DATA_5,
    inout  wire FT601_DATA_6,  inout wire FT601_DATA_7,
    inout  wire FT601_DATA_8,  inout wire FT601_DATA_9,
    inout  wire FT601_DATA_10, inout wire FT601_DATA_11,
    inout  wire FT601_DATA_12, inout wire FT601_DATA_13,
    inout  wire FT601_DATA_14, inout wire FT601_DATA_15,
    inout  wire FT601_DATA_16, inout wire FT601_DATA_17,
    inout  wire FT601_DATA_18, inout wire FT601_DATA_19,
    inout  wire FT601_DATA_20, inout wire FT601_DATA_21,
    inout  wire FT601_DATA_22, inout wire FT601_DATA_23,
    inout  wire FT601_DATA_24, inout wire FT601_DATA_25,
    inout  wire FT601_DATA_26, inout wire FT601_DATA_27,
    inout  wire FT601_DATA_28, inout wire FT601_DATA_29,
    inout  wire FT601_DATA_30, inout wire FT601_DATA_31,
    inout  wire FT601_BE_0, inout wire FT601_BE_1,
    inout  wire FT601_BE_2, inout wire FT601_BE_3,
    input  wire FT601_TXE_N,             // low = TX FIFO has space
    output wire FT601_WR_N,
    input  wire FT601_RXF_N,             // low = OUT FIFO has data (cmd path)
    output wire FT601_RD_N,
    output wire FT601_OE_N,
    input  wire FT601_SIWU_N,            // reserved (ext 10k pullup), unused
    output wire FT601_RESET_N,
    input  wire FT601_WAKEUP_N           // USB suspend flag, unused in v1
);

    // ===================================================== clock buffering
    // GW5A has NO BUFG primitive (zero references in UG306/DS1103; bare
    // instantiation fails with EX3937 on Education edition).  The documented
    // GCLK buffer is DCE (UG306 3.1, Table 3-1): ports CLKIN/CE/CLKOUT,
    // CE active-HIGH ("时钟使能信号，高电平有效") -> tie 1'b1 for always-on.
    // DCE output drives the GCLK global network directly.
    wire clk_vid;                          // ~148.5 MHz (2K144 CVT-RB)
    wire clk_usb;                          // 100 MHz from FT601
    DCE u_buf_vid (.CLKIN(RX_PCLK),   .CE(1'b1), .CLKOUT(clk_vid));
    DCE u_buf_usb (.CLKIN(FT601_CLK), .CE(1'b1), .CLKOUT(clk_usb));

    // ===================================================== reset generators
    (* keep *) reg [11:0] por_vid = 12'hFFF;
    (* keep *) reg [11:0] por_usb = 12'hFFF;
    wire vid_rstn = (por_vid == 12'h0);
    wire usb_rstn = (por_usb == 12'h0);
    always @(posedge clk_vid) if (!vid_rstn) por_vid <= por_vid - 12'd1;
    always @(posedge clk_usb) if (!usb_rstn) por_usb <= por_usb - 12'd1;

    // ===================================================== video input bus
    wire [59:0] qd_raw = {
        B_R11, B_R10, B_R9, B_R8, B_R7, B_R6, B_R5, B_R4, B_R3, B_R2,  // QD[59:50]
        B_G11, B_G10, B_G9, B_G8, B_G7, B_G6, B_G5, B_G4, B_G3, B_G2,  // QD[49:40]
        B_B11, B_B10, B_B9, B_B8, B_B7, B_B6, B_B5, B_B4, B_B3, B_B2,  // QD[39:30]
        A_R11, A_R10, A_R9, A_R8, A_R7, A_R6, A_R5, A_R4, A_R3, A_R2,  // QD[29:20]
        A_G11, A_G10, A_G9, A_G8, A_G7, A_G6, A_G5, A_G4, A_G3, A_G2,  // QD[19:10]
        A_B11, A_B10, A_B9, A_B8, A_B7, A_B6, A_B5, A_B4, A_B3, A_B2}; // QD[9:0]

    wire [59:0] word_r, word_f;
    wire de_s, hs_s, vs_s;

    // ===================================================== video telemetry
    // hs_tick_v / htot_ok_v / stable_v feed the USB-domain supervision.
    // (frame_tick_v / htot_pclk_v / de_pclk_v removed: dangling telemetry
    // with no consumer anywhere -- video_stream no longer exports them.)
    wire hs_tick_v, htot_ok_v, stable_v;

    // ===================================================== CDC signals
    reg  [1:0] sync_en_q;
    reg  [1:0] s_x0 [0:11];
    reg  [1:0] s_y0 [0:11];
    reg  [1:0] s_st [0:15];
    wire       stream_en_v;
    wire [11:0] crop_x0_v, crop_y0_v;
    wire [15:0] status_v;

    wire       stream_en;                 // USB-domain stream gate (init_ctrl)
    wire [11:0] crop_x0, crop_y0;         // USB-domain crop regs (cmd_rx)
    wire [15:0] status_bits;              // USB-domain status (init_ctrl)
    wire [7:0] cmd_last;
    wire       cmd_soft_rst_req;
    wire       cmd_soft_rst;

    reg [7:0]  act_div;
    reg        act_tog;
    (* keep *) reg [1:0] act_q;
    reg  [1:0] htot_q, stab_q;
    wire       vid_act_edge;
    wire       htot_ok_u, stab_u;

    // ===================================================== line FIFO wires
    wire        fifo_wr_en, fifo_wr_full;
    wire [11:0] fifo_free;
    wire [63:0] fifo_wr_data;
    wire        fifo_rd_en, fifo_rd_valid, fifo_rd_empty;
    wire [63:0] fifo_rd_data;

    // ===================================================== FT601 bus wires
    wire [31:0] wr_o_data;
    wire [3:0]  wr_o_be;
    wire        wr_o_wr_n;
    wire [31:0] data_bus_in;
    wire        cmd_oe_n, cmd_rd_n, cmd_wr, cmd_active;

    // ===================================================== I2C wires
    wire        scl_drv_n, sda_drv_n;
    wire        i2c_busy, i2c_done, i2c_ack_err, i2c_stuck;
    wire [7:0]  i2c_rx, i2c_tx;
    wire [2:0]  i2c_cmd;
    wire        i2c_ack_mst, i2c_wr;
    wire [10:0] rom_raddr;
    wire [31:0] rom_rdata;
    wire        sysrstn_n, hpd_assert, ft601_rstn_n;

    // EDID phase (init_ctrl <-> edid_init script/byte ROMs)
    wire [10:0] scr_raddr;
    wire [31:0] scr_rdata;
    wire [7:0]  edid_baddr, edid_bdata;

    // ===================================================== clock-domain CDC
    // USB -> video: quasi-static registers, 2FF sync, latched at vs_rise
    integer si;
    always @(posedge clk_vid or negedge vid_rstn) begin
        if (!vid_rstn) begin
            sync_en_q <= 2'b00;
            for (si = 0; si < 12; si = si + 1) begin
                s_x0[si] <= 2'b00; s_y0[si] <= 2'b00;
            end
            for (si = 0; si < 16; si = si + 1) s_st[si] <= 2'b00;
        end else begin
            sync_en_q <= {sync_en_q[0], stream_en};
            for (si = 0; si < 12; si = si + 1) begin
                s_x0[si] <= {s_x0[si][0], crop_x0[si]};
                s_y0[si] <= {s_y0[si][0], crop_y0[si]};
            end
            for (si = 0; si < 16; si = si + 1)
                s_st[si] <= {s_st[si][0], status_bits[si]};
        end
    end
    assign stream_en_v = sync_en_q[1];
    genvar gi;
    generate
        for (gi = 0; gi < 12; gi = gi + 1) begin : g_crop
            assign crop_x0_v[gi] = s_x0[gi][1];
            assign crop_y0_v[gi] = s_y0[gi][1];
        end
        for (gi = 0; gi < 16; gi = gi + 1) begin : g_st
            assign status_v[gi] = s_st[gi][1];
        end
    endgenerate

    // video -> USB: activity toggle + level telemetry
    always @(posedge clk_vid or negedge vid_rstn) begin
        if (!vid_rstn) begin
            act_div <= 8'd0; act_tog <= 1'b0;
        end else if (hs_tick_v) begin
            act_div <= act_div + 8'd1;
            if (act_div == 8'd15) begin
                act_div <= 8'd0;
                act_tog <= ~act_tog;         // toggle every 16 source lines
            end
        end
    end
    always @(posedge clk_usb or negedge usb_rstn) begin
        if (!usb_rstn) begin
            act_q <= 2'b00; htot_q <= 2'b00; stab_q <= 2'b00;
        end else begin
            act_q  <= {act_q[0],  act_tog};
            htot_q <= {htot_q[0], htot_ok_v};
            stab_q <= {stab_q[0], stable_v};
        end
    end
    assign vid_act_edge = act_q[1] ^ act_q[0];
    assign htot_ok_u    = htot_q[1];
    assign stab_u       = stab_q[1];

    // ===================================================== video capture
    iddr_bus_in u_iddr (
        .rstn    (vid_rstn),
        .clk_vid (clk_vid),
        .qd_in   (qd_raw),
        .de_in   (RX_DE),
        .hs_in   (RX_HSYNC),
        .vs_in   (RX_VSYNC),
        .word_r  (word_r),
        .word_f  (word_f),
        .de_s    (de_s),
        .hs_s    (hs_s),
        .vs_s    (vs_s)
    );

    video_stream #(.BT709(BT709)) u_vstream (
        .clk_vid      (clk_vid),
        .rstn         (vid_rstn),
        .word_r       (word_r),
        .word_f       (word_f),
        .de_s         (de_s),
        .hs_s         (hs_s),
        .vs_s         (vs_s),
        .stream_en    (stream_en_v),
        .crop_x0      (crop_x0_v),
        .crop_y0      (crop_y0_v),
        .status_bits  (status_v),
        .hs_tick      (hs_tick_v),
        .htot_ok      (htot_ok_v),
        .stream_stable(stable_v),
        .wr_en        (fifo_wr_en),
        .wr_data      (fifo_wr_data),
        .wr_full      (fifo_wr_full),
        .wr_free_words(fifo_free)
    );

    async_fifo_dc #(.DW(64), .DEPTH_LOG2(FIFO_LOG2)) u_linefifo (
        .wr_clk       (clk_vid),
        .wr_rstn      (vid_rstn),
        .wr_en        (fifo_wr_en),
        .wr_data      (fifo_wr_data),
        .wr_full      (fifo_wr_full),
        .wr_free_words(fifo_free),
        .rd_clk       (clk_usb),
        .rd_rstn      (usb_rstn),
        .rd_en        (fifo_rd_en),
        .rd_data      (fifo_rd_data),
        .rd_valid     (fifo_rd_valid),
        .rd_empty     (fifo_rd_empty)
    );

    // ===================================================== FT601
    ft601_master u_ft601 (
        .clk_usb      (clk_usb),
        .rstn         (usb_rstn),
        .fifo_rd_en   (fifo_rd_en),
        .fifo_rd_valid(fifo_rd_valid),
        .fifo_rd_data (fifo_rd_data),
        .fifo_rd_empty(fifo_rd_empty),
        .txe_n_pad    (FT601_TXE_N),
        .cmd_inhibit  (cmd_active),
        .o_data       (wr_o_data),
        .o_be         (wr_o_be),
        .o_wr_n       (wr_o_wr_n)
    );

    ft601_cmd_rx #(.EN_CMD_PATH(EN_CMD_PATH)) u_cmd (
        .clk_usb         (clk_usb),
        .rstn            (usb_rstn),
        .rxf_n_pad       (FT601_RXF_N),
        .data_in_pad     (data_bus_in),
        .oe_n            (cmd_oe_n),
        .rd_n            (cmd_rd_n),
        .cmd_wr          (cmd_wr),
        .cmd_x0          (crop_x0),
        .cmd_y0          (crop_y0),
        .cmd_soft_rst_req(cmd_soft_rst_req),
        .cmd_last        (cmd_last),
        .cmd_active      (cmd_active)
    );

    // DATA/BE direction: FPGA drives while writing; releases only while the
    // cmd path reads (EN_CMD_PATH=0 -> cmd_active always 0 -> always driven).
    assign data_bus_in = {
        FT601_DATA_31, FT601_DATA_30, FT601_DATA_29, FT601_DATA_28,
        FT601_DATA_27, FT601_DATA_26, FT601_DATA_25, FT601_DATA_24,
        FT601_DATA_23, FT601_DATA_22, FT601_DATA_21, FT601_DATA_20,
        FT601_DATA_19, FT601_DATA_18, FT601_DATA_17, FT601_DATA_16,
        FT601_DATA_15, FT601_DATA_14, FT601_DATA_13, FT601_DATA_12,
        FT601_DATA_11, FT601_DATA_10, FT601_DATA_9,  FT601_DATA_8,
        FT601_DATA_7,  FT601_DATA_6,  FT601_DATA_5,  FT601_DATA_4,
        FT601_DATA_3,  FT601_DATA_2,  FT601_DATA_1,  FT601_DATA_0 };

    assign FT601_DATA_0  = cmd_active ? 1'bz : wr_o_data[0];
    assign FT601_DATA_1  = cmd_active ? 1'bz : wr_o_data[1];
    assign FT601_DATA_2  = cmd_active ? 1'bz : wr_o_data[2];
    assign FT601_DATA_3  = cmd_active ? 1'bz : wr_o_data[3];
    assign FT601_DATA_4  = cmd_active ? 1'bz : wr_o_data[4];
    assign FT601_DATA_5  = cmd_active ? 1'bz : wr_o_data[5];
    assign FT601_DATA_6  = cmd_active ? 1'bz : wr_o_data[6];
    assign FT601_DATA_7  = cmd_active ? 1'bz : wr_o_data[7];
    assign FT601_DATA_8  = cmd_active ? 1'bz : wr_o_data[8];
    assign FT601_DATA_9  = cmd_active ? 1'bz : wr_o_data[9];
    assign FT601_DATA_10 = cmd_active ? 1'bz : wr_o_data[10];
    assign FT601_DATA_11 = cmd_active ? 1'bz : wr_o_data[11];
    assign FT601_DATA_12 = cmd_active ? 1'bz : wr_o_data[12];
    assign FT601_DATA_13 = cmd_active ? 1'bz : wr_o_data[13];
    assign FT601_DATA_14 = cmd_active ? 1'bz : wr_o_data[14];
    assign FT601_DATA_15 = cmd_active ? 1'bz : wr_o_data[15];
    assign FT601_DATA_16 = cmd_active ? 1'bz : wr_o_data[16];
    assign FT601_DATA_17 = cmd_active ? 1'bz : wr_o_data[17];
    assign FT601_DATA_18 = cmd_active ? 1'bz : wr_o_data[18];
    assign FT601_DATA_19 = cmd_active ? 1'bz : wr_o_data[19];
    assign FT601_DATA_20 = cmd_active ? 1'bz : wr_o_data[20];
    assign FT601_DATA_21 = cmd_active ? 1'bz : wr_o_data[21];
    assign FT601_DATA_22 = cmd_active ? 1'bz : wr_o_data[22];
    assign FT601_DATA_23 = cmd_active ? 1'bz : wr_o_data[23];
    assign FT601_DATA_24 = cmd_active ? 1'bz : wr_o_data[24];
    assign FT601_DATA_25 = cmd_active ? 1'bz : wr_o_data[25];
    assign FT601_DATA_26 = cmd_active ? 1'bz : wr_o_data[26];
    assign FT601_DATA_27 = cmd_active ? 1'bz : wr_o_data[27];
    assign FT601_DATA_28 = cmd_active ? 1'bz : wr_o_data[28];
    assign FT601_DATA_29 = cmd_active ? 1'bz : wr_o_data[29];
    assign FT601_DATA_30 = cmd_active ? 1'bz : wr_o_data[30];
    assign FT601_DATA_31 = cmd_active ? 1'bz : wr_o_data[31];
    assign FT601_BE_0    = cmd_active ? 1'bz : wr_o_be[0];
    assign FT601_BE_1    = cmd_active ? 1'bz : wr_o_be[1];
    assign FT601_BE_2    = cmd_active ? 1'bz : wr_o_be[2];
    assign FT601_BE_3    = cmd_active ? 1'bz : wr_o_be[3];
    assign FT601_WR_N    = wr_o_wr_n;
    assign FT601_OE_N    = cmd_oe_n;
    assign FT601_RD_N    = cmd_rd_n;

    // ===================================================== I2C + supervision
    wire scl_in = it68051_pcscl;
    wire sda_in = it68051_pcsda;
    assign it68051_pcscl = scl_drv_n ? 1'bz : 1'b0;
    assign it68051_pcsda = sda_drv_n ? 1'bz : 1'b0;

    reg scl_q, sda_q, scl_qq, sda_qq;      // 2FF pad feedback sync
    always @(posedge clk_usb or negedge usb_rstn) begin
        if (!usb_rstn) begin
            scl_q <= 1'b1; sda_q <= 1'b1; scl_qq <= 1'b1; sda_qq <= 1'b1;
        end else begin
            scl_q <= scl_in;  scl_qq <= scl_q;
            sda_q <= sda_in;  sda_qq <= sda_q;
        end
    end

    i2c_master_od #(.CLK_DIV(125)) u_i2c (            // 200 kHz SCL
        .clk      (clk_usb),
        .rstn     (usb_rstn),
        .scl_drv_n(scl_drv_n),
        .sda_drv_n(sda_drv_n),
        .scl_in   (scl_qq),
        .sda_in   (sda_qq),
        .cmd      (i2c_cmd),
        .ack_mst  (i2c_ack_mst),
        .tx_byte  (i2c_tx),
        .cmd_wr   (i2c_wr),
        .busy     (i2c_busy),
        .done     (i2c_done),
        .rx_byte  (i2c_rx),
        .ack_err  (i2c_ack_err),
        .scl_stuck(i2c_stuck)
    );

    it68051_init_rom #(.DEPTH(191)) u_rom (
        .clk   (clk_usb),
        .raddr (rom_raddr),
        .rdata (rom_rdata)
    );

    // custom 2560x1440@144 EDID + IT68051 write-phase script (auto-generated
    // from fpga_fw/edid/edid_2k144.bin; SCRIPT_DEPTH must match the
    // EDID_SCR_DEPTH parameter of u_ctrl)
    edid_init #(.EDID_DEPTH(256), .SCRIPT_DEPTH(528)) u_edid (
        .clk      (clk_usb),
        .rstn     (usb_rstn),
        .scr_raddr(scr_raddr),
        .scr_rdata(scr_rdata),
        .byte_addr(edid_baddr),
        .byte_data(edid_bdata)
    );

    init_ctrl #(
        .EN_EDID        (1),
        .EN_EDID_RB     (1),
        .EDID_SCR_DEPTH (528)
    ) u_ctrl (
        .clk_usb         (clk_usb),
        .rstn            (usb_rstn),
        .i2c_cmd         (i2c_cmd),
        .i2c_ack_mst     (i2c_ack_mst),
        .i2c_tx          (i2c_tx),
        .i2c_wr          (i2c_wr),
        .i2c_busy        (i2c_busy),
        .i2c_done        (i2c_done),
        .i2c_rx          (i2c_rx),
        .i2c_ack_err     (i2c_ack_err),
        .i2c_stuck       (i2c_stuck),
        .rom_raddr       (rom_raddr),
        .rom_rdata       (rom_rdata),
        .scr_raddr       (scr_raddr),
        .scr_rdata       (scr_rdata),
        .edid_baddr      (edid_baddr),
        .edid_bdata      (edid_bdata),
        .vid_act_edge    (vid_act_edge),
        .vid_htot_ok     (htot_ok_u),
        .vid_stable      (stab_u),
        .sysrstn_n       (sysrstn_n),
        .hpd_assert      (hpd_assert),
        .ft601_rstn_n    (ft601_rstn_n),
        .stream_en       (stream_en),
        .status_bits     (status_bits),
        .cmd_soft_rst_req(cmd_soft_rst_req),
        .cmd_soft_rst    (cmd_soft_rst)
    );

    assign it68051_sysrstn = sysrstn_n;
    assign it68051_hpd     = hpd_assert ? 1'b0 : 1'b1;   // LOW = HPD valid
    assign FT601_RESET_N = ft601_rstn_n;

    // inputs present on the board but unused by v1 logic (kept for bring-up
    // probes; the ports stay so the .cst pin locations remain valid)
    (* keep *) reg unused_int, unused_siwu, unused_wakeup;
    always @(posedge clk_usb) begin
        unused_int    <= it68051_int;
        unused_siwu   <= FT601_SIWU_N;
        unused_wakeup <= FT601_WAKEUP_N;
    end

endmodule
