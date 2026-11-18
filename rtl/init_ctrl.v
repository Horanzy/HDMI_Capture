// -----------------------------------------------------------------------------
// init_ctrl.v -- IT68051 boot, init-table replay, EDID upload, stream
//                supervision, HPD (FT601 clock domain, 100 MHz)
//
// Sequence:
//   M_POR      : 20 ms fabric settle; FT601_RESET_N released (its POR is the
//                external 10k + 0.1uF RC, FPGA keeps it released)
//   M_RST_LO   : 68051_SYSRSTN low 10 ms (IT68051 hardware reset input)
//   M_RST_WAIT : SYSRSTN high, 100 ms boot delay
//   M_FETCH    : load next ROM op; OP_END -> EDID phase (EN_EDID) or M_TBLDONE
//   M_RUN      : execute op over i2c_master_od
//                WRITE  = bank switch + read-modify-write  (firmware hdmirxset)
//                WRITEF = bank switch + plain write        (firmware hdmirxwr)
//                DELAY ms, POLL (200 ms timeout then continue), END
//                Device address: write 0x90 / read 0x91 (7-bit 0x48;
//                PCADR pulled low on this board: R8(0R, pull-up to
//                OVDD33) is DNP / not populated, R6(10k) pulls PCADR
//                to GND -> address 0x90 per IT68051 datasheet Table 4-3)
//   EDID phase (after the init table, BEFORE video wait / HPD -- same order
//                as the EVB: iTE6805_Init_fsm() replays INIT_HDMI_TABLE at
//                iTE6805_DRV.c:2528 then calls iTE6805_EDID_Init() at :2562):
//     driven from edid_init script ROM (opwords 5=EDWR / 6=EDRD), mirroring
//     iTE6805_EDID.c: enable EDID RAM access (bank0 0x4B=0xA9), write the
//     254 payload bytes to EDID SRAM slave 0xA8 (checksum offsets 127/255
//     not written, HW takes them from regs 0xC9/0xCA), program 0xC9/0xC6/
//     0xC7/0xC8/0xCA for both banks, enable internal EDID (0xC5[0]=0) and
//     pulse 0xC5[4] with 1 ms per bank, then (EN_EDID_RB) read back at
//     0xA9 and compare.  HPD is only asserted later in M_WAIT->M_ONLINE,
//     so the EDID is always complete before the source sees HPD (the EVB
//     holds HPD low during the SRAM write, EDID.c:74-79).
//     EDRD read-back mismatch counts into ack_cnt (status [11:8]); an
//     address NACK on 0xA9 is not an error (readability not guaranteed by
//     EVB source) and jumps straight to the script END.
//   M_TBLDONE  : init_done (EDID included), 10 ms settle, then assert HPD
//   M_WAIT     : HPD stays asserted while the source reads EDID and starts
//                video; stream_en opens only after stable video
//   M_ONLINE   : HPD remains asserted.  Supervision: loss of stream_ok for
//                STREAM_LOST_MS, or host SOFT_RESET, releases HPD and replays
//                the full init + EDID (M_RST_LO).
//   M_LOST/M_ERR : cooldown, then full re-init.  HPD is asserted as soon as
//                the fresh EDID is complete, not gated on a video signal that
//                a normal HDMI source cannot produce before HPD.
//
// Fix vs v1: DELAY ops no longer issue a stray I2C START (which made the
// engine complete after ~10 us instead of the programmed ms -- the EDID
// 0xC5[4] shadow-reset pulse and the CAOF delays need real milliseconds).
//
// status_bits layout (word 3 of every packet, frozen):
//   [15:12] init_err_cnt   [11:8] i2c_ack_err_cnt (+ EDID read-back mismatches
//          seen during the EDID phase)
//   [7] stream_ok [6] htot_ok [5] vid_stable [4] init_done
//   [3] hpd_active [2] online [1] i2c_fail  [0] replay_active (table or EDID)
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module init_ctrl #(
    parameter AUTO_FORMAT       = 0,   // 0 = fixed YUV422; 1 = INT/AVI automatic format
    parameter STREAM_STABLE_MS = 100,
    parameter STREAM_LOST_MS   = 50,
    parameter POLL_TIMEOUT_MS  = 250,
    parameter EN_EDID          = 1,    // 0 = v1 behaviour (no EDID upload)
    parameter EN_EDID_RB       = 1,    // 0 = skip the EDID read-back words
    parameter EDID_SCR_DEPTH   = 528   // must match edid_init SCRIPT_DEPTH
) (
    input  wire        clk_usb,
    input  wire        rstn,

    // i2c_master_od interface
    output reg  [2:0]  i2c_cmd,
    output reg         i2c_ack_mst,
    output reg  [7:0]  i2c_tx,
    output reg         i2c_wr,
    input  wire        i2c_busy,
    input  wire        i2c_done,
    input  wire [7:0]  i2c_rx,
    input  wire        i2c_ack_err,
    input  wire        i2c_stuck,

    // init table ROM
    output reg  [10:0] rom_raddr,
    input  wire [31:0] rom_rdata,

    // EDID phase script ROM + EDID byte ROM (edid_init)
    output reg  [10:0] scr_raddr,
    input  wire [31:0] scr_rdata,
    output wire [7:0]  edid_baddr,
    input  wire [7:0]  edid_bdata,

    // video telemetry (synchronized into this domain upstream)
    input  wire        vid_act_edge,
    input  wire        vid_htot_ok,
    input  wire        vid_stable,
    input  wire        vid_int_event,      // synchronized falling edge of IT68051 INT/SCDT

    // outputs
    output reg         sysrstn_n,
    output reg         hpd_assert,       // 1 = drive HPD pad LOW (source valid)
    output reg         ft601_rstn_n,
    output reg         stream_en,
    output reg  [2:0]  pixel_fmt,          // 0 raw422, 3 RGB10, 4 YUV44410; fixed top uses 5 packed422
    output reg  [11:0] crop_x0,            // measured-timing centre, updated at video service
    output reg  [11:0] crop_y0,
    output reg  [15:0] status_bits,
    input  wire        cmd_soft_rst_req,
    output reg         cmd_soft_rst
);

    localparam [7:0] DEV_W = 8'h90, DEV_R = 8'h91, REG_BANK = 8'h0F;
    localparam [7:0] DEV_EDID_W = 8'hA8, DEV_EDID_R = 8'hA9;  // EDID SRAM slave
    localparam [3:0] OP_WRITE = 4'h0, OP_WRITEF = 4'h1, OP_DELAY = 4'h2,
                     OP_POLL  = 4'h3, OP_END    = 4'h4,
                     OP_EDWR  = 4'h5, OP_EDRD   = 4'h6, OP_READ = 4'h7;
    localparam [7:0] STABLE_MS = STREAM_STABLE_MS[7:0];
    localparam [7:0] LOST_MS   = STREAM_LOST_MS[7:0];
    localparam [7:0] POLL_MS   = POLL_TIMEOUT_MS[7:0];
    localparam [10:0] SCR_END  = EDID_SCR_DEPTH - 1;   // script END word index

    // ------------------------------------------------------------ timers
    // 'tms' is owned exclusively by the main FSM block below (cleared at state
    // transitions, advanced once per millisecond there).  This tick generator
    // only counts tcnt and produces tick_ms -- driving tms from BOTH blocks is
    // a multi-driver net (Gowin EX1999/EX2000, real synthesis error).
    reg [16:0] tcnt;
    reg [7:0]  tms;
    wire       tick_ms = (tcnt == 17'd99_999);
    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn)        tcnt <= 17'd0;
        else if (tick_ms) tcnt <= 17'd0;
        else              tcnt <= tcnt + 17'd1;
    end

    // ---------------------------------------------- stream activity monitor
    reg [1:0]  act_d;
    wire       act_edge = act_d[1] ^ act_d[0];
    reg [16:0] alive_cnt;
    reg        alive;
    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            act_d <= 2'b00; alive_cnt <= 17'd0; alive <= 1'b0;
        end else begin
            act_d <= {act_d[0], vid_act_edge};
            if (act_edge) begin
                alive_cnt <= 17'd0; alive <= 1'b1;
            end else if (alive_cnt == 17'd99_999) begin
                alive_cnt <= 17'd0; alive <= 1'b0;
            end else alive_cnt <= alive_cnt + 17'd1;
        end
    end
    wire stream_ok = alive & vid_htot_ok & vid_stable;

    // -------------------------------------------------------- state encoding
    localparam [3:0]
        M_POR = 4'd0, M_RST_LO = 4'd1, M_RST_WAIT = 4'd2, M_FETCH = 4'd3,
        M_RUN = 4'd4, M_NEXT   = 4'd5, M_TBLDONE  = 4'd6, M_WAIT  = 4'd7,
        M_ONLINE = 4'd8, M_LOST = 4'd9, M_ERR = 4'd10, M_DECODE = 4'd11,
        M_SVC_START = 4'd12;

    // byte steps for WRITE/WRITEF/POLL ops
    localparam [4:0]
        T_START  = 5'd0,  T_AW     = 5'd1,  T_BREG  = 5'd2,  T_BVAL  = 5'd3,
        T_BSTOP  = 5'd4,  T_START2 = 5'd5,  T_AW2   = 5'd6,  T_RREG  = 5'd7,
        T_BRANCH = 5'd8,  T_START3 = 5'd9,  T_AR    = 5'd10, T_RX    = 5'd11,
        T_RSTOP  = 5'd12, T_START4 = 5'd13, T_AW3   = 5'd14, T_RREG2 = 5'd15,
        T_VAL    = 5'd16, T_VSTOP2 = 5'd17;

    reg [3:0]  main_st;
    reg [4:0]  step;
    reg [31:0] opword;
    reg [3:0]  opcode;
    reg [11:0] opaddr;
    reg [7:0]  opmask, opval;
    reg [7:0]  rd_val;
    reg [3:0]  ack_cnt, init_err_cnt;
    reg        i2c_fail, init_done;
    reg [7:0]  stable_ms, lost_ms;
    reg        soft_seen;
    reg        edid_phase;                  // executing edid_init script ROM
    reg        rb_jmp;                      // EDRD NACK -> jump to script END
    // Video service reads AVI DB1/2/3/4, HActive and VActive before applying
    // output format and crop.  Phases 0..7 are reads, 8..10 configure 6E/6B/6C,
    // 11 writes the selected CSC table, 12..14 clear INT; phase 14 completes.
    reg [3:0]  service_phase;
    reg [2:0]  det_fmt;
    reg [7:0]  det_out;
    reg [1:0]  det_csc;
    reg [1:0]  det_input_fmt;
    reg [1:0]  det_colorimetry;
    reg [2:0]  det_ext_colorimetry;
    reg [1:0]  det_rgb_range;
    reg [7:0]  det_hlo, det_vlo;
    reg [14:0] det_width;
    (* syn_preserve = 1 *) reg [4:0] csc_index;
    (* syn_preserve = 1 *) reg [1:0] csc_sel;
    (* keep = 1 *) wire [7:0] csc_data;
    reg        det_crop_valid;
    reg        det_changed, svc_from_online, detect_done;

    csc_matrix_rom u_csc_rom (.matrix_sel(csc_sel), .byte_index(csc_index),
                              .byte_data(csc_data));

    function [2:0] fmt_from_input;
        input [1:0] c;
        begin
            if (!AUTO_FORMAT)
                fmt_from_input = 3'd0; // logical fixed-422; top maps this to packed fmt 5
            else begin
                case (c)
                    2'b00: fmt_from_input = 3'd3; // RGB10
                    2'b01: fmt_from_input = 3'd4; // YUV44410
                    default: fmt_from_input = 3'd0; // YUV422/YUV420 -> raw422 slots
                endcase
            end
        end
    endfunction
    function [7:0] out_from_input;
        input [1:0] c;
        begin
            if (!AUTO_FORMAT)
                out_from_input = 8'h05; // fixed 10-bit YUV422 for every input
            else begin
                case (c)
                    2'b00: out_from_input = 8'h01; // RGB, 10-bit
                    2'b01: out_from_input = 8'h09; // YUV444, 10-bit
                    default: out_from_input = 8'h05; // YUV422, 10-bit
                endcase
            end
        end
    endfunction

    wire [7:0] op_bank = opaddr[11:8];
    wire [7:0] op_reg  = opaddr[7:0];
    wire       plain_w = (opcode == OP_WRITEF) || (opmask == 8'hFF);
    wire [7:0] wr_val  = (service_phase == 4'd11) ? csc_data :
                         (plain_w ? opval : ((rd_val & ~opmask) | opval));
    wire       poll_hit = ((rd_val & opmask) == opval);
    wire       edid_op  = (opcode == OP_EDWR) || (opcode == OP_EDRD);
    wire [31:0] opw     = edid_phase ? scr_rdata : rom_rdata;
    wire [14:0] meas_height = {1'b0, i2c_rx[5:0], det_vlo};
    wire [14:0] crop_x_calc = (det_width - 15'd640) >> 1;
    wire [14:0] crop_y_calc = (meas_height - 15'd640) >> 1;

    // EDID byte ROM address = byte index carried in opaddr[7:0]
    assign edid_baddr = opaddr[7:0];

    // combinational I2C command for the current step
    reg       iss_wr;
    reg [2:0] iss_cmd;
    reg [7:0] iss_tx;
    always @(*) begin
        iss_wr  = 1'b0;
        iss_cmd = 3'h0;
        iss_tx  = 8'h0;
        // DELAY must not issue any bus traffic (its ms timer drives M_NEXT)
        if (main_st == M_RUN && opcode != OP_DELAY) begin
            case (step)
                T_START:  begin iss_cmd = 3'h1;                               iss_wr = 1'b1; end
                T_AW:     begin iss_cmd = 3'h2; iss_tx = edid_op ? DEV_EDID_W : DEV_W;    iss_wr = 1'b1; end
                T_BREG:   begin iss_cmd = 3'h2; iss_tx = edid_op ? opaddr[7:0] : REG_BANK; iss_wr = 1'b1; end
                T_BVAL:   begin iss_cmd = 3'h2; iss_tx = (opcode == OP_EDWR) ? edid_bdata : op_bank; iss_wr = 1'b1; end
                T_BSTOP:  begin iss_cmd = 3'h4;                               iss_wr = 1'b1; end
                T_START2: begin iss_cmd = 3'h1;                               iss_wr = 1'b1; end
                T_AW2:    begin iss_cmd = 3'h2; iss_tx = (opcode == OP_EDRD) ? DEV_EDID_R : DEV_W; iss_wr = 1'b1; end
                T_RREG:   begin iss_cmd = 3'h2; iss_tx = op_reg;              iss_wr = 1'b1; end
                T_BRANCH: begin iss_cmd = 3'h4;                               iss_wr = 1'b1; end
                T_START3: begin iss_cmd = 3'h1;                               iss_wr = 1'b1; end
                T_AR:     begin iss_cmd = 3'h2; iss_tx = DEV_R;               iss_wr = 1'b1; end
                T_RX:     begin iss_cmd = 3'h3;                               iss_wr = 1'b1; end
                T_RSTOP:  begin iss_cmd = 3'h4;                               iss_wr = 1'b1; end
                T_START4: begin iss_cmd = 3'h1;                               iss_wr = 1'b1; end
                T_AW3:    begin iss_cmd = 3'h2; iss_tx = DEV_W;               iss_wr = 1'b1; end
                T_RREG2:  begin iss_cmd = 3'h2; iss_tx = op_reg;              iss_wr = 1'b1; end
                T_VAL:    begin iss_cmd = 3'h2; iss_tx = wr_val;              iss_wr = 1'b1; end
                T_VSTOP2: begin iss_cmd = 3'h4;                               iss_wr = 1'b1; end
                default:  begin iss_wr = 1'b0; end
            endcase
        end
    end

    always @(posedge clk_usb or negedge rstn) begin
        if (!rstn) begin
            main_st <= M_POR; step <= T_START;
            opword <= 32'h0; opcode <= 4'h0; opaddr <= 12'h0;
            opmask <= 8'h0; opval <= 8'h0; rd_val <= 8'h0;
            ack_cnt <= 4'd0; init_err_cnt <= 4'd0; i2c_fail <= 1'b0;
            init_done <= 1'b0;
            stable_ms <= 8'd0; lost_ms <= 8'd0; soft_seen <= 1'b0;
            i2c_cmd <= 3'h0; i2c_ack_mst <= 1'b0; i2c_tx <= 8'h0; i2c_wr <= 1'b0;
            rom_raddr <= 11'd0;
            scr_raddr <= 11'd0; edid_phase <= 1'b0; rb_jmp <= 1'b0;
            sysrstn_n <= 1'b0; hpd_assert <= 1'b0; ft601_rstn_n <= 1'b0;
            stream_en <= 1'b0; status_bits <= 16'h0; cmd_soft_rst <= 1'b0;
            pixel_fmt <= 3'd0;
            crop_x0 <= 12'd960; crop_y0 <= 12'd400;
            service_phase <= 4'd0; det_fmt <= 3'd0; det_out <= 8'h05; det_csc <= 2'd0;
            det_input_fmt <= 2'd0; det_colorimetry <= 2'd0;
            det_ext_colorimetry <= 3'd0; det_rgb_range <= 2'd0;
            det_hlo <= 8'd0; det_vlo <= 8'd0; det_width <= 15'd0;
            csc_index <= 5'd0; csc_sel <= 2'd2;
            det_crop_valid <= 1'b0;
            det_changed <= 1'b0; svc_from_online <= 1'b0; detect_done <= 1'b0;
            tms <= 8'd0;
        end else begin
            i2c_wr       <= 1'b0;
            cmd_soft_rst <= 1'b0;

            // issue command when the engine is idle (1-cycle turnaround)
            if (iss_wr && !i2c_busy && !i2c_wr) begin
                i2c_cmd     <= iss_cmd;
                i2c_ack_mst <= 1'b0;
                i2c_tx      <= iss_tx;
                i2c_wr      <= 1'b1;
            end

            // advance the FSM millisecond timer (this block owns tms; state
            // code below may clear it in the same cycle -- last assignment
            // wins in this always block, so transition clears keep priority)
            if (tick_ms)
                tms <= tms + 8'd1;

            case (main_st)
                M_POR: begin
                    sysrstn_n    <= 1'b0;
                    hpd_assert   <= 1'b0;
                    ft601_rstn_n <= 1'b1;
                    stream_en    <= 1'b0;
                    init_done    <= 1'b0;
                    if (tms == 8'd20) begin tms <= 8'd0; main_st <= M_RST_LO; end
                end

                M_RST_LO: begin
                    sysrstn_n  <= 1'b0;
                    hpd_assert <= 1'b0;
                    stream_en  <= 1'b0;
                    pixel_fmt <= 3'd7; // sentinel: force first 0x6B/0x6C configuration
                    detect_done <= 1'b0;
                    service_phase <= 4'd0;
                    if (tms == 8'd10) begin tms <= 8'd0; main_st <= M_RST_WAIT; end
                end

                M_RST_WAIT: begin
                    sysrstn_n <= 1'b1;
                    if (tms == 8'd100) begin
                        tms        <= 8'd0;
                        rom_raddr  <= 11'd0;
                        scr_raddr  <= 11'd0;
                        edid_phase <= 1'b0;
                        rb_jmp     <= 1'b0;
                        ack_cnt    <= 4'd0;
                        i2c_fail   <= 1'b0;
                        main_st    <= M_FETCH;
                    end
                end

                // wait one cycle for the ROM registered read (raddr set in
                // M_NEXT / M_RST_WAIT), then latch the op
                M_FETCH: begin
                    main_st <= M_DECODE;
                end
                M_DECODE: begin
                    opword <= opw;
                    opcode <= opw[31:28];
                    opaddr <= opw[27:16];
                    opmask <= opw[15:8];
                    opval  <= opw[7:0];
                    tms    <= 8'd0;                 // fresh timer per op
                    rd_val <= 8'h0;
                    if (opw[31:28] == OP_END) begin
                        if (edid_phase) begin
                            edid_phase <= 1'b0;     // EDID script finished
                            main_st    <= M_TBLDONE;
                        end else if (EN_EDID) begin
                            edid_phase <= 1'b1;     // switch to EDID script
                            scr_raddr  <= 11'd0;
                            main_st    <= M_FETCH;  // wait for script word 0
                        end else begin
                            main_st <= M_TBLDONE;   // v1: no EDID phase
                        end
                    end else begin
                        step    <= T_START;
                        main_st <= M_RUN;
                    end
                end

                M_RUN: begin
                    if (i2c_done) begin
                        // an address NACK on 0xA9 during EDRD is soft (EDID
                        // RAM readability is not guaranteed by EVB source)
                        if (i2c_ack_err &&
                            !((opcode == OP_EDRD) && (step == T_AW2)))
                            ack_cnt <= ack_cnt + 4'd1;
                        case (opcode)
                            OP_WRITE, OP_WRITEF: begin
                                case (step)
                                    T_BVAL:   step <= plain_w ? T_START2 : T_BRANCH;
                                    T_RREG:   step <= plain_w ? T_VAL    : T_START3;
                                    T_VAL:    step <= T_VSTOP2;
                                    T_VSTOP2: begin
                                        if (service_phase == 4'd8) begin
                                            pixel_fmt <= det_fmt;
                                            stream_en <= 1'b0;
                                            service_phase <= 4'd9;
                                            opcode <= OP_WRITE;
                                            opaddr <= 12'h06B;
                                            opmask <= 8'h0F; opval <= det_out;
                                            step <= T_START;
                                        end else if (service_phase == 4'd9) begin
                                            service_phase <= 4'd10;
                                            opcode <= OP_WRITE;
                                            opaddr <= 12'h06C;
                                            opmask <= 8'h03; opval <= {6'd0, det_csc};
                                            step <= T_START;
                                        end else if (service_phase == 4'd10) begin
                                            if (det_csc == 2'd2) begin
                                                service_phase <= 4'd11;
                                                csc_index <= 5'd0;
                                                opcode <= OP_WRITEF;
                                                opaddr <= 12'h170;
                                                opmask <= 8'hFF; opval <= csc_data;
                                            end else begin
                                                service_phase <= 4'd12;
                                                opcode <= OP_WRITEF;
                                                opaddr <= 12'h012;
                                                opmask <= 8'hFF; opval <= 8'hFF;
                                            end
                                            step <= T_START;
                                        end else if (service_phase == 4'd11) begin
                                            if (csc_index == 5'd21) begin
                                                service_phase <= 4'd12;
                                                opcode <= OP_WRITEF;
                                                opaddr <= 12'h012;
                                            end else begin
                                                csc_index <= csc_index + 5'd1;
                                                opaddr <= opaddr + 12'd1;
                                            end
                                            opmask <= 8'hFF;
                                            opval <= (csc_index == 5'd21) ? 8'hFF : csc_data;
                                            step <= T_START;
                                        end else if (service_phase == 4'd12) begin
                                            service_phase <= 4'd13;
                                            opcode <= OP_WRITEF;
                                            opaddr <= 12'h011;
                                            opmask <= 8'hFF; opval <= 8'hFF;
                                            step <= T_START;
                                        end else if (service_phase == 4'd13) begin
                                            service_phase <= 4'd14;
                                            opcode <= OP_WRITEF;
                                            opaddr <= 12'h010;
                                            opmask <= 8'hFF; opval <= 8'hFF;
                                            step <= T_START;
                                        end else if (service_phase == 4'd14) begin
                                            service_phase <= 4'd0;
                                            detect_done <= 1'b1;
                                            stable_ms <= 8'd0;
                                            lost_ms <= 8'd0;
                                            tms <= 8'd0;
                                            if (svc_from_online && !det_changed && det_crop_valid) begin
                                                stream_en <= 1'b1;
                                                main_st <= M_ONLINE;
                                            end else begin
                                                main_st <= M_WAIT;
                                            end
                                        end else begin
                                            main_st <= M_NEXT;
                                        end
                                    end
                                    T_RX:     begin rd_val <= i2c_rx; step <= T_RSTOP; end
                                    T_RSTOP:  step <= T_START4;
                                    default:  step <= step + 5'd1;
                                endcase
                            end
                            OP_EDWR: begin
                                // START, 0xA8, byte index, data, STOP
                                case (step)
                                    T_BVAL:   step <= T_BSTOP;
                                    T_BSTOP:  main_st <= M_NEXT;
                                    default:  step <= step + 5'd1;
                                endcase
                            end
                            OP_EDRD: begin
                                // set pointer: START, 0xA8, index, STOP
                                // read: START2, 0xA9, RX, STOP, compare
                                // NOTE: the walk SKIPS T_BVAL (T_BREG -> T_BSTOP
                                // directly).  T_BVAL would TX one extra byte to
                                // the EDID SRAM slave and (a) corrupt the
                                // just-written EDID RAM byte, (b) advance the
                                // slave's pointer so the 0xA9 read returns
                                // idx+1 -- every compare then mismatches,
                                // ack_cnt hits 8 and the FSM falls into M_ERR
                                // forever (HPD never asserts).
                                if (!EN_EDID_RB) begin
                                    main_st <= M_NEXT;   // compile-time off
                                end else begin
                                    case (step)
                                        T_BREG:  step <= T_BSTOP;  // no data byte
                                        T_BSTOP: step <= T_START2;
                                        T_AW2:   step <= T_RX;      // 0xA9 -> one data byte, no main-chip register phase
                                        T_RX:    begin rd_val <= i2c_rx; step <= T_RSTOP; end
                                        T_RSTOP: begin
                                            if (i2c_ack_err) begin
                                                rb_jmp  <= 1'b1;  // not readable
                                                main_st <= M_NEXT;
                                            end else begin
                                                if (rd_val != edid_bdata)
                                                    ack_cnt <= ack_cnt + 4'd1;
                                                main_st <= M_NEXT;
                                            end
                                        end
                                        default: step <= step + 5'd1;
                                    endcase
                                end
                            end
                            OP_READ: begin
                                // Banked AVI/timing reads used by video service.
                                case (step)
                                    T_RX: begin
                                        rd_val <= i2c_rx;
                                        case (service_phase)
                                            4'd0: begin // bank2 AVI DB1: Y1/Y0 color format
                                                det_input_fmt <= i2c_rx[6:5];
                                                det_fmt <= fmt_from_input(i2c_rx[6:5]);
                                                det_out <= out_from_input(i2c_rx[6:5]);
                                                det_csc <= (!AUTO_FORMAT && i2c_rx[6:5] == 2'b00) ? 2'd2 : 2'd0;
                                            end
                                            4'd1: det_colorimetry <= i2c_rx[7:6]; // AVI DB2
                                            4'd2: begin // AVI DB3: extended colorimetry + RGB range
                                                det_ext_colorimetry <= i2c_rx[6:4];
                                                det_rgb_range <= i2c_rx[3:2];
                                            end
                                            4'd3: begin // AVI DB4 VIC resolves RGB "depends on VIC"
                                                if (det_rgb_range == 2'd0)
                                                    det_rgb_range <= (i2c_rx[6:0] >= 7'd2) ? 2'd1 : 2'd2;
                                            end
                                            4'd4: det_hlo <= i2c_rx; // bank0 0x9D
                                            4'd5: begin // bank0 0x9E; YUV420 reports half width
                                                det_width <= {1'b0, i2c_rx[5:0], det_hlo} <<
                                                             (det_input_fmt == 2'b11);
                                            end
                                            4'd6: det_vlo <= i2c_rx; // bank0 0xA4
                                            4'd7: begin // bank0 0xA5
                                                crop_x0 <= crop_x_calc[11:0];
                                                crop_y0 <= crop_y_calc[11:0];
                                                det_crop_valid <= (det_width >= 15'd644) &&
                                                    (meas_height >= 15'd644) &&
                                                    !det_width[0] && !det_vlo[0];
                                                det_changed <= (det_fmt != pixel_fmt) ||
                                                    ({3'd0, crop_x0} != crop_x_calc) ||
                                                    ({3'd0, crop_y0} != crop_y_calc);
                                                csc_sel <= {(det_colorimetry == 2'b10),
                                                            (det_rgb_range == 2'd2)};
                                            end
                                            default: ;
                                        endcase
                                        step <= T_RSTOP;
                                    end
                                    T_RSTOP: begin
                                        if (service_phase < 4'd7) begin
                                            service_phase <= service_phase + 4'd1;
                                            opcode <= OP_READ;
                                            case (service_phase)
                                                4'd0: opaddr <= 12'h216; // AVI DB2
                                                4'd1: opaddr <= 12'h217; // AVI DB3
                                                4'd2: opaddr <= 12'h218; // AVI DB4 / VIC
                                                4'd3: opaddr <= 12'h09D; // HActive low
                                                4'd4: opaddr <= 12'h09E; // HActive high
                                                4'd5: opaddr <= 12'h0A4; // VActive low
                                                default: opaddr <= 12'h0A5; // VActive high
                                            endcase
                                            opmask <= 8'h00; opval <= 8'h00;
                                        end else begin
                                            service_phase <= 4'd8;
                                            opcode <= OP_WRITE;
                                            opaddr <= 12'h06E;
                                            opmask <= 8'h80; opval <= 8'h00; // manual CSC enable
                                        end
                                        step <= T_START;
                                    end
                                    default: step <= step + 5'd1;
                                endcase
                            end
                            OP_POLL: begin
                                case (step)
                                    T_RX:     begin rd_val <= i2c_rx; step <= T_RSTOP; end
                                    T_RSTOP:  begin
                                        if (poll_hit || tms >= POLL_MS)
                                            main_st <= M_NEXT;      // hit or timeout
                                        else
                                            step <= T_START2;       // read again
                                    end
                                    default: step <= step + 5'd1;
                                endcase
                            end
                            default: main_st <= M_NEXT;             // (DELAY here too)
                        endcase
                    end
                    // DELAY op runs purely on the ms timer (no bus traffic:
                    // the issuer skips OP_DELAY, see the iss_* block)
                    if (opcode == OP_DELAY && !i2c_busy) begin
                        if (tms >= opword[23:8]) begin
                            tms     <= 8'd0;
                            main_st <= M_NEXT;
                        end
                    end
                    // hard errors abort the replay into the error state
                    if (i2c_stuck || (ack_cnt == 4'd8)) begin
                        i2c_fail <= 1'b1;
                        init_err_cnt <= init_err_cnt + 4'd1;
                        main_st  <= M_ERR;
                    end
                end

                M_NEXT: begin
                    if (rb_jmp) begin
                        scr_raddr <= SCR_END;       // jump to script END word
                        rb_jmp    <= 1'b0;
                    end else if (edid_phase) begin
                        scr_raddr <= scr_raddr + 11'd1;
                    end else begin
                        rom_raddr <= rom_raddr + 11'd1;
                    end
                    main_st   <= M_FETCH;
                end

                M_TBLDONE: begin
                    init_done <= 1'b1;
                    if (tms >= 8'd10) begin
                        tms <= 8'd0; stable_ms <= 8'd0; lost_ms <= 8'd0;
                        hpd_assert <= 1'b1;       // EDID is ready: let the source start video
                        detect_done <= 1'b0;
                        main_st <= M_WAIT;
                    end
                end

                M_WAIT: begin
                    if (cmd_soft_rst_req) begin
                        main_st <= M_RST_LO;
                    end else if (vid_int_event && detect_done) begin
                        svc_from_online <= 1'b0;
                        main_st <= M_SVC_START;
                    end else if (stream_ok && !detect_done) begin
                        svc_from_online <= 1'b0;
                        main_st <= M_SVC_START;
                    end else if (stream_ok && det_crop_valid) begin
                        if (tick_ms) stable_ms <= stable_ms + 8'd1;
                        if (stable_ms >= STABLE_MS) begin
                            stable_ms  <= 8'd0;
                            stream_en  <= 1'b1;
                            lost_ms    <= 8'd0;
                            main_st    <= M_ONLINE;
                        end
                    end else begin
                        stable_ms <= 8'd0;
                    end
                end

                M_ONLINE: begin
                    if (cmd_soft_rst_req) begin
                        hpd_assert <= 1'b0;
                        stream_en  <= 1'b0;
                        main_st    <= M_RST_LO;
                    end else if (vid_int_event) begin
                        stream_en <= 1'b0; // block transitional packets before inspecting AVI
                        svc_from_online <= 1'b1;
                        main_st <= M_SVC_START;
                    end else if (!stream_ok) begin
                        stream_en <= 1'b0; // no transitional/garbage lines while loss is evaluated
                        if (tick_ms) lost_ms <= lost_ms + 8'd1;
                        if (lost_ms >= LOST_MS) begin
                            lost_ms    <= 8'd0;
                            hpd_assert <= 1'b0;
                            stream_en  <= 1'b0;
                            main_st    <= M_LOST;
                        end
                    end else begin
                        lost_ms <= 8'd0;
                        stream_en <= 1'b1; // short telemetry glitches recover without re-init
                    end
                end

                M_SVC_START: begin
                    service_phase <= 4'd0;
                    opcode <= OP_READ;
                    opaddr <= 12'h215; // bank2 AVI DB1[6:5]: actual input color format
                    opmask <= 8'h00;
                    opval <= 8'h00;
                    rd_val <= 8'h00;
                    step <= T_START;
                    main_st <= M_RUN;
                end

                M_LOST: begin
                    if (tms >= 8'd100) begin
                        tms <= 8'd0; main_st <= M_RST_LO;   // full re-init
                    end
                end

                M_ERR: begin
                    hpd_assert <= 1'b0;
                    stream_en  <= 1'b0;
                    if (tms >= 8'd200) begin
                        tms <= 8'd0; main_st <= M_RST_LO;   // retry after 200 ms
                    end
                end

                default: main_st <= M_POR;
            endcase

            if (cmd_soft_rst_req) begin
                cmd_soft_rst <= 1'b1;
                soft_seen    <= 1'b1;
            end

            status_bits <= {init_err_cnt, ack_cnt,
                            stream_ok, vid_htot_ok, vid_stable, init_done,
                            hpd_assert, (main_st == M_ONLINE), i2c_fail,
                            (main_st == M_RUN)};
        end
    end

endmodule
