# -----------------------------------------------------------------------------
# timing.sdc -- FULL timing constraints for it68051_capture_top (GW5A-25)
#
# >>> mem写修复后首次使用；若 get_nets 不被教育版SDC支持导致解析报错，
# >>> 换用 timing_debug.sdc（最小回退版）并按 AGENTS.md 记录报错原文。
#
# Clocks
#   clk_vid : IT68051 PCLK, 148.5 MHz (2K144 CVT-RB), ball M10 = GCLKT_9B,
#             buffered to the global network via DCE.
#   clk_usb : FT601 FIFO bus clock, 100 MHz, ball H4 = GCLKT_15, same DCE flow.
#
#   create_clock uses NET-level anchors [get_nets {clk_vid_gowin} /
#   clk_usb_gowin].  These are the buffered clock net names the P&R timing
#   analysis actually reports (TA1132 warnings listed them verbatim in the
#   flood-era runs).  Port-level create_clock on RX_PCLK/FT601_CLK does not
#   propagate through the DCE primitive: the tool then checks nothing inside
#   the design (all internal paths unchecked).  If this SDC file fails to
#   parse (get_nets rejected by the education SDC reader), fall back to
#   timing_debug.sdc and record the verbatim error in AGENTS.md.
#
# External AC parameters (source documents, see docs/README.md):
#   IT68051 dual-pixel DDR (datasheet 7.5): TPSRd=1.22 / TPSFd=1.00 ns setup,
#   TPHRd=1.42 / TPHFd=0.84 ns hold at PCLK;  DE/HS/V single-edge
#   TPSs=2.88 / TPHs=1.88 ns.
#   FT601 245-sync FIFO (datasheet Table 4.2): slave outputs T1=3.0/T2=3.5 ns
#   around CLK rising; master inputs T3=1.0 ns setup / T4=4.8 ns hold.
#   The FPGA launches FT601 DATA/BE/WR_N/OE_N/RD_N from NEGEDGE IOB flops:
#     setup  : tCO(falling) <= 5.0 - T3 = 4.0 ns   (checked by -max 1.0)
#     hold   : next launch is a full period later -> always satisfied
# -----------------------------------------------------------------------------

create_clock -name clk_vid -period 6.734 [get_nets {clk_vid_gowin}]
create_clock -name clk_usb -period 10.000 [get_nets {clk_usb_gowin}]

# The two domains meet only inside the gray-pointer async FIFO.
# (set_clock_groups showed back-end problems on this education toolchain;
# two explicit false paths are the sanctioned equivalent.)
set_false_path -from [get_clocks {clk_vid}] -to [get_clocks {clk_usb}]
set_false_path -from [get_clocks {clk_usb}] -to [get_clocks {clk_vid}]

# ------------------------- IT68051 QD[59:0] dual-pixel DDR video bus ---------
# rising-edge captures (word_r = word at PCLK rising)
set_input_delay -clock clk_vid -max 5.514 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]
set_input_delay -clock clk_vid -min 1.000 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]
# falling-edge captures (word_f = word at PCLK falling)
set_input_delay -clock clk_vid -clock_fall -max 5.314 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]
set_input_delay -clock clk_vid -clock_fall -min 0.840 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]

# ------------------------- IT68051 single-edge syncs -------------------------
set_input_delay -clock clk_vid -max 3.854 [get_ports {RX_DE RX_HSYNC RX_VSYNC}]
set_input_delay -clock clk_vid -min 1.880 [get_ports {RX_DE RX_HSYNC RX_VSYNC}]

# ------------------------- FT601 slave-driven inputs -------------------------
# (T1/T2: valid from CLK-3.0 to CLK+3.5 -> max = 10-3.0, min = 3.5)
set_input_delay -clock clk_usb -max 7.000 [get_ports {FT601_TXE_N FT601_RXF_N}]
set_input_delay -clock clk_usb -min 3.500 [get_ports {FT601_TXE_N FT601_RXF_N}]
# DATA driven by FT601 only during cmd-path reads
set_input_delay -clock clk_usb -max 7.000 [get_ports {FT601_DATA_* FT601_BE_*}]
set_input_delay -clock clk_usb -min 3.500 [get_ports {FT601_DATA_* FT601_BE_*}]

# ------------------------- FT601 master-driven outputs -----------------------
# launched at the falling edge (see header)
set_output_delay -clock clk_usb -max 1.000 [get_ports {FT601_WR_N FT601_OE_N FT601_RD_N}]
set_output_delay -clock clk_usb -min 4.800 [get_ports {FT601_WR_N FT601_OE_N FT601_RD_N}]
set_output_delay -clock clk_usb -max 1.000 [get_ports {FT601_DATA_* FT601_BE_*}]
set_output_delay -clock clk_usb -min 4.800 [get_ports {FT601_DATA_* FT601_BE_*}]

# ------------------------- quasi-static / slow pins --------------------------
# I2C programming port (200 kHz bit engine, timings self-consistent).
# Port names are the RTL names it68051_* (netlist nets 68051_* cannot be
# Verilog identifiers; the .cst comments carry the netlist names).
set_false_path -from [get_ports {it68051_pcscl it68051_pcsda}]
set_false_path -to   [get_ports {it68051_pcscl it68051_pcsda}]
# board-level resets / HPD (milliseconds domain)
set_false_path -to   [get_ports {it68051_sysrstn it68051_hpd FT601_RESET_N}]
# asynchronous board inputs used only as status
set_false_path -from [get_ports {it68051_int FT601_SIWU_N FT601_WAKEUP_N}]
