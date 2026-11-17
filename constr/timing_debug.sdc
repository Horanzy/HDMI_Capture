# -----------------------------------------------------------------------------
# timing_debug.sdc -- MINIMAL debug constraints (known to pass P&R)
#
# >>> 时序覆盖不全，仅用于P&R卡死时的回退（正式约束见 timing.sdc）。
#
# Contents: port-level create_clock on the two clock balls, the two cross-
# clock false paths, and false paths on the quasi-static / slow pins only.
# NO input/output delays at all -- the tool therefore does not check the
# IT68051 DDR video margins or the FT601 T3/T4 launch/hold windows.  Use this
# only to get a bitstream when the full SDC blocks P&R, and record the full
# SDC's failure in AGENTS.md so the constraint work can resume.
# -----------------------------------------------------------------------------

create_clock -name clk_vid -period 6.734 [get_ports {RX_PCLK}]
create_clock -name clk_usb -period 10.000 [get_ports {FT601_CLK}]

# the two domains meet only inside the gray-pointer async FIFO
set_false_path -from [get_clocks {clk_vid}] -to [get_clocks {clk_usb}]
set_false_path -from [get_clocks {clk_usb}] -to [get_clocks {clk_vid}]

# quasi-static / slow pins only (RTL port names, lowercase it68051_*)
set_false_path -from [get_ports {it68051_pcscl it68051_pcsda}]
set_false_path -to   [get_ports {it68051_pcscl it68051_pcsda}]
set_false_path -to   [get_ports {it68051_sysrstn it68051_hpd FT601_RESET_N}]
set_false_path -from [get_ports {it68051_int FT601_SIWU_N FT601_WAKEUP_N}]
