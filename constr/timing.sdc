# 100MHz board-level ESTIMATE, not final signoff without stackup/actual length data.
# Assumption: 150ps/in. +/-400mil residual mismatch -> conservative 70ps.
# Same-direction video/FT601 inputs: common clock/data delay cancels; +/-70ps remains.
# FT601 outputs: 2000mil clock to FPGA + matched data return -> C+D about 600ps.
create_clock -name rx_pclk -period 6.734 [get_ports {RX_PCLK}]
create_generated_clock -name clk_vid -source [get_ports {RX_PCLK}] -master_clock rx_pclk -divide_by 1 [get_pins {u_buf_vid/CLKOUT}]
create_clock -name ft601_clk -period 10.000 [get_ports {FT601_CLK}]
create_generated_clock -name clk_usb -source [get_ports {FT601_CLK}] -master_clock ft601_clk -divide_by 1 [get_pins {u_buf_usb/CLKOUT}]

set_false_path -from [get_clocks {clk_vid}] -to [get_clocks {clk_usb}]
set_false_path -from [get_clocks {clk_usb}] -to [get_clocks {clk_vid}]

set_input_delay -clock rx_pclk -max 3.924 [get_ports {RX_DE RX_HSYNC RX_VSYNC}]
set_input_delay -clock rx_pclk -min 1.810 [get_ports {RX_DE RX_HSYNC RX_VSYNC}]
set_input_delay -clock ft601_clk -max 7.070 [get_ports {FT601_TXE_N FT601_RXF_N}]
set_input_delay -clock ft601_clk -min 3.430 [get_ports {FT601_TXE_N FT601_RXF_N}]
set_output_delay -clock ft601_clk -max 1.670 [get_ports {FT601_WR_N FT601_OE_N FT601_RD_N}]
set_output_delay -clock ft601_clk -min -4.270 [get_ports {FT601_WR_N FT601_OE_N FT601_RD_N}]

set_input_delay -clock rx_pclk -max 2.217 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]
set_input_delay -clock rx_pclk -min 0.930 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]
set_input_delay -clock rx_pclk -clock_fall -add_delay -max 2.017 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]
set_input_delay -clock rx_pclk -clock_fall -add_delay -min 0.770 [get_ports {A_B* A_G* A_R* B_B* B_G* B_R*}]

set_input_delay -clock ft601_clk -max 7.070 [get_ports {FT601_DATA_0 FT601_DATA_1 FT601_DATA_2 FT601_DATA_3 FT601_DATA_4 FT601_DATA_5 FT601_DATA_6 FT601_DATA_7 FT601_DATA_8 FT601_DATA_9 FT601_DATA_10 FT601_DATA_11 FT601_DATA_12 FT601_DATA_13 FT601_DATA_14 FT601_DATA_15 FT601_DATA_16 FT601_DATA_17 FT601_DATA_18 FT601_DATA_19 FT601_DATA_20 FT601_DATA_21 FT601_DATA_22 FT601_DATA_23 FT601_DATA_24 FT601_DATA_25 FT601_DATA_26 FT601_DATA_27 FT601_DATA_28 FT601_DATA_29 FT601_DATA_30 FT601_DATA_31}]
set_input_delay -clock ft601_clk -min 3.430 [get_ports {FT601_DATA_0 FT601_DATA_1 FT601_DATA_2 FT601_DATA_3 FT601_DATA_4 FT601_DATA_5 FT601_DATA_6 FT601_DATA_7 FT601_DATA_8 FT601_DATA_9 FT601_DATA_10 FT601_DATA_11 FT601_DATA_12 FT601_DATA_13 FT601_DATA_14 FT601_DATA_15 FT601_DATA_16 FT601_DATA_17 FT601_DATA_18 FT601_DATA_19 FT601_DATA_20 FT601_DATA_21 FT601_DATA_22 FT601_DATA_23 FT601_DATA_24 FT601_DATA_25 FT601_DATA_26 FT601_DATA_27 FT601_DATA_28 FT601_DATA_29 FT601_DATA_30 FT601_DATA_31}]
set_output_delay -clock ft601_clk -max 1.670 [get_ports {FT601_DATA_* FT601_BE_*}]
set_output_delay -clock ft601_clk -min -4.270 [get_ports {FT601_DATA_* FT601_BE_*}]

set_false_path -from [get_ports {it68051_pcscl it68051_pcsda}]
set_false_path -to   [get_ports {it68051_pcscl it68051_pcsda}]
set_false_path -to   [get_ports {it68051_sysrstn it68051_hpd FT601_RESET_N}]
set_false_path -from [get_ports {it68051_int FT601_SIWU_N FT601_WAKEUP_N}]
