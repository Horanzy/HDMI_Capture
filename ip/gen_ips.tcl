# Regenerate DDR3 + PLL for the exact production FPGA, not the tutorial ES part.
set ip_root [file dirname [file normalize [info script]]]
create_project -name ipgen -dir $ip_root -pn GW5A-LV25UG324C2/I1 -device_version A -force

create_ipc -name ddr3 -module_name DDR3_Memory_Interface_Top -language Verilog \
    -file_name ddr3_memory_interface -dir $ip_root -force
set ddr [get_ips DDR3_Memory_Interface_Top]
set_property -dict {
    CONFIG.Memory_Type Components
    CONFIG.Memory_Clock 200
    CONFIG.CLK_Ratio 1:4
    CONFIG.Dq_Width 16
    CONFIG.Dram_Width 16
    CONFIG.Ecc false
    CONFIG.Rank_Address 1
    CONFIG.Bank_Address 3
    CONFIG.Row_Address 14
    CONFIG.Column_Address 10
    CONFIG.Burst_Mode 8
    CONFIG.Burst_Type SEQ
    CONFIG.CAS_Latency 5
    CONFIG.CW_Latency 5
    CONFIG.Additive_Latency 0
    CONFIG.Wrire_Recovery 6
    CONFIG.Rtt_Nom 40
    CONFIG.Rtt_Wr OFF
    CONFIG.Addr_Cmd_Mode 1T
    CONFIG.OUTPUT_DRV LOW
    CONFIG.User_Refresh false
    CONFIG.tRTP_Period 7500
    CONFIG.tRP_Period 12500
    CONFIG.tWTR_Period 7500
    CONFIG.tRC_Period 55000
    CONFIG.tRAS_Period 37500
    CONFIG.tRCD_Period 12500
    CONFIG.tFAW_Period 40000
    CONFIG.tRRD_Period 7500
    CONFIG.tCKE_Period 75000
    CONFIG.tREFI_Period 7800000
    CONFIG.tRFC_Period 160000
    CONFIG.tDLLK 512
    CONFIG.Debug_Parameter_Enable false
    CONFIG.Debug_Port_Enable false
} $ddr
report_property $ddr
generate_target $ddr

create_ipc -name clock_plladv -module_name Gowin_PLL_DDR3 -language Verilog \
    -file_name gowin_pll_ddr3 -dir $ip_root -force
set pll [get_ips Gowin_PLL_DDR3]
set_property -dict {
    CONFIG.General_Mode true
    CONFIG.Adcanced_Mode false
    CONFIG.CLKIN_FREQ 50
    CONFIG.LOCK_EN true
    CONFIG.CLKOUT0_FREQ 50
    CONFIG.CLKOUT1_EN false
    CONFIG.CLKOUT1_FREQ 50
    CONFIG.CLKOUT2_EN true
    CONFIG.CLKOUT2_FREQ 200
    CONFIG.PLL_INIT_CLK_FREQ 50
} $pll
report_property $pll
generate_target $pll
file copy -force [file join $ip_root ipgen src pll_init.v] \
    [file join $ip_root gowin_pll_ddr3 pll_init.v]

run close
exit
