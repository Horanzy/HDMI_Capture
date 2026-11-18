# Regenerate DDR3 + PLL for the exact production FPGA, not the tutorial ES part.
set ip_root [file dirname [file normalize [info script]]]
create_project -name ipgen -dir $ip_root -pn GW5A-LV25UG324C2/I1 -device_version A -force

create_ipc -name ddr3 -module_name DDR3_Memory_Interface_Top -language Verilog \
    -file_name ddr3_memory_interface -dir $ip_root -force
set ddr [get_ips DDR3_Memory_Interface_Top]
set_property -dict {
    CONFIG.Memory_Type Components
    CONFIG.Memory_Clock 400
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

# The external-mDRP PLL wrapper and its source IPC are checked in from the
# official chapter-38 GW5A-25 flow (50 MHz in, 50/400 MHz outputs).
# Gowin 1.9.11.03 crashes in read_ipc for this legacy IPC, so this script only
# regenerates the DDR PHY; do not silently replace the verified PLL wrapper.

run close
exit
