//Copyright (C)2014-2023 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.9 (64-bit)
//Part Number: GW5A-LV25UG324C2/I1
//Device: GW5A-25
//Device Version: A
//Created Time: Wed Mar 19 14:42:20 2025

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    Gowin_PLL_DDR3 your_instance_name(
        .lock(lock_o), //output lock
        .clkout0(clkout0_o), //output clkout0
        .clkout1(clkout1_o), //output clkout1
        .clkout2(clkout2_o), //output clkout2
        .mdrdo(mdrdo_o), //output [7:0] mdrdo
        .clkin(clkin_i), //input clkin
        .reset(reset_i), //input reset
        .mdclk(mdclk_i), //input mdclk
        .mdopc(mdopc_i), //input [1:0] mdopc
        .mdainc(mdainc_i), //input mdainc
        .mdwdi(mdwdi_i) //input [7:0] mdwdi
    );

//--------Copy end-------------------
