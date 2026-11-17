///*****************************************
//  Copyright (C) 2009-2020
//  ITE Tech. Inc. All Rights Reserved
//  Proprietary and Confidential
///*****************************************
//   @file   <IO.h>
//   @author Kuro.Chung@ite.com.tw
//   @date   2020/09/28
//   @fileversion: iTE6805_MCUSRC_1.57
//******************************************/

#ifndef _IO_h_
#define _IO_h_
#include "typedef.h"


iTE_u1 i2c_write_byte(iTE_u8 address, iTE_u8 offset, iTE_u8 byteno, iTE_u8 *p_data, iTE_u8 device);
iTE_u1 i2c_read_byte(iTE_u8 address, iTE_u8 offset, iTE_u8 byteno, iTE_u8 *p_data, iTE_u8 device);

#endif
