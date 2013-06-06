/********************************************************************************/
/**
 * \name ad5933.h
 * 
 * This is the main file. It calls all low level functions, initiates libusb, and
 * communicate with a high level application through a udp interface.
 *
 * Supported OSs: Mac OSx
 *
 * Copyright 2013 David Cemin <davidcemin at gmail dot com>
 *
 * This file may be distributed and/or modified under the terms of the 
 * GNU General Public License version 2 as published by the Free Software 
 * Foundation. (See COPYING.GPL for details.)
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 */
/********************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "xusb.h"

/****************************************************************************/

/*Register Map*/
#define AD5933_CTRL_REG_MSB		0x80
#define AD5933_CTRL_REG_LSB		0x81
#define AD5933_START_FREQ1		0x82
#define AD5933_START_FREQ2		0x83
#define AD5933_START_FREQ3		0x84
#define AD5933_FREQ_INC1		0x85
#define AD5933_FREQ_INC2		0x86
#define AD5933_FREQ_INC3		0x87
#define AD5933_NUMB_INCR_MSB	0x88
#define AD5933_NUMB_INCR_LSB	0x89
#define AD5933_NUMB_SETT_MSB	0x8A
#define AD5933_NUMB_SETT_LSB	0x8B
#define AD5933_STS_REG			0x8f
#define AD5933_TEMP_DATA_MSB	0x92
#define AD5933_TEMP_DATA_LSB	0x93
#define AD5933_REAL_DATA_MSB	0x94
#define AD5933_REAL_DATA_LSB	0x95
#define AD5933_IMAG_DATA_MSB	0x96
#define AD5933_IMAG_DATA_LSB	0x97

/*Control register masks*/
/*Bits 15 to 12*/
#define MASK_INIT_START_FREQ	(0x10) 
#define MASK_START_FREQ_SWEEP	(0x20)
#define MASK_INC_FREQ			(0x30)
#define MASK_REPEAT_FREQ		(0x40)
#define MASK_MEAS_TEMP			(0x90)
#define MASK_PD_MODE			(0xa0)
#define MASK_SB_MODE			(0xb0)

/*Bits 10 to 9*/
#define MASK_OUTPUT_2Vpp		(0x00)
#define MASK_OUTPUT_200mVpp		(0x02)
#define MASK_OUTPUT_400mVpp		(0x04)
#define MASK_OUTPUT_1Vpp		(0x06)

/*Bit 8*/
#define MASK_PGA_GAIN5x			(0x0) /*bit 9: 0: pga=5; 1: pga=1*/
#define MASK_PGA_GAIN1x			(0x1) /*bit 9: 0: pga=5; 1: pga=1*/

/*Bits 7 to 0*/
#define MASK_RESET_TRUE			(0x10)
#define MASK_RESET_FALSE		(0x00)
#define MASK_CLK_SRC_EXT		(0x08) /*0: internal; 1: external*/
#define MASK_CLK_SRC_INT		(0x08) /*0: internal; 1: external*/

/*Status register masks*/
#define MASK_STS_TEMP_VALID		(0x01)
#define MASK_STS_IMPED_VALID	(0x02)
#define MASK_STS_FRQ_SWP_RDY	(0x03)


/********************************************************************************/

#define AD5933_RES_FEEDBACK_VAL	(200)
#define AD5933_RES_CALIB_VAL	(561)
#define INT_CLK_FREQ			(16) /*MHz*/
