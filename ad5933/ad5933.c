/********************************************************************************/
/**
 * \name ad5933.c
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

#include <strings.h> 	/*bzero*/
#include <math.h>

#include "ad5933.h"

/********************************************************************************/
/* Private stuff																*/
/********************************************************************************/

typedef struct {
	unsigned char real_msb;
	unsigned char real_lsb;
	unsigned char imag_msb;
	unsigned char imag_lsb;
} st_imped_data_raw_t;

typedef struct {
	double magnitude;
	double phase;
} st_imped_data_t;

/********************************************************************************/

static inline char
ad5933_reg_write(usb_dev_handle *h, unsigned char addr, unsigned char val, unsigned char sz)
{
	unsigned char ret = 0;
	unsigned char dum = val & 0xff;

	printf("addr 0x%x data 0x%x\n\r", addr, (val<<8) | addr);
	ret = usb_control_msg(h, 0x40, 0xde, 0x0d, val << 8 | addr, &dum, sz, 1000);

	return (ret);
}

/********************************************************************************/

static inline char
ad5933_reg_read(usb_dev_handle *h, unsigned char reg, unsigned char *val, unsigned char sz)
{
	unsigned char ret = 0;

	ret = usb_control_msg(h, 0xc0, 0xde, 0x0d, reg, val, sz, 1000);

	return (ret);
}

/********************************************************************************/

static inline char
ad5933_get_status(usb_dev_handle *h)
{
	char sts_reg = 0x0;

	ad5933_reg_read(h, AD5933_STS_REG, &sts_reg, 1);

	printf("status reg = 0x%x\n\r", sts_reg);

	return (sts_reg);
}

/********************************************************************************/

static inline char
ad5933_reset(usb_dev_handle *h)
{
	unsigned char ctrl_lsb = 0x00;

	ad5933_reg_read(h, AD5933_CTRL_REG_LSB, &ctrl_lsb, 1);

	ctrl_lsb |= MASK_RESET_TRUE;
	ad5933_reg_write(h, AD5933_CTRL_REG_LSB, ctrl_lsb, 1);
	ad5933_reg_read(h, AD5933_CTRL_REG_LSB, &ctrl_lsb, 1);

	//printf("CTRL LSB = 0x%x\n", ctrl_lsb);


	return (0);
}

/****************************************************************************/

static inline char
ad5933_imped_read(usb_dev_handle *h, st_imped_data_raw_t *ir)
{
	unsigned char ret = 0;

	bzero(ir, sizeof(ir));

	ret |= ad5933_reg_read(h, AD5933_REAL_DATA_MSB, &(ir->real_msb), 1);
	ret |= ad5933_reg_read(h, AD5933_REAL_DATA_LSB, &(ir->real_lsb), 1);
	ret |= ad5933_reg_read(h, AD5933_IMAG_DATA_MSB, &(ir->imag_msb), 1);
	ret |= ad5933_reg_read(h, AD5933_IMAG_DATA_MSB, &(ir->imag_lsb), 1);

	return (ret);
}

/********************************************************************************/

static inline double
ad5933_magnitude_calc(unsigned char rmsb, unsigned char rlsb, unsigned char imsb, unsigned char ilsb)
{
	double r = (rmsb << 8) | (rlsb);
	double i = (imsb << 8) | (ilsb);
	double m = sqrt( pow(r,2)+pow(i,2) );
	return m;
}

/********************************************************************************/

/*freq is in kHz*/
static inline char
ad5933_start_freq_calc(usb_dev_handle *h, double freq)
{
	unsigned int freq_code = 0;
	unsigned char f1 = 0;
	unsigned char f2 = 0;
	unsigned char f3 = 0;

	freq_code = freq/(INT_CLK_FREQ*250)*pow(2,27);

	printf("F = 0x%x\n", freq_code);

	f1 = (freq_code >> 16) & 0xff;
	f2 = (freq_code >> 8) & 0xff;
	f3 = (freq_code) & 0xff;

	ad5933_reg_write(h, AD5933_START_FREQ1, f1, 1);
	ad5933_reg_write(h, AD5933_START_FREQ2, f2, 1);
	ad5933_reg_write(h, AD5933_START_FREQ3, f3, 1);

	return (0);
}

/********************************************************************************/

static inline char
ad5933_numb_incr_set(usb_dev_handle *h, unsigned int inc)
{
	unsigned char msb = (inc >> 8) & 0xff;
	unsigned char lsb = inc & 0xff;

	ad5933_reg_write(h, AD5933_NUMB_INCR_MSB, msb, 1);
	ad5933_reg_write(h, AD5933_NUMB_INCR_LSB, lsb, 1);

	return (0);
}

/****************************************************************************/

/*step is in hertz*/
static inline char
ad5933_freq_inc_set(usb_dev_handle *h, double step)
{
	unsigned int step_code = 0;
	unsigned char f1 = 0;
	unsigned char f2 = 0;
	unsigned char f3 = 0;

	step_code = pow(2,27)*step*4/(INT_CLK_FREQ*pow(10,6));

	f1 = (step_code >> 16) & 0xff;
	f2 = (step_code >> 8) & 0xff;
	f3 = (step_code) & 0xff;

	printf("S = 0x%x\n\r", step_code);

	ad5933_reg_write(h, AD5933_FREQ_INC1, f1, 1);
	ad5933_reg_write(h, AD5933_FREQ_INC2, f2, 1);
	ad5933_reg_write(h, AD5933_FREQ_INC3, f3, 1);

	return (0);
}

/****************************************************************************/

/*mult can be 0, 1 or 3*/
static inline char
ad5933_settling_set(usb_dev_handle *h, unsigned int s, unsigned char mult)
{
	unsigned char msb = (mult << 3) | ( (s << 8) & 0xff );
	unsigned char lsb = s & 0xff;

	ad5933_reg_write(h, AD5933_NUMB_SETT_MSB, msb, 1);
	ad5933_reg_write(h, AD5933_NUMB_SETT_LSB, lsb, 1);

	return (0);
}

/****************************************************************************/

/*gf = gfe-9;*/
static inline char
ad5933_imped_calibration(usb_dev_handle *h, double *gf)
{
	unsigned char ret = 0;
	st_imped_data_raw_t ir;
	double gf1 = 0.0;
	double gf2 = 0.0;
	double m1 = 0.0;
	double m2 = 0.0;
	short real = 0x0000;
	short imag = 0x0000;
	unsigned char ctrl_reg = 0;
	unsigned int steps = 200;
	unsigned int inc = 1;
	unsigned int i = 0;
	unsigned char sweep_ok = 0;

	/*AD5933.pdf page 22*/
	/*1: Start frequency register*/
	ad5933_start_freq_calc(h, 60);
	/*2: Number of increments register*/
	ad5933_numb_incr_set(h, steps);
	/*3: Frequency increment register*/
	ad5933_freq_inc_set(h, 1);
	/* Settling time*/
	ad5933_settling_set(h, 15, 0);

	/*Place ad5933 in standby mode*/
	ad5933_reset(h);
	ad5933_reg_read(h, AD5933_CTRL_REG_MSB, &ctrl_reg, 1);
	ctrl_reg |= MASK_SB_MODE;
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_reg, 1);

	/*Start Frequency*/
	ctrl_reg = MASK_INIT_START_FREQ | MASK_OUTPUT_2Vpp | MASK_PGA_GAIN5x;
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_reg, 1);
	usleep(2000);

	/*Calculate first point*/
	ad5933_imped_read(h, &ir);
	m1 = ad5933_magnitude_calc(ir.real_msb, ir.real_lsb, ir.imag_msb, ir.imag_lsb);

	/*start sweep*/
	ctrl_reg = MASK_START_FREQ_SWEEP | MASK_OUTPUT_2Vpp | MASK_PGA_GAIN5x;
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_reg, 1);

	ctrl_reg = 0;
	ad5933_reg_read(h, AD5933_CTRL_REG_MSB, &ctrl_reg, 1);

	while ( sweep_ok != MASK_STS_FRQ_SWP_RDY)
	{
		unsigned char sts = 0;
		unsigned char ctrl_inc = 0;
		int j = 0;

		ad5933_reg_read(h, AD5933_CTRL_REG_MSB, &ctrl_inc, 1);
		sts = ad5933_get_status(h);
		while ( ( sts & MASK_STS_IMPED_VALID ) != MASK_STS_IMPED_VALID) {
			printf("waiting imp valid..0x%x %d\n\r", sts, j);
			usleep(1000);
			j++;
			if (j==11)
				break;
		}
		ad5933_imped_read(h, &ir);
		m2 = ad5933_magnitude_calc(ir.real_msb, ir.real_lsb, ir.imag_msb, ir.imag_lsb);
		printf("m1 = %04f m2 = %04f\n\r", m1, m2);

		i += inc;
		ctrl_inc |= MASK_INC_FREQ;
		ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_inc, 1);
		sweep_ok = ad5933_get_status(h) & MASK_STS_FRQ_SWP_RDY;
		printf("sweep ok 0x%x \n\r", sweep_ok);
	}

	printf("-------------------------------------\n\r");
	printf("%04f %04f\n\r", m1, m2);
	printf("-------------------------------------\n\r");

	/*two measures, we are considering that it varies linearly with the frequency.
	 * The smaller the difference (delta F) in frequency, the better */
	*gf = (pow(10,9)/AD5933_RES_FEEDBACK_VAL)/( ((m2-m1)/2) + m2);

	return (ret);
}

/********************************************************************************/

static inline char
ad5933_imped_calc(usb_dev_handle *h, st_imped_data_t *imped, double gf)
{
	st_imped_data_raw_t i;
	double m = 0;

	ad5933_imped_read(h, &i);

	printf("0x%x 0x%x 0x%x 0x%x\n\r", i.real_msb, i.real_lsb, i.imag_msb, i.imag_lsb);
	m = ad5933_magnitude_calc(i.real_msb, i.real_lsb, i.imag_msb, i.imag_lsb);

	imped->magnitude = 1/(pow(10,-9)*m*gf);

	return (0);	
}
/********************************************************************************/

static inline short
ad5933_get_temperature(usb_dev_handle *h)
{
	short temp = 0x0;
	char t0 = 0;
	char t1 = 0;
	char control_reg_msb = 0x0;

	usb_control_msg(h, 0xc0, 0xde, 0x0d, AD5933_CTRL_REG_MSB, &control_reg_msb, 1, 1000);
	control_reg_msb = MASK_MEAS_TEMP;
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, control_reg_msb, 0);

	printf("Temperature %s !!\n\r", (ad5933_get_status(h) & MASK_STS_TEMP_VALID) ? "Ready" : "Not Ready");

	ad5933_reg_read(h, AD5933_TEMP_DATA_MSB, &t0, 1);
	ad5933_reg_read(h, AD5933_TEMP_DATA_LSB, &t1, 1);

	/*if positive (bit 13 = 0), temp = adc/32;
	 * if negative (bit 13 = 1), temp = (adc - 16384)/32
	 */
	if (t0 >> 5)
		temp = ( ( ((t0&0xff)<<8) | (t1&0xff) ) - 16384) >> 5;
	else
		temp = ( ((t0&0xff)<<8) |  (t1&0xff) ) >> 5;

	printf("0x%x 0x%x %d \n", t0, t1, temp);

	return (temp);
}


/********************************************************************************/
/* Public stuff																	*/
/********************************************************************************/

int
main (int argc, char **argv)
{
	int vendor_id = 0x0456; /*Analog Devices*/
	int prod_id = 0xb203;  /*Prod id =)*/
	int rv = 0;
	unsigned char rst_dbg[10];
	struct usb_device *usbdev = NULL;
	struct usb_dev_handle *usbdevhandle = NULL;
	const char *file = "../fw/hex/AD5933_34FW.hex";
	//const char *file = "../fw/hex/hello.ihx";
	unsigned int err = 0;
	unsigned char buff_w = 0;
	st_imped_data_t imped;
	st_imped_data_raw_t ir;
	double gf = 0.0;

	bzero(&imped, sizeof(imped));
	bzero(&ir, sizeof(ir));

	/* Initializing libusb*/
	usb_init();
	usb_set_debug(2);

	/*find busses*/
	if ( (rv = usb_find_busses() ) < 0) 
	{
		fprintf(stderr, "usb_find_busses() failed! (rv=%d)\n", rv);
		return (1);
	}

	/*find devices*/
	if ( (rv = usb_find_devices() ) < 0) 
	{
		fprintf(stderr, "usb_find_devices() failed! (rv=%d)\n", rv);
		return (1);
	}

	if ( !(usbdev = xusb_find_device(vendor_id, prod_id) ) )
	{
		fprintf(stderr, "Device with vendor_id=0x%04x, prod_id=%04x not attached.\n", vendor_id, prod_id);
		return (1);
	}

	fprintf(stdout, "Using ID %04x:%04x on %s.%s.\n",
			usbdev->descriptor.idVendor, usbdev->descriptor.idProduct,
			usbdev->bus->dirname, usbdev->filename);

	/*opening device*/
	if ( !(usbdevhandle = xusb_open(usbdev)) )
		return(1);
	
	usb_set_configuration(usbdevhandle,1);
	usb_claim_interface(usbdevhandle,0);
	usb_set_altinterface(usbdevhandle,1);

	//fprintf(stdout, "Programming 8051 using %s \n", file);
	//err += xusb_program_hex_file(file, usbdevhandle);

	ad5933_reset(usbdevhandle);
	ad5933_get_temperature(usbdevhandle);
	ad5933_imped_calibration(usbdevhandle, &gf);
	ad5933_imped_calc(usbdevhandle, &imped, gf);

	printf("GF = %04f\n\r", gf);
	printf("IMP = %04f\n\r", imped.magnitude);

    //usb_close(usbdevhandle);
	return (0);
}


