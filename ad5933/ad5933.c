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
	double m;
} st_imped_data_t;

/********************************************************************************/

static inline char
ad5933_reg_write(usb_dev_handle *h, unsigned char addr, unsigned char val)
{
	unsigned char ret = 0;
	unsigned char dum = val & 0xff;

	//printf("addr 0x%x data 0x%x\n\r", addr, (val<<8) | addr);
	ret = usb_control_msg(h, 0x40, 0xde, 0x0d, val << 8 | addr, &dum, 1, 1000);

	return (ret);
}

/********************************************************************************/

static inline char
ad5933_reg_read(usb_dev_handle *h, unsigned char reg, unsigned char *val)
{
	unsigned char ret = 0;

	ret = usb_control_msg(h, 0xc0, 0xde, 0x0d, reg, val, 1, 1000);

	return (ret);
}

/********************************************************************************/

static inline char
ad5933_get_status(usb_dev_handle *h)
{
	char sts_reg = 0x0;

	ad5933_reg_read(h, AD5933_STS_REG, &sts_reg);

	//printf("status reg = 0x%x\n\r", sts_reg);

	return (sts_reg);
}

/********************************************************************************/

static inline char
ad5933_reset(usb_dev_handle *h)
{
	unsigned char ctrl_lsb = 0x00;

	ad5933_reg_read(h, AD5933_CTRL_REG_LSB, &ctrl_lsb);

	ctrl_lsb |= MASK_RESET_TRUE;
	ad5933_reg_write(h, AD5933_CTRL_REG_LSB, ctrl_lsb);

	return (0);
}

/****************************************************************************/

static inline char
ad5933_imped_read(usb_dev_handle *h, st_imped_data_raw_t *ir)
{
	unsigned char ret = 0;

	bzero(ir, sizeof(ir));

	ret |= ad5933_reg_read(h, AD5933_REAL_DATA_MSB, &(ir->real_msb));
	ret |= ad5933_reg_read(h, AD5933_REAL_DATA_LSB, &(ir->real_lsb));
	ret |= ad5933_reg_read(h, AD5933_IMAG_DATA_MSB, &(ir->imag_msb));
	ret |= ad5933_reg_read(h, AD5933_IMAG_DATA_MSB, &(ir->imag_lsb));

	return (ret);
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

	//printf("F = 0x%x\n", freq_code);

	f1 = (freq_code >> 16) & 0xff;
	f2 = (freq_code >> 8) & 0xff;
	f3 = (freq_code) & 0xff;

	ad5933_reg_write(h, AD5933_START_FREQ1, f1);
	ad5933_reg_write(h, AD5933_START_FREQ2, f2);
	ad5933_reg_write(h, AD5933_START_FREQ3, f3);

	return (0);
}

/********************************************************************************/

static inline char
ad5933_numb_incr_set(usb_dev_handle *h, unsigned int inc)
{
	unsigned char msb = (inc >> 8) & 0xff;
	unsigned char lsb = inc & 0xff;

	ad5933_reg_write(h, AD5933_NUMB_INCR_MSB, msb);
	ad5933_reg_write(h, AD5933_NUMB_INCR_LSB, lsb);

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

	//printf("S = 0x%x\n\r", step_code);

	ad5933_reg_write(h, AD5933_FREQ_INC1, f1);
	ad5933_reg_write(h, AD5933_FREQ_INC2, f2);
	ad5933_reg_write(h, AD5933_FREQ_INC3, f3);

	return (0);
}

/****************************************************************************/

/*mult can be 0, 1 or 3*/
static inline char
ad5933_settling_set(usb_dev_handle *h, unsigned int s, unsigned char mult)
{
	unsigned char msb = (mult << 3) | ( (s << 8) & 0xff );
	unsigned char lsb = s & 0xff;

	ad5933_reg_write(h, AD5933_NUMB_SETT_MSB, msb);
	ad5933_reg_write(h, AD5933_NUMB_SETT_LSB, lsb);

	return (0);
}

/****************************************************************************/

static inline double
ad5933_magnitude_calc(st_imped_data_raw_t ir)
{
	unsigned char rmsb = ir.real_msb;
   	unsigned char rlsb = ir.real_lsb;
   	unsigned char imsb = ir.imag_msb;
   	unsigned char ilsb = ir.imag_lsb;
	double r = (rmsb << 8) | (rlsb);
	double i = (imsb << 8) | (ilsb);
	double m = sqrt( pow(r,2)+pow(i,2) );
	return m;
}

/********************************************************************************/

static inline double
ad5933_phase_calc(st_imped_data_raw_t ir)
{
	unsigned char rmsb = ir.real_msb;
   	unsigned char rlsb = ir.real_lsb;
   	unsigned char imsb = ir.imag_msb;
   	unsigned char ilsb = ir.imag_lsb;
	double ph = 0.0;
	double r = (rmsb << 8) | (rlsb);
	double i = (imsb << 8) | (ilsb);

	if ( (r>0) && (i>0) ) /*first quadrant*/
		ph = atan(i/r)*180/M_PI;
	else if ( (r<0) && (i>0) ) /*second quadrant*/
		ph = atan(i/r)*180/M_PI + 180;
	else if ( (r<0) && (i<0) ) /*third quadrant*/
		ph = atan(i/r)*180/M_PI - 180;
	else /*fourth quadrant*/
		ph = atan(i/r)*180/M_PI + 360;

	return (ph);
}

/****************************************************************************/

static inline char
ad5933_imped_calc(usb_dev_handle *h, st_imped_data_t *imped, double gf)
{
	st_imped_data_raw_t i;

	ad5933_imped_read(h, &i);

	//printf("0x%x 0x%x 0x%x 0x%x\n\r", i.real_msb, i.real_lsb, i.imag_msb, i.imag_lsb);
	imped->m = ad5933_magnitude_calc(i);

	imped->magnitude = 1/(pow(10,-9)*imped->m*gf);
	imped->phase = ad5933_phase_calc(i);

	return (0);	
}

/********************************************************************************/

/**
 * \brief  Calculates a vectors mean
 * \param  val Pointer to the vector
 * \param  n nmemb of val
 * \param  mean Pointer to mean result
 * \return Vectors mean
 */
static double
dataMean(double *val, int n)
{	
	double ret;
	int i;
	double nmemb = n - 1;
	ret = 0.0;

	for (i = 0; i < n; i++) {
		//printf("%04f\n", val[i]);
		ret += val[i];
	}
	
	ret /= nmemb;

	return (ret);
}

/******************************************************************************/

/**
 * \brief Calculates max value of a vector
 * \param period pointer to period vector
 * \param nmemb number of members in period
 * \param max pointer to max value
 * \return void
 */
static void maxValue(double *period, int nmemb, double *max)
{
	int i;
	
	*max = fabs(period[1]);

	for (i = 1; i < nmemb; i++) 
		if (fabs(period[i]) > *max) 
			*max = fabs(period[i]);
}

/******************************************************************************/

/**
 * \brief Calculates min value of a vector
 * \param period pointer to period vector
 * \param nmemb number of members in period
 * \param min pointer to min value
 * \return void
 */
static void minValue(double *period, int nmemb, double *min)
{
	int i;
	
	*min = fabs(period[1]);

	for (i = 1; i < nmemb; i++) 
		if (fabs(period[i]) < *min) 
			*min = fabs(period[i]);
}

/******************************************************************************/

/**
 * \brief  Calculates the variance and standard deviation
 * \param  period pointer to period array
 * \param  nmemb number of members
 * \param  mean Mean of the period values
 * \param  variance pointer to the result
 * \param  stddev pointer to standard deviation
 * \return void
 */
static void variance_stddev(double *period, int nmemb, double mean, double *variance, double *stddev)
{
	int i;
	double sum = 0.0;
	double calc = 0.0;

	for (i = 0; i < nmemb; i++){
		calc = period[i] - mean;
		if(calc < CALCERROR)
			calc = 0;
		sum += pow(calc, 2);
	}
	*variance = (double)(sum / (nmemb));
	*stddev = (double)sqrt(*variance);
}

/******************************************************************************/

static inline char
ad5933_imp_avg_calc(st_imped_data_t *imp_data, double *m, double *magnitude, double *phase)
{
	imp_data->m = dataMean(m, NUM_AVG_POINTS);
	imp_data->magnitude = dataMean(magnitude, NUM_AVG_POINTS);
	imp_data->phase = dataMean(phase, NUM_AVG_POINTS);

	return (0);
}

/****************************************************************************/

/*gf = gfe-9;*/
static inline char
ad5933_imped_sweep(usb_dev_handle *h, unsigned char cal, double *gf, st_imped_data_t *imped)
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
	double step = 0.0;
	unsigned char sweep_ok = 0;
	unsigned int avg_points = 0;
	st_imped_data_t imp_data[NUM_SAMPLES];
	st_imped_data_t imp_data_avg;
	unsigned int sample = 0;
	double m[NUM_AVG_POINTS];
	double magnitude[NUM_AVG_POINTS];
	double phase[NUM_AVG_POINTS];
	double mag[NUM_SAMPLES];
	double ph[NUM_SAMPLES];
	unsigned int i;

	bzero(mag, sizeof(mag));
	bzero(ph, sizeof(ph));



	bzero(m, sizeof(m));
	bzero(magnitude, sizeof(magnitude));
	bzero(phase, sizeof(phase));
	bzero(imp_data, sizeof(imp_data));
	bzero(&imp_data_avg, sizeof(imp_data_avg));

	/*AD5933.pdf page 22*/
	/*1: Start frequency register*/
	ad5933_start_freq_calc(h, FREQ_LOW);
	/*2: Number of increments register*/
	ad5933_numb_incr_set(h, NUM_SAMPLES);
	/*3: Frequency increment register*/
	ad5933_freq_inc_set(h, FREQ_STEP);
	/* Settling time*/
	ad5933_settling_set(h, 15, 0);

	/*Place ad5933 in standby mode*/
	ad5933_reset(h);
	ad5933_reg_read(h, AD5933_CTRL_REG_MSB, &ctrl_reg);
	ctrl_reg |= MASK_SB_MODE;
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_reg);

	/*Start Frequency*/
	ctrl_reg = MASK_INIT_START_FREQ | MASK_OUTPUT_2Vpp | MASK_PGA_GAIN5x;
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_reg);
	usleep(2000);

	/*start sweep*/
	ctrl_reg = MASK_START_FREQ_SWEEP | MASK_OUTPUT_2Vpp | MASK_PGA_GAIN5x;
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_reg);

	while (sweep_ok != MASK_STS_FRQ_SWP_RDY)
	{
		unsigned char sts = 0;
		unsigned char ctrl_inc = 0;
		int j = 0;
		
		while ((sts = (ad5933_get_status(h) & MASK_STS_IMPED_VALID)) != MASK_STS_IMPED_VALID) {
			//printf("waiting imp valid..0x%x %d\n\r", sts, j);
			usleep(1000);
			j++;
			if (j==10)
				break;
		}

		//TODO: Average is still not working well.
		//ad5933_reg_read(h, AD5933_CTRL_REG_MSB, &ctrl_inc);
		if (avg_points < NUM_AVG_POINTS) {
			ad5933_imped_calc(h, &imp_data_avg, *gf);
			m[avg_points] = imp_data_avg.m;
			magnitude[avg_points] = imp_data_avg.magnitude;
			phase[avg_points] = imp_data_avg.phase;
			//printf("m = %04f\n\r", m[avg_points]);
			ctrl_inc = MASK_REPEAT_FREQ;
			ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_inc);
			//printf("point %d\n\r", avg_points);
			avg_points++;
		}
		else {
			//ad5933_imp_avg_calc(&imp_data[sample], m, magnitude, phase);
			ad5933_imped_calc(h, &imp_data[sample], *gf);
			ctrl_inc = MASK_INC_FREQ;
			ad5933_reg_write(h, AD5933_CTRL_REG_MSB, ctrl_inc);
			avg_points = 0;
			sweep_ok = ad5933_get_status(h) & MASK_STS_FRQ_SWP_RDY;
			//printf("sweep ok 0x%x i = %f\n\r", sweep_ok, step);
			if (step < (NUM_SAMPLES*FREQ_STEP))
				sweep_ok = 0;
			step += FREQ_STEP;
			sample++;
		}
	}
	/*two measures, we are considering that it varies linearly with the frequency.
	 * The smaller the difference (delta F) in frequency, the better */
	if (cal) {
		m1 = imp_data[0].m;
		m2 = imp_data[NUM_SAMPLES-1].m;
		*gf = (pow(10,9)/AD5933_RES_CALIB_VAL)/( ((m2-m1)/2) + m2);
		//printf("m1 = %04f m2 %04f\n\r", m1, m2);
	}
	for (i=0; i<NUM_SAMPLES;i++) {
		mag[i] = imp_data[i].magnitude;
		ph[i] = imp_data[i].phase;
	}
	
	imped->magnitude = dataMean(mag, NUM_SAMPLES);
	imped->phase = dataMean(ph, NUM_SAMPLES);
	
	return (ret);
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
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, control_reg_msb);

	printf("Temperature %s !!\n\r", (ad5933_get_status(h) & MASK_STS_TEMP_VALID) ? "Ready" : "Not Ready");

	ad5933_reg_read(h, AD5933_TEMP_DATA_MSB, &t0);
	ad5933_reg_read(h, AD5933_TEMP_DATA_LSB, &t1);

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

static char
data_save(st_imped_data_t imped, unsigned int t)
{
	FILE *fd;

	if ( (fd = fopen("data/imped_data.txt", "a+")) == NULL)
	{
		fprintf(stderr, "Error opening file for writing!\n\r");
		return (-1);
	}

	fprintf(fd, "%d\t%f\t%f\n\r", t, imped.magnitude, imped.phase);

	fclose(fd);

	return (0);
}

/********************************************************************************/

static char
ad5933_data_collect(usb_dev_handle *h, double gf)
{
	unsigned int t = 0;
	unsigned int i;
	unsigned char pd = MASK_PD_MODE;
	st_imped_data_t imped;

	while(1)
	{
		bzero(&imped, sizeof(imped));
		ad5933_imped_sweep(h, 0, &gf, &imped);
		ad5933_reg_write(h, AD5933_CTRL_REG_MSB, pd);
		printf("t = %d Z = %04f PH: %04f \n\r", t, imped.magnitude, imped.phase);;
		data_save(imped, t);

		/*wait a minute*/
		for (i = 0; i < MEASURE_INTERVAL; i++) {
			fprintf(stdout, "%d\r", i);
			fflush(stdout);
			sleep(1);
		}
		printf("\n");
		t++;
	}

	return (0);
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
	struct usb_dev_handle *h = NULL;
	const char *file = "../fw/hex/AD5933_34FW.hex";
	//const char *file = "../fw/hex/hello.ihx";
	unsigned int err = 0;
	unsigned char buff_w = 0;
	st_imped_data_t imped;
	st_imped_data_raw_t ir;
	double gf = 0.0;
	unsigned char pd = MASK_PD_MODE;

	bzero(&imped, sizeof(imped));
	bzero(&ir, sizeof(ir));

	/* Initializing libusb*/
	usb_init();
	//usb_set_debug(2);

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
	if ( !(h = xusb_open(usbdev)) )
		return(1);

	printf("%d\n\r", usb_set_configuration(h,1)) ;
	printf("%d\n\r", usb_claim_interface(h,0)  ) ;
	printf("%d\n\r", usb_set_altinterface(h,1) );

	//usb_reset(h);
	//fprintf(stdout, "Programming 8051 using %s \n", file);
	//err += xusb_program_hex_file(file, h);

	ad5933_reset(h);
	printf("Calibrating...\n\r");
	ad5933_imped_sweep(h, 1, &gf, &imped);
	printf("GF = %04f\n\r", gf);
	printf("Running!\n\r");
	ad5933_data_collect(h, gf);

	ad5933_get_temperature(h);

	printf("Power Down\n\r");
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, pd);

    //usb_close(h);
	return (0);
}


