/********************************************************************************/
/**
 * \name ad5933_utils.h
 *
 * This file implements the statistical analysis. All functions not related
 * to ad5933 whatsoever should be here.
 *
 * Supported OSs: Mac OSx
 *
 * Copyright 2013 David Cemin <davidcemin at gmail dot com>
  
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 */
/********************************************************************************/

#include <stdio.h>
#include <strings.h> 	/*bzero*/
#include <math.h>
#include <usb.h>
#include <stdint.h>


/********************************************************************************/

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
#define NUM_AVG_POINTS			(5)
#define NUM_SAMPLES				(200)
#define FREQ_STEP				(1)
#define FREQ_LOW				(60.0)
#define MEASURE_INTERVAL		(10) /*seconds*/
#define FREQ_HIGH				(FREQ_LOW+NUM_SAMPLES*FREQ_STEP)

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

/**
 * \brief  Writes register inside ad5933 (through usb->i2c interfaces)
 * \param  h Pointer to ad5933/usb device handle
 * \param  addr Address to write
 * \param  val Value to write at addr
 * \return Number of written bytes (always 1), < 0 in case of error
 */
inline int
ad5933_reg_write(usb_dev_handle *h, unsigned char addr, unsigned char val);

/********************************************************************************/

/**
 * \brief  Reads registers inside ad5933 (through usb->i2c interfaces)
 * \param  h Pointer to ad5933/usb device handle
 * \param  addr Address to read
 * \param  val Pointer to read value at addr
 * \return Number of read bytes (always 1), < 0 in case of error
 */
inline int
ad5933_reg_read(usb_dev_handle *h, unsigned char reg, unsigned char *val);

/********************************************************************************/

/**
 * \brief  Read status register
 * \param  h Handler to usb
 * \return Status register content, < 0 in case of error
 */
inline char
ad5933_get_status(usb_dev_handle *h);

/********************************************************************************/

/**
 * \brief Resets ad5933 chip
 * \param h Handler to usb
 * \return 1 in success (number of bytes written), < 0 in case of error;
 */
inline int
ad5933_reset(usb_dev_handle *h);

/********************************************************************************/

/**
 * \brief Reads AD5933 temperature sensor
 * \param h Pointer to usb handler
 * \return Temperature (16 bits) 
 */
inline uint16_t
ad5933_get_temperature(usb_dev_handle *h);

/********************************************************************************/

/**
 * \brief Read impedance data (real and imaginary)
 * \param h Pointer to usb handler
 * \param ir Pointer to impedance_raw structure
 * \return Number of read bytes, < 0 in case of at least 3 errors
 */
inline int
ad5933_imped_read(usb_dev_handle *h, st_imped_data_raw_t *ir);

/********************************************************************************/

/**
 * \brief Calcuates impedance magnitude (Z)
 * \param ir Structure containing impedance data
 * \return Calculated magnitude
 */
inline double
ad5933_magnitude_calc(st_imped_data_raw_t ir);

/********************************************************************************/

/**
 * \brief Calculate impedance's phase
 * \param ir Structure containing impedance data
 * \return Calculated phase
 */
inline double
ad5933_phase_calc(st_imped_data_raw_t ir);

/********************************************************************************/

/**
 * \brief Calculates current impedance
 * \param h Pointer to usb handler
 * \param imped Pointer to impedance structure
 * \param gf Calculated gain
 * \return 0
 */
inline char
ad5933_imped_calc(usb_dev_handle *h, st_imped_data_t *imped, double gf);

/********************************************************************************/

/**
 * \brief Writes frequency values into ad5933 registers
 * \param h USB handler
 * \param freq Frequency in KHz
 * \return Number of written bytes (3), < 0 in case of error
 */
inline int
ad5933_start_freq_calc(usb_dev_handle *h, double freq);

/********************************************************************************/

/**
 * \brief Writes NUMBER_OF_INCREMENTS register
 * \param h Usb Handler
 * \param inc Number of increments (steps)
 */
inline int
ad5933_numb_incr_set(usb_dev_handle *h, unsigned int inc);

/********************************************************************************/

/**
 * \brief Writes step value into ad5933 registers
 * \param h USB handler
 * \param step Step value (is a float number, so it accepts 0.1, 1.32 etc).
 * Step is measured in Hertz (HZ).
 * \return Number of written bytes (3), < 0 in case of error; 
 */
inline int
ad5933_freq_inc_set(usb_dev_handle *h, double step);

/********************************************************************************/

/**
 * \brief Writes settling time registers
 * \param h Pointer to USB handler
 * \param s Settling time (cycles, so is an integer)
 * \param mult Multiplier: 0 1 or 3:
 * 0 = Number of cycles x 1;
 * 1 = Number of cycles x 2;
 * 3 = Number of cycles x 4;
 * Please refer to AD5933 datasheet for more details
 * \return Number of written bytes (2), < 0 in case of error
 */
inline int
ad5933_settling_set(usb_dev_handle *h, unsigned int s, unsigned char mult);

/********************************************************************************/

/**
 * \brief Stores arrays into an average value inside imp_data structure
 * \param imp_data Pointer to impedance data structure
 * \param m vector containing read magnitude (not the real one after gf)
 * \param magnitude Vector containing calculated magnitude
 * \param phase Vector containing phase
 * \return 0
 */
inline char
ad5933_imp_avg_calc(st_imped_data_t *imp_data, double *m, double *magnitude, double *phase);

/********************************************************************************/

/**
 * \brief Performs a Impedance sweep
 * \param h Pointer to USB handler
 * \param cal 1 = calibrate (writes gf pointer), 0 = do not calibrate
 * \param gf Pointer to GF (gf = gf x 10e9). It will be overwritten when cal = 1
 * \param imped Pointer to impedance data
 * \return 0 ok, -1 error;
 */
inline char
ad5933_imped_sweep(usb_dev_handle *h, unsigned char cal, double *gf, st_imped_data_t *imped);

/********************************************************************************/

/**
 * \brief Infinite loop collecting data. Config is done through macros.
 * \param h Pointer to usb handler
 * \param gf Gain times 10e9
 * \return 0
 */
inline char
ad5933_data_collect(usb_dev_handle *h, double gf);

/********************************************************************************/

/**
 * \brief Saves impedance data into a file called imped_data.txt
 * \param imped Structure containing both Z and PH data;
 * \param t Current time (in seconds)
 * \return -1 error saving data, 0 success.
 */
char data_save(st_imped_data_t imped, unsigned int t);

/********************************************************************************/


