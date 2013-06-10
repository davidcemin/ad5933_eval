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

#include "ad5933.h"
#include "ad5933_utils.h" 

/********************************************************************************/
/* Private stuff																*/
/********************************************************************************/


/********************************************************************************/
/* Public stuff																	*/
/********************************************************************************/

int
main (int argc, char **argv)
{
	int vendor_id = 0x0456; /*Analog Devices*/
	int prod_id = 0xb203;  /*Prod id =)*/
	int rv = 0;
	struct usb_device *usbdev = NULL;
	struct usb_dev_handle *h = NULL;
	const char *file = "../fw/hex/AD5933_34FW.hex";
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
	ad5933_get_temperature(h);
	printf("Calibrating...\n\r");
	ad5933_imped_sweep(h, 1, &gf, &imped);
	printf("GF = %04f\n\r", gf);
	printf("Running!\n\r");
	ad5933_data_collect(h, gf);

	printf("Power Down\n\r");
	ad5933_reg_write(h, AD5933_CTRL_REG_MSB, pd);

    //usb_close(h);
	return (0);
}


