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


#include <stdio.h>
#include <stdlib.h>

#include "xusb.h"
#include "ad5933.h"

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

	/* Initializing libusb*/
	usb_init();

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

	return (0);
}


