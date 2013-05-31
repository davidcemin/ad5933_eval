/********************************************************************************/
/**
 * \name xusb.c
 * This file contains the low level routines to communicate through usb interface.
 * It uses libusb as wrapper.
 *
 * Supported OSs: Mac OSx
 *
 * Some functions here were based on cycfx2dev.cc - Cypress FX2 device class by
 * Wolfgang Wieser ] wwieser (a) gmx <*> de
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
#include <errno.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/time.h>

#include "xusb.h"

/********************************************************************************/
/* Private stuff */
/********************************************************************************/

/**
 * \brief  Program one line of an Intel Hex file
 * \param  buf Pointer to data buffer
 * \param  path Path to the file. Used in error reporting
 * \param  line Line where an error has occured.
 * \param  usbdevhandle Pointer to usb dev handle.
 * \return 1 in case of error, -1 EOF, 0 otherwise
 */
static int
xusb_program_hex_line(const char *buf, const char *path, int line, usb_dev_handle *usbdevhandle)
{
	const char *s = buf;
	unsigned int nbytes = 0;
	unsigned int addr = 0;
	unsigned int type = 0;

	if (*s != ':')
	{
		fprintf(stderr, "%s:%d: Format violation (1)\n", path, line);
		return (1);
	}
	++s;

	if (sscanf(s, "%02x%04x%02x", &nbytes, &addr, &type) != 3)
	{
		fprintf(stderr, "%s:%d: Format violation (2)\n", path, line);
		return (1);
	}
	s += 8;

	switch (type)
	{
		case 0:
			{
				assert(nbytes > 0 && nbytes < 256);
				unsigned char data[nbytes];
				unsigned char chsum = nbytes+addr+(addr>>8)+type;
				unsigned int file_chsum = 0;
				unsigned int i;

				for (i = 0; i < nbytes; i++)
				{
					unsigned int d = 0;
					if (sscanf(s, "%02x", &d) != 1)
					{
						fprintf(stderr, "%s:%d: Format violation (3)\n", path, line);
						return (1);
					}
					s += 2;
					data[i] = d;
					chsum += d;
				}
				if (sscanf(s, "%02x", &file_chsum) != 1)
				{
					fprintf(stderr, "%s:%d: Format violation (4)\n", path, line);
					return (1);
				}
				if ((chsum+file_chsum) & 0xff)
				{
					fprintf(stderr, "%s:%d: Checksum mismatch (%u/%u)\n", path, line, chsum, file_chsum);
					return (1);
				}
				if (xusb_write_ram(addr, data, nbytes, usbdevhandle) )
					return (1);
			}
			break;
		case 1:
			/*EOF Marker*/
			return (-1);
		default:
			fprintf(stderr, "%s:%d: Unknown entry type %d\n", path, line, type);
			return (1);
	}

	return (0);
}
/********************************************************************************/


/********************************************************************************/
/* Public stuff */
/********************************************************************************/
int 
xusb_open(struct usb_device *_usbdev)
{
	struct usb_device *usbdev = _usbdev;
	usb_dev_handle *usbhandle;
	//xusb_close();

	usbhandle = usb_open(usbdev);
	if (!usbhandle)
   	{
		fprintf(stderr, "Failed to open device: %s\n", usb_strerror());
		return -1;
	}

	return (0);
}

/********************************************************************************/

int
xusb_close(usb_dev_handle * usbhandle)
{
	int rv = 0;
	if (usbhandle)
   	{
		rv = usb_close(usbhandle);
		usbhandle = NULL;
		if (rv)
			fprintf(stderr, "Closing USB device: %s\n", usb_strerror());
	}
	return (rv);
}

/********************************************************************************/

int
xusb_write_ram(size_t addr, const unsigned char *data, size_t nbytes, usb_dev_handle * usbdevhandle)
{
	int n_errors = 0;
	const size_t chunk_size = 16;
	const unsigned char *d = data;
	const unsigned char *dend = data+nbytes;
	
	if (!usbdevhandle)
   	{
		fprintf(stderr, "write_ram: Device not connected!\n");
		return(-1);
	}

	while(d<dend) 
	{
		size_t bs = dend-d;
		size_t dl_addr = addr+(d-data);
		int requesttype = 0x40; /*TODO*/
		int request = 0xa0; /*TODO*/
		if (bs > chunk_size)
			bs = chunk_size;
		if (usb_control_msg( usbdevhandle, requesttype, request, dl_addr, 0, (char*)d, bs, 1000) < 0)
		{
			fprintf(stderr, "Writing %zu bytes at 0x%zx: %s\n", 
					bs, dl_addr, usb_strerror());
			++n_errors;
		}
		d += bs;
	}
	return (n_errors);
}

/********************************************************************************/

int
xusb_read_ram(size_t addr, unsigned char *data, size_t nbytes, usb_dev_handle *usbdevhandle)
{
	int n_errors = 0;
	const size_t chunk_size = 16;
	unsigned char *d = data;
	unsigned char *dend = data + nbytes;

	if (!usbdevhandle)
	{
		fprintf(stderr, "read_ram: Device not connected!\n");
		return (-1);
	}

	while(d<dend)
	{
		size_t bs = dend-d;
		size_t rd_addr = addr+(d-data);
		int requesttype = 0xc0; /*TODO*/
		int request = 0xa0;
		if (bs > chunk_size)
			bs = chunk_size;
		if ( usb_control_msg (usbdevhandle, requesttype, request, rd_addr, 0, (char*)d, bs, 1000)< 0 )
		{
			fprintf(stderr, "Reading %zu bytes at 0x%zx: %s\n", bs, rd_addr, usb_strerror());
			++n_errors;
		}
		d += bs;
	}
	return (n_errors);
}
/********************************************************************************/

int
xusb_reset(unsigned char running, usb_dev_handle *usbdevhandle)
{
	unsigned char val = running ? 0 : 1;

	return (xusb_write_ram(XUSB_RESET_ADDR, &val, 1, usbdevhandle) );
}

/********************************************************************************/

int
xusb_program_hex_file(const char *path, usb_dev_handle *usbdevhandle)
{
	FILE *fp;
	int n_errors = 0;
	const size_t buflen = 1024;
	char buf[buflen];
	int line = 1;

	if ( !usbdevhandle )
	{
		fprintf(stderr, "program_hex_file: Not connected!\n");
		return (1);
	}
	if ( !(fp = fopen(path, "r") ) )
	{
		fprintf(stderr, "Failed to open %s: %s\n", path, strerror(errno));
		return (2);
	}

	for(;;++line)
	{
		int rv = 0;
		*buf='\0';
		if (!fgets(buf, buflen, fp))
		{
			if (feof(fp)) break;
			fprintf(stderr, "Reading %s (line %d): %s\n", path, line, strerror(ferror(fp)));
			fclose(fp);
			fp = NULL;
			return (3);
		}
		rv = xusb_program_hex_line(buf, path, line, usbdevhandle);
		if (rv < 0)
			break;
		if (rv)
			++n_errors;
	}
	if (fp)
		fclose(fp);

	return (n_errors ? -1 : 0);
}

/********************************************************************************/


