/********************************************************************************/
/**
 * \name xusb.h
 * This file contains the low level routines to communicate through usb interface.
 * It uses libusb as wrapper.
 *
 * Supported OSs: Mac OSx
 *
 * Copyright 2013 David Cemin <davidcemin at gmail dot com>
 */
/********************************************************************************/

#ifndef __XUSB_H__
#define __XUSB_H__ 1

#include <usb.h>

/********************************************************************************/

#define XUSB_RESET_ADDR		0xE600

/********************************************************************************/

/**
 * \brief Open usb device
 * \param _usbdev Pointer to usb device that will be opened
 * \return 0 on success, 1 on error.
 */
int
xusb_open(struct usb_device *_usbdev);

/********************************************************************************/

/**
 * \brief Close usb device
 * \param usbhandle Pointer to usb_dev_handle structure
 * \return 0 on success
 */
int
xusb_close(usb_dev_handle * usbhandle);

/********************************************************************************/

/**
 * \brief  Writes data into device's ram
 * \param  addr Start address
 * \param  data Pointer to data to write
 * \param  nbytes Size of data
 * \param  usbdevhandle Pointer to usb dev handle structure
 * \return Number of writing errors, -1 if dev is closed
 */
int
xusb_write_ram(size_t addr, const unsigned char *data, size_t nbytes, usb_dev_handle * usbdevhandle);
/********************************************************************************/

/**
 * \brief  Reads data from device's ram
 * \param  addr Start address
 * \param  data Pointer to data to write
 * \param  nbytes Size of data
 * \param  usbdevhandle Pointer to usb dev handle structure
 * \return Number of writing errors, -1 if dev is closed
 */
int
xusb_read_ram(size_t addr, unsigned char *data, size_t nbytes, usb_dev_handle *usbdevhandle);

/********************************************************************************/

/**
 * \brief  Put USB device in a reset state
 * \param  running Flag: 1 = running; 0 = Reset;
 * \param  usbdevhandle Pointer to usb dev handle structure
 * \return Same as xusb_write_ram()
 */
int
xusb_reset(unsigned char running, usb_dev_handle *usbdevhandle);

/********************************************************************************/

/**
 * \brief  Reads an Intel HEX file and program the device
 * \param  path Path to the file
 * \param  usbdevhandle Pointer to opened usb device
 * \return 0 ok, 1 errors on writing, <0 other kind of errors
 */
int
xusb_program_hex_file(const char *path, usb_dev_handle *usbdevhandle);

/********************************************************************************/

#endif /*__XUSB_H__*/