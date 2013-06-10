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


