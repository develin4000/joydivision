/* Name: usbcalls.c
 * Project: usbcalls library
 * Author: Christian Starkjohann
 * Creation Date: 2006-02-02
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: Proprietary, free under certain conditions. See Documentation.
 * This Revision: $Id$
 */

/* This file includes the appropriate implementation based on platform
 * specific defines.
 */


#ifdef __MORPHOS__ 
# include "usb-morphos.c"
#elif defined __AROS__
# include "usb-aros.c"
#elif defined __amigaos4__
# include "usb-amigaos4.c"
#elif defined WIN32
# include "usb-windows.c"
#else
# include "usb-libusb.c"
#endif

/*
#ifdef __MORPHOS__
#   include "usb-morphos.c"
#else

#if defined(WIN32)
#   include "usb-windows.c"
#else
// e.g. defined(__APPLE__)
#   include "usb-libusb.c"
#endif
#endif
*/
