/*
->================================================================================<-
->= JoyDivision - USB joystick adapter - (c) Copyright 2016-2019 OnyxSoft        =<-
->================================================================================<-
->= Version  : 0.4                                                               =<-
->= File     : joystick.c                                                        =<-
->= Author   : Stefan Blixth (stefan@onyxsoft.se)                                =<-
->= Compiled : 2019-12-24                                                        =<-
->================================================================================<-
->=                                                                              =<-
->= This file is part of JoyDivision - USB joystick adapter                      =<-
->=                                                                              =<-
->= JoyDivision is free software; you can redistribute it and/or modify          =<-
->= it under the terms of the GNU General Public License as published by         =<-
->= the Free Software Foundation; either version 2 of the License, or            =<-
->= (at your option) any later version.                                          =<-
->=                                                                              =<-
->= This program is distributed in the hope that it will be useful,              =<-
->= but WITHOUT ANY WARRANTY; without even the implied warranty of               =<-
->= MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                =<-
->= GNU General Public License for more details.                                 =<-
->=                                                                              =<-
->= You should have received a copy of the GNU General Public License along      =<-
->= with this program; if not, write to the Free Software Foundation, Inc.,      =<-
->= 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.                  =<-
->=                                                                              =<-
->= This project is based on V-USB code from Objective Development Software GmbH =<-
->= http://www.obdev.at/products/vusb/                                           =<-
->=                                                                              =<-
->= Partial based on code/ideas from Raphaël Assénat                             =<-
->= http://www.raphnet.net/                                                      =<-
->=                                                                              =<-
->= Partial based on code/ideas from Andreas Paul                                =<-
->= http://www.hexagons.de/                                                      =<-
->=                                                                              =<-
->================================================================================<-
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#include "usbdrv.h"
#include "oddebug.h"

#define SETBIT(ADDRESS,BIT)   ((ADDRESS) |= (1<<(BIT))) 
#define CLEARBIT(ADDRESS,BIT) ((ADDRESS) &= ~(1<<(BIT))) 
#define TESTBIT(ADDRESS,BIT)  ((ADDRESS) & (1<<(BIT)))

#define JOYPORT1   0
#define JOYPORT2   1

#ifdef DUAL_JOYDIVISION

// USB configuration descriptor
PROGMEM const char usbDescriptorConfiguration[] = {
  9,                                       // sizeof(usbDescriptorConfiguration): length of descriptor in bytes
  USBDESCR_CONFIG,                         // descriptor type
  9 + (2*(9+9+7)), 0,                      // total length of data returned (including inlined descriptors)
  2,                                       // number of interfaces in this configuration
  1,                                       // index of this configuration
  0,                                       // configuration name string index
  USBATTR_BUSPOWER,                        // attributes
  USB_CFG_MAX_BUS_POWER/2,                 // max USB current in 2mA units

  // interface 1 descriptor follows inline:
  9,                                       // sizeof(usbDescrInterface): length of descriptor in bytes
  USBDESCR_INTERFACE,                      // descriptor type
  JOYPORT1,                                // index of this interface
  0,                                       // alternate setting for this interface
  USB_CFG_HAVE_INTRIN_ENDPOINT,            // endpoints excl 0: number of endpoint descriptors to follow
  USB_CFG_INTERFACE_CLASS,
  USB_CFG_INTERFACE_SUBCLASS,
  USB_CFG_INTERFACE_PROTOCOL,
  3,                                       // string index for interface
  9,                                       // sizeof(usbDescrHID): length of descriptor in bytes
  USBDESCR_HID,                            // descriptor type: HID
  0x01, 0x01,                              // BCD representation of HID version
  0x00,                                    // target country code
  0x01,                                    // number of HID Report (or other HID class) Descriptor infos to follow
  0x22,                                    // descriptor type: report
  USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH, 0, // total length of report descriptor

  // endpoint descriptor for endpoint 1
  7,                                       // sizeof(usbDescrEndpoint)
  USBDESCR_ENDPOINT,                       // descriptor type = endpoint
  (char)0x81,                              // IN endpoint number 1
  0x03,                                    // attrib: Interrupt endpoint
  8, 0,                                    // maximum packet size
  USB_CFG_INTR_POLL_INTERVAL,              // in ms

  // interface 2 descriptor follows inline:
  9,                                       // sizeof(usbDescrInterface): length of descriptor in bytes
  USBDESCR_INTERFACE,                      // descriptor type
  JOYPORT2,                                // index of this interface
  0,                                       // alternate setting for this interface
  1,                                       // endpoints excl 0: number of endpoint descriptors to follow
  USB_CFG_INTERFACE_CLASS,
  USB_CFG_INTERFACE_SUBCLASS,
  USB_CFG_INTERFACE_PROTOCOL,
  4,                                       // string index for interface
  9,                                       // sizeof(usbDescrHID): length of descriptor in bytes
  USBDESCR_HID,                            // descriptor type: HID
  0x01, 0x01,                              // BCD representation of HID version
  0x00,                                    // target country code
  0x01,                                    // number of HID Report (or other HID class) Descriptor infos to follow
  0x22,                                    // descriptor type: report
  USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH, 0, // total length of report descriptor

  // endpoint descriptor for endpoint 3
  7,                                       // sizeof(usbDescrEndpoint)
  USBDESCR_ENDPOINT,                       // descriptor type = endpoint
  (char)(0x80 | USB_CFG_EP3_NUMBER),       // IN endpoint number 3
  0x03,                                    // attrib: Interrupt endpoint
  8, 0,                                    // maximum packet size
  USB_CFG_INTR_POLL_INTERVAL,              // in ms
};

#endif


const char usbHidReportDescriptor[] PROGMEM =
{
    // Joystick
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
      0xa1, 0x02,                  //   COLLECTION (Physical)

#ifdef THEC64
         // Axis
         0x05, 0x01,               //     USAGE_PAGE (Generic Desktop)
         0x09, 0x30,               //     USAGE (X)
         0x09, 0x31,               //     USAGE (Y)
         0x15, 0x00,               //     LOGICAL_MINIMUM (0)
         0x25, 0xff,               //     LOGICAL_MAXIMUM (255)
         0x75, 0x08,               //     REPORT_SIZE (8)
         0x95, 0x02,               //     REPORT_COUNT (2)
         0x81, 0x02,               //     INPUT (Data,Var,Abs)

         // Buttons
         0x05, 0x09,               //     USAGE_PAGE (Button)

         0x19, 0x01,               //     USAGE_MINIMUM (Button 1)
         0x29, 0x08,               //     USAGE_MAXIMUM (Button 8)
         0x15, 0x00,               //     LOGICAL_MINIMUM (0)
         0x25, 0x01,               //     LOGICAL_MAXIMUM (1)
         0x95, 0x08,               //     REPORT_COUNT (8)
         0x75, 0x01,               //     REPORT_SIZE (1)
         0x81, 0x02,               //     INPUT (Data,Var,Abs)
#else
         // Axis
         0x05, 0x01,               //     USAGE_PAGE (Generic Desktop)
         0x09, 0x30,               //     USAGE (X)
         0x09, 0x31,               //     USAGE (Y)
         0x15, 0x81,               //     LOGICAL_MINIMUM (-127)
         0x25, 0x7f,               //     LOGICAL_MAXIMUM (127)
         0x75, 0x08,               //     REPORT_SIZE (8)
         0x95, 0x02,               //     REPORT_COUNT (2)
         0x81, 0x02,               //     INPUT (Data,Var,Abs)

         // Buttons
         0x05, 0x09,               //     USAGE_PAGE (Button)

         0x19, 0x01,               //     USAGE_MINIMUM (Button 1)
         0x29, 0x03,               //     USAGE_MAXIMUM (Button 3)
         0x15, 0x00,               //     LOGICAL_MINIMUM (0)
         0x25, 0x01,               //     LOGICAL_MAXIMUM (1)
         0x95, 0x03,               //     REPORT_COUNT (3)

         0x75, 0x01,               //     REPORT_SIZE (1)
         0x81, 0x02,               //     INPUT (Data,Var,Abs)
         0x95, 0x01,               //     REPORT_COUNT (1)
         0x75, 0x05,               //     REPORT_SIZE (5)
         0x81, 0x03,               //     INPUT (Constant,Var,Abs)
#endif

      0xc0,                        //   END_COLLECTION
    0xc0                           // END_COLLECTION
};


typedef struct
{
#ifdef THEC64
   uchar axis_x;
   uchar axis_y;
#else
   char axis_x;
   char axis_y;
#endif
   uchar buttons;
}report_t;


#ifdef DUAL_JOYDIVISION
static report_t reportBuffer[2];
 static PROGMEM const char DeviceStr[] = "Dual JoyDivision #x";
 static PROGMEM const char VendorStr[] = "OnyxSoft";
static const char Desc0Str[] = { 4, 3, 0x09, 0x04 };

usbMsgLen_t usbFunctionDescriptor(struct usbRequest *rq)
{
  static char buf[(sizeof(DeviceStr) > sizeof(VendorStr) ? sizeof(DeviceStr) : sizeof(VendorStr))*2];

  uchar ix = rq->wValue.bytes[0];
  const char* s;
  uchar l;
  uchar ifchar = 0;
  if (ix == 0) { // String0
    usbMsgPtr = (usbMsgPtr_t)Desc0Str;
    return 4;
  }
  else if (ix == 1) { // Vendor
    s = VendorStr;
    l = sizeof(VendorStr) * 2;
  }
  else if (ix == 2) { // Product
    s = DeviceStr;
    l = (sizeof(DeviceStr) - 3) * 2;
  }
  else if (ix == 3 || ix == 4) { // Interface 1 or 2
    s = DeviceStr;
    l = sizeof(DeviceStr) * 2;
    ifchar = '1' + ix - 3;
  }
  else {
    return 0;
  }

  // copy from ROM to RAM with char to wchar conversion
  char* d = buf + 2;
  char c;
  while ((c = pgm_read_byte(s++)) != 0) {
    *d = c;
    d += 2;
  }

  buf[0] = l; // length
  buf[1] = 3; // string descriptor mark
  if (ifchar)
    buf[(sizeof(DeviceStr)-1) * 2] = ifchar; // patch interface number
  usbMsgPtr = (usbMsgPtr_t)buf;
  return l;
}

#else
static report_t reportBuffer;
#endif

static uchar idleRate = 0;            // repeat rate for keyboards, never used for mice
static uchar currState = 0;           // Keeps track on the current state


report_t buildReport(uchar joyport)
{
#ifdef THEC64
   uchar axis_x=0x7f, axis_y=0x7f;
   char thec64 = 1;
#else
   char axis_x=0, axis_y=0;
   char thec64 = 0;
#endif
   uchar b1=0, b2=0, b3=0, tmp;
   report_t activeBuffer;
   
#ifdef DUAL_JOYDIVISION
   activeBuffer = reportBuffer[joyport];
#else
   activeBuffer = reportBuffer;
#endif
   
   if (joyport == JOYPORT1)
   {
      currState = PINC;
      tmp = currState^0xff;

      // Read the status of the axis...
      if (tmp & (1<<PC2)) {axis_x = (thec64) ? 0xff : 0x7f;} // Right
      if (tmp & (1<<PC3)) {axis_x = (thec64) ? 0x00 : 0x81;} // Left
      if (tmp & (1<<PC4)) {axis_y = (thec64) ? 0xff : 0x7f;} // Down
      if (tmp & (1<<PC5)) {axis_y = (thec64) ? 0x00 : 0x81;} // Up

      // Read the status of the buttons...
      if (tmp & (1<<PC1)) {b1 = 1;} // Button 1
      if (tmp & (1<<PC0)) {b3 = 1;} // Button 3

      currState = PINB;
      tmp = currState^0xff;

      if (tmp & (1<<PB0)) {b2 = 1;} // Button 2
   }
   else // joyport == JOYPORT2
   {
      currState = PIND;
      tmp = currState^0xff;

      // Read the status of the axis...
      if (tmp & (1<<PD4)) {axis_x = (thec64) ? 0xff : 0x7f;} // Right
      if (tmp & (1<<PD5)) {axis_x = (thec64) ? 0x00 : 0x81;} // Left
      if (tmp & (1<<PD6)) {axis_y = (thec64) ? 0xff : 0x7f;} // Down
      if (tmp & (1<<PD7)) {axis_y = (thec64) ? 0x00 : 0x81;} // Up

      // Read the status of the buttons...
      if (tmp & (1<<PD3)) {b3 = 1;} // Button 3

      currState = PINB;
      tmp = currState^0xff;

      if (tmp & (1<<PB2)) {b1 = 1;} // Button 1
      if (tmp & (1<<PB4)) {b2 = 1;} // Button 2

   }
   activeBuffer.axis_x  = axis_x;
   activeBuffer.axis_y  = axis_y;

#ifdef THEC64
   if (b1) activeBuffer.buttons |= 0x41;
#else
   if (b1) activeBuffer.buttons |= 0x01;
#endif
   if (b2) activeBuffer.buttons |= 0x02;
   if (b3) activeBuffer.buttons |= 0x04;

   return activeBuffer;
}

uchar usbFunctionSetup(uchar data[8])
{
   usbRequest_t *rq = (void *)data;

   if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS)
   {
      if(rq->bRequest == USBRQ_HID_GET_REPORT)
      {  
         usbMsgPtr = (usbMsgPtr_t)&reportBuffer;
         return sizeof(reportBuffer);
      }
      else if(rq->bRequest == USBRQ_HID_GET_IDLE)
      {
         usbMsgPtr = (usbMsgPtr_t)&idleRate;
         return 1;
      }
      else if(rq->bRequest == USBRQ_HID_SET_IDLE)
      {
         idleRate = rq->wValue.bytes[1];
         return 0;
      }
   }

   return 0;
}


static void hardwareInit(void)
{
   uchar i, j; // Test for USB Reset

   /*      1 _____________ 5
             \ o o o o o /
              \ o o o o /
            6  ¯¯¯¯¯¯¯¯¯ 9

   Pin   Joystick     AVR-Pin (Port 1)   AVR-Pin (Port 2)
   1     Up           PC5                PD7
   2     Down         PC4                PD6
   3     Left         PC3                PD5
   4     Right        PC2                PD4
   5     n/c          PC0                PD3
   6     Button1      PC1                PB2
   7     +5V          PB1                PB3
   8     GND          GND                GND
   9     Button2      PB0                PB4  */

   // Init PortB
   DDRB  = 0x00;
   PORTB = 0b00110101;

   // Init PortC
   DDRC  = 0x00;
   PORTC = 0b00111111;

   // Init PortD
   DDRD  = 0x00;
   PORTD = 0b11111001;

   j = 0;
   while(--j)
   {              /* USB Reset by device only required on Watchdog Reset */
      i = 0;
      while(--i); /* delay >10ms for USB reset */
   }

   DDRD = 0x00;    /* 0000 0000 bin: remove USB reset condition */
                   /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
   TCCR0 = 5;      /* timer 0 prescaler: 1024 */

   TCCR2 = (1<<WGM21)|(1<<CS22)|(1<<CS21)|(1<<CS20);
   OCR2 = 196;     // for 60 hz

/*
   // Enable following if you own a JoyFi wireless transmitter/reciever -> http://store.ribit.se/
   SETBIT(DDRB, PB1);  // Enable PB1 as an output
   SETBIT(PORTB, PB1); // Enable PB1 / VCC
   SETBIT(DDRB, PB3);  // Enable PB3 as an output
   SETBIT(PORTB, PB3); // Enable PB3 / VCC
*/
}


int __attribute__((noreturn)) main(void)
{
   uchar i;
   report_t reportBufferPort[2];

   wdt_enable(WDTO_2S);
   hardwareInit();
   odDebugInit();

   usbInit();
   usbDeviceDisconnect();  // enforce re-enumeration, do this while interrupts are disabled!
   i = 0;

   while(--i)
   {                       // fake USB disconnect for > 250 ms
      wdt_reset();
      _delay_ms(1);
   }

   usbDeviceConnect();
   sei();

   for(;;)
   {
      wdt_reset();
      usbPoll();

      if (usbInterruptIsReady())
      {
         // Add check to only report when changes have been made...
         reportBufferPort[JOYPORT1] = buildReport(JOYPORT1);
         usbSetInterrupt((void *)&reportBufferPort[JOYPORT1], sizeof(report_t));

#ifdef DUAL_JOYDIVISION
         reportBufferPort[JOYPORT2] = buildReport(JOYPORT2);
         usbSetInterrupt3((void *)&reportBufferPort[JOYPORT2], sizeof(report_t));
#endif
      }
   }
}

