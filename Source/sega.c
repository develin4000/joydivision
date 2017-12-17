/*
->================================================================================<-
->= JoyDivision - USB joystick adapter - (c) Copyright 2016-2017 OnyxSoft        =<-
->================================================================================<-
->= Version  : 0.3                                                               =<-
->= File     : sega.c                                                            =<-
->= Author   : Stefan Blixth (stefan@onyxsoft.se)                                =<-
->= Compiled : 2017-12-17                                                        =<-
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

#define PADTYPE_3BUTTONS   3
#define PADTYPE_6BUTTONS   5
#define PADTYPE_UNDEFINED  0

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
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x04,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)

    // Buttons
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x00,                    //     REPORT_COUNT (0)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x03,                    //     INPUT (Constant,Var,Abs)

    // Axis
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)

    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

typedef struct
{
   uchar buttons;
   char  axis_x;
   char  axis_y;
}report_t;

#ifdef DUAL_JOYDIVISION
static report_t reportBuffer[2];
static PROGMEM const char DeviceStr[] = "Dual JoyDivision #x";
static PROGMEM const char VendorStr[] = "OnyxSoft";
static const char Desc0Str[] = { 4, 3, 0x09, 0x05 };

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
static uchar btnCountPort1 = PADTYPE_UNDEFINED;  // Keep track on the gamepads number of buttons...
static uchar btnCountPort2 = PADTYPE_UNDEFINED;  // Keep track on the gamepads number of buttons...


report_t buildReport(uchar joyport)
{
   char axis_x=0, axis_y=0;
   uchar buttons=0;
   report_t activeBuffer;
   
#ifdef DUAL_JOYDIVISION
   activeBuffer = reportBuffer[joyport];
#else
   activeBuffer = reportBuffer;
#endif
   
   if (joyport == JOYPORT1)
   {
      if (!TESTBIT(PINC, PC5)) axis_y = 0x81;   // Check if Up-axis is active
      if (!TESTBIT(PINC, PC4)) axis_y = 0x7f;   // Check if Down-axis is active
      if (!TESTBIT(PINC, PC3)) axis_x = 0x81;   // Check if Left-axis is active
      if (!TESTBIT(PINC, PC2)) axis_x = 0x7f;   // Check if Right-axis is active
      if (!TESTBIT(PINC, PC1)) buttons |= 0x02; // Check if Button B is active
      if (!TESTBIT(PINB, PB0)) buttons |= 0x04; // Check if Button C is active

      CLEARBIT(PORTB, PB1); // Set SELECT pin low...
      _delay_us(50);

      if (!TESTBIT(PINC, PC1)) buttons |= 0x01; // Check if Button A is active
      if (!TESTBIT(PINB, PB0)) buttons |= 0x40; // Check if Start button  is active

      SETBIT(PORTB, PB1); // Set SELECT pin high...

      // Make some spart check to determine the number of buttons that really are present here...
      CLEARBIT(PORTB, PB1); // Set SELECT pin low...
      _delay_us(15);
      SETBIT(PORTB, PB1); // Set SELECT pin high...
      _delay_us(15);
      CLEARBIT(PORTB, PB1); // Set SELECT pin low...
      _delay_us(15);

      // Check the bits on PC3, PC4 & PC5. On 6-button pads this shall all be high in this cycle...
      if (btnCountPort1 == PADTYPE_UNDEFINED)
      {
         if ((!TESTBIT(PINC, PC5)) && (!TESTBIT(PINC, PC4)) && (!TESTBIT(PINC, PC3)))
            btnCountPort1 = PADTYPE_6BUTTONS;
         else
            btnCountPort1 = PADTYPE_3BUTTONS;
      }

      SETBIT(PORTB, PB1); // Set SELECT pin high...
      _delay_us(15);

      if (btnCountPort1 != PADTYPE_3BUTTONS)
      {
         if (!TESTBIT(PINC, PC5)) buttons |= 0x20; // Check if Button Z is active
         if (!TESTBIT(PINC, PC4)) buttons |= 0x10; // Check if Button Y is active
         if (!TESTBIT(PINC, PC3)) buttons |= 0x08; // Check if Button X is active
         if (!TESTBIT(PINC, PC2)) buttons |= 0x80; // Check if Mode is active
      }

      CLEARBIT(PORTB, PB1); // Set SELECT pin low...
      _delay_us(15);
      SETBIT(PORTB, PB1); // Set SELECT pin high...
      _delay_us(15);
   }
   else // joyport == JOYPORT2
   {
      if (!TESTBIT(PIND, PD7)) axis_y = 0x81;   // Check if Up-axis is active
      if (!TESTBIT(PIND, PD6)) axis_y = 0x7f;   // Check if Down-axis is active
      if (!TESTBIT(PIND, PD5)) axis_x = 0x81;   // Check if Left-axis is active
      if (!TESTBIT(PIND, PD4)) axis_x = 0x7f;   // Check if Right-axis is active
      if (!TESTBIT(PINB, PB2)) buttons |= 0x02; // Check if Button B is active
      if (!TESTBIT(PINB, PB4)) buttons |= 0x04; // Check if Button C is active

      CLEARBIT(PORTB, PB3); // Set SELECT pin low...
      _delay_us(50);

      if (!TESTBIT(PINB, PB2)) buttons |= 0x01; // Check if Button A is active
      if (!TESTBIT(PINB, PB4)) buttons |= 0x40; // Check if Start button  is active

      SETBIT(PORTB, PB3); // Set SELECT pin high...

      // Make some spart check to determine the number of buttons that really are present here...
      CLEARBIT(PORTB, PB3); // Set SELECT pin low...
      _delay_us(15);
      SETBIT(PORTB, PB3); // Set SELECT pin high...
      _delay_us(15);
      CLEARBIT(PORTB, PB3); // Set SELECT pin low...
      _delay_us(15);

      // Check the bits on PD5, PD6 & PD7. On 6-button pads this shall all be high in this cycle...
      if (btnCountPort2 == PADTYPE_UNDEFINED)
      {
         if ((!TESTBIT(PIND, PD7)) && (!TESTBIT(PIND, PD6)) && (!TESTBIT(PIND, PD5)))
            btnCountPort2 = PADTYPE_6BUTTONS;
         else
            btnCountPort2 = PADTYPE_3BUTTONS;
      }

      SETBIT(PORTB, PB3); // Set SELECT pin high...
      _delay_us(15);

      if (btnCountPort2 != PADTYPE_3BUTTONS)
      {
         if (!TESTBIT(PIND, PD7)) buttons |= 0x20; // Check if Button Z is active
         if (!TESTBIT(PIND, PD6)) buttons |= 0x10; // Check if Button Y is active
         if (!TESTBIT(PIND, PD5)) buttons |= 0x08; // Check if Button X is active
         if (!TESTBIT(PIND, PD4)) buttons |= 0x80; // Check if Mode is active
      }

      CLEARBIT(PORTB, PB3); // Set SELECT pin low...
      _delay_us(15);
      SETBIT(PORTB, PB3); // Set SELECT pin high...
      _delay_us(15);
   }
   
   activeBuffer.axis_x  = axis_x;
   activeBuffer.axis_y  = axis_y;
   activeBuffer.buttons = buttons;

   return activeBuffer;
}

uchar usbFunctionSetup(uchar data[8])
{
   usbRequest_t *rq = (void *)data;

   if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS)
   {
      if(rq->bRequest == USBRQ_HID_GET_REPORT)
      {
         usbMsgPtr = (usbMsgPtr_t )&reportBuffer;
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


   Sega info borrowed from : http://segaretro.org/Six_Button_Control_Pad_(Mega_Drive)
   http://www.cs.cmu.edu/~chuck/infopg/segasix.txt

   Technical Information

   In order to maintain backwards-compatibility with existing games, the 6-button gamepad acts the same as the 3-button if the 
   controller inputs are read by the program a maximum of four times per frame. Reading the 3-button gamepad takes two cycles 
   (once with TH=1 to read Up, Down, Left, Right, B, and C; and once with TH=0 to read A and Start), and the 6-button gamepad 
   allows this to occur twice. It then reports the state of the extra buttons when the TH line is changed after the fourth cycle.

   The full controller state is read in 8 cycles:

   Cycle    TH out   TR in    TL in    D3 in    D2 in    D1 in    D0 in
   1        HI       C        B        Right    Left     Down     Up
   2        LO       Start    A        0        0        Down     Up
   3        HI       C        B        Right    Left     Down     Up
   4        LO       Start    A        0        0        Down     Up
   5        HI       C        B        Right    Left     Down     Up
   6        LO       Start    A        0        0        0        0
   7        HI       C        B        Mode     X        Y        Z
   8        LO       Start    A        ---      ---      ---      ---

   Note: For the 8th cycle, '---' indicates "don't care". These values are all 1 on the 6-button controller, but may be mapped to 
   other buttons on third-party controllers. Software expecting 6-button controllers should simply ignore these values. (6-button 
   detection should be done on the 6th cycle, which presents four 0's on the D lines.)

   The controller's internal counter automatically resets after the eighth cycle. It also resets if it doesn't detect a rising-
   edge (0 to 1) transition on the TH line within 1.5 ms, which ensures that most games that don't support the extra buttons won't 
   read them accidentally.

   Some games incorrectly read the controller more than twice per frame, which may cause them to read the wrong set of buttons. If 
   the Mode button is held in while starting the Mega Drive console, the 6-button gamepad will disable the extra buttons and act   
   identically to a 3-button gamepad. (That is, the controller's internal counter will reset after the first two cycles instead of 
   after eight cycles.) 

----

   This means that we need to "clock" the states for at least 8 "rounds" to be able to read the gamepad states.

   Pin   Sega MD-GamePad     AVR-Pin (Port 1)   AVR-Pin (Port 2)
   1     Up / Z              PC5                PD7
   2     Down / Y            PC4                PD6
   3     Left / X            PC3                PD5
   4     Right / Mode        PC2                PD4
   5     +5V                 PC0                PD3
   6     B / A               PC1                PB2
   7     Select              PB1                PB3
   8     GND                 GND                GND
   9     C / Start           PB0                PB4  */

   // Init PortB
   DDRB  = 0x00;
   PORTB = 0b00111101;

   // Init PortC
   DDRC  = 0x00;
   PORTC = 0b00111110;

   // Init PortD
   DDRD  = 0x00;
   PORTD = 0b11111001;

   SETBIT(DDRB, PB1);  // Enable PB1 (SELECT) as an output
   SETBIT(PORTB, PB1); // SELECT pin high from start...
   SETBIT(DDRC, PC0);  // Enable PC1 as an output
   SETBIT(PORTC, PC0); // Enable PB1 / VCC

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

#ifdef DUAL_JOYDIVISION
   SETBIT(DDRB, PB3);  // Enable PB3 (SELECT) as an output
   SETBIT(PORTB, PB3); // SELECT pin high from start...
   SETBIT(DDRD, PD3);  // Enable PC1 as an output
   SETBIT(PORTD, PD3); // Enable PB1 / VCC
#endif
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

