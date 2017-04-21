/*
->===================================================<-
->= bootloadHID.MorphOS - © Copyright 2016 OnyxSoft =<-
->===================================================<-
->= Version  : 0.1                                  =<-
->= File     : usb-morphos.c                        =<-
->= Author   : Stefan Blixth                        =<-
->= Compiled : 2016-08-02                           =<-
->===================================================<-

This is an adaptation of Christian Starkjohanns bootloadHID commandline tool for MorphOS (using Posiedon USB-Stack).
Thanks to Filip "widelec" Maryjañski for helping me out with the poseidon calls =)

*/

#include <stdio.h>
#include <string.h>

#include <proto/exec.h>
#include <proto/poseidon.h>
#include <devices/usb_hid.h>

#include "usbcalls.h"
#include "debug.h"
   
struct NepClassBHT
{
    struct PsdDevice     *nch_Device;       /* Up linkage */
    struct PsdPipe       *nch_Pipe;         /* Endpoint 0 pipe */
    struct PsdAppBinding *nch_PAB;          /* Application Binding */
    struct Task          *nch_Task;         /* this task */
    struct MsgPort       *nch_TaskMsgPort;  /* io message port */
};

struct Library *PsdBase = NULL;
struct NepClassBHT *nch = NULL;

static int  usesReportIDs;

void ReleaseFunc(void)
{
   struct Hook *hook = (struct Hook *)REG_A0;
   APTR object = (APTR)REG_A2;
   APTR message = (APTR)REG_A1;

   debug_print("%s : %s (%d)\n", __FILE__ , __func__, __LINE__);
   return;
}

static struct EmulLibEntry ReleaseGate = {TRAP_LIB, 0, (void(*)(void*)) ReleaseFunc};
static struct Hook ReleaseHook = {{NULL, NULL}, (HOOKFUNC)&ReleaseGate, {NULL, NULL}};

int usbOpenDevice(usbDevice_t **device, int vendor, char *vendorName, int product, char *productName, int _usesReportIDs)
{
   int errorCode = USB_ERROR_NOTFOUND;
   struct PsdDevice *pd = NULL;
   ULONG unit = 0;
   usbDevice_t *okdevice = (usbDevice_t *)1;

   debug_print("%s : %s (%d)\n", __FILE__ , __func__, __LINE__);

   if((PsdBase = OpenLibrary("poseidon.library", 4)))
   {
      *device = okdevice;

      do
      {
         pd = psdFindDevice(pd,
                            DA_VendorID, vendor,
                            DA_ProductID, product,
                            DA_Manufacturer, vendorName,
                            DA_ProductName, productName,
                            TAG_END);
      }
      while(pd && (unit--));

      if(!pd)
      {
         return USB_ERROR_NOTFOUND;
      }

      if ((nch = psdAllocVec(sizeof(struct NepClassBHT))))
      {
         nch->nch_Device = pd;
         nch->nch_PAB = psdClaimAppBinding(ABA_Device, pd,
                                           ABA_ReleaseHook, &ReleaseHook,
                                           ABA_UserData, nch,
                                           ABA_ForceRelease, TRUE,
                                           TAG_END);
         if(nch->nch_PAB)
         {
            nch->nch_Task = FindTask(NULL);

            if ((nch->nch_TaskMsgPort = CreateMsgPort()))
            {
               if((nch->nch_Pipe = psdAllocPipe(nch->nch_Device, nch->nch_TaskMsgPort, NULL)))
               {
                  usesReportIDs = _usesReportIDs;
                  return USB_ERROR_NONE;
               }
               else
                  fprintf(stderr, "Error: usbOpenDevice: cannot create Poseidon pipe!\n");

               DeleteMsgPort(nch->nch_TaskMsgPort);
            }
            else
               fprintf(stderr, "Error: usbOpenDevice: cannot create MsgPort\n");

            psdReleaseAppBinding(nch->nch_PAB);
         }
         else
         {
            fprintf(stderr, "Error: usbOpenDevice: cannot bind app\n");
            errorCode = USB_ERROR_ACCESS;
         }
      }
      CloseLibrary(PsdBase);
   }
   return errorCode;   
}


void usbCloseDevice(usbDevice_t *device)
{
   debug_print("%s : %s (%d)\n", __FILE__ , __func__, __LINE__);

   if (nch)
   {
      if (nch->nch_Pipe) psdFreePipe(nch->nch_Pipe);
      if (nch->nch_TaskMsgPort) DeleteMsgPort(nch->nch_TaskMsgPort);
      if (nch->nch_PAB) psdReleaseAppBinding(nch->nch_PAB);
      psdFreeVec(nch);
   }

   if (PsdBase) CloseLibrary(PsdBase);
}

int usbSetReport(usbDevice_t *device, int reportType, char *buffer, int len)
{
   LONG ioerr;
   int bytesSent;
   debug_print("%s : %s (%d)\n", __FILE__ , __func__, __LINE__);

   psdPipeSetup(nch->nch_Pipe, URTF_OUT|URTF_CLASS|URTF_INTERFACE, UHR_SET_REPORT, (ULONG) ((reportType << 8) | buffer[0]), 0);
   ioerr = psdDoPipe(nch->nch_Pipe, buffer, len);
   bytesSent = psdGetPipeActual(nch->nch_Pipe);

   if(bytesSent != len)
   {
      if(bytesSent < 0)
         fprintf(stderr, "Error sending message: %s\n", psdNumToStr(NTS_IOERR, ioerr, "unknown"));
      return USB_ERROR_IO;
   }
   return 0;
}

/* ------------------------------------------------------------------------- */

int usbGetReport(usbDevice_t *device, int reportType, int reportNumber, char *buffer, int *len)
{
   LONG ioerr;
   int bytesReceived, maxLen = *len;
   debug_print("%s : %s (%d)\n", __FILE__ , __func__, __LINE__);

   psdPipeSetup(nch->nch_Pipe, URTF_IN|URTF_CLASS|URTF_INTERFACE, UHR_GET_REPORT, reportType << 8 | reportNumber, 0);
   ioerr = psdDoPipe(nch->nch_Pipe, buffer, maxLen);
   bytesReceived = psdGetPipeActual(nch->nch_Pipe);

   if(bytesReceived < 0)
   {
      fprintf(stderr, "Error sending message: %s\n", psdNumToStr(NTS_IOERR, ioerr, "unknown"));
      return USB_ERROR_IO;
   }
   
   *len = bytesReceived;
   
   if(!usesReportIDs)
   {
      buffer[-1] = reportNumber;  /* add dummy report ID */
     *len++;
   }
   return 0;
}
