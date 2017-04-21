/*!
   Copyright © OnyxSoft 2010-2013

   \file    debug.h
   \brief   Take care and prints out our debug data for us

   \author  Stefan Blixth
   \date    2013-10-29
   \version 0.3

   \todo    -
*/


#ifndef DEBUG_H_
#define DEBUG_H_

#if !defined(__MORPHOS__) || !defined (__amigaos4__) || !defined(__AROS__) || !defined(__AMIGA__)
   #include <stdio.h>
#else
   #ifdef __AROS__
      #include <aros/debug.h>
   #else
      #if defined(__amigaos3__) || defined(__MORPHOS__)
         #include <clib/debug_protos.h>
      #endif
   #endif
#endif

#ifdef USEDEBUG
   #define D(x) x

   #if defined(__amigaos4__)
      #define debug_print(args...)  { DebugPrintF(args); }
   #elif defined(__amigaos3__)
      #define debug_print(args...)  { kprintf(args); }
   #elif defined(__MORPHOS__)
     #define debug_print(args...)   { KPrintF((CONST_STRPTR)args); }
   #else
      #define debug_print(args...)  { fprintf(stderr, args); }
   #endif
#else
   #define D(x)
   #define debug_print(...)
#endif

#endif /* DEBUG_H_ */

