/*************************************************************************
**************************************************************************
**                                                                      **
**  Project:   common                                                   **
**  Title:     FIFO implementation                                      **
**  File:      fifo.c                                                   **
**  Date:      12.02.2006                                               **
**  Version:   1.00                                                     **
**  Plattform: WinAVR-20060421 & AVR Studio 4.12.498 & STK500           **
**                                                                      **
**  Created by Gerald Ebert                                             **
**                                                                      **
**************************************************************************
*************************************************************************/

//#include "avr_defs.h"
#include "fifo.h"

//========================================================================
//========================================================================
// FIFO management for keyboard buffer
//========================================================================

//------------------------------------------------------------------------
// Initialize FIFO struct
//------------------------------------------------------------------------

void FIFO_init(LPtFIFO FIFO, LPBYTE Buffer, BYTE size)
{
   FIFO->Buffer = Buffer;
   FIFO->sizeCur = FIFO->readPos = FIFO->writePos = 0;
   FIFO->sizeMax = 0;
   if ( Buffer != NULL )
      FIFO->sizeMax = size;
}

//------------------------------------------------------------------------
// Add a BYTE to FIFO
//------------------------------------------------------------------------

BOOL FIFO_in(LPtFIFO FIFO, BYTE data)
{
   BYTE w = FIFO->writePos;
   BYTE m = FIFO->sizeMax;
   BYTE s = FIFO->sizeCur;
	
   if ( s < m )
   {
      FIFO->Buffer[w++] = data;
      if ( w >= m )	   w = 0;
      s++;
      FIFO->writePos = w;
      FIFO->sizeCur = s;
      return TRUE;
   }
   return FALSE;
}

//------------------------------------------------------------------------
// Get a BYTE from FIFO
//------------------------------------------------------------------------

BYTE FIFO_out(LPtFIFO FIFO)
{
   BYTE r = FIFO->readPos;
   BYTE m = FIFO->sizeMax;
   BYTE s = FIFO->sizeCur;
   BYTE data = 0;
	
   if ( s > 0 )
   {
      data = FIFO->Buffer[r++];
      if ( r >= m )    r = 0;
      s--;
      FIFO->readPos = r;
      FIFO->sizeCur = s;
   }
   return data;
}

//========================================================================
