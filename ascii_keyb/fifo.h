/*************************************************************************
**************************************************************************
**                                                                      **
**  Project:   common                                                   **
**  Title:     FIFO declaration                                         **
**  File:      fifo.h                                                   **
**  Date:      12.02.2006                                               **
**  Version:   1.00                                                     **
**  Plattform: WinAVR-20060421 & AVR Studio 4.12.498 & STK500           **
**                                                                      **
**  Created by Gerald Ebert                                             **
**                                                                      **
**************************************************************************
*************************************************************************/

#ifndef  _FIFO_H_
  #define _FIFO_H_

  #include "types.h"
//========================================================================
// Defines
//========================================================================

typedef struct
{
	LPBYTE  Buffer;
	BYTE    sizeMax;
	BYTE    sizeCur;
	BYTE    readPos;
	BYTE    writePos;
}
	tFIFO, * LPtFIFO;


#define FIFO_isempty(FIFO)   (FIFO.sizeCur == 0)
#define FIFO_isfilled(FIFO)  (FIFO.sizeCur != 0)


//========================================================================
// Exported Functions
//========================================================================

void FIFO_init(LPtFIFO FIFO, LPBYTE Buffer, BYTE size); // Initialize FIFO struct
BOOL FIFO_in(LPtFIFO FIFO, BYTE data);                  // Add a BYTE to FIFO
BYTE FIFO_out(LPtFIFO FIFO);                            // Get a BYTE from FIFO

//========================================================================

#endif  // _FIFO_H_
