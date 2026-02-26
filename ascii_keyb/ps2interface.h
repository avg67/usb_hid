/*************************************************************************
**************************************************************************
**                                                                      **
**  Project:   PS/2 Keyboard => NKC                                     **
**  Title:     PS/2 Interface => NKC Declaration                        **
**  File:      ps2interface.h                                           **
**  Date:      12.02.2007                                               **
**  Version:   0.95                                                     **
**  Plattform: WinAVR-20060421 & AVR Studio 4.12.498 & STK500           **
**                                                                      **
**  Created by Gerald Ebert                                             **
**                                                                      **
**************************************************************************
*************************************************************************/

#ifndef _PS2INTERFACE_H_
  #define _PS2INTERFACE_H_

//========================================================================
// Defines
//========================================================================

//------------------------------
// INT0-ISR for PS/2 communication

/*#define ISC0MASK               (BV(ISC01)|BV(ISC00))
#define ISC0RISE               (BV(ISC01)|BV(ISC00))
#define ISC0FALL               (BV(ISC01))

#define PS2DATA_PIN             (PIND & BV(PD1))
#define PS2DATA_CLK             BV(PD2)
#define PS2DATA_OUT             BV(PD1)
#define PS2DATA_PORT            PORTD
#define PS2DATA_DDR             DDRD

#define INTMODE_LISTEN          0
#define INTMODE_RECEIVE         1
#define INTMODE_SENDSTART       2 
#define INTMODE_SEND            3
#define INTMODE_ACK             4*/

//------------------------------
// Machine states

#define PS2STATE_IDLE           0
#define PS2STATE_DATA_RECEIVED  1
#define PS2STATE_DATA_SEND      2

//------------------------------
// Host to keyboard PS/2 device

#define ERROR_CODE          0xFC
#define RESEND_CODE         0xFE
#define RESET_CODE          0xFF

#define PWRON_CODE          0xAA
#define ACK_CODE            0xFA

//========================================================================
// Exported Variables
//========================================================================

extern BYTE volatile PS2State;                   // The current working state

extern BYTE volatile rData;                      // Received scan code
extern BYTE volatile rParityBits;                // Received parity count
extern BYTE volatile rParity;                    // Received parity bit
extern BYTE volatile sData;                      // The scan code to send

//========================================================================
// Exported Functions
//========================================================================

void  Wait_us(WORD us);                          // Delay function

//void Send2PS2Device(BYTE data);                  // Send a Byte to PS/2 device
//BOOL WaitForACKorData(void);                     // Wait for a Byte received from Keyboard

//========================================================================

#endif // _PS2INTERFACE_H_
