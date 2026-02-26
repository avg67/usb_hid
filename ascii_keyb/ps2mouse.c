/*************************************************************************
**************************************************************************
**                                                                      **
**  Project:   PS/2 Mouse => NKC                                        **
**  Title:     PS/2 Mouse => NKC Implementation                         **
**  File:      ps2mouse.c                                               **
**  Date:      10.01.2018                                               **
**  Version:   0.96                                                     **
**  Plattform: WinAVR-20060421 & AVR Studio 4.12.498 & STK500           **
**                                                                      **
**  Created by Gerald Ebert                                             **
**                                                                      **
**************************************************************************
 Version   Description/Changes

   0.80    Project start. PS/2 Interface and management taken from keyboard project
   0.90    Scancode -> Mouse position decoder with error management added
   0.91    Interface to NKC-HCOPY added, compatible to future NKC-KEY4
   0.94    Made some bugfixes
   0.95    Activity LED controller added, compatible to future NKC-KEY4	
   0.96	   Fix mousemovement / Jens Mewes

**************************************************************************
*************************************************************************/

#include "avr_defs.h"
#include "ps2interface.h"

//========================================================================
// Hardwiring the ATtiny2313
//========================================================================
/*
                             ____  ____
                   (RESET\) -|   \/   |- (Vcc)
              nc  (RxD/PD0) -|        |- (PB7) nc
              nc  (TxD/PD1) -|        |- (PB6) KEY4-mouseButton3 (middle)
              nc      (PA1) -|        |- (PB5) KEY4-mouseButton2 (right)
        NKC-INT\      (PA0) -|        |- (PB4) KEY4-mouseButton1 (left)
        PS/2-Clk (INT0/PD2) -|        |- (PB3) KEY4-mouseDown
   Ext. Activity (INT1/PD3) -|        |- (PB2) KEY4-mouseUp
       PS/2-Data      (PD4) -|        |- (PB1) KEY4-mouseRight
              nc      (PD5) -|        |- (PB0) KEY4-mouseLeft
                      (GND) -|________|- (PD6) Activity LED

*/

//========================================================================
// Defines
//========================================================================

//------------------------------
// NKC-KEY interface

#define NKC_MOUSELEFT        BV(PB0)
#define NKC_MOUSERIGHT       BV(PB1)
#define NKC_MOUSEUP          BV(PB2)
#define NKC_MOUSEDOWN        BV(PB3)
#define NKC_MOUSEBUTTON1     BV(PB4)
#define NKC_MOUSEBUTTON2     BV(PB5)
#define NKC_MOUSEBUTTON3     BV(PB6)
#define NKC_MOUSEBUTTONMASK  (NKC_MOUSEBUTTON1 | NKC_MOUSEBUTTON2 | NKC_MOUSEBUTTON3)
#define NKCDATA_PORT         PORTB
// --- Make a short pulse to increment the external counters
#define NKC_MOUSELEFT_COUNT  { NKCDATA_PORT |= NKC_MOUSELEFT;   NOP;   NKCDATA_PORT &= ~NKC_MOUSELEFT;  }
#define NKC_MOUSERIGHT_COUNT { NKCDATA_PORT |= NKC_MOUSERIGHT;  NOP;   NKCDATA_PORT &= ~NKC_MOUSERIGHT; }
#define NKC_MOUSEUP_COUNT    { NKCDATA_PORT |= NKC_MOUSEUP;     NOP;   NKCDATA_PORT &= ~NKC_MOUSEUP;    }
#define NKC_MOUSEDOWN_COUNT  { NKCDATA_PORT |= NKC_MOUSEDOWN;   NOP;   NKCDATA_PORT &= ~NKC_MOUSEDOWN;  }

//------------------------------
// External signals

#define ISC1MASK            (BV(ISC11)|BV(ISC10))
#define ISC1RISE            (BV(ISC11)|BV(ISC10))
#define ISC1FALL            (BV(ISC11))

#define NKCINT_OUT          BV(PA0)
#define NKCSIG_PORT         PORTA
// --- Signal NKC new mouse data received (-> NKC-INT\)
#define NKC_INT_REQ         { NKCSIG_PORT &= ~NKCINT_OUT;    Wait_us(32);   NKCSIG_PORT |= NKCINT_OUT; }

#define LEDctrl_EXT         BV(PD3)
#define LEDctrl_OUT         BV(PD6)
#define LEDctrl_PORT        PORTD
// --- Set timer clock prescaler to 1/64. Set timer register to generate a 20 Hz interrupt
#define LEDctrl_INIT        { TCCR1B = 0x03; }
#define LEDctrl_ENABLE      { CLI;   LEDctrl_PORT &= ~LEDctrl_OUT;   TCNT1 = 0xE796;   TIFR |= BV(TOV1); \
                                     TIMSK |= BV(TOIE1);   SEI; }
#define LEDctrl_DISABLE     { CLI;   LEDctrl_PORT |=  LEDctrl_OUT;   TIMSK &= ~BV(TOIE1);   SEI;}

//------------------------------
// Host to mouse commands

#define SET_RESOLUTION       0xE8
#define ECHO_CODE            0xEE
#define GET_DEVICE_ID        0xF2
#define SET_SAMPLE_RATE      0xF3
#define DATA_REPORT_ENABLE   0xF4
#define DATA_REPORT_DISABLE  0xF5
#define SET_DEFAULT_CODE     0xF6

#define RESOLUTION1cmm       0x00
#define RESOLUTION2cmm       0x01
#define RESOLUTION4cmm       0x02
#define RESOLUTION8cmm       0x03

#define SAMPLERATE_10sps       10
#define SAMPLERATE_20sps       20
#define SAMPLERATE_40sps       40
#define SAMPLERATE_60sps       60
#define SAMPLERATE_80sps       80
#define SAMPLERATE_100sps     100
#define SAMPLERATE_200sps     200

//========================================================================
// Local Variables
// (use volatile when variable is accessed from interrupts)
//========================================================================

static BYTE mousePacket [3];                     // Buffer for mouse packet data
static BYTE mpCounter;                           // Counter for incoming packet data

static char XLpos, XRpos, YUpos, YDpos;          // The current relative mouse position
static BYTE MouseButtons;                        // Current mouse button states

//========================================================================
//========================================================================
// Interrupt Sevice Routines
//========================================================================
//========================================================================

//------------------------------------------------------------------------
// Default Interrupt Sevice Routine
//------------------------------------------------------------------------

EMPTY_INTERRUPT(__vector_default);

//------------------------------------------------------------------------
// External Interrupt 1 Interrupt Sevice Routine
//------------------------------------------------------------------------

// Any other controller can indicate an activity over the INT1 pin.
// In case of future NKC-KEY4 the keyboard controller indicates its
// activity here because it has not enough flash memory left to handle this. 

ISR(INT1_vect)
{
   LEDctrl_ENABLE;                               // Switch activity led on
}


//------------------------------------------------------------------------
// TIMER1 Output Compare A Match Interrupt Sevice Routine
//------------------------------------------------------------------------

ISR(TIMER1_OVF_vect)
{
   LEDctrl_DISABLE;                              // Switch activity led off
}

//========================================================================
//========================================================================
// Handle Mouse Power On Sequence
//========================================================================

void HandlePowerOn(void)
{
   WaitForACKorData();                           // Wait until mouse data byte received
   if ( rData == 0x00 )                          // If mouse sends it's ID byte then ...
   {
      Send2PS2Device(SET_SAMPLE_RATE);           // Send Set Sample Rate command
      WaitForACKorData();                        // Wait until mouse acknoledge received
      Send2PS2Device(SAMPLERATE_40sps);          // Send Sample to 40 samples/sec (like Windows98)
      WaitForACKorData();                        // Wait until mouse acknoledge received

      Send2PS2Device(SET_RESOLUTION);            // Send Set Resolution command
      WaitForACKorData();                        // Wait until mouse acknoledge received
      Send2PS2Device(RESOLUTION1cmm);            // Send Resolution to 1 counts/mm, best for NKC
      WaitForACKorData();                        // Wait until mouse acknoledge received

      Send2PS2Device(DATA_REPORT_ENABLE);        // Send Enable Data Reporting command
      WaitForACKorData();                        // Wait until mouse acknoledge received
   }
}

//========================================================================
//========================================================================
// Decode mouse packet (3 Byte) to mouse postion
//========================================================================

void DecodeMouse(BYTE data)
{
   BYTE mpc = mpCounter;                         // Load mouse packet counter
   BYTE mpi;

   mousePacket[mpc++] = data;                    // Store byte to mouse packet buffer
   if ( mpc > 2 )                                // Mouse packet complete ?
   {
      mpc = 0;                                   // Reset packet counter and decode packet
      mpi = mousePacket[0];
      if ( (mpi & 0x08) )                        // Must be TRUE
      {
         MouseButtons = ((mpi & 0x07) << 4);     // Mouse button states

         if ( (mpi & 0x20) )                     // Left movement
            XLpos -= mousePacket[1];
         else                                    // Right movement
            XRpos += mousePacket[1];
         if ( (mpi & 0x10) )                     // Up movement
            YUpos -= mousePacket[2];
         else                                    // Down movement
            YDpos += mousePacket[2];
      }
      else                                       // ERROR, try reset mouse
      {
         Send2PS2Device(RESET_CODE);             // Send repeat last byte command on fail
         WaitForACKorData();                     // Wait until mouse acknoledge received
      }
   }
   mpCounter = mpc;                              // Restore mouse packet counter
}

//========================================================================
//========================================================================
// Output mouse position to counters
//========================================================================

void TransferData2NKC(void)
{
   BYTE temp;
   BOOL bPosDataOut = TRUE;

    // Output mouse buttons
    temp = NKCDATA_PORT;
    temp |= NKC_MOUSEBUTTONMASK;
    temp &= ~MouseButtons;
    NKCDATA_PORT = temp;
    // Output relative mouse position
    while ( bPosDataOut )
    {
       bPosDataOut = FALSE;
       // Output left position
       if ( XLpos > 0 )
       {
          XLpos--;
          NKC_MOUSELEFT_COUNT;
          bPosDataOut = TRUE;
       }
       // Output right position
       if ( XRpos > 0 )
       {
          XRpos--;
          NKC_MOUSERIGHT_COUNT;
          bPosDataOut = TRUE;
       }
       // Output up position
       if ( YUpos > 0 )
       {
          YUpos--;
          NKC_MOUSEUP_COUNT;
          bPosDataOut = TRUE;
       }
       // Output down position
       if ( YDpos > 0 )
       {
          YDpos--;
          NKC_MOUSEDOWN_COUNT;
          bPosDataOut = TRUE;
       }
       if ( bPosDataOut )
          Wait_us(12);                      // Wait 12 µs, must be more than double time for latch and clear
    }
    // Signal NKC new mouse data received (-> NKC-INT\)
    NKC_INT_REQ;
}

//========================================================================
//========================================================================
// MAIN
//========================================================================

int main (void)
{
   BYTE temp;
   BOOL bRepeatSend = FALSE;                  

   DDRA  = 0xFF;    PORTA = 0xFF;                // Set all ports to output first
   DDRB  = 0xFF;    PORTB = 0xFF;
   DDRD  = 0xFF;    PORTD = 0xFF;

   Set_Sleep_Mode(SLEEP_MODE_IDLE);              // Initialize Sleep Mode for AVR
   WDTI_Enable(WDT_65MS);                        // Enable Watchdog Timer Interrupt
   LEDctrl_INIT;                                 // Initialze TIMER1 for activity led control

   PS2DATA_DDR  &= ~(PS2DATA_CLK | PS2DATA_OUT | LEDctrl_EXT);     // Clock, data and external activity pin as input
   MCUCR = (MCUCR & (~(ISC0MASK | ISC1MASK))) | ISC0FALL | ISC1FALL; // Set INT0 on falling edge and INT1 on falling edge
   GIMSK |= BV(INT0) | BV(INT1);                 // Enable external INT0 and INT1
   NKCDATA_PORT = NKC_MOUSEBUTTONMASK;           // No button pressed and counters to low level

   SEI;                                          // Enable all used ISR

   while ( TRUE )
   {
      WaitForACKorData();                        // Wait until keyboard data byte received

      // Check parity bit
      temp = rParityBits & 0x01;
      if ( temp != rParity && (! bRepeatSend) )
      {
         Send2PS2Device(RESEND_CODE);            // Send repeat last packet command on fail
         bRepeatSend = TRUE;                     // Only one repeat command per packet
         mpCounter = 0;                          // Reset mouse packet counter
      }
      else
      {
         bRepeatSend = FALSE;                    // Reset repeat indicator

         if ( rData == PWRON_CODE )              // Keyboard Power On received
            HandlePowerOn();
         else
         {
            LEDctrl_ENABLE;                      // Switch activity led on
            DecodeMouse(rData);                  // Decode mouse packet code to mouse position
            TransferData2NKC();                  // Transfer mouse position and button data to NKC
         }
      }
   }
}

//========================================================================
