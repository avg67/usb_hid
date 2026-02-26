/*************************************************************************
**************************************************************************
**                                                                      **
**  Project:   PS/2 Keyboard => NKC                                     **
**  Title:     PS/2 Interface => NKC Implementation                     **
**  File:      ps2interface.c                                           **
**  Date:      12.02.2007                                               **
**  Version:   0.95                                                     **
**  Plattform: WinAVR-20060421 & AVR Studio 4.12.498 & STK500           **
**                                                                      **
**  Created by Gerald Ebert                                             **
**                                                                      **
**************************************************************************
 Version   Description/Changes

 Version   Description/Changes

   0.80    Project start
   0.85    At last the PS/2 Interface is functioning in both direction
   0.86    Watchdog timer implemented to avoid hangup on protocol errors
   0.90    Opimizing code, flash memory is running out (99,8 %)
   0.95    Made some bugfixes

**************************************************************************
*************************************************************************/


//#include "avr_defs.h"
#include "types.h"
#include "ps2interface.h"


//========================================================================
// Global Variables
// (use volatile when variable is accessed from interrupts)
//========================================================================

BYTE PS2State;                          // The current working state

BYTE  rData;                             // Received scan code
BYTE  rParityBits;                       // Received parity count
BYTE  rParity;                           // Received parity bit
BYTE  sData;                             // The scan code to send

//========================================================================
// Local Variables
// (use volatile when variable is accessed from interrupts)
//========================================================================

/*static BYTE volatile IntMode;                   // Mode of INT0-ISR
static BOOL volatile IntEdge;                    // FALSE = falling  TRUE = raising
static BYTE volatile BitCount;                   // Counts received bits
static BYTE volatile sParityBits;                // The parity count for send data
*/
//========================================================================
//========================================================================
// Delay Function
//========================================================================
/**/
void  Wait_us(WORD us)
{
   WORD count;

#if F_CPU <= 4000000
   BYTE i = 1;
#elif F_CPU <= 8000000
   BYTE i = 2;
#elif F_CPU <= 12000000
   BYTE i = 3;
#else 
   BYTE i = 4;
#endif

   while ( i-- )
		// 16-bit count, 4 cycles/loop
		__asm__ __volatile__ ( "movw %0, %1"  "\n\t" "1: sbiw %0,1" "\n\t" "brne 1b" "\n\t" : "=&w"(count) : "w"(us) );
}*/

//========================================================================
//========================================================================
// Interrupt Sevice Routines
//========================================================================
//========================================================================

//------------------------------------------------------------------------
// INT0 Interrupt Routine 
//------------------------------------------------------------------------

ISR(INT0_vect)
{
   BOOL sAcknoledge;
   BYTE bc, edge;

   WDT;                                          // Reset watchdog timer
   bc = BitCount;                                // Load bit counter
   edge = IntEdge;                               // Load edge detection
   // Init registers for receive mode
   if ( IntMode == INTMODE_LISTEN )
   {
      rParityBits = bc = 0;                      // Reset bit counter and clear parity counter
      MCUCR = (MCUCR & (~ISC0MASK))|ISC0RISE;    // Set INT0 on rising edge
      edge = TRUE;
	   IntMode = INTMODE_RECEIVE;                 // Activate receive mode
	}
	// Receive data from keyboard
   else if ( IntMode == INTMODE_RECEIVE )
	{
      if ( edge )                                // Routine entered at rising edge
      {
         if ( ++bc >= 11 )                       // All bits received?
		   {
            IntMode = INTMODE_LISTEN;            // Return to listen mode
		      PS2State = PS2STATE_DATA_RECEIVED;   // Signal -> byte from PS/2 interface received
         }
         MCUCR = (MCUCR & (~ISC0MASK))|ISC0FALL; // Set interrupt on falling edge
		   edge = FALSE;
      }
      else                                       // Routine entered at falling edge
      {   
         if ( bc > 0 && bc < 9 )                 // Bit 1 to 8 is data
	      { 
            rData >>= 1;                         // Shift data for new bit
	         if ( PS2DATA_PIN )
	         {
               rData |=  0x80;                   // Store a received 1 bit; 0 bit is default
               rParityBits++;                    // Update parity count
            }
         }
	      else if ( bc == 9 )                     // Bit 9 is parity bit
            rParity =  (PS2DATA_PIN ?0 :1);
	      MCUCR = (MCUCR & (~ISC0MASK))|ISC0RISE; // Set INT0 on rising edge
	      edge = TRUE;
      }
   }

	// Prepare to send data to keyboard
   else if ( IntMode == INTMODE_SENDSTART )
	{
      sParityBits = bc = 0;                      // Reset bit counter and clear parity counter
      IntMode = INTMODE_SEND;                    // Activate send mode
      // We are still on falling edge
   }
   // Send data to keyboard
   else if ( IntMode == INTMODE_SEND )
	{
	   if ( ++bc < 9 )                            // Bit 1 to 8 are data bits
	   {
	      if ( (sData & 0x01) )          
	      {
 	         PS2DATA_PORT |= PS2DATA_OUT;         // Send a 1
		      sParityBits++;                       // Update parity count
		   }
		   else
		      PS2DATA_PORT &= ~PS2DATA_OUT;        // Send a 0
 		   sData >>= 1;                            // Next data bit
	   }
	   else if ( bc == 9 )                        // Bit 9 is parity bit
	   {
	      if ( (sParityBits & 0x01) )             // Use odd parity
	         PS2DATA_PORT &= ~PS2DATA_OUT;        // Send a 0
         else
	         PS2DATA_PORT |= PS2DATA_OUT;         // Send a 1
         bc++;
	   }
	   else                                       // All bits send!
	   {                                       
	      PS2DATA_PORT |= PS2DATA_OUT;            // Release data line; set to high
         PS2DATA_DDR  &= ~PS2DATA_OUT;           // Set Data pin as input
         PS2DATA_PORT |= PS2DATA_OUT;	          // Set pullups
 	      IntMode = INTMODE_ACK;                  // Activate acknoledge mode
         // We are still on falling edge
	   }
   }
   // Detects the acknolegde bit from keyboard after data send
   else if ( IntMode == INTMODE_ACK )
	{
	   if ( edge )                                // Routine entered at rising edge
	   {
         MCUCR = (MCUCR & (~ISC0MASK))|ISC0FALL; // Set interrupt on falling edge
		   edge = FALSE;
	      if ( bc > 11 )                          // Acknoledge bit received?
	      {
            IntMode = INTMODE_LISTEN;            // Return to listen mode
			   PS2State = PS2STATE_DATA_SEND;       // Signal -> byte send to PS/2 interface
         }
	   }
	   else
	   {                                          // Read acknoledge from keyboard
         sAcknoledge = (PS2DATA_PIN ?FALSE :TRUE);
	      if ( sAcknoledge )   bc = 12;           // Acknoledge from keyboard detected
         MCUCR = (MCUCR & (~ISC0MASK))|ISC0RISE; // Set INT0 on rising edge
		   edge = TRUE;
	   }
   }
	BitCount = bc;                                // Restore bit counter
	IntEdge = edge;                               // Restore edge detection
}

//------------------------------------------------------------------------
// Watchdog Timer Interrupt Routine 
//------------------------------------------------------------------------

ISR(WDT_OVERFLOW_vect)
{
   // INT0-ISR must be in listen mode at this time.
   // We set this state if it is not.

   MCUCR = (MCUCR & (~ISC0MASK))|ISC0FALL;       // Set interrupt on falling edge
   IntEdge = FALSE;
   IntMode = INTMODE_LISTEN;                     // Force listen mode
   PS2State = PS2STATE_IDLE;                     // Reset Maschine to idle mode
}

//========================================================================
//========================================================================
// Send a Byte to PS/2 device
//========================================================================

void Send2PS2Device(BYTE data)
{
    WDT;    
    Wait_us(100);                                // Wait 100�s to be sure interface is idle
    GIMSK &= ~(BV(INT0));                        // Disable external INT0 first
    PS2DATA_DDR  |= PS2DATA_CLK;	                // Set clock pin to output
    PS2DATA_PORT &= ~PS2DATA_CLK;                // 1. Set clock line to low level
    Wait_us(100);                                // Wait 100�s to block interface sending data
    PS2DATA_PORT &= ~PS2DATA_OUT; 	             // 2. Set data line to low level
    PS2DATA_DDR  |= PS2DATA_OUT;	                // Set data pin to output
    PS2DATA_PORT &= ~PS2DATA_OUT; 	             // Hold data line at low level
	 NOP;
    PS2DATA_PORT |= PS2DATA_CLK;                 // 3. Set clock line to high level
	 NOP;
    PS2DATA_DDR  &= ~PS2DATA_CLK;                // Set clock pin to input
    PS2DATA_PORT |= PS2DATA_CLK;                 // Set pullup
	 NOP;
	 sData = data;                                // Copy data to output buffer
    IntMode = INTMODE_SENDSTART;                 // Activate send mode
    GIMSK |= BV(INT0);                           // Enable external INT0
    while ( PS2State != PS2STATE_DATA_SEND )     // Wait here until all bit are send
       SLEEP;
	 PS2State = PS2STATE_IDLE;                    // Reset State to idle.
}

//========================================================================
//========================================================================
// Wait for a Byte received from PS/2 Device
//========================================================================

BOOL WaitForACKorData(void)
{
   while ( PS2State != PS2STATE_DATA_RECEIVED )  // Wait until data byte received
      SLEEP;

   PS2State = PS2STATE_IDLE;                     // Reset State to idle.
   return (rData == ACK_CODE);                   // Return interface acknoledge received
}

//========================================================================
