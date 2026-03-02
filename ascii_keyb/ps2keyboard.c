/*************************************************************************
**************************************************************************
**                                                                      **
**  Project:   PS/2 Keyboard => NKC                                     **
**  Title:     PS/2 Keyboard => NKC Implementation                      **
**  File:      ps2keyboard.c                                            **
**  Date:      23.02.2007                                               **
**  Version:   0.96                                                     **
**  Plattform: WinAVR-20060421 & AVR Studio 4.12.498 & STK500           **
**                                                                      **
**  Created by Gerald Ebert                                             **
**                                                                      **
**	Anpassung f�r die KEYr4 von Jens Mewes                              **
**                                                                      **
**************************************************************************
 Version   Description/Changes

   0.80    Project start
   0.85    At last the PS/2 Interface is functioning in both direction
   0.86    Watchdog timer implemented to avoid hangup on protocol errors
   0.90    Scancode -> ASCII/NKC translater added
   0.91    Opimizing code, flash memory is running out (99,8 %)
   0.92    Interface to NKC-KEY added, compatible to future NKC-KEY4
   0.93    Opimizing code again, flash memory is running out (100,4 % !)
   0.94    Made some bugfixes
   0.95    Activity indictor added, compatible to future NKC-KEY4	
   0.96    Bugfix for Key4-Ready Signal with old NKC Software by Jens Mewes
   0.97	   Reset-Pulse duration set to 5ms by Jens Mewes
   0.98    Adapted for teh KEYr4 by Jens Mewes

**************************************************************************
*************************************************************************/

//#include "avr_defs.h"
//#include <FreeRTOS.h>
//#include <semphr.h>
#include "types.h"
#include "fifo.h"
#include <stdio.h>
#include "bflb_gpio.h"
#include "bflb_uart.h"
#include "hid_sync_led_control.h"
//#include "ps2interface.h"


//========================================================================
// Hardwiring the ATtiny2313 (new for KEYr4)
//========================================================================
/*
                             ____  ____
                   (RESET\) -|   \/   |- (Vcc)
              NC  (RxD/PD0) -|        |- (PB7) NC
       PS/2-Data  (TxD/PD1) -|        |- (PB6) KEY4-D6
              NC      (PA1) -|        |- (PB5) KEY4-D5
              NC      (PA0) -|        |- (PB4) KEY4-D4
        PS/2-Clk (INT0/PD2) -|        |- (PB3) KEY4-D3
     KEY4-Strobe (INT1/PD3) -|        |- (PB2) KEY4-D2
      KEY4-Ready      (PD4) -|        |- (PB1) KEY4-D1
              NC      (PD5) -|        |- (PB0) KEY4-D0
                      (GND) -|________|- (PD6) NKC-RESET\

   * KEY4-Strobe is active high
*/

//========================================================================
// Defines
//========================================================================

//------------------------------
// NKC-KEY interface
#if 0
#define NKCREADY_PIN         (PIND & BV(PD4))
#define NKCSTROBE_OUT        BV(PD3)
#define NKCSTROBE_PORT		 PORTD
#define NKCDATA_PORT         PORTB
// --- Handle strobe signal to latch keyboard data
#define NKC_STROBE           { NKCSTROBE_PORT |= NKCSTROBE_OUT;   NOP;   NKCSTROBE_PORT &= ~NKCSTROBE_OUT; }

//------------------------------
// External signals

#define ISC1MASK            (BV(ISC11)|BV(ISC10))
#define ISC1RISE            (BV(ISC11)|BV(ISC10))

#define NKCRESET_OUT        BV(PD6)
#define NKCSIG_PORT         PORTD

#define NKCREADY_IN         BV(PD4)
#endif
//------------------------------
// Host to keyboard commands

#define EXT_CODE            0xE0
#define SETLED_CODE         0xED
#define ECHO_CODE           0xEE
#define BREAK_CODE          0xF0
#define SET_TYPEMATIC_RATE  0xF3
#define KBD_ENABLE          0xF4

#define TYPEMATICRATE_30cps 0x00
#define TYPEMATICRATE_24cps 0x02
#define TYPEMATICRATE_21cps 0x04
#define TYPEMATICRATE_16cps 0x07
#define TYPEMATICRATE_12cps 0x0A
#define TYPEMATICRATE_10cps 0x0C
#define TYPEMATICRATE_8cps  0x0F
#define TYPEMATICRATE_6cps  0x12

#define TYPEMAT_DELAY_1s    0x60
#define TYPEMAT_DELAY_750ms 0x40
#define TYPEMAT_DELAY_500ms 0x20
#define TYPEMAT_DELAY_250ms 0x00

//------------------------------
// Keyboard LEDs

#define SCROLL_LOCK_LED     0x04
#define NUM_LOCK_LED        0x01
#define CAPS_LOCK_LED       0x02

//------------------------------
// Scancodes for special keys

#define LSHIFT_KEY          0x12
#define RSHIFT_KEY          0x59
#define SHIFTLOCK_KEY       0x58
#define CTRL_KEY            0x14
#define LALT_KEY            0x11
#define NUM_KEY             0x77
#define SCROLLLOCK_KEY		0x7E

//------------------------------
// Codes for Keystates

#define LSHIFT_STATE        0x01
#define RSHIFT_STATE        0x02
#define SHIFTLOCK_STATE     0x04
#define SHIFT_STATE_MASK    0x07
#define CTRL_STATE          0x08
#define SCROLLLOCK_STATE    0x10
#define LALT_STATE          0x20
#define RALT_STATE          0x40
#define ALT_STATE_MASK      0x60
#define NUMLOCK_STATE       0x80

//------------------------------
// FIFO

#define KEYBOARDBUFFER_SIZE   32

//========================================================================
// Local Variables
// (use volatile when variable is accessed from interrupts)
//========================================================================

static BOOL bKeyUpSeen;                          // Key Up (Break) code is previous data
static BOOL bExtenSeen;                          // Extention code is previous data
static BYTE KeyStates;                           // States of Shift, Ctrl, Alt keys is down

//static BYTE KeyboardBuffer [KEYBOARDBUFFER_SIZE]; // Keyboard buffer (32 bytes)
//static tFIFO KeyboardFIFO;                        // Management struct for Keyboard buffer

//static BOOL first;

//========================================================================
//========================================================================
// Interrupt Sevice Routines
//========================================================================
//========================================================================

//------------------------------------------------------------------------
// Default Interrupt Sevice Routine
//------------------------------------------------------------------------

//EMPTY_INTERRUPT(__vector_default);

//========================================================================
//========================================================================
// Transfer a byte to FIFO
//========================================================================

extern struct bflb_device_s *uart1;
//#include "bflb_uart.h"


// ============================================================================
// PUBLIC FUNCTION - Thread-safe wrapper to set LEDs from any thread
// ============================================================================

int set_keyboard_leds_thread_safe(int hid_index, uint8_t led_state);
/*{
    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    struct hid_led_request led_req;

    // If we're already in USB thread, call directly (avoid deadlock)
    if (current_task == usb_hid_task_handle) {
        printf("Setting LED from USB thread (direct call)\r\n");
        led_req.hid_index = hid_index;
        led_req.led_state = led_state;
        return set_keyboard_leds_internal(&led_req);
    }

    // From different thread: queue the request
    printf("Setting LED from different thread (queued)\r\n");
    
    led_req.hid_index = hid_index;
    led_req.led_state = led_state;
    led_req.completion_sem = xSemaphoreCreateBinary();
    led_req.result = -1;

    if (!led_req.completion_sem) {
        printf("Failed to create semaphore\r\n");
        return -1;
    }

    // Send request to USB thread queue
    BaseType_t queue_result = xQueueSend(hid_led_request_queue, &led_req, pdMS_TO_TICKS(100));
    if (queue_result != pdTRUE) {
        printf("Failed to queue LED request (queue full?)\r\n");
        vSemaphoreDelete(led_req.completion_sem);
        return -1;
    }

    // Wait for completion with timeout
    BaseType_t sem_result = xSemaphoreTake(led_req.completion_sem, pdMS_TO_TICKS(1000));
    if (sem_result != pdTRUE) {
        printf("LED request timeout\r\n");
        vSemaphoreDelete(led_req.completion_sem);
        return -255;
    }

    int result = led_req.result;
    vSemaphoreDelete(led_req.completion_sem);
    return result;
}*/

void TransferData2FIFO(const BYTE code)
{
    //FIFO_in(&KeyboardFIFO, code);
    if (code>=' ') {
      printf("Char:'%c' ",code);
    }else{
      printf("<0x%02X>",code);
    }
    bflb_uart_putchar(uart1,code);
}

//========================================================================
//========================================================================
// Transfer a byte to NKC
//========================================================================
/*
void TransferData2NKC(void)
{
   BYTE   data;

   //if ((first && FIFO_isfilled(KeyboardFIFO)) || (NKCREADY_PIN && FIFO_isfilled(KeyboardFIFO)))
   if ((first && FIFO_isfilled(KeyboardFIFO)) || (FIFO_isfilled(KeyboardFIFO)))

//   if ( NKCREADY_PIN && FIFO_isfilled(KeyboardFIFO) ) // Do we have data in keyboard buffer
   {                                                  // and NKC is ready to receive data?
      data = FIFO_out(&KeyboardFIFO);            // Get next data byte
#if 0      
      NKCDATA_PORT = data;                       // Transfer to NKC with 
	   NKCSTROBE_PORT &= ~NKCSTROBE_OUT;			 // strobe low
      NKC_STROBE;                                // Handle keyboard data strobe
#endif
	  first = FALSE;
   }
}*/

extern struct bflb_device_s *gpio;

//========================================================================
//========================================================================
// Set Keyboard LEDs
//========================================================================

void SetKeyboardLEDs(void)
{
   uint8_t LEDs = 0;

   if ( (KeyStates & SHIFTLOCK_STATE) )         LEDs |= CAPS_LOCK_LED;
   if ( (KeyStates & NUMLOCK_STATE) )           LEDs |= NUM_LOCK_LED;
   if ( (KeyStates & SCROLLLOCK_STATE) )        LEDs |= SCROLL_LOCK_LED;
#if 0
   Send2PS2Device(SETLED_CODE);                  // Send set led command
   WaitForACKorData();                           // Wait until keyboard acknoledge received
   Send2PS2Device(LEDs);                         // Send led bits
#endif

   set_keyboard_leds_thread_safe(0,LEDs);
      

}

//========================================================================
//========================================================================
// Handle Keyboard Power On Sequence
//========================================================================
#if 0
void HandlePowerOn(void)
{
   KeyStates = NUMLOCK_STATE;                    // Set numlock after keyboard startup
   SetKeyboardLEDs();

   Send2PS2Device(SET_TYPEMATIC_RATE);           // Send Set Typematic Rate/Delay command
   WaitForACKorData();                           // Wait until keyboard acknoledge received
   Send2PS2Device(TYPEMAT_DELAY_500ms|TYPEMATICRATE_16cps); // Send Typematic to 500ms / 16 cps (like Windows98)
   WaitForACKorData();                           // Wait until keyboard acknoledge received

   Send2PS2Device(KBD_ENABLE);                   // Send Enable Keyboard command
   WaitForACKorData();                           // Wait until keyboard acknoledge received
}
#endif

//========================================================================
//========================================================================
// Scancode Decoder - Fills keybuffer for output to NKC
//========================================================================
//========================================================================

//------------------------------------------------------------------------
// Code table for scancodes without shift key - Standard keypad
//------------------------------------------------------------------------

static const BYTE ScanCodesNormal [] =
{
//  -00-  -01-, -02-, -03-, -04-, -05-, -06-, -07-, -08-, -09-, -0A-, -0B-, -0C-, -0D-, -0E-, -0F-
    0x00, 0x92, 0x90, 0x8E, 0x8C, 0x8A, 0x8B, 0x95, 0x94, 0x93, 0x91, 0x8F, 0x8D, 0x09,  '^', 0x00,
//  -10-, -11-, -12-, -13-, -14-, -15-, -16-, -17-, -18-, -19-, -1A-, -1B-, -1C-, -1D-, -1E-, -1F-
    0x00, 0x00, 0x00, 0x00, 0x00,  'q',  '1', 0x00, 0x00, 0x00,  'y',  's',  'a',  'w',  '2', 0x00,
//  -20-, -21-, -22-, -23-, -24-, -25-, -26-, -27-, -28-, -29-, -2A-, -2B-, -2C-, -2D-, -2E-, -2F-
    0x00,  'c',  'x',  'd',  'e',  '4',  '3', 0x00, 0x00,  ' ',  'v',  'f',  't',  'r',  '5', 0x00,
//  -30-, -31-, -32-, -33-, -34-, -35-, -36-, -37-, -38-, -39-, -3A-, -3B-, -3C-, -3D-, -3E-, -3F-
    0x00,  'n',  'b',  'h',  'g',  'z',  '6', 0x00, 0x00, 0x00,  'm',  'j',  'u',  '7',  '8', 0x00,
//  -40-, -41-, -42-, -43-, -44-, -45-, -46-, -47-, -48-, -49-, -4A-, -4B-, -4C-, -4D-, -4E-, -4F-
    0x00,  ',',  'k',  'i',  'o',  '0',  '9', 0x00, 0x00,  '.',  '-',  'l', 0x7C,  'p', 0x7E, 0x00,
//  -50-, -51-, -52-, -53-, -54-, -55-, -56-, -57-, -58-, -59-, -5A-, -5B-, -5C-, -5D-, -5E-, -5F-
    0x00, 0x00, 0x7B, 0x00, 0x7D, 0x60, 0x00, 0x00, 0x00, 0x00, 0x0D,  '+', 0x00,  '#', 0x00, 0x00,
//  -60-, -61-, -62-, -63-, -64-, -65-, -66-, -67-,
    0x00,  '<', 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
};

//------------------------------------------------------------------------
// Code table for scancodes with shift key - Standard keypad
//------------------------------------------------------------------------

static const BYTE ScanCodesShift [] =
{
//  -00-  -01-, -02-, -03-, -04-, -05-, -06-, -07-, -08-, -09-, -0A-, -0B-, -0C-, -0D-, -0E-, -0F-
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x40, 0x00,
//  -10-, -11-, -12-, -13-, -14-, -15-, -16-, -17-, -18-, -19-, -1A-, -1B-, -1C-, -1D-, -1E-, -1F-
    0x00, 0x00, 0x00, 0x00, 0x00,  'Q',  '!', 0x00, 0x00, 0x00,  'Y',  'S',  'A',  'W', 0x22, 0x00,
//  -20-, -21-, -22-, -23-, -24-, -25-, -26-, -27-, -28-, -29-, -2A-, -2B-, -2C-, -2D-, -2E-, -2F-
    0x00,  'C',  'X',  'D',  'E',  '$',  '#', 0x00, 0x00,  ' ',  'V',  'F',  'T',  'R',  '%', 0x00,
//  -30-, -31-, -32-, -33-, -34-, -35-, -36-, -37-, -38-, -39-, -3A-, -3B-, -3C-, -3D-, -3E-, -3F-
    0x00,  'N',  'B',  'H',  'G',  'Z',  '&', 0x00, 0x00, 0x00,  'M',  'J',  'U',  '/',  '(', 0x00,
//  -40-, -41-, -42-, -43-, -44-, -45-, -46-, -47-, -48-, -49-, -4A-, -4B-, -4C-, -4D-, -4E-, -4F-
    0x00,  ';',  'K',  'I',  'O',  '=',  ')', 0x00, 0x00,  ':',  '_',  'L', 0x5C,  'P',  '?', 0x00,
//  -50-, -51-, -52-, -53-, -54-, -55-, -56-, -57-, -58-, -59-, -5A-, -5B-, -5C-, -5D-, -5E-, -5F-
    0x00, 0x00, 0x5B, 0x00, 0x5D, 0x27, 0x00, 0x00, 0x00, 0x00, 0x0D,  '*', 0x00, 0x27, 0x00, 0x00,
//  -60-, -61-, -62-, -63-, -64-, -65-, -66-, -67-, -68-,
    0x00,  '>', 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00,
};

//------------------------------------------------------------------------
// Code table for scancodes without num key - Numeric keypad
//------------------------------------------------------------------------

static const BYTE ScanCodesNumeric [] =
{
//                                                  -68-, -69-, -6A-, -6B-, -6C-, -6D-, -6E-, -6F-
                                                    0x96, 0x87, 0x00, 0x80, 0x86, 0x00, 0x00, 0x00,
//  -70-, -71-, -72-, -73-, -74-, -75-, -76-, -77-, -78-, -79-, -7A-, -7B-, -7C-, -7D-, -7E-, -7F-
    0x88, 0x89, 0x83, 0x00, 0x81, 0x82, 0x1B, 0x00, 0x00,  '+', 0x85,  '-', 0x97, 0x84, 0x00, 0x00,
};

//------------------------------------------------------------------------
// Code table for scancodes with num key - Numeric keypad
//------------------------------------------------------------------------

static const BYTE ScanCodesNumericNum [] =
{
//                                                  -68-, -69-, -6A-, -6B-, -6C-, -6D-, -6E-, -6F-
                                                    0x96,  '1', 0x00,  '4',  '7', 0x00, 0x00, 0x00,
//  -70-, -71-, -72-, -73-, -74-, -75-, -76-, -77-, -78-, -79-, -7A-, -7B-, -7C-, -7D-, -7E-, -7F-
     '0',  ',',  '2',  '5',  '6',  '8', 0x1B, 0x00, 0x00,  '+',  '3',  '-', 0x97,  '9', 0x00, 0x00,
};

//------------------------------------------------------------------------
// Code table for scancodes with num key - Numeric keypad
//------------------------------------------------------------------------

static const BYTE ScanCodesNumericScroll [] =
{
//                                                  -68-, -69-, -6A-, -6B-, -6C-, -6D-, -6E-, -6F-
                                                    0x96, 0x02, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00,
//  -70-, -71-, -72-, -73-, -74-, -75-, -76-, -77-, -78-, -79-, -7A-, -7B-, -7C-, -7D-, -7E-, -7F-
    0x88, 0x89, 0x83, 0x00, 0x81, 0x82, 0x1B, 0x00, 0x00,  '+', 0x85,  '-', 0x10, 0x84, 0x00, 0x00,
};


//------------------------------------------------------------------------
// Code table for scancodes of special keys (> 0x80 in other tables)
//------------------------------------------------------------------------

// NKC codes without ctrl key
static const BYTE SpecialCodes [] =
{
  0x13, 0x00,      // -80-    Move cursor left     / Arrow left
  0x04, 0x00,      // -81-    Move cursor right    / Arrow right
  0x05, 0x00,      // -82-    Move cursor up       / Arrow up
  0x18, 0x00,      // -83-    Move cursor down     / Arrow down
  0x12, 0x00,      // -84-    Page up              / Page up
  0x03, 0x00,      // -85-    Page down            / Page down
  0x11,  'S',      // -86-    Start of line        / Home
  0x11,  'D',      // -87-    End of Line          / End
  0x16, 0x00,      // -88-    Insert/Override      / Insert
  0x07, 0x00,      // -89-    Delete right char    / Delete

  0x0A, 0x00,      // -8A-    Help                 / F1
  0x0B,  'A',      // -8B-    Assembler            / F2
  0x0C, 0x00,      // -8C-    Repeat Search        / F3
  0x11,  'F',      // -8D-    Search               / F4
  0x0B,  'B',      // -8E-    Block begin          / F5
  0x0B,  'K',      // -8F-    Block end            / F6
  0x0B,  'C',      // -90-    Copy Block           / F7
  0x0B,  'V',      // -91-    Move Block           / F8
  0x10, 0x00,      // -92-    Characterset         / F9
  0x0B,  'X',      // -93-    End Editor           / F10
  0x00, 0x00,      // -94-                         / F11
  0x00, 0x00,      // -95-                         / F12
  0x11,  'B',      // -96-                         / WinL
  0x1b,  'P',      // -97-                         / Druck
};

// NKC codes with ctrl key
static const BYTE SpecialCodesCtrl [] =
{
  0x01, 0x00,      // -80-    Move word left       / Arrow left
  0x06, 0x00,      // -81-    Move word right      / Arrow right
  0x1A, 0x00,      // -82-    Move line up         / Arrow up
  0x17, 0x00,      // -83-    Move line down       / Arrow down
  0x11,  'E',      // -84-    Move to top of Page  / Page up
  0x11,  'X',      // -85-    Move to bottom of page / Page down
  0x11,  'R',      // -86-    Start of Text        / Home
  0x11,  'C',      // -87-    End of Text          / End
  0x0E, 0x00,      // -88-    Insert line          / Insert
  0x19, 0x00,      // -89-    Delete line          / Delete

  0x0A, 0x00,      // -8A-    Help                 / F1
  0x0B,  'A',      // -8B-    Assembler            / F2
  0x0C, 0x00,      // -8C-    Repeat Search        / F3
  0x11,  'A',      // -8F-    Replace              / F4
  0x0B,  'H',      // -8E-    Delete blockmarks    / F5
  0x0B,  'Y',      // -8D-    Deleteblock          / F6
  0x11,  'T',      // -90-    Split line           / F7
  0x11,  'V',      // -91-    Merge line           / F8
  0x1B,  'S',      // -92-    Scrollmode           / F9
  0x0B,  'Q',      // -93-    End Editor           / F10
  0x00, 0x00,      // -94-                         / F11
  0x00, 0x00,      // -95-                         / F12
  0x11,  'B',      // -96-                         / WinL
  0x1b,  'P',      // -97-                         / Druck
};

//------------------------------------------------------------------------
// Translates scancodes to NKC codes
//------------------------------------------------------------------------

void SetKeyState(BYTE nState)
{
   if ( bKeyUpSeen )   { KeyStates &= ~nState;   bKeyUpSeen = FALSE; }
   else                  KeyStates |= nState; 
   bExtenSeen = FALSE;
}

//------------------------------------------------------------------------


void DecodeKey2NKC(BYTE ScanCode)
{
   BYTE      NKCcode = 0;
   BYTE      a, b;
   LPCPBYTE  cp;
   //int		 i;


   switch ( ScanCode )
   {
     case EXT_CODE:
       bExtenSeen = TRUE;
       break;

     case BREAK_CODE:
       bKeyUpSeen = TRUE;
       break;

     case LSHIFT_KEY:
       if ( bExtenSeen )    bKeyUpSeen = bExtenSeen = FALSE; // Ignore fake shift key
       else
         SetKeyState(LSHIFT_STATE);
       break;

     case RSHIFT_KEY:
       SetKeyState(RSHIFT_STATE);
       break;

     case CTRL_KEY:
       SetKeyState(CTRL_STATE);
       break;

     case LALT_KEY:
       if ( bExtenSeen )
          SetKeyState(RALT_STATE);
       else
          SetKeyState(LALT_STATE);
       break;

     case SHIFTLOCK_KEY:
       if ( bKeyUpSeen )    bKeyUpSeen = FALSE;
       else                 KeyStates ^= SHIFTLOCK_STATE;
       SetKeyboardLEDs();
       break;

     case NUM_KEY:
       if ( bKeyUpSeen )   {bKeyUpSeen = FALSE; }
       else                 KeyStates ^= NUMLOCK_STATE; 
       SetKeyboardLEDs();
       break;

	 case SCROLLLOCK_KEY:
       if ( bKeyUpSeen )   {bKeyUpSeen = FALSE; }
       else                 KeyStates ^= SCROLLLOCK_STATE; 
       SetKeyboardLEDs();
       break;


     default:
       if ( ScanCode == 0x83 )   ScanCode = 0x02;     // Handle F7 key. Remap scancode
       if ( ScanCode == 0x78 )   ScanCode = 0x08;     // Handle F11 key. Remap scancode
	   if ( ScanCode == 0x1F )   ScanCode = 0x68;     // Handle WinL Key. Remap scancode
       if ( bKeyUpSeen )    bKeyUpSeen = bExtenSeen = FALSE;
       else if ( bExtenSeen )
       {
          bExtenSeen = FALSE;
          // Handle keys with Extension Code
		    if (ScanCode == 0x4A )                 // '/' key on numeric keypad
		       NKCcode = '/';
		    else if (ScanCode == 0x5A )            // 'Enter' key on numeric keypad
		       //NKCcode = GET_PBYTE(ScanCodesNormal+ScanCode);
             NKCcode = ScanCodesNormal[ScanCode];
            else if ( ScanCode > 0x67 && ScanCode < 0x80 )
			{
			   if ( (KeyStates & SCROLLLOCK_STATE) )
		  	       //NKCcode = GET_PBYTE(ScanCodesNumericScroll+ScanCode-0x68);
                NKCcode = ScanCodesNumericScroll[ScanCode-0x68];
			   else
                   //NKCcode = GET_PBYTE(ScanCodesNumeric+ScanCode-0x68);
                   NKCcode = ScanCodesNumeric[ScanCode-0x68];
			}
       }
	   else if ( ScanCode == 0x15 && KeyStates & RALT_STATE)
	   {
	       NKCcode = 0x40;						   // AltGr+q = @
		   break;
       }
       // Handle Standard keypad
       else if ( ScanCode < 0x68 )
       {
          if ( (KeyStates & CTRL_STATE) )
          {
             //NKCcode = GET_PBYTE(ScanCodesNormal+ScanCode);
             NKCcode = ScanCodesNormal[ScanCode];
             if ( NKCcode >= 0x60 && NKCcode <= 0x7F )
                NKCcode &= 0x1F;                 // Create CTRL-Codes
             else if (NKCcode < 0x80 )           
                NKCcode = 0;                     // Everything else but special codes must be zero
          }
          else if ( (KeyStates & SHIFT_STATE_MASK) )
             //NKCcode = GET_PBYTE(ScanCodesShift+ScanCode);
             NKCcode = ScanCodesShift[ScanCode];
          else
             //NKCcode = GET_PBYTE(ScanCodesNormal+ScanCode);
             NKCcode = ScanCodesNormal[ScanCode];
       }
       // Handle Numeric keypad
       else if ( ScanCode < 0x80 )
       {
          ScanCode -= 0x68;
		  if ( (KeyStates & NUMLOCK_STATE) )
             //NKCcode = GET_PBYTE(ScanCodesNumericNum+ScanCode);
             NKCcode = ScanCodesNumericNum[ScanCode];
          else
             //NKCcode = GET_PBYTE(ScanCodesNumeric+ScanCode);
             NKCcode = ScanCodesNumeric[ScanCode];
       }
       break;
   }

   if ( NKCcode )
   {
      a = (KeyStates & CTRL_STATE);
      b = (KeyStates & ALT_STATE_MASK);
      if ( (NKCcode >= 0x80) || (a && NKCcode == 0x1B) )
      {  // Handle NKC extension code table
         if ( (a && b && NKCcode == 0x89 ) || (a && NKCcode == 0x1B) )
         { // Ctrl+Alt+Del detected or CTRL+ESC
           // Reset NKC - CPU (-> NKC-RESET\)
            /*NKCSIG_PORT &= ~NKCRESET_OUT;
            for (i=0;i<20;i++){
            Wait_us(256);
            }
            NKCSIG_PORT |= NKCRESET_OUT;*/
            printf("ctrl-alt-delete\n");
            return;
         }
         else
         {
            NKCcode <<= 1;                          // => NKCcode -= 0x80; NKCcode *= 2;
            cp = (LPCPBYTE)SpecialCodes;
            if ( a )
               cp = (LPCPBYTE)SpecialCodesCtrl;
            //if ( (a = GET_PBYTE(cp+NKCcode)) )      // Must not be zero
            if ( (a = cp[NKCcode]) )      // Must not be zero
            {
               TransferData2FIFO(a);                // Add byte to keyboard buffer
               //if ( (a = GET_PBYTE(cp+NKCcode+1)) ) // Must not be zero
               if ( (a = cp[NKCcode+1]) )           // Must not be zero
                  TransferData2FIFO(a);             // Add byte to keyboard buffer
            }
         }
      }
	  else
	  {
         // Handle standard code table
         TransferData2FIFO(NKCcode);
      }
   }
}


//========================================================================
//========================================================================
// MAIN
//========================================================================
#if 0
int main (void)
{
   BYTE temp;
   BOOL bRepeatSend = FALSE;                

   DDRA  = 0xFF;    PORTA = 0xFF;                // Set all ports to output first
   DDRB  = 0xFF;    PORTB = 0xFF;
   DDRD  = 0xFF;    PORTD = 0xFF;

   Set_Sleep_Mode(SLEEP_MODE_IDLE);              // Initialize Sleep Mode for AVR
   WDTI_Enable(WDT_32MS);                        // Enable Watchdog Timer Interrupt

   NKCSTROBE_PORT &= ~NKCSTROBE_OUT;               // Set strobe to low level so NKC can latch the reset of key data port
   PS2DATA_DDR  &= ~(PS2DATA_CLK | PS2DATA_OUT | NKCREADY_IN); // Clock, data and ready pin as input
   MCUCR = (MCUCR & (~(ISC0MASK|ISC1MASK)))|ISC0FALL|ISC1RISE; // Set INT0 on falling edge, INT1 on rising edge
   GIMSK |= BV(INT0);                 // Enable external INT0

   FIFO_init(&KeyboardFIFO, KeyboardBuffer, KEYBOARDBUFFER_SIZE);

   first = TRUE;

   SEI;                                          // Enable all used ISR

   while ( TRUE )
   {
       // Wait until data byte received
      while ( PS2State != PS2STATE_DATA_RECEIVED )
      {
         SLEEP;
         TransferData2NKC();                     // Transfer next data byte to NKC
      }
      PS2State = PS2STATE_IDLE;                  // Reset State to idle.
      // Check parity bit
      temp = rParityBits & 0x01;
      if ( temp != rParity && (! bRepeatSend) )
      {
         Send2PS2Device(RESEND_CODE);            // Send repeat last byte command on fail
         bRepeatSend = TRUE;                     // Only one reapeat command per data byte
      }
      else
      {
         bRepeatSend = FALSE;                    // Reset repeat indicator
         if ( rData == PWRON_CODE )              // Keyboard Power On received
            HandlePowerOn();                     // Handle Keyboard Power On Sequence
         else
         {
            DecodeKey2NKC(rData);                // Decode scancode to NKC code
         }
      }
      TransferData2NKC();                        // Transfer next data byte to NKC
   }
}
#endif
//========================================================================
