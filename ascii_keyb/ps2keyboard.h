/*************************************************************************
**************************************************************************
**                                                                      **
**  Project:   PS/2 Keyboard => NKC                                     **
**  Title:     PS/2 Interface => NKC Declaration                        **
**  File:      ps2interface.h                                           **
**  Date:      12.02.2007                                               **
**  Version:   0.95                                                     **
**  Plattform:                                                          **
**                                                                      **
**  Created by Gerald Ebert                                             **
**                                                                      **
**************************************************************************
*************************************************************************/

#ifndef _PS2KEYBOARD_H_
  #define _PS2KEYBOARD_H_

  #include "types.h"

  void DecodeKey2NKC(BYTE ScanCode);

#endif // _PS2KEYBOARD_H_
