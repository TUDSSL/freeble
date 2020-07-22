/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co KG                  *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 2014  SEGGER Microcontroller GmbH & Co KG                *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File    : RTT_Syscalls.c
Purpose : Low-level functions for using printf() via RTT in GCC
--------  END-OF-HEADER  ---------------------------------------------
*/
#include "SEGGER_RTT.h"
#include "simple_uart.h"

/*********************************************************************
*
*       Function prototypes
*
**********************************************************************
*/
int _write(int file, char *ptr, int len);

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*       _write()
*
* Function description
*   Low-level write function.
*   libc subroutines will use this system routine for output to all files,
*   including stdout.
*   Write data via RTT.
*/
int _write(int file, char *ptr, int len) {
  (void) file;  /* Not used, avoid warning */
  SEGGER_RTT_Write(0, ptr, len);
#ifdef UART_DEBUG
  log_uart_len(ptr, (uint8_t)len+1);
#endif
  return len;
}

/****** End Of File *************************************************/