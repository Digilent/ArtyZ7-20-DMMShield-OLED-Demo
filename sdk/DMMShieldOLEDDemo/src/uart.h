/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    uart.h

  @Description
        This file groups the declarations of the functions that implement
        the UART module (defined in uart.c).

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library

 */

#ifndef UART_H
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "xuartps.h"
#include "xparameters.h"

#define UART_DRIVER		XUartPs
#define	MAX_RCVCMD_LEN 0x40	// maximum number of characters a CR+LF terminated string

/************************** Function Prototypes ******************************/
u8 UART_Init(u32 dwBaudRate);

int UART_GetString(char* pchBuff, int cchBuff);
void UART_PutString(char szData[]);
#define UART_PutString1(x,...) 	{ xil_printf(x,##__VA_ARGS__); print("\r\n"); }

#ifdef __cplusplus
}
#endif

#endif
