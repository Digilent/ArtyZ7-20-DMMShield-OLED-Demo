/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    uart.c

  @Description
        This file groups the functions that implement some of the utilities functions, like delay.
        utils.h file needs to be included in the files where those functions are used.


  @Author
    Cristian Fatu
    cristian.fatu@digilent.ro

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library

 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
#include "utils.h"
/* ************************************************************************** */

/* ------------------------------------------------------------ */
/***    Delay100Us
**
**	Synopsis:
**		Delay100Us(t10usDelay)
**
**	Parameters:
**		t10usDelay - the amount of time you wish to delay in hundreds of microseconds
**
**	Return Values:
**      none
**
**	Errors:
**		none
**
**	Description:
**		This procedure delays program execution for the specified number
**      of microseconds. This delay is not precise.
**
**	Note:
**		This routine is written with the assumption that the
**		system clock is 40 MHz.
*/
void DelayAprox10Us( unsigned int  t10usDelay )
{
    int j;
    while ( 0 < t10usDelay )
    {
        t10usDelay--;
        j = 750;
        while ( 0 < j )
        {
            j--;
        }   // end while
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing
        asm volatile("nop"); // do nothing

    }   // end while
}
/* ------------------------------------------------------------ */
/***    GetBufferChecksum
**
**	Synopsis:
**		GetBufferChecksum(*pBuf, len)
**
**	Parameters:
**		pBuf - buffer for which the checksum is computed
**      len - buffer length on which the checksum is computed
**
**	Return Values:
**      returns the value of checksum, computed for the specified pBuf, on the specified len
**
**	Errors:
**		none
**
**	Description:
**		This function computes the checksum for the specified parameters: buffer and its length
**      Returns the value of checksum
**
**	Note:
**		This routine is written with the assumption that the
**		system clock is 40 MHz.
*/
unsigned char GetBufferChecksum(unsigned char *pBuf, int len)
{
    int i;
    unsigned char checksum = 0;
    for(i =0; i < len; i++)
    {
        checksum += pBuf[i];
    }
    return checksum;
}


/* *****************************************************************************
 End of File
 */
