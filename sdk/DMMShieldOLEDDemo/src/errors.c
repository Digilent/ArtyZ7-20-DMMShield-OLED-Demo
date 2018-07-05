/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    errors.c

  @Description
        This module groups the error related functions.


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
#include <stdio.h>
#include <string.h>
#include "stdint.h"
#include "errors.h"


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Constants                                                         */
/* ************************************************************************** */
/* ************************************************************************** */
#define PREFIX_SIZE 10

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Global Variables                                                  */
/* ************************************************************************** */
/* ************************************************************************** */
char szLastError[MSG_ERROR_SIZE];
char prefixes[2][PREFIX_SIZE];

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

void ERRORS_Init(const char *szPrefixSuccess, const char *szPrefixError)
{
    strncpy(prefixes[(int)PREFIX_SUCCESS], szPrefixSuccess, PREFIX_SIZE - 1);
    strncpy(prefixes[(int)PREFIX_ERROR], szPrefixError, PREFIX_SIZE - 1);
}

/* ------------------------------------------------------------ */
/***    ERRORS_GetPrefixedMessageString
**
**	Synopsis:
**		
**
**	Parameters:
**      uint8_t bErrCode  - The error code for which the error string is requested/needed to be transmitted further to the user
**      char *szContent         - The characters string acting as content for some of the error messages
**      char *pSzErr            - String to receive the error meaning
**		
**
**	Return Values:
**      ERRVAL_SUCCESS                   0   - success
**      ERRVAL_CMD_MISSINGCODE        0xF9   - The provided code is not among accepted values
**
**	Errors:
**		none
**
**	Description:
**		This function copies in the szLastError the error message corresponding to the provided error code, 
**      for all error codes except ERRVAL_SUCCESS.
**      Then it copies in the pSzErr the properly prefixed message.
**      Therefore is important that the caller of this function allocates enough space in pSzErr (70 characters).
**      Some of the error messages include the string provided in the szContent parameter. The parameter is ignored for the other error messages.
**      If the error code is among the defined ones, a specific error string is copied in the pSzErr
**      and ERRVAL_SUCCESS is returned.
**      If the error is not among the defined ones, ERRVAL_CMD_MISSINGCODE is returned and pSzErr is not altered.
**		
*/
uint8_t ERRORS_GetPrefixedMessageString(uint8_t bErrCode, char *szContent, char *pSzErr)
{
    uint8_t bResult = ERRVAL_SUCCESS;
    msg_prefix_status prefix;
    switch(bErrCode)
    {
        case ERRVAL_SUCCESS: 
//          the message is in pSzErr string
            strcpy(szLastError, pSzErr);
            prefix = PREFIX_SUCCESS;
            break;
        case ERRVAL_DMM_CFGVERIFY:
            strcpy(szLastError, "DMM Configuration verify error");
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_EPROM_WRTIMEOUT:
            strcpy(szLastError, "EPROM write data ready timeout");
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_EPROM_CRC:
            strcpy(szLastError, "Invalid EPROM checksum");
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_EPROM_MAGICNO:
            strcpy(szLastError, "Invalid EPROM magic number");  
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_DMM_IDXCONFIG:
            strcpy(szLastError, "Invalid scale index");
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_DMM_VALIDDATATIMEOUT:
            strcpy(szLastError, "Valid DMM data timeout");  
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_CALIB_NANDOUBLE:
            strcpy(szLastError, "The provided reference value has no valid value.");  
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_CMD_WRONGPARAMS:
            strcpy(szLastError, "The expected parameters were not provided on the UART command.");  
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_CMD_MISSINGCODE:
            strcpy(szLastError, "The provided code is not among accepted values.");  
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_EPROM_VERIFY:
            strcpy(szLastError, "EPROM verify error.");  
            prefix = PREFIX_ERROR;
            break;
        case ERRVAL_EPROM_ADDR_VIOLATION:
            strcpy(szLastError, "EPROM address violation: attempt to write over system data.");  
            prefix = PREFIX_ERROR;
            break;            
        case ERRVAL_CMD_VALWRONGUNIT:
            sprintf(szLastError, "The provided value \"%s\" has a wrong measure unit.", szContent);  
            prefix = PREFIX_ERROR;
            break;          
        case ERRVAL_CMD_VALFORMAT:
            sprintf(szLastError, "The provided value \"%s\" has a wrong format.", szContent);  
            prefix = PREFIX_ERROR;
            break;       
        case ERRVAL_DMM_MEASUREDISPERSION:
            // szLastError already contains the error message
            prefix = PREFIX_ERROR;
            break;   
        case ERRVAL_CALIB_MISSINGMEASUREMENT:
            strcpy(szLastError, "A measurement must be performed before calling the finalize calibration.");
            prefix = PREFIX_ERROR;
            break;       
        case ERRVAL_DMM_GENERICERROR:
//          the message is in pSzErr string
            strcpy(szLastError, pSzErr);
            prefix = PREFIX_ERROR;
            break;   
        case ERRVAL_DMM_UARTERROR:
        	strcpy(szLastError, "UART Init error");
            prefix = PREFIX_ERROR;
            break;
        default:
            bResult = ERRVAL_CMD_MISSINGCODE;
            break;        
    }
    ERRORS_PrefixMessage(prefix, pSzErr, szLastError);

    return bResult;
    
}
/* ------------------------------------------------------------ */
/***    ERRORS_GetszLastError
**
**	Synopsis:
**		
**
**	Parameters:
**      none
**		
**	Return Values:
**      char szLastError - last saved error code
**
**	Errors:
**		none
**
**	Description:
**		This function returns the last saved error code.
**      
**		
*/
char *ERRORS_GetszLastError()
{
    return szLastError;
}

char* ERRORS_PrefixMessage(msg_prefix_status prefix, char *pDestString, const char *szMsg)
{
    if(prefix != PREFIX_EMPTY)
    {
        strcpy(pDestString, prefixes[(int)prefix]); // copy prefix
        if(szMsg[0])
        {
            strcat(pDestString, ", ");
        }        
    }
    else
    {
        pDestString[0] = 0; // empty string
    }
    strcat(pDestString, szMsg);
    strcat(pDestString, "\r\n");
    return pDestString;
}

/* *****************************************************************************
 End of File
 */
