/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    dmmcmd.c

  @Description
        The DMMCMD module groups the functions that implement the DMM command dispatch module.
        The basic functionality is to interpret the UART received content, recognize specific commands
        and call the appropriate functions from DMM, CALIB, SERIALNO modules.
        The module also provides an initialization function, that initializes the above mentioned modules
        whose functions are called from within the command interpreter function.
        The module also initializes a PmodOLED and implements displaying the basic DMM information on the PmodOLED.
        The "Interface functions" section groups functions that can also be called by User.
        The "Local functions" section groups low level functions that are only called from within the current module.
		In order to successfully communicate you must set your terminal to 115200 Baud, 8 data bits, 1 stop bit, no parity, and configure the transmitted content to be followed by CR+LF
		The PmodOLED must be plugged in JA Pmod connector.

  @Author
    Cristian Fatu
    cristian.fatu@digilent.ro

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library
 	 Cristian Fatu - 2018/06/29 - Modified for OLED Demo, to initialize PmodOLED, display on PmodOLED

 */

#include "xparameters.h"
#include "dmmcmd.h"
#include "errors.h"
#include "serialno.h"
#include "dmm.h"
#include "calib.h"
#include "uart.h"
#include "utils.h"
#include "PmodOLED.h"

#define MAX_CMD_LENGTH			100

#define CMD_REPEAT_THRESHOLD 	50000000




/********************* Global Constant Definitions ***************************/
const cmd_map_t uartCommands[] = {
	{"DMMConfig",   		CMD_Config},
	{"DMMCalibP",   		CMD_CalibP},
	{"DMMCalibN",   		CMD_CalibN},
	{"DMMCalibZ",   		CMD_CalibZ},
	{"DMMMeasureRep",   	CMD_MeasureRep},
	{"DMMMeasureStop",		CMD_MeasureStop},
	{"DMMMeasureRaw",   	CMD_MeasureRaw},
	{"DMMMeasureAvg",   	CMD_MeasureAvg},
	{"DMMSaveEPROM",   		CMD_SaveEPROM},
	{"DMMVerifyEPROM",  	CMD_VerifyEPROM},
	{"DMMExportCalib",   	CMD_ExportCalib},
	{"DMMImportCalib",		CMD_ImportCalib},
	{"DMMMeasureForCalibP", CMD_MeasureForCalibP},
	{"DMMMeasureForCalibN", CMD_MeasureForCalibN},
	{"DMMFinalizeCalibP",	CMD_FinalizeCalibP},
	{"DMMFinalizeCalibN",   CMD_FinalizeCalibN},
	{"DMMRestoreFactCalibs",CMD_RestoreFactCalibs},
	{"DMMReadSerialNo",   	CMD_ReadSerialNo}
};

const char rgScales[][20] = {"Resistance50M", "Resistance5M", "Resistance500k", "Resistance50k", "Resistance5k", "Resistance500", "Resistance50",
                         "VoltageDC50", "VoltageDC5", "VoltageDC500m", "VoltageDC50m",
                         "VoltageAC30", "VoltageAC5", "VoltageAC500m", "VoltageAC50m",
                         "CurrentDC5", "CurrentAC5",
                         "Continuity", "Diode",
                         "CurrentDC500m", "CurrentDC50m", "CurrentDC5m", "CurrentDC500u",
                         "CurrentAC500m", "CurrentAC50m", "CurrentAC5m", "CurrentAC500u"};
/********************* Global Variables Definitions ***************************/
char szMsg[200];


char *pszLastErr;

// variables used in multiple functions// allocate them only once.
char szVal[20];
char szRefVal[20];
double dRefVal, dMeasuredVal, dispersion;


// flags for repeated value and repeated raw value
uint8_t fRepGetVal = 0;
uint8_t fRepGetRaw = 0;
uint8_t fRepBlock = 0;

PmodOLED myPmodOLEDDevice;


const u8 pmodOLED_orientation = 0b0;  //set up for Normal PmodOLED(false) vs normal Onboard OLED(true)
const u8 pmodOLED_invert = 0b0;       //true = whitebackground/black letters      false = black background /white letters
/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions Prototypes                                        */
/* ************************************************************************** */
/* ************************************************************************** */

cmd_key_t DMMCMD_CmdDecode(char *szCmd);
void DMMCMD_ProcessCmd(cmd_key_t keyCmd);
char* DMMCMD_CmdGetNextArg();
uint8_t DMMCMD_ProcessRepeatedCmd();
// individual commands functions
u8 DMMCMD_CmdConfig(char const *arg0);
u8 DMMCMD_CmdMeasureRep();
u8 DMMCMD_CmdMeasureStop();
u8 DMMCMD_CmdMeasureRaw();
u8 DMMCMD_CmdMeasureAvg();
u8 DMMCMD_CmdCalibP(char const *arg0);
u8 DMMCMD_CmdCalibN(char const *arg0);
u8 DMMCMD_CmdCalibZ();
u8 DMMCMD_CmdSaveEPROM();
u8 DMMCMD_CmdVerifyEPROM();
u8 DMMCMD_CmdExportCalib();
u8 DMMCMD_CmdImportCalib(char const *arg0, char const *arg1, char const *arg2);
u8 DMMCMD_CmdMeasureForCalibP();
u8 DMMCMD_CmdMeasureForCalibN();
u8 DMMCMD_CmdFinalizeCalibP(char const *arg0);
u8 DMMCMD_CmdFinalizeCalibN(char const *arg0);
u8 DMMCMD_CmdRestoreFactCalib();
u8 DMMCMD_CmdReadSerialNo();
void DMMCMD_PmodOLEDDisplay(char *pszVal);
/********************* Function Definitions ***************************/

/***	DMMCMD_Init()
**
**	Parameters:
**		none
**
**	Return Value:
**		uint8_t
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_EPROM_MAGICNO            0xFD    // wrong Magic No. when reading data from EPROM
**          ERRVAL_EPROM_CRC                0xFE    // wrong CRC when reading data from EPROM
**
**	Description:
**		This function initializes the modules involved in the DMMCMD module.
**      It initializes the DMM, UART, CALIB and SERIALNO modules.
**      It also initializes PmodOLED.
**      The return values are related to errors when calibration is read from user calibration area of EPROM during calibration initialization call.
**      The function returns ERRVAL_SUCCESS for success.
**      The function returns ERRVAL_EPROM_MAGICNO when a wrong magic number was detected in the data read from EPROM.
**      The function returns ERRVAL_EPROM_CRC when the checksum is wrong for the data read from EPROM.
**
*/
u8 DMMCMD_Init()
{
	uint8_t bErrCode = ERRVAL_SUCCESS;
    DMM_Init();

	bErrCode = CALIB_Init();
	// no need to process error code as this can be the first run of DMMShield (Calibration not present)
    bErrCode = UART_Init(115200);
    if(bErrCode == ERRVAL_SUCCESS)
    {
    	SERIALNO_Init();
    }
	pszLastErr = ERRORS_GetszLastError();

	// initialize PmodOLED
    OLED_Begin(&myPmodOLEDDevice, XPAR_PMODOLED_0_AXI_LITE_GPIO_BASEADDR, XPAR_PMODOLED_0_AXI_LITE_SPI_BASEADDR, pmodOLED_orientation, pmodOLED_invert);
    //Turn automatic updating off
    OLED_SetCharUpdate(&myPmodOLEDDevice, 0);
    OLED_DisplayOn(&myPmodOLEDDevice);

    DMMCMD_PmodOLEDDisplay("No value");
	return bErrCode;
}

/***	DMMCMD_CheckForCommand()
**
**	Parameters:
**          none
**
**
**	Return Value:
**          none
**
**	Description:
**		This function checks on UART if a command was received.
**      It compares the received command with the commands defined in the commands array. If recognized, the command is processed accordingly.
**      It also performs the repeated commands.
**
*/
void DMMCMD_CheckForCommand()
{
    char uartCmd[MAX_RCVCMD_LEN];
    int cchi;
    cchi = UART_GetString(uartCmd, MAX_RCVCMD_LEN);
    if(cchi > 0)
    {
	    sprintf(szMsg, "Received command: %s\r\n", uartCmd);
	    UART_PutString(szMsg);
	    DMMCMD_ProcessCmd(DMMCMD_CmdDecode(uartCmd));
	    fRepBlock = 0;
    }

    DMMCMD_ProcessRepeatedCmd();
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */
/*****************************************************************************/

/*****************************************************************************/
/***	DMMCMD_CmdDecode
**
**	Parameters:
**		char *szCmd       - zero terminated string that contains the command to be searched
**
**	Return Value:
**          cmd_key_t - the command enumeration value for the found command, or INVALID if command was not found.
**
**	Description:
**		This function tries to identify a command among the defined commands.
**      It compares the received command with the commands defined in the list of defined commands.
 *      If the command is found, then the command key is returned. Otherwise INVALID
**      If the command is not found, INVALID enumeration value is returned.
**
*/
cmd_key_t DMMCMD_CmdDecode(char *szCmd)
{

	int idxCmd;
	char* szJustCmd = strtok(szCmd, " "); // the following calls to strtok function will continue from this point.
	pszLastErr[0] = 0;  // empty last error string

	if (szJustCmd)
	{
		// compare string with values from array of defined commands
		for(idxCmd = 0; idxCmd < sizeof(uartCommands)/sizeof(uartCommands[0]); idxCmd++)
        {
			if(!strcmp(szJustCmd, uartCommands[idxCmd].pchCmd))
            {
				return uartCommands[idxCmd].eCmd;
			}
		}
		strcpy(pszLastErr,"Unrecognized command:");
		strcat(pszLastErr, szJustCmd);
	}
	else
	{
		strcpy(pszLastErr,"Empty command");
	}

	return INVALID;
}

/*****************************************************************************/
/***	DMMCMD_CmdGetNextArg
**
**	Parameters:
**		<none>
**
**	Return Value:
**          char *  string containing next command argument
**
**	Description:
**		This function is used to tokenize a command in order to retrieve the next argument, using strtok
**      function, considering arguments as tokens separated by blank character.
**      Calling the function repeatedly will cycle through the arguments, returning NULL when
**      the end is reached.
**      It must be called immediately after CmdDecode, once for every argument expected. No calls to
**      strtok is permitted in the mean time.
**
*/
char* DMMCMD_CmdGetNextArg()
{
	return strtok(NULL, ",");
}


/***	DMMCMD_CMD_ProcessCmd
**
**	Parameters:
**     cmd_key_t keyCmd           - the enumerator key corresponding to the command
**
**	Return Value:
**		none
**
**	Description:
**		This function calls the processing function corresponding to the provided enumerator key.
**      It properly provides the command arguments.
**
**
*/
void DMMCMD_ProcessCmd(cmd_key_t keyCmd)
{
    switch(keyCmd)
    {
        case CMD_Config:
        	DMMCMD_CmdConfig(DMMCMD_CmdGetNextArg());
            break;
        case CMD_CalibP:
        	DMMCMD_CmdCalibP(DMMCMD_CmdGetNextArg());
            break;
        case CMD_CalibN:
        	DMMCMD_CmdCalibN(DMMCMD_CmdGetNextArg());
            break;
        case CMD_CalibZ:
        	DMMCMD_CmdCalibZ();
            break;
        case CMD_MeasureRep:
        	DMMCMD_CmdMeasureRep();
            break;
        case CMD_MeasureRaw:
        	DMMCMD_CmdMeasureRaw();
            break;
        case CMD_MeasureStop:
        	DMMCMD_CmdMeasureStop();
            break;
        case CMD_MeasureAvg:
        	DMMCMD_CmdMeasureAvg();
            break;
        case CMD_SaveEPROM:
        	DMMCMD_CmdSaveEPROM();
            break;
        case CMD_VerifyEPROM:
        	DMMCMD_CmdVerifyEPROM();
            break;
        case CMD_ExportCalib:
        	DMMCMD_CmdExportCalib();
            break;
        case CMD_ImportCalib:
        	DMMCMD_CmdImportCalib(DMMCMD_CmdGetNextArg(), DMMCMD_CmdGetNextArg(), DMMCMD_CmdGetNextArg());
            break;
        case CMD_MeasureForCalibP:
        	DMMCMD_CmdMeasureForCalibP();
            break;
        case CMD_MeasureForCalibN:
        	DMMCMD_CmdMeasureForCalibN();
            break;
        case CMD_FinalizeCalibP:
        	DMMCMD_CmdFinalizeCalibP(DMMCMD_CmdGetNextArg());
            break;
        case CMD_FinalizeCalibN:
        	DMMCMD_CmdFinalizeCalibN(DMMCMD_CmdGetNextArg());
            break;
        case CMD_RestoreFactCalibs:
        	DMMCMD_CmdRestoreFactCalib();
            break;
        case CMD_ReadSerialNo:
        	DMMCMD_CmdReadSerialNo();
            break;
//        case CMD_NONE:
        default:
        	// do nothing
            break;
    }
    DelayAprox10Us(1000);
    return;
}



/***	DMMCMD_CmdConfig
**
**	Parameters:
**     char const *arg0           - the character string containing the first command argument, to be interpreted as scale index (integer)
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS            0      // success
**          ERRVAL_DMM_IDXCONFIG     0xFC    // error, wrong scale index
**          ERRVAL_DMM_CFGVERIFY     0xF5    // DMM Configuration verify error
**
**	Description:
**		This function implements the DMMConfig text command of DMMCMD module.
**      It searches the argument among the defined scales in order to detect the scale index,
**      then it calls DMM_SetScale providing the scale index as parameter.
**      The function sends over UART the success message or the error message.
**      The function returns the error code, which is the error code returned by the DMM_SetScale function.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdConfig(char const *arg0)
{
	u8 bErrCode = ERRVAL_SUCCESS;
	int idxScale;
    for(idxScale = 0; idxScale < sizeof(rgScales)/sizeof(rgScales[0]); idxScale++)
    {
        if(!strcmp(arg0, rgScales[idxScale]))
        {
            bErrCode = DMM_SetScale(idxScale);// send the selected configuration to the DMM
            if(bErrCode == ERRVAL_SUCCESS)
            {
                sprintf(szMsg, "PASS, Selected scale index is: %d\r\n", idxScale);
                DMMCMD_PmodOLEDDisplay("No value");
            }
            else
            {
                bErrCode = ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
            }
            UART_PutString(szMsg);
            return bErrCode;
        }
    }
    sprintf(szMsg, "FAIL, Missing valid configuration: \"%s\"\r\n", arg0);
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdMeasureRep
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS            0      // success
**
**	Description:
**		This function initiates the DMMMeasureRep repeated command session of DMMCMD module.
**      The function always returns success: ERRVAL_SUCCESS.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
uint8_t DMMCMD_CmdMeasureRep()
{
	fRepGetVal = 1;
	fRepGetRaw = 0;
    strcpy(szMsg, "Measure repeated");
    ERRORS_GetPrefixedMessageString(ERRVAL_SUCCESS, "", szMsg);
    UART_PutString(szMsg);
    return ERRVAL_SUCCESS;
}


/***	DMMCMD_CmdMeasureStop
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS            0      // success
**
**	Description:
**		This function terminates the DMMMeasureRep and DMMMeasureRaw repeated command sessions of DMMCMD module.
**      The function always returns success: ERRVAL_SUCCESS.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
uint8_t DMMCMD_CmdMeasureStop()
{
	fRepGetVal = 0;
	fRepGetRaw = 0;
    strcpy(szMsg, "Stop repeated");
    ERRORS_GetPrefixedMessageString(ERRVAL_SUCCESS, "", szMsg);
    UART_PutString(szMsg);
    return ERRVAL_SUCCESS;
}

/***	DMMCMD_CmdMeasureRaw
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS            0      // success
**
**	Description:
**		This function implements the DMMMeasureRaw text command of DMMCMD module.
**		The function calls the DMM_DGetValue without calibration parameters being applied.
**		In case of success, the returned value is formatted and sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdMeasureRaw()
{
	u8 bErrCode = ERRVAL_SUCCESS;
	char szVal[20];
	double dMeasuredVal;
	DMM_SetUseCalib(0);
    dMeasuredVal = DMM_DGetValue(&bErrCode);
	DMM_SetUseCalib(1);
    if(bErrCode == ERRVAL_SUCCESS)
    {
		DMM_FormatValue(dMeasuredVal, szVal, 1);
        sprintf(szMsg, "Raw Value: %s\r\n", szVal);
    }
    else
    {
        // like this, prefixing is skipped for ERRVAL_SUCCESS
        ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    }
    UART_PutString(szMsg);
    return bErrCode;
}


/***	DMMCMD_CmdMeasureAvg
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS              0       // success
**          ERRVAL_DMM_VALIDDATATIMEOUT 0xFA    // valid data DMM timeout
**          ERRVAL_DMM_IDXCONFIG        0xFC    // error, wrong current scale index
**
**	Description:
**		This function implements the DMMMeasureAVG text command of DMMCMD module.
**		The function calls the DMM_DGetAvgValue.
**		In case of success, the returned value is formatted and sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The function returns the error code, which is the error code raised by the DMM_DGetAvgValue function.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdMeasureAvg()
{
	u8 bErrCode = ERRVAL_SUCCESS;
	char szVal[20];
	double dMeasuredVal;
    dMeasuredVal = DMM_DGetAvgValue(MEASURE_CNT_AVG, &bErrCode);
    if(bErrCode == ERRVAL_SUCCESS)
    {
        DMM_FormatValue(dMeasuredVal, szVal, 1);
        sprintf(szMsg, "Avg. Value: %s\r\n", szVal);
    }
    else
    {
        // like this, prefixing is skipped for ERRVAL_SUCCESS
        ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    }
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdCalibP
**
**	Parameters:
**     char const *arg0           - the character string containing the first command argument, to be interpreted as reference value
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**          ERRVAL_CMD_VALWRONGUNIT         0xF4    // The provided value has a wrong measure unit.
**          ERRVAL_CMD_VALFORMAT            0xF2    // The numeric value cannot be extracted from the provided string.
**          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
**          ERRVAL_DMM_MEASUREDISPERSION    0xF1    // The calibration measurement dispersion exceeds accepted range
**          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration function.
**
**	Description:
**		This function implements the DMMCalibP text command of DMMCMD module.
**      It interprets the argument as reference value by calling DMM_InterpretValue function.
**      then it calls CALIB_CalibOnPositive providing the reference value as parameter and collecting the measured value and dispersion.
**		In case of success, the function builds the message using the formatted strings for reference value, measured value and dispersion and eventually
**      the calibration coefficients. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The return values are possible errors of DMM_InterpretValue and DMMCalibP functions.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdCalibP(char const *arg0)
{
	u8 bErrCode = ERRVAL_SUCCESS;
    bErrCode = DMM_InterpretValue((char *)arg0, &dRefVal);
    if(bErrCode == ERRVAL_SUCCESS)
    {
		bErrCode = CALIB_CalibOnPositive(dRefVal, &dMeasuredVal, 0, &dispersion, 0);
        if(bErrCode == ERRVAL_SUCCESS)
        {
            DMM_FormatValue(dRefVal, szRefVal, 1);
            DMM_FormatValue(dMeasuredVal, szVal, 1);
    		sprintf(szMsg, "Calibration on positive done. Reference: %s, Measured: %s, Dispersion: %.2f%%", szRefVal, szVal, dispersion);
    		if(pszLastErr[0])
    		{
    			// append last error string to the message (used for calibration coefficients)
    			strcat(szMsg, ", ");
    			strcat(szMsg, pszLastErr);
    		}
        }
        ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);

    }
    else
    {
    	ERRORS_GetPrefixedMessageString(bErrCode, (char *)arg0, szMsg);
    }
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdCalibN
**
**	Parameters:
**     char const *arg0           - the character string containing the first command argument, to be interpreted as reference value
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**          ERRVAL_CMD_VALWRONGUNIT         0xF4    // The provided value has a wrong measure unit.
**          ERRVAL_CMD_VALFORMAT            0xF2    // The numeric value cannot be extracted from the provided string.
**          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
**          ERRVAL_DMM_MEASUREDISPERSION    0xF1    // The calibration measurement dispersion exceeds accepted range
**          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration function.
**
**	Description:
**		This function implements the DMMCalibN text command of DMMCMD module.
**      It interprets the argument as reference value by calling DMM_InterpretValue function.
**      then it calls CALIB_CalibOnNegative providing the reference value as parameter and collecting the measured value and dispersion.
**		In case of success, the function builds the message using the formatted strings for reference value, measured value and dispersion and eventually
**      the calibration coefficients. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The return values are possible errors of DMM_InterpretValue and DMMCalibN functions.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdCalibN(char const *arg0)
{
	u8 bErrCode = ERRVAL_SUCCESS;
    bErrCode = DMM_InterpretValue((char *)arg0, &dRefVal);
    if(bErrCode == ERRVAL_SUCCESS)
    {
		bErrCode = CALIB_CalibOnNegative(dRefVal, &dMeasuredVal, 0, &dispersion, 0);
        if(bErrCode == ERRVAL_SUCCESS)
        {
            DMM_FormatValue(dRefVal, szRefVal, 1);
            DMM_FormatValue(dMeasuredVal, szVal, 1);
    		sprintf(szMsg, "Calibration on negative done. Reference: %s, Measured: %s, Dispersion: %.2f%%", szRefVal, szVal, dispersion);
    		if(pszLastErr[0])
    		{
    			// append last error string to the message (used for calibration coefficients)
    			strcat(szMsg, ", ");
    			strcat(szMsg, pszLastErr);
    		}
        }
        ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);

    }
    else
    {
    	ERRORS_GetPrefixedMessageString(bErrCode, (char *)arg0, szMsg);
    }
    UART_PutString(szMsg);
    return bErrCode;
}


/***	DMMCMD_CmdCalibZ
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**          ERRVAL_CMD_VALWRONGUNIT         0xF4    // The provided value has a wrong measure unit.
**          ERRVAL_CMD_VALFORMAT            0xF2    // The numeric value cannot be extracted from the provided string.
**          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
**          ERRVAL_DMM_MEASUREDISPERSION    0xF1    // The calibration measurement dispersion exceeds accepted range
**          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration function.**
**	Description:
**		This function implements the DMMCalibZ text command of DMMCMD module.
**      It calls CALIB_CalibOnZero collecting the measured value and dispersion.
**		In case of success, the function builds the message using the formatted string for measured value, dispersion and eventually
**      the calibration coefficients. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The return values are possible errors of DMMCalibZ functions.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdCalibZ()
{
	u8 bErrCode = ERRVAL_SUCCESS;
	bErrCode = CALIB_CalibOnZero(&dMeasuredVal, &dispersion, 0);
    if(bErrCode == ERRVAL_SUCCESS)
    {
         DMM_FormatValue(dMeasuredVal, szVal, 1);
         sprintf(szMsg, "Calibration on zero done. Measured Value: %s, Dispersion: %.2f%%", szVal, dispersion);
         if(pszLastErr[0])
         {
             // append last error string to the message (used for calibration coefficients)
             strcat(szMsg, ", ");
             strcat(szMsg, pszLastErr);
         }
	}
	ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);

    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdSaveEPROM
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_EPROM_WRTIMEOUT          0xFF    // EPROM write data ready timeout
**
**	Description:
**		This function implements the DMMSaveEPROM text command of DMMCMD module.
**      It calls CALIB_WriteAllCalibsToEPROM_User collecting the number of modified scales or error code.
**		In case of success, the function builds the message using the the number of modified scales. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The function returns the error code, which is the error code returned by the CALIB_WriteAllCalibsToEPROM_User function.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdSaveEPROM()
{
	u8 bErrCode = ERRVAL_SUCCESS;
    bErrCode = CALIB_WriteAllCalibsToEPROM_User();
    if (bErrCode != ERRVAL_EPROM_WRTIMEOUT)
    {
        sprintf(szMsg, "%d calibrations written to EPROM", bErrCode);
        bErrCode = ERRVAL_SUCCESS;
    }
    ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);

    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdVerifyEPROM
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_GENERICERROR         0xEF    // Generic error
**          ERRVAL_EPROM_MAGICNO            0xFD    // wrong Magic No. when reading data from EPROM
**          ERRVAL_EPROM_CRC                0xFE    // wrong CRC when reading data from EPROM
**
**	Description:
**		This function implements the DMMVerifyEPROM text command of DMMCMD module.
**      It calls CALIB_VerifyEPROM collecting the error code.
**		In case of success and verify error, the function builds separate messages and then the messages are sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The function returns the error code, which is the error code returned by the CALIB_VerifyEPROM function, with the exception
**      that ERRVAL_DMM_GENERICERROR is returned in the case of verify error.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdVerifyEPROM()
{
	u8 bErrCode = ERRVAL_SUCCESS;
    bErrCode = CALIB_VerifyEPROM();
    switch(bErrCode)
    {
        case ERRVAL_SUCCESS:
            strcpy(szMsg, "EPROM Calibration data is verified");
            break;
        case ERRVAL_EPROM_VERIFY:
            strcpy(szMsg, "EPROM Calibration data mismatch values found");
            bErrCode = ERRVAL_DMM_GENERICERROR;
            break;
        default:
            break;
    }
    ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);

    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdExportCalib
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_EPROM_MAGICNO            0xFD    // wrong Magic No. when reading data from EPROM
**          ERRVAL_EPROM_CRC                0xFE    // wrong CRC when reading data from EPROM
**
**	Description:
**		This function implements the DMMExportCalib text command of DMMCMD module.
**      It calls CALIB_ExportCalibs_User collecting the exported string.
**		In case of success, the function sends the success message and the exported text over UART.
**		In case of error, the error specific message is sent over UART.
**      The function returns the error code, which is the error code returned by the CALIB_ExportCalibs_User function.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdExportCalib()
{
	u8 bErrCode = ERRVAL_SUCCESS;
    char calExp[1024];
    bErrCode = CALIB_ExportCalibs_User(calExp);
    strcpy(szMsg, "Calibration data is exported");
    ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    UART_PutString(szMsg);
    UART_PutString(calExp);
    return bErrCode;
}

/***	DMMCMD_CmdImportCalib
**
**	Parameters:
**     char const *arg0           - the character string containing the first command argument, to be interpreted as scale index (integer)
**     char const *arg1           - the character string containing the second command argument, to be interpreted as Mult. coefficient (float)
**     char const *arg2           - the character string containing the third command argument, to be interpreted as Add. coefficient (float)
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_CMD_WRONGPARAMS          0xF9    // wrong parameters when sending UART commands
**          ERRVAL_DMM_GENERICERROR         0xEF    // Generic error, parameters cannot be properly interpreted
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**
**	Description:
**		This function implements the DMMImportCalib text command of DMMCMD module.
**      It interprets the first parameter as scale index, the second as Mult. coefficient, and the third as Add. coefficient.
**      In case these parameters do not fit, specific error messages are sent over UART and ERRVAL_CMD_WRONGPARAMS or ERRVAL_DMM_GENERICERROR errors are returned.
**      It calls CALIB_ImportCalibCoefficients providing the scale index, Mult. coefficient and Add. coefficient.
**		In case of success, the function sends the success message over UART.
**		In case of error, the error specific message is sent over UART.
**      The function returns the error code, which is the error code detected when parameters are interpreted or returned by the CALIB_ImportCalibCoefficients function.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdImportCalib(char const *arg0, char const *arg1, char const *arg2)
{
	u8 bErrCode = ERRVAL_SUCCESS;
	int idxCfg;
    float fValM, fValA;
    if(!arg0 || !arg1 || !arg2)
    {
    	bErrCode = ERRVAL_CMD_WRONGPARAMS;
    }
    if(bErrCode == ERRVAL_SUCCESS)
    {
		// idxScale
		if (!sscanf(arg0, "%d", &idxCfg))
		{
			strcpy(szMsg, "Invalid value, provide an integer number for the first token, corresponding to scale index");
			bErrCode = ERRVAL_DMM_GENERICERROR;
		}
		else
		{	// Mult coefficient
			if (!sscanf(arg1, "%f", &fValM))
			{
				strcpy(szMsg, "Invalid value, provide a float number for the second token, corresponding to Mult. coefficient");
				bErrCode = ERRVAL_DMM_GENERICERROR;
			}
			else
			{	// Add coefficient
				if (!sscanf(arg2, "%f", &fValA))
				{
					strcpy(szMsg, "Invalid value, provide a float number for the third token, corresponding to Add. coefficient");
					bErrCode = ERRVAL_DMM_GENERICERROR;
				}
			}
		}
    }
    if(bErrCode == ERRVAL_SUCCESS)
    {
        bErrCode = CALIB_ImportCalibCoefficients(idxCfg, fValM, fValA);
    }
    ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdMeasureForCalibP
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
**
**	Description:
**		This function implements the DMMMeasureForCalibP text command of DMMCMD module.
**      It calls CALIB_MeasureForCalibPositiveVal collecting the measured value.
**		In case of success, the function builds the message using the formatted string for measured value. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The return values are possible errors of CALIB_MeasureForCalibPositiveVal functions.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdMeasureForCalibP()
{
	u8 bErrCode = ERRVAL_SUCCESS;
	double dMeasuredVal;
	bErrCode = CALIB_MeasureForCalibPositiveVal(&dMeasuredVal);

	if(bErrCode == ERRVAL_SUCCESS)
	{
		DMM_FormatValue(dMeasuredVal, szVal, 1);
		sprintf(szMsg, "Calibration positive measurement done. Measured Value: %s", szVal);
	}
	ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdMeasureForCalibN
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**          ERRVAL_DMM_VALIDDATATIMEOUT     0xFA    // valid data DMM timeout
**
**	Description:
**		This function implements the DMMMeasureForCalibN text command of DMMCMD module.
**      It calls CALIB_MeasureForCalibNegativeVal collecting the measured value.
**		In case of success, the function builds the message using the formatted string for measured value. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The return values are possible errors of CALIB_MeasureForCalibNegativeVal functions.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdMeasureForCalibN()
{
	u8 bErrCode = ERRVAL_SUCCESS;
	double dMeasuredVal;

	bErrCode = CALIB_MeasureForCalibNegativeVal(&dMeasuredVal);

	if(bErrCode == ERRVAL_SUCCESS)
	{
		DMM_FormatValue(dMeasuredVal, szVal, 1);
		sprintf(szMsg, "Calibration negative measurement done. Measured Value: %s", szVal);
	}
	ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdFinalizeCalibP
**
**	Parameters:
**     char const *arg0           - the character string containing the first command argument, to be interpreted as reference value
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**          ERRVAL_CMD_VALWRONGUNIT         0xF4    // The provided value has a wrong measure unit.
**          ERRVAL_CMD_VALFORMAT            0xF2    // The numeric value cannot be extracted from the provided string.
**          ERRVAL_DMM_MEASUREDISPERSION    0xF1    // The calibration measurement dispersion exceeds accepted range
**          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration function.
**
**	Description:
**		This function implements the DMMFinalizeCalibP text command of DMMCMD module.
**      It interprets the argument as reference value by calling DMM_InterpretValue function.
**      then it calls CALIB_CalibOnPositive providing the reference value as parameter and collecting the measured value and dispersion.
**		In case of success, the function builds the message using the formatted strings for reference value, measured value and dispersion and eventually
**      the calibration coefficients. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The return values are possible errors of DMM_InterpretValue and DMMCalibP functions.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdFinalizeCalibP(char const *arg0)
{
	u8 bErrCode = ERRVAL_SUCCESS;
	bErrCode = DMM_InterpretValue((char *)arg0, &dRefVal);
	if(bErrCode == ERRVAL_SUCCESS)
	{
		DMM_FormatValue(dRefVal, szRefVal, 1);
		bErrCode = CALIB_CalibOnPositive(dRefVal, &dMeasuredVal, 1, &dispersion, 0);
		if(bErrCode == ERRVAL_SUCCESS)
		{
			DMM_FormatValue(dMeasuredVal, szVal, 1);
			sprintf(szMsg, "Calibration on positive done. Reference: %s, Measured: %s, Dispersion: %.2f%%", szRefVal, szVal, dispersion);
			if(pszLastErr[0])
			{
				// append last error string to the message (used for calibration coefficients)
				strcat(szMsg, ", ");
				strcat(szMsg, pszLastErr);
			}
		}
		ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
	}
    else
    {
    	ERRORS_GetPrefixedMessageString(bErrCode, (char *)arg0, szMsg);
    }
	UART_PutString(szMsg);
	return bErrCode;
}

/***	DMMCMD_CmdFinalizeCalibN
**
**	Parameters:
**     char const *arg0           - the character string containing the first command argument, to be interpreted as reference value
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_DMM_IDXCONFIG            0xFC    // wrong scale index
**          ERRVAL_CMD_VALWRONGUNIT         0xF4    // The provided value has a wrong measure unit.
**          ERRVAL_CMD_VALFORMAT            0xF2    // The numeric value cannot be extracted from the provided string.
**          ERRVAL_DMM_MEASUREDISPERSION    0xF1    // The calibration measurement dispersion exceeds accepted range
**          ERRVAL_CALIB_MISSINGMEASUREMENT 0xF0    // A measurement must be performed before calling the finalize calibration function.
**
**	Description:
**		This function implements the DMMCalibN text command of DMMCMD module.
**      It interprets the argument as reference value by calling DMM_InterpretValue function.
**      then it calls CALIB_CalibOnNegative providing the reference value as parameter and collecting the measured value and dispersion.
**		In case of success, the function builds the message using the formatted strings for reference value, measured value and dispersion and eventually
**      the calibration coefficients. Then the message is sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The return values are possible errors of DMM_InterpretValue and DMMCalibN functions.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdFinalizeCalibN(char const *arg0)
{
	u8 bErrCode = ERRVAL_SUCCESS;
	bErrCode = DMM_InterpretValue((char *)arg0, &dRefVal);
	if(bErrCode == ERRVAL_SUCCESS)
	{
		DMM_FormatValue(dRefVal, szRefVal, 1);
        bErrCode = CALIB_CalibOnNegative(dRefVal, &dMeasuredVal, 1, &dispersion, 0);
        if(bErrCode == ERRVAL_SUCCESS)
        {
            DMM_FormatValue(dMeasuredVal, szVal, 1);
            sprintf(szMsg, "Calibration on negative done. Reference: %s, Measured: %s, Dispersion: %.2f%%", szRefVal, szVal, dispersion);
            if(pszLastErr[0])
            {
                // append last error string to the message (used for calibration coefficients)
                strcat(szMsg, ", ");
                strcat(szMsg, pszLastErr);
            }
        }
        ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
	}
    else
    {
    	ERRORS_GetPrefixedMessageString(bErrCode, (char *)arg0, szMsg);
    }
	UART_PutString(szMsg);
	return bErrCode;
}

/***	DMMCMD_CmdRestoreFactCalib
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS                  0       // success
**          ERRVAL_EPROM_MAGICNO            0xFD    // wrong Magic No. when reading data from EPROM
**          ERRVAL_EPROM_CRC                0xFE    // wrong CRC when reading data from EPROM
**          ERRVAL_EPROM_WRTIMEOUT          0xFF    // EPROM write data ready timeout
**
**	Description:
**		This function implements the DMMDRestoreFactCalib text command of DMMCMD module.
**      It calls CALIB_RestoreAllCalibsFromEPROM_Factory.
**		In case of success, the function sends the success message and the exported text over UART.
**		In case of error, the error specific message is sent over UART.
**      The function returns the error code, which is the error code returned by the CALIB_RestoreAllCalibsFromEPROM_Factory function.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdRestoreFactCalib()
{
	u8 bErrCode = ERRVAL_SUCCESS;
    bErrCode = CALIB_RestoreAllCalibsFromEPROM_Factory();
    strcpy(szMsg, "Calibration data restored from FACTORY EPROM");
    ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_CmdReadSerialNo
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS              0       // success
**          ERRVAL_EPROM_CRC            0xFE    // wrong CRC when reading data from EPROM
**          ERRVAL_EPROM_MAGICNO        0xFD    // wrong Magic No. when reading data from EPROM
**
**	Description:
**		This function implements the DMMReadSerialNo text command of DMMCMD module.
**      It calls SERIALNO_ReadSerialNoFromEPROM and collects the serial number string.
**		In case of success, the function sends the success message containing the serial number over UART.
**		In case of error, the error specific message is sent over UART.
**      The function returns the error code, which is the error code returned by the SERIALNO_ReadSerialNoFromEPROM function.
**      The function is called by DMMCMD_ProcessCmd function.
**
*/
u8 DMMCMD_CmdReadSerialNo()
{
	u8 bErrCode = ERRVAL_SUCCESS;
    char szSerialNo[SERIALNO_SIZE + 1];
    bErrCode = SERIALNO_ReadSerialNoFromEPROM(szSerialNo);
    if (bErrCode == ERRVAL_SUCCESS)
    {
        sprintf(szMsg, "SerialNo = \"%s\"", szSerialNo);
    }
    ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
    UART_PutString(szMsg);
    return bErrCode;
}

/***	DMMCMD_ProcessRepeatedCmd
**
**	Parameters:
**     none
**
**	Return Value:
**		uint8_t     - the error code
**          ERRVAL_SUCCESS              0       // success
**          ERRVAL_DMM_VALIDDATATIMEOUT 0xFA    // valid data DMM timeout
**          ERRVAL_DMM_IDXCONFIG        0xFC    // error, wrong current scale index
**
**	Description:
**		This function implements the repeated session functionality for DMMMeasureRep and DMMMeasureRaw text commands of DMMCMD module.
**		The function calls the DMM_DGetValue, eventually without calibration parameters being applied for DMMMeasureRaw.
**		In case of success, the returned value is formatted and sent over UART.
**		In case of error, the error specific message is sent over UART.
**      The function is called by DMMCMD_ProcessCmd function.
*/
uint8_t DMMCMD_ProcessRepeatedCmd()
{
	uint8_t bErrCode = ERRVAL_SUCCESS;
    if((fRepGetVal || fRepGetRaw) && !fRepBlock)
    {
        if(fRepGetRaw)
        {
        	DMM_SetUseCalib(0);
        }
        dMeasuredVal = DMM_DGetValue(&bErrCode);
        DMM_SetUseCalib(1);
        if(bErrCode == ERRVAL_SUCCESS)
        {
            if(fRepGetVal)
            {
                DMM_FormatValue(dMeasuredVal, szVal, 1);
                sprintf(szMsg, "Value: %s\r\n", szVal);
            	DMMCMD_PmodOLEDDisplay(szVal);
            }
            else
            {
                sprintf(szMsg, "Raw Value: %.6lf\r\n", dMeasuredVal);
            }
        }
        else
        {
            bErrCode = ERRORS_GetPrefixedMessageString(bErrCode, "", szMsg);
        }
        UART_PutString(szMsg);
    }
    return bErrCode;
}

/***	DMMCMD_PmodOLEDDisplay
**
**	Parameters:
**     char const *pszVal           - the character string containing the value information
**
**	Return Value:
**		<none>
**
**	Description:
**		This function implements the regular display on PmpdOLED.
**		It detects the current selected scale and displays it on the second row.
**		It displays the value information on the forth row.
**
**
*/
void DMMCMD_PmodOLEDDisplay(char *pszVal)
{
	int idxScale = DMM_GetCurrentScale();
    OLED_ClearBuffer(&myPmodOLEDDevice);

    if(idxScale != -1)
    {
    	OLED_SetCursor(&myPmodOLEDDevice, (16 - strlen(rgScales[idxScale]))/2, 1);
    	OLED_PutString(&myPmodOLEDDevice, (char *)rgScales[idxScale]);
    }
    else
    {
    	OLED_SetCursor(&myPmodOLEDDevice, 4, 1);
    	OLED_PutString(&myPmodOLEDDevice, "No scale");

    }

	OLED_SetCursor(&myPmodOLEDDevice, (16 - strlen(pszVal))/2, 3);
    OLED_PutString(&myPmodOLEDDevice, pszVal);
    OLED_Update(&myPmodOLEDDevice);}
