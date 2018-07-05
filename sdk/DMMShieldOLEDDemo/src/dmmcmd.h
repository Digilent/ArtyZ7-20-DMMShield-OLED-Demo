/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    dmmcmd.h

  @Description
        This file contains the declaration for the functions of the DMMCMD module.

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library

 */
#ifndef CMD_H_
#define CMD_H_

/***************************** Include Files *********************************/
#include "string.h"
#include "stdlib.h"
#include "xstatus.h"

/************************** Type definitions ******************************/
typedef enum {
	CMD_NONE = -1, // No command
	INVALID = 0, // Invalid command
/* Add/remove test constants below this line */
	CMD_Config,
	CMD_CalibP,
	CMD_CalibN,
	CMD_CalibZ,
	CMD_MeasureRep,
	CMD_MeasureStop,
	CMD_MeasureRaw,
	CMD_MeasureAvg,
	CMD_SaveEPROM,
	CMD_VerifyEPROM,
	CMD_ExportCalib,
	CMD_ImportCalib,
	CMD_MeasureForCalibP,
	CMD_MeasureForCalibN,
	CMD_FinalizeCalibP,
	CMD_FinalizeCalibN,
	CMD_RestoreFactCalibs,
	CMD_ReadSerialNo

} cmd_key_t;

// structure mapping command key to command string
typedef struct {
	char *pchCmd;
	cmd_key_t eCmd;
} cmd_map_t;
/************************** Definitions ******************************/

/************************** Function Prototypes ******************************/
u8 DMMCMD_Init();
void DMMCMD_CheckForCommand();




#endif /* CMD_H_ */
