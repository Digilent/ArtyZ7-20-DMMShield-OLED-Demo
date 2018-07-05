/* ************************************************************************** */
/** Descriptive File Name

  @Company
 Digilent

  @File Name
    gpio.h

  @Description
        This file contains the definitions needed for GPIO module.

  @Author
    Cristian Fatu
    cristian.fatu@digilent.ro

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library

 */



#ifndef __GPIO_H_
#define __GPIO_H_

/***************************** Include Files *********************************/

#include "xparameters.h"
#include "xgpio.h"


/************************** Constant Definitions *****************************/
// one channel for output pins, one channel for input pin
#define GPIO_OUTPUT_CHANNEL	1	// corresponding to GPIO  channel
#define GPIO_INPUT_CHANNEL	2	// corresponding to GPIO2 channel

// output pins
#define GPIO_Mask_CS_EPROM 	1
#define GPIO_Mask_CS_DMM 	2
#define GPIO_Mask_CLK 		4
#define GPIO_Mask_MOSI 		8

#define GPIO_Mask_RLD 		16
#define GPIO_Mask_RLU 		32
#define GPIO_Mask_RLI 		64

// input pins
#define GPIO_Mask_MISO 		1

extern XGpio Gpio; /* The Instance of the GPIO Driver, declared in GPIO.c */

/***************** Function prototypes *********************/
int GPIO_Init();
void GPIO_SetOutputValue(u32 dwMask, u8 bVal);
/***************** Macros (Inline Functions) Definitions *********************/
#define GPIO_SetValue_CS_EPROM(val) \
		GPIO_SetOutputValue(GPIO_Mask_CS_EPROM, val)

#define GPIO_SetValue_CS_DMM(val) \
		GPIO_SetOutputValue(GPIO_Mask_CS_DMM, val)

#define GPIO_SetValue_CLK(val) \
		GPIO_SetOutputValue(GPIO_Mask_CLK, val)

#define GPIO_SetValue_MOSI(val) \
		GPIO_SetOutputValue(GPIO_Mask_MOSI, val)

#define GPIO_SetValue_RLD(val) \
		GPIO_SetOutputValue(GPIO_Mask_RLD, val)

#define GPIO_SetValue_RLU(val) \
		GPIO_SetOutputValue(GPIO_Mask_RLU, val)

#define GPIO_SetValue_RLI(val) \
		GPIO_SetOutputValue(GPIO_Mask_RLI, val)


#define GPIO_Get_MISO() \
        (XGpio_DiscreteRead(&Gpio, GPIO_INPUT_CHANNEL) & GPIO_Mask_MISO)


#endif
