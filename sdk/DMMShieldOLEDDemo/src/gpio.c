/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    gpio.c

  @Description
        This file groups the functions that implement GPIO module.


  @Author
    Cristian Fatu
    cristian.fatu@digilent.ro

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library

 */

#include <stdio.h>
#include "platform.h"
#include "xparameters.h"

#include "gpio.h"

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#define GPIO_DMMSHIELD_DEVICE_ID  XPAR_GPIO_0_DEVICE_ID

u32 dwStoreOutputGroupVal;
XGpio Gpio; /* The Instance of the GPIO Driver */

int GPIO_Init()
{
	int Status;
	/* Initialize the GPIO driver */
	Status = XGpio_Initialize(&Gpio, GPIO_DMMSHIELD_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Set the direction for the 7 output signals: CS_EPROM, CS_DMM, MOSI, CLK, RLI, RLU, RLD*/
//	unsigned int uiMask = 0xFFFFFFFF & ~(
//			GPIO_Mask_CS_EPROM 	|
//			GPIO_Mask_CS_DMM 	|
//			GPIO_Mask_CLK 		|
//			GPIO_Mask_MOSI 		|
//			GPIO_Mask_RLD 		|
//			GPIO_Mask_RLU 		|
//			GPIO_Mask_RLI 	);
//	XGpio_SetDataDirection(&Gpio, GPIO_OUTPUT_CHANNEL, uiMask);	// last 7 bits output
	XGpio_SetDataDirection(&Gpio, 1, 0xFFFFFF80);	// last 7 bits output
	/* Set the direction for the input signal MISO */
	XGpio_SetDataDirection(&Gpio, GPIO_INPUT_CHANNEL, GPIO_Mask_MISO);	// last 1 bit input

	dwStoreOutputGroupVal = XGpio_DiscreteRead(&Gpio, GPIO_OUTPUT_CHANNEL);

    // // Deactivate CS_DMM
    GPIO_SetValue_CS_DMM(1);

    // Deactivate EPROM SS
    GPIO_SetValue_CS_EPROM(0);

	return Status;
}

void GPIO_SetOutputValue(u32 dwMask, u8 bVal)
{
	// update group value
	if(bVal)
	{
		dwStoreOutputGroupVal |= dwMask;
	}
	else
	{
		dwStoreOutputGroupVal &= ~dwMask;
	}

	// write group value
	XGpio_DiscreteWrite(&Gpio, GPIO_OUTPUT_CHANNEL, dwStoreOutputGroupVal);
}

