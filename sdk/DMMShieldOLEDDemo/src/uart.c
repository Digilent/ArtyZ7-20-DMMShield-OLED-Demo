/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Digilent

  @File Name
    uart.c

  @Description
        This file groups the functions that implement the UART_PS functionality.
        This module implements the UART functionality connected to the USB - UART
        interface of the Zybo Z7-20 board. It provides basic functions to configure UARTPS_0 and
        transmit / receive with interrupt functions.
        The "Interface functions" section groups functions that can also be called by User.
        The "Local functions" section groups low level functions that are only called from within current module.
		The code is adapted using xuartps_intr_example.c Xilinx example (2016.4).


  @Author
    Cristian Fatu
    cristian.fatu@digilent.ro

  @Versioning:
 	 Cristian Fatu - 2018/06/29 - Initial release, DMMShield Library

 */

/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
#include "uart.h"
#include "errors.h"
#include <stdarg.h>
#include "xparameters.h"
#include "xplatform_info.h"
#include "xuartps.h"
#include "xscugic.h"

/************************** Constant Definitions *****************************/


#define INTC		XScuGic
#define UART_DEVICE_ID		XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_INT_IRQ_ID		XPAR_XUARTPS_0_INTR


// global variables, to communicate between interrupt handler and other
#define RCV_BUFFER_SIZE	100

/* ************************************************************************** */
/* Section: Global Variables                                                  */
/* ************************************************************************** */
XUartPs UartPs;		/* Instance of the UART Device */
INTC InterruptController;	/* Instance of the Interrupt Controller */


static u8 RecvBuffer[RCV_BUFFER_SIZE];	/* Buffer for Receiving Data */

/*
 * The following counters are used to determine when the entire buffer has
 * been sent and received.
 */
volatile int TotalReceivedCount = 0;
volatile int TotalErrorCount;

/* ************************************************************************** */
// Section: Local Functions Prototypes                                        */
/* ************************************************************************** */

void UART_SendBlock(XUartPs* UartInst, char const* rgbBuffer, u32 cbRequested);
XStatus UART_ConfigureUARTPS(XUartPs *UartInstPtr, INTC *IntcInstPtr, u32 dwBaudRate);
static int UART_SetupInterruptSystem(INTC *IntcInstancePtr,
				XUartPs *UartInstancePtr,
				u16 UartIntrId);

void UART_Handler(void *CallBackRef, u32 Event, unsigned int EventData);
/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/***	UART_Init
**
**	Parameters:
**		u32 dwBaudRate- UART baud rate.
**                                     for example 115200 corresponds to 115200 baud
**
**	Return Value:
**		u8	- Error code
**			ERRVAL_SUCCESS                  0       // success
**			ERRVAL_DMM_UARTERROR         	0xEE    // UART Init error
**
**	Description:
**		This function initializes the UART-PS controller at the specified baud rate,
**		triggering interrupt on UART events.
**
*/
u8 UART_Init(u32 dwBaudRate)
{	XStatus Status;
	Status = UART_ConfigureUARTPS(&UartPs, &InterruptController, dwBaudRate);
	// initialize receive
	TotalReceivedCount = 0;
	XUartPs_Recv(&UartPs, (u8 *)RecvBuffer, RCV_BUFFER_SIZE);

	return Status == XST_SUCCESS ? ERRVAL_SUCCESS: ERRVAL_DMM_UARTERROR;
}



/***	UART_PutString
**
**	Parameters:
**          char szData[] -   the zero terminated string containing characters to be transmitted over UART.
**
**	Return Value:
**
**
**	Description:
**		This function transmits all the characters from a zero terminated string over UART1. The terminator character is not sent.
**
**
*/
void UART_PutString(char szData[])
{
	UART_SendBlock(&UartPs, szData, strlen(szData));
}

/***	UART_GetString
**
**	Parameters:
**		char* pchBuff   - pointer to a char buffer to hold the received zero terminated string
**		int cchBuff     - size of the buffer to hold the zero terminated string
**
**
**	Return Value:
**          uint8_t     -The receive status
**                  > 0 - the number of characters contained in the string
**                  0	- a string hasn't been received
**
**	Description:
**		This function provides a zero terminated string received over UARTPS_0
**      using the information provided by the UART interrupt handler.
**		If a received string is available, the newline ('\n\) and CR ('\r') characters are stripped and zero terminator is appended.
**		The string is copied in the pchBuff string and its length is returned.
**		Otherwise, the function returns 0.
**
*/
int UART_GetString(char* pchBuff, int cchBuff)
{
	int nResult = TotalReceivedCount;
	if(nResult > 0)
	{
		while(RecvBuffer[nResult - 1] == '\r' || RecvBuffer[nResult - 1] == '\n')
		{
			nResult--;
		}
		RecvBuffer[nResult] = 0;

		strcpy(pchBuff, (const char *)RecvBuffer);
		// initialize receive
		TotalReceivedCount = 0;
		XUartPs_Recv(&UartPs, (u8 *)RecvBuffer, RCV_BUFFER_SIZE);
	}
    return nResult;
}

/* ************************************************************************** */
/* ************************************************************************** */
// Section: Local Functions                                                   */
/* ************************************************************************** */
/* ************************************************************************** */

/***	UART_ConfigureUARTPS
**
**	Parameters:
**		XUartPs *UartInstPtr	- pointer to the instance of the UART driver
**		INTC *IntcInstPtr		- pointer to the instance of the Scu Gic driver
**		u32 dwBaudRate- UART baud rate.
**                                     for example 115200 corresponds to 115200 baud
**
**	Return Value:
**		XStatus	- Error code
**			XST_SUCCESS if successful, otherwise XST_FAILURE.
**
**	Description:
**		This function configures the UART-PS interface at the specified baud rate, with interrupt.
**		The code is adapted using xuartps_intr_example.c Xilinx example (2016.4).
**
**
*/
XStatus UART_ConfigureUARTPS(XUartPs *UartInstPtr, INTC *IntcInstPtr, u32 dwBaudRate)
{
	XUartPs_Config *Config;
	XStatus Status;
	u32 IntrMask;
	/*
	 * Initialize the UART driver so that it's ready to use.
	 * Look up the configuration in the config. table then initialize it.
	 */
	Config = XUartPs_LookupConfig(UART_DEVICE_ID);
	if(NULL == Config) {
		return XST_FAILURE;
	}

	Status = XUartPs_CfgInitialize(UartInstPtr, Config, Config->BaseAddress);
		if (Status != XST_SUCCESS) {
			return XST_FAILURE;
		}

	//Reconfigure default baud rate to the requested one
	Status = XUartPs_SetBaudRate(UartInstPtr, dwBaudRate);
	if (Status != XST_SUCCESS) {
		//XST_UART_BAUD_ERROR if baud rate cannot be synthesized with acceptable error
		return Status;
	}
	/* Check hardware build */
	Status = XUartPs_SelfTest(UartInstPtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the UART to the interrupt subsystem such that interrupts
	 * can occur. This function is application specific.
	 */
	Status = UART_SetupInterruptSystem(IntcInstPtr, UartInstPtr, UART_INT_IRQ_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handlers for the UART that will be called from the
	 * interrupt context when data has been sent and received, specify
	 * a pointer to the UART driver instance as the callback reference
	 * so the handlers are able to access the instance data
	 */
	XUartPs_SetHandler(UartInstPtr, (XUartPs_Handler)UART_Handler, UartInstPtr);

	/*
	 * Enable the interrupt of the UART so interrupts will occur, setup
	 * a local loopback so data that is sent will be received.
	 */
	IntrMask =
		XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
		XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
		XUARTPS_IXR_RXOVR;

	if (UartInstPtr->Platform == XPLAT_ZYNQ_ULTRA_MP) {
		IntrMask |= XUARTPS_IXR_RBRK;
	}

	XUartPs_SetInterruptMask(UartInstPtr, IntrMask);


	/*
	 * Set the receiver timeout. If it is not set, and the last few bytes
	 * of data do not trigger the over-water or full interrupt, the bytes
	 * will not be received. By default it is disabled.
	 *
	 * The setting of 8 will timeout after 8 x 4 = 32 character times.
	 * Increase the time out value if baud rate is high, decrease it if
	 * baud rate is low.
	 */
	XUartPs_SetRecvTimeout(UartInstPtr, 8);

	return ERRVAL_SUCCESS;
}


/***	UART_SendBlock
**
**	Parameters:
**		UART_DRIVER* UartInst	- UART_PS driver instance
**		char const* rgbBuffer	- pointer to a char buffer containing characters to be sent
**		u32 cbRequested     	- number of characters to send
**
**
**	Return Value:
**          u32     -	number of received characters
**
**	Description:
**		This function sends data to over UART. Calling the functions will block until
** 	all the data can be sent to the UART controller's FIFO
**
*/
void UART_SendBlock(UART_DRIVER* UartInst, char const* rgbBuffer, u32 cbRequested)
{
	u32 cbSent;
	do {
		cbSent = XUartPs_Send(UartInst, (u8*)rgbBuffer, cbRequested);
		cbRequested -= cbSent;
		rgbBuffer += cbSent;
	} while (cbRequested > 0);
}

/*****************************************************************************/

/***	UART_SetupInterruptSystem
**
**	Parameters:
**		INTC *IntcInstPtr		- pointer to the instance of the Scu Gic driver
**		XUartPs *UartInstPtr	- pointer to the instance of the UART driver
**		u16 UartIntrId     		- the interrupt Id and is typically XPAR_<UARTPS_instance>_INTR value from xparameters.h.
**
**
**	Return Value:
**          u32     -	number of received characters
**
**	Description:
** 			This function sets up the interrupt system so interrupts can occur for the
**			Uart.
**
*/
static int UART_SetupInterruptSystem(INTC *IntcInstancePtr,
				XUartPs *UartInstancePtr,
				u16 UartIntrId)
{
	int Status;

#ifndef TESTAPP_GEN
	XScuGic_Config *IntcConfig; /* Config for interrupt controller */

	/* Initialize the interrupt controller driver */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the interrupt controller interrupt handler to the
	 * hardware interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler) XScuGic_InterruptHandler,
				IntcInstancePtr);
#endif

	/*
	 * Connect a device driver handler that will be called when an
	 * interrupt for the device occurs, the device driver handler
	 * performs the specific interrupt processing for the device
	 */
	Status = XScuGic_Connect(IntcInstancePtr, UartIntrId,
				  (Xil_ExceptionHandler) XUartPs_InterruptHandler,
				  (void *) UartInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Enable the interrupt for the device */
	XScuGic_Enable(IntcInstancePtr, UartIntrId);


#ifndef TESTAPP_GEN
	/* Enable interrupts */
	 Xil_ExceptionEnable();
#endif

	return XST_SUCCESS;
}


/***	UART_Handler
**
**	Parameters:
**		void *CallBackRef		- CallBackRef contains a callback reference from the driver, in this case it is the instance pointer for the XUartPs driver.
**		u32 Event				- Event contains the specific kind of event that has occurred.
**		unsigned int EventData	- EventData contains the number of bytes sent or received for sent and receive events.
**
**
**	Return Value:
**         <none>
**
**	Description:
** 		This function is the handler which performs processing to handle data events
**		from the device.  It is called from an interrupt context. so the amount of
** 		processing should be minimal.
** 		Basically it deals with receive events, filling TotalReceivedCount with the number of received bytes
**
*/
void UART_Handler(void *CallBackRef, u32 Event, unsigned int EventData)
{

	/* All of the data has been received */
	if (Event == XUARTPS_EVENT_RECV_DATA) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received, but not the expected number of bytes, a
	 * timeout just indicates the data stopped for 8 character times
	 */
	if (Event == XUARTPS_EVENT_RECV_TOUT) {
		TotalReceivedCount = EventData;
	}

	/*
	 * Data was received with an error, keep the data but determine
	 * what kind of errors occurred
	 */
	if (Event == XUARTPS_EVENT_RECV_ERROR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}

	/*
	 * Data was received with an parity or frame or break error, keep the data
	 * but determine what kind of errors occurred. Specific to Zynq Ultrascale+
	 * MP.
	 */
	if (Event == XUARTPS_EVENT_PARE_FRAME_BRKE) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}

	/*
	 * Data was received with an overrun error, keep the data but determine
	 * what kind of errors occurred. Specific to Zynq Ultrascale+ MP.
	 */
	if (Event == XUARTPS_EVENT_RECV_ORERR) {
		TotalReceivedCount = EventData;
		TotalErrorCount++;
	}
}

