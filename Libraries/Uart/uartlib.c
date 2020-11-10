/** @file uartlib.c
 *
 * @brief Library for the initialization and the interrupt controller for the
 * UART module.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */
 
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "Libraries/Uart/uartlib.h"
#include "utils/uartstdio.h"

static bool character_received = false;
 
/*!  
* @brief Configure the UART0 module at 115200 bauds with system clock.
*   
* @param[in] Void.   
*  
* @return Void. 
*/
void
uart_configure (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
    
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTEnable(UART0_BASE);
}

/*!  
* @brief Check if there has been input from the user.
*   
* @param[in] Void.   
*  
* @return The status of the interrupt.
*/
bool
is_character_received (void)
{
    return character_received;
}

/*!  
* @brief Clear the interrupt status from the UART module.
*   
* @param[in] Void.   
*  
* @return Void. 
*/
void
uart_clear (void)
{
    character_received = false;
}

/*!  
* @brief UART interrupt handler
*   
* @param[in] Void.   
*  
* @return Void. 
*/
void
uart_isr (void)
{
	uint32_t ui32Status;
	ui32Status = UARTIntStatus(UART0_BASE, true);
	// Clear the asserted interrupts.
	//
	UARTIntClear(UART0_BASE, ui32Status);
	
	while (UARTCharsAvail(UART0_BASE))
	{
		UARTgetc();
	}
    
    character_received = true;
}
 
/*** end of file ***/
