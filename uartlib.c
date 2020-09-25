/** @file uartlib.c  
*   
* @brief En esta libería se implementa la configuración y el
*  manejo de interrupciones del modulo UART.
* @par         
* COPYRIGHT NOTICE: Christian Sandoval - 16250.  
*/ 
 
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "arm_math.h"
#include "uartlib.h"
#include "timer0lib.h"
#include "timer1lib.h"
#include "pwmlib.h"
#include "pidlib.h"

 
/*!  
* @brief Identify the larger of two 8-bit integers.  
*   
* @param[in] None.    
*  
* @return None  
*/
void
UARTConfigure(void)
{
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	ROM_FPUEnable();
	ROM_FPULazyStackingEnable();


	//
	// Enable the peripherals used by this example.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable processor interrupts.
	//
	ROM_IntMasterEnable();

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure UART clock and baud rate
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
	UARTStdioConfig(0, 115200, ROM_SysCtlClockGet());

	ROM_IntEnable(INT_UART0);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	UARTEnable(UART0_BASE);

}

void
UARTIntHandler(void)
{
	uint32_t ui32Status;
	

	//
	// Get the interrrupt status.
	//
	ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

	//
	// Clear the asserted interrupts.
	//
	UARTIntClear(UART0_BASE, ui32Status);
	counter = 0;
	motor_enable = true;
	time_flag = false;
	if (uart_fcn == true)
	{
		Timer0Configure();
		Timer1Configure();
		if(MOTOR1 == true)
		{
			pid1Config();
		}
		
		if(MOTOR2 == true) 
		{
			pid2Config();
		}
	}
	
	while (ROM_UARTCharsAvail(UART0_BASE))
	{
		UARTprintf("%c",UARTgetc());
	}
}
 
/*** end of file ***/
