/** @file timer0lib.c  
*   
* @brief En esta libería se implementa la configuración y el
*  manejo de interrupciones del Timer 0.
* @par         
* COPYRIGHT NOTICE: Christian Sandoval - 16250.  
*/ 
 
#include <stdint.h> 
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "arm_math.h"
#include "Libraries/Timer0/timer0lib.h"

 
/*!  
* @brief Identify the larger of two 8-bit integers.  
*   
* @param[in] None.    
*  
* @return None  
*/
void
Timer0Configure (void)
{
		//
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		//
    // Configure the two 32-bit periodic timers.
    //
		ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, (ROM_SysCtlClockGet()/150));
		//
    // Setup the interrupts for the timer timeouts.
    //
		ROM_IntEnable(INT_TIMER0A);
		ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	
		ROM_TimerEnable(TIMER0_BASE, TIMER_A);
		
}
 
void
Timer0IntHandler(uint32_t g_ui32Flags)
{

	//
	// Despejamos las interrupciones 
	//
	ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	//
	// Toggle La bandera para la primera vez
	//
	HWREGBITW(&g_ui32Flags, 0) ^= 1;

	// Calculamos la ecuación de salida
	if (MOTOR1 == true)
	{
		duty_0 = arm_pid_f32(&PID_0, pid_error_0);
	}
	if (MOTOR2 == true)
	{
		duty_1 = arm_pid_f32(&PID_1, pid_error_1);
	}
	ROM_IntMasterEnable();		//Habilitamos Interrupciones generales
}
/*** end of file ***/
