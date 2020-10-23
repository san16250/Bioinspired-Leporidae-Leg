/** @file timer1lib.c  
*   
* @brief En esta libería se implementa la configuración y el
*  manejo de interrupciones del Timer 1.
* @par         
* COPYRIGHT NOTICE: Christian Sandoval - 16250.  
*/ 
 
#include <stdint.h> 
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "Libraries/Timer1/timer1lib.h"
#include "utils/uartstdio.h"
#include "arm_math.h"
#include "pidlib.h"
#include "Libraries/Uart/uartlib.h"


/*!  
* @brief Identify the larger of two 8-bit integers.  
*   
* @param[in] None.    
*  
* @return None  
*/
void
Timer1Configure (void)
{
		//
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
		//
    // Configure the two 32-bit periodic timers.
    //
		ROM_TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
		ROM_TimerLoadSet(TIMER1_BASE, TIMER_A, (ROM_SysCtlClockGet()/6));
		//
    // Setup the interrupts for the timer timeouts.
    //
		ROM_IntEnable(INT_TIMER1A);
		ROM_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
		//
    // Enable the timers.
    //
		ROM_TimerEnable(TIMER1_BASE, TIMER_A);
}

void
Timer1IntHandler(uint32_t g_ui32Flags)
{
	//
	// Despejamos las interrupciones 
	//
	ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Toggle La bandera para la primera vez
	//
	HWREGBITW(&g_ui32Flags, 0) ^= 1;
	
	
	//UARTprintf("%u \n", counter);
	
	time_flag = !time_flag;

	if (time_flag == true)
	{
		if (MOTOR1 == true)
		{
			arm_pid_reset_f32(&PID_0);
			PID_0.Kp = KP_POS_0_0;			//Proportional
			PID_0.Ki = KI_POS_0_0;			//Integral
			PID_0.Kd = KD_POS_0_0;			//Derivative
			arm_pid_init_f32(&PID_0, 1);
		}
		if (MOTOR2 == true)
		{
			arm_pid_reset_f32(&PID_1);
			PID_1.Kp = KP_POS_1_0;			//Proportional
			PID_1.Ki = KI_POS_1_0;			//Integral
			PID_1.Kd = KD_POS_1_0;			//Derivative
			arm_pid_init_f32(&PID_1, 1);
		}
	}
	else
	{
		if (MOTOR1 == true)
		{
			arm_pid_reset_f32(&PID_0);
			PID_0.Kp = KP_POS_0_1;			//Proportional
			PID_0.Ki = KI_POS_0_1;			//Integral
			PID_0.Kd = KD_POS_0_1;			//Derivative
			arm_pid_init_f32(&PID_0, 1);
		}
		if (MOTOR2 == true)
		{
			arm_pid_reset_f32(&PID_1);
			PID_1.Kp = KP_POS_1_1;			//Proportional
			PID_1.Ki = KI_POS_1_1;			//Integral
			PID_1.Kd = KD_POS_1_1;			//Derivative
			arm_pid_init_f32(&PID_1, 1);
		}
		
	}
	
	if (uart_fcn == true)
	{
		counter += 1;
	}
	ROM_IntMasterEnable();		//Habilitamos Interrupciones generales
	
}
 
/*** end of file ***/
