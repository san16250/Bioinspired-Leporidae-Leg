/** @file pwmlib.c  
*   
* @brief En esta libería se implementa la configuración y el
*  manejo de interrupciones del modulo PWM.
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
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "pwmlib.h"


 
/*!  
* @brief Identify the larger of two 8-bit integers.  
*   
* @param[in] None.    
*  
* @return None  
*/
void
PWMGen0Configure(void)
{
		// Configure the clock of the PWM
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
		
	// Configure PWM module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	
	// Wait for the PWM0 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
	{
	}
	
	//Enable the peripherals used by the PWM_1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//Configure the Pins to be used
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 |GPIO_PIN_7);
	
	//Configure the PWM options
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | 
									PWM_GEN_MODE_NO_SYNC);
	
	// Establish PWM period
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (uint32_t)pwm_period);
	
	
	//Enable the pwm generator
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,0);
	
	// Enable processor interrupts.
	ROM_IntMasterEnable();
	ROM_IntEnable(INT_PWM0_0_TM4C123);
	
	
	//Enable Interrupt
	ROM_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);
	ROM_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0,PWM_INT_CNT_ZERO);
}

void
PWMGen0IntHandler(void)
{
	uint32_t ui32StatusPWM_0;

	//
	// Get the interrrupt status.
	//
	ui32StatusPWM_0 = ROM_PWMGenIntStatus(PWM0_BASE, PWM_GEN_0 ,true);

	//
	// Clear the asserted interrupts.
	//
	ROM_PWMGenIntClear(PWM0_BASE, PWM_GEN_0 ,ui32StatusPWM_0);
	
	// Limitamos a un ciclo de trabajo de 100%
	if(duty_1 > 100) duty_1 = 100;
	else if (duty_1 < -100) duty_1 = -100;
	if (motor_enable == true)
	{
		if (duty_1 >= 0)
		{
			PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
			PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,(pwm_period*(duty_1/100)));
		}
		else if (duty_1 < 0)
		{
			PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
			PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,(pwm_period*(-duty_1/100)));
		}
	}
	else
	{
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
	}

}

void
PWMGen2Configure(void)
{
	// Configure the clock of the PWM
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
		
	// Configure PWM module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	
	// Wait for the PWM0 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
	{
	}
	
	//Enable the peripherals used by the PWM_0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
	//Configure the Pins to be used
	GPIOPinConfigure(GPIO_PE4_M0PWM4);
	GPIOPinConfigure(GPIO_PE5_M0PWM5);
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 |GPIO_PIN_5);
	
	//Configure the PWM options
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | 
									PWM_GEN_MODE_NO_SYNC);
	
	// Establish PWM period
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, (uint32_t)pwm_period);
	
	
	//Enable the pwm generator
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	
	PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
	
	// Enable processor interrupts.
	ROM_IntMasterEnable();
	ROM_IntEnable(INT_PWM0_2_TM4C123);
	
	//Enable Interrupt
	
	ROM_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_2);
	ROM_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_2,PWM_INT_CNT_ZERO);
}	

void
PWMGen2IntHandler(void)
{
	uint32_t ui32StatusPWM_2;

	//
	// Get the interrrupt status.
	//
	ui32StatusPWM_2 = ROM_PWMGenIntStatus(PWM0_BASE, PWM_GEN_2 ,true);

	//
	// Clear the asserted interrupts.
	//
	ROM_PWMGenIntClear(PWM0_BASE, PWM_GEN_2 ,ui32StatusPWM_2);
	
	// Limitamos a un ciclo de trabajo de 100%
	if(duty_0 > 100) duty_0 = 100;
	else if (duty_0 < -100) duty_0 = -100;
	if (motor_enable == true)
	{
		if (duty_0 >= 0)
		{
			PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
			PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,(pwm_period*(duty_0/100)));
		}
		else if (duty_0 < 0)
		{
			PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
			PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, (pwm_period*(duty_0/100)));
		}
	}
	else
	{
		PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, false);
		PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
	}
	
}

/*** end of file ***/
