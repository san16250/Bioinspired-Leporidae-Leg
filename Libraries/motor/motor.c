/** @file motor.c
 *
 * @brief Module for speed control of a dc motor using PWM.
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
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "Libraries/motor/motor.h"


/*!
 * @brief Configure the PWM module 0 Generator 0 operation for the motor 1.
 *
 * @param[in] num1 The first number to be compared.
 * @param[in] num2 The second number to be compared.
 *
 * @return The value of the larger number.
 */
void
motor1_configure(uint32_t pwm_period)
{
    uint32_t temp;
    temp = SysCtlClockGet()/pwm_period;
    
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
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, temp);
	
	
	//Enable the pwm generator
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);
    
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
}

/*!
 * @brief Configure the PWM module 0 Generator 2 operation for the motor 2.
 *
 * @param[in] num1 The first number to be compared.
 * @param[in] num2 The second number to be compared.
 *
 * @return The value of the larger number.
 */
void
motor2_configure(uint32_t pwm_period)
{
    uint32_t temp;
    temp = SysCtlClockGet()/pwm_period;
    
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
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, temp);
	
	//Enable the pwm generator
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, false);
    
    
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
    
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
    
}	

/*!
 * @brief Write a duty cycle to the pwm module
 *
 * @param[in] pwm_base Base used for the pwm.
 * @param[in] pwm_out Module of the base used for the pwm.
 * @param[in] width Duty cycle in %.
 * @param[in] pwm_period Period for the clock of the pwm.
 *
 * @return Void.
 */
void
motor1_velocity_write(uint32_t pwm_base, uint32_t pwm_out, int32_t width,
uint32_t pwm_period)
{
    uint32_t duty;
    int8_t sign;
    
    sign = (width < 0) ? -1 : 1;
    width = (width < 0) ? -width : width;
    width = (width > 100) ? 100 : width;
    duty = (width) * (SysCtlClockGet()/(pwm_period * 100));
    if (sign > 0)
    {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_PIN_0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);
    }
    PWMPulseWidthSet(pwm_base, PWM_OUT_0, duty);
    if (duty == 0)
    {
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 0);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
    } 
}

/*!
 * @brief Write a duty cycle to the pwm module in motor2
 *
 * @param[in] pwm_base Base used for the pwm.
 * @param[in] pwm_out Module of the base used for the pwm.
 * @param[in] width Duty cycle in %.
 * @param[in] pwm_period Period for the clock of the pwm.
 *
 * @return Void.
 */
void
motor2_velocity_write(uint32_t pwm_base, uint32_t pwm_out, int32_t width,
uint32_t pwm_period)
{
    uint32_t duty;
    int8_t sign;
    
    sign = (width < 0) ? -1 : 1;
    width = (width < 0) ? -width : width;
    width = (width > 100) ? 100 : width;
    duty = (width) * (SysCtlClockGet()/(pwm_period * 100));
    if (sign > 0)
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
    }
    else
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }
    
    PWMPulseWidthSet(pwm_base, PWM_OUT_4, duty);
    if (duty == 0)
    {
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
    } 
}

/*** end of file ***/
