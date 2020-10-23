//*****************************************************************************
// Christian Sandoval
// 16250
// Programa para control de ambas piernas con PID
// Creado 13/07/2020
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "Libraries/Uartstdio/uartstdio.h"
#include "arm_math.h"
#include "Libraries/Uart/uartlib.h"
#include "Libraries/Timer0/timer0lib.h"
#include "Libraries/Timer1/timer1lib.h"
#include "Libraries/PWM/pwmlib.h"
#include "pidlib.h"

//*****************************************************************************
//
// Macros
//
//*****************************************************************************

//*****************************************************************************
// Operation Options
//*****************************************************************************
#define MOTOR1_RATIO (75)
#define ENCODER1_PULSES (12)
/*
* PID PARAMETERS FOR MOTOR 1
*/
float KP_POS_0_0 = 140;
float KI_POS_0_0 = 0.02599;
float KD_POS_0_0 = 70;
float KP_POS_0_1 = 5.5;
float KI_POS_0_1 = 0.02599;
float KD_POS_0_1 = 2.7235;
#define WANTED_POS_0_0 (360)
#define WANTED_POS_0_1 (400)
//ARM PID instance, float_32 format
arm_pid_instance_f32 PID_0;
bool MOTOR1 = true;					//False -> OFF			True -> ON



#define MOTOR2_RATIO (100)
#define ENCODER2_PULSES (12)
/*
*	PID PARAMETERS FOR MOTOR 2
*/

float KP_POS_1_0 = 110;
float KI_POS_1_0 = 0.02599;
float KD_POS_1_0 = 25;
float KP_POS_1_1 = 1.5;
float KI_POS_1_1 = 0.1;
float KD_POS_1_1 = 15;
#define WANTED_POS_1_0 (360)
#define WANTED_POS_1_1 (280)
//ARM PID instance, float_32 format
arm_pid_instance_f32 PID_1;

bool MOTOR2 = true;				//False -> OFF			True -> ON

//*****************************************************************************
// Program constants
//*****************************************************************************
#define PWM_P (10000)					//Frequency of PWM
//*****************************************************************************
//
// Variable declaration
//
//*****************************************************************************
// First motor variables
float pid_error_0, duty_0, current_pos_0;

// Second motor variables
float pid_error_1, duty_1, current_pos_1;		//Variables for PID control


// PWM Operation
float pwm_period; // variable for the speed of the dc motor.

bool time_flag = false;
bool uart_fcn = true;
uint8_t counter = 0;
bool motor_enable = true;


//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configuration of Motor 1
//
//*****************************************************************************
void
Motor1Config(void)
{
	//***************************************************************************
	// Configuration of the quadrature encoder
	//***************************************************************************
	// Enable QEI Module 0 pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);


	// Wait for the QEI0 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0))
	{
	}

	// Unlock pin PD7
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	// Set pins for PHA0 and PHB0
	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);

	// Configure pins for use by the QEI peripheral
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	//Make sure quadrature encoder is off
	QEIDisable(QEI0_BASE);
	QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR |
		QEI_INTTIMER | QEI_INTINDEX);

	//Configure quadrature encoder using FT0481 top limit
	QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |
		QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), MOTOR1_RATIO*ENCODER1_PULSES*2);

	//Enable quadrature encoder
	QEIEnable(QEI0_BASE);

	QEIPositionSet(QEI0_BASE, MOTOR1_RATIO*ENCODER1_PULSES);

	//Enable noise filter
	QEIFilterDisable(QEI0_BASE);
	QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_2);
	QEIFilterEnable(QEI0_BASE);
}

//*****************************************************************************
//
// Configuration of Motor 2
//
//*****************************************************************************
void
Motor2Config(void)
{
	//***************************************************************************
	// Configuration of the quadrature encoder
	//***************************************************************************
	// Enable QEI Module 1 pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

	// Wait for the QEI1 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1))
	{
	}

	//Set pins for PHA1 and PHB1
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);

	//Configure pins for use by the QEI peripheral
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

	//Make sure quadrature encoder is off
	QEIDisable(QEI1_BASE);
	QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR |
		QEI_INTTIMER | QEI_INTINDEX);

	//Configure quadrature encoder using FT0481 top limit
	QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET |
		QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP), MOTOR2_RATIO*ENCODER2_PULSES*2);


	//Enable quadrature encoder
	QEIEnable(QEI1_BASE);

	QEIPositionSet(QEI1_BASE, MOTOR2_RATIO*ENCODER2_PULSES);


	//Enable noise filter
	QEIFilterDisable(QEI1_BASE);
	QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_2);
	QEIFilterEnable(QEI1_BASE);
}


//*****************************************************************************
//
// Main Loop
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
	SysCtlDelay((uint32_t)ROM_SysCtlClockGet);
		if (uart_fcn == true) motor_enable = false;
		pwm_period = (float)(SysCtlClockGet()/PWM_P);
		UARTConfigure();

		if (uart_fcn == false)
		{
			Timer0Configure();
			Timer1Configure();
			if(MOTOR1 == true)
			{
				Motor1Config();
				PWMGen2Configure();
				pid1Config();
			}

			if(MOTOR2 == true)
			{
				Motor2Config();
				PWMGen0Configure();
				pid2Config();
			}
	  }


		Timer0Configure();
		Timer1Configure();
		if (MOTOR1 == true)
		{
			Motor1Config();
			PWMGen2Configure();
			pid1Config();
		}
		if (MOTOR2 == true)
		{
			Motor2Config();
			PWMGen0Configure();
			pid2Config();
		}

    while(1)
    {
			//
			// PID motor 1
			//
			if (MOTOR1 == true)
			{
				if (SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0))
				{
					current_pos_0 = ((float)QEIPositionGet(QEI0_BASE)*(360.0/(MOTOR1_RATIO*ENCODER1_PULSES)));
				}
				// Calculamos el error
				if (time_flag == false)
				{
					pid_error_0 = WANTED_POS_0_0 - current_pos_0;
				}
				if (time_flag == true)
				{
					pid_error_0 = WANTED_POS_0_1 - current_pos_0;
				}
			}

			//
			// PID motor 2
			//
			if(MOTOR2 == true)
			{
				if (SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1))
				{
					current_pos_1 = ((float)QEIPositionGet(QEI1_BASE)*(360.0/(MOTOR2_RATIO*ENCODER2_PULSES)));
				}
				// Calculamos el error
				if (time_flag == false) pid_error_1 = WANTED_POS_1_0 - current_pos_1;
				if (time_flag == true) pid_error_1 = WANTED_POS_1_1 - current_pos_1;
			}

		if (counter >= 2)
		{
			ROM_SysCtlPeripheralDisable(SYSCTL_PERIPH_TIMER1);
			motor_enable = true;
		}
			//UARTprintf("%i \n", (int32_t)time_flag);
			//if (MOTOR1 == true && MOTOR2 == false) UARTprintf("%i, %i, %i\n",(int32_t)current_pos_0, (int32_t)pid_error_0, (int32_t)duty_0);
			// Current Pos in UART

			if (MOTOR1 == true && MOTOR2 == false) UARTprintf("%3u , %3i\n",(uint32_t)current_pos_0, (int32_t)duty_0);

			if (MOTOR1 == true && MOTOR2 == false) UARTprintf("%3u\n",(uint32_t)current_pos_0);

			if (MOTOR1 == true && MOTOR2 == true)UARTprintf("%3u, %3u \n", (uint32_t)current_pos_0,
				(uint32_t)current_pos_1);


    }
}
