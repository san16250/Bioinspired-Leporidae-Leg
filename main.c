/** @file main.c
 *
 * @brief Main module to manage all the operations
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
#include <stdio.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "Libraries/Uartstdio/uartstdio.h"
#include "Libraries/encoder/encoder.h"
#include "Libraries/motor/motor.h"
#include "Libraries/Uart/uartlib.h"
#include "Libraries/Timer0/timer0lib.h"
#include "Libraries/Timer1/timer1lib.h"
#include "Libraries/pidlib/pidlib.h"

#define MOTOR1_ENABLE (1u)
#define MOTOR1_RATIO (75u)
#define ENCODER1_PULSES (12u)
#define WANTED_POS_0_0 (360.0f)
#define WANTED_POS_0_1 (400.0f)

#define MOTOR2_ENABLE (1u)
#define MOTOR2_RATIO (100u)
#define ENCODER2_PULSES (12u)
#define WANTED_POS_1_0 (360.0f)
#define WANTED_POS_1_1 (280.0f)

#define PWM_P (10000u)

#if 0
#define NDEBUG (1u)
#endif

#define UART (1u)

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


uint32_t g_ui32Flags;
static bool timer1_status = false;
/*
* PID PARAMETERS FOR MOTOR 1
*/
static float KP_POS_0_0 = 140.0f;
static float KI_POS_0_0 = 0.02599f;
static float KD_POS_0_0 = 70.0f;
static float KP_POS_0_1 = 5.5f;
static float KI_POS_0_1 = 0.02599f;
static float KD_POS_0_1 = 2.7235f;

/*
*	PID PARAMETERS FOR MOTOR 2
*/
#if 0
static float KP_POS_1_0 = 110.0f;
static float KI_POS_1_0 = 0.02599f;
static float KD_POS_1_0 = 25.0f;
#endif
static float KP_POS_1_0 = 1.5f;
static float KI_POS_1_0 = 0.01;
static float KD_POS_1_0 = 15.0f;
static float KP_POS_1_1 = 1.5f;
static float KI_POS_1_1 = 0.1f;
static float KD_POS_1_1 = 15.0f;


void hardware_setup (void);

/*!
 * @brief Main cycle of the program.
 *
 * @param[in] Void.
 *
 * @return Void.
 */
int
main(void)
{
    hardware_setup();
    #ifndef NDEBUG
    char * tmpSign;
    float tmpVal;
    int32_t tmpInt1;
    float tmpFrac;
    int32_t tmpInt2;
    #endif
    
    float encoder1_pos;
    float encoder2_pos;
    float error_1;
    float error_2;
    float duty_1;
    float duty_2;
    bool reference_change_flag = true;
    float motor1_desired_position = 0;
    float motor2_desired_position = 0;
    bool uart_status = false;
    bool timer0_status = false;
    timer1_status = false;
    uint8_t timer1_interrupt_counter = 3;
    while(1)
    {
        #ifdef UART
        uart_status = is_character_received();
        if ((uart_status == true) && (timer1_interrupt_counter != 2))
        {
            timer1_interrupt_counter = 1;
        }
        uart_clear();
        #endif
        #ifndef UART
        timer1_interrupt_counter = 0;
        #endif
        
        timer0_status = is_timer0_done();
        if (timer0_status == true)
        {
            encoder1_pos = get_position_in_degrees(QEI0_BASE, MOTOR1_RATIO,
                                               ENCODER1_PULSES);
            encoder2_pos = get_position_in_degrees(QEI1_BASE, MOTOR2_RATIO,
                                               ENCODER2_PULSES);
            error_1 = motor1_desired_position - encoder1_pos;
            error_2 = motor2_desired_position - encoder2_pos;
            duty_1 = pid_calc(0, error_1);
            duty_2 = pid_calc(1, error_2);
            #ifdef MOTOR1_ENABLE
            motor_velocity_write(PWM0_BASE, PWM_GEN_0, duty_1, PWM_P);
            #endif
            #ifdef MOTOR2_ENABLE
            motor_velocity_write(PWM0_BASE, PWM_GEN_2, duty_2, PWM_P);
            #endif
        }
        reset_timer0();
        
        timer1_status = is_timer1_done();
        if (timer1_status == true)
        {
            if (timer1_interrupt_counter < 3)
            {
                reference_change_flag = !reference_change_flag;
            }
            if (reference_change_flag == true)
            {
                pid_config(0, KP_POS_0_0, KI_POS_0_0, KD_POS_0_0);
                pid_config(1, KP_POS_1_0, KI_POS_1_0, KD_POS_1_0);
                motor1_desired_position = WANTED_POS_0_0;
                motor2_desired_position = WANTED_POS_1_0;
            }
            else
            {
                pid_config(0, KP_POS_0_1, KI_POS_0_1, KD_POS_0_1);
                pid_config(1, KP_POS_1_1, KI_POS_1_1, KD_POS_1_1);
                motor1_desired_position = WANTED_POS_0_1;
                motor2_desired_position = WANTED_POS_1_1;
            }
            timer1_interrupt_counter = (timer1_interrupt_counter < 3) ?
                timer1_interrupt_counter + 1 : timer1_interrupt_counter;
        }
        reset_timer1();
        
        #ifndef NDEBUG
        tmpSign = (encoder1_pos < 0) ? "-" : "";
        tmpVal = (encoder1_pos < 0) ? -encoder1_pos : encoder1_pos;
        
        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("Encoder 1: %s%d.%04d, ", tmpSign, tmpInt1, tmpInt2);
     

        tmpSign = (encoder2_pos < 0) ? "-" : "";
        tmpVal = (encoder2_pos < 0) ? -encoder2_pos : encoder2_pos;
        
        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("Encoder 2: %s%d.%04d, ", tmpSign, tmpInt1, tmpInt2);


        tmpSign = (duty_1 < 0) ? "-" : "";
        tmpVal = (duty_1 < 0) ? -duty_1 : duty_1;
        
        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("Error 1: %s%d.%04d, ", tmpSign, tmpInt1, tmpInt2);


        tmpSign = (duty_2 < 0) ? "-" : "";
        tmpVal = (duty_2 < 0) ? -duty_2 : duty_2;
        
        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("Error 2: %s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
        #endif
    }
}

/*!
 * @brief Setup the hardware to be used during the program.
 *
 * @param[in] Void.
 *
 * @return Void.
 */
void
hardware_setup (void)
{
    // Setting the system clock to 80Mhz.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
	SysCtlDelay((uint32_t)SysCtlClockGet);
    
    uart_configure();

    qei_module0_config(MOTOR1_RATIO, ENCODER1_PULSES, true);
    qei_module1_config(MOTOR2_RATIO, ENCODER2_PULSES, false);
    
    motor1_configure(PWM_P);
    motor2_configure(PWM_P);
    
    timer0configure(150u);
    timer1configure(1u);
    while(!timer1_status)
    {
        timer1_status = is_timer1_done();
    }
    
    pid_config(0u, KP_POS_0_0, KI_POS_0_0, KD_POS_0_0);
    pid_config(1u, KP_POS_1_0, KI_POS_1_0, KD_POS_1_0);
}

/*** end of file ***/
