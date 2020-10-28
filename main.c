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
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "Libraries/Uartstdio/uartstdio.h"
#include "Libraries/encoder/encoder.h"
#include "Libraries/motor/motor.h"
#include "Libraries/Uart/uartlib.h"
#include "Libraries/Timer0/timer0lib.h"
#include "Libraries/Timer1/timer1lib.h"
#include "Libraries/pidlib/pidlib.h"
#include "Libraries/mpu6050/mpu6050.h"

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

#define SENSITIVITY_SCALE_ACCEL (16384.0f)
#define SENSITIVITY_SCALE_GYRO (131.0f)
#define GYRO_OFF_X (-1.41185f)
#define GYRO_OFF_Y (-0.54515f)
#define GYRO_OFF_Z (-1.07898f)
#define ACC_X_MAX (1.0083f)
#define ACC_Y_MAX (1.0110f)
#define ACC_Z_MAX (1.0000f)
#define ACC_X_MIN (-0.9800f)
#define ACC_Y_MIN (-0.9749f)
#define ACC_Z_MIN (-1.0333f)
#define TIME_SAMPLE (0.0170f)
#define LAMBDA (0.9795f)
#define NDEBUG (1u)


#ifndef NDEBUG
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
    struct Adrr_value
    {
        uint8_t accel_xout_h;
        uint8_t accel_xout_l;
        uint8_t accel_yout_h;
        uint8_t accel_yout_l;
        uint8_t accel_zout_h;
        uint8_t accel_zout_l;
        uint8_t temp_out_h;
        uint8_t temp_out_l;
        uint8_t gyro_xout_h;
        uint8_t gyro_xout_l;
        uint8_t gyro_yout_h;
        uint8_t gyro_yout_l;
        uint8_t gyro_zout_h;
        uint8_t gyro_zout_l;
    } adrr_value;
    int16_t accel_xout;
    int16_t accel_yout;
    int16_t accel_zout;
    int16_t gyro_xout;
    int16_t gyro_yout;
    int16_t gyro_zout;
    float ax;
    float ay;
    float az;
    float wx;
    float wx_n_1;
    float wy;
    float wy_n_1;
    float wz;
    float phi;
    float theta;
    float temp;
    float phi_n;
    float phi_n_1;
    float theta_n;
    float theta_n_1;
    float phi_gyro;
    float phi_gyro_n_1;
    float theta_gyro;
    float theta_gyro_n_1;
    float phi_accel;
    float phi_accel_n_1;
    float theta_accel;
    float theta_accel_n_1;
    float roll;
    float pitch;
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
        
        mpu6050_burst_read(0x68, 0x3B, 0x48, (uint8_t *)&adrr_value);
        // Sorting the accelerometer values to make a hole uint16_t variable.
        //
        accel_xout = ((adrr_value.accel_xout_h << 8) |
                       adrr_value.accel_xout_l);
        accel_yout = ((adrr_value.accel_yout_h << 8) |
                       adrr_value.accel_yout_l);
        accel_zout = ((adrr_value.accel_zout_h << 8) |
                       adrr_value.accel_zout_l);
        gyro_xout = ((adrr_value.gyro_xout_h << 8) | adrr_value.gyro_xout_l);
        gyro_yout = ((adrr_value.gyro_yout_h << 8) | adrr_value.gyro_yout_l);
        gyro_zout = ((adrr_value.gyro_zout_h << 8) | adrr_value.gyro_zout_l);

        //Obtaining the measurements with the unit g.
        //
        ax = accel_xout/SENSITIVITY_SCALE_ACCEL;
        #ifndef NDEBUG
        tmpSign = (ax < 0) ? "-" : "";
        tmpVal = (ax < 0) ? -ax : ax;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
        #endif

        ay = accel_yout/SENSITIVITY_SCALE_ACCEL;
        #ifndef NDEBUG
        tmpSign = (ay < 0) ? "-" : "";
        tmpVal = (ay < 0) ? -ay : ay;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
        #endif

        az = accel_zout/SENSITIVITY_SCALE_ACCEL;
        #ifndef NDEBUG
        tmpSign = (az < 0) ? "-" : "";
        tmpVal = (az < 0) ? -az : az;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
        #endif

        wx = gyro_xout/SENSITIVITY_SCALE_GYRO;
        #ifndef NDEBUG
        tmpSign = (wx < 0) ? "-" : "";
        tmpVal = (wx < 0) ? -wx : wx;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d, ", tmpSign, tmpInt1, tmpInt2);
        #endif

        wy = gyro_yout/SENSITIVITY_SCALE_GYRO;
        #ifndef NDEBUG
        tmpSign = (wy < 0) ? "-" : "";
        tmpVal = (wy < 0) ? -wy : wy;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d, ", tmpSign, tmpInt1, tmpInt2);
        #endif

        wz = gyro_zout/SENSITIVITY_SCALE_GYRO;
        #ifndef NDEBUG
        tmpSign = (wz < 0) ? "-" : "";
        tmpVal = (wz < 0) ? -wz : wz;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
        #endif

        // Normalizing the vectors.
        //
        wx = wx - GYRO_OFF_X;
        wy = wy - GYRO_OFF_Y;
        wz = wz - GYRO_OFF_Z;

        ax = (2*ax - (ACC_X_MAX + ACC_X_MIN)) / (ACC_X_MAX - ACC_X_MIN);
        ay = (2*ay - (ACC_Y_MAX + ACC_Y_MIN)) / (ACC_Y_MAX - ACC_Y_MIN);
        az = (2*az - (ACC_Z_MAX + ACC_Z_MIN)) / (ACC_Z_MAX - ACC_Z_MIN);

        // Estimates for the roll and pitch from accelerometer values.
        phi = -57.3 * atan2(ax, ay);
        temp = sqrt(ax * ax + az * az);
        theta = 57.3 * atan2(ay, temp);

        // Estimates for the roll and pitch from gyroscope values.
        phi_n = 0.5f * TIME_SAMPLE * (wx + wx_n_1) + phi_n_1;
        theta_n = 0.5f * TIME_SAMPLE * (wy + wy_n_1) + theta_n_1;

        wx_n_1 = wx;
        wy_n_1 = wy;
        phi_n_1 = phi_n;
        theta_n_1 = theta_n;

        // Complimentary filtered system implementing a Lowpass filter.
        // This is used to filter the high-frequency noise from the
        // accelerometer and filter the low-frequency noise form the
        // gyroscope.
        //
        phi_gyro = LAMBDA * (phi_n - phi_n_1 + phi_gyro_n_1);
        theta_gyro = LAMBDA * (theta_n - theta_n_1 + theta_gyro_n_1);

        phi_accel = ((1 - LAMBDA) * phi) + (LAMBDA * phi_accel_n_1);
        theta_accel = ((1 - LAMBDA) * theta) + (LAMBDA * theta_accel_n_1);

        phi_gyro_n_1 = phi_gyro;
        theta_gyro_n_1 = theta_gyro;
        phi_accel_n_1 = phi_accel;
        theta_accel_n_1 = theta_accel;

        // Printing the values for the roll and pitch angles as degrees,
        // using float type variables.
        //
        roll = phi_gyro + phi_accel;
        tmpSign = (roll < 0) ? "-" : "";
        tmpVal = (roll < 0) ? -roll : roll;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d, ", tmpSign, tmpInt1, tmpInt2);

        pitch = theta_gyro + theta_accel;
        tmpSign = (pitch < 0) ? "-" : "";
        tmpVal = (pitch < 0) ? -pitch : pitch;

        tmpInt1 = tmpVal;
        tmpFrac = tmpVal - tmpInt1;
        tmpInt2 = trunc(tmpFrac * 10000);
        UARTprintf("%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
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

    mpu6050_init(0x68);
}

/*** end of file ***/
