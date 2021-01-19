/** @file motor.h
 *
 * @brief Module for speed control of a dc motor with an encoder.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */

#ifndef MOTOR_H
#define MOTOR_H

void motor1_configure(uint32_t pwm_period);
void motor2_configure(uint32_t pwm_period);
void motor1_velocity_write(uint32_t pwm_base, uint32_t pwm_out, int32_t width,
    uint32_t pwm_period);
void motor2_velocity_write(uint32_t pwm_base, uint32_t pwm_out, int32_t width,
    uint32_t pwm_period);


#endif /* MOTOR_H */

/*** end of file ***/
