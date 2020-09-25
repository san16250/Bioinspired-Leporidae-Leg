/** @file pidlib.c  
*   
* @brief En esta libería se implementa la configuración y el
*  manejo de interrupciones del modulo PID
* @par         
* COPYRIGHT NOTICE: Christian Sandoval - 16250.  
*/ 

#include <stdint.h> 
#include <stdbool.h>
#include "arm_math.h"
#include "pidlib.h"
void
pid1Config(void)
{
	//Set PID parameters
	PID_0.Kp = KP_POS_0_0;			//Proportional
	PID_0.Ki = KI_POS_0_0;			//Integral
	PID_0.Kd = KD_POS_0_0;			//Derivative

	//PID initialization
	arm_pid_init_f32(&PID_0, 1);
}

void
pid1Reset(void)
{
	arm_pid_reset_f32(&PID_0);
	PID_0.Kp = KP_POS_0_1;			//Proportional
	PID_0.Ki = KI_POS_0_1;			//Integral
	PID_0.Kd = KD_POS_0_1;			//Derivative
	arm_pid_init_f32(&PID_0, 1);
}

void
pid2Config(void)
{
	//Set PID parameters
	PID_1.Kp = KP_POS_1_0;			//Proportional
	PID_1.Ki = KI_POS_1_0;			//Integral
	PID_1.Kd = KD_POS_1_0;			//Derivative

	//PID initialization
	arm_pid_init_f32(&PID_1, 1);
}
/*** end of file ***/
