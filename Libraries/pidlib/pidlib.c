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
#include "Libraries/pidlib/pidlib.h"


static arm_pid_instance_f32 PID_0;
static arm_pid_instance_f32 PID_1;

/*!  
* @brief Configure and reset the PID instance.
*   
* @param[in] module Module of the PID to calculate.  
* @param[in] kp Porportional constant of the PID.
* @param[in] ki Integral constant of the PID.
* @param[in] kd Derivative constant of the PID.
*  
* @return Void. 
*/
void
pid_config (uint8_t  module, float kp, float ki, float kd)
{
    arm_pid_instance_f32 * temp;
    if (module == 0)
    {
        temp = &PID_0;
    }
    else
    {
        temp = &PID_1;
    }
    arm_pid_reset_f32(temp);
	temp->Kp = kp;			//Proportional
	temp->Ki = ki;			//Integral
	temp->Kd = kd;			//Derivative
	//PID initialization
	arm_pid_init_f32(temp, 1);
}

/*!  
* @brief Compute the error for the given PID module.
*   
* @param[in] module Module of the PID to use. Either 0 or 1.
* @param[in] error The difference between the desired position and the current
* encoder position.
*  
* @return The output of the PID calculation. 
*/
float
pid_calc (uint8_t module, float error)
{
    arm_pid_instance_f32 * temp;
    if (module == 0)
    {
        temp = &PID_0;
    }
    else
    {
        temp = &PID_1;
    }
    float out;
    out = arm_pid_f32(temp, error);
return out;
}
