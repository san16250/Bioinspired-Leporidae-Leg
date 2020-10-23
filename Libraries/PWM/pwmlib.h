/** @file pwmlib.h  
 *   
 * @brief Configuration and interrupt handler of timer 1  
 *  
 * @par         
 * COPYRIGHT NOTICE: Christian Sandoval - 16250  
 */  
 
#ifndef PWMLIB_H
#define PWMLIB_H 

extern bool motor_enable;
extern float pwm_period;
extern float duty_1, duty_0;
void PWMGen0Configure(void);
void PWMGen2Configure(void);
 
#endif /* PWMLIB */ 
 
/*** end of file ***/
