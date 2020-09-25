/** @file timer0lib.h  
 *   
 * @brief Configuration and interrupt handler of timer 10 
 *  
 * @par         
 * COPYRIGHT NOTICE: Christian Sandoval - 16250  
 */  
 
#ifndef TIMER0LIB_H
#define TIMER0LIB_H
extern bool MOTOR1; 
extern bool MOTOR2;
extern float duty_0, pid_error_0;
extern float duty_1, pid_error_1;
extern arm_pid_instance_f32 PID_0, PID_1;

void Timer0Configure(void);
void Timer0IntHandler(uint32_t g_ui32Flags);
 
#endif /* TIMER0LIB_H */ 
 
/*** end of file ***/
