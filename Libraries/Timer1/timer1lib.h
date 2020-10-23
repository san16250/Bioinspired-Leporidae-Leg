/** @file timer1lib.h  
 *   
 * @brief Configuration and interrupt handler of timer 1  
 *  
 * @par         
 * COPYRIGHT NOTICE: Christian Sandoval - 16250  
 */  
 
#ifndef TIMER1LIB_H
#define TIMER1LIB_H

extern bool MOTOR1, MOTOR2;
extern bool time_flag;
extern bool uart_fcn;
extern uint8_t counter;
extern float duty_0, duty_1;
extern float pid_error_0, pid_error_1;
void Timer1Configure(void);
void Timer1IntHandler(uint32_t g_ui32Flags);
 
#endif /* TIMER1LIB_H */ 
 
/*** end of file ***/
