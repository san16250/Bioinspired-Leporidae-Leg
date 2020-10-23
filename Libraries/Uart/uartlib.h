/** @file uartlib.h  
 *   
 * @brief Configuration and interrupt handler of timer 1  
 *  
 * @par         
 * COPYRIGHT NOTICE: Christian Sandoval - 16250  
 */  
 
#ifndef UARTLIB_H
#define UARTLIB_H 
extern uint8_t counter;
extern bool time_flag;
extern bool uart_fcn;
extern bool motor_enable;
extern void Motor1Config(void);
extern void Motor2Config(void);
void UARTConfigure(void);
void UARTIntHandler(void);
 
#endif /* UARTLIB */ 
 
/*** end of file ***/
