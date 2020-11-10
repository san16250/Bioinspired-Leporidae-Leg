/** @file uartlib.h
 *
 * @brief Library for the initialization and the interrupt controller for the
 * UART module.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */
 
#ifndef UARTLIB_H
#define UARTLIB_H 

void uart_configure(void);
bool is_character_received(void);
void uart_clear(void);
void uart_isr(void);
 
#endif /* UARTLIB */ 
 
/*** end of file ***/
