/** @file timer0lib.h
 *
 * @brief Control the timer 0.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */
 
#ifndef TIMER0LIB_H
#define TIMER0LIB_H

void timer0configure(uint32_t frequency);
bool is_timer0_done(void);
void reset_timer0(void);
void timer0_isr(uint32_t g_ui32Flags);
 
#endif /* TIMER0LIB_H */ 
 
/*** end of file ***/
