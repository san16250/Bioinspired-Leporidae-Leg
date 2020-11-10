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
 
#ifndef TIMER2LIB_H
#define TIMER2LIB_H

void timer2configure(uint32_t frequency);
bool is_timer2_done(void);
void reset_timer2(void);
void timer2_isr(uint32_t g_ui32Flags);
 
#endif /* TIMER2LIB_H */ 
 
/*** end of file ***/
