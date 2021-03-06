/** @file timer1lib.h
 *
 * @brief Control the timer 1.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */ 
 
#ifndef TIMER1LIB_H
#define TIMER1LIB_H

void timer1configure(uint32_t frequency);
bool is_timer1_done(void);
void reset_timer1(void);
void timer1_isr(uint32_t g_ui32Flags);
 
#endif /* TIMER1LIB_H */ 
 
/*** end of file ***/
