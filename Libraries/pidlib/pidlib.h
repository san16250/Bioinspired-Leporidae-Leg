/** @file pidlib.h
 *
 * @brief PID library to configure, calculate and reset the constants.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */
 
#ifndef PIDLIB_H
#define PIDLIB_H

void pid_config(uint8_t  module, float kp, float ki, float kd);
float pid_calc(uint8_t module, float error);
 
#endif /* PIDLIB */ 
 
/*** end of file ***/
