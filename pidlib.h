/** @file pidlib.h  
 *   
 * @brief   
 *  
 * @par         
 * COPYRIGHT NOTICE: Christian Sandoval - 16250  
 */  
 
#ifndef PIDLIB_H
#define PIDLIB_H
extern arm_pid_instance_f32 PID_0;
extern float KP_POS_0_0, KI_POS_0_0, KD_POS_0_0;
extern float KP_POS_0_1, KI_POS_0_1, KD_POS_0_1;
extern arm_pid_instance_f32 PID_1;
extern float KP_POS_1_0, KI_POS_1_0, KD_POS_1_0;
extern float KP_POS_1_1, KI_POS_1_1, KD_POS_1_1;
void pid1Config(void);
void pid1Reset(void);
void pid2Config(void);
 
#endif /* PIDLIB */ 
 
/*** end of file ***/
