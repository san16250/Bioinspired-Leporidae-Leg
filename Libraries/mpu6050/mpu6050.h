/** @file module.c
 *
 * @brief A description of the module’s purpose.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */

#ifndef MPU6050_H
#define MPU6050_H

void mpu6050_init(uint8_t mpu6050_address);
void mpu6050_single_write(uint8_t mpu6050_address, uint8_t reg,
                          uint8_t reg_value);
uint8_t mpu6050_single_read(uint8_t mpu6050_address, uint8_t reg);

#endif /* MPU6050_H */

/*** end of file ***/
