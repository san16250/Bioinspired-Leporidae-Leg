/** @file mpu6050.c
 *
 * @brief Module for managing the functions of the MPU6050.
 *
 * @par
 * COPYRIGHT NOTICE: (c) 2018 Barr Group. All rights reserved.
 * Propietary: Christian Sandoval - san16250@uvg.edu.gt
 * Universidad del Valle de Guatemala.
 *
 * Please cite this code if used even if its just some parts.
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"

#include "Libraries/mpu6050/mpu6050.h"

/*!
 * @brief Write in a single register of the MPU6050.
 *
 * @param[in] mpu6050_address The address of the MPU6050 as a slave.
 * @param[in] reg The register to write.
 * @param[in] reg_value The value to set in the register
 *
 * @return void.
 */
void
mpu6050_single_write (uint8_t mpu6050_address, uint8_t reg, uint8_t reg_value)
{
    I2CMasterSlaveAddrSet(I2C0_BASE, mpu6050_address, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE))
    {
    }
    I2CMasterDataPut(I2C0_BASE, reg_value);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE))
    {
    }
}

/*!
 * @brief Configuration for the MPU6050.
 *
 * @param[in] mpu6050_address The address of the MPU6050 as a slave.
 *
 * @return void.
 */
void
mpu6050_init (uint8_t mpu6050_address)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
    I2CMasterSlaveAddrSet(I2C0_BASE, mpu6050_address, false);
    mpu6050_single_write(mpu6050_address, 0x6B, 0x00);
}

/*!
 * @brief Read a single register of the MPU6050.
 *
 * @param[in] mpu6050_address The address of the MPU6050 as a slave.
 * @param[in] reg The register to write.
 *
 * @return The value of the register as a uint8_t.
 */
uint8_t
mpu6050_single_read (uint8_t mpu6050_address, uint8_t reg)
{
    I2CMasterSlaveAddrSet(I2C0_BASE, mpu6050_address, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE))
    {
    }
    I2CMasterSlaveAddrSet(I2C0_BASE, mpu6050_address, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C0_BASE))
    {
    }
return (uint8_t)I2CMasterDataGet(I2C0_BASE);
}

/*!
 * @brief Read a single register of the MPU6050.
 *
 * @param[in] mpu6050_address The address of the MPU6050 as a slave.
 * @param[in] reg The register to write.
 *
 * @return The value of the register as a uint8_t.
 */
void
mpu6050_burst_read (uint8_t mpu6050_address, uint8_t reg_init, uint8_t reg_end,
                    uint8_t * addr_value)
{
    
    I2CMasterSlaveAddrSet(I2C0_BASE, mpu6050_address, false);
    I2CMasterDataPut(I2C0_BASE, reg_init);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while(I2CMasterBusy(I2C0_BASE))
    {
    }
    I2CMasterSlaveAddrSet(I2C0_BASE, mpu6050_address, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE))
    {
    }
    *addr_value++ = (uint8_t)I2CMasterDataGet(I2C0_BASE);
    for (uint8_t i = reg_init + 1; i < reg_end; i++)
    {
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C0_BASE))
        {
        }
        *addr_value++ = (uint8_t)I2CMasterDataGet(I2C0_BASE);
    }
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE))
    {
    }
    *addr_value++ = (uint8_t)I2CMasterDataGet(I2C0_BASE);
}

/*** end of file ***/
