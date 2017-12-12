/**********************************************************************
* - Description:		esp32-mpu6050
* - File:				i2c_driver.h
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Mohamed El-Sabagh
* - Target:				ESP32
* - Created:			2017-12-11
* - Last changed:		2017-12-11
*
**********************************************************************/
#ifndef _I2C_DRIVER_H
#define _I2C_DRIVER_H

#include <stdint.h>
#include "i2c_driver.h"
#include "driver/i2c.h"

void vI2CInit();
esp_err_t sI2cMasterReadSlave(uint8_t* data_rd, size_t size, uint8_t slave_address);
esp_err_t sI2cMasterWriteSlave(uint8_t* data_wr, size_t size, uint8_t slave_address);

#endif
