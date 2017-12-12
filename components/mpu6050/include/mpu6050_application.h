/**********************************************************************
* - Description:		esp32-mpu6050
* - File:				mpu6050_application.h
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Mohamed El-Sabagh
* - Target:				ESP32
* - Created:			2017-12-11
* - Last changed:		2017-12-11
*
**********************************************************************/
#ifndef _MPU6050_APPLICATION_H
#define _MPU6050_APPLICATION_H

#include <stdint.h>

extern int16_t accelration_x;
extern int16_t accelration_y;
extern int16_t accelration_z;

extern double gforce_x;
extern double gforce_y;
extern double gforce_z;

extern int16_t gyro_x;
extern int16_t gyro_y;
extern int16_t gyro_z;

extern double rot_x;
extern double rot_y;
extern double rot_z;

extern double temperature;

void vMPU6050Task( void *pvParameters );

#endif
