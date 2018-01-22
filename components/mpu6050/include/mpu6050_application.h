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

#define CALIBRATION_MODE_OFF        0
#define CALIBRATION_MODE_SIMPLE     1
#define CALIBRATION_MODE_ACCURATE   2

extern double a_x;
extern double a_y;
extern double a_z;

extern int16_t accelration_x;
extern int16_t accelration_y;
extern int16_t accelration_z;

extern double g_x;
extern double g_y;
extern double g_z;

extern int16_t gyro_x;
extern int16_t gyro_y;
extern int16_t gyro_z;

extern int16_t cal_a_x;
extern int16_t cal_a_y;
extern int16_t cal_a_z;

extern double angle_pitch;
extern double angle_roll;

extern double temperature;

extern uint8_t calibration_mode;

void vMPU6050Task( void *pvParameters );

#endif
