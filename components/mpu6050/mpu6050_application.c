/**********************************************************************
* - Description:		esp32-mpu6050
* - File:				mpu6050_application.c
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Mohamed El-Sabagh
* - Target:				ESP32
* - Created:			2017-12-11
* - Last changed:		2017-12-11
*
**********************************************************************/
#include "mpu6050_application.h"
#include "i2c_application.h"
#include "AppConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#define MPU6050_SLAVE_ADDR          0x68

#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_GYRO_CONFIG         0x1B

int16_t accelration_x = 0;
int16_t accelration_y = 0;
int16_t accelration_z = 0;

double gforce_x = 0.0;
double gforce_y = 0.0;
double gforce_z = 0.0;

int16_t gyro_x = 0.0;
int16_t gyro_y = 0.0;
int16_t gyro_z = 0.0;

double rot_x = 0.0;
double rot_y = 0.0;
double rot_z = 0.0;

double temperature = 0.0;

static I2C_Status i2cStatus;
static uint8_t mpu6050Buffer[MICRO_BUFFER_SIZE];

#define MPU6050_SUCESS					 	       0
#define MPU6050_FAILURE						       1
#define ACCELEROMETER_READING_FREQUENCY_MS	     1000	// 1 second

#define _2G_SCALE               0x00
#define _4G_SCALE               0x08
#define _8G_SCALE               0x10
#define _16G_SCALE              0x18

#define _250_DEGREE             0x00
#define _500_DEGREE             0x08
#define _1000_DEGREE            0x10
#define _2000_DEGREE            0x18

#define LSB_Sensitivity_2G      16384.0
#define LSB_Sensitivity_4G      8192.0
#define LSB_Sensitivity_8G      4096.0
#define LSB_Sensitivity_16G     2048.0

#define LSB_Sensitivity_250     131.0
#define LSB_Sensitivity_500     65.5
#define LSB_Sensitivity_1000    32.8
#define LSB_Sensitivity_2000    16.4

static uint8_t uMPU6050Init();
static uint8_t uMPU6050ReadAccelerometer();
static uint8_t uMPU6050ReadGyroscope();
static uint8_t uMPU6050ReadTemperature();

/**
  * @brief  MPU6050 task.
  * @param  None
  * @retval None
  */
void vMPU6050Task( void *pvParameters )
{
    if (uMPU6050Init() != MPU6050_SUCESS) {
        vTaskDelete(NULL);
        return;
    }

    for(;;)
	{
        if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
            continue;
        }

        if (uMPU6050ReadGyroscope() != MPU6050_SUCESS) {
            continue;
        }

        if (uMPU6050ReadTemperature() != MPU6050_SUCESS) {
            continue;
        }

        printf("##### Acceleration #####\n");
        printf("Acceleration X = %d\n", accelration_x);
        printf("Acceleration Y = %d\n", accelration_y);
        printf("Acceleration Z = %d\n", accelration_z);
        printf("\n");
        printf("##### Gravity force #####\n");
        printf("G Force X = %f\n", gforce_x);
        printf("G Force Y = %f\n", gforce_y);
        printf("G Force Z = %f\n", gforce_z);
        printf("\n");
        printf("##### Gyroscope #####\n");
        printf("Gyroscope X = %d\n", gyro_x);
        printf("Gyroscope Y = %d\n", gyro_y);
        printf("Gyroscope Z = %d\n", gyro_z);
        printf("\n");
        printf("##### Rotation #####\n");
        printf("Rotation X = %f\n", rot_x);
        printf("Rotation Y = %f\n", rot_x);
        printf("Rotation Z = %f\n", rot_x);
        printf("\n");
        printf("##### Temperature #####\n");
        printf("Temperature = %f\n", temperature);

        vTaskDelay( ACCELEROMETER_READING_FREQUENCY_MS / portTICK_RATE_MS );
    }

    vTaskDelete(NULL);
}

/**
  * @brief  Configure the MPU 6050.
  * @param  None
  * @retval Success or Fail
  */
uint8_t uMPU6050Init()
{
    portBASE_TYPE xStatus;

    // Change status from sleep to running
    mpu6050Buffer[0] = MPU6050_PWR_MGMT_1;
    mpu6050Buffer[1] = 0x00;
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 2;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    // Configure accelerometer
    mpu6050Buffer[0] = MPU6050_ACCEL_CONFIG;
    mpu6050Buffer[1] = _2G_SCALE;        // +/-2g
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 2;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    // Configure gyroscope
    mpu6050Buffer[0] = MPU6050_GYRO_CONFIG;
    mpu6050Buffer[1] = _250_DEGREE;        // +/-250degree
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 2;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    return MPU6050_SUCESS;
}

/**
  * @brief  Read accelerometer.
  * @param  None
  * @retval Success or Fail
  */
uint8_t uMPU6050ReadAccelerometer()
{
    portBASE_TYPE xStatus;

    // Read accelerometer
    mpu6050Buffer[0] = MPU6050_ACCEL_XOUT_H;
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 1;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 6;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    accelration_x =
        ((uint16_t)(mpu6050Buffer[0] << 8)) |
        ((uint16_t)(mpu6050Buffer[1]));

    accelration_y =
        ((uint16_t)(mpu6050Buffer[2] << 8)) |
        ((uint16_t)(mpu6050Buffer[3]));

    accelration_z =
        ((uint16_t)(mpu6050Buffer[4] << 8)) |
        ((uint16_t)(mpu6050Buffer[5]));

    gforce_x = accelration_x / LSB_Sensitivity_2G;
    gforce_y = accelration_y / LSB_Sensitivity_2G;
    gforce_z = accelration_z / LSB_Sensitivity_2G;

    return MPU6050_SUCESS;
}

/**
  * @brief  Read gyroscope.
  * @param  None
  * @retval Success or Fail
  */
uint8_t uMPU6050ReadGyroscope()
{
    portBASE_TYPE xStatus;

    // Read accelerometer
    mpu6050Buffer[0] = MPU6050_GYRO_XOUT_H;
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 1;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 6;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    gyro_x =
        ((uint16_t)(mpu6050Buffer[0] << 8)) |
        ((uint16_t)(mpu6050Buffer[1]));

    gyro_y =
        ((uint16_t)(mpu6050Buffer[2] << 8)) |
        ((uint16_t)(mpu6050Buffer[3]));

    gyro_z =
        ((uint16_t)(mpu6050Buffer[4] << 8)) |
        ((uint16_t)(mpu6050Buffer[5]));

    rot_x = gyro_x / LSB_Sensitivity_250;
    rot_y = gyro_y / LSB_Sensitivity_250;
    rot_z = gyro_z / LSB_Sensitivity_250;

    return MPU6050_SUCESS;
}

/**
  * @brief  Read temperature.
  * @param  None
  * @retval Success or Fail
  */
uint8_t uMPU6050ReadTemperature()
{
    portBASE_TYPE xStatus;

    // Read accelerometer
    mpu6050Buffer[0] = MPU6050_TEMP_OUT_H;
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 1;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 2;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    int16_t raw_temp =
        ((uint16_t)(mpu6050Buffer[0] << 8)) |
        ((uint16_t)(mpu6050Buffer[1]));

    temperature = (raw_temp / 340.0) + 36.53;

    return MPU6050_SUCESS;
}
