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
#include "kalman_filter.h"
#include <math.h>
#include "esp_log.h"

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
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A

double a_x = 0.0;
double a_y = 0.0;
double a_z = 0.0;

int16_t accelration_x;
int16_t accelration_y;
int16_t accelration_z;

double g_x = 0.0;
double g_y = 0.0;
double g_z = 0.0;

int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;

int16_t cal_a_x = 0.0;
int16_t cal_a_y = 0.0;
int16_t cal_a_z = 0.0;

double angle_pitch = 0.0;
double angle_roll = 0.0;

double temperature = 0.0;

double senistivity_a_x = 0.0;
double senistivity_a_y = 0.0;
double senistivity_a_z = 0.0;

static I2C_Status i2cStatus;
static uint8_t mpu6050Buffer[MICRO_BUFFER_SIZE];

#define MPU6050_SUCESS					 	       0
#define MPU6050_FAILURE						       1
#define ACCELEROMETER_READING_FREQUENCY_MS	     REFRESH_RATE * 1000	// 5ms second

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

#define THRESHOLD               20
#define THRESHOLD_NEG           (-1 * THRESHOLD)

#define NUMBER_READINGS_CALIBRATION                     1000
#define NUMBER_READINGS_CALIBRATION_DISCARDED           100
#define POSITIONING_DELAY                               5000

uint8_t calibration_mode = CALIBRATION_MODE_ACCURATE;

static uint8_t uMPU6050Init();
static uint8_t uMPU6050ReadAccelerometer();
static uint8_t uMPU6050ReadGyroscope();
static uint8_t uMPU6050ReadTemperature();
static void vMPU6050CalculateAngles();
static uint8_t uGetDirection();
static void uMPU6050Calibration();

static angle_calculations pitch_angle_calculations = {
    .angle = 0.0,
    .gyro_y = 0.0,
    .q_bias = 0.0,
    .angle_err = 0.0,
    .q_angle = 0.1,
    .q_gyro = 0.1,
    .r_angle = 0.5,
    .dt = REFRESH_RATE,
    .c_0 = 1,
    .pct_0 = 0.0,
    .pct_1 = 0.0,
    .e = 0.0,
    .k_0 = 0,
    .k_1 = 0,
    .t_0 = 0,
    .t_1 = 0,
    .pdot = {0, 0, 0, 0},
    .pp = {{1, 0}, {0, 1}}
};
static angle_calculations roll_angle_calculations = {
    .angle = 0.0,
    .gyro_y = 0.0,
    .q_bias = 0.0,
    .angle_err = 0.0,
    .q_angle = 0.1,
    .q_gyro = 0.1,
    .r_angle = 0.5,
    .dt = REFRESH_RATE,
    .c_0 = 1,
    .pct_0 = 0.0,
    .pct_1 = 0.0,
    .e = 0.0,
    .k_0 = 0,
    .k_1 = 0,
    .t_0 = 0,
    .t_1 = 0,
    .pdot = {0, 0, 0, 0},
    .pp = {{1, 0}, {0, 1}}
};

/**
  * @brief  MPU6050 task.
  * @param  None
  * @retval None
  */
void vMPU6050Task( void *pvParameters )
{
    uint16_t count = 0;

    if (uMPU6050Init() != MPU6050_SUCESS) {
        vTaskDelete(NULL);
        return;
    }

    uMPU6050Calibration();

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

        vMPU6050CalculateAngles();
        count++;
        if (count >= 20)
        {
            ESP_LOGI("mpu6050", "Acc: ( %.3f, %.3f, %.3f)", a_x, a_y, a_z);
            ESP_LOGI("mpu6050", "Gyro: ( %.3f, %.3f, %.3f)", g_x, g_y, g_z);
            ESP_LOGI("mpu6050", "FPitch: %.3f", angle_pitch);
            ESP_LOGI("mpu6050", "FRoll: %.3f", angle_roll);
            ESP_LOGI("mpu6050", "Temperature: %.3f", temperature);

            uGetDirection();

            count = 0;
        }

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

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

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

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    // Configure gyroscope
    mpu6050Buffer[0] = MPU6050_GYRO_CONFIG;
    mpu6050Buffer[1] = _2000_DEGREE;        // +/-2000degree
    // Write data to i2c
    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 2;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CWriteBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

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

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 6;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of reading process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfRead, portMAX_DELAY );

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

    accelration_x += cal_a_x;
    accelration_y += cal_a_y;
    accelration_z += cal_a_z;

    a_x = -accelration_x / senistivity_a_x;
    a_y = -accelration_y / senistivity_a_y;
    a_z = -accelration_z / senistivity_a_z;

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

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 6;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of reading process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfRead, portMAX_DELAY );

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

    g_x = gyro_x / LSB_Sensitivity_2000;
    g_y = gyro_y / LSB_Sensitivity_2000;
    g_z = gyro_z / LSB_Sensitivity_2000;

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

    // Wait for end of writing process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfWrite, portMAX_DELAY );

    if (xStatus == pdFAIL)
	{
		return MPU6050_FAILURE;
	}

    i2cStatus.pBuffer = mpu6050Buffer;
    i2cStatus.sDataLength = 2;
    i2cStatus.slaveAddress = MPU6050_SLAVE_ADDR;

    xStatus = xQueueSend( xQueueI2CReadBuffer, (void *)&i2cStatus, portMAX_DELAY );

    // Wait for end of reading process
    xStatus = xSemaphoreTake( xBinarySemaphoreI2CAppEndOfRead, portMAX_DELAY );

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

/**
  * @brief  Calculate rolling and pitch angles.
  * @param  None
  * @retval None
  */
void vMPU6050CalculateAngles()
{
    double pitch = atan(a_x / a_z)*57.2958;        // convert the radian to degrees
    double roll = atan(a_y / a_z)*57.2958;         // convert the radian to degrees
    angle_pitch = filter(pitch, g_y, &pitch_angle_calculations);
    angle_roll = filter(roll, -g_x, &roll_angle_calculations);
}

/**
  * @brief  Get Direction of the MPU 6050, whether it's faced upside down or right position.
  * @param  None
  * @retval 0 not flipped, 1 rotated, 2 180 degree flipped
  */
uint8_t uGetDirection()
{
    uint8_t status = 0;     // 0 not flipped, 1 rotated, 2 180 degree flipped
    if (angle_pitch > THRESHOLD || angle_pitch < THRESHOLD_NEG || angle_roll > THRESHOLD || angle_roll < THRESHOLD_NEG)
    {
        status = 1;
    }
    else
    {
        // angles of pitch and rotation are almost zero, check whether flipped 180 degree or 0 degree
        if (a_z > 0)
        {
            // flipped 180 degree
            status = 2;
        }
    }

    if (status == 0)
    {
        ESP_LOGI("mpu6050", "MPU6050 is right side");
    }
    else if (status == 1)
    {
        ESP_LOGI("mpu6050", "MPU6050 is rotated");
    }
    else if (status == 2)
    {
        ESP_LOGI("mpu6050", "MPU6050 is 180 degree flipped");
    }

    return status;
}

/**
  * @brief  Calibrate MPU6050.
  * @param  None
  * @retval None
  */
void uMPU6050Calibration()
{
    uint16_t i = 0;
    // accelerometer mean readings
    int32_t total_ax_mean = 0;
    int32_t total_ay_mean = 0;
    int32_t total_az_mean = 0;

    /* Portrait up */
    // accelerometer
    int16_t p_u_ax = 0;
    int16_t p_u_ay = 0;
    int16_t p_u_az = 0;

    /* Landscape left */
    // accelerometer
    int16_t l_l_ax = 0;
    int16_t l_l_ay = 0;
    int16_t l_l_az = 0;

    /* Portrait down */
    // accelerometer
    int16_t p_d_ax = 0;
    int16_t p_d_ay = 0;
    int16_t p_d_az = 0;

    /* Landscape right */
    // accelerometer
    int16_t l_r_ax = 0;
    int16_t l_r_ay = 0;
    int16_t l_r_az = 0;

    /* Front */
    // accelerometer
    int16_t f_ax = 0;
    int16_t f_ay = 0;
    int16_t f_az = 0;

    /* Back */
    // accelerometer
    int16_t b_ax = 0;
    int16_t b_ay = 0;
    int16_t b_az = 0;

    cal_a_x = 0;
    cal_a_y = 0;
    cal_a_z = 0;

    senistivity_a_x = LSB_Sensitivity_2G;
    senistivity_a_y = LSB_Sensitivity_2G;
    senistivity_a_z = LSB_Sensitivity_2G;

    if (calibration_mode == CALIBRATION_MODE_OFF) {
        return;
    } else if (calibration_mode == CALIBRATION_MODE_SIMPLE) {
        ESP_LOGI("mpu6050", "Place the accelerometer on it's front end and wait util calibration is done");
        vTaskDelay( POSITIONING_DELAY / portTICK_RATE_MS );

        for (i = 0; i < NUMBER_READINGS_CALIBRATION + NUMBER_READINGS_CALIBRATION_DISCARDED; i++) {
            // discard first few readings
            if (i < NUMBER_READINGS_CALIBRATION_DISCARDED) {
                continue;
            }

            if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
                break;
            }

            total_ax_mean += accelration_x;
            total_ay_mean += accelration_y;
            total_az_mean += accelration_z;

            vTaskDelay( 20 / portTICK_RATE_MS );
        }

        total_ax_mean /= NUMBER_READINGS_CALIBRATION;
        total_ay_mean /= NUMBER_READINGS_CALIBRATION;
        total_az_mean /= NUMBER_READINGS_CALIBRATION;

        ESP_LOGI("mpu6050", "Mean Acc: ( %.3d, %.3d, %.3d)", total_ax_mean, total_ay_mean, total_az_mean);

        cal_a_x = -1 * total_ax_mean;
        cal_a_y = -1 * total_ay_mean;
        cal_a_z = LSB_Sensitivity_2G - total_az_mean;
    } else if (calibration_mode == CALIBRATION_MODE_ACCURATE) {
        /************ Front Side  ************/
        ESP_LOGI("mpu6050", "Place the accelerometer on it's front end and donot touch it");
        vTaskDelay( POSITIONING_DELAY / portTICK_RATE_MS );

        for (i = 0; i < NUMBER_READINGS_CALIBRATION + NUMBER_READINGS_CALIBRATION_DISCARDED; i++) {
            // discard first few readings
            if (i < NUMBER_READINGS_CALIBRATION_DISCARDED) {
                continue;
            }

            if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
                break;
            }

            total_ax_mean += accelration_x;
            total_ay_mean += accelration_y;
            total_az_mean += accelration_z;

            vTaskDelay( 20 / portTICK_RATE_MS );
        }

        total_ax_mean /= NUMBER_READINGS_CALIBRATION;
        total_ay_mean /= NUMBER_READINGS_CALIBRATION;
        total_az_mean /= NUMBER_READINGS_CALIBRATION;

        f_ax = total_ax_mean;
        f_ay = total_ay_mean;
        f_az = total_az_mean;

        /************ Back Side  ************/
        ESP_LOGI("mpu6050", "Place the accelerometer on it's back end and donot touch it");
        vTaskDelay( POSITIONING_DELAY / portTICK_RATE_MS );

        for (i = 0; i < NUMBER_READINGS_CALIBRATION + NUMBER_READINGS_CALIBRATION_DISCARDED; i++) {
            // discard first few readings
            if (i < NUMBER_READINGS_CALIBRATION_DISCARDED) {
                continue;
            }

            if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
                break;
            }

            total_ax_mean += accelration_x;
            total_ay_mean += accelration_y;
            total_az_mean += accelration_z;

            vTaskDelay( 20 / portTICK_RATE_MS );
        }

        total_ax_mean /= NUMBER_READINGS_CALIBRATION;
        total_ay_mean /= NUMBER_READINGS_CALIBRATION;
        total_az_mean /= NUMBER_READINGS_CALIBRATION;

        b_ax = total_ax_mean;
        b_ay = total_ay_mean;
        b_az = total_az_mean;

        /************ Portrait Up Side  ************/
        ESP_LOGI("mpu6050", "Place the accelerometer on it's portrait up side and donot touch it");
        vTaskDelay( POSITIONING_DELAY / portTICK_RATE_MS );

        for (i = 0; i < NUMBER_READINGS_CALIBRATION + NUMBER_READINGS_CALIBRATION_DISCARDED; i++) {
            // discard first few readings
            if (i < NUMBER_READINGS_CALIBRATION_DISCARDED) {
                continue;
            }

            if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
                break;
            }

            total_ax_mean += accelration_x;
            total_ay_mean += accelration_y;
            total_az_mean += accelration_z;

            vTaskDelay( 20 / portTICK_RATE_MS );
        }

        total_ax_mean /= NUMBER_READINGS_CALIBRATION;
        total_ay_mean /= NUMBER_READINGS_CALIBRATION;
        total_az_mean /= NUMBER_READINGS_CALIBRATION;

        p_u_ax = total_ax_mean;
        p_u_ay = total_ay_mean;
        p_u_az = total_az_mean;

        /************ Portrait Down Side  ************/
        ESP_LOGI("mpu6050", "Place the accelerometer on it's portrait down side and donot touch it");
        vTaskDelay( POSITIONING_DELAY / portTICK_RATE_MS );

        for (i = 0; i < NUMBER_READINGS_CALIBRATION + NUMBER_READINGS_CALIBRATION_DISCARDED; i++) {
            // discard first few readings
            if (i < NUMBER_READINGS_CALIBRATION_DISCARDED) {
                continue;
            }

            if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
                break;
            }

            total_ax_mean += accelration_x;
            total_ay_mean += accelration_y;
            total_az_mean += accelration_z;

            vTaskDelay( 20 / portTICK_RATE_MS );
        }

        total_ax_mean /= NUMBER_READINGS_CALIBRATION;
        total_ay_mean /= NUMBER_READINGS_CALIBRATION;
        total_az_mean /= NUMBER_READINGS_CALIBRATION;

        p_d_ax = total_ax_mean;
        p_d_ay = total_ay_mean;
        p_d_az = total_az_mean;

        /************ Landscape Left Side  ************/
        ESP_LOGI("mpu6050", "Place the accelerometer on it's landscape left side and donot touch it");
        vTaskDelay( POSITIONING_DELAY / portTICK_RATE_MS );

        for (i = 0; i < NUMBER_READINGS_CALIBRATION + NUMBER_READINGS_CALIBRATION_DISCARDED; i++) {
            // discard first few readings
            if (i < NUMBER_READINGS_CALIBRATION_DISCARDED) {
                continue;
            }

            if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
                break;
            }

            total_ax_mean += accelration_x;
            total_ay_mean += accelration_y;
            total_az_mean += accelration_z;

            vTaskDelay( 20 / portTICK_RATE_MS );
        }

        total_ax_mean /= NUMBER_READINGS_CALIBRATION;
        total_ay_mean /= NUMBER_READINGS_CALIBRATION;
        total_az_mean /= NUMBER_READINGS_CALIBRATION;

        l_l_ax = total_ax_mean;
        l_l_ay = total_ay_mean;
        l_l_az = total_az_mean;

        /************ Landscape Right Side  ************/
        ESP_LOGI("mpu6050", "Place the accelerometer on it's landscape right side and donot touch it");
        vTaskDelay( POSITIONING_DELAY / portTICK_RATE_MS );

        for (i = 0; i < NUMBER_READINGS_CALIBRATION + NUMBER_READINGS_CALIBRATION_DISCARDED; i++) {
            // discard first few readings
            if (i < NUMBER_READINGS_CALIBRATION_DISCARDED) {
                continue;
            }

            if (uMPU6050ReadAccelerometer() != MPU6050_SUCESS) {
                break;
            }

            total_ax_mean += accelration_x;
            total_ay_mean += accelration_y;
            total_az_mean += accelration_z;

            vTaskDelay( 20 / portTICK_RATE_MS );
        }

        total_ax_mean /= NUMBER_READINGS_CALIBRATION;
        total_ay_mean /= NUMBER_READINGS_CALIBRATION;
        total_az_mean /= NUMBER_READINGS_CALIBRATION;

        l_r_ax = total_ax_mean;
        l_r_ay = total_ay_mean;
        l_r_az = total_az_mean;

        // Readings are ready

        senistivity_a_x = (l_r_ax - l_l_ax) / 2;
        senistivity_a_y = (p_u_ay - p_d_ay) / 2;
        senistivity_a_z = (f_az - b_az) / 2;

        cal_a_x = -1 * (f_ax + b_ax + p_u_ax + p_d_ax) / 4;
        cal_a_y = -1 * (f_ay + b_ay + l_l_ay + l_r_ay) / 4;
        cal_a_z = -1 * (l_l_az + l_r_az + p_u_az + p_d_az) / 4;
    }


    ESP_LOGI("mpu6050", "Calibration is done");
    ESP_LOGI("mpu6050", "Cal Acc: ( %.3d, %.3d, %.3d)", cal_a_x, cal_a_y, cal_a_z);
}
