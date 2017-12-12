/**********************************************************************
* - Description:		esp32-mpu6050
* - File:				i2c_application.h
* - Compiler:			xtensa-esp32
* - Debugger:			USB2USART
* - Author:				Mohamed El-Sabagh
* - Target:				ESP32
* - Created:			2017-12-11
* - Last changed:		2017-12-11
*
**********************************************************************/
#ifndef _I2C_APPLICATION_H
#define _I2C_APPLICATION_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* Declare a variable of type xQueueHandle.  This is used to store the queue
	that is accessed by any task wants to write to I2C. */
extern QueueHandle_t xQueueI2CWriteBuffer;
extern QueueHandle_t xQueueI2CReadBuffer;

//// Those semaphores are used only for apps who would like to block waiting for end of
//// transmission or reception if not required to use it's ok
extern SemaphoreHandle_t xBinarySemaphoreI2CAppEndOfWrite;
extern SemaphoreHandle_t xBinarySemaphoreI2CAppEndOfRead;

/* Structure to manipulate buffer sent or received over I2C */
typedef struct I2C_Structure
{
	uint8_t* pBuffer;
	uint16_t sDataLength;
	uint8_t slaveAddress;
} I2C_Status;

void vI2CWrite( void *pvParameters );
void vI2CRead( void *pvParameters );


#endif
