/*---------------------------------------------------------------------------/
/  APPConfig - Application 	Ver 1.00
/---------------------------------------------------------------------------*/

#ifndef _APP_CONFIG_H
#define _APP_CONFIG_H

#define _APPConfig 100	/* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations
/---------------------------------------------------------------------------*/


#define DEBUG	  1
/* This option Enables the debug interface. */

#define	MICRO_BUFFER_SIZE			32
#define	MINI_BUFFER_SIZE			64
#define	TINY_BUFFER_SIZE			128
#define	SMALL_BUFFER_SIZE			256
#define	MEDIUM_BUFFER_SIZE			512
#define	BIG_BUFFER_SIZE				1024
#define	LARGE_BUFFER_SIZE			2048
#define	MASSIVE_BUFFER_SIZE			4096
#define	ENORMUS_BUFFER_SIZE			8192
/* Those options specify the size of the buffers used between different
/  interfaces. */

#define  osPriorityIdle             configMAX_PRIORITIES - 6
#define  osPriorityLow              configMAX_PRIORITIES - 5
#define  osPriorityBelowNormal      configMAX_PRIORITIES - 4
#define  osPriorityNormal       	configMAX_PRIORITIES - 3
#define  osPriorityAboveNormal      configMAX_PRIORITIES - 2
#define  osPriorityHigh             configMAX_PRIORITIES - 1
#define  osPriorityRealtime         configMAX_PRIORITIES - 0

#endif
