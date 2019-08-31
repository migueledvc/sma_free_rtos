/*=============================================================================
 * Copyright (c) 2019, SMA Project
 * Leandro Torrent <leandrotorrent92@gmail.com>
 * Miguel del Valle <m.e.delvallecamino@ieee.org>
 * All rights reserved.
 * License: bsd-3-clause (see LICENSE.txt)
 * Date: 2019/07/27
 * Version: 1.0
 *===========================================================================*/

/*=====[Avoid multiple inclusion - begin]====================================*/

#ifndef __SMA_H__
#define __SMA_H__

/*=====[Inclusions of public function dependencies]==========================*/
/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sapi.h"


/* Demo includes. */
#include "supporting_functions.h"
#include "sapi.h"
#include "sapi_bme280.h"
#include <stdint.h>
#include <stddef.h>
#include "ff.h"
#include "fssdc.h"

/*=====[C++ - begin]=========================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*=====[Definition macros of public constants]===============================*/
/**\I2C baudrate */
#define BME280_I2C_RATE	100000
/*=====[Public function-like macros]=========================================*/

/*=====[Definitions of public data types]====================================*/
/* Declare a variable of type QueueHandle_t.  This is used to store the queue
that is accessed by all three tasks. */
QueueHandle_t xQueue, xQueueSd;
/*=====[Prototypes (declarations) of public functions]=======================*/
void bme280Task( void *pvParameters );
void am2301Task( void *pvParameters );
void vSenderTask2( void *pvParameters );
void vReceiverTask( void *pvParameters );
void sdTask( void *pvParameters );
/*=====[Prototypes (declarations) of public interrupt functions]=============*/

/*=====[C++ - end]===========================================================*/

#ifdef __cplusplus
}
#endif

/*=====[Avoid multiple inclusion - end]======================================*/

#endif /* __SMA_H__ */
