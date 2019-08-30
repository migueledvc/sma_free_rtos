/*=============================================================================
 * Copyright (c) 2019, SMA Project
 * Leandro Torrent <leandrotorrent92@gmail.com>
 * Miguel del Valle <m.e.delvallecamino@ieee.org>
 * All rights reserved.
 * License: bsd-3-clause (see LICENSE.txt)
 * Date: 2019/07/27
 * Version: 1.0
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Demo includes. */
#include "supporting_functions.h"
#include "sapi.h"
#include "sma.h"

/*-----------------------------------------------------------*/
int main( void )
{
   /* The queue is created to hold a maximum of 5 long values. */
   xQueue = xQueueCreate( 5, sizeof( real32_t ) );

   boardConfig(); // Inicializar y configurar la plataforma
   uartConfig( UART_USB, 115200 ); // Inicializar periferico UART_USB
   i2cInit(I2C0, BME280_I2C_RATE);

   if( xQueue != NULL ) {

	   xTaskCreate( bme280Task, (const char *) "bme280",
			   configMINIMAL_STACK_SIZE * 2, ( void * ) 2750,
				  tskIDLE_PRIORITY + 2, NULL );

	  xTaskCreate( am2301Task, (const char *) "am2301",
    		  configMINIMAL_STACK_SIZE * 2, ( void * ) 1000,
			  tskIDLE_PRIORITY + 2, NULL );

      xTaskCreate( vSenderTask2, (const char *) "Sender2",
    		  configMINIMAL_STACK_SIZE * 2, ( void * ) 5000,
			  tskIDLE_PRIORITY + 2, NULL );

      /* Create the task that will read from the queue.  The task is created with
      priority 2, so above the priority of the sender tasks. */
      xTaskCreate( vReceiverTask,(const char *) "Receiver",
    		  configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 3, NULL );

      /* Start the scheduler so the created tasks start executing. */
      vTaskStartScheduler();

   } else {
      /* The queue could not be created. */
   }

   /* The following line should never be reached because vTaskStartScheduler()
   will only return if there was not enough FreeRTOS heap memory available to
   create the Idle and (if configured) Timer tasks.  Heap management, and
   techniques for trapping heap exhaustion, are described in the book text. */
   for( ;; );
   return 0;
}
