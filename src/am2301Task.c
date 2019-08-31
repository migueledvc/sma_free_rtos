#include "sma.h"

void am2301Task( void *pvParameters )
{
   real32_t lValueToSend;
   BaseType_t xStatus;

   dht11Init( GPIO1 ); // Inicializo el sensor DHT11

   real32_t humidity = 0, temperature = 0;

   /* As per most tasks, this task is implemented within an infinite loop. */
   for( ;; ) {
      /* The first parameter is the queue to which data is being sent.  The
      queue was created before the scheduler was started, so before this task
      started to execute.

      The second parameter is the address of the data to be sent.

      The third parameter is the Block time � the time the task should be kept
      in the Blocked state to wait for space to become available on the queue
      should the queue already be full.  In this case we don�t specify a block
      time because there should always be space in the queue. */
	   if( dht11Read(&humidity, &temperature) ) {
	   	   //gpioWrite( LEDG, ON );
	   	   //gpioWrite( LEDR, OFF );

	      lValueToSend = temperature;

	      xStatus = xQueueSendToBack( xQueue, &lValueToSend, 0 );

	      if( xStatus != pdPASS ) {
	    	  /* We could not write to the queue because it was full � this must
         	 be an error as the queue should never contain more than one item! */
	    	  vPrintString( "Could not send to the queue.\r\n" );
	      }
	   }
	   else {
    	  //gpioWrite( LEDG, OFF );
    	  //gpioWrite( LEDR, ON );
	   }
	   vTaskDelay( 1000 / portTICK_RATE_MS );
   }
}
