#include "sma.h"

void vSenderTask2( void *pvParameters )
{
   real32_t lValueToSend;
   BaseType_t xStatus;

   /* Two instances are created of this task so the value that is sent to the
   queue is passed in via the task parameter rather than be hard coded.  This way
   each instance can use a different value.  Cast the parameter to the required
   type. */

   real32_t dato = 5000;

   lValueToSend = dato;

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
      xStatus = xQueueSendToBack( xQueue, &lValueToSend, 0 );

      if( xStatus != pdPASS ) {
         /* We could not write to the queue because it was full � this must
         be an error as the queue should never contain more than one item! */
         vPrintString( "Could not send to the queue.\r\n" );
      }
      vTaskDelay( ((int32_t )pvParameters) / portTICK_RATE_MS );
   }
}
