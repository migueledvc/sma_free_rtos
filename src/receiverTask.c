#include "sma.h"

void vReceiverTask( void *pvParameters )
{
   /* Declare the variable that will hold the values received from the queue. */
   real32_t lReceivedValue;
   BaseType_t xStatus;
   const TickType_t xTicksToWait = pdMS_TO_TICKS( 2050UL );
   static char uartBuff[10];

   /* This task is also defined within an infinite loop. */
   for( ;; ) {
      /* As this task unblocks immediately that data is written to the queue this
      call should always find the queue empty. */
      if( uxQueueMessagesWaiting( xQueue ) != 0 ) {
         vPrintString( "Queue should have been empty!\r\n" );
      }

      /* The first parameter is the queue from which data is to be received.  The
      queue is created before the scheduler is started, and therefore before this
      task runs for the first time.

      The second parameter is the buffer into which the received data will be
      placed.  In this case the buffer is simply the address of a variable that
      has the required size to hold the received data.

      the last parameter is the block time � the maximum amount of time that the
      task should remain in the Blocked state to wait for data to be available should
      the queue already be empty. */
      xStatus = xQueueReceive( xQueue, &lReceivedValue, xTicksToWait );

      if( xStatus == pdPASS ) {
         /* Data was successfully received from the queue, print out the received
         value. */
         //vPrintStringAndNumber( "Received = ", lReceivedValue );

    	  uartWriteString( UART_USB, "Temperatura: " );
    	  floatToString( lReceivedValue, uartBuff, 1 );
    	  uartWriteString( UART_USB, uartBuff);
    	  uartWriteString( UART_USB, " grados C\r\n" );


      } else {
         /* We did not receive anything from the queue even after waiting for 100ms.
         This must be an error as the sending tasks are free running and will be
         continuously writing to the queue. */
         vPrintString( "Could not receive from the queue.\r\n" );
      }
   }
}

