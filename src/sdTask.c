/*=============================================================================
 * Copyright (c) 2019, SMA Project
 * Leandro Torrent <leandrotorrent92@gmail.com>
 * Miguel del Valle <m.e.delvallecamino@ieee.org>
 * All rights reserved.
 * License: bsd-3-clause (see LICENSE.txt)
 * Date: 2019/07/27
 * Version: 1.0
 *===========================================================================*/
/* Date: 2019-09-02 */

/*=====[Inclusions of function dependencies]=================================*/
#include "ff.h"
#include "fssdc.h"
#include "sma.h"
/*=====[Definition macros of private constants]==============================*/

/*=====[Private function-like macros]========================================*/

/*=====[Definitions of private data types]===================================*/

/*=====[Definitions of external public global variables]=====================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/
static char buf[6];
static FATFS fs;      // <-- FatFs work area needed for each volume
static FIL fp;        // <-- File object needed for each open file

/*=====[Prototypes (declarations) of private functions]======================*/

/*=====[Implementations of public functions]=================================*/

void sdTask( void *pvParameters )
{
   spiConfig( SPI0 );

   /* Declare the variable that will hold the values received from the queue. */
   real32_t lReceivedValue;
   BaseType_t xStatus;
   const TickType_t xTicksToWait = pdMS_TO_TICKS( 2500UL );
   
   uint8_t i = 0;
   int n = 0;
   int nbytes = 0;

   FSSDC_InitSPI ();
   if( f_mount( &fs, "SDC:", 0 ) != FR_OK ){
      while (1) {
         gpioToggle( LED1 );
         uartWriteString( UART_USB, "SD no disponible\r\n" );
         vTaskDelay( 1000 / portTICK_RATE_MS );
      }
   }

   /* This task is also defined within an infinite loop. */
   for( ;; ) {
      /* As this task unblocks immediately that data is written to the queue this
      call should always find the queue empty. */
      if( uxQueueMessagesWaiting( xQueueSd ) != 0 ) {
         vPrintString( "Queue should have been empty!\r\n" );
      }

      xStatus = xQueueReceive( xQueueSd, &lReceivedValue, xTicksToWait );

      if( xStatus == pdPASS ) {
         /* Data was successfully received from the queue, print out the received
         value. */
         //vPrintStringAndNumber( "Received = ", lReceivedValue );

    	  uartWriteString( UART_USB, "Temperatura: " );
    	  floatToString( lReceivedValue, buf, 2 );
    	  uartWriteString( UART_USB, buf);
    	  uartWriteString( UART_USB, " grados C\r\n" );
 		/* -------save in sd------------------------------------------------------*/
   	if( f_open( &fp, "SDC:/log.txt", FA_WRITE | FA_OPEN_APPEND ) == FR_OK ) {
         n = 6;
         f_write( &fp,"temperature\r",12, &nbytes );               
         f_write( &fp, buf, n, &nbytes );
         n = 7;
         f_write( &fp,"grados\r\n",n, &nbytes );  
         f_close(&fp);
         if( nbytes == n ){
            uartWriteString( UART_USB, "Escribio correctamente\r\n ");
            gpioWrite( LEDG, ON );
         } else {
            gpioWrite( LEDR, ON );
            uartWriteString( UART_USB, "Error al escribir\r\n ");
         }
    } else{
         uartWriteString( UART_USB, "Error al abrir el archivo\r\n" );
         gpioWrite( LEDR, ON );
      }      
		/* ---------fin save in sd-----------------------------------------------*/

      } else {
         /* We did not receive anything from the queue even after waiting*/
         vPrintString( "Could not receive from the queue.\r\n" );
      }
   }
}
