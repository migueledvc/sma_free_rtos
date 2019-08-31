# sma_free_rtos
El proyecto implementa un primer prototipo de 
Sistema Monitor Ambiental para salas de equipos críticos.

Características:

	Sistema operativo: FreeRTOS
	Hardware: EDU-CIAA
	Sensores: BME-280 y AM2301
	Salidas: UART y SD
	 
El sistema crea las siguientes tareas:

1) bme280Task: Inicia y se ejecuta cada 2750 ms. Envía a dos colas de mensajes.
2) am2301Task: Inicia y se ejecuta cada 1000 ms.
3) vSenderTask2: Utilizada para desarrollo y testing. A futuro incorpora sensor de humo.
4) sdTask: Utilizada para recibir de la cola xQueueSd.
5) vReceiverTask: Utilizada para recibir de diferentes fuentes y enviar a la UART.

Se le da menor prioridad a las tareas que reciben.




