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
#include "sma.h"
#include "sapi_bme280.h" // tested on GY-BME/P280
#include "sapi.h"		// <= sAPI header

void bme280Task(void *pvParameters) {
	real32_t lValueToSend;
	BaseType_t xStatus, xStatusSd;

	struct bme280_dev dev;
	int8_t rslt = BME280_OK;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = bme280I2cRead;
	dev.write = bme280I2cWrite;
	dev.delay_ms = bme280Delay_ms;

	// ----- bme sensor init -----------------------------------
	rslt = bme280Init(&dev);
	if (rslt != BME280_OK) {
		gpioWrite(LED3, ON);
		vTaskDelay(500 / portTICK_RATE_MS);
		gpioWrite(LED3, OFF);
	} else {
		gpioWrite(LED3, ON);
		vTaskDelay(2000 / portTICK_RATE_MS);
		gpioWrite(LED3, OFF);
	}

	// ----- bme280 sensor setup---------------------------------
	/* Recommended mode operation: Indoor navigation */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_1X;
	dev.settings.osr_t = BME280_OVERSAMPLING_16X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;
	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;

	rslt = bme280SetSensorSettings(settings_sel, &dev);
	rslt = bme280SetSensorMode(BME280_NORMAL_MODE, &dev);

	//real32_t dato = 5000;


	/* As per most tasks, this task is implemented within an infinite loop. */
	for (;;) {
		dev.delay_ms(2750);

		rslt = bme280GetSensorData(BME280_ALL, &comp_data, &dev);

		bme280PrintSensorData(&comp_data);

		lValueToSend = comp_data.temperature;

		gpioToggle(LED3);

		xStatus = xQueueSendToBack(xQueue, &lValueToSend, 0);
		xStatusSd = xQueueSendToBack(xQueueSd, &lValueToSend, 0);

		if (xStatus != pdPASS) {
			/* We could not write to the queue because it was full � this must
			 be an error as the queue should never contain more than one item! */
			vPrintString("Could not send to the queue.\r\n");
		}
		if (xStatusSd != pdPASS) {
			/* We could not write to the queue because it was full � this must
			 be an error as the queue should never contain more than one item! */
			vPrintString("Could not send to the queue.\r\n");
		}

	}
}
