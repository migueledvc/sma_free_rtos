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

#ifndef _SAPI_BME280_H_
#define _SAPI_BME280_H_

/*==================[inclusions]=============================================*/
#include "sma.h"
#include <stdint.h>
#include <stddef.h>
#include "sapi_datatypes.h"
/*==================[c++]====================================================*/
#ifdef __cplusplus
extern "C" {
#endif
/*==================[macros]=================================================*/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
#endif

/**@}*/
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *) 0)
#endif
#endif

/********************************************************/

#ifndef BME280_FLOAT_ENABLE
#define BME280_FLOAT_ENABLE
#endif

#ifndef BME280_FLOAT_ENABLE
#ifndef BME280_64BIT_ENABLE
#define BME280_64BIT_ENABLE
#endif
#endif

#ifndef TRUE
#define TRUE                              UINT8_C(1)
#endif
#ifndef FALSE
#define FALSE                             UINT8_C(0)
#endif

/**\name I2C addresses */
#define BME280_I2C_ADDR_PRIM              UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC               UINT8_C(0x77)

/**\name BME280 chip identifier */
#define BME280_CHIP_ID                    UINT8_C(0x60) //tested on GY-BME/P 280 3.3
//#define BME280_CHIP_ID                    UINT8_C(0x58)

/**\name Register Address */
#define BME280_CHIP_ID_ADDR               UINT8_C(0xD0)
#define BME280_RESET_ADDR                 UINT8_C(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR UINT8_C(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR   UINT8_C(0xE1)
#define BME280_PWR_CTRL_ADDR              UINT8_C(0xF4)
#define BME280_CTRL_HUM_ADDR              UINT8_C(0xF2)
#define BME280_CTRL_MEAS_ADDR             UINT8_C(0xF4)
#define BME280_CONFIG_ADDR                UINT8_C(0xF5)
#define BME280_DATA_ADDR                  UINT8_C(0xF7)

/**\name API success code */
#define BME280_OK                         INT8_C(0)

/**\name API error codes */
#define BME280_E_NULL_PTR                 INT8_C(-1)
#define BME280_E_DEV_NOT_FOUND            INT8_C(-2)
#define BME280_E_INVALID_LEN              INT8_C(-3)
#define BME280_E_COMM_FAIL                INT8_C(-4)
#define BME280_E_SLEEP_MODE_FAIL          INT8_C(-5)

/**\name API warning codes */
#define BME280_W_INVALID_OSR_MACRO        INT8_C(1)

/**\name Macros related to size */
#define BME280_TEMP_PRESS_CALIB_DATA_LEN  UINT8_C(26)
#define BME280_HUMIDITY_CALIB_DATA_LEN    UINT8_C(7)
#define BME280_P_T_H_DATA_LEN             UINT8_C(8)

/**\name Sensor power modes */
#define BME280_SLEEP_MODE                 UINT8_C(0x00)
#define BME280_FORCED_MODE                UINT8_C(0x01)
#define BME280_NORMAL_MODE                UINT8_C(0x03)

/**\name Macro to combine two 8 bit data's to form a 16 bit data */
#define BME280_CONCAT_BYTES(msb, lsb)            (((uint16_t)msb << 8) | (uint16_t)lsb)

#define BME280_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     ((data << bitname##_POS) & bitname##_MSK))
#define BME280_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | \
     (data & bitname##_MSK))

#define BME280_GET_BITS(reg_data, bitname)       ((reg_data & (bitname##_MSK)) >> \
                                                  (bitname##_POS))
#define BME280_GET_BITS_POS_0(reg_data, bitname) (reg_data & (bitname##_MSK))

/**\name Macros for bit masking */
#define BME280_SENSOR_MODE_MSK      UINT8_C(0x03)
#define BME280_SENSOR_MODE_POS      UINT8_C(0x00)

#define BME280_CTRL_HUM_MSK         UINT8_C(0x07)
#define BME280_CTRL_HUM_POS         UINT8_C(0x00)

#define BME280_CTRL_PRESS_MSK       UINT8_C(0x1C)
#define BME280_CTRL_PRESS_POS       UINT8_C(0x02)

#define BME280_CTRL_TEMP_MSK        UINT8_C(0xE0)
#define BME280_CTRL_TEMP_POS        UINT8_C(0x05)

#define BME280_FILTER_MSK           UINT8_C(0x1C)
#define BME280_FILTER_POS           UINT8_C(0x02)

#define BME280_STANDBY_MSK          UINT8_C(0xE0)
#define BME280_STANDBY_POS          UINT8_C(0x05)

/**\name Sensor component selection macros
 * These values are internal for API implementation. Don't relate this to
 * data sheet.
 */
#define BME280_PRESS                UINT8_C(1)
#define BME280_TEMP                 UINT8_C(1 << 1)
#define BME280_HUM                  UINT8_C(1 << 2)
#define BME280_ALL                  UINT8_C(0x07)

/**\name Settings selection macros */
#define BME280_OSR_PRESS_SEL        UINT8_C(1)
#define BME280_OSR_TEMP_SEL         UINT8_C(1 << 1)
#define BME280_OSR_HUM_SEL          UINT8_C(1 << 2)
#define BME280_FILTER_SEL           UINT8_C(1 << 3)
#define BME280_STANDBY_SEL          UINT8_C(1 << 4)
#define BME280_ALL_SETTINGS_SEL     UINT8_C(0x1F)

/**\name Oversampling macros */
#define BME280_NO_OVERSAMPLING      UINT8_C(0x00)
#define BME280_OVERSAMPLING_1X      UINT8_C(0x01)
#define BME280_OVERSAMPLING_2X      UINT8_C(0x02)
#define BME280_OVERSAMPLING_4X      UINT8_C(0x03)
#define BME280_OVERSAMPLING_8X      UINT8_C(0x04)
#define BME280_OVERSAMPLING_16X     UINT8_C(0x05)

/**\name Standby duration selection macros */
#define BME280_STANDBY_TIME_0_5_MS    (0x00)
#define BME280_STANDBY_TIME_62_5_MS (0x01)
#define BME280_STANDBY_TIME_125_MS  (0x02)
#define BME280_STANDBY_TIME_250_MS  (0x03)
#define BME280_STANDBY_TIME_500_MS  (0x04)
#define BME280_STANDBY_TIME_1000_MS (0x05)
#define BME280_STANDBY_TIME_10_MS   (0x06)
#define BME280_STANDBY_TIME_20_MS   (0x07)

/**\name Filter coefficient selection macros */
#define BME280_FILTER_COEFF_OFF     (0x00)
#define BME280_FILTER_COEFF_2       (0x01)
#define BME280_FILTER_COEFF_4       (0x02)
#define BME280_FILTER_COEFF_8       (0x03)
#define BME280_FILTER_COEFF_16      (0x04)

/*!
 * @brief Interface selection Enums
 */
enum bme280_intf {
	/*! SPI interface */
	BME280_SPI_INTF,

	/*! I2C interface */
	BME280_I2C_INTF
};

/*!
 * @brief Type definitions
 */
typedef int8_t (*bme280_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr,
		uint8_t *data, uint16_t len);
typedef void (*bme280_delay_fptr_t)(uint32_t period);

/*!
 * @brief Calibration data
 */
struct bme280_calib_data {
	/**
	 * @ Trim Variables
	 */

	/**@{*/
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
	int32_t t_fine;

	/**@}*/
};

/*!
 * @brief bme280 sensor structure which comprises of temperature, pressure and
 * humidity data
 */
#ifdef BME280_FLOAT_ENABLE
struct bme280_data {
	/*! Compensated pressure */
	double pressure;

	/*! Compensated temperature */
	double temperature;

	/*! Compensated humidity */
	double humidity;
};
#else
struct bme280_data
{
	/*! Compensated pressure */
	uint32_t pressure;

	/*! Compensated temperature */
	int32_t temperature;

	/*! Compensated humidity */
	uint32_t humidity;
};
#endif /* BME280_USE_FLOATING_POINT */

/*!
 * @brief bme280 sensor structure which comprises of uncompensated temperature,
 * pressure and humidity data
 */
struct bme280_uncomp_data {
	/*! un-compensated pressure */
	uint32_t pressure;

	/*! un-compensated temperature */
	uint32_t temperature;

	/*! un-compensated humidity */
	uint32_t humidity;
};

/*!
 * @brief bme280 sensor settings structure which comprises of mode,
 * oversampling and filter settings.
 */
struct bme280_settings {
	/*! pressure oversampling */
	uint8_t osr_p;

	/*! temperature oversampling */
	uint8_t osr_t;

	/*! humidity oversampling */
	uint8_t osr_h;

	/*! filter coefficient */
	uint8_t filter;

	/*! standby time */
	uint8_t standby_time;
};

/*!
 * @brief bme280 device structure
 */
struct bme280_dev {
	/*! Chip Id */
	uint8_t chip_id;

	/*! Device Id */
	uint8_t dev_id;

	/*! SPI/I2C interface */
	enum bme280_intf intf;

	/*! Read function pointer */
	bme280_com_fptr_t read;

	/*! Write function pointer */
	bme280_com_fptr_t write;

	/*! Delay function pointer */
	bme280_delay_fptr_t delay_ms;

	/*! Trim data */
	struct bme280_calib_data calib_data;

	/*! Sensor settings */
	struct bme280_settings settings;
};

/*==================[USER external functions declaration]=========================*/
/*!
 *  @brief This is the sAPI functions for bme280 .
 *  @param sensor address
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280I2cRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);

int8_t bme280I2cWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);

int8_t bme280SpiRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);

int8_t bme280SpiWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len);

void bme280PrintSensorData(struct bme280_data *comp_data);

void bme280Delay_ms(uint32_t period);

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 *
 *  @param[in,out] dev : Structure instance of bme280_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280Init(struct bme280_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len : No of bytes of data to write..
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280SetRegs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len,
		const struct bme280_dev *dev);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len : No of bytes of data to be read.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280GetRegs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		const struct bme280_dev *dev);

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros         |   Functionality
 * -----------------------|----------------------------------------------
 * BME280_OSR_PRESS_SEL    |   To set pressure oversampling.
 * BME280_OSR_TEMP_SEL     |   To set temperature oversampling.
 * BME280_OSR_HUM_SEL    |   To set humidity oversampling.
 * BME280_FILTER_SEL     |   To set filter setting.
 * BME280_STANDBY_SEL  |   To set standby duration setting.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280SetSensorSettings(uint8_t desired_settings,
		const struct bme280_dev *dev);

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280GetSensorSettings(struct bme280_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 *    sensor_mode           |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280SetSensorMode(uint8_t sensor_mode, const struct bme280_dev *dev);

/*!
 * @brief This API gets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] sensor_mode : Pointer variable to store the power mode.
 *
 *   sensor_mode            |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280GetSensorMode(uint8_t *sensor_mode, const struct bme280_dev *dev);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280SoftReset(const struct bme280_dev *dev);

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BME280_PRESS
 *     2       | BME280_TEMP
 *     4       | BME280_HUM
 *     7       | BME280_ALL
 *
 * @param[out] comp_data : Structure instance of bme280_data.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280GetSensorData(uint8_t sensor_comp, struct bme280_data *comp_data,
		struct bme280_dev *dev);

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 *
 *  @param[in] reg_data     : Contains register data which needs to be parsed
 *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
 *  and humidity data.
 */
void bme280ParseSensorData(const uint8_t *reg_data,
		struct bme280_uncomp_data *uncomp_data);

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected by the
 * user.
 *
 * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
 * humidity.
 * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
 * humidity data.
 * @param[out] comp_data : Contains the compensated pressure and/or temperature
 * and/or humidity data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bme280CompensateData(uint8_t sensor_comp,
		const struct bme280_uncomp_data *uncomp_data,
		struct bme280_data *comp_data, struct bme280_calib_data *calib_data);

/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_BME280_H_ */
