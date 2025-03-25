/**
 * @file    SHT3X_DRIVER.h
 * @brief   SHT3x Temperature and Humidity Sensor Driver (I2C)
 *
 * @details This driver allows initialization, reading temperature and humidity data,
 *          and managing measurement resolution for the SHT3x sensor family using I2C interface.
 *
 * @note    Uses I2C3 by default (hi2c3). Make sure the port is properly initialized.
 *
 * @date    December 19, 2024
 * @author  iek
 */

#ifndef INC_SHT3X_DRIVER_H_
#define INC_SHT3X_DRIVER_H_

#include <stdint.h>
/**
 * @brief  HAL library include for STM32 platform.
 *
 * @note   The user must include the correct HAL header file based on their MCU family.
 *         For example:
 *         @code
 *         #include "stm32g0xx_hal.h"   // For STM32G0 series
 *         #include "stm32f4xx_hal.h"   // For STM32F4 series
 *         #include "stm32l4xx_hal.h"   // For STM32L4 series
 *         @endcode
 *         Make sure to match this include with your STM32 device.
 */
#include "stm32f4xx_hal.h"
/**
 * @brief  I2C handle used for communication with the SHT3x sensor.
 *
 * @note   The user must define the I2C handle according to the hardware setup.
 *         For example, if I2C1 is used:
 *         @code
 *         extern I2C_HandleTypeDef hi2c1;
 *         @endcode
 *         Replace 'hi2c3' with the appropriate I2C handle name.
 */
extern I2C_HandleTypeDef hi2c3;


#define SHT3X_I2C_ADDR  0x44			/**< Default I2C address for SHT3x */
#define SHT3X_CMD_SOFT_RESET  0x30A2	/**< Soft reset command */

#define CRC8_POLYNOMIAL  0x31 			/**< CRC-8 polynomial used for data check */
#define CRC8_INIT        0xFF			/**< Initial value for CRC computation */

/**
 * @brief Measurement resolution options for SHT3x.
 */
typedef enum {
	LOW_RESOLUTION = 0x2416, 			/**< Low repeatability */
	MEDIUM_RESOLUTION = 0x240B, 		/**< Medium repeatability */
	HIGH_RESOLUTION = 0x2400 			/**< High repeatability */
} resolution_t;

/* ======================== FUNCTION PROTOTYPES ======================== */
static uint8_t CalculateCRC(uint8_t *data, uint8_t length);
void SHT3x_Init(void);
void SHT3x_ReadTempAndHumidity(float *temperature, float *humidity,
		resolution_t resolution);

#endif /* INC_SHT3X_DRIVER_H_ */
