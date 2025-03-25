/**
 * @file    SHT3X_DRIVER.c
 * @brief   Source file for SHT3x Temperature and Humidity Sensor driver
 * @date    December 19, 2024
 * @author  iek
 */


#include "SHT3X_DRIVER.h"

/**
 * @brief  Pointer to the I2C port used for SHT3x communication.
 *
 * @note   This should match the I2C handle defined in the header file.
 *         For example, if you defined `extern I2C_HandleTypeDef hi2c3;` in the header,
 *         then this should be:
 *         @code
 *         I2C_HandleTypeDef *port = &hi2c3;
 *         @endcode
 *         Change to match the I2C peripheral you are using.
 */
I2C_HandleTypeDef *port = &hi2c3;

/**
 * @brief  Calculates CRC8 checksum for 2-byte data using SHT3x's polynomial.
 *
 * @param  data: Pointer to the data buffer.
 * @param  length: Number of bytes to calculate CRC over.
 *
 * @retval The calculated CRC8 checksum.
 *
 * @note   Used internally for verifying data integrity received from the sensor.
 */
static uint8_t CalculateCRC(uint8_t *data, uint8_t length) {
    uint8_t crc = CRC8_INIT;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}


/**
 * @brief  Initializes the SHT3x sensor by sending a soft reset command.
 *
 * @note   After reset, sensor needs some delay (~20 ms) before ready to use.
 *
 * @retval None
 *
 * @code
 * SHT3x_Init();
 * @endcode
 */
void SHT3x_Init(void) {
    uint8_t cmd[2] = {SHT3X_CMD_SOFT_RESET >> 8, SHT3X_CMD_SOFT_RESET & 0xFF};
    HAL_I2C_Master_Transmit(port, SHT3X_I2C_ADDR << 1, cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(20);
}




/**
 * @brief  Reads temperature and humidity values from the SHT3x sensor.
 *
 * @param[out] temperature: Pointer to float that will hold temperature in °C.
 * @param[out] humidity: Pointer to float that will hold relative humidity in %.
 * @param[in]  resolution: Measurement resolution (HIGH_RESOLUTION, MEDIUM_RESOLUTION, LOW_RESOLUTION).
 *
 * @retval None
 *
 * @note
 *  - Performs I2C communication to request measurement.
 *  - Waits ~30ms before reading result.
 *  - Checks CRC to ensure valid data.
 *
 * @code
 * float temp, hum;
 * SHT3x_ReadTempAndHumidity(&temp, &hum, HIGH_RESOLUTION);
 * @endcode
 */
void SHT3x_ReadTempAndHumidity(float *temperature, float *humidity, resolution_t resolution) {
    uint8_t cmd[2] = {resolution >> 8, resolution & 0xFF};
    uint8_t data[6];

    HAL_I2C_Master_Transmit(port, SHT3X_I2C_ADDR << 1, cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(30);
    HAL_I2C_Master_Receive(port, SHT3X_I2C_ADDR << 1, data, 6, HAL_MAX_DELAY);

    if (CalculateCRC(&data[0], 2) != data[2] || CalculateCRC(&data[3], 2) != data[5]) {
        return;
    }

    uint16_t rawTemp = (data[0] << 8) | data[1];
    uint16_t rawHum  = (data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * ((float)rawTemp / 65535.0f);
    *humidity    = 100.0f * ((float)rawHum / 65535.0f);
}





