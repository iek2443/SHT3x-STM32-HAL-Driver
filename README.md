# SHT3x-STM32-HAL-Driver


A lightweight and well-documented driver for the **Sensirion SHT3x** temperature and humidity sensors (SHT30, SHT31, SHT35) using **STM32 HAL I2C** interface.

---

## ✨ Features

- ✅ Supports SHT30, SHT31, and SHT35
- ✅ Compatible with STM32 HAL (I2C)
- ✅ Temperature & Humidity readout
- ✅ Configurable measurement resolution: Low / Medium / High
- ✅ CRC8 integrity check
- ✅ Clean modular C code with Doxygen comments

---

## 🔧 Requirements

- STM32 HAL library (e.g., `stm32f4xx_hal.h`, `stm32g0xx_hal.h`, etc.)
- I2C communication enabled
- SHT3x sensor connected via I2C (default address: `0x44`)

---

## ⚙️ Configuration

You **must define and assign** the I2C handle used by the sensor.

### In the `.h` file:

```c
#include "stm32f4xx_hal.h"       // Include the correct HAL header for your STM32 series
extern I2C_HandleTypeDef hi2c3;
```
### In the `.c` file:
```c
I2C_HandleTypeDef *port = &hi2c3; // This must match the I2C handle defined in your .h file
```
###🚀 Usage Example
```c
#include "SHT3X_DRIVER.h"

float temperature = 0.0f;
float humidity = 0.0f;

int main(void) {
    HAL_Init();
    MX_I2C3_Init(); // Replace with your I2C init function

    SHT3x_Init();

    while (1) {
        SHT3x_ReadTempAndHumidity(&temperature, &humidity, HIGH_RESOLUTION);
        HAL_Delay(1000);
    }
}
```
