ADS126x STM32H7 HAL Driver

This driver ports the Arduino ADS1262 library to STM32H7 using the same port abstraction style as `LDC1101`.

- Public headers: `Inc/ADS126X.h`, `Inc/ADS126X_port.h`
- Sources: `Src/ADS126X.c`, `Src/ADS126X_port.c`
- CMake target: `ADS126x` (INTERFACE)

Quick usage (example):

```c
#include "ADS126X.h"

extern SPI_HandleTypeDef hspi2; // Configure CPOL=0, CPHA=1 in CubeMX
ADS126X_t ads;

void ads_init_example(void) {
    ADS126X_init_default(&ads, &hspi2,
                         GPIOB, GPIO_PIN_12,   // CS
                         GPIOC, GPIO_PIN_3,    // DRDY (optional)
                         GPIOA, GPIO_PIN_1,    // START (optional)
                         GPIOA, GPIO_PIN_2);   // PWDN (optional)
}

int32_t ads_read_once(void) {
    ADS126X_data_t d;
    if (ADS126X_read_data(&ads, &d) == ADS126X_OK) {
        return d.value;
    }
    return 0;
}
```

Notes:
- SPI must be configured for 8-bit, MSB-first, mode 1 (CPOL=0, CPHA=1).
- GPIO pins must be initialized before calling the driver.
- Data read returns 6 bytes: status, 32-bit sample, CRC.
