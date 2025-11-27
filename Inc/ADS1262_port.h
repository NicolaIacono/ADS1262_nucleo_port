/*
 * ADS1262_port.h - Hardware abstraction layer interface for ADS1262 driver portability.
 */
#ifndef ADS126X_PORT_H
#define ADS126X_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_task_serial.h" // for __println

#define ADS_VERBOSE

void ads_port_cs_reset(GPIO_TypeDef* cs_port, uint16_t cs_pin);
void ads_port_cs_set(GPIO_TypeDef* cs_port, uint16_t cs_pin);

void ads_port_start_reset(GPIO_TypeDef* cs_port, uint16_t cs_pin);
void ads_port_start_set(GPIO_TypeDef* cs_port, uint16_t cs_pin);

void ads_port_pwdn_reset(GPIO_TypeDef* cs_port, uint16_t cs_pin);
void ads_port_pwdn_set(GPIO_TypeDef* cs_port, uint16_t cs_pin);

HAL_StatusTypeDef ads_port_spi_transmit_receive(SPI_HandleTypeDef* hspi, const uint8_t* pTxData,
                                                uint8_t* pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef ads_port_spi_transmit(SPI_HandleTypeDef* hspi, const uint8_t* pData,
                                        uint16_t Size, uint32_t Timeout);

#ifdef ADS_VERBOSE
#define ads_port_print_string(...) __println(__VA_ARGS__)
#else
#define ads_port_print_string(...)
#endif
#define ads_port_delay(ms) vTaskDelay(pdMS_TO_TICKS(ms))

#ifdef __cplusplus
}
#endif

#endif /* ADS126X_PORT_H */
