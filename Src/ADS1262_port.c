/*
 * ADS1262_port.c - Platform-specific HAL implementations for ADS1262 driver.
 */
#include "ADS1262_port.h"

void ads_port_cs_reset(GPIO_TypeDef* cs_port, uint16_t cs_pin) {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

void ads_port_cs_set(GPIO_TypeDef* cs_port, uint16_t cs_pin) {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}

void ads_port_start_reset(GPIO_TypeDef* start_port, uint16_t start_pin) {
    HAL_GPIO_WritePin(start_port, start_pin, GPIO_PIN_RESET);
}

void ads_port_start_set(GPIO_TypeDef* start_port, uint16_t start_pin) {
    HAL_GPIO_WritePin(start_port, start_pin, GPIO_PIN_SET);
}

void ads_port_pwdn_reset(GPIO_TypeDef* pwdn_port, uint16_t pwdn_pin) {
    HAL_GPIO_WritePin(pwdn_port, pwdn_pin, GPIO_PIN_RESET);
}

void ads_port_pwdn_set(GPIO_TypeDef* pwdn_port, uint16_t pwdn_pin) {
    HAL_GPIO_WritePin(pwdn_port, pwdn_pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef ads_port_spi_transmit_receive(SPI_HandleTypeDef* hspi, const uint8_t* pTxData,
                                                uint8_t* pRxData, uint16_t Size, uint32_t Timeout) {
    return HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, Timeout);
}

HAL_StatusTypeDef ads_port_spi_transmit(SPI_HandleTypeDef* hspi, const uint8_t* pData,
                                        uint16_t Size, uint32_t Timeout) {
    return HAL_SPI_Transmit(hspi, (uint8_t*)pData, Size, Timeout);
}
