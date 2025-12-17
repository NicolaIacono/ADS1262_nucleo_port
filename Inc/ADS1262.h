/*
 * ADS1262.h - C port of Arduino ADS1262 library for STM32H7
 */
#ifndef ADS1262_C_H
#define ADS1262_C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ADS1262_port.h"
#include <stdint.h>

#define CONFIG_SPI_MASTER_DUMMY 0xFF

/* Commands */
#define NOP 0x00   // No operation
#define RESET 0x06 // Reset to power-up values
#define START 0x08 // Start conversions
#define STOP 0x0A  // Stop conversions
#define RDATA 0x12 // Read data by command
/* Calibration commands. They need further implementation */
#define SYOCAL 0x16 // System offset calibration
#define SYGCAL 0x17 // System gain calibration
#define SFOCAL 0x19 // Self offset calibration
/* Register read/write commands */
#define RREG 0x20 // Read n registers starting at address
#define WREG 0x40 // Write n registers starting at address

// Register addresses
#define ID 0x00 /* 000: ADS1262, 001: ADS1263 */
#define POWER 0x01
#define INTERFACE 0x02
#define MODE0 0x03
#define MODE1 0x04
#define MODE2 0x05
#define INPMUX 0x06
#define OFCAL0 0x07
#define OFCAL1 0x08
#define OFCAL2 0x09
#define FSCAL0 0x0A
#define FSCAL1 0x0B
#define FSCAL2 0x0C
#define IDACMUX 0x0D
#define IDACMAG 0x0E
#define REFMUX 0x0F
#define TDACP 0x10
#define TDACN 0x11
#define GPIOCON 0x12
#define GPIODIR 0x13
#define GPIODAT 0x14

/* registers masks */
#define POWER_RESET_MASK 0x10
/* command masks */
#define REG_CMD_MASK 0x1F

/* Input multiplexer settings */
#define MUXP_AIN0 0x0
#define MUXP_AIN1 0x1
#define MUXP_AIN2 0x2
#define MUXP_AIN3 0x3
#define MUXP_AIN4 0x4
#define MUXP_AIN5 0x5
#define MUXP_AIN6 0x6
#define MUXP_AIN7 0x7
#define MUXP_AIN8 0x8
#define MUXP_AIN9 0x9
#define MUXP_AINCOM 0xA
#define MUXP_TSENSP 0xB
#define MUXP_AVDD 0xC
#define MUXP_DVDD 0xD
#define MUXP_TDACP 0xE
#define MUXP_FLOAT 0xF

#define MUXN_AIN0 0x0
#define MUXN_AIN1 0x1
#define MUXN_AIN2 0x2
#define MUXN_AIN3 0x3
#define MUXN_AIN4 0x4
#define MUXN_AIN5 0x5
#define MUXN_AIN6 0x6
#define MUXN_AIN7 0x7
#define MUXN_AIN8 0x8
#define MUXN_AIN9 0x9
#define MUXN_AINCOM 0xA
#define MUXN_TSENSN 0xB
#define MUXN_AVSS 0xC
#define MUXN_DVSS 0xD
#define MUXN_TDACN 0xE
#define MUXN_FLOAT 0xF

// Device context
typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef*      cs_port;
    uint16_t           cs_pin;
    GPIO_TypeDef*      drdy_port;
    uint16_t           drdy_pin; // optional
    GPIO_TypeDef*      start_port;
    uint16_t           start_pin; // optional
    GPIO_TypeDef*      pwdn_port;
    uint16_t           pwdn_pin; // optional
} ads1262_t;

// API functions

/* ===================== Application functions ===================== */
void ads_init_default(ads1262_t* dev);
int  ads_reset(ads1262_t* dev);
void ads_self_calibration(ads1262_t* dev);
int ads_select_input(ads1262_t* dev, uint8_t pos_channel, uint8_t neg_channel);
void ads_select_input_fast(ads1262_t* dev, uint8_t pos_channel, uint8_t neg_channel);
void ads_start_conversion(ads1262_t* dev);
void ads_stop_conversion(ads1262_t* dev);
void ads_read_data_direct(ads1262_t* dev, uint8_t* rx_buff);

/* ===================== Communication basic functions ===================== */
void    ads_spi_send_command(ads1262_t* dev, uint8_t cmd);
int     ads_reg_write(ads1262_t* dev, uint8_t reg_addr, uint8_t data);
int     ads_reg_write_fast(ads1262_t* dev, uint8_t reg_addr, uint8_t data);
int     ads_reg_write_and_check(ads1262_t* dev, uint8_t reg_addr, uint8_t data);
uint8_t ads_reg_read(ads1262_t* dev, uint8_t reg_addr);
// void ads_reg_write_consecutive(ads1262_t *dev, uint8_t reg_start_addr,
//                                uint8_t *tx_buff, uint8_t length);
// void ads_reg_read_consecutive(ads1262_t *dev, uint8_t reg_start_addr,
// uint8_t *rx_buff, uint8_t *dummy_tx_buff,
// uint8_t length);

/* ===================== low-level port functions ===================== */
void ads_cs_reset(ads1262_t* dev);
void ads_cs_set(ads1262_t* dev);
void ads_start_reset(ads1262_t* dev);
void ads_start_set(ads1262_t* dev);
void ads_pwdn_reset(ads1262_t* dev);
void ads_pwdn_set(ads1262_t* dev);
void ads_delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* ADS1262_C_H */
