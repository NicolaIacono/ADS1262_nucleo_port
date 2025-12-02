/*
 * ADS1262.c - C port of Arduino ADS1262 library for STM32H7
 */
#include "ADS1262.h"
#include "ADS1262_port.h"
#include <stdint.h>

/* ===================== Application functions ===================== */

void ads_init_default(ads1262_t* dev) {
    if (!dev || !dev->hspi || !dev->cs_port || !dev->pwdn_port || !dev->drdy_port ||
        !dev->start_port || !dev->pwdn_port)
        return;

    /* Power up the device */
    ads_pwdn_reset(dev); /* Turn off */
    ads_delay(500);
    ads_pwdn_set(dev); /* Turn on */
    ads_delay(500);
    ads_start_reset(dev); /* Ensure START is low */
    ads_cs_reset(dev);    /* Enable CS (low) */

    uint8_t id_reg_val = ads_reg_read(dev, 0x00);
    uint8_t id         = (id_reg_val >> 5);
    uint8_t revision   = id_reg_val & 0x1F;
    ads_port_print_string("Device ID: %d\n", id);
    ads_port_print_string("Revision: %d\n", revision);
    uint8_t power_reg_val = ads_reg_read(dev, POWER);
    uint8_t reset_bit     = (power_reg_val >> 4) & 1;
    ads_port_print_string("POWER: 0x%02X\n", power_reg_val);
    ads_port_print_string("RESET: %d\n", reset_bit);
    ads_delay(100);

    ads_port_print_string("Initializing ADS1262...\n");

    ads_port_print_string("Resetting ADS1262...\n");
    if (ads_reset(dev) != 0) {
        ads_port_print_string("Error: ADS1262 reset failed!\n");
        return;
    } else {
        ads_port_print_string("ADS1262 reset successful.\n");
    }

    ads_port_print_string("Reading device ID...\n");
    id_reg_val = ads_reg_read(dev, ID);
    id         = (id_reg_val >> 5);
    revision   = id_reg_val & 0x1F;
    ads_port_print_string("Device ID: %d\n", id);
    ads_port_print_string("Revision: %d\n", revision);

    /* Default ADS configuration */
    ads_port_print_string("Configuring ADS126x registers...\n");
    if (ads_reg_write_and_check(dev, MODE1, (uint8_t)0x00) !=
        0) { /* sync1 filter and rest default */
        ads_port_print_string("Error writing MODE1 register!\n");
        return;
    }
    ads_delay(1);
    if (ads_reg_write_and_check(dev, MODE2, (uint8_t)0x89) != 0) { /* DR = 1200 SPS, bypass PGA */
        ads_port_print_string("Error writing MODE2 register!\n");
        return;
    }

    ads_delay(100);
    /* Perform self offset calibration */
    ads_port_print_string("Performing self offset calibration...\n");
    ads_self_calibration(dev);
    ads_port_print_string("Self offset calibration completed.\n");
    ads_delay(100);

    /* Differential input between AIN6 and AIN7 (7-6) */
    ads_select_input(dev, MUXP_AIN6, MUXN_AIN7);
    ads_delay(1);
}

int ads_reset(ads1262_t* dev) {
    if (!dev || !dev->pwdn_port)
        return -1;
    // Try to reset using the PWDN pin if available

    HAL_GPIO_WritePin(dev->pwdn_port, dev->pwdn_pin, GPIO_PIN_SET);
    ads_port_delay(10);
    HAL_GPIO_WritePin(dev->pwdn_port, dev->pwdn_pin, GPIO_PIN_RESET);
    ads_port_delay(10);
    HAL_GPIO_WritePin(dev->pwdn_port, dev->pwdn_pin, GPIO_PIN_SET);
    ads_port_delay(10);

    // Check POWER register to confirm reset
    uint8_t power_reg = ads_reg_read(dev, POWER);
    if (power_reg & POWER_RESET_MASK) {
        // Reset happened successfully
        // Clear the reset bit by writing back to the POWER register
        ads_reg_write(dev, POWER, power_reg & ~POWER_RESET_MASK);
        return 0;
    }

    // If not reset, return error
    return -1;
}

void ads_self_calibration(ads1262_t* dev) {
    ads_port_print_string("Starting self offset calibration...\n");
    /* Follow the datasheet provided instruction */
    /* Set to continuous conversion mode */
    uint8_t mode0_reg = ads_reg_read(dev, MODE0);
    mode0_reg &= ~(1 << 6);
    ads_reg_write(dev, MODE0, mode0_reg);
    /* Select ADC reference voltage to V_AVDD - V_AVSS */
    uint8_t refmux_val = 0x24;
    ads_reg_write_and_check(dev, REFMUX, refmux_val);
    /* Select both inputs to float */
    ads_select_input(dev, MUXP_FLOAT, MUXN_FLOAT);
    /* Start conversion */
    ads_start_conversion(dev);
    /* Send calibration command, keeping cs tied low. */
    ads_cs_reset(dev);
    uint8_t cmd = SFOCAL;
    ads_port_spi_transmit(dev->hspi, &cmd, 1, 10);
    ads_port_delay(500); /* Wait for calibration to complete */
    ads_cs_set(dev);
    ads_delay(10);
    /* Stop conversion */
    ads_stop_conversion(dev);
    /* Self offset calibration completed */
    ads_port_print_string("Self offset calibration completed.\n");

    /* Read OFFSETCAL registers to verify calibration */
    uint8_t offsetcal0 = ads_reg_read(dev, 0x7);
    uint8_t offsetcal1 = ads_reg_read(dev, 0x8);
    uint8_t offsetcal2 = ads_reg_read(dev, 0x9);

    uint32_t offset_code =
        ((uint32_t)offsetcal0 << 16) | ((uint32_t)offsetcal1 << 8) | (uint32_t)offsetcal2;
    ads_port_print_string("OFFSETCAL Code: 0x%06X\n", offset_code);
}

void ads_select_input(ads1262_t* dev, uint8_t muxp, uint8_t muxn) {
    ads_reg_write_and_check(dev, INPMUX, (uint8_t)(((muxp << 4) & 0xF0) | (muxn & 0x0F)));
}

void ads_select_input_fast(ads1262_t* dev, uint8_t muxp, uint8_t muxn) {
    ads_reg_write_fast(dev, INPMUX, (uint8_t)(((muxp << 4) & 0xF0) | (muxn & 0x0F)));
}

void ads_start_conversion(ads1262_t* dev) { ads_start_set(dev); }
void ads_stop_conversion(ads1262_t* dev) { ads_start_reset(dev); }

void ads_read_data_direct(ads1262_t* dev, uint8_t* rx_buff) {
    /* Use ADC1 read data direct functional mode */
    /* Transfer bytes that are not a command and read the data from the output
     * shift register */
    uint8_t tx_buff[6] = {CONFIG_SPI_MASTER_DUMMY, CONFIG_SPI_MASTER_DUMMY,
                          CONFIG_SPI_MASTER_DUMMY, CONFIG_SPI_MASTER_DUMMY,
                          CONFIG_SPI_MASTER_DUMMY, CONFIG_SPI_MASTER_DUMMY};
    ads_cs_reset(dev);
    ads_port_spi_transmit_receive(dev->hspi, tx_buff, (uint8_t*)rx_buff, 6, 10);
    ads_cs_set(dev);
}

/* ===================== Communication basic functions ===================== */

void ads_spi_send_command(ads1262_t* dev, uint8_t cmd) {
    ads_cs_reset(dev);
    ads_delay(1);
    ads_port_spi_transmit(dev->hspi, &cmd, 1, 10);
    ads_delay(1);
    ads_cs_set(dev);
}

int ads_reg_write(ads1262_t* dev, uint8_t reg_addr, uint8_t data) {
    uint8_t cmd             = (uint8_t)((reg_addr & REG_CMD_MASK) | WREG);
    uint8_t number_of_bytes = 1 - 1; // zero based
    uint8_t buf[3]          = {cmd, number_of_bytes, data};

    ads_cs_reset(dev);
    ads_delay(1);
    if (ads_port_spi_transmit(dev->hspi, buf, sizeof(buf), 10) != 0)
        return -1;
    ads_delay(1);
    ads_cs_set(dev);
    return 0;
}

int ads_reg_write_fast(ads1262_t* dev, uint8_t reg_addr, uint8_t data) {
    uint8_t cmd             = (uint8_t)((reg_addr & REG_CMD_MASK) | WREG);
    uint8_t number_of_bytes = 1 - 1; // zero based
    uint8_t buf[3]          = {cmd, number_of_bytes, data};

    ads_cs_reset(dev);
    if (ads_port_spi_transmit(dev->hspi, buf, sizeof(buf), 10) != 0)
        return -1;
    ads_cs_set(dev);
    return 0;
}

int ads_reg_write_and_check(ads1262_t* dev, uint8_t reg_addr, uint8_t data) {
    if (ads_reg_write(dev, reg_addr, data) != 0)
        return -1;

    ads_delay(1); // allow register to update
    uint8_t read_back = ads_reg_read(dev, reg_addr);
    return (read_back == data) ? 0 : -1;
}

uint8_t ads_reg_read(ads1262_t* dev, uint8_t reg_addr) {
    uint8_t cmd             = (uint8_t)((reg_addr & REG_CMD_MASK) | RREG);
    uint8_t number_of_bytes = 1 - 1; // zero based
    uint8_t tx_buff[3]      = {cmd, number_of_bytes, (uint8_t)CONFIG_SPI_MASTER_DUMMY};
    uint8_t rx_buff[3]      = {0};

    ads_cs_reset(dev);
    ads_delay(1);
    if (ads_port_spi_transmit_receive(dev->hspi, tx_buff, rx_buff, 3, 10) != 0) {
        ads_cs_set(dev);
        ads_port_print_string("Error reading register...\n");
        return 0;
    }
    ads_delay(1);
    ads_cs_set(dev);

    return rx_buff[2];
}

// void ads_reg_write_consecutive(ads1262_t *dev, uint8_t reg_start_addr,
//                                uint8_t *tx_buff, uint8_t length) {
//   uint8_t cmd = (uint8_t)((reg_start_addr & REG_CMD_MASK) | WREG);
//   uint8_t number_of_bytes = length - 1; // zero based

//   uint8_t cmd_buf[2] = {cmd, number_of_bytes};

//   ads_cs_reset(dev);
//   ads_delay(1);
//   ads_port_spi_transmit(dev->hspi, cmd_buf, sizeof(cmd_buf), 10);
//   ads_port_spi_transmit(dev->hspi, tx_buff, length, 10);
//   ads_delay(1);
//   ads_cs_set(dev);
// }

// void ads_reg_read_consecutive(ads1262_t *dev, uint8_t reg_start_addr,
//                               uint8_t *rx_buff, uint8_t *dummy_tx_buff,
//                               uint8_t length) {
//   uint8_t cmd = (uint8_t)((reg_start_addr & REG_CMD_MASK) | RREG);
//   uint8_t number_of_bytes = length - 1; // zero based

//   uint8_t cmd_buf[2] = {cmd, number_of_bytes};
//   uint8_t rx_hdr[2] = {0}; /* Dummy rx buffer */
//   for (uint8_t i = 0; i < length; i++) {
//     dummy_tx_buff[i] = CONFIG_SPI_MASTER_DUMMY;
//   }
//   ads_cs_reset(dev);
//   ads_delay(1);
//   ads_port_spi_transmit_receive(dev->hspi, cmd_buf, rx_hdr, 2, 10);
//   ads_port_spi_transmit_receive(dev->hspi, dummy_tx_buff, rx_buff, length,
//   10); ads_delay(1); ads_cs_set(dev);
// }

/* ===================== low-level port functions ===================== */

void ads_cs_reset(ads1262_t* dev) {
    if (dev && dev->cs_port)
        ads_port_cs_reset(dev->cs_port, dev->cs_pin);
}

void ads_cs_set(ads1262_t* dev) {
    if (dev && dev->cs_port)
        ads_port_cs_set(dev->cs_port, dev->cs_pin);
}

void ads_start_reset(ads1262_t* dev) {
    if (dev && dev->start_port)
        ads_port_start_reset(dev->start_port, dev->start_pin);
}

void ads_start_set(ads1262_t* dev) {
    if (dev && dev->start_port)
        ads_port_start_set(dev->start_port, dev->start_pin);
}

void ads_pwdn_reset(ads1262_t* dev) {
    if (dev && dev->pwdn_port)
        ads_port_pwdn_reset(dev->pwdn_port, dev->pwdn_pin);
}

void ads_pwdn_set(ads1262_t* dev) {
    if (dev && dev->pwdn_port)
        ads_port_pwdn_set(dev->pwdn_port, dev->pwdn_pin);
}

void ads_delay(uint32_t ms) { ads_port_delay(ms); }