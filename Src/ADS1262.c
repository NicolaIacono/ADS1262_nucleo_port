/*
 * ADS1262.c - C port of Arduino ADS1262 library for STM32H7
 */
#include "ADS1262.h"
#include "ADS1262_port.h"
#include "stm32h7xx_hal_def.h"
#include <stdint.h>

/* ===================== Application functions ===================== */

void ads_init_default(ads1262_t* dev) {
    if (!dev || !dev->hspi || !dev->cs_port || !dev->pwdn_port || !dev->drdy_port ||
        !dev->start_port || !dev->pwdn_port)
        return;

    /* Power up the device */
    ads_pwdn_reset(dev);  /* Turn off */
    ads_start_reset(dev); /* Ensure START is low */
    ads_cs_set(dev);      /* Disable CS */
    ads_delay(50);
    ads_pwdn_set(dev); /* Power on */
    ads_delay(200);

    ads_port_print_string("Initializing ADS1262...");
    ads_port_print_string("Resetting ADS1262...");
    while (ads_reset(dev) != 0) {
        ads_port_print_string("Error: ADS1262 reset failed!");
        ads_delay(500);
    }

    uint8_t id_reg_val = ads_reg_read(dev, ID);
    uint8_t id         = (id_reg_val >> 5);
    uint8_t revision   = id_reg_val & 0x1F;
    ads_port_print_string("Device ID: %d", id);
    ads_port_print_string("Revision: %d", revision);

    /* Default ADS configuration */
    ads_port_print_string("Configuring ADS126x registers...");
    /* POWER REGISTER - level shift voltage to aincom pin enabled */
    if (ads_reg_write_and_check(dev, POWER, (uint8_t)0x13) != 0) {
        ads_port_print_string("Error writing POWER register!");
        return;
    }
    /* INTERFACE REGISTER */
    if (ads_reg_write_and_check(dev, INTERFACE, (uint8_t)0x05) !=
        0) { /* sync1 filter and rest default */
        ads_port_print_string("Error writing INTERFACE register!");
        return;
    }
    /* MODE REGISTERS */
    if (ads_reg_write_and_check(dev, MODE0, (uint8_t)0x00) !=
        0) { 
        ads_port_print_string("Error writing MODE0 register!");
        return;
    }
    if (ads_reg_write_and_check(dev, MODE1, (uint8_t)0x00) !=
        0) {
        ads_port_print_string("Error writing MODE1 register!");
        return;
    }
    if (ads_reg_write_and_check(dev, MODE2, (uint8_t)0x09) != 0) { /* DR = 1200 SPS, bypass PGA */
        ads_port_print_string("Error writing MODE2 register!");
        return;
    }
    /* INPMUX REGISTER */
    if (ads_reg_write_and_check(dev, INPMUX, (uint8_t)0x67) != 0) { /* DR = 1200 SPS, bypass PGA */
        ads_port_print_string("Error writing INPMUX register!");
        return;
    }
    /* REFMUX REGISTER */
    if (ads_reg_write_and_check(dev, REFMUX, (uint8_t)0x00) != 0) {
        ads_port_print_string("Error writing REFMUX register!");
        return;
    }

    /* Perform self offset calibration */
    ads_port_print_string("Performing self offset calibration...");
    ads_self_calibration(dev);
    ads_port_print_string("Self offset calibration completed.");
    ads_delay(100);

    /* Differential input between AIN6 and AIN7 (7-6) */
    ads_select_input(dev, MUXP_AIN1, MUXN_AIN0);
    ads_delay(100);
}

int ads_reset(ads1262_t* dev) {
    if (!dev || !dev->pwdn_port)
        return -1;
    // Try to reset using the PWDN pin if available
    HAL_GPIO_WritePin(dev->pwdn_port, dev->pwdn_pin, GPIO_PIN_SET);
    ads_port_delay(100);
    HAL_GPIO_WritePin(dev->pwdn_port, dev->pwdn_pin, GPIO_PIN_RESET);
    ads_port_delay(100);
    HAL_GPIO_WritePin(dev->pwdn_port, dev->pwdn_pin, GPIO_PIN_SET);
    ads_port_delay(100);
    // Check POWER register to confirm reset
    uint8_t power_reg = ads_reg_read(dev, POWER);
    if (power_reg & POWER_RESET_MASK) {
        // Reset happened successfully
        // Clear the reset bit by writing back to the POWER register
        ads_reg_write(dev, POWER, (power_reg & ~POWER_RESET_MASK));
        return 0;
    }
    // If not reset, return error
    return -1;
}

void ads_self_calibration(ads1262_t* dev) {
    ads_port_print_string("Starting self offset calibration...");
    /* Follow the datasheet provided instruction */
    /* Enable continuous conversion mode */
    if (ads_reg_write_and_check(dev, MODE0, 0x00) != 0) {
        ads_port_print_string("Error writing MODE0 register during calibration!");
        return;
    }
    /* Set reference */
    if (ads_reg_write_and_check(dev, REFMUX, 0x00) != 0) {
        ads_port_print_string("Error writing REFMUX register during calibration!");
        return;
    }
    /* Select both inputs to float */
    if (ads_select_input(dev, MUXP_FLOAT, MUXN_FLOAT) != 0) {
        ads_port_print_string("Error selecting input channels during calibration!");
        return;
    }
    /* Start conversion */
    ads_start_conversion(dev);
    /* Send calibration command, keeping cs tied low. */
    ads_cs_reset(dev);
    uint8_t cmd = SFOCAL;
    ads_port_spi_transmit(dev->hspi, &cmd, 1, 10);
    ads_port_delay(5000); /* Wait for calibration to be completed */
    ads_cs_set(dev);
    ads_delay(100);
    /* Stop conversion */
    ads_stop_conversion(dev);
    /* Self offset calibration completed */
    ads_port_print_string("Self offset calibration completed.");

    /* Read OFFSETCAL registers to verify calibration */
    uint8_t offsetcal0 = ads_reg_read(dev, 0x7);
    uint8_t offsetcal1 = ads_reg_read(dev, 0x8);
    uint8_t offsetcal2 = ads_reg_read(dev, 0x9);

    int32_t offset_code = (int32_t)((offsetcal2 << 16) | (offsetcal1 << 8) | offsetcal0);
    ads_port_print_string("OFFSETCAL (base 10): %d", offset_code);
}

int ads_select_input(ads1262_t* dev, uint8_t muxp, uint8_t muxn) {
    uint8_t reg_val = ((muxp << 4) & 0xF0) | (muxn & 0x0F);
    return ads_reg_write_and_check(dev, INPMUX, reg_val);
}

void ads_select_input_fast(ads1262_t* dev, uint8_t muxp, uint8_t muxn) {
    uint8_t reg_val = ((muxp << 4) & 0xF0) | (muxn & 0x0F);
    ads_reg_write_fast(dev, INPMUX, reg_val);
}

void ads_start_conversion(ads1262_t* dev) {
    ads_start_set(dev);
    ads_delay(20);
}
void ads_stop_conversion(ads1262_t* dev) {
    ads_start_reset(dev);
    ads_delay(20);
}

void ads_read_data_direct(ads1262_t* dev, uint8_t* rx_buff) {
    /* Use ADC1 read data direct functional mode */
    /* Transfer bytes that are not a command and read the data from the output
     * shift register */
    static uint8_t tx_buff[6] = {CONFIG_SPI_MASTER_DUMMY, CONFIG_SPI_MASTER_DUMMY,
                                 CONFIG_SPI_MASTER_DUMMY, CONFIG_SPI_MASTER_DUMMY,
                                 CONFIG_SPI_MASTER_DUMMY, CONFIG_SPI_MASTER_DUMMY};
    ads_cs_reset(dev);
    ads_port_spi_transmit_receive(dev->hspi, tx_buff, rx_buff, 6, 100);
    ads_cs_set(dev);
}

/* ===================== Communication basic functions ===================== */

void ads_spi_send_command(ads1262_t* dev, uint8_t cmd) {
    ads_cs_reset(dev);
    ads_delay(2);
    ads_cs_set(dev);
    ads_delay(2);
    ads_cs_reset(dev);
    ads_delay(2);
    ads_port_spi_transmit(dev->hspi, &cmd, 1, HAL_MAX_DELAY);
    ads_delay(2);
    ads_cs_set(dev);
}

int ads_reg_write(ads1262_t* dev, uint8_t reg_addr, uint8_t data) {
    uint8_t cmd    = (uint8_t)((reg_addr & REG_CMD_MASK) | WREG);
    uint8_t buf[3] = {cmd, 0, data};

    ads_cs_reset(dev);
    ads_delay(2);
    ads_cs_set(dev);
    ads_delay(2);
    ads_cs_reset(dev);
    ads_delay(2);
    if (ads_port_spi_transmit(dev->hspi, buf, sizeof(buf), HAL_MAX_DELAY) != 0)
        return -1;
    ads_delay(2);
    ads_cs_set(dev);
    return 0;
}

int ads_reg_write_fast(ads1262_t* dev, uint8_t reg_addr, uint8_t data) {
    uint8_t cmd    = (uint8_t)((reg_addr & REG_CMD_MASK) | WREG);
    uint8_t buf[3] = {cmd, 0, data};

    ads_cs_reset(dev);
    ads_cs_set(dev);
    ads_cs_reset(dev);
    if (ads_port_spi_transmit(dev->hspi, buf, sizeof(buf), 0) != 0)
        return -1;
    ads_cs_set(dev);
    return 0;
}

int ads_reg_write_and_check(ads1262_t* dev, uint8_t reg_addr, uint8_t data) {
    if (ads_reg_write(dev, reg_addr, data) != 0)
        return -1;
    ads_delay(10);
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
        ads_port_print_string("Error reading register...");
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
    // if (dev && dev->cs_port)
    ads_port_cs_reset(dev->cs_port, dev->cs_pin);
}

void ads_cs_set(ads1262_t* dev) {
    // if (dev && dev->cs_port)
    ads_port_cs_set(dev->cs_port, dev->cs_pin);
}

void ads_start_reset(ads1262_t* dev) {
    // if (dev && dev->start_port)
    ads_port_start_reset(dev->start_port, dev->start_pin);
}

void ads_start_set(ads1262_t* dev) {
    // if (dev && dev->start_port)
    ads_port_start_set(dev->start_port, dev->start_pin);
}

void ads_pwdn_reset(ads1262_t* dev) {
    // if (dev && dev->pwdn_port)
    ads_port_pwdn_reset(dev->pwdn_port, dev->pwdn_pin);
}

void ads_pwdn_set(ads1262_t* dev) {
    // if (dev && dev->pwdn_port)
    ads_port_pwdn_set(dev->pwdn_port, dev->pwdn_pin);
}

void ads_delay(uint32_t ms) { ads_port_delay(ms); }
