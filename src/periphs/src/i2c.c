/**
 * This file is part of the Titan Flight Computer Project
 * Copyright (c) 2024 UW SARP
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 * 
 * @file modules/periphs/src/i2c.c
 * @authors Charles Faisandier, Joshua Beard
 * @brief I2C driver implementation
 */

#include <stdint.h>
#include <stdbool.h>

#include "periphs/include/i2c.h"
#include "internal/include/mmio.h"
#include "internal/include/dma.h"
#include "util/include/errc.h"

#define I2C_INSTANCE 1 

#define I2C_INSTANCE_COUNT 4
#define I2C_DMA_REQ_COUNT 2

/**************************************************************************************************
 * @section Internal Data Structures and Data Types
 **************************************************************************************************/
// Used to find the right request number for the I2C instance
// Index 0 is for RX request, 1 is for TX
// I2C4 is handled by DMAMUX2 so It's excluded for now.
static const uint32_t i2c_dmamux_req[I2C_INSTANCE_COUNT][I2C_DMA_REQ_COUNT] = {
    [1] = {
        [0] = 33,
        [1] = 34,
    },
    [2] = {
        [0] = 35,
        [1] = 36,
    },
    [3] = {
        [0] = 73,
        [1] = 74,
    },
};

// Tracks whether I2C is currently in use
bool i2c_busy = false;

// Timeout for single I2C read/write operation. Set by configuration.
uint32_t i2c_timeout = 0;

// Stores the user-defined callback for DMA completion (Restored from previous context)
static dma_callback_t i2c_dma_callback = NULL;

/**************************************************************************************************
 * @section Private Helper Functions
 **************************************************************************************************/
inline static enum ti_errc_t check_i2c_config(i2c_config_t *config) {
    if (config == NULL) {
        return TI_ERRC_INVALID_ARG; //I2C configuration struct is NULL
    }
    if (config->digital_filter < 0 || config->digital_filter > 15) {
        return TI_ERRC_INVALID_ARG; //I2C configuration digital_filter invalid value
    }
    if (config->scl_pin < 0 || config->scl_pin > 139) {
        return TI_ERRC_INVALID_ARG; //I2C configuration scl_pin invalid value
    }
    if (config->sda_pin < 0 || config->sda_pin > 139) {
        return TI_ERRC_INVALID_ARG; //I2C configuration sda_pin invalid value
    }
    return TI_ERRC_NONE;
}

enum ti_errc_t i2c_transmit_check_params(uint16_t addr, uint8_t *buff, size_t size) {
    if (buff == NULL) {
        return TI_ERRC_INVALID_ARG; //Invalid buffer for I2C transmission
    }
    if (size == 0) {
        return TI_ERRC_INVALID_ARG; //Prevent configuration of zero-byte transfer
    }
    return TI_ERRC_NONE;;
}

/**************************************************************************************************
 * @section Public Function Implementation 
 **************************************************************************************************/
enum ti_errc_t i2c_init(i2c_config_t *config, dma_callback_t callback) {
    enum ti_errc_t errc;

    // Check parameters
    if ((errc = check_i2c_config(config)) != TI_ERRC_NONE) return errc;

    // Store the user callback (Restored from previous context)
    i2c_dma_callback = callback;

    // 1. Enable the I2C clock
    SET_FIELD(RCC_APB1LENR, RCC_APB1LENR_I2CxEN[I2C_INSTANCE]);

    // 2. Enable the GPIO clock and configure pins
    tal_enable_clock(config->scl_pin);
    tal_enable_clock(config->sda_pin);

    // Alternate function mode
    tal_set_mode(config->sda_pin, 2); 
    tal_set_mode(config->scl_pin, 2);
    tal_alternate_mode(config->sda_pin, 4); 
    tal_alternate_mode(config->scl_pin, 4);

    // Open drain
    tal_set_drain(config->sda_pin, 1); 
    tal_set_drain(config->scl_pin, 1);

    // High speed
    tal_set_speed(config->sda_pin, 3);
    tal_set_speed(config->scl_pin, 3);

    // Pull up
    tal_pull_pin(config->sda_pin, 1); 
    tal_pull_pin(config->scl_pin, 1);

    // 3. Disable peripheral before configuring
    WRITE_FIELD(I2Cx_CR1[I2C_INSTANCE], I2Cx_CR1_PE, 0);
    
    // 4. Configure addressing mode (7-bit or 10-bit)
    if (config->addr_mode == I2C_ADDR_10BIT) {
        SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_ADD10);
    } else {
        CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_ADD10);
    }

    // 5. Configure analog and digital filters
    if (config->analog_filter) CLR_FIELD(I2Cx_CR1[I2C_INSTANCE], I2Cx_CR1_ANFOFF);
    WRITE_FIELD(I2Cx_CR1[I2C_INSTANCE], I2Cx_CR1_DNF, config->digital_filter);

    // 6. Set timing
    *I2Cx_TIMINGR[I2C_INSTANCE] = config->timing;

    // 7. Set the timeout
    i2c_timeout = config->timeout;

    // 8. Re-enable the I2C peripheral
    SET_FIELD(I2Cx_CR1[I2C_INSTANCE], I2Cx_CR1_PE);

    return TI_ERRC_NONE;
}

enum ti_errc_t i2c_read_async(uint16_t addr, uint8_t *rx_data, size_t size) {
    enum ti_errc_t errc;

    // Check parameters
    if ((errc = i2c_transmit_check_params(addr, rx_data, size)) != TI_ERRC_NONE) {
        return errc;
    }

    // Check if busy
    if (i2c_busy) {
        return TI_ERRC_BUSY; //I2C is busy
    }
    i2c_busy = true;

    // Configure and start DMA transfer
    dma_transfer_t rx_transfer = {
        .request_id = i2c_dmamux_req[I2C_INSTANCE][0], // RX request ID
        .direction = PERIPH_TO_MEM,
        .src_data_size = 1,     // I2C peripheral register is 8-bit (1 byte)
        .dest_data_size = 1,    // Reading into an 8-bit buffer (1 byte)
        .priority = 0,          // Low priority default
        .callback = i2c_dma_callback, // Use stored callback
        .src = I2Cx_RXDR[I2C_INSTANCE], // Assuming instance 0, depends on implementation
        .dest = rx_data,
        .size = size,
        .context = &i2c_busy,
        .disable_mem_inc = false,
    };
    if ((errc = dma_start_transfer(&rx_transfer)) != TI_ERRC_NONE) {
        i2c_busy = false;
        return errc;
    }

    // Configure and enable I2C for a read
    if (READ_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_ADD10)) {
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_10BIT, addr);
    } else {
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_7BIT, addr);
    }
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_AUTOEND);
    WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_NBYTES, size);
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RD_WRN); // Set to 1 for a read operation
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_PECBYTE);
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RELOAD);

    // Enable the DMA requests and start the transfer
    SET_FIELD(I2Cx_CR1[I2C_INSTANCE], I2Cx_CR1_RXDMAEN); // Enable DMA for reception
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_START);  // Send the START condition

    return TI_ERRC_NONE;
}

enum ti_errc_t i2c_write_async(uint16_t addr, uint8_t *tx_data, size_t size) {
    enum ti_errc_t errc;

    // 1. Check parameters
    if ((errc = (i2c_transmit_check_params(addr, tx_data, size))) != TI_ERRC_NONE) {
        return errc;
    }

    // 2. Check if the I2C bus is busy using an atomic operation.
    if (i2c_busy) {
        return TI_ERRC_BUSY; //I2C is busy
    }
    i2c_busy = true;

    // 3. Configure the DMA transfer
    dma_transfer_t tx_transfer = {
        .request_id = i2c_dmamux_req[I2C_INSTANCE][1], // TX request ID
        .direction = MEM_TO_PERIPH,
        .src_data_size = 1,     // I2C peripheral register is 8-bit (1 byte)
        .dest_data_size = 1,    // Reading into an 8-bit buffer (1 byte)
        .priority = 0,          // Low priority default
        .callback = i2c_dma_callback, // Use stored callback
        .src = tx_data,
        .dest = I2Cx_TXDR[I2C_INSTANCE],
        .size = size,
        .context = (void*)&i2c_busy,
        .disable_mem_inc = false,
    };

    // 4. Start the DMA transfer
    if ((errc = dma_start_transfer(&tx_transfer)) != TI_ERRC_NONE) {
        i2c_busy = false;
        return errc;
    }

    // 5. Configure and enable the I2C peripheral for a write
    if (READ_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_ADD10)) {
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_10BIT, addr);
    } else {
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_7BIT, addr);
    }
    WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_NBYTES, size);    // Set number of bytes
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RD_WRN);            // Set to 0 for a write operation
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_AUTOEND);           // Auto-generate STOP condition
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_PECBYTE);           // Disable PEC
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RELOAD);            // Disable reload

    // 6. Enable the DMA requests and start the transfer
    SET_FIELD(I2Cx_CR1[I2C_INSTANCE], I2Cx_CR1_TXDMAEN); // Enable DMA for transmission
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_START);   // Send the START condition

    return TI_ERRC_NONE;
}

enum ti_errc_t i2c_read_blocking(uint16_t addr, uint8_t *rx_data, size_t size) {
    enum ti_errc_t errc;

    // Check parameters
    if ((errc = i2c_transmit_check_params(addr, rx_data, size)) != TI_ERRC_NONE) {
        return errc;
    }

    // Check if busy
    if (i2c_busy) {
        return TI_ERRC_BUSY; //I2C is busy
    }
    i2c_busy = true;

    // Configure and enable I2C for a read
    if (READ_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_ADD10)) {
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_10BIT, addr);
    } else {
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_7BIT, addr);
    }
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_AUTOEND);
    WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_NBYTES, size);
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RD_WRN); // Set to 1 for a read operation
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_PECBYTE);
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RELOAD);

    // Start the transfer
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_START);  // Send the START condition

    // Read the data byte by byte in a blocking loop.
    for (size_t i = 0; i < size; ++i) {
        // Wait until the receive data register is not empty (RXNE flag).
        uint32_t timeout = 0;
        while (!READ_FIELD(I2Cx_ISR[I2C_INSTANCE], I2Cx_ISR_RXNE)) {
            if (timeout++ > i2c_timeout) {
                i2c_busy = false;
                return TI_ERRC_TIMEOUT; //I2C read timeout
            }
        }
        // Read the received byte from the data register.
        rx_data[i] = (uint8_t)READ_FIELD(I2Cx_RXDR[I2C_INSTANCE], I2Cx_RXDR_RXDATA);
    }
    
    // Wait for the STOP condition to be sent (STOPF flag).
    uint32_t timeout_stop = 0;
    while (!READ_FIELD(I2Cx_ISR[I2C_INSTANCE], I2Cx_ISR_STOPF)) {
         if (timeout_stop++ > i2c_timeout) {
            i2c_busy = false;
            return TI_ERRC_TIMEOUT; //I2C stop condition timeout
        }
    }
    // Clear the STOPF flag by writing 1 to it.
    WRITE_FIELD(I2Cx_ICR[I2C_INSTANCE], I2Cx_ICR_STOPCF, 1);


    i2c_busy = false;
    return TI_ERRC_NONE;
}

enum ti_errc_t i2c_write_blocking(uint16_t addr, uint8_t *tx_data, size_t size) {
enum ti_errc_t errc;

    // 1. Check parameters
    if ((errc = i2c_transmit_check_params(addr, tx_data, size)) != TI_ERRC_NONE) {
        return errc;
    }

    // 2. Check if busy
    if (i2c_busy) {
        return TI_ERRC_BUSY; //I2C is busy
    }
    i2c_busy = true;

    // 3. Configure the I2C peripheral for a write
    // The peripheral's NBYTES register will handle the number of bytes to transfer.
    WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_NBYTES, size);
    
    // Set the slave address and the read/write direction bit.
    if (READ_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_ADD10)) {
        // For 10-bit addressing, the address is written directly to the SADD_10BIT field.
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_10BIT, addr);
    } else {
        // For 7-bit addressing, the address is shifted left by one.
        WRITE_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_SADD_7BIT, addr);
    }
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RD_WRN);     // Set to 0 for a write operation
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_AUTOEND);    // Auto-generate STOP condition
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_PECBYTE);    // Disable PEC
    CLR_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_RELOAD);     // Disable reload

    // 4. Send the START condition to begin the transaction.
    SET_FIELD(I2Cx_CR2[I2C_INSTANCE], I2Cx_CR2_START);

    // 5. Write the data byte by byte in a blocking loop.
    for (size_t i = 0; i < size; ++i) {
        // Wait until the transmit data register is empty (TXIS flag).
        uint32_t timeout = 0;
        while (!READ_FIELD(I2Cx_ISR[I2C_INSTANCE], I2Cx_ISR_TXIS)) {
            if (timeout++ > i2c_timeout) {
                i2c_busy = false;
                return TI_ERRC_TIMEOUT; //I2C write timeout
            }
        }
        // Write the next byte to the transmit data register.
        WRITE_FIELD(I2Cx_TXDR[I2C_INSTANCE], I2Cx_TXDR_TXDATA, tx_data[i]);
    }
    
    // 6. Wait for the STOP condition to be sent (STOPF flag).
    uint32_t timeout_stop = 0;
    while (!READ_FIELD(I2Cx_ISR[I2C_INSTANCE], I2Cx_ISR_STOPF)) {
         if (timeout_stop++ > i2c_timeout) {
            i2c_busy = false;
            return TI_ERRC_TIMEOUT; //I2C write timeout
        }
    }
    // Clear the STOPF flag by writing 1 to it.
    WRITE_FIELD(I2Cx_ICR[I2C_INSTANCE], I2Cx_ICR_STOPCF, 1);

    i2c_busy = false;

    return TI_ERRC_NONE;
}