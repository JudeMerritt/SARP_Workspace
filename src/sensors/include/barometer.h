//TODO: add improved comments with @brief

#pragma once
#include <stdint.h>
#include "periphs/include/spi.h"
#include "util/include/errc.h"

// Oversampling ratio 
typedef enum {
    OSR_256  = 0x00, // 256 samples per measurement 
    OSR_512  = 0x02, // 512 samples per measurement 
    OSR_1024 = 0x04, // 1024 samples per measurement 
    OSR_2048 = 0x06, // 2048 samples per measurement 
    OSR_4096 = 0x08  // 4096 samples per measurement 
}barometer_osr_t;

// Configuration data
typedef struct {
    uint16_t sens;     // C1 Presure sensitivity 
    uint16_t off;      // C2 Pressure offset
    uint16_t tcs;      // C3 Temperature coefficient of pressure sensitivity
    uint16_t tco;      // C4 Temperature coefficient of pressure offset
    uint16_t t_ref;    // C5 Reference temperature
    uint16_t tempsens; // C6 Temperature coefficient of the temperature 
}barometer_config_data_t;

// Results
typedef struct {
    float pressure;    // Temperature compensated pressure from 10 mbar to 1200 mbar with 0.01 mbar resolution
    float temperature; // Temperature from -40 C to 85 C with 0.01 C resulution
}barometer_result_t;

// Barometer struct 
typedef struct {
    spi_device_t device;                 // SPI instance and CS pin
    barometer_osr_t osr;                 // Oversampling setting
    barometer_config_data_t config_data; // Device configuration
    barometer_result_t result;           // Result (pressure and temperature)
} barometer_t;

// Descriptive comment goes here
ti_errc_t barometer_init(barometer_t *dev);

// Descriptive comment goes here
ti_errc_t get_barometer_data(barometer_t *dev);