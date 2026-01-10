#include <stdint.h>
#include "include/mmio.h"
#include "myWork/systick.h"

typedef enum {
    GREEN,
    YELLOW,
    RED
}led_t;

typedef struct {
    uint8_t port;         // Which port (GPIOB = 1, GPIOE = 4)
    uint8_t pin;          // Which pin
} led_config_t;

static const led_config_t LED_MAP[] = {
    [GREEN]  = {1, 0},
    [YELLOW] = {4, 1},
    [RED]    = {1, 14}
};

void led_init(int led) {
    // Set bus clock
    switch (led) {
        case (GREEN):
        case (RED):
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOBEN);
            break;
        case (YELLOW):
            SET_FIELD(RCC_AHB4ENR, RCC_AHB4ENR_GPIOEEN);
            break;
    }

    // Set pin mode to output
    WRITE_FIELD(GPIOx_MODER[LED_MAP[led].port], GPIOx_MODER_MODEx[LED_MAP[led].pin], 0b01);
}

// Toggle the led on or off
void toggle_led(int led) {
    TOGL_FIELD(GPIOx_ODR[LED_MAP[led].port], GPIOx_ODR_ODx[LED_MAP[led].pin]);
}