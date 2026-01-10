#include <stdint.h>
#include "include/mmio.h"

typedef enum {
    GREEN,
    YELLOW,
    RED
}led_t;

typedef struct {
    field32_t rcc_field;  // Which clock to enable
    uint8_t port;         // Which port (GPIOB = 1, GPIOE = 4)
    uint8_t pin;          // Which pin
} led_config_t;

static const led_config_t LED_MAP[] = {
    [GREEN]    = {RCC_AHB4ENR_GPIOBEN, 1, 0},
    [YELLOW]  = {RCC_AHB4ENR_GPIOBEN, 4, 1},
    [RED] = {RCC_AHB4ENR_GPIOEEN, 1, 14}
};

void led_init(int led) {
    // Set bus clock
    SET_FIELD(RCC_AHB1ENR, LED_MAP[led].rcc_field);

    // Set pin mode to output
    WRITE_FIELD(GPIOx_MODER[LED_MAP[led].port], GPIOx_MODER_MODEx[LED_MAP[led].pin], 0b01);
}

// Toggle the led on or off
void toggle_led(int led) {
    TOGL_FIELD(GPIOx_ODR[LED_MAP[led].port], GPIOx_ODR_ODx[LED_MAP[led].pin]);
}