#include <stdint.h>
#include "include/mmio.h"
#include "myWork/systick.h"
#include "myWork/led.h"

typedef struct {
    uint8_t port; // Which port (GPIOB = 1, GPIOE = 4)
    uint8_t pin;  // Which pin
} led_config_t;

static const led_config_t LED_MAP[] = {
    [GREEN]  = {1, 0}, //PB0
    [YELLOW] = {4, 1}, //PE1
    [RED]    = {1, 14} //PB14
};

void led_init(int led) {
    // Set AHB4 bus clock
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

void toggle_led(int led) {
    TOGL_FIELD(GPIOx_ODR[LED_MAP[led].port], GPIOx_ODR_ODx[LED_MAP[led].pin]);
}

void led_countdown(int time) {
    if (READ_FIELD(STK_CSR, STK_CSR_ENABLE) == 0) return;

    int time_ms = time * 1000;

    toggle_led(GREEN);
    systick_delay(time_ms);
    toggle_led(YELLOW);
    systick_delay(time_ms);
    toggle_led(RED);
    systick_delay(time_ms);

    toggle_led(RED);
    toggle_led(YELLOW);
    toggle_led(GREEN);
}