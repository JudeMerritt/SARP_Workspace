#include <stdint.h>
#pragma once

typedef enum {
    GREEN,
    YELLOW,
    RED
}led_t;

/**
 * @brief Prepares one of the user LEDs to be toggled on or off.
 * 
 * @param led the user led that you want to initialize:
 *            0 -> GREEN | 1 -> YELLOW | 2 -> RED
 */
void led_init(int led);

/**
 * @brief Toggles a specified LED on or off, depending on its current state.
 * 
 * @param led the user led that you want to toggle:
 *            0 -> GREEN | 1 -> YELLOW | 2 -> RED
 */
void toggle_led(int led);

/**
 * @brief Initiates a countdown using the GREEN, YELLOW, and RED LEDs. The LEDs will
 * turn on one at a time, then turn off simultaneously at the end of the countdown. 
 * led_countdown assumes that all LEDs are turned off when called, and will only work if the systick 
 * timer has been initialized. 
 * 
 * @param time specifies the amount of time in seconds that illapses between
 * LED flash. The total countdown time will be time * 3.
 */
void led_countdown(int time);