#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <esp_attr.h>
#include <soc/rtc.h>
#include <soc/mcpwm_periph.h>
#include <math.h>
#include <driver/mcpwm.h>

#define LO1 12  // GPIO12 PWM output pin Low Side
#define HO1 13  // GPIO13 PWM output pin High Side
#define LO2 14  // GPIO14 PWM output pin Low Side
#define HO2 27  // GPIO27 PWM output pin High Side

const float freqCarry = 20000.0; // 1kHz
const float freqMod = 50.0; // 50Hz
const int amplitude = 300;
const float sampleNum = (float) (freqCarry/freqMod);
int i;

void IRAM_ATTR MCPWM_ISR(void*) {
    float sineValue;
    int sineVal;

    WRITE_PERI_REG(0x33ff5e11c, BIT(3)); // Clear the interrupt flag

    sineValue = amplitude * sin(2 * 3.14159265359 * freqMod * i / freqCarry);
    sineVal = (int) (sineValue + 0.5); // Round the value
    if (sineVal > 0) {
        WRITE_PERI_REG(0x3ff5e040, sineVal); // Set the duty cycle
        gpio_set_level(LO2, 1); // Set the GPIO pin
    }
    if (sineVal < 0) {
        WRITE_PERI_REG(0x3ff5e040, 400 + sineVal); // Set the duty cycle
        gpio_set_level(HO2, 1); // Set the GPIO pin
    }
    if ( i==0 ) {
        WRITE_PERI_REG(0x3ff5e040, 0); // Set the duty cycle
        gpio_set_level(HO2, 0); // Set the GPIO pin
    }
    if ( i==sampleNum/2 ) {
        WRITE_PERI_REG(0x3ff5e040, 400); // Set the duty cycle
        gpio_set_level(LO2, 0); // Set the GPIO pin
        //i = 0;
    }
    i++;
    if (i > sampleNum) {
        i = 0;
    }

}

void app_main() {
    gpio_set_direction(LO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(HO1, GPIO_MODE_OUTPUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, LO1);   // Set the GPIO pin for MCPWM0A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, HO1);   // Set the GPIO pin for MCPWM0B

    mcpwm_config_t pwm_config;
    pwm_config.frequency = freqCarry * 2;   // up-down counter, thus frequency is double
    pwm_config.cmpr_a = 0;                  // initial duty cycle
    pwm_config.cmpr_b = 0;                  // initial duty cycle
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; // Up-down counter
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0; // Active high duty cycle

    WRITE_PERI_REG(0x3FF5E000, 0x00000000); // Disable the low side driver

    float tmrRegVal = (((1/freqCarry)/2)/62.5E-9)-1; // (float) (80000000.0 / (pwm_config.frequency * sampleNum));
    WRITE_PERI_REG(0x3FF5E004, (int) tmrRegVal << 8); // Set the timer period
    WRITE_PERI_REG(0x3ff5e03c, 0x00000001); // Enable the timer

    // mcpwm_isr_register(MCPWM_UNIT_0, MCPWM_ISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 31, 31);
    WRITE_PERI_REG(0x3ff5e110, BIT(3)); // Clear the interrupt flag
}
