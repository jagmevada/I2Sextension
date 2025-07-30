
#include <Arduino.h>
#include "driver/i2s.h"
#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"

#define CLOCK_GPIO    GPIO_NUM_0
#define MCPWM_UNIT    MCPWM_UNIT_0
#define MCPWM_TIMER   MCPWM_TIMER_0
#define MCPWM_OP      MCPWM_OPR_A
// PCM1808 control pins
#define PCM1808_FORMAT_PIN 5
#define PCM1808_MD1_PIN    17
#define PCM1808_MD0_PIN    16

void setsck(){
    mcpwm_group_set_resolution(MCPWM_UNIT_0, 80000000); // 16 MHz
    mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, 80000000); // 16 MHz
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_4);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 2051000;       // Set frequency to 2 MHz
    pwm_config.cmpr_a = 50.0;             // Set duty cycle to 50%
    pwm_config.cmpr_b = 50.0;             // Not used for single output but required
    pwm_config.counter_mode = MCPWM_UP_COUNTER; // Up counter mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;   // Active high duty
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    Serial.println("pwm:");
    Serial.print( mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
    Serial.println("Hz");
}

void setadcios(){
        // Initialize PCM1808 control pins
    pinMode(PCM1808_FORMAT_PIN, OUTPUT);
    pinMode(PCM1808_MD1_PIN, OUTPUT);
    pinMode(PCM1808_MD0_PIN, OUTPUT);
    // Set initial states (FORMAT=LOW, MD1=HIGH, MD0=HIGH)
    digitalWrite(PCM1808_FORMAT_PIN, LOW); // I2S format
    digitalWrite(PCM1808_MD1_PIN, HIGH);
    digitalWrite(PCM1808_MD0_PIN, HIGH);
    delay(10); // Allow pins to settle
}

// Define I2S port
static const i2s_port_t I2S_PORT = I2S_NUM_0;
    mcpwm_config_t pwm_config;
void setup() {
    setCpuFrequencyMhz(240); 
    Serial.begin(115200);
    delay(50); // Allow serial and system to settle
    setsck();
    setadcios();
}

void loop() {
    // Your main application code goes here
    delay(1000); 
}
