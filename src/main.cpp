// PCM1808 control pins
#define PCM1808_FORMAT_PIN 5
#define PCM1808_MD1_PIN    17
#define PCM1808_MD0_PIN    16
#include <Arduino.h>
#include "driver/i2s.h"

// Define I2S port
static const i2s_port_t I2S_PORT = I2S_NUM_0;

void setup() {
    Serial.begin(115200);
    delay(50); // Allow serial and system to settle

    // Initialize PCM1808 control pins
    pinMode(PCM1808_FORMAT_PIN, OUTPUT);
    pinMode(PCM1808_MD1_PIN, OUTPUT);
    pinMode(PCM1808_MD0_PIN, OUTPUT);

    // Set initial states (FORMAT=LOW, MD1=HIGH, MD0=HIGH)
    digitalWrite(PCM1808_FORMAT_PIN, LOW); // I2S format
    digitalWrite(PCM1808_MD1_PIN, HIGH);
    digitalWrite(PCM1808_MD0_PIN, HIGH);
    delay(10); // Allow pins to settle
    Serial.println("Starting I2S clock output on GPIO0...");

    // Define I2S configuration (can be minimal for just clock output)
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // Master mode for generating clock
        .sample_rate = 192000, // Example sample rate, adjust as needed
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0, // Default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = true // Optional: set to true for high accuracy clock
    };

    // Define I2S pin configuration, mapping BCLK to GPIO0
    i2s_pin_config_t pin_config = {
        .bck_io_num = GPIO_NUM_0, // SCK (BCLK) routed to GPIO0
        .ws_io_num = I2S_PIN_NO_CHANGE, 
        .data_out_num = I2S_PIN_NO_CHANGE, 
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    // Install and configure the I2S driver
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S driver: %d\n", err);
        return;
    }
    Serial.println("I2S driver installed.");

    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S pins: %d\n", err);
        return;
    }
    Serial.println("I2S pins set.");
    Serial.println("I2S SCK should now be outputting on GPIO0.");

    // You can optionally output a dummy stream to generate the clock
    // For example:
    // int16_t samples[64]; 
    // memset(samples, 0, sizeof(samples));
    // while(1) {
    //    i2s_write(I2S_PORT, samples, sizeof(samples), &bytes_written, portMAX_DELAY);
    // }
}

void loop() {
    // Your main application code goes here
    delay(1000); 
}
