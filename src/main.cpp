


#include <Arduino.h>
#include "driver/i2s.h"
#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"

// #define CLOCK_GPIO    GPIO_NUM_0
#define MCPWM_UNIT    MCPWM_UNIT_0
#define MCPWM_TIMER   MCPWM_TIMER_0
#define MCPWM_OP      MCPWM_OPR_A
// PCM1808 control pins
#define PCM1808_FORMAT_PIN 5
#define PCM1808_MD1_PIN    17
#define PCM1808_MD0_PIN    16

// I2S1 (ADC/PCM1808) pin macros
#define I2S1_BCK_PIN   21
#define I2S1_WS_PIN    18
#define I2S1_DIN_PIN   19

// I2S0 (DAC/PCM5202) pin macros
#define I2S0_BCK_PIN   27
#define I2S0_WS_PIN    25
#define I2S0_DOUT_PIN  26

void setsck();
void setadcios();
void configure_i2s1_slave_rx_pcm1808();
void configure_i2s0_master_tx_pcm5202();
// Define I2S port
static const i2s_port_t I2S_PORT = I2S_NUM_0;

void setup() {
    setCpuFrequencyMhz(240); 
    Serial.begin(115200);
    Serial1.begin(2500000 , SERIAL_8N1, -1, 2); // baud, config, RX=-1 (not used), TX=GPIO0
    Serial2.begin(2500000, SERIAL_8N1, 15, -1); // baud, config, RX=GPIO15, TX not used
    delay(50); // Allow serial and system to settle
    setsck();
    setadcios();
    configure_i2s1_slave_rx_pcm1808();
    configure_i2s0_master_tx_pcm5202();
}

void loop() {
    Serial2.print("UUUUUUUUUUU");
    // I2S loopback: read from I2S1 (ADC) and write to I2S0 (DAC)
    static uint8_t audio_buf[8192]; // 256 bytes = 32 stereo frames (32-bit slot)
    size_t bytes_read = 0, bytes_written = 0;
    // Read from I2S1 (slave, PCM1808 ADC)
    esp_err_t res = i2s_read(I2S_NUM_1, audio_buf, sizeof(audio_buf), &bytes_read, 10);
    i2s_write(I2S_NUM_0, audio_buf, bytes_read, &bytes_written,10);
    // if (res == ESP_OK && bytes_read > 0) {
    //     // Write to I2S0 (master, PCM5202 DAC)
    //     i2s_write(I2S_NUM_0, audio_buf, bytes_read, &bytes_written,1);
    //     Serial.println(bytes_read);
    // }
    // else{
    //     Serial1.println("nok");
    // }
}

void setsck(){
    mcpwm_group_set_resolution(MCPWM_UNIT_0, 160000000); // 16 MHz
    mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, 160000000); // 16 MHz
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_4);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 8000000;       // Set frequency to 2 MHz
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

// Configure I2S_NUM_1 as slave RX for PCM1808 (8kHz, 512kHz BCK, 24-bit data, 32-bit slot, stereo)
void configure_i2s1_slave_rx_pcm1808() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
        .sample_rate =31250,/// it must be clock/256
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 30,  // 
        .dma_buf_len = 512, // 64 frames per buffer (adjust as needed)
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        .bits_per_chan = I2S_BITS_PER_CHAN_32BIT
    }; 

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S1_BCK_PIN,
        .ws_io_num = I2S1_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S1_DIN_PIN
    };

    static const int i2s_num = 0; // i2s port number
    i2s_driver_install(I2S_NUM_1, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_1);

}

void configure_i2s0_master_tx_pcm5202() {

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 31250, /// it must be clock/256
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 30,
        .dma_buf_len = 512, // 64 frames per buffer (adjust as needed)
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        .bits_per_chan = I2S_BITS_PER_CHAN_32BIT
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S0_BCK_PIN,
        .ws_io_num = I2S0_WS_PIN,
        .data_out_num = I2S0_DOUT_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);
}