


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

// --- Audio circular buffer for RX->TX ---
#define AUDIO_CIRC_BUF_SIZE (32*1024) // 32KB, adjust as needed
static uint8_t audio_circ_buf[AUDIO_CIRC_BUF_SIZE];
static volatile size_t audio_circ_head = 0;
static volatile size_t audio_circ_tail = 0;
static volatile size_t audio_circ_count = 0;
SemaphoreHandle_t audio_circ_mutex;

// --- FreeRTOS task handles ---
TaskHandle_t i2s_rx_task_handle = NULL;
TaskHandle_t i2s_tx_task_handle = NULL;

void i2s_rx_task(void *param);
void i2s_tx_task(void *param);

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
    audio_circ_mutex = xSemaphoreCreateMutex();
    // Start RX and TX tasks (core 0 for RX, core 1 for TX)
    xTaskCreatePinnedToCore(i2s_rx_task, "i2s_rx_task", 4096, NULL, 2, &i2s_rx_task_handle, 0);
    xTaskCreatePinnedToCore(i2s_tx_task, "i2s_tx_task", 4096, NULL, 2, &i2s_tx_task_handle, 1);
}

void loop() {
    // Non-blocking: all audio handled in FreeRTOS tasks
    delay(10);
}

// --- I2S RX Task: Reads from I2S1 (ADC/PCM1808) and puts into circular buffer ---
void i2s_rx_task(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t rx_buf[2048];
    size_t bytes_read = 0;
    while (1) {
        esp_err_t res = i2s_read(I2S_NUM_1, rx_buf, sizeof(rx_buf), &bytes_read, 10);
        if (res == ESP_OK && bytes_read > 0) {
            xSemaphoreTake(audio_circ_mutex, portMAX_DELAY);
            size_t space = AUDIO_CIRC_BUF_SIZE - audio_circ_count;
            size_t to_copy = (bytes_read < space) ? bytes_read : space;
            if (to_copy > 0) {
                size_t first_chunk = AUDIO_CIRC_BUF_SIZE - audio_circ_head;
                if (to_copy <= first_chunk) {
                    memcpy(&audio_circ_buf[audio_circ_head], rx_buf, to_copy);
                } else {
                    memcpy(&audio_circ_buf[audio_circ_head], rx_buf, first_chunk);
                    memcpy(&audio_circ_buf[0], rx_buf + first_chunk, to_copy - first_chunk);
                }
                audio_circ_head = (audio_circ_head + to_copy) % AUDIO_CIRC_BUF_SIZE;
                audio_circ_count += to_copy;
            }
            xSemaphoreGive(audio_circ_mutex);
        }
         vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1)); // min 1ms before calling to this function.
    }
}

// --- I2S TX Task: Reads from circular buffer and writes to I2S0 (DAC/PCM5202) ---
void i2s_tx_task(void *param) {
    uint8_t tx_buf[2048];
    size_t bytes_written = 0;
    while (1) {
        xSemaphoreTake(audio_circ_mutex, portMAX_DELAY);
        size_t to_copy = (audio_circ_count < sizeof(tx_buf)) ? audio_circ_count : sizeof(tx_buf);
        if (to_copy > 0) {
            size_t first_chunk = AUDIO_CIRC_BUF_SIZE - audio_circ_tail;
            if (to_copy <= first_chunk) {
                memcpy(tx_buf, &audio_circ_buf[audio_circ_tail], to_copy);
            } else {
                memcpy(tx_buf, &audio_circ_buf[audio_circ_tail], first_chunk);
                memcpy(tx_buf + first_chunk, &audio_circ_buf[0], to_copy - first_chunk);
            }
            audio_circ_tail = (audio_circ_tail + to_copy) % AUDIO_CIRC_BUF_SIZE;
            audio_circ_count -= to_copy;
        }
        xSemaphoreGive(audio_circ_mutex);
        if (to_copy > 0) {
            i2s_write(I2S_NUM_0, tx_buf, to_copy, &bytes_written, 10);
        } else {
            vTaskDelay(1); // No data, yield
        }
    }
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
        .dma_buf_count = 2,  // 
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
        .dma_buf_count = 2,
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