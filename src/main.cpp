#include <Arduino.h>
#include "driver/i2s.h"
#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"

// #define CLOCK_GPIO    GPIO_NUM_0
#define MCPWM_UNIT    MCPWM_UNIT_0
#define MCPWM_TIMER   MCPWM_TIMER_0
#define MCPWM_OP      MCPWM_OPR_A



#define SCK_OUT 4
#define UART_RX_PIN 15
#define UART_TX_PIN 2

// #define CLOCK_GPIO    GPIO_NUM_0
#define MCPWM_UNIT    MCPWM_UNIT_0
#define MCPWM_TIMER   MCPWM_TIMER_0
#define MCPWM_OP      MCPWM_OPR_A
// PCM1808 control pins
#define PCM1808_FORMAT_PIN 16
#define PCM1808_MD1_PIN    17
#define PCM1808_MD0_PIN    5

// I2S1 (ADC/PCM1808) pin macros
#define I2S1_BCK_PIN   18
#define I2S1_WS_PIN    21
#define I2S1_DIN_PIN   19

// I2S0 (DAC/PCM5202) pin macros
#define I2S0_BCK_PIN   25
#define I2S0_WS_PIN    27
#define I2S0_DOUT_PIN  26


void setsck();
void setadcios();
void configure_i2s1_slave_rx_pcm1808();
void configure_i2s0_master_tx_pcm5202();
// Define I2S port
static const i2s_port_t I2S_PORT = I2S_NUM_0;



// --- Audio frame and buffer macros ---
#define I2S_FRAME_SIZE      1024
#define FRAME_HEADER_SIZE   4
#define FRAME_CHECKSUM_SIZE 1
#define AUDIO_FRAME_TOTAL_SIZE (FRAME_HEADER_SIZE + I2S_FRAME_SIZE + FRAME_CHECKSUM_SIZE)
#define AUDIO_TX_CIRC_BUF_SIZE (AUDIO_FRAME_TOTAL_SIZE * 8) // 8 frames in TX buffer
#define AUDIO_RX_CIRC_BUF_SIZE (I2S_FRAME_SIZE * 8) // 8 frames in RX buffer

// TX buffer: I2S RX -> UART TX
static uint8_t audio_tx_circ_buf[AUDIO_TX_CIRC_BUF_SIZE];
static volatile size_t audio_tx_circ_head = 0;
static volatile size_t audio_tx_circ_tail = 0;
static volatile size_t audio_tx_circ_count = 0;
SemaphoreHandle_t audio_tx_circ_mutex;

// RX buffer: UART RX -> I2S TX
static uint8_t audio_rx_circ_buf[AUDIO_RX_CIRC_BUF_SIZE];
static volatile size_t audio_rx_circ_head = 0;
static volatile size_t audio_rx_circ_tail = 0;
static volatile size_t audio_rx_circ_count = 0;
SemaphoreHandle_t audio_rx_circ_mutex;

// --- FreeRTOS task handles ---
TaskHandle_t i2s_rx_task_handle = NULL;
TaskHandle_t uart_tx_task_handle = NULL;
TaskHandle_t uart_rx_task_handle = NULL;
TaskHandle_t i2s_tx_task_handle = NULL;

void i2s_rx_task(void *param);
void uart_tx_task(void *param);
void uart_rx_task(void *param);
void i2s_tx_task(void *param);

// Move large buffers to static/global to avoid stack overflow
static uint8_t i2s_rx_task_rx_buf[I2S_FRAME_SIZE];
static uint8_t i2s_rx_task_frame_buf[AUDIO_FRAME_TOTAL_SIZE];
static uint8_t uart_tx_task_frame_buf[AUDIO_FRAME_TOTAL_SIZE];
static uint8_t uart_rx_task_uart_rx_buf[AUDIO_FRAME_TOTAL_SIZE * 2];
static uint8_t uart_rx_task_frame_buf[AUDIO_FRAME_TOTAL_SIZE];
static uint8_t i2s_tx_task_tx_buf[I2S_FRAME_SIZE];

void setup() {
    setCpuFrequencyMhz(240); 
    Serial.begin(921600);
    Serial1.begin(1000000, SERIAL_8N1, -1, 2, false, AUDIO_FRAME_TOTAL_SIZE<<1);  // TX = GPIO2
    Serial2.begin(1000000, SERIAL_8N1, 15, -1, false, AUDIO_FRAME_TOTAL_SIZE<<1); // RX = GPIO15
    delay(50); // Allow serial and system to settle
    setsck();
    setadcios();
    configure_i2s1_slave_rx_pcm1808();
    configure_i2s0_master_tx_pcm5202();
    audio_tx_circ_mutex = xSemaphoreCreateMutex();
    audio_rx_circ_mutex = xSemaphoreCreateMutex();
    // Start tasks: RX (core 0), UART TX (core 1), UART RX (core 1), I2S TX (core 0)
    xTaskCreatePinnedToCore(i2s_rx_task, "i2s_rx_task", 8192, NULL, 2, &i2s_rx_task_handle, 0);
    xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 8192, NULL, 2, &uart_tx_task_handle, 1);
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 8192, NULL, 2, &uart_rx_task_handle, 1);
    xTaskCreatePinnedToCore(i2s_tx_task, "i2s_tx_task", 8192, NULL, 2, &i2s_tx_task_handle, 0);
}

void loop() {
    // Non-blocking: all audio handled in FreeRTOS tasks
    delay(10);
}
    size_t bytes_read = 0;
// --- I2S RX Task: Reads from I2S1, frames, and puts into TX buffer ---
void i2s_rx_task(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    static uint32_t frame_seq = 0;
    while (1) {
        esp_err_t res = i2s_read(I2S_NUM_1, i2s_rx_task_rx_buf, I2S_FRAME_SIZE, &bytes_read, 16);
        if (res == ESP_OK && bytes_read == I2S_FRAME_SIZE) {
            // Build header (4 bytes: 0xA5, 0x5A, seq_hi, seq_lo)
            i2s_rx_task_frame_buf[0] = 0xA5;
            i2s_rx_task_frame_buf[1] = 0x5A;
            i2s_rx_task_frame_buf[2] = (frame_seq >> 8) & 0xFF;
            i2s_rx_task_frame_buf[3] = frame_seq & 0xFF;
            memcpy(&i2s_rx_task_frame_buf[FRAME_HEADER_SIZE], i2s_rx_task_rx_buf, I2S_FRAME_SIZE);
            // Compute checksum (simple XOR of all data bytes)
            uint8_t cksum = 0;
            for (size_t i = 0; i < I2S_FRAME_SIZE; ++i) cksum ^= i2s_rx_task_rx_buf[i];
            i2s_rx_task_frame_buf[FRAME_HEADER_SIZE + I2S_FRAME_SIZE] = cksum;

            xSemaphoreTake(audio_tx_circ_mutex, portMAX_DELAY);
            size_t space = AUDIO_TX_CIRC_BUF_SIZE - audio_tx_circ_count;
            if (space >= AUDIO_FRAME_TOTAL_SIZE) {
                size_t first_chunk = AUDIO_TX_CIRC_BUF_SIZE - audio_tx_circ_head;
                if (AUDIO_FRAME_TOTAL_SIZE <= first_chunk) {
                    memcpy(&audio_tx_circ_buf[audio_tx_circ_head], i2s_rx_task_frame_buf, AUDIO_FRAME_TOTAL_SIZE);
                } else {
                    memcpy(&audio_tx_circ_buf[audio_tx_circ_head], i2s_rx_task_frame_buf, first_chunk);
                    memcpy(&audio_tx_circ_buf[0], i2s_rx_task_frame_buf + first_chunk, AUDIO_FRAME_TOTAL_SIZE - first_chunk);
                }
                audio_tx_circ_head = (audio_tx_circ_head + AUDIO_FRAME_TOTAL_SIZE) % AUDIO_TX_CIRC_BUF_SIZE;
                audio_tx_circ_count += AUDIO_FRAME_TOTAL_SIZE;
                frame_seq++;
            }
            xSemaphoreGive(audio_tx_circ_mutex);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}



// --- UART TX Task: Reads from TX buffer and sends to Serial1 ---
void uart_tx_task(void *param) {
    while (1) {
        xSemaphoreTake(audio_tx_circ_mutex, portMAX_DELAY);
        size_t available = audio_tx_circ_count;
        if (available >= AUDIO_FRAME_TOTAL_SIZE) {
            size_t first_chunk = AUDIO_TX_CIRC_BUF_SIZE - audio_tx_circ_tail;
            if (AUDIO_FRAME_TOTAL_SIZE <= first_chunk) {
                memcpy(uart_tx_task_frame_buf, &audio_tx_circ_buf[audio_tx_circ_tail], AUDIO_FRAME_TOTAL_SIZE);
            } else {
                memcpy(uart_tx_task_frame_buf, &audio_tx_circ_buf[audio_tx_circ_tail], first_chunk);
                memcpy(uart_tx_task_frame_buf + first_chunk, &audio_tx_circ_buf[0], AUDIO_FRAME_TOTAL_SIZE - first_chunk);
            }
            audio_tx_circ_tail = (audio_tx_circ_tail + AUDIO_FRAME_TOTAL_SIZE) % AUDIO_TX_CIRC_BUF_SIZE;
            audio_tx_circ_count -= AUDIO_FRAME_TOTAL_SIZE;
        }
        xSemaphoreGive(audio_tx_circ_mutex);
        // release semaphore and do uart write task
        if (available >= AUDIO_FRAME_TOTAL_SIZE) {
            size_t uart_sent = 0;
            while (uart_sent < AUDIO_FRAME_TOTAL_SIZE) {
                int n = Serial1.write(uart_tx_task_frame_buf + uart_sent, AUDIO_FRAME_TOTAL_SIZE - uart_sent);
                if (n > 0) uart_sent += n;
                else vTaskDelay(1);
            }
        } else {
            vTaskDelay(1);
        }
    }
}

             // unsigned long start_time = micros();
            // unsigned long end_time = micros();    
            // Serial.printf("Execution time: %lu us, b: %d\n", end_time - start_time, bytes_read);

// --- UART RX Task: Reads frames from Serial2, validates, and puts audio data into RX buffer ---
void uart_rx_task(void *param) {
    size_t uart_rx_len = 0;
    while (1) {
        // Read available bytes from Serial2

        size_t free_space = sizeof(uart_rx_task_uart_rx_buf) - uart_rx_len;
            int avail = Serial2.available();

            if (avail > 0 && free_space > 0) {
                size_t to_read = (avail < free_space) ? avail : free_space;
                
                // unsigned long start_time = micros();
                int bytes_read = Serial2.readBytes(uart_rx_task_uart_rx_buf + uart_rx_len, to_read);
                // unsigned long end_time = micros();

                uart_rx_len += bytes_read;

                // Serial.printf("[UART RX] Read %d bytes in %lu us → %.2f us/byte, Total: %d\n",
                //     bytes_read, end_time - start_time,
                //     bytes_read ? (float)(end_time - start_time) / bytes_read : 0.0,
                //     uart_rx_len);
            }
            // Try to extract frames
        if (uart_rx_len > 0) {
            // Serial.printf("[UART RX] Collected %d bytes\n", uart_rx_len);
        }

        while (uart_rx_len >= AUDIO_FRAME_TOTAL_SIZE) {
            // Look for header
            int frame_start = -1;
            for (size_t i = 0; i <= uart_rx_len - AUDIO_FRAME_TOTAL_SIZE; ++i) {
                if (uart_rx_task_uart_rx_buf[i] == 0xA5 && uart_rx_task_uart_rx_buf[i+1] == 0x5A) {
                    frame_start = i;
                    // Serial.println("frame syncd");
                    break;
                }
            }
            if (frame_start < 0) {
                // No header found, discard old bytes
                uart_rx_len = 0;
                Serial.println("no header");
                break;
            }
            if (uart_rx_len - frame_start < AUDIO_FRAME_TOTAL_SIZE) {
                // Not enough data for a full frame
                if (frame_start > 0) {
                    memmove(uart_rx_task_uart_rx_buf, uart_rx_task_uart_rx_buf + frame_start, uart_rx_len - frame_start);
                    uart_rx_len -= frame_start;
                }
                break;
            }
            memcpy(uart_rx_task_frame_buf, uart_rx_task_uart_rx_buf + frame_start, AUDIO_FRAME_TOTAL_SIZE);
            // Validate checksum
            uint8_t cksum = 0;
            for (size_t i = 0; i < I2S_FRAME_SIZE; ++i){ 
                cksum ^= uart_rx_task_frame_buf[FRAME_HEADER_SIZE + i];
            }
            if (uart_rx_task_frame_buf[FRAME_HEADER_SIZE + I2S_FRAME_SIZE] == cksum) {
                // Valid frame: put audio data into RX buffer
                // Serial.println("checksum OK");
                xSemaphoreTake(audio_rx_circ_mutex, portMAX_DELAY);
                size_t space = AUDIO_RX_CIRC_BUF_SIZE - audio_rx_circ_count;
                if (space >= I2S_FRAME_SIZE) {
                    size_t first_chunk = AUDIO_RX_CIRC_BUF_SIZE - audio_rx_circ_head;
                    if (I2S_FRAME_SIZE <= first_chunk) {
                        memcpy(&audio_rx_circ_buf[audio_rx_circ_head], uart_rx_task_frame_buf + FRAME_HEADER_SIZE, I2S_FRAME_SIZE);
                    } else {
                        memcpy(&audio_rx_circ_buf[audio_rx_circ_head], uart_rx_task_frame_buf + FRAME_HEADER_SIZE, first_chunk);
                        memcpy(&audio_rx_circ_buf[0], uart_rx_task_frame_buf + FRAME_HEADER_SIZE + first_chunk, I2S_FRAME_SIZE - first_chunk);
                    }
                    audio_rx_circ_head = (audio_rx_circ_head + I2S_FRAME_SIZE) % AUDIO_RX_CIRC_BUF_SIZE;
                    audio_rx_circ_count += I2S_FRAME_SIZE;
                }
                xSemaphoreGive(audio_rx_circ_mutex);
            }
            else{
                 Serial.println("checksum NOK");
            }
            // Remove processed frame from buffer
            size_t remove_len = frame_start + AUDIO_FRAME_TOTAL_SIZE;
            uart_rx_len -= remove_len;
            if (uart_rx_len > 0) memmove(uart_rx_task_uart_rx_buf, uart_rx_task_uart_rx_buf + remove_len, uart_rx_len);
        }
        vTaskDelay(1);
    }
}

// --- I2S TX Task: Reads audio data from RX buffer and writes to I2S0 (DAC/PCM5202) ---
void i2s_tx_task(void *param) {
    while (1) {
        xSemaphoreTake(audio_rx_circ_mutex, portMAX_DELAY);
        size_t available = audio_rx_circ_count;
        if (available >= I2S_FRAME_SIZE) {
            size_t first_chunk = AUDIO_RX_CIRC_BUF_SIZE - audio_rx_circ_tail;
            if (I2S_FRAME_SIZE <= first_chunk) {
                memcpy(i2s_tx_task_tx_buf, &audio_rx_circ_buf[audio_rx_circ_tail], I2S_FRAME_SIZE);
            } else {
                memcpy(i2s_tx_task_tx_buf, &audio_rx_circ_buf[audio_rx_circ_tail], first_chunk);
                memcpy(i2s_tx_task_tx_buf + first_chunk, &audio_rx_circ_buf[0], I2S_FRAME_SIZE - first_chunk);
            }
            audio_rx_circ_tail = (audio_rx_circ_tail + I2S_FRAME_SIZE) % AUDIO_RX_CIRC_BUF_SIZE;
            audio_rx_circ_count -= I2S_FRAME_SIZE;
        }
        xSemaphoreGive(audio_rx_circ_mutex);
        if (available >= I2S_FRAME_SIZE) {
            size_t bytes_written = 0;
            i2s_write(I2S_NUM_0, i2s_tx_task_tx_buf, I2S_FRAME_SIZE, &bytes_written, 10);
        } else {
            vTaskDelay(1);
        }
    }
}

void setsck(){
    mcpwm_group_set_resolution(MCPWM_UNIT_0, 160000000); // 16 MHz
    mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, 160000000); // 16 MHz
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_4);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 2000000;       // Set frequency to 2 MHz
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
        .sample_rate =7812,/// it must be clock/256
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,  // 
        .dma_buf_len = 64, // 64 frames per buffer (adjust as needed)
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
        .sample_rate = 7812, /// it must be clock/256
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 64, // 64 frames per buffer (adjust as needed)
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