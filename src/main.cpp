#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
// UART buffer and frame macros
#define UART_FRAME_SIZE 8192
#define UART_HEADER_32 0xA5B6C7D8u


// Mutex for uart_rx_buf (optional, for thread safety)
static SemaphoreHandle_t uart_rx_mutex = NULL;
// Mutex for I2S circular buffer
static SemaphoreHandle_t i2s_circ_mutex = NULL;

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

// UART buffer and frame macros
#define UART_FRAME_SIZE 8192
#define UART_HEADER_32 0xA5B6C7D8u
#define I2S_CIRC_BUF_SIZE (UART_FRAME_SIZE * 2)  // define once to avoid mistakes


size_t i2s_circ_head = 0, i2s_circ_count = 0;
uint8_t i2s_circ_buf[I2S_CIRC_BUF_SIZE] = {0};
static uint8_t uart_rx_buf[UART_FRAME_SIZE * 4];
static size_t uart_rx_buf_len = 0;



void setsck();
void setadcios();
void configure_i2s1_slave_rx_pcm1808();
void configure_i2s0_master_tx_pcm5202();
void uart_rx_timer_cb(TimerHandle_t xTimer);
void i2s_rx_task(void *pvParameters);

// Define I2S port
static const i2s_port_t I2S_PORT = I2S_NUM_0;




// Helper: Calculate simple 8-bit checksum
uint8_t calc_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += data[i];
    return sum;
}


// Helper: Find 32-bit frame header in buffer
int find_uart_frame(const uint8_t *buf, size_t len) {
    for (size_t i = 0; i + UART_FRAME_SIZE + 6 <= len; ++i) {
        uint32_t hdr = ((uint32_t)buf[i] << 24) | ((uint32_t)buf[i+1] << 16) | ((uint32_t)buf[i+2] << 8) | buf[i+3];
        if (hdr == UART_HEADER_32) return i;
    }
    return -1;
}




void setup() {
    // Create mutex for I2S circular buffer
    i2s_circ_mutex = xSemaphoreCreateMutex();
    // Create mutex for UART RX buffer
    uart_rx_mutex = xSemaphoreCreateMutex();
    // Start FreeRTOS timer for UART RX (every 1ms)

    setCpuFrequencyMhz(240); 
    Serial.begin(115200);
    Serial1.begin(2500000 , SERIAL_8N1, -1, 2, false, 8192); // baud, config, RX=-1 (not used), TX=GPIO0
    Serial2.begin(2500000, SERIAL_8N1, 15, -1, false, 8192); // baud, config, RX=GPIO15, TX not used
    delay(50); // Allow serial and system to settle
    setsck();
    setadcios();
    configure_i2s1_slave_rx_pcm1808();
    configure_i2s0_master_tx_pcm5202();

    TimerHandle_t uart_rx_timer = xTimerCreate("uart_rx_timer", pdMS_TO_TICKS(1), pdTRUE, NULL, uart_rx_timer_cb);
    xTimerStart(uart_rx_timer, 0);
        // Start I2S RX FreeRTOS task (last, after all init)
    xTaskCreatePinnedToCore(i2s_rx_task, "i2s_rx_task", 4096, NULL, 2, NULL, 1);
}

void loop() {
    // I2S1 -> Serial1 -> Serial2 -> I2S0 loopback
    static uint8_t uart_tx_buf[UART_FRAME_SIZE + 4 + 1];           // header + frame + checksum
    size_t bytes_written = 0;
    // 1. I2S RX and circular buffer accumulation is now handled by FreeRTOS task
    // 2. Send out all complete UART_FRAMEs in circular buffer
    
    /*
    if (i2s_circ_mutex && xSemaphoreTake(i2s_circ_mutex, 1) == pdTRUE) {
        while (i2s_circ_count >= UART_FRAME_SIZE) {
            for (size_t i = 0; i < UART_FRAME_SIZE; ++i) {
                uart_tx_buf[4 + i] = i2s_circ_buf[(i2s_circ_head + i) % I2S_CIRC_BUF_SIZE];
            }
            // Add header & checksum
            uint32_t header = UART_HEADER_32;
            memcpy(uart_tx_buf, &header, 4);
            uart_tx_buf[UART_FRAME_SIZE + 4] = calc_checksum(uart_tx_buf + 4, UART_FRAME_SIZE);
            Serial1.write(uart_tx_buf, UART_FRAME_SIZE + 5);

            i2s_circ_head = (i2s_circ_head + UART_FRAME_SIZE) % I2S_CIRC_BUF_SIZE;
            i2s_circ_count -= UART_FRAME_SIZE;
        }
        xSemaphoreGive(i2s_circ_mutex);
    }  
    */

    // 3. UART RX buffering is now handled by FreeRTOS timer
    // (nothing needed here)
/*
    // 4. Search for valid frame in uart_rx_buf (protected by mutex)
    if (uart_rx_mutex && xSemaphoreTake(uart_rx_mutex, 1) == pdTRUE) {
        int frame_start = find_uart_frame(uart_rx_buf, uart_rx_buf_len);
        while (frame_start >= 0 && uart_rx_buf_len - frame_start >= UART_FRAME_SIZE + 5) {
            uint8_t *frame = uart_rx_buf + frame_start;
            // Validate checksum
            uint8_t expected = calc_checksum(frame + 4, UART_FRAME_SIZE);
            uint8_t received = frame[UART_FRAME_SIZE + 4];
            if (received == expected) {
                Serial.println("\tfound");
                // 5. Valid frame: send to I2S0 (DAC)
                size_t bytes_written = 0;
                esp_err_t err = i2s_write(I2S_NUM_0, frame + 4, UART_FRAME_SIZE, &bytes_written, 10);
                if (err != ESP_OK || bytes_written != UART_FRAME_SIZE) {
                    Serial.printf("[I2S] Write failed or incomplete: err=%d, written=%d\n", err, (int)bytes_written);
                }
            } else {
                Serial.printf("[UART] Checksum mismatch: got=0x%02X, expected=0x%02X\n", received, expected);
            }
            // Remove processed bytes
            size_t remove_len = frame_start + UART_FRAME_SIZE + 5;
            if (remove_len <= uart_rx_buf_len) {
                memmove(uart_rx_buf, uart_rx_buf + remove_len, uart_rx_buf_len - remove_len);
                uart_rx_buf_len -= remove_len;
            } else {
                // Defensive fallback if something went wrong
                uart_rx_buf_len = 0;
                Serial.println("[WARN] Buffer reset due to invalid remove_len");
            }
            // Search again in updated buffer
            frame_start = find_uart_frame(uart_rx_buf, uart_rx_buf_len);
        }
        xSemaphoreGive(uart_rx_mutex);
    }
    */
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
        .dma_buf_count = 20,  // 
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
        .dma_buf_count = 20,
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


// Timer callback for UART RX buffering
void uart_rx_timer_cb(TimerHandle_t xTimer) {
    if (xSemaphoreTake(uart_rx_mutex, 1) == pdTRUE) {
        int avail = Serial2.available();
        if (avail > 0 && uart_rx_buf_len + avail < sizeof(uart_rx_buf)) {
            int n = Serial2.readBytes(uart_rx_buf + uart_rx_buf_len, avail);
            uart_rx_buf_len += n;
        }else {
            uart_rx_buf_len = 0; // flush corrupted buffer
            Serial.println("[WARN] UART RX buffer overflow!");
        }
        xSemaphoreGive(uart_rx_mutex);
    }
}


// FreeRTOS task for I2S read and circular buffer accumulation
void i2s_rx_task(void *pvParameters) {
    static uint8_t i2s_rx_buf[UART_FRAME_SIZE];
    for (;;) {
        size_t bytes_read = 0;
        esp_err_t res = i2s_read(I2S_NUM_1, i2s_rx_buf, sizeof(i2s_rx_buf), &bytes_read, 10);
        if (res == ESP_OK && bytes_read > 0) {
            if (i2s_circ_mutex && xSemaphoreTake(i2s_circ_mutex, 1) == pdTRUE) {

                for (size_t i = 0; i < bytes_read; ++i) {
                    if (i2s_circ_count < I2S_CIRC_BUF_SIZE) {
                        size_t tail = (i2s_circ_head + i2s_circ_count) % I2S_CIRC_BUF_SIZE;
                        i2s_circ_buf[tail] = i2s_rx_buf[i];
                        i2s_circ_count++;
                    }else {
                        Serial.println("[WARN] I2S buffer overflow!");
                    }
                }
                xSemaphoreGive(i2s_circ_mutex);
            }
        }
        vTaskDelay(1); // yield to other tasks
    }
}