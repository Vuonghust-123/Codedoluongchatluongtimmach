#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include <driver/i2s_types_legacy.h>
#include "max30102.h"

// ========== CONFIG ==========
#define UART_NUM UART_NUM_0
#define BUF_SIZE 1024

// ECG
#define ADC_CHANNEL ADC_CHANNEL_6 // GPIO34
#define ADC_UNIT ADC_UNIT_1
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ECG_SAMPLE_RATE 500

// PPG - MAX30102
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define PPG_SAMPLE_RATE 400

// PCG - INMP441
#define I2S_NUM I2S_NUM_0
#define I2S_SCK_PIN 32
#define I2S_WS_PIN 25
#define I2S_SD_PIN 33
#define PCG_SAMPLE_RATE 1500
#define dmaDesc 6
#define dmaLen 128

// ========== BIẾN TOÀN CỤC ==========
static const char *TAG = "HE_THONG";
esp_timer_handle_t ecg_timer = NULL;
esp_timer_handle_t ppg_timer = NULL;
TaskHandle_t pcg_task_handle = NULL;

i2c_dev_t max_dev;
struct max30102_record max_record;
i2s_chan_handle_t rx_channel = NULL;
int16_t buffer16[dmaLen * dmaDesc / sizeof(int32_t) * 3 / 2] = {0};
int32_t buffer32[dmaLen * dmaDesc / sizeof(int32_t)] = {0};

// ========== HÀM ĐO ECG ==========
void ecg_callback(void *arg) {
    int val = adc1_get_raw(ADC_CHANNEL);
    printf("%d\n", val);
}

void start_ecg() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    esp_timer_create_args_t timer_args = {
        .callback = &ecg_callback,
        .arg = NULL,
        .name = "ecg_timer"
    };
    esp_timer_create(&timer_args, &ecg_timer);
    esp_timer_start_periodic(ecg_timer, 1000000 / ECG_SAMPLE_RATE);
}

// ========== HÀM ĐO PPG ==========
void ppg_callback(void *arg) {
    max30102_check(&max_record, &max_dev);
    while (max30102_available(&max_record)) {
        unsigned long red = max30102_getFIFORed(&max_record);
        unsigned long ir = max30102_getFIFOIR(&max_record);
        printf("%ld,%ld\n", red, ir);
        max30102_nextSample(&max_record);
    }
}

void start_ppg() {
    i2cdev_init();
    memset(&max_dev, 0, sizeof(i2c_dev_t));
    max30102_initDesc(&max_dev, 0, I2C_SDA_GPIO, I2C_SCL_GPIO);
    if (max30102_readPartID(&max_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Khong tim thay MAX30102!");
        return;
    }
    max30102_init(0x2F, 4, 2, PPG_SAMPLE_RATE, 411, 16384, &max_record, &max_dev);
    max30102_clearFIFO(&max_dev);

    esp_timer_create_args_t timer_args = {
        .callback = &ppg_callback,
        .arg = NULL,
        .name = "ppg_timer"
    };
    esp_timer_create(&timer_args, &ppg_timer);
    esp_timer_start_periodic(ppg_timer, 1000000 / PPG_SAMPLE_RATE);
}

// ========== HÀM ĐO PCG ==========
void read_pcg_task(void *arg) {
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = dmaDesc,
        .dma_frame_num = dmaLen,
        .auto_clear = true,
    };
    i2s_new_channel(&chan_cfg, NULL, &rx_channel);

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = PCG_SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_1152,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
        },
        .gpio_cfg = {
            .bclk = I2S_SCK_PIN,
            .ws = I2S_WS_PIN,
            .din = I2S_SD_PIN,
            .dout = I2S_PIN_NO_CHANGE,
        }
    };
    i2s_channel_init_std_mode(rx_channel, &std_cfg);
    i2s_channel_enable(rx_channel);

    size_t bytes_read;
    while (true) {
        i2s_channel_read(rx_channel, &buffer32, sizeof(buffer32), &bytes_read, 100);
        int samples = bytes_read / sizeof(int32_t);
        for (int i = 0; i < samples; ++i) {
            buffer16[i] = (int16_t)(buffer32[i] >> 8);
            printf("%d\n", buffer16[i]);
        }
    }
}

void start_pcg() {
    xTaskCreatePinnedToCore(read_pcg_task, "read_pcg", 1024 * 10, NULL, 5, &pcg_task_handle, 1);
}

// ========== NHẬN INPUT UART ==========
void print_menu() {
    printf("\nNHAN PHIM 1-3 DE LUA CHON DO TIN HIEU:\n1. ECG\n2. PPG\n3. PCG\n");
}

void app_main(void) {
    // Cấu hình UART để nhận input từ bàn phím qua USB
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    print_menu();
    uint8_t data[2];

    while (true) {
        int len = uart_read_bytes(UART_NUM, data, 1, portMAX_DELAY);
        if (len > 0) {
            char c = data[0];
            if (c == '1') {
                printf("Dang bat dau do ECG...\n");
                start_ecg();
                break;
            } else if (c == '2') {
                printf("Dang bat dau do PPG...\n");
                start_ppg();
                break;
            } else if (c == '3') {
                printf("Dang bat dau do PCG...\n");
                start_pcg();
                break;
            } else {
                printf("Ban nhap khong hop le! Vui long nhap lai!\n1. ECG\n2. PPG\n3. PCG\n");
            }
        }
    }
}
