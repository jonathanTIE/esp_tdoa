/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_timer.h"
#include <sys/time.h>

#define BUFF_ADC_LEN 256
#define GPIO_OUTPUT GPIO_NUM_4

static const char *TAG = "ADC_READ";
static TaskHandle_t s_task_handle;
static int64_t last_sent = 0;
static const int64_t interval = 100000; // microseconds -> 10 Hz

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = BUFF_ADC_LEN,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_digi_pattern_config_t pattern_config = {
        .atten = ADC_ATTEN_DB_0,
        .channel = ADC_CHANNEL_3,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    adc_continuous_config_t dig_cfg = {
         
        .pattern_num = 1,
        .sample_freq_hz = 20000,//131072, // 2^17 hz
        .adc_pattern = &pattern_config,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    *out_handle = handle;

}

static void gpio_emittor_init() {
    uint64_t GPIO_OUTPUT_PIN_SEL = (1ULL<<GPIO_OUTPUT);
     gpio_config_t io_conf = {
        //disable interrupt
        .intr_type = GPIO_INTR_DISABLE,
        //set as output mode
        .mode = GPIO_MODE_OUTPUT,
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        //disable pull-down mode
        .pull_down_en = 0,
        //disable pull-up mode
        .pull_up_en = 0,
     };
    //configure GPIO with the given settings
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static void uart_init() {
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
}
void app_main(void)
{
    //INIT GPIO
    gpio_emittor_init();

    //INIT UART


    // INIT ADC
    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(&handle);

    s_task_handle = xTaskGetCurrentTaskHandle();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    esp_err_t ret;
    uint8_t result[BUFF_ADC_LEN] = {0};
    uint32_t ret_num = 0;
    memset(result, 0xcc, BUFF_ADC_LEN);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (1) {
            if(esp_timer_get_time() - last_sent > interval) {
                //Trigger emittor on US distance ranger at 10Hz
                last_sent = esp_timer_get_time();
                gpio_set_level(GPIO_OUTPUT, 1);
                vTaskDelay(0.01 / portTICK_PERIOD_MS); // 0.01 ms -> 10 us
                gpio_set_level(GPIO_OUTPUT, 0);
                ESP_LOGI(TAG, "Triggered emittor");
            }
            ret = adc_continuous_read(handle, result, BUFF_ADC_LEN, &ret_num, 0);
                if (ret == ESP_OK) {
                    for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                        uint16_t adc_data = (p)->type1.data;
                        if(adc_data > 20) {
                            ESP_LOGI(TAG, " %"PRIx16, adc_data);
                        }

                    }
                    /**
                     * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                     * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                     * usually you don't need this delay (as this task will block for a while).
                     */
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                } 
        }
    }
}
