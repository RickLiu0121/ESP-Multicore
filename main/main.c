#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_adc/adc_continuous.h"
// #include "driver/adc.h"


#define CORE0 0
#define CORE1 1
#define BUF_SIZE 1024
#define RING_BUF_SIZE 20
#define GPIO_OUTPUT_IO_35   CONFIG_GPIO_OUTPUT_35
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_OUTPUT_IO_35)
//pre_compilation declarations for ADC

#define ADC_UNIT  ADC_UNIT_1
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_ATTEN  ADC_ATTEN_DB_0
#define ADC_BIT_WIDTH  SOC_ADC_DIGI_MAX_BITWIDTH
#define READ_LEN_FROM_POOL 256

static const char* TAG = "adc_state";
static TaskHandle_t s_task_handle;
static volatile int isr_counter = 0;


//Declare Variables for Ring Buffer
static volatile int ring_buf[RING_BUF_SIZE];
static volatile int head = 0;
static volatile int tail = 0;
static float adc_avg = 0;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}
void adc_init() {
    //Continuously sample ADC1
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[READ_LEN_FROM_POOL] = {0};
    memset(result, 0xcc, READ_LEN_FROM_POOL);

    s_task_handle = xTaskGetCurrentTaskHandle();
    // static adc_channel_t channel[1] = {ADC_CHANNEL_7};
    // adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 8,
    .conv_frame_size = 4,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));
    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 20 * 1000,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
        .pattern_num = 1,
    };
    adc_digi_pattern_config_t adc_pattern =  {
        .atten = ADC_ATTEN,
        .channel = ADC_CHANNEL_7,
        .unit = ADC_UNIT,
        .bit_width = ADC_BIT_WIDTH,
    };

        // ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        // ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        // ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    
    dig_cfg.adc_pattern = &adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));
    // *out_handle = handle;
    char unit[] = "ADC_UNIT_1";
    int count = 0;
    int voltage = 0;

    while (1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ret = adc_continuous_read(handle, result, READ_LEN_FROM_POOL, &ret_num, 0);
        if (ret == ESP_OK) {
            ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);
            for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                uint32_t chan_num = p->type1.channel;
                uint32_t data = p->type2.data;
                voltage = data * 3300/4095;
                count++;
                /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
                // if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                ESP_LOGI(TAG, "Unit: %s, Channel: %"PRIu32", Value: %"PRIx32, unit, chan_num, data);
                printf("Vout is: %d \n", voltage);
                // } else {
                //     ESP_LOGW(TAG, "Invalid data [%s_%"PRIu32"_%"PRIx32"]", unit, chan_num, data);
                // }
            }
            printf("Numbers of data collected: %d", count);
            count = 0;
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else if (ret == ESP_ERR_TIMEOUT) {
            //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
            break;
        }
    }
    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));

}
static bool IRAM_ATTR onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void * user_Ctx) {
    isr_counter++;
    // printf("isr_counter value: %d", isr_counterx);
    if (!(isr_counter % 10)) 
        ESP_EARLY_LOGI(TAG, "isr_counter value: %d", isr_counter);    
    return true;
}

static void hw_timer(){
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    // Set the timer's alarm action
    // Register timer event callback functions, allowing user context to be carried
   
    gptimer_event_callbacks_t cbs = {
        .on_alarm = onTimer, // Call the user callback function when the alarm event occurs
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,      // When the alarm event occurs, the timer will automatically reload to 0
        .alarm_count = 1000000, // Set the actual alarm period, since the resolution is 1us, 1000000 represents 1s
        .flags.auto_reload_on_alarm = true, // Enable auto-reload function
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
   
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}
static void echo_task(void * arg){
    QueueHandle_t uart_queue;
    // ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 5, &uart_queue, 0));
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    static char cmd_buf[BUF_SIZE];
    static size_t idx = 0;
    char avg_command[] = "avg";
    bzero(cmd_buf, BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE - 1, 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            // data[len] = '\0';  // null-terminate so it becomes a C string
            // printf("%d",len);
            for (int i = 0; i < len; i++) {
                uint8_t ch = data[i];
                // printf((char *)ch);
                if (idx < BUF_SIZE - 1) {
                    cmd_buf[idx] = (char)ch;
                    idx++;
                }
                if (ch == 0x0D) {
                    printf("\n");
                    cmd_buf[idx-1] = '\0';
                    //echo back the entire string
                    // uart_write_bytes(UART_NUM_0, (const char *) &cmd_buf, idx);
                    // printf("\n");
                    if (strcmp(cmd_buf, avg_command) == 0) {
                        printf("Average: \n");
                    }
                    bzero(cmd_buf, BUF_SIZE);
                    idx = 0;
                } else {
                    uart_write_bytes(UART_NUM_0, (const char *) &ch, sizeof(uint8_t));
                }
            }
        }
    }

}

void app_main(void)
{
    // ESP_ERROR_CHECK(nvs_flash_init());
    

    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     ESP_LOGI(TAG, "Main Task Alive");
    // }
    // gpio_task();
    adc_init();

    xTaskCreate(echo_task, "uart_echo_task", CONFIG_EXAMPLE_TASK_STACK_SIZE, NULL, 10, NULL);

};
