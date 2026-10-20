#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"  
#include "driver/gpio.h"        // v6.1 依然使用此头文件
#include "esp_timer.h"          // esp_timer API 在 v6.1 中保持稳定

#define PIN_BTN_BOOT    GPIO_NUM_9
#define PIN_WELD_OUT    GPIO_NUM_10

esp_timer_handle_t weld_timer;

void weld_off_callback(void* arg) {
    gpio_set_level(PIN_WELD_OUT, 0);
    printf("点焊结束 (50ms)，已关断 GPIO10\n");
}

void app_main(void)
{
    printf("开始点焊时序模拟实验 (ESP-IDF v6.1)...\n");

    // 1. 配置输出引脚
    gpio_config_t io_out_conf = {
        .pin_bit_mask = (1ULL << PIN_WELD_OUT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_out_conf);
    gpio_set_level(PIN_WELD_OUT, 0);

    // 2. 配置输入引脚 (BOOT 按键)
    gpio_config_t io_in_conf = {
        .pin_bit_mask = (1ULL << PIN_BTN_BOOT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE   // 使用轮询方式，暂不配置中断
    };
    gpio_config(&io_in_conf);

    // 3. 创建定时器
    const esp_timer_create_args_t timer_args = {
        .callback = &weld_off_callback,
        .name = "weld_timer"
    };
    esp_timer_create(&timer_args, &weld_timer);

    int last_btn_state = 1;
    bool is_welding = false;

    while (1) {
        int current_btn_state = gpio_get_level(PIN_BTN_BOOT);

        if (last_btn_state == 1 && current_btn_state == 0 && !is_welding) {
            printf("检测到脚踏按下！开启点焊输出...\n");
            gpio_set_level(PIN_WELD_OUT, 1);
            is_welding = true;
            esp_timer_start_once(weld_timer, 50000); // 50ms = 50000us
        }

        if (is_welding && !esp_timer_is_active(weld_timer)) {
            is_welding = false;
        }

        last_btn_state = current_btn_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}