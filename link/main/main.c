#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BLINKER"

#define LED_IO 2
#define LED_RED 23

void app_main()
{
    gpio_config_t io_conf = {};

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_IO);
    io_conf.pin_bit_mask = (1ULL << LED_RED);
    // UTILIZADAS APENAS NOS INPUTS
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);

    for (int i = 0; i < 3; i++)
    {
        gpio_set_level(LED_IO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_IO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "VALOR DO I %d", i);
    }

}