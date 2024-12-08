#include <cmath>
#include <numbers>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

constexpr const gpio_num_t LED_GPIO = GPIO_NUM_21;

extern "C" void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(LED_GPIO, 0x01);

    float angle = 0.0f;

    while (true)
    {
        float value = std::sin(angle);

        fputs("\r\tamplitude: ", stdout);

        uint8_t i = 0;

        for (; i < (16 + (value * 16)); i++)
            putc('I', stdout);

        for (; i < 32; i++)
            putc(' ', stdout);

        fflush(stdout);

        angle += 0.025f;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
