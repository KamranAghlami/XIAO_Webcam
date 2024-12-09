#include <algorithm>

#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_device_uac.h"

#define UAC_ENABLE 1

constexpr const char *TAG = "XIAO_Webcam";
constexpr const gpio_num_t LED_GPIO = GPIO_NUM_21;

int16_t value = 0;

#if UAC_ENABLE
static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    auto rx_handle = static_cast<i2s_chan_handle_t>(arg);

    if (i2s_channel_read(rx_handle, buf, len, bytes_read, 100) == ESP_OK && *bytes_read > 4)
    {
        auto samples_begin = reinterpret_cast<int16_t *>(buf);
        auto samples_end = samples_begin + (*bytes_read / sizeof(int16_t));

        value = *std::max_element(samples_begin, samples_end) << 2;
    }
    else
        ESP_LOGW(TAG, "read failed!");

    return ESP_OK;
}
#endif

static void mic_task(void *arg)
{
    i2s_chan_handle_t rx_handle = nullptr;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle));

    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(CONFIG_MIC_SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = static_cast<gpio_num_t>(CONFIG_MIC_PIN_CLK),
            .din = static_cast<gpio_num_t>(CONFIG_MIC_PIN_DATA),
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

#if UAC_ENABLE
    uac_device_config_t config{
        .skip_tinyusb_init = false,
        .output_cb = nullptr,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = nullptr,
        .set_volume_cb = nullptr,
        .cb_ctx = rx_handle,
    };

    uac_device_init(&config);
#else
    uint8_t buffer[CONFIG_MIC_BIT_DEPTH * 512];
#endif

    while (true)
    {
#if !UAC_ENABLE
        size_t bytes_read = 0;

        if (i2s_channel_read(rx_handle, buffer, sizeof(buffer), &bytes_read, 100) == ESP_OK && bytes_read > 4)
        {
            auto samples_begin = reinterpret_cast<int16_t *>(&buffer[0]);
            auto samples_end = samples_begin + (bytes_read / sizeof(int16_t));

            value = *std::max_element(samples_begin, samples_end) << 2;
        }
        else
            ESP_LOGW(TAG, "read failed!");
#endif

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(i2s_channel_disable(rx_handle));
    ESP_ERROR_CHECK(i2s_del_channel(rx_handle));
}

extern "C" void app_main(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0x01);

    TaskHandle_t mic_task_handle = nullptr;

    xTaskCreatePinnedToCore(mic_task, "Microphone", 12U * 1024U, nullptr, 5, &mic_task_handle, !CONFIG_ESP_MAIN_TASK_AFFINITY);

    while (true)
    {
        fputs("\r\tamplitude: ", stdout);

        uint8_t i = 0;

        for (; i < (value / 1024); i++)
            putc('I', stdout);

        for (; i < 32; i++)
            putc(' ', stdout);

        fflush(stdout);

        gpio_set_level(LED_GPIO, value < 0x1fff);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
