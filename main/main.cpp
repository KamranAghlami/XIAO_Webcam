#include "driver/gpio.h"
#include "driver/i2s_pdm.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_device_uac.h"

constexpr const gpio_num_t BTN_GPIO = GPIO_NUM_0;
constexpr const gpio_num_t LED_GPIO = GPIO_NUM_21;

static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    auto rx_handle = static_cast<i2s_chan_handle_t>(arg);

    return i2s_channel_read(rx_handle, buf, len, bytes_read, 20);
}

extern "C" void app_main(void)
{
    gpio_reset_pin(BTN_GPIO);
    gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_GPIO, GPIO_PULLUP_ONLY);

    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0x01);

    while (!gpio_get_level(BTN_GPIO))
        vTaskDelay(pdMS_TO_TICKS(10));

    i2s_chan_handle_t rx_handle = nullptr;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

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

    uac_device_config_t config{
        .skip_tinyusb_init = false,
        .output_cb = nullptr,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = nullptr,
        .set_volume_cb = nullptr,
        .cb_ctx = rx_handle,
    };

    ESP_ERROR_CHECK(uac_device_init(&config));

    while (true)
    {
        if (!gpio_get_level(BTN_GPIO))
        {
            gpio_set_level(LED_GPIO, 0x00);

            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(i2s_channel_disable(rx_handle));
    ESP_ERROR_CHECK(i2s_del_channel(rx_handle));

    esp_restart();
}
