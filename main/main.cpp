#include <memory>
#include <cstring>

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>
#include <usb_device_uac.h>

constexpr const gpio_num_t BTN_GPIO = GPIO_NUM_0;
constexpr const gpio_num_t LED_GPIO = GPIO_NUM_21;
constexpr const size_t sample_size = CONFIG_MIC_BIT_DEPTH / 8;
constexpr const size_t buffer_size = 2 * sample_size * CONFIG_MIC_SAMPLE_RATE / (1000 / CONFIG_UAC_MIC_INTERVAL_MS);

static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
{
    auto stream_buffer = static_cast<StreamBufferHandle_t>(arg);

    *bytes_read = xStreamBufferReceive(stream_buffer, buf, len, portMAX_DELAY);

    if (size_t sample_count = *bytes_read / 2)
    {
        auto samples_begin = reinterpret_cast<int16_t *>(buf);
        auto samples_end = samples_begin + sample_count;

        for (; samples_begin != samples_end; samples_begin++)
        {
            static int16_t prev_input = 0;
            static int16_t prev_output = 0;

            int16_t &sample = *samples_begin;
            int16_t temp = sample;

            sample = sample - prev_input + 0.99f * prev_output;

            prev_input = temp;
            prev_output = sample;
        }
    }

    return ESP_OK;
}

bool IRAM_ATTR on_receive(i2s_chan_handle_t handle, i2s_event_data_t *event, void *arg)
{
    auto stream_buffer = static_cast<StreamBufferHandle_t>(arg);
    auto higher_priority_task_woken = pdFALSE;

    xStreamBufferSendFromISR(stream_buffer, event->dma_buf, event->size, &higher_priority_task_woken);

    return higher_priority_task_woken;
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

    auto stream_buffer = xStreamBufferCreate(buffer_size, sample_size);

    uac_device_config_t config = {
        .skip_tinyusb_init = false,
        .output_cb = nullptr,
        .input_cb = uac_device_input_cb,
        .set_mute_cb = nullptr,
        .set_volume_cb = nullptr,
        .cb_ctx = stream_buffer,
    };

    ESP_ERROR_CHECK(uac_device_init(&config));

    i2s_chan_handle_t rx_handle = nullptr;
    const i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &rx_handle));

    const i2s_pdm_rx_config_t pdm_rx_cfg = {
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

    const i2s_event_callbacks_t event_callbacks = {
        .on_recv = on_receive,
        .on_recv_q_ovf = nullptr,
        .on_sent = nullptr,
        .on_send_q_ovf = nullptr,
    };

    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle, &event_callbacks, stream_buffer));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    gpio_set_level(LED_GPIO, 0x00);

    while (true)
    {
        if (!gpio_get_level(BTN_GPIO))
            break;

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    gpio_set_level(LED_GPIO, 0x01);

    ESP_ERROR_CHECK(i2s_channel_disable(rx_handle));
    ESP_ERROR_CHECK(i2s_del_channel(rx_handle));

    vStreamBufferDelete(stream_buffer);

    esp_restart();
}
