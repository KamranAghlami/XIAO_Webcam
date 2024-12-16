#include <memory>
#include <cstring>

#include <driver/gpio.h>
#include <driver/i2s_pdm.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include <usb_device_uac.h>

#include "ring_buffer.h"

constexpr const char *TAG = "main";
constexpr const gpio_num_t BTN_GPIO = GPIO_NUM_0;
constexpr const gpio_num_t LED_GPIO = GPIO_NUM_21;

using buffer_type = ring_buffer<10 * (CONFIG_MIC_BIT_DEPTH / 8) * CONFIG_MIC_SAMPLE_RATE / (1000 / CONFIG_UAC_MIC_INTERVAL_MS)>;

// static esp_err_t uac_device_input_cb(uint8_t *buf, size_t len, size_t *bytes_read, void *arg)
// {
//     const auto buffer = static_cast<buffer_type *>(arg);

//     *bytes_read = buffer->read(buf, len);

//     if (size_t sample_count = *bytes_read / 2)
//     {
//         auto samples_begin = reinterpret_cast<int16_t *>(buf);
//         auto samples_end = samples_begin + sample_count;

//         for (; samples_begin != samples_end; samples_begin++)
//         {
//             static int16_t prev_input = 0;
//             static int16_t prev_output = 0;

//             int16_t &sample = *samples_begin;
//             int16_t temp = sample;

//             sample = sample - prev_input + 0.99f * prev_output;

//             prev_input = temp;
//             prev_output = sample;
//         }
//     }

//     return ESP_OK;
// }

volatile size_t received = 0;
volatile size_t overflowed = 0;

bool on_receive(i2s_chan_handle_t handle, i2s_event_data_t *event, void *arg)
{
    const auto buffer = static_cast<buffer_type *>(arg);
    const auto written = buffer->write(event->dma_buf, event->size);

    const auto state = taskENTER_CRITICAL_FROM_ISR();

    received += written;
    overflowed += event->size - written;

    taskEXIT_CRITICAL_FROM_ISR(state);

    return false;
}

bool on_receive_overflow(i2s_chan_handle_t handle, i2s_event_data_t *event, void *arg)
{
    const auto state = taskENTER_CRITICAL_FROM_ISR();

    overflowed += event->size;

    taskEXIT_CRITICAL_FROM_ISR(state);

    return false;
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

    auto buffer = std::make_unique<buffer_type>();

    // uac_device_config_t config = {
    //     .skip_tinyusb_init = false,
    //     .output_cb = nullptr,
    //     .input_cb = uac_device_input_cb,
    //     .set_mute_cb = nullptr,
    //     .set_volume_cb = nullptr,
    //     .cb_ctx = buffer.get(),
    // };

    // ESP_ERROR_CHECK(uac_device_init(&config));

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
        .on_recv_q_ovf = on_receive_overflow,
        .on_sent = nullptr,
        .on_send_q_ovf = nullptr,
    };

    ESP_ERROR_CHECK(i2s_channel_register_event_callback(rx_handle, &event_callbacks, buffer.get()));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    portMUX_TYPE mux;
    portMUX_INITIALIZE(&mux);

    auto now = esp_timer_get_time();
    size_t read = 0;
    float average = 44.1f;

    while (true)
    {
        if (!gpio_get_level(BTN_GPIO))
            break;

        uint8_t buf[128];
        size_t r = 0;

        do
        {
            r = buffer->read(buf, sizeof(buf));
            read += r;
        } while (r);

        if (esp_timer_get_time() > now + 1000000)
        {
            taskENTER_CRITICAL(&mux);
            size_t recv = received;
            size_t ofld = overflowed;

            received = 0;
            overflowed = 0;
            taskEXIT_CRITICAL(&mux);

            average = (0.90f * average) + (0.10f * (read / 2000));

            ESP_LOGI(TAG, "received: %zu, read: %zu, overflowed: %zu, average: %.03fks/s", recv / 2, read / 2, ofld / 2, average);

            read = 0;
            now = esp_timer_get_time();
        }
    }

    ESP_ERROR_CHECK(i2s_channel_disable(rx_handle));
    ESP_ERROR_CHECK(i2s_del_channel(rx_handle));

    esp_restart();
}
