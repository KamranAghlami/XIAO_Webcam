#pragma once

#include <array>

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/task.h>

template <std::size_t N>
class ring_buffer
{
public:
    using buffer_type = std::array<uint8_t, N>;

    ring_buffer()
        : m_buffer(),
          m_size(0),
          m_read_index(0),
          m_write_index(0)
    {
        portMUX_INITIALIZE(&m_mux);
    }

    ~ring_buffer()
    {
    }

    size_t read(void *data, const size_t size)
    {
        auto dest = static_cast<uint8_t *>(data);
        size_t bytes_read = 0;

        taskENTER_CRITICAL(&m_mux);

        while (bytes_read < size && m_size > 0)
        {
            *dest++ = m_buffer[m_read_index];
            m_read_index = (m_read_index + 1) % N;
            m_size = m_size - 1;
            bytes_read++;
        }

        taskEXIT_CRITICAL(&m_mux);

        return bytes_read;
    }

    size_t write(const void *data, const size_t size)
    {
        auto src = static_cast<const uint8_t *>(data);
        size_t bytes_written = 0;

        const auto state = taskENTER_CRITICAL_FROM_ISR();

        while (bytes_written < size && m_size < N)
        {
            m_buffer[m_write_index] = *src++;
            m_write_index = (m_write_index + 1) % N;
            m_size = m_size + 1;
            bytes_written++;
        }

        taskEXIT_CRITICAL_FROM_ISR(state);

        return bytes_written;
    }

private:
    buffer_type m_buffer;
    portMUX_TYPE m_mux;
    volatile std::size_t m_size;
    volatile std::size_t m_read_index;
    volatile std::size_t m_write_index;
};