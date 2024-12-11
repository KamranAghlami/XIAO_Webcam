#pragma once

#include <array>
#include <condition_variable>
#include <mutex>

template <typename S, std::size_t N>
class triple_buffer
{
public:
    using sample_type = S;
    using buffer_type = std::array<sample_type, N>;

    triple_buffer()
    {
    }

    ~triple_buffer()
    {
        const std::lock_guard lock_guard(m_mutex);

        m_quit = true;

        m_condition_variable.notify_all();
    }

    buffer_type *get_back_buffer()
    {
        return m_buffer_back;
    }

    void swap_buffers()
    {
        const std::lock_guard lock_guard(m_mutex);

        std::swap(m_buffer_back, m_buffer_orphan);

        m_ready = true;

        m_condition_variable.notify_one();
    }

    buffer_type *get_front_buffer()
    {
        std::unique_lock unique_lock(m_mutex);

        if (!m_ready)
        {
            auto should_wake = [this]()
            {
                return m_ready || m_quit;
            };

            m_condition_variable.wait(unique_lock, should_wake);
        }

        if (!m_quit)
        {
            std::swap(m_buffer_front, m_buffer_orphan);

            m_ready = false;
        }

        return m_buffer_front;
    }

private:
    buffer_type m_buffer_1 = {};
    buffer_type m_buffer_2 = {};
    buffer_type m_buffer_3 = {};

    buffer_type *m_buffer_back = &m_buffer_1;
    buffer_type *m_buffer_orphan = &m_buffer_2;
    buffer_type *m_buffer_front = &m_buffer_3;

    bool m_quit = false;
    bool m_ready = false;
    std::mutex m_mutex;
    std::condition_variable m_condition_variable;
};