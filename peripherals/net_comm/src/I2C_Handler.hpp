#include <array>
#include <cstdint>
#include <netinet/in.h>
#include <vector>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>

#include "CircularBufferQueue.hpp"
#include "zero_mate/external_peripheral.hpp"
#include "Util.hpp"

// Forward declarations
template<std::size_t Size>
class I2C_Master;

template<std::size_t Size>
class I2C_Slave;

enum class I2C_State : uint8_t
{
    IDLE,
    ADDRESS,
    READ_BYTE,
    WRITE_BYTE,
    RESPONSE
};

struct I2C_Master_P
{
    std::uint32_t id{ 0 };
    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
    std::vector<int> slave_fds;
};

struct I2C_Slave_P
{
    std::uint32_t id{ 0 };
    std::uint8_t address{ 0 };
    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
    int master_fd{ -1 };
};

class I2C_Base
{
public:
    using pin_write_t = std::function<void(std::uint8_t, std::uint8_t)>;

protected:
    static constexpr std::size_t BUFFER_SIZE = 128;
    static constexpr uint32_t MASK_PIN = 1U << 31U;
    static constexpr uint32_t MASK_VALUE = 1U << 30U;
    static constexpr uint32_t MASK_DELTA = 0x3FFFFFFF;
    static constexpr std::size_t BACKOFF_CYCLES = 1024;

    pin_write_t pin_write;
    bool scl_lvl = true;
    bool sda_lvl = true;
    I2C_State state = I2C_State::IDLE;
    uint8_t bit_count = 0;
    bool is_read = false;
    bool ack_from_slave = false;

    std::array<uint8_t, 16> slave_send_buf{};
    size_t slave_send_idx = 0;

    std::atomic<bool> running{ false };
    std::thread receiver;
    std::array<int, 2> close_pipefd{ -1, -1 };

    zero_mate::IExternal_Peripheral::Halt_t halt_func;
    zero_mate::IExternal_Peripheral::Start_t start_func;

    I2C_Base(zero_mate::IExternal_Peripheral::Halt_t halt_func,
             zero_mate::IExternal_Peripheral::Start_t start_func,
             pin_write_t pin_write)
    : pin_write(std::move(pin_write))
    , halt_func(halt_func)
    , start_func(start_func)
    {
    }

    void _cleanup()
    {
        if (close_pipefd[0] != -1)
        {
            close(close_pipefd[0]);
        }
        if (close_pipefd[1] != -1)
        {
            close(close_pipefd[1]);
        }
        close_pipefd = { -1, -1 };
    }

public:
    I2C_Base(const I2C_Base&) = delete;
    I2C_Base(I2C_Base&&) = delete;
    I2C_Base& operator=(const I2C_Base&) = delete;
    I2C_Base& operator=(I2C_Base&&) = delete;

    virtual ~I2C_Base()
    {
        receiver_stop();
    }

    void receiver_stop()
    {
        if (running.exchange(false))
        {
            bool close_msg = true;
            if (close_pipefd[1] != -1)
            {
                write(close_pipefd[1], &close_msg, sizeof(bool));
            }
            if (receiver.joinable())
            {
                receiver.join();
            }
            _cleanup();
        }
    }
};

template<std::size_t Size>
class I2C_Master final : public I2C_Base
{
    I2C_Master_P config;
    std::array<uint32_t, BUFFER_SIZE> buf{ 0 };
    size_t buf_idx = 0;

    TSP::Queue::Buffer<std::uint8_t, BUFFER_SIZE> queue_buf;
    TSP::Queue::Writer<std::uint8_t, BUFFER_SIZE> queue_writer;
    TSP::Queue::Reader<std::uint8_t, BUFFER_SIZE> queue_reader;
    TSP::BF::Backoff backoff_fast{ BACKOFF_CYCLES };

    uint8_t shift_reg = 0;

public:
    I2C_Master(I2C_Master_P config,
               zero_mate::IExternal_Peripheral::Halt_t halt_func,
               zero_mate::IExternal_Peripheral::Start_t start_func,
               pin_write_t pin_write)
    : I2C_Base(halt_func, start_func, std::move(pin_write))
    , config(std::move(config))
    , queue_writer(&queue_buf)
    , queue_reader(&queue_buf)
    {
    }

    void process_bit(const std::pair<std::uint8_t, std::uint8_t>& pair, const uint32_t delta)
    {
        const auto& [bit, pin] = pair;
        const bool is_high = (bit != 0);
        const bool is_scl = (pin == config.scl_pin);

        if (is_scl)
        {
            handle_scl_master(is_high, delta);
        }
        else
        {
            handle_sda_master(is_high, delta);
        }
    }

    void start_receiver()
    {
        receiver = std::thread(&std::remove_reference_t<decltype(*this)>::receiver_thread, this);
    }

    void receiver_thread()
    {
        running = true;
        pipe(close_pipefd.data());

        std::vector<struct pollfd> fds;
        fds.emplace_back(close_pipefd[0], POLLIN, 0);
        for (int fd : config.slave_fds)
        {
            fds.emplace_back(fd, POLLIN, 0);
        }

        while (running)
        {
            if (poll(fds.data(), fds.size(), -1) <= 0)
            {
                break;
            }
            if (fds[0].revents & POLLIN)
            {
                break;
            }

            for (size_t i = 1; i < fds.size(); ++i)
            {
                if (fds[i].revents & POLLIN)
                {
                    receive_from_slave(fds[i].fd);
                }
            }
        }
    }

private:
    void handle_scl_master(const bool is_high, const uint32_t delta)
    {
        bool sync_point = false;

        if (!scl_lvl && is_high) // Rising edge
        {
            if (state != I2C_State::IDLE)
            {
                bit_count++;
            }

            if (state == I2C_State::ADDRESS && bit_count == 8)
            {
                is_read = sda_lvl;
            }
            
            // Pop bit Slave sent after previous falling edge
            const bool slave_drives_now =
            (state == I2C_State::READ_BYTE && bit_count <= 8) || (state == I2C_State::RESPONSE && ack_from_slave);
            
            if (slave_drives_now)
            {
                int retries = 100000;
                while (!queue_reader.try_advance() && retries-- > 0)
                {
                    cpu_relax();
                }

                if (queue_reader.try_advance())
                {
                    uint8_t val = queue_reader.peek();
                    pin_write(config.sda_pin, val);
                    queue_reader.advance();
                }
                else
                {
                    pin_write(config.sda_pin, 1);
                }
            }
        }
        else if (scl_lvl && !is_high) // Falling edge
        {
            if (state == I2C_State::ADDRESS && bit_count == 8)
            {
                state = I2C_State::RESPONSE;
                bit_count = 0;
                ack_from_slave = true;
                sync_point = true;
            }
            else if (state == I2C_State::WRITE_BYTE && bit_count == 8)
            {
                state = I2C_State::RESPONSE;
                bit_count = 0;
                ack_from_slave = true;
                sync_point = true;
            }
            else if (state == I2C_State::READ_BYTE && bit_count == 8)
            {
                state = I2C_State::RESPONSE;
                bit_count = 0;
                ack_from_slave = false;
                sync_point = false;
            }
            else if (state == I2C_State::RESPONSE)
            {
                state = is_read ? I2C_State::READ_BYTE : I2C_State::WRITE_BYTE;
                bit_count = 0;
                if (is_read && !sda_lvl)
                {
                    sync_point = true;
                }
            }
            else if (state == I2C_State::READ_BYTE && bit_count < 8)
            {
                sync_point = true;
            }
        }

        scl_lvl = is_high;
        pack_and_send(is_high, true, delta, sync_point);
    }

    void handle_sda_master(bool is_high, uint32_t delta)
    {
        if (scl_lvl)
        {
            if (sda_lvl && !is_high) // START
            {
                while (queue_reader.try_advance())
                {
                    queue_reader.advance();
                }
                state = I2C_State::ADDRESS;
                bit_count = 0;
            }
            else if (!sda_lvl && is_high) // STOP
            {
                state = I2C_State::IDLE;
            }
        }
        sda_lvl = is_high;
        pack_and_send(is_high, false, delta, false);
    }

    void pack_and_send(const bool value, const bool is_scl, const uint32_t delta, const bool sync_point)
    {
        const uint32_t packed = (delta & MASK_DELTA) | (value ? MASK_VALUE : 0) | (is_scl ? MASK_PIN : 0);
        buf[buf_idx++] = packed;

        bool should_flush = sync_point || buf_idx == BUFFER_SIZE;
        if (is_scl && (state == I2C_State::READ_BYTE || (state == I2C_State::RESPONSE && ack_from_slave)))
        {
            should_flush = true;
        }

        if (should_flush)
        {
            for (int fd : config.slave_fds)
            {
                send(fd, buf.data(), buf_idx * sizeof(uint32_t), 0);
            }
            buf_idx = 0;
            if (sync_point && !queue_reader.try_advance())
            {
                halt_func();
            }
        }
    }

    void receive_from_slave(const int fd)
    {
        std::array<uint8_t, BUFFER_SIZE> rx_buf{ 0 };
        ssize_t n = recv(fd, rx_buf.data(), rx_buf.size(), 0);
        if (n > 0)
        {
            for (ssize_t i = 0; i < n; ++i)
            {
                queue_writer.insert_with_backoff(rx_buf[i], backoff_fast);
            }
            start_func(); // Resume Master to drain the newly arrived bits
        }
    }
};

template<std::size_t Size>
class I2C_Slave final : public I2C_Base
{
    I2C_Slave_P config;
    bool matched_address = false;
    uint8_t shift_reg = 0;

    TSP::BF::Backoff backoff_fast{ BACKOFF_CYCLES };

public:
    I2C_Slave(I2C_Slave_P config,
              zero_mate::IExternal_Peripheral::Halt_t halt_func,
              zero_mate::IExternal_Peripheral::Start_t start_func,
              pin_write_t pin_write)
    : I2C_Base(halt_func, start_func, std::move(pin_write))
    , config(std::move(config))
    {
    }

    void process_bit(const std::pair<std::uint8_t, std::uint8_t>& pair, const uint32_t delta)
    {
        const auto& [bit, pin] = pair;
        if (pin == config.sda_pin && matched_address)
        {
            const bool slave_owns_bus =
            (state == I2C_State::READ_BYTE) || (state == I2C_State::RESPONSE && ack_from_slave);

            if (slave_owns_bus)
            {
                slave_send_buf[slave_send_idx++] = (bit != 0 ? 1 : 0);

                bool should_flush = false;
                if (state == I2C_State::RESPONSE && ack_from_slave)
                {
                    should_flush = true;
                }
                else if (state == I2C_State::READ_BYTE)
                {
                    should_flush = true;
                }

                if (should_flush)
                {
                    send(config.master_fd, slave_send_buf.data(), slave_send_idx, 0);
                    slave_send_idx = 0;
                }
            }
        }
    }

    void start_receiver()
    {
        receiver = std::thread(&std::remove_reference_t<decltype(*this)>::receiver_thread, this);
    }

    void receiver_thread()
    {
        running = true;
        pipe(close_pipefd.data());

        std::array<struct pollfd, 2> fds{ 0 };
        fds[0] = { .fd = close_pipefd[0], .events = POLLIN, .revents = 0 };
        fds[1] = { .fd = config.master_fd, .events = POLLIN, .revents = 0 };

        while (running)
        {
            if (poll(fds.data(), fds.size(), -1) <= 0)
            {
                break;
            }
            if (fds[0].revents & POLLIN)
            {
                break;
            }
            if (fds[1].revents & POLLIN)
            {
                receive_from_master(config.master_fd);
            }
        }
    }

private:
    void receive_from_master(int fd)
    {
        std::array<std::uint32_t, BUFFER_SIZE> recv_buf{ 0 };
        const auto received = recv(fd, recv_buf.data(), recv_buf.size() * sizeof(uint32_t), 0);
        if (received <= 0)
        {
            return;
        }

        const std::size_t count = received / sizeof(uint32_t);
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::uint32_t packed = recv_buf[i];
            const bool is_scl = static_cast<bool>(packed & MASK_PIN);
            const bool value = static_cast<bool>(packed & MASK_VALUE);
            const std::uint32_t delta = (packed & MASK_DELTA);

            auto start_wait = std::chrono::high_resolution_clock::now();
            while (
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start_wait)
            .count() < delta)
            {
                cpu_relax();
            }

            if (is_scl)
            {
                if (!scl_lvl && value)
                {
                    handle_scl_slave_rising(value);
                }
            }
            else
            {
                handle_sda_slave(value);
            }

            if (is_scl)
            {
                if (scl_lvl && !value)
                {
                    handle_scl_slave_falling(value);
                }
            }

            if (matched_address || state == I2C_State::ADDRESS)
            {
                pin_write(is_scl ? config.scl_pin : config.sda_pin, value);
            }

            if (is_scl)
            {
                scl_lvl = value;
            }
        }
    }

    void handle_scl_slave_rising([[maybe_unused]] bool is_high)
    {
        if (state != I2C_State::IDLE)
        {
            bit_count++;
            if (state == I2C_State::ADDRESS && bit_count <= 8)
            {
                shift_reg = (shift_reg << 1U) | (sda_lvl ? 1 : 0);
            }
            else if (state == I2C_State::WRITE_BYTE && bit_count <= 8)
            {
                shift_reg = (shift_reg << 1U) | (sda_lvl ? 1 : 0);
            }
        }
    }

    void handle_scl_slave_falling([[maybe_unused]] bool is_high)
    {
        if (state == I2C_State::ADDRESS && bit_count == 8)
        {
            is_read = static_cast<bool>(shift_reg & 0x01U);
            matched_address = ((shift_reg >> 1U) == config.address);
            state = I2C_State::RESPONSE;
            bit_count = 0;
            ack_from_slave = matched_address;
        }
        else if (state == I2C_State::WRITE_BYTE && bit_count == 8)
        {
            state = I2C_State::RESPONSE;
            bit_count = 0;
            ack_from_slave = matched_address;
        }
        else if (state == I2C_State::READ_BYTE && bit_count == 8)
        {
            state = I2C_State::RESPONSE;
            bit_count = 0;
            ack_from_slave = false;
        }
        else if (state == I2C_State::RESPONSE)
        {
            state = is_read ? I2C_State::READ_BYTE : I2C_State::WRITE_BYTE;
            bit_count = 0;
            shift_reg = 0;
        }
    }

    void handle_sda_slave(bool is_high)
    {
        if (scl_lvl)
        {
            if (sda_lvl && !is_high) // START
            {
                state = I2C_State::ADDRESS;
                bit_count = 0;
                matched_address = false;
                shift_reg = 0;
            }
            else if (!sda_lvl && is_high) // STOP
            {
                state = I2C_State::IDLE;
            }
        }
        sda_lvl = is_high;
    }
};
