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

#include "CircularBufferQueue.hpp"
#include "zero_mate/external_peripheral.hpp"
#include "Util.hpp"

using I2C_P = struct I2C_ProtocolInfo
{
    bool master_mode;
    std::uint8_t address{ 0 };

    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
    // Broadcast address and intent everywhere
    // START signal | 7 bit address | 1 bit intent (0 - Write | 1 - Read)
    std::vector<struct sockaddr_in> slaves;
    std::vector<int> slave_fds;

    struct sockaddr_in master;
    int master_fd;
};

class I2C_Handler
{
    static constexpr std::size_t BUFFER_SIZE = 128;

    // Bit 31: Pin (0 = SDA, 1 = SCL)
    // Bit 30: Value (0 = Low, 1 = High)
    // Bits 29-0: Delta (Nanoseconds since last transition)
    static constexpr uint32_t MASK_PIN = 1U << 31U;
    static constexpr uint32_t MASK_VALUE = 1U << 30U;
    static constexpr uint32_t MASK_DELTA = 0x3FFFFFFF;
    static constexpr int BYTE_BIT_COUNT = 8;

    static constexpr std::size_t RECV_BUF_SIZE = BUFFER_SIZE;
    static constexpr std::size_t BACKOFF_CYCLE_COUNT = 1024;

    I2C_P config;
    TSP::Queue::Buffer<std::uint8_t, RECV_BUF_SIZE> queue_buf;
    TSP::Queue::Writer<std::uint8_t, RECV_BUF_SIZE> queue_writer;
    TSP::Queue::Reader<std::uint8_t, RECV_BUF_SIZE> queue_reader;
    TSP::BF::Backoff backoff_fast{ BACKOFF_CYCLE_COUNT };

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin_func;
    zero_mate::IExternal_Peripheral::Halt_t halt_func;
    zero_mate::IExternal_Peripheral::Start_t start_func;

    // Physical bus state tracking
    bool scl_lvl = true;
    bool sda_lvl = true;

    // Logical state tracking for synchronization
    uint8_t bit_count = 0;
    bool in_transaction = false;
    bool is_read = false;
    uint8_t shift_reg = 0;
    bool matched_address = false;

    enum class Phase
    {
        ADDRESS,
        DATA
    } phase = Phase::ADDRESS;

    std::array<uint32_t, BUFFER_SIZE> buf{ 0 };
    size_t buf_idx = 0;

    std::atomic<bool> running{ false };
    std::thread receiver;
    // 0 -> Read end
    // 1 -> Write end
    std::array<int, 2> close_pipefd{ 0 };

public:
    explicit I2C_Handler(I2C_P config,
                         zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                         zero_mate::IExternal_Peripheral::Halt_t halt_func,
                         zero_mate::IExternal_Peripheral::Start_t start_func)
    : config(std::move(config))
    , set_pin_func(set_pin)
    , queue_writer(&queue_buf)
    , queue_reader(&queue_buf)
    , halt_func(halt_func)
    , start_func(start_func)
    {
    }
    ~I2C_Handler() = default;

    I2C_Handler(const I2C_Handler&) = delete;
    I2C_Handler& operator=(const I2C_Handler&) = delete;

    I2C_Handler(I2C_Handler&&) noexcept = delete;
    I2C_Handler& operator=(I2C_Handler&&) noexcept = delete;

    inline void process_bit(const std::pair<std::uint8_t, std::uint8_t>& pair, uint32_t delta)
    {
        if (config.master_mode)
        {
            process_bit_master(pair, delta);
        }
        else
        {
            process_bit_slave(pair, delta);
        }
    }

    inline void process_bit_slave(const std::pair<std::uint8_t, std::uint8_t>& pair, uint32_t delta)
    {
        const auto& [bit, pin] = pair;
        bool is_high = (bit != 0);
        bool is_scl = (pin == config.scl_pin);

        if (is_scl)
        {
            if (!scl_lvl && is_high && in_transaction) // Rising edge
            {
                bit_count++;
                if (phase == Phase::ADDRESS && bit_count <= 8)
                {
                    shift_reg = (shift_reg << 1) | (sda_lvl ? 1 : 0);
                }
            }
            else if (scl_lvl && !is_high && in_transaction) // Falling edge
            {
                if (bit_count == 8 && phase == Phase::ADDRESS)
                {
                    is_read = (shift_reg & 0x01);
                    if ((shift_reg >> 1) == config.address)
                    {
                        matched_address = true;
                    }
                }

                if (bit_count == 9)
                {
                    bit_count = 0;
                    phase = Phase::DATA;
                }
            }
            scl_lvl = is_high;
        }
        else
        {
            // Detect START/STOP
            if (scl_lvl)
            {
                if (sda_lvl && !is_high) // START
                {
                    in_transaction = true;
                    bit_count = 0;
                    phase = Phase::ADDRESS;
                    shift_reg = 0;
                    matched_address = false;
                }
                else if (!sda_lvl && is_high) // STOP
                {
                    in_transaction = false;
                }
            }
            sda_lvl = is_high;
        }

        // Send bits back to master if we are the addressed slave and we own the bus
        if (in_transaction && matched_address && !is_scl)
        {
            bool slave_drives = false;
            if (phase == Phase::ADDRESS && bit_count == 8)
            {
                slave_drives = true; // ACK
            }
            else if (phase == Phase::DATA)
            {
                if (is_read && bit_count < 8)
                {
                    slave_drives = true; // Data
                }
                else if (!is_read && bit_count == 8)
                {
                    slave_drives = true; // ACK
                }
            }

            if (slave_drives)
            {
                uint8_t raw_bit = is_high ? 1 : 0;
                send(config.master_fd, &raw_bit, sizeof(uint8_t), 0);
            }
        }
    }

    inline void process_bit_master(const std::pair<std::uint8_t, std::uint8_t>& pair, uint32_t delta)
    {
        const auto& [bit, pin] = pair;
        bool is_high = (bit != 0);
        bool is_scl = (pin == config.scl_pin);

        // 1. Determine current bus ownership
        bool slave_drives = false;
        if (in_transaction)
        {
            if (phase == Phase::ADDRESS && bit_count == 8)
            {
                slave_drives = true;
            }
            else if (phase == Phase::DATA)
            {
                if ((is_read && bit_count < 8) || (!is_read && bit_count == 8))
                {
                    slave_drives = true;
                }
            }
        }

        // 2. Only pack transitions originating from the Master
        if (is_scl || !slave_drives)
        {
            uint32_t packed = (delta & MASK_DELTA);
            if (is_scl)
            {
                packed |= MASK_PIN;
            }
            if (is_high)
            {
                packed |= MASK_VALUE;
            }
            buf[buf_idx++] = packed;
        }

        // 3. Update state machine and handle synchronization
        bool sync_point = false;
        if (is_scl)
        {
            if (!scl_lvl && is_high && in_transaction) // Rising edge
            {
                bit_count++;
                if (bit_count == BYTE_BIT_COUNT && phase == Phase::ADDRESS)
                {
                    is_read = sda_lvl;
                }
            }
            else if (scl_lvl && !is_high && in_transaction) // Falling edge
            {
                if (bit_count == BYTE_BIT_COUNT)
                {
                    sync_point = true;
                }
                else if (bit_count == BYTE_BIT_COUNT + 1)
                {
                    bit_count = 0;
                    phase = Phase::DATA;
                }

                // Determine if slave drives the NEXT bit
                bool slave_drives_next = false;
                if (phase == Phase::ADDRESS && bit_count == BYTE_BIT_COUNT)
                {
                    slave_drives_next = true;
                }
                else if (phase == Phase::DATA)
                {
                    if ((is_read && bit_count < BYTE_BIT_COUNT) || (!is_read && bit_count == BYTE_BIT_COUNT))
                    {
                        slave_drives_next = true;
                    }
                }

                if (slave_drives_next && queue_reader.try_advance())
                {
                    set_pin_func(config.sda_pin, static_cast<bool>(queue_reader.peek()));
                    queue_reader.advance();
                }
            }
            scl_lvl = is_high;
        }
        else
        {
            // Detect START/STOP on SDA transitions while SCL is high
            if (scl_lvl)
            {
                if (sda_lvl && !is_high)
                {
                    in_transaction = true;
                    bit_count = 0;
                    phase = Phase::ADDRESS;
                }
                else if (!sda_lvl && is_high)
                {
                    in_transaction = false;
                }
            }
            sda_lvl = is_high;
        }

        if (sync_point || buf_idx == BUFFER_SIZE)
        {
            send_datagram();
            if (sync_point)
            {
                halt_func();
            }
        }
    }

    void _cleanup()
    {
        close(close_pipefd[0]);
        close(close_pipefd[1]);
    }

    void receiver_thread()
    {
        running = true;
        pipe(close_pipefd.data());

        std::vector<struct pollfd> fds;
        fds.emplace_back(close_pipefd[0], POLLIN, 0);

        if (config.master_mode)
        {
            for (const auto& slave : config.slave_fds)
            {
                fds.emplace_back(slave, POLLIN);
            }
        }
        else
        {
            fds.emplace_back(config.master_fd, POLLIN);
        }

        while (running)
        {
            const int ret_val = poll(fds.data(), fds.size(), -1);
            if (ret_val < 1)
            {
                running = false;
                _cleanup();
                break;
            }

            if (POLLIN & fds[0].revents)
            {
                running = false;
            }

            if (!running)
            {
                _cleanup();
                break;
            }

            if (config.master_mode)
            {
                int fd_to_read_from = 0;
                for (std::size_t i = 1; i < fds.size(); i++)
                {
                    if (POLLIN & fds[i].revents)
                    {
                        fd_to_read_from = fds[i].fd;
                        break;
                    }
                }
                if (fd_to_read_from != 0)
                {
                    receive_from_slave(fd_to_read_from);
                }
            }
            else
            {
                receive_from_master(fds[1].fd);
            }
        }
    }

    void receive_from_master(int socket_fd)
    {
        std::array<std::uint32_t, BUFFER_SIZE> recv_buf{ 0 };
        const auto bytes_received = recv(socket_fd, recv_buf.data(), recv_buf.size() * sizeof(uint32_t), 0);

        if (bytes_received <= 0)
            return;
        const auto count = static_cast<std::size_t>(bytes_received) / sizeof(std::uint32_t);

        for (std::size_t i = 0; i < count; i++)
        {
            const std::uint32_t pin_info = recv_buf[i];
            const bool is_scl = static_cast<bool>(pin_info & MASK_PIN);
            const bool value = static_cast<bool>(pin_info & MASK_VALUE);
            const std::uint32_t delta = pin_info & MASK_DELTA;

            const auto wait_start = std::chrono::high_resolution_clock::now();
            while (
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - wait_start)
            .count() < delta)
            {
                cpu_relax();
            }

            set_pin_func((is_scl ? config.scl_pin : config.sda_pin), value);
        }
    }

    void receive_from_slave(int socket_fd)
    {
        std::array<std::uint8_t, BUFFER_SIZE> recv_buf{ 0 };
        const auto bytes_received = recv(socket_fd, recv_buf.data(), recv_buf.size(), 0);

        if (bytes_received <= 0)
            return;

        queue_writer.insert_with_backoff(recv_buf[0], backoff_fast);

        start_func(); // gui should catch this request and startup execution
        for (int i = 1; i < bytes_received; i++)
        {
            queue_writer.insert_with_backoff(recv_buf[i], backoff_fast);
        }
    }

    void receiver_stop()
    {
        running = false;
        bool close_msg = true;
        write(close_pipefd[1], &close_msg, sizeof(bool));

        if (receiver.joinable())
        {
            receiver.join();
        }
    }

private:
    void send_datagram()
    {
        if (buf_idx == 0)
            return;

        for (int fd : config.slave_fds)
        {
            send(fd, buf.data(), buf_idx * sizeof(uint32_t), 0);
        }
        buf_idx = 0;
    }
};
