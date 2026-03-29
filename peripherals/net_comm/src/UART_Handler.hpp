#include <cstdint>
#include <netinet/in.h>
#include <array>
#include <poll.h>
#include <chrono>
#include <functional>

#include "CircularBufferQueue.hpp"

using UART_P = struct UART_ProtocolInfo
{

    std::uint32_t baudrate{ UINT32_MAX };
    std::uint32_t tx_pin{ UINT32_MAX }; // transmit
    std::uint32_t rx_pin{ UINT32_MAX }; // receive (ignore the callback on this one)

    // summed up to create buf size
    std::uint32_t start_bits{ UINT32_MAX };
    std::uint32_t data_bits{ UINT32_MAX };
    std::uint32_t parity_bits{ UINT32_MAX };
    std::uint32_t stop_bits{ UINT32_MAX };

    // Point-to-point connection
    struct sockaddr_in other_side{ };
    int other_side_fd{ -1 };
};

struct UART_HandlerContext
{
    std::function<void(std::uint8_t, std::uint8_t)> pin_write;
    const std::atomic<std::uint64_t>* total_cycles;
};

template<std::size_t QUEUE_SIZE>
class UART_Handler final
{
public:
    using pin_write_t = std::function<void(std::uint8_t, std::uint8_t)>;

private:
    UART_P config;
    std::size_t bit_count{ 0 };

    static constexpr std::size_t MAX_BIT_COUNT = 64; // 8 bytes
    static constexpr std::uint32_t MASK_TIME = 0x7FFFFFFFU;
    static constexpr std::uint32_t MASK_BIT_VALUE = 1U << 31U;

    std::array<std::uint32_t, MAX_BIT_COUNT> buf{ 0 };

    pin_write_t pin_write;
    const std::atomic<std::uint64_t>* m_total_cycles;

    std::jthread receiver;
    std::atomic<bool> running{ false };
    std::array<int, 2> close_pipe{ -1, -1 };

public:
    explicit UART_Handler(const UART_P& config, const UART_HandlerContext& ctx)
    : config(config)
    , pin_write(std::move(ctx.pin_write))
    , m_total_cycles(ctx.total_cycles)
    {
    }

    [[nodiscard]] std::size_t bit_buffer_size() const noexcept
    {
        return config.start_bits + config.data_bits + config.parity_bits + config.stop_bits;
    }

    inline void process_bit(const std::pair<std::uint8_t, std::uint8_t>& pair, const std::uint32_t& delta)
    {
        const auto& [pin, bit] = pair;
        uint32_t packed = (delta & MASK_TIME);
        if (bit > 0)
        {
            packed |= MASK_BIT_VALUE;
        }

        buf[bit_count++] = packed;

        if (bit_count == bit_buffer_size() || bit_count >= MAX_BIT_COUNT)
        {
            send_datagram();
            bit_count = 0;
        }
    }

    void start_receiver()
    {
        running = true;
        receiver = std::jthread([this](std::stop_token stop_token) { this->receiver_thread(stop_token); });
    }

    void receiver_thread(std::stop_token stop_token)
    {
        // here it is simple, just expose the emulator writer queue
        // receive data, parse out clock for writing bits to pin
        pipe(close_pipe.data());

        std::array<struct pollfd, 2> fds{ 0 };
        fds[0] = { .fd = close_pipe[0], .events = POLLIN, .revents = 0 };
        fds[1] = { .fd = config.other_side_fd, .events = POLLIN, .revents = 0 };

        while (!stop_token.stop_requested() && running)
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
                if (!receive_datagram(fds[1].fd))
                {
                    break;
                }
            }
        }
        running = false;
    }

    [[nodiscard]] bool is_alive() const
    {
        return running.load();
    }

    void receiver_stop()
    {
        if (running.exchange(false))
        {
            receiver.request_stop();
            bool close_msg = true;
            if (close_pipe[1] != -1)
            {
                write(close_pipe[1], &close_msg, sizeof(bool));
            }
            if (receiver.joinable())
            {
                receiver.join();
            }
            _cleanup();
        }
        // simple again, force the socket to die and exit
    }

private:
    void _cleanup()
    {
        if (close_pipe[0] != -1)
        {
            close(close_pipe[0]);
        }
        if (close_pipe[1] != -1)
        {
            close(close_pipe[1]);
        }
        close_pipe = { -1, -1 };
    }

    bool receive_datagram(int fd)
    {
        std::array<std::uint32_t, MAX_BIT_COUNT> recv_buf{ 0 };
        const auto received = recv(fd, recv_buf.data(), recv_buf.size() * sizeof(uint32_t), 0);
        if (received <= 0)
        {
            return false;
        }

        const std::size_t count = received / sizeof(uint32_t);
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::uint32_t packed = recv_buf[i];
            const bool is_high = static_cast<bool>(packed & MASK_BIT_VALUE);
            const std::uint32_t delta = packed & MASK_TIME;

            const std::uint64_t start_cycles = m_total_cycles->load();
            while (running && (m_total_cycles->load() - start_cycles < delta))
            {
                cpu_relax();
            }

            if (!running)
            {
                return false;
            }

            pin_write(config.rx_pin, is_high ? 1 : 0);
        }
        return true;
    }

    void send_datagram()
    {
        send(config.other_side_fd, buf.data(), bit_count * sizeof(std::uint32_t), 0);
    }
};
