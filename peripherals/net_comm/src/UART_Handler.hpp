#include <cstdint>
#include <netinet/in.h>
#include <array>

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
    struct sockaddr_in other_side{};
};

class UART_Handler final
{
private:
    UART_P config;
    std::size_t bit_count{ 0 };

    static constexpr std::size_t BUFFER_SIZE = 128;
    static constexpr std::uint32_t MASK_TIME = 0x7FFFFFFFU;
    static constexpr std::uint32_t MASK_BIT_VALUE = 1U << 31U;
    std::array<std::uint32_t, BUFFER_SIZE> buf{ 0 };

public:
    explicit UART_Handler(const UART_P& config)
    : config(config)
    {
    }

    inline void process_bit(const std::pair<std::uint8_t, std::uint8_t>& pair, const std::uint32_t& delta)
    {
        const auto& [bit, pin] = pair;
        uint32_t packed = (delta & MASK_TIME);
        if (bit > 0)
        {
            packed |= MASK_BIT_VALUE;
        }

        buf[bit_count++] = packed;

        if (bit_count == BUFFER_SIZE)
        {
            send_datagram();
            bit_count = 0;
        }
    }

    void receiver_thread()
    {
        // here it is simple, just expose the emulator writer queue
        // receive data, parse out clock for writing bits to pin
    }

    void receiver_stop()
    {
        // simple again, force the socket to die and exit
    }

private:
    void send_datagram()
    {
    }
};
