#include <array>
#include <cstdint>
#include <netinet/in.h>
#include <vector>

#include "CircularBufferQueue.hpp"
#include "zero_mate/external_peripheral.hpp"

using I2C_P = struct I2C_ProtocolInfo
{
    bool master_mode;

    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
    // Broadcast address and intent everywhere
    // START signal | 7 bit address | 1 bit intent (0 - Write | 1 - Read)
    std::vector<struct sockaddr_in> slaves;
    struct sockaddr_in master;
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

    static constexpr std::size_t RECV_BUF_SIZE = BUFFER_SIZE;

    I2C_P config;
    TSP::Queue::Reader<std::uint8_t, RECV_BUF_SIZE> rx_queue;

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin_func;
    zero_mate::IExternal_Peripheral::Halt_t halt_func;

    // Physical bus state tracking
    bool scl_lvl = true;
    bool sda_lvl = true;

    // Logical state tracking for synchronization
    uint8_t bit_count = 0;
    bool in_transaction = false;
    bool is_read = false;
    enum class Phase
    {
        ADDRESS,
        DATA
    } phase = Phase::ADDRESS;

    std::array<uint32_t, BUFFER_SIZE> buf{ 0 };
    size_t buf_idx = 0;

public:
    explicit I2C_Handler(I2C_P config,
                         zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                         TSP::Queue::Buffer<std::uint8_t, RECV_BUF_SIZE>* rx_buf,
                         zero_mate::IExternal_Peripheral::Halt_t halt_func)
    : config(std::move(config))
    , set_pin_func(set_pin)
    , rx_queue(rx_buf)
    , halt_func(halt_func)
    {
    }
    ~I2C_Handler() = default;

    I2C_Handler(const I2C_Handler&) = delete;
    I2C_Handler& operator=(const I2C_Handler&) = delete;

    I2C_Handler(I2C_Handler&&) noexcept = default;
    I2C_Handler& operator=(I2C_Handler&&) noexcept = default;

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
                if (bit_count == 8 && phase == Phase::ADDRESS)
                {
                    is_read = sda_lvl;
                }
            }
            else if (scl_lvl && !is_high && in_transaction) // Falling edge
            {
                if (bit_count == 8)
                {
                    sync_point = true;
                }
                else if (bit_count == 9)
                {
                    bit_count = 0;
                    phase = Phase::DATA;
                }

                // Determine if slave drives the NEXT bit
                bool slave_drives_next = false;
                if (phase == Phase::ADDRESS && bit_count == 8)
                {
                    slave_drives_next = true;
                }
                else if (phase == Phase::DATA)
                {
                    if ((is_read && bit_count < 8) || (!is_read && bit_count == 8))
                    {
                        slave_drives_next = true;
                    }
                }

                if (slave_drives_next && rx_queue.try_advance())
                {
                    set_pin_func(config.sda_pin, rx_queue.peek());
                    rx_queue.advance();
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

    void receiver_thread()
    {
    }

    void receiver_stop()
    {
    }

private:
    void send_datagram()
    {
        for (const auto& other_side : config.slaves)
        {
        }
        buf_idx = 0;
    }
};
