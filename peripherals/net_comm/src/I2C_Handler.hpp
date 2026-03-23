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

enum class I2C_Packet_Type : uint8_t
{
    I2C_START,
    I2C_STOP,
    I2C_ADDRESS,
    I2C_WRITE_BYTE,
    I2C_READ_BYTE,
    I2C_ACK,
    I2C_DATA
};

struct I2C_Packet
{
    I2C_Packet_Type type;
    uint8_t value;
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
    using pin_read_t = std::function<std::uint8_t(std::uint8_t)>;

protected:
    static constexpr std::size_t BUFFER_SIZE = 128;
    static constexpr uint32_t MASK_PIN = 1U << 31U;
    static constexpr uint32_t MASK_VALUE = 1U << 30U;
    static constexpr uint32_t MASK_DELTA = 0x3FFFFFFF;
    static constexpr std::size_t BACKOFF_CYCLES = 1024;

    pin_write_t pin_write;
    pin_read_t pin_read;
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
             pin_write_t pin_write,
             pin_read_t pin_read)
    : pin_write(std::move(pin_write))
    , pin_read(std::move(pin_read))
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
               pin_write_t pin_write,
               pin_read_t pin_read)
    : I2C_Base(halt_func, start_func, std::move(pin_write), std::move(pin_read))
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
    void handle_scl_master(const bool is_high, [[maybe_unused]] const uint32_t delta)
    {
        bool wait_for_response = false;

        if (!scl_lvl && is_high) // Rising edge
        {
            if (state != I2C_State::IDLE)
            {
                bit_count++;

                if (state == I2C_State::ADDRESS || state == I2C_State::WRITE_BYTE)
                {
                    shift_reg = static_cast<uint8_t>((shift_reg << 1U) | (sda_lvl ? 1U : 0U));
                }
            }

            if (state == I2C_State::ADDRESS && bit_count == 8)
            {
                is_read = static_cast<bool>(shift_reg & 0x01U);
            }
        }
        else if (scl_lvl && !is_high) // Falling edge
        {
            const bool slave_drives_next =
            (state == I2C_State::READ_BYTE && bit_count < 8) || (state == I2C_State::RESPONSE && ack_from_slave && bit_count == 0);

            if (slave_drives_next)
            {
                if (queue_reader.try_advance())
                {
                    uint8_t val = queue_reader.peek();
                    pin_write(config.sda_pin, val);
                    sda_lvl = (val != 0);
                    queue_reader.advance();
                }
                else
                {
                    pin_write(config.sda_pin, 1);
                    sda_lvl = true;
                }
            }
            else
            {
                pin_write(config.sda_pin, 1); // Release SDA for Master to drive
                sda_lvl = true;
            }

            if (state == I2C_State::ADDRESS && bit_count == 8)
            {
                send_packet(I2C_Packet_Type::I2C_ADDRESS, shift_reg);
                state = I2C_State::RESPONSE;
                bit_count = 0;
                wait_for_response = true;
            }
            else if (state == I2C_State::WRITE_BYTE && bit_count == 8)
            {
                send_packet(I2C_Packet_Type::I2C_WRITE_BYTE, shift_reg);
                state = I2C_State::RESPONSE;
                bit_count = 0;
                wait_for_response = true;
            }
            else if (state == I2C_State::READ_BYTE && bit_count == 8)
            {
                state = I2C_State::RESPONSE;
                bit_count = 0;
                ack_from_slave = false;
            }
            else if (state == I2C_State::RESPONSE)
            {
                if (is_read && bit_count == 0)
                {
                    // Master just sent ACK/NACK to the slave (after reading a byte)
                    send_packet(I2C_Packet_Type::I2C_ACK, sda_lvl ? 0 : 1);
                }

                state = is_read ? I2C_State::READ_BYTE : I2C_State::WRITE_BYTE;
                bit_count = 0;
                shift_reg = 0;

                if (is_read)
                {
                    send_packet(I2C_Packet_Type::I2C_READ_BYTE, 0);
                    wait_for_response = true;
                }
            }
        }

        scl_lvl = is_high;
        if (wait_for_response)
        {
            halt_func();
        }
    }

    void handle_sda_master(bool is_high, [[maybe_unused]] uint32_t delta)
    {
        if (scl_lvl)
        {
            if (sda_lvl && !is_high) // START
            {
                while (queue_reader.try_advance())
                {
                    queue_reader.advance();
                }
                send_packet(I2C_Packet_Type::I2C_START, 0);
                state = I2C_State::ADDRESS;
                bit_count = 0;
                shift_reg = 0;
            }
            else if (!sda_lvl && is_high) // STOP
            {
                send_packet(I2C_Packet_Type::I2C_STOP, 0);
                state = I2C_State::IDLE;
            }
        }
        sda_lvl = is_high;
    }

    void send_packet(I2C_Packet_Type type, uint8_t value)
    {
        I2C_Packet packet{ type, value };
        for (int fd : config.slave_fds)
        {
            send(fd, &packet, sizeof(packet), 0);
        }
    }

    void receive_from_slave(const int fd)
    {
        I2C_Packet packet{};
        ssize_t n = recv(fd, &packet, sizeof(packet), 0);
        if (n == sizeof(packet))
        {
            if (packet.type == I2C_Packet_Type::I2C_ACK)
            {
                ack_from_slave = (packet.value != 0);
                // For RESPONSE state, the master expects 1 bit (ACK)
                queue_writer.insert_with_backoff(ack_from_slave ? 0 : 1, backoff_fast);

                // If we are in the RESPONSE state and waiting for this bit
                if (state == I2C_State::RESPONSE && bit_count == 0)
                {
                    if (queue_reader.try_advance())
                    {
                        uint8_t val = queue_reader.peek();
                        pin_write(config.sda_pin, val);
                        sda_lvl = (val != 0);
                        queue_reader.advance();
                    }
                }
            }
            else if (packet.type == I2C_Packet_Type::I2C_DATA)
            {
                // Push 8 bits into the queue
                for (int i = 7; i >= 0; --i)
                {
                    queue_writer.insert_with_backoff((packet.value >> i) & 0x01U, backoff_fast);
                }

                // If we are already in READ_BYTE and waiting for the first bit, drive it now
                if (state == I2C_State::READ_BYTE && bit_count == 0)
                {
                    if (queue_reader.try_advance())
                    {
                        uint8_t val = queue_reader.peek();
                        pin_write(config.sda_pin, val);
                        sda_lvl = (val != 0);
                        queue_reader.advance();
                    }
                }
            }
            start_func();
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
              pin_write_t pin_write,
              pin_read_t pin_read)
    : I2C_Base(halt_func, start_func, std::move(pin_write), std::move(pin_read))
    , config(std::move(config))
    {
    }

    void process_bit(const std::pair<std::uint8_t, std::uint8_t>& pair, const uint32_t delta)
    {
        // Slave as a proxy doesn't process bits from local pins in this model,
        // it only responds to network packets.
        (void)pair;
        (void)delta;
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
        I2C_Packet packet{};
        const auto received = recv(fd, &packet, sizeof(packet), 0);
        if (received != sizeof(packet))
        {
            return;
        }

        switch (packet.type)
        {
            case I2C_Packet_Type::I2C_START:
                start_local();
                matched_address = false;
                break;

            case I2C_Packet_Type::I2C_STOP:
                stop_local();
                matched_address = false;
                break;

            case I2C_Packet_Type::I2C_ADDRESS:
            {
                start_local(); // Ensure START if not already
                bit_bang_byte_local(packet.value);
                bool ack = read_ack_local();
                matched_address = (packet.value >> 1U) == config.address;
                // If matched, we must ACK back to Master
                if (matched_address)
                {
                    ack = true;
                }
                send_packet(I2C_Packet_Type::I2C_ACK, ack ? 1 : 0);
                break;
            }

            case I2C_Packet_Type::I2C_WRITE_BYTE:
                bit_bang_byte_local(packet.value);
                send_packet(I2C_Packet_Type::I2C_ACK, read_ack_local() ? 1 : 0);
                break;

            case I2C_Packet_Type::I2C_READ_BYTE:
                send_packet(I2C_Packet_Type::I2C_DATA, read_byte_local());
                break;

            case I2C_Packet_Type::I2C_ACK:
                write_ack_local(packet.value != 0);
                break;

            default:
                break;
        }
    }

    void send_packet(I2C_Packet_Type type, uint8_t value)
    {
        I2C_Packet packet{ type, value };
        send(config.master_fd, &packet, sizeof(packet), 0);
    }

    void bit_bang_byte_local(uint8_t value)
    {
        for (int i = 7; i >= 0; --i)
        {
            pin_write(config.sda_pin, (value >> i) & 0x01U);
            pin_write(config.scl_pin, 1);
            pin_write(config.scl_pin, 0);
        }
    }

    uint8_t read_byte_local()
    {
        uint8_t value = 0;
        pin_write(config.sda_pin, 1); // Release SDA
        for (int i = 7; i >= 0; --i)
        {
            pin_write(config.scl_pin, 1);
            value = static_cast<uint8_t>(value | (pin_read(config.sda_pin) << i));
            pin_write(config.scl_pin, 0);
        }
        return value;
    }

    void start_local()
    {
        pin_write(config.sda_pin, 1);
        pin_write(config.scl_pin, 1);
        pin_write(config.sda_pin, 0);
        pin_write(config.scl_pin, 0);
    }

    void stop_local()
    {
        pin_write(config.sda_pin, 0);
        pin_write(config.scl_pin, 1);
        pin_write(config.sda_pin, 1);
    }

    bool read_ack_local()
    {
        pin_write(config.sda_pin, 1); // Release SDA
        pin_write(config.scl_pin, 1);
        bool ack = (pin_read(config.sda_pin) == 0);
        pin_write(config.scl_pin, 0);
        return ack;
    }

    void write_ack_local(bool ack)
    {
        pin_write(config.sda_pin, ack ? 0 : 1);
        pin_write(config.scl_pin, 1);
        pin_write(config.scl_pin, 0);
        pin_write(config.sda_pin, 1); // Release SDA
    }
};
