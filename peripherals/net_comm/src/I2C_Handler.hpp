#include <array>
#include <cstdint>
#include <netinet/in.h>
#include <vector>
#include <unordered_map>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>

#include "Protocol.hpp"
#include "CircularBufferQueue.hpp"
#include "zero_mate/external_peripheral.hpp"

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
    std::uint32_t bus_id{ 0 };
    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
};

struct I2C_Slave_P
{
    std::uint32_t bus_id{ 0 };
    std::uint8_t address{ 0 };
    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
    int master_fd{ -1 };
};

struct I2C_HandlerContext
{
    zero_mate::IExternal_Peripheral::Halt_t halt;
    zero_mate::IExternal_Peripheral::Start_t start;
    std::function<void(std::uint8_t, std::uint8_t)> pin_write;
    std::function<std::uint8_t(std::uint8_t)> pin_read;
    const std::atomic<std::uint64_t>* total_cycles;
    TSP::BF::SemBackoff& reader_backoff;
    TSP::BF::SemBackoff& writer_backoff;
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
    static constexpr std::uint8_t HANDSHAKE_MAGIC_BYTE = 0x5A;

    pin_write_t pin_write;
    pin_read_t pin_read;
    bool scl_lvl = true;
    bool sda_lvl = true;
    I2C_State state = I2C_State::IDLE;
    uint8_t bit_count = 0;
    bool is_read = false;
    bool ack_from_slave = false;

    std::array<uint8_t, 16> slave_send_buf{ };
    size_t slave_send_idx = 0;

    std::jthread receiver;
    std::atomic<bool> running{ false };
    std::array<int, 2> close_pipefd{ -1, -1 };

    zero_mate::IExternal_Peripheral::Halt_t halt_func;
    zero_mate::IExternal_Peripheral::Start_t start_func;
    const std::atomic<std::uint64_t>* m_total_cycles;

    I2C_Base(const I2C_HandlerContext& ctx)
    : pin_write(std::move(ctx.pin_write))
    , pin_read(std::move(ctx.pin_read))
    , halt_func(ctx.halt)
    , start_func(ctx.start)
    , m_total_cycles(ctx.total_cycles)
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

    virtual void add_slave(int fd, std::uint32_t slave_id)
    {
        (void)fd;
        (void)slave_id;
    }

    virtual void remove_slave(std::uint32_t slave_id)
    {
        (void)slave_id;
    }

    virtual std::size_t get_slave_count() const
    {
        return 0;
    }

    void receiver_stop()
    {
        if (running.exchange(false))
        {
            receiver.request_stop();
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

    [[nodiscard]] bool is_alive() const
    {
        return running.load();
    }
};

template<std::size_t Size>
class I2C_Master final : public I2C_Base
{
    static constexpr std::size_t MAX_SLAVES = 16;

    I2C_Master_P config;
    std::array<uint32_t, BUFFER_SIZE> buf{ 0 };
    size_t buf_idx = 0;

    TSP::Queue::Buffer<std::uint8_t, BUFFER_SIZE> queue_buf;
    TSP::Queue::Writer<std::uint8_t, BUFFER_SIZE> queue_writer;
    TSP::Queue::Reader<std::uint8_t, BUFFER_SIZE> queue_reader;

    TSP::BF::SemBackoff& m_reader_backoff;
    TSP::BF::SemBackoff& m_writer_backoff;

    std::array<int, 2> wake_pipefd{ -1, -1 };
    mutable std::mutex slave_fds_mutex;

    std::array<int, MAX_SLAVES> m_slave_fds{ };
    std::array<std::uint32_t, MAX_SLAVES> m_slave_ids{ };
    std::size_t m_slave_count = 0;
    std::unordered_map<int, std::size_t> m_fd_to_idx;
    std::unordered_map<std::uint32_t, int> m_slave_id_to_fd;

    uint8_t shift_reg = 0;

public:
    I2C_Master(const I2C_Master_P& config, const I2C_HandlerContext& ctx)
    : I2C_Base(ctx)
    , config(config)
    , queue_writer(&queue_buf)
    , queue_reader(&queue_buf)
    , m_reader_backoff(ctx.reader_backoff)
    , m_writer_backoff(ctx.writer_backoff)
    {
        m_slave_fds.fill(-1);
    }

    ~I2C_Master() override
    {
        I2C_Base::receiver_stop();

        {
            std::lock_guard<std::mutex> lock(slave_fds_mutex);
            for (std::size_t i = 0; i < m_slave_count; ++i)
            {
                send_disconnect(m_slave_fds[i]);
                close(m_slave_fds[i]);
            }
        }

        if (wake_pipefd[0] != -1)
        {
            close(wake_pipefd[0]);
        }
        if (wake_pipefd[1] != -1)
        {
            close(wake_pipefd[1]);
        }
    }

    void add_slave(int fd, std::uint32_t slave_id) override
    {
        {
            std::lock_guard<std::mutex> lock(slave_fds_mutex);
            if (m_slave_count < MAX_SLAVES)
            {
                m_slave_fds[m_slave_count] = fd;
                m_slave_ids[m_slave_count] = slave_id;
                m_fd_to_idx[fd] = m_slave_count;
                m_slave_id_to_fd[slave_id] = fd;
                m_slave_count++;
            }
        }
        wake_thread();
    }

    void remove_slave(std::uint32_t slave_id) override
    {
        {
            std::lock_guard<std::mutex> lock(slave_fds_mutex);
            if (m_slave_id_to_fd.contains(slave_id))
            {
                _remove_slave_at(m_fd_to_idx.at(m_slave_id_to_fd.at(slave_id)));
            }
        }
        this->start_func();
        wake_thread();
    }

    std::size_t get_slave_count() const override
    {
        std::lock_guard<std::mutex> lock(slave_fds_mutex);
        return m_slave_count;
    }

private:
    void _remove_slave_at(std::size_t idx)
    {
        send_disconnect(m_slave_fds[idx]);
        close(m_slave_fds[idx]);
        m_slave_id_to_fd.erase(m_slave_ids[idx]);
        m_fd_to_idx.erase(m_slave_fds[idx]);

        if (idx < m_slave_count - 1)
        {
            m_slave_fds[idx] = m_slave_fds[m_slave_count - 1];
            m_slave_ids[idx] = m_slave_ids[m_slave_count - 1];
            m_fd_to_idx[m_slave_fds[idx]] = idx;
            m_slave_id_to_fd[m_slave_ids[idx]] = m_slave_fds[idx];
        }

        m_slave_count--;
        m_slave_fds[m_slave_count] = -1;
    }

    void send_disconnect(int fd)
    {
        handshake::DisconnectMessage msg{ };
        msg.config.protocol_id = handshake::ProtocolID::I2C;
        msg.config.config.i2c.bus_id = config.bus_id;
        msg.config.config.i2c.is_master = 1;

        send(fd, &msg, sizeof(msg), 0);
    }

    void wake_thread()
    {
        if (wake_pipefd[1] != -1)
        {
            bool msg = true;
            write(wake_pipefd[1], &msg, sizeof(bool));
        }
    }

    void remove_slave_by_fd(int fd)
    {
        {
            std::lock_guard<std::mutex> lock(slave_fds_mutex);
            if (m_fd_to_idx.contains(fd))
            {
                _remove_slave_at(m_fd_to_idx[fd]);
            }
        }
        this->start_func();
        wake_thread();
    }

public:
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
        running = true;
        pipe(wake_pipefd.data());
        this->receiver = std::jthread([this](std::stop_token stop_token) { this->receiver_thread(stop_token); });
    }

    void receiver_thread(std::stop_token stop_token)
    {
        pipe(close_pipefd.data());

        while (!stop_token.stop_requested() && running)
        {
            std::vector<struct pollfd> fds;
            fds.emplace_back(close_pipefd[0], POLLIN, 0);
            fds.emplace_back(wake_pipefd[0], POLLIN, 0);

            {
                std::lock_guard<std::mutex> lock(slave_fds_mutex);
                for (std::size_t i = 0; i < m_slave_count; ++i)
                {
                    fds.emplace_back(m_slave_fds[i], POLLIN, 0);
                }
            }

            if (poll(fds.data(), fds.size(), -1) <= 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                break;
            }

            if (fds[0].revents & POLLIN)
            {
                break;
            }

            if (fds[1].revents & POLLIN)
            {
                bool msg;
                read(wake_pipefd[0], &msg, sizeof(bool));
                continue; // Rebuild fds
            }

            bool rebuild = false;
            for (size_t i = 2; i < fds.size(); ++i)
            {
                if (fds[i].revents & (POLLIN | POLLERR | POLLHUP))
                {
                    if (fds[i].revents & POLLIN)
                    {
                        if (!receive_from_slave(fds[i].fd))
                        {
                            remove_slave_by_fd(fds[i].fd);
                            rebuild = true;
                        }
                    }
                    else
                    {
                        remove_slave_by_fd(fds[i].fd);
                        rebuild = true;
                    }
                }
            }
            if (rebuild)
            {
                continue;
            }
        }
        running = false;
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
            const bool slave_drives_next = (state == I2C_State::READ_BYTE && bit_count < 8) ||
                                           (state == I2C_State::RESPONSE && ack_from_slave && bit_count == 0);

            if (slave_drives_next)
            {
                if (queue_reader.try_advance())
                {
                    uint8_t val = queue_reader.peek();
                    pin_write(config.sda_pin, val);
                    sda_lvl = (val != 0);
                    queue_reader.advance();
                    m_writer_backoff.wake();
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

        {
            std::lock_guard<std::mutex> lock(slave_fds_mutex);
            if (wait_for_response && m_slave_count > 0)
            {
                halt_func();
            }
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
                    m_writer_backoff.wake();
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
        std::lock_guard<std::mutex> lock(slave_fds_mutex);
        for (std::size_t i = 0; i < m_slave_count; ++i)
        {
            send(m_slave_fds[i], &packet, sizeof(packet), 0);
        }
    }

    bool receive_from_slave(const int fd)
    {
        uint8_t buffer[16];
        ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0)
        {
            return false;
        }

        if (buffer[0] == HANDSHAKE_MAGIC_BYTE)
        {
            // It's likely a DisconnectMessage
            return false;
        }

        if (n != sizeof(I2C_Packet))
        {
            return false;
        }

        const auto* packet = reinterpret_cast<const I2C_Packet*>(buffer);

        if (packet->type == I2C_Packet_Type::I2C_ACK)
        {
            ack_from_slave = (packet->value != 0);
            // For RESPONSE state, the master expects 1 bit (ACK)
            queue_writer.insert_with_backoff(ack_from_slave ? 0 : 1, m_writer_backoff);
            m_reader_backoff.wake();

            // If we are in the RESPONSE state and waiting for this bit
            if (state == I2C_State::RESPONSE && bit_count == 0)
            {
                if (queue_reader.try_advance())
                {
                    uint8_t val = queue_reader.peek();
                    pin_write(config.sda_pin, val);
                    sda_lvl = (val != 0);
                    queue_reader.advance();
                    m_writer_backoff.wake();
                }
            }
        }
        else if (packet->type == I2C_Packet_Type::I2C_DATA)
        {
            // Push 8 bits into the queue
            for (int i = 7; i >= 0; --i)
            {
                queue_writer.insert_with_backoff((packet->value >> i) & 0x01U, m_writer_backoff);
                m_reader_backoff.wake();
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
                    m_writer_backoff.wake();
                }
            }
        }
        start_func();
        return true;
    }
};

template<std::size_t Size>
class I2C_Slave final : public I2C_Base
{
    I2C_Slave_P config;
    bool matched_address = false;
    uint8_t shift_reg = 0;

    TSP::Queue::Buffer<std::uint8_t, BUFFER_SIZE> queue_buf;
    TSP::Queue::Writer<std::uint8_t, BUFFER_SIZE> queue_writer;
    TSP::Queue::Reader<std::uint8_t, BUFFER_SIZE> queue_reader;

    TSP::BF::SemBackoff& m_reader_backoff;
    TSP::BF::SemBackoff& m_writer_backoff;

public:
    I2C_Slave(const I2C_Slave_P& config, const I2C_HandlerContext& ctx)
    : I2C_Base(ctx)
    , config(config)
    , queue_writer(&queue_buf)
    , queue_reader(&queue_buf)
    , m_reader_backoff(ctx.reader_backoff)
    , m_writer_backoff(ctx.writer_backoff)
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
        running = true;
        this->receiver = std::jthread([this](std::stop_token stop_token) { this->receiver_thread(stop_token); });
    }

    void receiver_thread(std::stop_token stop_token)
    {
        pipe(close_pipefd.data());

        std::array<struct pollfd, 2> fds{ 0 };
        fds[0] = { .fd = close_pipefd[0], .events = POLLIN, .revents = 0 };
        fds[1] = { .fd = config.master_fd, .events = POLLIN, .revents = 0 };

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
                if (!receive_from_master(config.master_fd))
                {
                    break;
                }
            }
        }
        running = false;
    }

private:
    [[nodiscard]] bool receive_from_master(int fd)
    {
        uint8_t buffer[16];
        const auto received = recv(fd, buffer, sizeof(buffer), 0);
        if (received <= 0)
        {
            return false;
        }

        if (buffer[0] == HANDSHAKE_MAGIC_BYTE)
        {
            return false; // Disconnect
        }

        if (received != sizeof(I2C_Packet))
        {
            return false;
        }

        const auto* packet = reinterpret_cast<const I2C_Packet*>(buffer);

        switch (packet->type)
        {
            case I2C_Packet_Type::I2C_START:
                start_local();
                matched_address = false;
                break;

            case I2C_Packet_Type::I2C_STOP:
                stop_local();
                matched_address = false;
                break;

            case I2C_Packet_Type::I2C_ADDRESS: {
                start_local(); // Ensure START if not already
                bit_bang_byte_local(packet->value);
                bool ack = read_ack_local();
                matched_address = (packet->value >> 1U) == config.address;
                // If matched, we must ACK back to Master
                if (matched_address)
                {
                    ack = true;
                }
                send_packet(I2C_Packet_Type::I2C_ACK, ack ? 1 : 0);
                break;
            }

            case I2C_Packet_Type::I2C_WRITE_BYTE:
                bit_bang_byte_local(packet->value);
                send_packet(I2C_Packet_Type::I2C_ACK, read_ack_local() ? 1 : 0);
                break;

            case I2C_Packet_Type::I2C_READ_BYTE:
                send_packet(I2C_Packet_Type::I2C_DATA, read_byte_local());
                break;

            case I2C_Packet_Type::I2C_ACK:
                write_ack_local(packet->value != 0);
                break;

            default:
                break;
        }
        return true;
    }

    void send_packet(I2C_Packet_Type type, uint8_t value)
    {
        I2C_Packet packet{ .type = type, .value = value };
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
