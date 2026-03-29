// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.hpp
/// \brief Defines a remote GPIO peripheral communicating via UDP
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <utility>
#include <variant>
#include <vector>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <array>
#include <optional>
#include <unordered_map>

#ifndef ZERO_MATE_UNIT_TESTS
    #include "imgui.h"
#endif

#include "zero_mate/external_peripheral.hpp"
#include "CircularBufferQueue.hpp"
#include "Util.hpp"
#include "UART_Handler.hpp"
#include "I2C_Handler.hpp"

using Protocol = std::variant<UART_P, I2C_Master_P, I2C_Slave_P>;

enum class ConnectionStatus
{
    Defined,
    Connecting,
    Connected,
    Failed
};

using conn_info = struct conn_info_struct
{
    Protocol protocol;

    bool explicit_clock{ };
    std::int8_t clock_unit{ };
    std::uint32_t clock_value{ };

    in_port_t opened_port{ }; // receiver port
    std::uint32_t net_id{ };

    ConnectionStatus status = ConnectionStatus::Defined;
    std::string error_msg;
    bool is_responder = false;

    // Remote side info
    std::string remote_ip;
    int remote_port;
    in_port_t remote_data_port{ 0 };

    int sockfd{ -1 };

    std::chrono::time_point<std::chrono::steady_clock> start_time;
};

namespace handshake
{
    static constexpr std::uint8_t MAGIC_BYTE = 0x5A;

    enum class MessageType : std::uint8_t
    {
        Conf = 0,
        Response = 1,
        FinalResponse = 2,
        Disconnect = 3,
    };

    enum class ProtocolID : std::uint8_t
    {
        UART = 0,
        I2C_Master = 1,
        I2C_Slave = 2,
    };

    struct UARTConfig
    {
        std::uint32_t baudrate;
        std::uint8_t data_bits;
        std::uint8_t start_bits;
        std::uint8_t parity_bits;
        std::uint8_t stop_bits;
    } __attribute__((packed));

    struct I2CConfig
    {
        std::uint32_t bus_id;
        std::uint8_t is_master;
        std::uint8_t address;
    } __attribute__((packed));

    struct ConfMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Conf;
        ProtocolID protocol_id;
        std::uint16_t port;
        std::uint32_t net_id;
        union {
            UARTConfig uart;
            I2CConfig i2c;
        } config;
    } __attribute__((packed));

    struct ResponseMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Response;
        std::uint8_t status; // 1: Accept, 0: Decline
        std::uint16_t port;
        std::uint32_t net_id;
    } __attribute__((packed));

    struct FinalResponseMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::FinalResponse;
        std::uint8_t status; // 1: Accept, 0: Decline
        std::uint32_t net_id;
    } __attribute__((packed));

    struct DisconnectMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Disconnect;
        std::uint32_t net_id;
    } __attribute__((packed));
}

using conn_id = std::uint64_t;
using pin_pair = std::pair<std::uint8_t, std::uint8_t>;

template<std::size_t QUEUE_SIZE>
using protocol_variant = std::variant<UART_Handler<QUEUE_SIZE>, I2C_Master<QUEUE_SIZE>, I2C_Slave<QUEUE_SIZE>>;

template<typename Type, std::size_t Size>
struct BitProcessorContext
{
    TSP::Queue::Buffer<Type, Size>* buf;
    const std::atomic<std::uint64_t>* total_cycles;
    TSP::BF::SemBackoff& reader_backoff;
    TSP::BF::SemBackoff& writer_backoff;
};

template<typename Type, std::size_t Size, typename Handler>
class BitProcessor final
{
public:
    using pin_write_t = std::function<void(std::uint8_t, std::uint8_t)>;
    using pin_read_t = std::function<std::uint8_t(std::uint8_t)>;

private:
    Handler handler;
    TSP::Queue::Reader<Type, Size> queue_reader;
    TSP::BF::SemBackoff& m_reader_backoff;
    TSP::BF::SemBackoff& m_writer_backoff;
    const std::atomic<std::uint64_t>* m_total_cycles;

    std::jthread sender;

public:
    template<typename... Args>
    BitProcessor(BitProcessorContext<Type, Size> ctx, Args&&... args)
    : handler(std::forward<Args>(args)...)
    , queue_reader(ctx.buf)
    , m_reader_backoff(ctx.reader_backoff)
    , m_writer_backoff(ctx.writer_backoff)
    , m_total_cycles(ctx.total_cycles)
    {
    }

    ~BitProcessor()
    {
        stop();
    }

    BitProcessor(const BitProcessor&) = delete;
    BitProcessor& operator=(const BitProcessor&) = delete;

    BitProcessor(BitProcessor&&) = delete;
    BitProcessor& operator=(BitProcessor&&) = delete;

    void start()
    {
        sender = std::jthread{ [this](std::stop_token stop_token) { this->run_sender(stop_token); } };
        handler.start_receiver();
    }

    void stop()
    {
        sender.request_stop();
        m_reader_backoff.wake();
        handler.receiver_stop();
    }

    [[nodiscard]] bool is_running() const
    {
        return !sender.get_stop_token().stop_requested() && handler.is_alive();
    }

    void add_slave(int fd, in_port_t handshake_port)
    {
        handler.add_slave(fd, handshake_port);
    }

    void remove_slave(in_port_t port)
    {
        handler.remove_slave(port);
    }

    [[nodiscard]] std::size_t get_slave_count() const
    {
        return handler.get_slave_count();
    }

private:
    void run_sender(std::stop_token stop_token)
    {
        std::uint64_t last_cycles = m_total_cycles->load();

        while (!stop_token.stop_requested())
        {
            if (!queue_reader.try_advance())
            {
                m_reader_backoff.wait(
                [this, &stop_token]() { return stop_token.stop_requested() || queue_reader.try_advance(); });
                continue;
            }
            m_reader_backoff.reset();

            const auto pair = queue_reader.peek();
            queue_reader.advance();
            m_writer_backoff.wake();

            const std::uint64_t now_cycles = m_total_cycles->load();
            const auto delta = static_cast<std::uint32_t>(now_cycles - last_cycles);
            last_cycles = now_cycles;
            handler.process_bit(pair, delta);
        }
    }
};

class GPIOConnection;

class GPIOServer final
{
public:
    static constexpr std::uint64_t BACKOFF_CYCLES = 1000;
    static constexpr std::uint64_t BACKOFF_CYCLES_RELAXED = 20'000;
    static constexpr auto NET_WAIT_TIME = std::chrono::microseconds{ 100 };
    static constexpr std::size_t MAX_CONNECTION_COUNT = 16;
    static constexpr std::size_t BUFFER_COUNT = MAX_CONNECTION_COUNT;
    static constexpr std::size_t BUFFER_SIZE = 512;
    static constexpr std::size_t QUEUE_SIZE = 64;

    struct ConnectionBackoffs
    {
        TSP::BF::SemBackoff out_queue_writer;
        TSP::BF::SemBackoff out_queue_reader;
        TSP::BF::SemBackoff handler_queue_writer;
        TSP::BF::SemBackoff handler_queue_reader;

        ConnectionBackoffs()
        : out_queue_writer(BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED, "out_queue_writer")
        , out_queue_reader(BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED, "out_queue_reader")
        , handler_queue_writer(BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED, "handler_queue_writer")
        , handler_queue_reader(BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED, "handler_queue_reader")
        {
        }

        ConnectionBackoffs(const ConnectionBackoffs&) = delete;
        ConnectionBackoffs& operator=(const ConnectionBackoffs&) = delete;
        ConnectionBackoffs(ConnectionBackoffs&&) = delete;
        ConnectionBackoffs& operator=(ConnectionBackoffs&&) = delete;
    };

private:
    TSP::Queue::Buffer<pin_pair, QUEUE_SIZE> pin_write_queue_buf{ };
    TSP::Queue::Reader<pin_pair, QUEUE_SIZE> pin_write_queue_reader;
    TSP::Queue::Writer<pin_pair, QUEUE_SIZE> pin_write_queue_writer;
    Spinlock pin_write_spinlock;

    TSP::BF::SemBackoff m_pin_write_writer_backoff{ BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED, "pin_write_writer" };
    TSP::BF::SemBackoff m_pin_write_reader_backoff{ BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED, "pin_write_reader" };

    // a bit map lookup might be best to assign new threads
    std::array<TSP::Queue::Buffer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_buffers;
    std::array<TSP::Queue::Writer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_writers;

    // this could be converted into a bit map, but bit instruction are extra instructions
    std::array<bool, MAX_CONNECTION_COUNT> connection_bit_map{ false };

    std::array<ConnectionBackoffs, MAX_CONNECTION_COUNT> m_backoffs;

    std::array<std::jthread, MAX_CONNECTION_COUNT> connection_threads;
    std::array<std::unique_ptr<GPIOConnection>, MAX_CONNECTION_COUNT> active_connections;
    std::array<conn_info, MAX_CONNECTION_COUNT> connection_data;
    std::array<std::atomic<bool>, MAX_CONNECTION_COUNT> connection_running;

    FastMap pin_to_conn_id;
    FastMap net_id_to_conn_id;
    std::unordered_map<std::uint32_t, std::size_t> m_net_id_to_idx;
    std::unordered_map<std::uint32_t, std::size_t> m_bus_id_to_idx;

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin;
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t func_read_pin;
    zero_mate::IExternal_Peripheral::Halt_t func_halt;
    zero_mate::IExternal_Peripheral::Start_t func_start;
    zero_mate::utils::CLogging_System* logging_system;
    const std::atomic<std::uint64_t>* m_total_cycles;

    std::atomic<bool> m_running{ true };
    std::jthread m_pin_write_thread;
    std::jthread m_server_thread;
    int m_handshake_socket{ -1 };
    uint16_t m_handshake_port{ 0 };

    // Pin write entirely here since there will be multiple writers, so spinlock is added to ensure safety
    void pin_write(const std::stop_token& stop_token);
    void unmap_connection(const std::size_t conn_index);
    [[nodiscard]] std::uint8_t find_free_index() const noexcept;

    // Handshake helpers
    void handle_handshake();
    void handle_conf_msg(const handshake::ConfMessage& msg, const struct sockaddr_in& addr);
    void handle_response_msg(const handshake::ResponseMessage& msg, const struct sockaddr_in& addr);
    void handle_final_response_msg(const handshake::FinalResponseMessage& msg, const struct sockaddr_in& addr);
    void handle_disconnect_msg(const handshake::DisconnectMessage& msg, const struct sockaddr_in& addr);
    void cleanup_finished_connections();

public:
    GPIOServer() = delete;

    GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin,
               zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t func_read_pin,
               zero_mate::IExternal_Peripheral::Halt_t func_halt,
               zero_mate::IExternal_Peripheral::Start_t func_start,
               zero_mate::utils::CLogging_System* logging_system,
               const std::atomic<std::uint64_t>* total_cycles);
    ~GPIOServer();

    GPIOServer(const GPIOServer& other) = delete;
    GPIOServer& operator=(const GPIOServer& other) = delete;

    GPIOServer(GPIOServer&& other) = delete;
    GPIOServer& operator=(GPIOServer&& other) = delete;

    void Init(uint16_t handshake_port);
    [[nodiscard]] bool Is_Initialized() const noexcept;
    void write_to_pin(const std::uint8_t pin, const std::uint8_t value);
    void route_pin_info(const pin_pair pin_info);
    void add_connection(const conn_info& info);
    void remove_connection(std::size_t i);
    void construct_connection(const conn_info& info);
    void add_slave_to_master(std::uint32_t bus_id, int fd, in_port_t handshake_port);
    void remove_slave_from_master(in_port_t port);
    void run(const std::stop_token& stop_token);
    void stop();

    // Handshake initiation
    void initiate_handshake(std::size_t idx);

    // used by GPIOConnection
    [[nodiscard]] zero_mate::IExternal_Peripheral::Halt_t get_halt() const
    {
        return func_halt;
    }
    [[nodiscard]] zero_mate::IExternal_Peripheral::Start_t get_start() const
    {
        return func_start;
    }
    [[nodiscard]] zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t get_set_pin() const
    {
        return func_set_pin;
    }
    [[nodiscard]] zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t get_read_pin() const
    {
        return func_read_pin;
    }
    [[nodiscard]] conn_info& get_connection_info(std::size_t idx)
    {
        return connection_data[idx];
    }
    [[nodiscard]] TSP::Queue::Buffer<pin_pair, BUFFER_SIZE>* get_out_queue_buffer(std::size_t idx)
    {
        return &out_queue_buffers[idx];
    }
    [[nodiscard]] ConnectionBackoffs& get_backoffs(std::size_t idx)
    {
        return m_backoffs[idx];
    }
    [[nodiscard]] std::atomic<bool>& is_server_running()
    {
        return m_running;
    }
    [[nodiscard]] std::atomic<bool>& is_connection_running(std::size_t idx)
    {
        return connection_running[idx];
    }
    [[nodiscard]] const std::atomic<std::uint64_t>* get_total_cycles() const
    {
        return m_total_cycles;
    }

    // UI access
    [[nodiscard]] const auto& get_connection_bit_map() const
    {
        return connection_bit_map;
    }
    [[nodiscard]] const auto& get_connection_data() const
    {
        return connection_data;
    }
    [[nodiscard]] std::size_t get_slave_count(std::size_t idx) const;
};

class GPIOConnection final
{
private:
    conn_info& connection;
    GPIOServer::ConnectionBackoffs& m_backoffs;

    TSP::Queue::Reader<pin_pair, GPIOServer::BUFFER_SIZE> m_queue_reader;

    int m_socket;
    struct sockaddr_in m_other_side;

    zero_mate::IExternal_Peripheral::Halt_t m_halt;
    zero_mate::IExternal_Peripheral::Start_t m_start;
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
    GPIOServer& m_server;

    std::atomic<bool>& m_server_running;
    std::atomic<bool>& m_connection_running;
    const std::atomic<std::uint64_t>* m_total_cycles;

    using processor_t =
    std::variant<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE, UART_Handler<GPIOServer::BUFFER_SIZE>>,
                 BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE, I2C_Master<GPIOServer::BUFFER_SIZE>>,
                 BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE, I2C_Slave<GPIOServer::BUFFER_SIZE>>>;

    std::optional<processor_t> m_processor;

public:
    GPIOConnection() = delete;
    GPIOConnection(std::size_t idx, GPIOServer& server);

    ~GPIOConnection();

    GPIOConnection(const GPIOConnection& other) = delete;
    void run(std::stop_token stop_token);

    // Helper for SM
    pin_pair read_queue();
    void send_to_network(const std::vector<std::uint8_t>& data);
    void write_to_pin(std::uint8_t pin, std::uint8_t value);
    void halt()
    {
        m_halt();
    }
    void start()
    {
        m_start();
    }
    [[nodiscard]] bool is_running() const
    {
        return m_server_running.load() && m_connection_running.load();
    }

    void add_slave(int fd, in_port_t handshake_port)
    {
        if (m_processor.has_value())
        {
            std::visit([fd, handshake_port](auto& p) { p.add_slave(fd, handshake_port); }, *m_processor);
        }
    }

    void remove_slave(in_port_t port)
    {
        if (m_processor.has_value())
        {
            std::visit([port](auto& p) { p.remove_slave(port); }, *m_processor);
        }
    }

    [[nodiscard]] std::size_t get_slave_count() const
    {
        if (m_processor.has_value())
        {
            return std::visit([](auto& p) { return p.get_slave_count(); }, *m_processor);
        }
        return 0;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
/// \class CRemote_GPIO
/// \brief External peripheral for UDP-based remote GPIO control.
// ---------------------------------------------------------------------------------------------------------------------
class Remote_GPIO final : public zero_mate::IExternal_Peripheral
{
public:
    Remote_GPIO() = delete;
    Remote_GPIO(const Remote_GPIO&) = delete;
    Remote_GPIO& operator=(const Remote_GPIO&) = delete;
    Remote_GPIO(Remote_GPIO&&) = delete;
    Remote_GPIO& operator=(Remote_GPIO&&) = delete;

    explicit Remote_GPIO(const std::string& name,
                         const std::vector<std::uint32_t>& pins,
                         zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                         zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                         zero_mate::IExternal_Peripheral::Halt_t halt,
                         zero_mate::IExternal_Peripheral::Start_t start,
                         zero_mate::utils::CLogging_System* logging_system);

    ~Remote_GPIO() final;

#ifndef ZERO_MATE_UNIT_TESTS
    void Render() final;
    void Set_ImGui_Context(void* context) final;
#endif

    void Increment_Passed_Cycles(std::uint32_t count) final;
    void GPIO_Subscription_Callback(std::uint32_t pin_idx) final;

private:
#ifndef ZERO_MATE_UNIT_TESTS
    // UI Rendering
    void Render_Settings();
    void Render_Mappings();
#endif

    // IExternal dependencies
    std::string name;
    std::vector<std::uint32_t> pins; // Available local pins
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin;
    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin;
    zero_mate::IExternal_Peripheral::Halt_t halt;
    zero_mate::IExternal_Peripheral::Start_t start;
    zero_mate::utils::CLogging_System* logging_system;

#ifndef ZERO_MATE_UNIT_TESTS
    ImGuiContext* ImGui_context;
#endif

    std::atomic<bool> m_running{ true };
    std::atomic<uint64_t> m_total_cycles{ 0 };
    GPIOServer server;
    std::jthread server_thread;

    // UI State
#ifndef ZERO_MATE_UNIT_TESTS
    struct AddConnectionState
    {
        int protocol_type = 0; // 0: UART, 1: I2C Master, 2: I2C Slave
        int net_id = 1;

        // UART
        int baudrate = 115200;
        int tx_pin = 0;
        int rx_pin = 0;
        int start_bits = 1;
        int data_bits = 8;
        int parity_bits = 0;
        int stop_bits = 1;
        char ip[64] = "127.0.0.1";
        int port = 12345;

        // I2C
        int scl_pin = 0;
        int sda_pin = 0;
        int address = 0x50;
        int i2c_id = 0;
    } m_ui_add_state;

    int m_ui_view_idx{ -1 };
    int m_ui_handshake_port{ 12344 };

    int ui_selected_local_pin_idx{ 0 };
    int ui_target_net_pin{ 0 };
    int ui_selected_net_pin_source{ 0 };
    int ui_target_local_pin_idx{ 0 };
#endif
};
