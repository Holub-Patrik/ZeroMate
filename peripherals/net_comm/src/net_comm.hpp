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
#include "Protocol.hpp"
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

using conn_id = std::uint64_t;
using pin_pair = std::pair<std::uint8_t, std::uint8_t>;

template<std::size_t QUEUE_SIZE>
using protocol_variant = std::variant<UART_Handler<QUEUE_SIZE>, I2C_Master<QUEUE_SIZE>, I2C_Slave<QUEUE_SIZE>>;

namespace command
{
    struct AddSlave
    {
        int fd;
        std::uint32_t slave_id;
    };

    struct RemoveSlave
    {
        std::uint32_t slave_id;
    };

    struct GetSlaveCount
    {
    };

    using Command = std::variant<AddSlave, RemoveSlave, GetSlaveCount>;

    enum class ResponseStatus
    {
        Success,
        Fail
    };

    struct Response
    {
        ResponseStatus status;
        std::uint32_t data; // e.g., slave count
    };
}

class GPIOConnection;

class GPIOServer final
{
public:
    static constexpr std::uint64_t BACKOFF_CYCLES_FAST = 1000;
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
        : out_queue_writer(BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED)
        , out_queue_reader(BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED)
        , handler_queue_writer(BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED)
        , handler_queue_reader(BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED)
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
    TSP::BF::SemBackoff pin_write_writer_backoff{ BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED };
    TSP::BF::SemBackoff pin_write_reader_backoff{ BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED };

    std::array<TSP::Queue::Buffer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_buffers;
    std::array<TSP::Queue::Writer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_writers;
    std::array<ConnectionBackoffs, MAX_CONNECTION_COUNT> m_backoffs;

    std::array<bool, MAX_CONNECTION_COUNT> connection_bit_map{ false };

    std::array<std::jthread, MAX_CONNECTION_COUNT> connection_threads;
    std::array<std::atomic<bool>, MAX_CONNECTION_COUNT> connection_running;

    std::array<std::unique_ptr<GPIOConnection>, MAX_CONNECTION_COUNT> active_connections;
    std::array<conn_info, MAX_CONNECTION_COUNT> connection_data;

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
    void add_slave_to_master(std::uint32_t bus_id, int fd, std::uint32_t slave_id);
    void remove_slave_from_master(std::uint32_t slave_id);
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

    using handler_t = std::variant<UART_Handler<GPIOServer::BUFFER_SIZE>,
                                   I2C_Master<GPIOServer::BUFFER_SIZE>,
                                   I2C_Slave<GPIOServer::BUFFER_SIZE>>;

    std::optional<handler_t> m_handler;

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

    command::Response execute(const command::Command& cmd);
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
