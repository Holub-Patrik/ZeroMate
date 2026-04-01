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

namespace net_comm
{
    static constexpr std::size_t MAX_CONNECTION_COUNT = 16;
    static constexpr std::size_t BUFFER_SIZE = 512;
    static constexpr std::size_t QUEUE_SIZE = 64;
}

using Protocol = std::variant<UART_P, I2C_Master_P, I2C_Slave_P>;

enum class ConnectionStatus : std::uint8_t
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

using handler_t =
std::variant<UART_Handler<net_comm::BUFFER_SIZE>, I2C_Master<net_comm::BUFFER_SIZE>, I2C_Slave<net_comm::BUFFER_SIZE>>;

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

    struct HasSlave
    {
        std::uint32_t slave_id;
    };

    using Command = std::variant<AddSlave, RemoveSlave, GetSlaveCount, HasSlave>;

    enum class ResponseStatus : std::uint8_t
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

class GPIOServer;

class GPIOConnection final
{
public:
    GPIOConnection() = default;
    ~GPIOConnection();

    GPIOConnection(const GPIOConnection&) = delete;
    GPIOConnection& operator=(const GPIOConnection&) = delete;

    void init(std::size_t idx, GPIOServer* server);
    void update();
    void run(std::stop_token stop_token);
    void clear();

    [[nodiscard]] bool is_defined() const noexcept
    {
        return m_is_defined;
    }
    void set_defined(bool defined)
    {
        m_is_defined = defined;
    }

    [[nodiscard]] conn_info& get_info()
    {
        return m_info;
    }
    [[nodiscard]] const conn_info& get_info() const
    {
        return m_info;
    }

    void start_thread();
    void stop_thread();

    command::Response execute(const command::Command& cmd);

private:
    std::size_t m_idx{ 0 };
    GPIOServer* m_server{ nullptr };
    conn_info m_info;
    bool m_is_defined{ false };

    std::optional<handler_t> m_handler;
    std::jthread m_thread;
    std::atomic<bool> m_running{ false };

    void write_to_pin(std::uint8_t pin, std::uint8_t value);
};

class GPIOServer final
{
public:
    static constexpr std::uint64_t BACKOFF_CYCLES_FAST = 1000;
    static constexpr std::uint64_t BACKOFF_CYCLES_RELAXED = 20'000;
    static constexpr auto NET_WAIT_TIME = std::chrono::microseconds{ 100 };

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
    };

private:
    TSP::Queue::Buffer<pin_pair, net_comm::QUEUE_SIZE> pin_write_queue_buf{ };
    TSP::Queue::Reader<pin_pair, net_comm::QUEUE_SIZE> pin_write_queue_reader;
    TSP::Queue::Writer<pin_pair, net_comm::QUEUE_SIZE> pin_write_queue_writer;

    Spinlock pin_write_spinlock;
    TSP::BF::SemBackoff pin_write_writer_backoff{ BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED };
    TSP::BF::SemBackoff pin_write_reader_backoff{ BACKOFF_CYCLES_FAST, BACKOFF_CYCLES_RELAXED };

    std::array<TSP::Queue::Buffer<pin_pair, net_comm::BUFFER_SIZE>, net_comm::MAX_CONNECTION_COUNT> out_queue_buffers;
    std::array<TSP::Queue::Writer<pin_pair, net_comm::BUFFER_SIZE>, net_comm::MAX_CONNECTION_COUNT> out_queue_writers;
    std::array<ConnectionBackoffs, net_comm::MAX_CONNECTION_COUNT> m_backoffs;

    std::array<GPIOConnection, net_comm::MAX_CONNECTION_COUNT> m_connections;

    FastMap pin_to_conn_id;
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

    void pin_write(const std::stop_token& stop_token);
    void unmap_connection(const std::size_t conn_index);
    [[nodiscard]] std::uint8_t find_free_index() const noexcept;

    void handle_handshake();
    void handle_conf_msg(const handshake::ConfMessage& msg, const struct sockaddr_in& addr);
    void handle_response_msg(const handshake::ResponseMessage& msg, const struct sockaddr_in& addr);
    void handle_disconnect_msg(const handshake::DisconnectMessage& msg, const struct sockaddr_in& addr);

public:
    GPIOServer() = delete;
    GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin,
               zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t func_read_pin,
               zero_mate::IExternal_Peripheral::Halt_t func_halt,
               zero_mate::IExternal_Peripheral::Start_t func_start,
               zero_mate::utils::CLogging_System* logging_system,
               const std::atomic<std::uint64_t>* total_cycles);
    ~GPIOServer();

    void Init(uint16_t handshake_port);
    [[nodiscard]] bool Is_Initialized() const noexcept;
    void write_to_pin(const std::uint8_t pin, const std::uint8_t value);
    void route_pin_info(const pin_pair pin_info);
    void add_connection(const conn_info& info);
    void remove_connection(std::size_t i);
    void add_slave_to_master(std::uint32_t bus_id, int fd, std::uint32_t slave_id);
    void remove_slave_from_master(std::uint32_t slave_id);
    void run(const std::stop_token& stop_token);
    void stop();

    void initiate_handshake(std::size_t idx);

    // Getters for GPIOConnection
    [[nodiscard]] zero_mate::IExternal_Peripheral::Halt_t get_halt() const
    {
        return func_halt;
    }
    [[nodiscard]] zero_mate::IExternal_Peripheral::Start_t get_start() const
    {
        return func_start;
    }
    [[nodiscard]] zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t get_read_pin() const
    {
        return func_read_pin;
    }
    [[nodiscard]] TSP::Queue::Buffer<pin_pair, net_comm::BUFFER_SIZE>* get_out_queue_buffer(std::size_t idx)
    {
        return &out_queue_buffers[idx];
    }
    [[nodiscard]] ConnectionBackoffs& get_backoffs(std::size_t idx)
    {
        return m_backoffs[idx];
    }
    [[nodiscard]] const std::atomic<std::uint64_t>* get_total_cycles() const
    {
        return m_total_cycles;
    }
    [[nodiscard]] int get_handshake_socket() const
    {
        return m_handshake_socket;
    }

    // UI access
    [[nodiscard]] const std::array<GPIOConnection, net_comm::MAX_CONNECTION_COUNT>& get_connections() const
    {
        return m_connections;
    }
    [[nodiscard]] std::size_t get_slave_count(std::size_t idx) const;
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
        static constexpr int DEFAULT_BAUDRATE = 115200;
        static constexpr int DEFAULT_PORT = 12345;
        static constexpr int DEFAULT_ADDRESS = 0x50;

        int protocol_type = 0; // 0: UART, 1: I2C Master, 2: I2C Slave
        int net_id = 1;

        // UART
        int baudrate = DEFAULT_BAUDRATE;
        int tx_pin = 0;
        int rx_pin = 0;
        int start_bits = 1;
        int data_bits = 8;
        int parity_bits = 0;
        int stop_bits = 1;
        char ip[64] = "127.0.0.1";
        int port = DEFAULT_PORT;

        // I2C
        int scl_pin = 0;
        int sda_pin = 0;
        int address = DEFAULT_ADDRESS;
        int i2c_id = 0;
    } m_ui_add_state;

    int m_ui_view_idx = -1;
    int m_ui_handshake_port{ AddConnectionState::DEFAULT_PORT };

    int ui_selected_local_pin_idx = 0;
    int ui_target_net_pin = 0;
    int ui_selected_net_pin_source = 0;
    int ui_target_local_pin_idx = 0;
#endif
};
