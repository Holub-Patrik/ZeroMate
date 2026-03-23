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
};

namespace handshake
{
    static constexpr std::uint8_t MAGIC_BYTE = 0x5A;

    enum class ProtocolID : std::uint8_t
    {
        UART = 0,
        I2C_Master = 1,
        I2C_Slave = 2,
    };

    struct ConfMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        std::uint16_t opened_port;
        ProtocolID protocol_id;
        std::uint32_t protocol_info; // Baudrate for UART, ID for I2C
        std::uint8_t explicit_clock;
        std::uint8_t clock_unit;
        std::uint32_t clock_value;
    } __attribute__((packed));

    struct AcceptDeclineMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        std::uint8_t accept;
        std::uint16_t port;
    } __attribute__((packed));

    struct AcceptAckMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        std::uint8_t ack;
    } __attribute__((packed));
}

using conn_id = std::uint64_t;
using pin_pair = std::pair<std::uint8_t, std::uint8_t>;

template<std::size_t QUEUE_SIZE>
using protocol_variant = std::variant<std::unique_ptr<UART_Handler<QUEUE_SIZE>>,
                                      std::unique_ptr<I2C_Master<QUEUE_SIZE>>,
                                      std::unique_ptr<I2C_Slave<QUEUE_SIZE>>>;

template<typename Type, std::size_t Size>
class BitProcessor final
{
public:
    using pin_write_t = std::function<void(std::uint8_t, std::uint8_t)>;
    using pin_read_t = std::function<std::uint8_t(std::uint8_t)>;

private:
    protocol_variant<Size> handler;
    TSP::Queue::Reader<Type, Size> queue_reader;
    TSP::BF::SemBackoff backoff;

    std::atomic<bool> running{ false };
    std::thread sender;

public:
    BitProcessor(protocol_variant<Size> variant, TSP::Queue::Buffer<Type, Size>* buf)
    : handler(std::move(variant))
    , queue_reader(buf)
    , backoff(100, 1000)
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
        running = true;
        sender = std::thread{ &BitProcessor<Type, Size>::run_sender, this };
        std::visit([](auto& h) -> void { h->start_receiver(); }, handler);
    }

    void stop()
    {
        if (running.exchange(false))
        {
            backoff.wake();
            if (sender.joinable())
            {
                sender.join();
            }

            std::visit([](auto& h) -> void { h->receiver_stop(); }, handler);
        }
    }

private:
    void run_sender()
    {
        auto last_time = std::chrono::high_resolution_clock::now();

        while (running)
        {
            if (!queue_reader.try_advance())
            {
                backoff.wait([this]() { return !running || queue_reader.try_advance(); });
                continue;
            }
            backoff.reset();

            const auto pair = queue_reader.peek();
            queue_reader.advance();

            const auto now = std::chrono::high_resolution_clock::now();
            const auto delta =
            static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_time).count());
            last_time = now;
            std::visit([pair, delta](auto& h) -> void { h->process_bit(pair, delta); }, handler);
        }
    }
};

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

private:
    // Pin write entirely here since there will be multiple writers, so spinlock is added to ensure safety
    TSP::Queue::Buffer<pin_pair, QUEUE_SIZE> pin_write_queue_buf{ };
    TSP::Queue::Reader<pin_pair, QUEUE_SIZE> pin_write_queue_reader;
    TSP::Queue::Writer<pin_pair, QUEUE_SIZE> pin_write_queue_writer;
    Spinlock pin_write_spinlock;

    TSP::BF::Backoff backoff_fast{ BACKOFF_CYCLES };
    TSP::BF::SemBackoff backoff_sem{ BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED };

    // a bit map lookup might be best to assign new threads
    std::array<TSP::Queue::Buffer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_buffers;
    std::array<TSP::Queue::Writer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_writers;

    // this could be converted into a bit map, but bit instruction are extra instructions
    std::array<bool, MAX_CONNECTION_COUNT> connection_bit_map{ };

    // abuse std::destroy_at{} and std::construct_at{} to use arrays
    std::array<std::thread, MAX_CONNECTION_COUNT> connection_threads;
    std::array<conn_info, MAX_CONNECTION_COUNT> connection_data;
    std::array<std::unique_ptr<std::atomic<bool>>, MAX_CONNECTION_COUNT> connection_running;

    FastMap pin_to_conn_id;
    FastMap net_id_to_conn_id;

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin;
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t func_read_pin;
    zero_mate::IExternal_Peripheral::Halt_t func_halt;
    zero_mate::IExternal_Peripheral::Start_t func_start;

    std::atomic<bool> m_running{ true };
    std::thread m_pin_write_thread;
    int m_handshake_socket{ -1 };
    uint16_t m_handshake_port{ 12344 };

    void pin_write();
    void unmap_connection(std::size_t i);

public:
    GPIOServer() = delete;

    explicit GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin,
                        zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t func_read_pin,
                        zero_mate::IExternal_Peripheral::Halt_t func_halt,
                        zero_mate::IExternal_Peripheral::Start_t func_start);
    ~GPIOServer();

    GPIOServer(const GPIOServer& other) = delete;
    GPIOServer& operator=(const GPIOServer& other) = delete;

    GPIOServer(GPIOServer&& other) = delete;
    GPIOServer& operator=(GPIOServer&& other) = delete;

    void write_to_pin(const std::uint8_t pin, const std::uint8_t value);
    void route_pin_info(const pin_pair pin_info);
    std::size_t create_connection(const conn_info& info);
    void connect_connection(std::size_t i);
    void remove_connection(std::size_t i);
    void construct_connection(const conn_info& info);
    void run();
    void stop();

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

    // UI access
    [[nodiscard]] const auto& get_connection_bit_map() const
    {
        return connection_bit_map;
    }
    [[nodiscard]] const auto& get_connection_data() const
    {
        return connection_data;
    }
};

class GPIOConnection final
{
private:
    conn_info& connection;

    TSP::Queue::Reader<pin_pair, GPIOServer::BUFFER_SIZE> m_queue_reader;

    int m_socket;
    struct sockaddr_in m_other_side;

    zero_mate::IExternal_Peripheral::Halt_t m_halt;
    zero_mate::IExternal_Peripheral::Start_t m_start;
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
    GPIOServer& m_server;

    std::atomic<bool>& m_server_running;
    std::atomic<bool>& m_connection_running;
    std::unique_ptr<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE>> m_processor;

public:
    GPIOConnection() = delete;
    GPIOConnection(conn_info& info,
                   TSP::Queue::Buffer<pin_pair, GPIOServer::BUFFER_SIZE>* buffer,
                   zero_mate::IExternal_Peripheral::Halt_t halt,
                   zero_mate::IExternal_Peripheral::Start_t start,
                   zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                   GPIOServer& server,
                   std::atomic<bool>& server_running,
                   std::atomic<bool>& connection_running);

    ~GPIOConnection();

    GPIOConnection(const GPIOConnection& other) = delete;
    void run();

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
    GPIOServer server;
    std::thread server_thread;

    // UI State
#ifndef ZERO_MATE_UNIT_TESTS
    struct AddConnectionState
    {
        int protocol_type = 0; // 0: UART, 1: I2C Master, 2: I2C Slave

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

    int ui_selected_local_pin_idx{ 0 };
    int ui_target_net_pin{ 0 };
    int ui_selected_net_pin_source{ 0 };
    int ui_target_local_pin_idx{ 0 };
#endif
};
