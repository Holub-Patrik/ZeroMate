// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.hpp
/// \brief Defines a remote GPIO peripheral communicating via UDP
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <variant>
#include <vector>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <array>

#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "CircularBufferQueue.hpp"
#include "Parser.hpp"

constexpr int RECV_BUF_SIZE = 64;
constexpr std::size_t QUEUE_SIZE = 32;

constexpr std::uint8_t MAGIC_BYTE = 60; // 0x00111100
constexpr std::uint8_t WRITE_ONE = UINT8_MAX;
constexpr std::uint8_t WRITE_ZERO = 0;

// this out to be enough since there are only 52 GPIO pins
constexpr std::uint8_t GPIOMapSize = 64;

using ProtocolEnum = enum : std::uint8_t
{
    UART = 0,
    I2C = 1,
    SPI = 2,
    GeneralBuffered = 3,
    GeneralUnbuffered = 4,
};

class FastMap
{
private:
    std::array<std::uint8_t, GPIOMapSize> _map;

public:
    FastMap()
    : _map()
    {
        for (std::uint8_t& mapping : _map)
        {
            mapping = UINT8_MAX;
        }
    }

    void set(const std::uint8_t src, const std::uint8_t dst) noexcept
    {
        _map[src] = dst;
    }

    [[nodiscard]] std::uint8_t get(const std::uint8_t src) const noexcept
    {
        return _map[src];
    }

    [[nodiscard]] bool contains(const std::uint8_t pos) const noexcept
    {
        return _map[pos] != UINT8_MAX;
    }

    [[nodiscard]] const std::array<std::uint8_t, GPIOMapSize>& _get_arr() const noexcept
    {
        return _map;
    }
};

class Spinlock final
{
private:
    std::atomic<bool> lock_ = { false };

public:
    Spinlock() = default;
    ~Spinlock() = default;

    Spinlock(const Spinlock& other) = delete;
    Spinlock& operator=(const Spinlock& other) = delete;

    Spinlock(Spinlock&& other) = delete;
    Spinlock& operator=(Spinlock&& other) = delete;

    void lock() noexcept
    {
        for (;;)
        {
            if (!lock_.exchange(true, std::memory_order_acquire))
            {
                return;
            }
            while (lock_.load(std::memory_order_relaxed))
            {
                cpu_relax();
            }
        }
    }

    bool try_lock() noexcept
    {
        return !lock_.load(std::memory_order_relaxed) && !lock_.exchange(true, std::memory_order_acquire);
    }

    void unlock() noexcept
    {
        lock_.store(false, std::memory_order_release);
    }
};

using BusConnection = struct BusConnection_struct
{
    std::uint32_t bus_id;
    std::vector<struct sockaddr_in> devices;
};

using P2PConnection = struct P2PConnection_struct
{
    struct sockaddr_in other_side;
};

using Connection = std::variant<BusConnection, P2PConnection>;

using UART_P = struct UART_ProtocolInfo
{

    std::uint32_t baudrate{ UINT32_MAX };
    std::uint32_t tx_pin{ UINT32_MAX }; // transmit
    std::uint32_t rx_pin{ UINT32_MAX }; // receive (ignore the callback on this one)
    // Point-to-point connection
    struct sockaddr_in other_side{};
};

using I2C_P = struct I2C_ProtocolInfo
{
    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
    // Broadcast address and intent everywhere
    // START signal | 7 bit address | 1 bit intent (0 - Write | 1 - Read)
    std::vector<struct sockaddr_in> slaves;
};

using SPI_P = struct SPI_ProtocolInfo
{
    std::uint32_t sclk_pin{ UINT32_MAX };
    std::uint32_t mosi_pin{ UINT32_MAX }; // master out slave in
    std::uint32_t miso_pin{ UINT32_MAX }; // master out slave in
    // Chip select here works basically as an index into the array
    // Of course chip select signal has to sent into the slave first
    std::uint32_t chip_select{ UINT32_MAX };
    std::vector<struct sockaddr_in> slaves;
};

// General buffered and unbuffered protocols
// They are general for a reason and they might be inadequate
// But they can be useful when speeds are low
using GeneralBuffered_P = struct GeneralBuffered_ProtocolInfo
{
    int buf_length{ -1 };
    struct sockaddr_in other_side{};

    FastMap net_to_local;
    FastMap local_to_net;
};

using GeneralUnbuffered_P = struct GeneralUnbuffered_ProtocolInfo
{
    struct sockaddr_in other_side{};
    FastMap net_to_local;
    FastMap local_to_net;
};

using Protocol = std::variant<UART_P, I2C_P, SPI_P, GeneralBuffered_P, GeneralUnbuffered_P>;

using protocol_info = struct protocol_info
{
    ProtocolEnum p{ UART };
    Protocol info;
};

using conn_info = struct conn_info_struct
{
    bool explicit_clock{};
    std::int8_t clock_unit{};
    std::uint32_t clock_value{};

    in_port_t opened_port{};
    protocol_info protocol;
    std::uint32_t net_id{};
};

using conn_id = std::uint64_t;
using pin_pair = std::pair<std::uint8_t, std::uint8_t>;

class IProtocolStateMachine
{
public:
    virtual ~IProtocolStateMachine() = default;
    virtual void run() = 0;
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

private:
    // Pin write entirely here since there will be multiple writers, so spinlock is added to ensure safety
    TSP::Queue::Buffer<pin_pair, QUEUE_SIZE> pin_write_queue_buf{};
    TSP::Queue::Reader<pin_pair, QUEUE_SIZE> pin_write_queue_reader;
    TSP::Queue::Writer<pin_pair, QUEUE_SIZE> pin_write_queue_writer;
    Spinlock pin_write_spinlock;

    TSP::BF::Backoff backoff_fast{ BACKOFF_CYCLES };
    TSP::BF::SemBackoff backoff_sem{ BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED };

    // a bit map lookup might be best to assign new threads
    std::array<TSP::Queue::Buffer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_buffers;
    std::array<TSP::Queue::Writer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_writers;

    // this could be converted into a bit map, but bit instruction are extra instructions
    std::array<bool, MAX_CONNECTION_COUNT> connection_bit_map{};

    // abuse std::destroy_at{} and std::construct_at{} to use arrays
    std::array<std::thread, MAX_CONNECTION_COUNT> connection_threads;
    std::array<conn_info, MAX_CONNECTION_COUNT> connection_data;

    FastMap pin_to_conn_id;
    FastMap net_id_to_conn_id;

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin;
    zero_mate::IExternal_Peripheral::Halt_t func_halt;
    zero_mate::IExternal_Peripheral::Start_t func_start;

    std::atomic<bool> m_running{ true };
    std::thread m_pin_write_thread;

    void pin_write();

public:
    GPIOServer() = delete;

    explicit GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin,
                        zero_mate::IExternal_Peripheral::Halt_t func_halt,
                        zero_mate::IExternal_Peripheral::Start_t func_start)
    : pin_write_queue_reader(&pin_write_queue_buf)
    , pin_write_queue_writer(&pin_write_queue_buf)
    , func_set_pin(func_set_pin)
    , func_halt(func_halt)
    , func_start(func_start)
    {
        for (std::size_t i = 0; i < out_queue_buffers.size(); i++)
        {
            std::construct_at(&out_queue_writers[i], &out_queue_buffers[i]);
        }
    }
    ~GPIOServer();

    GPIOServer(const GPIOServer& other) = delete;
    GPIOServer& operator=(const GPIOServer& other) = delete;

    GPIOServer(GPIOServer&& other) = delete;
    GPIOServer& operator=(GPIOServer&& other) = delete;

    void write_to_pin(const std::uint8_t pin, const std::uint8_t value);
    void route_pin_info(const pin_pair pin_info);
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
};

class GPIOConnection final
{
private:
    conn_info connection;
    std::unique_ptr<IProtocolStateMachine> m_sm;

    TSP::Queue::Reader<pin_pair, GPIOServer::BUFFER_SIZE> m_queue_reader;

    int m_socket;
    struct sockaddr_in m_other_side;

    zero_mate::IExternal_Peripheral::Halt_t m_halt;
    zero_mate::IExternal_Peripheral::Start_t m_start;
    GPIOServer& m_server;

    std::atomic<bool>& m_server_running;

public:
    GPIOConnection() = delete;
    GPIOConnection(const conn_info& info,
                   TSP::Queue::Buffer<pin_pair, GPIOServer::BUFFER_SIZE>* buffer,
                   zero_mate::IExternal_Peripheral::Halt_t halt,
                   zero_mate::IExternal_Peripheral::Start_t start,
                   GPIOServer& server,
                   std::atomic<bool>& server_running);

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
        return m_server_running.load();
    }
};

class UARTStateMachine : public IProtocolStateMachine
{
private:
    GPIOConnection& m_conn;
    UART_P m_params;

public:
    UARTStateMachine(GPIOConnection& conn, const UART_P& params)
    : m_conn(conn)
    , m_params(params)
    {
    }

    void run() override
    {
        while (m_conn.is_running())
        {
            const auto change = m_conn.read_queue();
            // TODO: UART logic
            (void)change;
        }
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

    void Render() final;
    void Set_ImGui_Context(void* context) final;
    void Increment_Passed_Cycles(std::uint32_t count) final;
    void GPIO_Subscription_Callback(std::uint32_t pin_idx) final;

private:
    // UI Rendering
    void Render_Settings();
    void Render_Mappings();

    // IExternal dependencies
    std::string name;
    std::vector<std::uint32_t> pins; // Available local pins
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin;
    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin;
    zero_mate::IExternal_Peripheral::Halt_t halt;
    zero_mate::IExternal_Peripheral::Start_t start;
    zero_mate::utils::CLogging_System* logging_system;
    ImGuiContext* ImGui_context;

    std::atomic<bool> m_running{ true };
    GPIOServer server;
    std::thread server_thread;

    // UI Helpers
    int ui_selected_local_pin_idx;
    int ui_target_net_pin;
    int ui_selected_net_pin_source;
    int ui_target_local_pin_idx;
};
