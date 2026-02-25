// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.hpp
/// \brief Defines a remote GPIO peripheral communicating via UDP
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <map>
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

inline void cpu_relax()
{
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #ifdef _MSC_VER
        #include <intrin.h>
    _mm_pause();
    #else
    __builtin_ia32_pause();
    #endif
#elif defined(__aarch64__) || defined(__arm__)
    asm volatile("yield");
#else
    // Fallback compiler barrier to force memory reload
    asm volatile("" ::: "memory");
#endif
}

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
    std::array<std::uint8_t, GPIOMapSize> m_map;

public:
    FastMap()
    : m_map()
    {
        for (std::uint8_t& mapping : m_map)
        {
            mapping = UINT8_MAX;
        }
    }

    void set(const std::uint8_t src, const std::uint8_t dst) noexcept
    {
        m_map[src] = dst;
    }

    [[nodiscard]] std::uint8_t get(const std::uint8_t src) const noexcept
    {
        return m_map[src];
    }

    [[nodiscard]] bool contains(const std::uint8_t pos) const noexcept
    {
        return m_map[pos] != UINT8_MAX;
    }

    [[nodiscard]] const std::array<std::uint8_t, GPIOMapSize>& _get_arr() const noexcept
    {
        return m_map;
    }
};

class Backoff final
{
private:
    const std::uint64_t max_cycles;
    std::uint64_t cycles{ 0 };

public:
    Backoff() = delete;
    ~Backoff() = default;

    explicit Backoff(const std::uint64_t max_cycles)
    : max_cycles(max_cycles)
    {
    }

    Backoff(const Backoff& other) = delete;
    Backoff& operator=(const Backoff& other) = delete;

    Backoff(Backoff&& other) = delete;
    Backoff& operator=(Backoff&& other) = delete;

    void wait() noexcept
    {
        if (cycles < max_cycles)
        {
            cycles++;
            return;
        }
        cpu_relax();
    }

    void reset() noexcept
    {
        cycles = 0;
    }
};

template<typename TimeUnit>
class SleepBackoff final
{
private:
    const std::uint64_t max_cycles_fast;
    const std::uint64_t max_cycles_relaxed;

    const TimeUnit wait_time{ std::chrono::microseconds{ 100 } };

    std::uint64_t cycles_fast{ 0 };
    std::uint64_t cycles_relaxed{ 0 };

public:
    SleepBackoff<TimeUnit>() = delete;
    ~SleepBackoff<TimeUnit>() = default;

    explicit SleepBackoff<TimeUnit>(const std::uint64_t max_cycles_fast,
                                    const std::uint64_t max_cycles_relaxed,
                                    const TimeUnit& wait_time)
    : max_cycles_fast(max_cycles_fast)
    , max_cycles_relaxed(max_cycles_relaxed)
    , wait_time(wait_time)
    {
    }

    SleepBackoff<TimeUnit>(const SleepBackoff<TimeUnit>& other) = delete;
    SleepBackoff<TimeUnit>& operator=(const SleepBackoff<TimeUnit>& other) = delete;

    SleepBackoff<TimeUnit>(SleepBackoff<TimeUnit>&& other) = delete;
    SleepBackoff<TimeUnit>& operator=(SleepBackoff<TimeUnit>&& other) = delete;

    void wait() noexcept
    {
        if (cycles_fast < max_cycles_fast)
        {
            cycles_fast++;
            return;
        }

        if (cycles_relaxed < max_cycles_relaxed)
        {
            cycles_relaxed++;
            cpu_relax();
            return;
        }

        std::this_thread::sleep_for(wait_time);
    }

    void reset() noexcept
    {
        cycles_fast = 0;
        cycles_relaxed = 0;
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

using conn_info = struct
{
    bool explicit_clock;
    std::int8_t clock_unit;
    std::uint32_t clock_value;

    in_port_t opened_port;
    protocol_info protocol;

    std::string name;
};

using conn_id = std::uint64_t;
using pin_pair = std::pair<std::uint8_t, std::uint8_t>;

class GPIOServer final
{
private:
    static constexpr std::uint64_t BACKOFF_CYCLES = 1024;
    static constexpr auto NET_WAIT_TIME = std::chrono::microseconds{ 100 };
    static constexpr std::size_t MAX_CONNECTION_COUNT = 16;
    static constexpr std::size_t BUFFER_COUNT = MAX_CONNECTION_COUNT;
    static constexpr std::size_t BUFFER_SIZE = 512;

    std::map<conn_id, conn_info> connection_map;
    // convert this one into a directo lookup later
    // This might a good usecase for the 3 array datastructure
    // - The 3 array solution might be good since connections opening/closing isn't guaranteed to be in order
    // This map is needed due to callback nature of handling the pin callback
    std::map<std::uint8_t, conn_id> pin_to_connection;
    std::map<conn_id, CB::Writer<std::uint8_t, QUEUE_SIZE>> connection_to_queue_map;

    CB::Buffer<pin_pair, QUEUE_SIZE> pin_write_queue_buf{};
    CB::Reader<pin_pair, QUEUE_SIZE> pin_write_queue_reader;
    CB::Writer<pin_pair, QUEUE_SIZE> pin_write_queue_writer;

    Spinlock pin_write_spinlock;
    Backoff backoff_fast{ BACKOFF_CYCLES };
    SleepBackoff<std::chrono::microseconds> backoff_net{ BACKOFF_CYCLES,
                                                         (BACKOFF_CYCLES * BACKOFF_CYCLES),
                                                         NET_WAIT_TIME };

    // a bit map lookup might be best to assign new threads
    std::array<CB::Buffer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_buffers;
    std::array<CB::Writer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_writers;

    // this could be converted into a bit map, but bit instruction are extra instructions
    std::array<bool, MAX_CONNECTION_COUNT> connection_bit_map{};
    // vector used even though it will be always MAX_CONNECTION_COUNT size
    std::vector<std::thread> connection_threads;
    std::vector<std::thread> connection_data;

    // Every connection when registered, will set entries in this map so the routing routes
    FastMap id_to_conn;

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin;

    void pin_write();

public:
    GPIOServer() = delete;

    explicit GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin)
    : pin_write_queue_reader(pin_write_queue_buf)
    , pin_write_queue_writer(pin_write_queue_buf)
    , func_set_pin(func_set_pin)
    {
        for (int i = 0; i < out_queue_buffers.size(); i++)
        {
            out_queue_writers[i] = std::move(CB::Writer<pin_pair, BUFFER_SIZE>{ out_queue_buffers[i] });
        }
    }
    ~GPIOServer() = default;

    GPIOServer(const GPIOServer& other) = delete;
    GPIOServer& operator=(const GPIOServer& other) = delete;

    GPIOServer(GPIOServer&& other) = delete;
    GPIOServer& operator=(GPIOServer&& other) = delete;

    void write_to_pin(const std::uint8_t pin, const std::uint8_t value);
    void route_pin_info(const pin_pair pin_info);
    void run();
};

class GPIOConnection final
{
private:
    conn_info connection;
    std::unique_ptr<Parser> parser;

    CB::Reader<std::uint8_t, QUEUE_SIZE> bit_queue;

public:
    GPIOConnection() = delete;
    void run();
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
    zero_mate::utils::CLogging_System* logging_system;
    ImGuiContext* ImGui_context;

    GPIOServer server{ set_pin };
    std::thread server_thread;

    // UI Helpers
    int ui_selected_local_pin_idx;
    int ui_target_net_pin;
    int ui_selected_net_pin_source;
    int ui_target_local_pin_idx;
};
