// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.hpp
/// \brief Defines a remote GPIO peripheral communicating via UDP
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <array>

#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"

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

class GPIOMap
{
private:
    std::array<std::uint8_t, GPIOMapSize> m_map;

public:
    GPIOMap()
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

class GPIOLastState
{
private:
    std::array<std::uint8_t, GPIOMapSize> m_map;

public:
    GPIOLastState()
    : m_map()
    {
        for (std::uint8_t& mapping : m_map)
        {
            mapping = 0;
        }
    }

    void set(const std::uint8_t pin, const std::uint8_t value) noexcept
    {
        m_map[pin] = value;
    }

    [[nodiscard]] std::uint8_t get(const std::uint8_t pin) noexcept
    {
        return m_map[pin];
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

// ---------------------------------------------------------------------------------------------------------------------
/// \class CRemote_GPIO
/// \brief External peripheral for UDP-based remote GPIO control.
// ---------------------------------------------------------------------------------------------------------------------
class CRemote_GPIO final : public zero_mate::IExternal_Peripheral
{
public:
    explicit CRemote_GPIO(const std::string& name,
                          const std::vector<std::uint32_t>& pins,
                          zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                          zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                          zero_mate::utils::CLogging_System* logging_system);

    ~CRemote_GPIO() override;

    void Render() override;
    void Set_ImGui_Context(void* context) override;
    void Increment_Passed_Cycles(std::uint32_t count) override;
    void GPIO_Subscription_Callback(std::uint32_t pin_idx) override;

private:
    // Network Management
    void Init_Socket();
    void Init_Listening_Thread();
    void Start_Listening_Thread();
    void Stop_Listening_Thread();
    void Listening_Loop();
    void Send_UDP_Packet(const std::uint8_t value, const std::uint8_t source_pin);

    // UI Rendering
    void Render_Settings();
    void Render_Mappings();

private:
    std::string m_name;
    std::vector<std::uint32_t> m_pins; // Available local pins
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
    zero_mate::utils::CLogging_System* m_logging_system;
    ImGuiContext* m_ImGui_context;

    // Networking
    int m_sockfd;
    struct sockaddr_in m_remote_addr;
    struct sockaddr_in m_local_addr;
    std::atomic<bool> m_running;
    std::thread m_listener_thread;

    // Configuration
    char m_remote_ip_buffer[16];
    int m_remote_port;
    int m_local_port;
    bool m_connected;

    // Mappings
    // Key: Local GPIO Pin, Value: Net Pin ID (Outbound)
    GPIOMap m_map_local_to_net;
    // Key: Net Pin ID, Value: Local GPIO Pin (Inbound)
    GPIOMap m_map_net_to_local;

    // Last state of pin to reduce send/recv calls
    GPIOLastState m_state_map;

    // UI Helpers
    int m_ui_selected_local_pin_idx;
    int m_ui_target_net_pin;
    int m_ui_selected_net_pin_source;
    int m_ui_target_local_pin_idx;
};
