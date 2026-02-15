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

constexpr std::uint8_t QuickMapSize = 64;

class GPIOMap
{
private:
    std::array<std::uint8_t, QuickMapSize> m_map;

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

    [[nodiscard]] const std::array<std::uint8_t, QuickMapSize>& _get_arr() const noexcept
    {
        return m_map;
    }
};

class GPIOLastState
{
private:
    std::array<std::uint8_t, QuickMapSize> m_map;

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
