// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.hpp
/// \brief Defines a remote GPIO peripheral communicating via UDP with a stateful bit-parser.
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <map>
#include <vector>
#include <string>
#include <netinet/in.h> // Assuming Linux/Unix for socket headers based on context
#include <arpa/inet.h>

#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"

// ---------------------------------------------------------------------------------------------------------------------
/// \class CBit_Stream_Parser
/// \brief A stateful parser that consumes bits to form commands. Bit-agnostic implementation.
// ---------------------------------------------------------------------------------------------------------------------
class CBit_Stream_Parser
{
public:
    struct TCommand
    {
        std::uint32_t net_pin_idx;
        bool value;
    };

    enum class NState
    {
        Header,
        Pin_Index,
        Value,
        Execute
    };

    // Protocol Constants
    static constexpr std::uint8_t Header_Bits_Len = 2;
    static constexpr std::uint8_t Header_Pattern = 0b01; // 0x01
    static constexpr std::uint8_t Pin_Bits_Len = 5;
    static constexpr std::uint8_t Value_Bits_Len = 1;

public:
    CBit_Stream_Parser();

    /// \brief Pushes a byte into the parser (processed MSB to LSB or user pref).
    /// \return A vector of commands parsed from this specific byte (could be 0 or more).
    std::vector<TCommand> Parse_Byte(std::uint8_t byte);

    /// \brief Resets the parser state.
    void Reset();

private:
    void Process_Bit(bool bit, std::vector<TCommand>& commands);

private:
    NState m_state;
    std::uint32_t m_accumulator;
    std::uint32_t m_bits_read;
    TCommand m_current_command;
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
    void Start_Listening_Thread();
    void Stop_Listening_Thread();
    void Listening_Loop();
    void Send_UDP_Packet(std::uint8_t payload);

    // Protocol Helper
    std::uint8_t Construct_Packet(std::uint32_t net_pin, bool value);

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
    std::map<std::uint32_t, std::uint32_t> m_map_local_to_net;
    // Key: Net Pin ID, Value: Local GPIO Pin (Inbound)
    std::map<std::uint32_t, std::uint32_t> m_map_net_to_local;

    // Parsing & Synchronization
    CBit_Stream_Parser m_parser;
    std::mutex m_command_queue_mutex;
    std::queue<CBit_Stream_Parser::TCommand> m_command_queue;

    // UI Helpers
    int m_ui_selected_local_pin_idx;
    int m_ui_target_net_pin;
    int m_ui_selected_net_pin_source;
    int m_ui_target_local_pin_idx;
};
