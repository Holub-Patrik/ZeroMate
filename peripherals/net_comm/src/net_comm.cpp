// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.cpp
/// \author Assistant
/// \brief Implementation of the Remote GPIO peripheral.
// ---------------------------------------------------------------------------------------------------------------------

#include "net_comm.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <cstring>

// =====================================================================================================================
// CBit_Stream_Parser Implementation
// =====================================================================================================================

CBit_Stream_Parser::CBit_Stream_Parser()
{
    Reset();
}

void CBit_Stream_Parser::Reset()
{
    m_state = NState::Header;
    m_accumulator = 0;
    m_bits_read = 0;
    m_current_command = { 0, false };
}

std::vector<CBit_Stream_Parser::TCommand> CBit_Stream_Parser::Parse_Byte(std::uint8_t byte)
{
    std::vector<TCommand> completed_commands;

    // Process byte from MSB (Bit 7) to LSB (Bit 0)
    // Example: 01000011 -> Process 0, then 1, then 0...
    for (int i = 7; i >= 0; --i)
    {
        bool bit = (byte >> i) & 1;
        Process_Bit(bit, completed_commands);
    }

    return completed_commands;
}

void CBit_Stream_Parser::Process_Bit(bool bit, std::vector<TCommand>& commands)
{
    // Shift bit into accumulator
    m_accumulator = (m_accumulator << 1) | bit;
    m_bits_read++;

    switch (m_state)
    {
        case NState::Header:
            // Keep only the last N bits in the accumulator
            m_accumulator &= ((1 << Header_Bits_Len) - 1);

            // Check if accumulator matches header pattern
            // Note: We don't check bits_read here strictly because we want a sliding window
            // to catch the header anywhere in the stream.
            if (m_accumulator == Header_Pattern)
            {
                m_state = NState::Pin_Index;
                m_accumulator = 0;
                m_bits_read = 0;
            }
            break;

        case NState::Pin_Index:
            if (m_bits_read == Pin_Bits_Len)
            {
                m_current_command.net_pin_idx = m_accumulator;
                m_state = NState::Value;
                m_accumulator = 0;
                m_bits_read = 0;
            }
            break;

        case NState::Value:
            if (m_bits_read == Value_Bits_Len)
            {
                m_current_command.value = (m_accumulator > 0);

                // Command is complete
                commands.push_back(m_current_command);

                // Reset for next command
                m_state = NState::Header;
                m_accumulator = 0;
                m_bits_read = 0;
            }
            break;

        case NState::Execute:
            // Should not be reachable in this logic flow
            break;
    }
}

// =====================================================================================================================
// CRemote_GPIO Implementation
// =====================================================================================================================

CRemote_GPIO::CRemote_GPIO(const std::string& name,
                           const std::vector<std::uint32_t>& pins,
                           zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                           zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                           zero_mate::utils::CLogging_System* logging_system)
: m_name(name)
, m_pins(pins)
, m_read_pin(read_pin)
, m_set_pin(set_pin)
, m_logging_system(logging_system)
, m_ImGui_context(nullptr)
, m_sockfd(-1)
, m_running(false)
, m_remote_port(8081)
, m_local_port(8080)
, m_connected(false)
, m_ui_selected_local_pin_idx(0)
, m_ui_target_net_pin(0)
, m_ui_selected_net_pin_source(0)
, m_ui_target_local_pin_idx(0)
{
    std::strncpy(m_remote_ip_buffer, "127.0.0.1", sizeof(m_remote_ip_buffer));

    // Subscribe to all pins passed in construction
    for (const auto& pin : m_pins)
    {
        m_gpio_subscription.insert(pin);
    }
}

CRemote_GPIO::~CRemote_GPIO()
{
    Stop_Listening_Thread();
    if (m_sockfd >= 0)
    {
        close(m_sockfd);
    }
}

void CRemote_GPIO::Set_ImGui_Context(void* context)
{
    m_ImGui_context = static_cast<ImGuiContext*>(context);
}

void CRemote_GPIO::Init_Socket()
{
    if (m_sockfd >= 0)
    {
        close(m_sockfd);
    }

    m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockfd < 0)
    {
        m_logging_system->Error("Failed to create UDP socket");
        return;
    }

    // Set non-blocking might be useful, but using a thread for recvfrom is cleaner
    // Bind Local
    std::memset(&m_local_addr, 0, sizeof(m_local_addr));
    m_local_addr.sin_family = AF_INET;
    m_local_addr.sin_addr.s_addr = INADDR_ANY;
    m_local_addr.sin_port = htons(m_local_port);

    if (bind(m_sockfd, (const struct sockaddr*)&m_local_addr, sizeof(m_local_addr)) < 0)
    {
        m_logging_system->Error("Failed to bind UDP socket");
        return;
    }

    // Setup Remote (Target)
    std::memset(&m_remote_addr, 0, sizeof(m_remote_addr));
    m_remote_addr.sin_family = AF_INET;
    m_remote_addr.sin_port = htons(m_remote_port);
    inet_pton(AF_INET, m_remote_ip_buffer, &m_remote_addr.sin_addr);

    m_connected = true;
    Init_Listening_Thread();
    const auto msg = std::string{ "UDP Socket Initialized on fd: " } + std::to_string(m_sockfd);
    m_logging_system->Info(msg.c_str());
}

void CRemote_GPIO::Init_Listening_Thread()
{
    m_running = true;
    m_listener_thread = std::thread(&CRemote_GPIO::Listening_Loop, this);
}

void CRemote_GPIO::Start_Listening_Thread()
{
    Stop_Listening_Thread(); // Ensure previous is stopped
    m_running = true;
    m_listener_thread = std::thread(&CRemote_GPIO::Listening_Loop, this);
}

void CRemote_GPIO::Stop_Listening_Thread()
{
    m_running = false;
    // We might need to send a dummy packet to self to unblock recvfrom,
    // or close socket. Closing socket is usually enough to wake recvfrom with error.
    if (m_sockfd >= 0)
    {
        shutdown(m_sockfd, SHUT_RDWR);
        close(m_sockfd);
        m_sockfd = -1;
    }

    if (m_listener_thread.joinable())
    {
        m_listener_thread.join();
    }
    m_connected = false;
}

void CRemote_GPIO::Listening_Loop()
{
    std::uint8_t buffer[64]; // Small buffer is fine for byte payloads
    struct sockaddr_in cliaddr;
    socklen_t len = sizeof(cliaddr);

    while (m_running)
    {
        if (m_sockfd < 0)
            break;

        ssize_t n = recvfrom(m_sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);

        // Process all received bytes
        for (ssize_t i = 0; i < n; ++i)
        {
            auto cmds = m_parser.Parse_Byte(buffer[i]);

            if (!cmds.empty())
            {
                std::lock_guard<std::mutex> lock(m_command_queue_mutex);
                for (const auto& cmd : cmds)
                {
                    m_command_queue.push(cmd);
                }
            }
        }
    }
}

void CRemote_GPIO::Send_UDP_Packet(const std::uint8_t payload)
{
    if (!m_connected || m_sockfd < 0)
    {
        return;
    }

    sendto(m_sockfd, &payload, sizeof(payload), 0, (const struct sockaddr*)&m_remote_addr, sizeof(m_remote_addr));
}

std::uint8_t CRemote_GPIO::Construct_Packet(const std::uint8_t net_pin, const bool value)
{
    // Format: 2 bits Header (01), 5 bits Pin, 1 bit Value
    std::uint8_t packet = 0;

    packet |= (CBit_Stream_Parser::Header_Pattern << 6); // Shift header to MSB
    packet |= ((net_pin & 0x1F) << 1);                   // Mask pin to 5 bits, shift
    packet |= (value ? 1 : 0);                           // LSB

    return packet;
}

void CRemote_GPIO::Increment_Passed_Cycles(std::uint32_t count)
{
    // Process the queue of incoming commands on the main thread
    // This is crucial for thread safety of m_set_pin

    std::lock_guard<std::mutex> lock(m_command_queue_mutex);
    while (!m_command_queue.empty())
    {
        auto cmd = m_command_queue.front();
        m_command_queue.pop();

        // Check if we have a mapping for this Net Pin to a Local GPIO
        if (m_map_net_to_local.contains(cmd.net_pin_idx))
        {
            std::uint8_t local_pin = m_map_net_to_local.get(cmd.net_pin_idx);
            m_set_pin(local_pin, cmd.value);

            // Log for debug
            std::string msg = "NetPin " + std::to_string(cmd.net_pin_idx) + " -> Local " + std::to_string(local_pin) +
                              " Val: " + std::to_string(static_cast<int>(cmd.value));
            m_logging_system->Debug(msg.c_str());
        }
        else
        {
            m_logging_system->Debug("No mapping found Increment_Passed_Cycles Net -> Local");
        }
    }
}

void CRemote_GPIO::GPIO_Subscription_Callback(std::uint32_t pin_idx)
{
    const std::uint8_t pin_idx_u8 = static_cast<std::uint8_t>(pin_idx);
    // Check if this local pin is mapped to a network pin (Outbound)
    if (m_map_local_to_net.contains(pin_idx_u8))
    {
        const std::uint8_t net_pin = m_map_local_to_net.get(pin_idx_u8);
        const bool state = m_read_pin(pin_idx);

        std::uint8_t payload = Construct_Packet(net_pin, state);
        Send_UDP_Packet(payload);
    }
    else
    {
        m_logging_system->Debug("No mapping found GPIO_Subscription_Callback Local -> Net");
    }
}

void CRemote_GPIO::Render()
{
    assert(m_ImGui_context != nullptr);
    ImGui::SetCurrentContext(m_ImGui_context);

    if (ImGui::Begin(m_name.c_str()))
    {
        Render_Settings();
        ImGui::Separator();
        Render_Mappings();
    }
    ImGui::End();
}

void CRemote_GPIO::Render_Settings()
{
    ImGui::Text("Network Configuration");

    ImGui::InputText("Remote IP", m_remote_ip_buffer, sizeof(m_remote_ip_buffer));
    ImGui::InputInt("Remote Port (TX)", &m_remote_port);
    ImGui::InputInt("Local Port (RX)", &m_local_port);

    if (!m_connected)
    {
        if (ImGui::Button("Connect / Bind"))
        {
            Init_Socket();
        }
    }
    else
    {
        if (ImGui::Button("Disconnect / Stop"))
        {
            Stop_Listening_Thread();
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Active");
    }
}

void CRemote_GPIO::Render_Mappings()
{
    ImGui::Text("Outbound Mapping (Local GPIO -> Net Pin)");

    // Dropdown for Local Pins (Source)
    if (ImGui::BeginCombo("##LocalSource", ("GPIO " + std::to_string(m_pins[m_ui_selected_local_pin_idx])).c_str()))
    {
        for (int i = 0; i < m_pins.size(); ++i)
        {
            bool is_selected = (m_ui_selected_local_pin_idx == i);
            if (ImGui::Selectable(("GPIO " + std::to_string(m_pins[i])).c_str(), is_selected))
            {
                m_ui_selected_local_pin_idx = i;
            }
            if (is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();
    ImGui::Text("->");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##NetTarget", &m_ui_target_net_pin);

    ImGui::SameLine();
    if (ImGui::Button("Add##Out"))
    {
        m_map_local_to_net.set(m_pins[m_ui_selected_local_pin_idx], m_ui_target_net_pin);
    }

    // List Outbound Mappings
    if (ImGui::BeginTable("OutboundMaps", 2, ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Local GPIO");
        ImGui::TableSetupColumn("Net Pin ID");
        ImGui::TableHeadersRow();

        std::size_t index = 0;
        for (const std::uint8_t mapping : m_map_local_to_net._get_arr())
        {
            if (mapping != UINT8_MAX)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%zu", index);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d", mapping);
                ImGui::SameLine();
                if (ImGui::SmallButton(("X##Out" + std::to_string(index)).c_str()))
                {
                    m_map_local_to_net.set(index, UINT8_MAX);
                }
            }
            index++;
        }
        ImGui::EndTable();
    }

    ImGui::Separator();

    ImGui::Text("Inbound Mapping (Net Pin -> Local GPIO)");

    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##NetSource", &m_ui_selected_net_pin_source);
    ImGui::SameLine();
    ImGui::Text("->");
    ImGui::SameLine();

    // Dropdown for Local Pins (Target)
    if (ImGui::BeginCombo("##LocalTarget", ("GPIO " + std::to_string(m_pins[m_ui_target_local_pin_idx])).c_str()))
    {
        for (int i = 0; i < m_pins.size(); ++i)
        {
            bool is_selected = (m_ui_target_local_pin_idx == i);
            if (ImGui::Selectable(("GPIO " + std::to_string(m_pins[i])).c_str(), is_selected))
            {
                m_ui_target_local_pin_idx = i;
            }
            if (is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();
    if (ImGui::Button("Add##In"))
    {
        m_map_net_to_local.set(m_ui_selected_net_pin_source, m_pins[m_ui_selected_net_pin_source]);
    }

    // List Inbound Mappings
    if (ImGui::BeginTable("InboundMaps", 2, ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Net Pin ID");
        ImGui::TableSetupColumn("Local GPIO");
        ImGui::TableHeadersRow();

        std::size_t index = 0;
        for (const std::uint8_t mapping : m_map_net_to_local._get_arr())
        {
            if (mapping != UINT8_MAX)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%zu", index);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d", mapping);
                ImGui::SameLine();
                if (ImGui::SmallButton(("X##Out" + std::to_string(index)).c_str()))
                {
                    m_map_net_to_local.set(index, UINT8_MAX);
                }
            }
            index++;
        }
        ImGui::EndTable();
    }
}

// Factory function
extern "C"
{
    zero_mate::IExternal_Peripheral::NInit_Status
    Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
                      const char* const name,
                      const std::uint32_t* const connection,
                      std::size_t pin_count,
                      zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                      zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                      zero_mate::utils::CLogging_System* logging_system)
    {
        std::vector<std::uint32_t> pins;
        for (std::size_t i = 0; i < pin_count; ++i)
        {
            pins.push_back(connection[i]);
        }

        *peripheral = new (std::nothrow) CRemote_GPIO(name, pins, read_pin, set_pin, logging_system);

        if (*peripheral == nullptr)
        {
            return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
        }

        return zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
