#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <atomic>
#include <cstring>

#include "fmt/format.h"
#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/RemoteProtocol.hpp"
#include "zero_mate/Protocol.hpp"
#include "CircularBufferQueue.hpp"

namespace zero_mate::peripheral
{
    class CRemote_UART final : public IExternal_Peripheral
    {
    public:
        static constexpr size_t QUEUE_SIZE = 1024;

        CRemote_UART(const std::string& name,
                     IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                     IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                     utils::CLogging_System* logging_system)
        : m_name{ name }
        , m_read_pin{ read_pin }
        , m_set_pin{ set_pin }
        , m_logging_system{ logging_system }
        , m_rx_reader(&m_rx_buffer_buf)
        , m_rx_writer(&m_rx_buffer_buf)
        , m_rx_backoff(10, 100)
        {
            void* proc = LIB_SELF();
            m_server_register = (remote_protocol::register_t)LIB_SYM(proc, "server_register_channel");
            m_server_unregister = (remote_protocol::unregister_t)LIB_SYM(proc, "server_unregister_channel");
            m_server_send = (remote_protocol::send_t)LIB_SYM(proc, "server_send_data");
            m_server_init_handshake = (remote_protocol::init_handshake_t)LIB_SYM(proc, "server_init_handshake");

            if (m_server_register)
            {
                if (m_logging_system)
                    m_logging_system->Debug("Remote UART: Registering channel");
                m_server_id = m_server_register("uart",
                                                On_Compare_Static,
                                                On_Receive_Static,
                                                On_Disconnect_Static,
                                                On_Handshake_Result_Static,
                                                this);
                if (m_logging_system)
                    m_logging_system->Debug(fmt::format("Remote UART: Registered with ID {}", m_server_id).c_str());
            }
            else
            {
                if (m_logging_system)
                    m_logging_system->Error("Remote UART: Failed to find server_register_channel symbol");
            }

            std::strncpy(m_remote_ip, "127.0.0.1", sizeof(m_remote_ip) - 1);
        }

        ~CRemote_UART() override
        {
            if (m_server_id != 0 && m_server_unregister)
                m_server_unregister(m_server_id);
        }

        void Increment_Passed_Cycles(uint32_t count) override
        {
            if (!m_connected)
                return;
            m_cpu_cycles += count;
            if (m_cpu_cycles >= m_baud_rate_ticks)
            {
                m_cpu_cycles = 0;
                BufferBit();
                DrainBit();
            }
        }

        void Render() override
        {
            if (m_imgui_context)
                ImGui::SetCurrentContext(static_cast<ImGuiContext*>(m_imgui_context));

            if (ImGui::Begin(m_name.c_str()))
            {
                if (!m_connected)
                {
                    ImGui::InputText("Remote IP", m_remote_ip, sizeof(m_remote_ip));
                    ImGui::InputInt("Remote Port", &m_remote_port);
                    ImGui::InputInt("Net ID", &m_net_id);
                    ImGui::InputInt("TX Pin", &m_tx_pin);
                    ImGui::InputInt("RX Pin", &m_rx_pin);
                    ImGui::InputInt("Baudrate", &m_baudrate_val);

                    if (ImGui::Button("Connect"))
                    {
                        if (m_logging_system)
                            m_logging_system->Debug(fmt::format("Remote UART: Connecting to {}:{}", m_remote_ip, m_remote_port).c_str());
                        m_baud_rate_ticks = (250000000 / m_baudrate_val);
                        if (m_server_id != 0 && m_server_init_handshake)
                        {
                            struct UARTPayload
                            {
                                uint8_t proto;
                                uint32_t baud;
                                uint32_t net_id;
                            } __attribute__((packed));
                            UARTPayload payload = { 0, (uint32_t)m_baudrate_val, (uint32_t)m_net_id };
                            m_server_init_handshake(m_server_id,
                                                    m_remote_ip,
                                                    (uint16_t)m_remote_port,
                                                    &payload,
                                                    sizeof(payload));
                        }
                    }
                }
                else
                {
                    ImGui::Text("UART (NetID: %d)", m_net_id);
                    ImGui::Text("Status: Connected");
                    if (ImGui::Button("Disconnect"))
                    {
                        m_connected = false;
                    }
                }
            }
            ImGui::End();
        }

        void Set_ImGui_Context(void* context) override
        {
            m_imgui_context = context;
        }

    private:
        static bool On_Compare_Static(void* context, const void* payload, size_t size)
        {
            return static_cast<CRemote_UART*>(context)->On_Compare(payload, size);
        }

        bool On_Compare(const void* payload, size_t size)
        {
            struct UARTPayload
            {
                uint8_t proto;
                uint32_t baud;
                uint32_t net_id;
            } __attribute__((packed));
            if (size < sizeof(UARTPayload))
                return false;
            const auto* p = static_cast<const UARTPayload*>(payload);
            return (p->proto == 0 && p->net_id == (uint32_t)m_net_id);
        }

        static void On_Receive_Static(void* context, const void* data, size_t size)
        {
            static_cast<CRemote_UART*>(context)->On_Receive(data, size);
        }

        void On_Receive(const void* data, size_t size)
        {
            const uint32_t* bits = static_cast<const uint32_t*>(data);
            size_t count = size / sizeof(uint32_t);
            for (size_t i = 0; i < count; ++i)
            {
                m_rx_writer.insert_with_backoff(bits[i], m_rx_backoff);
                m_rx_backoff.wake();
            }
        }

        static void On_Disconnect_Static(void* context)
        {
            static_cast<CRemote_UART*>(context)->m_connected = false;
        }

        static void On_Handshake_Result_Static(void* context, bool success)
        {
            static_cast<CRemote_UART*>(context)->m_connected = success;
        }

        void BufferBit()
        {
            bool tx_state = m_read_pin(m_tx_pin);
            uint32_t packed = m_baud_rate_ticks & 0x7FFFFFFF;
            if (tx_state)
                packed |= (1U << 31);
            if (m_server_send && m_server_id != 0)
                m_server_send(m_server_id, &packed, sizeof(packed));
        }

        void DrainBit()
        {
            if (m_rx_reader.try_advance())
            {
                uint32_t packed = m_rx_reader.peek();
                m_rx_reader.advance();
                m_set_pin(m_rx_pin, (packed & (1U << 31)) != 0);
            }
        }

        std::string m_name;
        int m_tx_pin{ 14 }, m_rx_pin{ 15 }, m_baudrate_val{ 115200 }, m_net_id{ 1 }, m_remote_port{ 5000 };
        char m_remote_ip[64]{ 0 };
        uint32_t m_baud_rate_ticks{ 0 }, m_cpu_cycles{ 0 }, m_server_id{ 0 };
        bool m_connected{ false };
        IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };
        remote_protocol::register_t m_server_register{ nullptr };
        remote_protocol::unregister_t m_server_unregister{ nullptr };
        remote_protocol::send_t m_server_send{ nullptr };
        remote_protocol::init_handshake_t m_server_init_handshake{ nullptr };
        TSP::Queue::Buffer<uint32_t, QUEUE_SIZE> m_rx_buffer_buf;
        TSP::Queue::Reader<uint32_t, QUEUE_SIZE> m_rx_reader;
        TSP::Queue::Writer<uint32_t, QUEUE_SIZE> m_rx_writer;
        TSP::BF::SemBackoff m_rx_backoff;
    };
}

extern "C"
{
    zero_mate::IExternal_Peripheral::NInit_Status
    Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
                      const char* const name,
                      const uint32_t* const,
                      size_t,
                      zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                      zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                      zero_mate::IExternal_Peripheral::Halt_t,
                      zero_mate::IExternal_Peripheral::Start_t,
                      zero_mate::utils::CLogging_System* logging_system)
    {
        *peripheral = new (std::nothrow) zero_mate::peripheral::CRemote_UART(name, read_pin, set_pin, logging_system);
        return (*peripheral == nullptr) ? zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error
                                        : zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
