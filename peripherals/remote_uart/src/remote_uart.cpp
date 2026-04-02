#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

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
        static constexpr uint32_t Clock_Rate = 250000000;

        enum class UART_State
        {
            Idle,
            Data_Bits,
            Stop_Bit
        };

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
                m_server_fd = m_server_register("uart",
                                                On_Compare_Static,
                                                On_Receive_Static,
                                                On_Disconnect_Static,
                                                On_Handshake_Result_Static,
                                                this);
            }

            std::strncpy(m_remote_ip, "127.0.0.1", sizeof(m_remote_ip) - 1);
        }

        ~CRemote_UART() override
        {
            m_running = false;
            if (m_rx_thread.joinable())
                m_rx_thread.join();

            if (m_server_fd != -1 && m_server_unregister)
                m_server_unregister(m_server_fd);
        }

        void Increment_Passed_Cycles(uint32_t count) override
        {
            if (!m_connected || m_bit_time_cycles == 0)
                return;

            // Process TX (Guest -> Network)
            if (m_tx_state == UART_State::Idle)
            {
                if (!m_read_pin(m_tx_pin)) // Start bit edge
                {
                    m_tx_state = UART_State::Data_Bits;
                    m_tx_timer = (m_bit_time_cycles * 3) / 2; // 1.5 bit times to Bit 0
                    m_tx_bit_count = 0;
                    m_tx_shift_reg = 0;
                }
            }
            else
            {
                if (count >= m_tx_timer)
                {
                    uint32_t remaining = count - m_tx_timer;
                    m_tx_timer = 0;
                    
                    if (m_tx_state == UART_State::Data_Bits)
                    {
                        if (m_read_pin(m_tx_pin)) m_tx_shift_reg |= (1U << m_tx_bit_count);
                        m_tx_bit_count++;
                        
                        if (m_tx_bit_count >= 8)
                        {
                            m_tx_state = UART_State::Stop_Bit;
                            m_tx_timer = m_bit_time_cycles;
                        }
                        else
                        {
                            m_tx_timer = m_bit_time_cycles;
                        }
                    }
                    else if (m_tx_state == UART_State::Stop_Bit)
                    {
                        if (m_logging_system)
                            m_logging_system->Debug(fmt::format("Remote UART: Sending 0x{:02X}", m_tx_shift_reg).c_str());
                        
                        send(m_server_fd, &m_tx_shift_reg, 1, 0);
                        m_tx_state = UART_State::Idle;
                    }
                    
                    if (m_tx_timer > remaining) m_tx_timer -= remaining;
                    else m_tx_timer = 0;
                }
                else
                {
                    m_tx_timer -= count;
                }
            }

            // Process RX (Network -> Guest)
            if (m_rx_state == UART_State::Idle)
            {
                if (m_rx_reader.try_advance())
                {
                    m_rx_current_byte = m_rx_reader.peek();
                    m_rx_reader.advance();
                    m_rx_backoff.wake();
                    
                    m_rx_state = UART_State::Data_Bits;
                    m_rx_timer = m_bit_time_cycles;
                    m_rx_bit_count = 0;
                    m_set_pin(m_rx_pin, false); // Start bit
                }
                else
                {
                    m_set_pin(m_rx_pin, true); // Idle High
                }
            }
            else
            {
                if (count >= m_rx_timer)
                {
                    uint32_t remaining = count - m_rx_timer;
                    m_rx_timer = 0;

                    if (m_rx_state == UART_State::Data_Bits)
                    {
                        m_set_pin(m_rx_pin, (m_rx_current_byte & (1U << m_rx_bit_count)) != 0);
                        m_rx_bit_count++;
                        
                        if (m_rx_bit_count >= 8)
                        {
                            m_rx_state = UART_State::Stop_Bit;
                            m_rx_timer = m_bit_time_cycles * 2; // 2 stop bits for safety
                        }
                        else
                        {
                            m_rx_timer = m_bit_time_cycles;
                        }
                    }
                    else if (m_rx_state == UART_State::Stop_Bit)
                    {
                        m_set_pin(m_rx_pin, true);
                        m_rx_state = UART_State::Idle;
                    }

                    if (m_rx_timer > remaining) m_rx_timer -= remaining;
                    else m_rx_timer = 0;
                }
                else
                {
                    m_rx_timer -= count;
                }
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
                        m_bit_time_cycles = (Clock_Rate / (uint32_t)m_baudrate_val) / 8;
                        if (m_server_fd != -1 && m_server_init_handshake)
                        {
                            struct UARTPayload
                            {
                                uint8_t proto;
                                uint32_t baud;
                                uint32_t net_id;
                            } __attribute__((packed));
                            UARTPayload payload = { 0, (uint32_t)m_baudrate_val, (uint32_t)m_net_id };
                            m_server_init_handshake(m_server_fd,
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
                        if (m_server_fd != -1 && m_server_unregister)
                        {
                            m_server_unregister(m_server_fd);
                            m_server_fd = -1;
                        }
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

        static void On_Receive_Static(void* /*context*/, const void* /*data*/, size_t /*size*/)
        {
        }

        static void On_Disconnect_Static(void* context)
        {
            static_cast<CRemote_UART*>(context)->m_connected = false;
        }

        static void On_Handshake_Result_Static(void* context, bool success, int fd, const char* remote_ip, uint16_t remote_port)
        {
            static_cast<CRemote_UART*>(context)->On_Handshake_Result(success, fd, remote_ip, remote_port);
        }

        void On_Handshake_Result(bool success, int fd, const char* remote_ip, uint16_t remote_port)
        {
            if (success)
            {
                struct sockaddr_in remote_addr{ };
                remote_addr.sin_family = AF_INET;
                remote_addr.sin_port = htons(remote_port);
                inet_pton(AF_INET, remote_ip, &remote_addr.sin_addr);

                if (connect(fd, (struct sockaddr*)&remote_addr, sizeof(remote_addr)) == 0)
                {
                    m_bit_time_cycles = (Clock_Rate / (uint32_t)m_baudrate_val) / 8;
                    if (m_logging_system)
                        m_logging_system->Info(fmt::format("Remote UART: Connected to {}:{} (bit time cycles: {})", remote_ip, remote_port, m_bit_time_cycles).c_str());
                    
                    m_connected = true;
                    m_running = true;
                    m_rx_thread = std::thread(&CRemote_UART::RX_Thread, this);
                }
                else
                {
                    if (m_logging_system)
                        m_logging_system->Error(fmt::format("Remote UART: Failed to connect to {}:{} (errno {})", remote_ip, remote_port, errno).c_str());
                }
            }
        }

        void RX_Thread()
        {
            uint8_t buffer[1024];
            while (m_running)
            {
                ssize_t received = recv(m_server_fd, buffer, sizeof(buffer), 0);
                if (received > 0)
                {
                    if (m_logging_system)
                        m_logging_system->Debug(fmt::format("Remote UART: Received {} bytes from network", received).c_str());

                    for (ssize_t i = 0; i < received; ++i)
                    {
                        m_rx_writer.insert_with_backoff(buffer[i], m_rx_backoff);
                        m_rx_backoff.wake();
                    }
                }
                else if (received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    if (m_logging_system)
                        m_logging_system->Info("Remote UART: Network connection closed");
                    m_connected = false;
                    break;
                }
            }
        }

        std::string m_name;
        int m_tx_pin{ 14 }, m_rx_pin{ 15 }, m_baudrate_val{ 115200 }, m_net_id{ 1 }, m_remote_port{ 5000 };
        char m_remote_ip[64]{ 0 };
        uint32_t m_bit_time_cycles{ 0 };
        int m_server_fd{ -1 };
        bool m_connected{ false };
        IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };
        remote_protocol::register_t m_server_register{ nullptr };
        remote_protocol::unregister_t m_server_unregister{ nullptr };
        remote_protocol::send_t m_server_send{ nullptr };
        remote_protocol::init_handshake_t m_server_init_handshake{ nullptr };
        
        TSP::Queue::Buffer<uint8_t, QUEUE_SIZE> m_rx_buffer_buf;
        TSP::Queue::Reader<uint8_t, QUEUE_SIZE> m_rx_reader;
        TSP::Queue::Writer<uint8_t, QUEUE_SIZE> m_rx_writer;
        TSP::BF::SemBackoff m_rx_backoff;

        UART_State m_tx_state{ UART_State::Idle };
        uint32_t m_tx_timer{ 0 };
        uint8_t m_tx_shift_reg{ 0 };
        uint8_t m_tx_bit_count{ 0 };

        UART_State m_rx_state{ UART_State::Idle };
        uint32_t m_rx_timer{ 0 };
        uint8_t m_rx_current_byte{ 0 };
        uint8_t m_rx_bit_count{ 0 };

        std::atomic<bool> m_running{ false };
        std::thread m_rx_thread;
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
