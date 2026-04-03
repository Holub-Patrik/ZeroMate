#include <utility>
#include <vector>
#include <string>
#include <array>
#include <atomic>
#include <cstring>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/RemoteProtocol.hpp"
#include "zero_mate/Protocol.hpp"
#include "CircularBufferQueue.hpp"

namespace
{
    enum class I2C_Packet_Type : uint8_t
    {
        I2C_START,
        I2C_STOP,
        I2C_ADDRESS,
        I2C_WRITE_BYTE,
        I2C_READ_BYTE,
        I2C_ACK,
        I2C_DATA
    };
    struct I2C_Packet
    {
        I2C_Packet_Type type;
        uint8_t value;
    };

    struct I2CPayload
    {
        uint8_t proto;
        uint8_t is_master;
        int32_t slave_id;
        uint32_t bus_id;
    } __attribute__((packed));

}

namespace zero_mate::peripheral
{
    class CRemote_I2C_Slave final : public IExternal_Peripheral
    {
    public:
        std::string m_name;
        int m_sda_pin{ 2 };
        int m_scl_pin{ 3 };
        int m_address{ 0x3C };
        int m_slave_id{ 10 };
        int m_bus_id{ 12 };
        int m_remote_port{ 5000 };
        char m_remote_ip[64]{ 0 };

        IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };

        int m_server_fd{ -1 };
        bool m_connected{ false };

        remote_protocol::register_t m_server_register{ nullptr };
        remote_protocol::unregister_t m_server_unregister{ nullptr };
        remote_protocol::init_handshake_t m_server_init_handshake{ nullptr };

        std::atomic<bool> m_running{ false };
        std::thread m_rx_thread;

        CRemote_I2C_Slave(std::string name,
                          IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                          IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                          utils::CLogging_System* logging_system)
        : m_name{ std::move(name) }
        , m_read_pin{ read_pin }
        , m_set_pin{ set_pin }
        , m_logging_system{ logging_system }
        {
            void* proc = LIB_SELF();
            m_server_register = (remote_protocol::register_t)LIB_SYM(proc, "server_register_channel");
            m_server_unregister = (remote_protocol::unregister_t)LIB_SYM(proc, "server_unregister_channel");
            m_server_init_handshake = (remote_protocol::init_handshake_t)LIB_SYM(proc, "server_init_handshake");

            if (m_server_register != nullptr)
            {
                m_server_fd = m_server_register("i2c_slave",
                                                On_Compare_Static,
                                                On_Disconnect_Static,
                                                On_Handshake_Result_Static,
                                                this);
            }

            std::strncpy(m_remote_ip, "127.0.0.1", sizeof(m_remote_ip) - 1);
        }

        ~CRemote_I2C_Slave() override
        {
            m_running = false;
            if (m_rx_thread.joinable())
            {
                m_rx_thread.join();
            }

            if (m_server_fd != -1 && (m_server_unregister != nullptr))
            {
                m_server_unregister(m_server_fd);
            }
        }

        void Render() override
        {
            if (m_imgui_context != nullptr)
            {
                ImGui::SetCurrentContext(static_cast<ImGuiContext*>(m_imgui_context));
            }
            if (ImGui::Begin(m_name.c_str()))
            {
                if (!m_connected)
                {
                    ImGui::InputText("Remote IP", m_remote_ip, sizeof(m_remote_ip));
                    ImGui::InputInt("Remote Port", &m_remote_port);
                    ImGui::InputInt("Slave ID", &m_slave_id);
                    ImGui::InputInt("Bus ID", &m_bus_id);
                    ImGui::InputInt("Address", &m_address);
                    ImGui::InputInt("SDA Pin", &m_sda_pin);
                    ImGui::InputInt("SCL Pin", &m_scl_pin);
                    if (ImGui::Button("Connect to Master"))
                    {
                        if (m_server_fd != -1 && (m_server_init_handshake != nullptr))
                        {
                            I2CPayload payload = { .proto = 1,
                                                   .is_master = 0,
                                                   .slave_id = (int32_t)m_slave_id,
                                                   .bus_id = (uint32_t)m_bus_id };

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
                    ImGui::Text("I2C Slave (Addr: 0x%02X, Bus: %d)", m_address, m_bus_id);
                    ImGui::Text("Status: Connected");
                    if (ImGui::Button("Disconnect"))
                    {
                        m_connected = false;
                        if (m_server_fd != -1 && (m_server_unregister != nullptr))
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
        static bool On_Compare_Static(void* /*context*/, const void* /*payload*/, size_t /*size*/)
        {
            return false;
        }

        void On_Receive(const void* data, size_t size)
        {
            if (size != sizeof(I2C_Packet))
            {
                return;
            }

            const auto* packet = static_cast<const I2C_Packet*>(data);
            switch (packet->type)
            {
                case I2C_Packet_Type::I2C_START:
                    start_local();
                    break;
                case I2C_Packet_Type::I2C_STOP:
                    stop_local();
                    break;
                case I2C_Packet_Type::I2C_ADDRESS: {
                    start_local();
                    bit_bang_byte_local(packet->value);
                    bool ack = (packet->value >> 1U) == (uint32_t)m_address;
                    Send_Packet(I2C_Packet_Type::I2C_ACK, ack ? 1 : 0);
                    break;
                }
                case I2C_Packet_Type::I2C_WRITE_BYTE:
                    bit_bang_byte_local(packet->value);
                    Send_Packet(I2C_Packet_Type::I2C_ACK, read_ack_local() ? 1 : 0);
                    break;
                case I2C_Packet_Type::I2C_READ_BYTE:
                    Send_Packet(I2C_Packet_Type::I2C_DATA, read_byte_local());
                    break;
                case I2C_Packet_Type::I2C_ACK:
                    write_ack_local(packet->value != 0);
                    break;
                default:
                    break;
            }
        }

        static void On_Disconnect_Static(void* context)
        {
            static_cast<CRemote_I2C_Slave*>(context)->m_connected = false;
        }

        static void
        On_Handshake_Result_Static(void* context, bool success, int fd, const char* remote_ip, uint16_t remote_port)
        {
            static_cast<CRemote_I2C_Slave*>(context)->On_Handshake_Result(success, fd, remote_ip, remote_port);
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
                    m_connected = true;
                    m_running = true;
                    m_rx_thread = std::thread(&CRemote_I2C_Slave::RX_Thread, this);
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
                    On_Receive(buffer, received);
                }
                else if (received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    m_connected = false;
                    break;
                }
            }
        }

        void bit_bang_byte_local(uint8_t value) const
        {
            for (int i = 7; i >= 0; --i)
            {
                m_set_pin(m_sda_pin, (value >> i) & 0x01U);
                m_set_pin(m_scl_pin, true);
                m_set_pin(m_scl_pin, false);
            }
        }

        uint8_t read_byte_local() const
        {
            uint8_t value = 0;
            m_set_pin(m_sda_pin, true);

            for (int i = 7; i >= 0; --i)
            {
                m_set_pin(m_scl_pin, true);
                value |= (m_read_pin(m_sda_pin) << i);
                m_set_pin(m_scl_pin, false);
            }

            return value;
        }

        void start_local() const
        {
            m_set_pin(m_sda_pin, true);
            m_set_pin(m_scl_pin, true);
            m_set_pin(m_sda_pin, false);
            m_set_pin(m_scl_pin, false);
        }

        void stop_local() const
        {
            m_set_pin(m_sda_pin, false);
            m_set_pin(m_scl_pin, true);
            m_set_pin(m_sda_pin, true);
        }

        bool read_ack_local() const
        {
            m_set_pin(m_sda_pin, true);
            m_set_pin(m_scl_pin, true);

            bool ack = m_read_pin(m_sda_pin);

            m_set_pin(m_scl_pin, false);
            return ack;
        }

        void write_ack_local(bool ack) const
        {
            m_set_pin(m_sda_pin, !ack);
            m_set_pin(m_scl_pin, true);
            m_set_pin(m_scl_pin, false);
            m_set_pin(m_sda_pin, true);
        }

        void Send_Packet(I2C_Packet_Type type, uint8_t value) const
        {
            I2C_Packet packet{ .type = type, .value = value };
            if (m_server_fd != -1)
            {
                send(m_server_fd, &packet, sizeof(packet), 0);
            }
        }
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
        *peripheral =
        new (std::nothrow) zero_mate::peripheral::CRemote_I2C_Slave(name, read_pin, set_pin, logging_system);
        return (*peripheral == nullptr) ? zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error
                                        : zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
