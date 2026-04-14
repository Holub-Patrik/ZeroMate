#include <vector>
#include <string>
#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstdio>

#include "fmt/format.h"
#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/gpio_server_abi.hpp"
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
        uint32_t bus_id;
    } __attribute__((packed));

    class CRemote_I2C_Slave final : public zero_mate::IExternal_Peripheral
    {
    public:
        std::string m_name;
        int m_sda_pin{ 18 };
        int m_scl_pin{ 19 };
        int m_address{ 0x76 };
        int m_bus_id{ 1 };
        int m_remote_port{ 9000 };
        char m_remote_ip[64]{ 0 };

        zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        zero_mate::utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };

        int m_server_fd{ -1 };
        int m_stop_pipe[2]{ -1, -1 };
        bool m_connected{ false };
        struct sockaddr_in m_remote_addr{ };

        zero_mate::remote_protocol::register_t m_server_register{ nullptr };
        zero_mate::remote_protocol::unregister_t m_server_unregister{ nullptr };
        zero_mate::remote_protocol::disconnect_t m_server_disconnect{ nullptr };
        zero_mate::remote_protocol::init_handshake_t m_server_init_handshake{ nullptr };

        std::atomic<bool> m_running{ false };
        std::thread m_rx_thread;

        CRemote_I2C_Slave(std::string name,
                          uint32_t sda_pin,
                          uint32_t scl_pin,
                          zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                          zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                          zero_mate::utils::CLogging_System* logging_system)
        : m_name{ std::move(name) }
        , m_sda_pin{ (int)sda_pin }
        , m_scl_pin{ (int)scl_pin }
        , m_read_pin{ read_pin }
        , m_set_pin{ set_pin }
        , m_logging_system{ logging_system }
        {
            if (pipe(m_stop_pipe) == -1)
            {
                if (m_logging_system != nullptr)
                {
                    m_logging_system->Error("Remote I2C Slave: Failed to create stop pipe");
                }
            }

            void* proc = LIB_OPEN_SERVER(LIB_NAME("gpio_server"));
            auto get_gs_abi =
            (zero_mate::peripheral::TGPIOServerABI (*)())LIB_LOOKUP_SYMBOL(proc, "Get_GPIO_Server_ABI");

            if (get_gs_abi != nullptr)
            {
                const auto abi = get_gs_abi();

                m_server_register = abi.register_channel;
                m_server_unregister = abi.unregister_channel;
                m_server_disconnect = abi.disconnect_channel;
                m_server_init_handshake = abi.init_handshake;
            }

            if (m_server_register != nullptr)
            {
                m_server_fd = m_server_register("i2c_slave",
                                                On_Compare_Static,
                                                On_Disconnect_Static,
                                                On_Handshake_Result_Static,
                                                this);
            }

            strncpy(m_remote_ip, "127.0.0.1", sizeof(m_remote_ip));
        }

        ~CRemote_I2C_Slave() override
        {
            Stop_RX_Thread();

            if (m_server_fd != -1 && (m_server_unregister != nullptr))
            {
                m_server_unregister(m_server_fd);
            }

            if (m_stop_pipe[0] != -1)
            {
                close(m_stop_pipe[0]);
            }
            if (m_stop_pipe[1] != -1)
            {
                close(m_stop_pipe[1]);
            }
        }

        void ResetState()
        {
            // Reset any internal state if needed
        }

        void Stop_RX_Thread()
        {
            if (!m_running)
            {
                return;
            }

            m_running = false;

            if (m_stop_pipe[1] != -1)
            {
                uint64_t val = 1;
                (void)write(m_stop_pipe[1], &val, sizeof(val));
            }

            ResetState();

            if (m_rx_thread.joinable())
            {
                m_rx_thread.join();
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
                    ImGui::InputInt("Bus ID", &m_bus_id);
                    ImGui::InputInt("Address", &m_address);
                    ImGui::InputInt("SDA Pin", &m_sda_pin);
                    ImGui::InputInt("SCL Pin", &m_scl_pin);
                    if (ImGui::Button("Connect to Master"))
                    {
                        ResetState();
                        if (m_server_fd != -1 && (m_server_init_handshake != nullptr))
                        {
                            I2CPayload payload = { .proto = 1, .is_master = 0, .bus_id = (uint32_t)m_bus_id };

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
                        if (m_server_fd != -1 && (m_server_disconnect != nullptr))
                        {
                            m_server_disconnect(m_server_fd);
                        }
                        m_connected = false;
                        Stop_RX_Thread();
                        ResetState();
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

            if (m_logging_system != nullptr)
            {
                m_logging_system->Debug(fmt::format("Remote I2C Slave: Received packet: {} (value: 0x{:02X})",
                                                    static_cast<int>(packet->type),
                                                    packet->value)
                                        .c_str());
            }

            switch (packet->type)
            {
                case I2C_Packet_Type::I2C_START:
                    start_local();
                    break;
                case I2C_Packet_Type::I2C_STOP:
                    stop_local();
                    break;
                case I2C_Packet_Type::I2C_ADDRESS: {
                    bit_bang_byte_local(packet->value);
                    Send_Packet(I2C_Packet_Type::I2C_ACK, read_ack_local() ? 1 : 0);
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

        static void On_Disconnect_Static(void* context, const char* remote_ip, uint16_t remote_port)
        {
            auto* slave = static_cast<CRemote_I2C_Slave*>(context);
            if (slave->m_logging_system)
            {
                slave->m_logging_system->Info(
                fmt::format("Remote I2C Slave: Disconnected from peer {}:{}", remote_ip, remote_port).c_str());
            }
            slave->m_connected = false;
            slave->Stop_RX_Thread();
            slave->ResetState();
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
                m_remote_addr.sin_family = AF_INET;
                m_remote_addr.sin_port = htons(remote_port);
                inet_pton(AF_INET, remote_ip, &m_remote_addr.sin_addr);

                if (connect(fd, (struct sockaddr*)&m_remote_addr, sizeof(m_remote_addr)) == 0)
                {
                    Stop_RX_Thread();
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
                struct pollfd pfds[2];
                pfds[0].fd = m_server_fd;
                pfds[0].events = POLLIN;
                pfds[1].fd = m_stop_pipe[0];
                pfds[1].events = POLLIN;

                const auto poll_ret = poll(pfds, 2, -1);

                if (poll_ret <= 0)
                {
                    continue;
                }

                if (pfds[1].revents & POLLIN)
                {
                    break;
                }

                if (!(pfds[0].revents & POLLIN))
                {
                    continue;
                }

                ssize_t received = recv(m_server_fd, buffer, sizeof(buffer), 0);
                if (received > 0)
                {
                    On_Receive(buffer, received);
                }
                else if (received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    if (m_logging_system)
                    {
                        m_logging_system->Info("Remote I2C Slave: Network connection closed");
                    }
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

            for (int i = 7; i >= 0; --i)
            {
                m_set_pin(m_scl_pin, true);
                value |= static_cast<uint8_t>(m_read_pin(m_sda_pin) << i);
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
            m_set_pin(m_scl_pin, false);
            m_set_pin(m_sda_pin, false);
            m_set_pin(m_scl_pin, true);
            m_set_pin(m_sda_pin, true);
        }

        bool read_ack_local() const
        {
            m_set_pin(m_scl_pin, true);
            bool ack = !m_read_pin(m_sda_pin);
            m_set_pin(m_scl_pin, false);
            return ack;
        }

        void write_ack_local(bool ack) const
        {
            m_set_pin(m_sda_pin, !ack);
            m_set_pin(m_scl_pin, true);
            m_set_pin(m_scl_pin, false);
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
                      const uint32_t* const connection,
                      size_t pin_count,
                      zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                      zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                      zero_mate::IExternal_Peripheral::Halt_t /* halt */,
                      zero_mate::IExternal_Peripheral::Start_t /* start */,
                      zero_mate::utils::CLogging_System* logging_system)
    {
        if (pin_count != 2)
        {
            return zero_mate::IExternal_Peripheral::NInit_Status::GPIO_Mismatch;
        }

        *peripheral =
        new (std::nothrow) CRemote_I2C_Slave(name, connection[0], connection[1], read_pin, set_pin, logging_system);
        return (*peripheral == nullptr) ? zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error
                                        : zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
