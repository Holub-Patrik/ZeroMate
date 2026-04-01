#include <vector>
#include <string>
#include <array>
#include <atomic>
#include <cstring>

#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/RemoteProtocol.hpp"
#include "zero_mate/Protocol.hpp"
#include "CircularBufferQueue.hpp"

namespace zero_mate::peripheral
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

    class CRemote_I2C_Slave final : public IExternal_Peripheral
    {
    public:
        CRemote_I2C_Slave(const std::string& name,
                          IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                          IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                          utils::CLogging_System* logging_system)
        : m_name{ name }
        , m_read_pin{ read_pin }
        , m_set_pin{ set_pin }
        , m_logging_system{ logging_system }
        {
            void* proc = LIB_SELF();
            m_server_register = (remote_protocol::register_t)LIB_SYM(proc, "server_register_channel");
            m_server_unregister = (remote_protocol::unregister_t)LIB_SYM(proc, "server_unregister_channel");
            m_server_send = (remote_protocol::send_t)LIB_SYM(proc, "server_send_data");
            m_server_init_handshake = (remote_protocol::init_handshake_t)LIB_SYM(proc, "server_init_handshake");

            if (m_server_register)
                m_server_id = m_server_register("i2c_slave",
                                                On_Compare_Static,
                                                On_Receive_Static,
                                                On_Disconnect_Static,
                                                On_Handshake_Result_Static,
                                                this);

            std::strncpy(m_remote_ip, "127.0.0.1", sizeof(m_remote_ip) - 1);
        }

        ~CRemote_I2C_Slave() override
        {
            if (m_server_id != 0 && m_server_unregister)
                m_server_unregister(m_server_id);
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
                    ImGui::InputInt("Slave ID", &m_slave_id);
                    ImGui::InputInt("Bus ID", &m_bus_id);
                    ImGui::InputInt("Address", &m_address);
                    ImGui::InputInt("SDA Pin", &m_sda_pin);
                    ImGui::InputInt("SCL Pin", &m_scl_pin);
                    if (ImGui::Button("Connect to Master"))
                    {
                        if (m_server_id != 0 && m_server_init_handshake)
                        {
                            struct I2CPayload
                            {
                                uint8_t proto;
                                uint8_t is_master;
                                int32_t slave_id;
                                uint32_t bus_id;
                            } __attribute__((packed));
                            I2CPayload payload = { 1, 0, (int32_t)m_slave_id, (uint32_t)m_bus_id };
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
                    ImGui::Text("I2C Slave (Addr: 0x%02X, Bus: %d)", m_address, m_bus_id);
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
        static bool On_Compare_Static(void* /*context*/, const void* /*payload*/, size_t /*size*/)
        {
            return false;
        }
        static void On_Receive_Static(void* context, const void* data, size_t size)
        {
            static_cast<CRemote_I2C_Slave*>(context)->On_Receive(data, size);
        }

        void On_Receive(const void* data, size_t size)
        {
            if (size != sizeof(I2C_Packet))
                return;
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
        static void On_Handshake_Result_Static(void* context, bool success)
        {
            static_cast<CRemote_I2C_Slave*>(context)->m_connected = success;
        }

        void bit_bang_byte_local(uint8_t value)
        {
            for (int i = 7; i >= 0; --i)
            {
                m_set_pin(m_sda_pin, (value >> i) & 0x01U);
                m_set_pin(m_scl_pin, 1);
                m_set_pin(m_scl_pin, 0);
            }
        }
        uint8_t read_byte_local()
        {
            uint8_t value = 0;
            m_set_pin(m_sda_pin, 1);
            for (int i = 7; i >= 0; --i)
            {
                m_set_pin(m_scl_pin, 1);
                value |= (m_read_pin(m_sda_pin) << i);
                m_set_pin(m_scl_pin, 0);
            }
            return value;
        }
        void start_local()
        {
            m_set_pin(m_sda_pin, 1);
            m_set_pin(m_scl_pin, 1);
            m_set_pin(m_sda_pin, 0);
            m_set_pin(m_scl_pin, 0);
        }
        void stop_local()
        {
            m_set_pin(m_sda_pin, 0);
            m_set_pin(m_scl_pin, 1);
            m_set_pin(m_sda_pin, 1);
        }
        bool read_ack_local()
        {
            m_set_pin(m_sda_pin, 1);
            m_set_pin(m_scl_pin, 1);
            bool ack = (m_read_pin(m_sda_pin) == 0);
            m_set_pin(m_scl_pin, 0);
            return ack;
        }
        void write_ack_local(bool ack)
        {
            m_set_pin(m_sda_pin, ack ? 0 : 1);
            m_set_pin(m_scl_pin, 1);
            m_set_pin(m_scl_pin, 0);
            m_set_pin(m_sda_pin, 1);
        }
        void Send_Packet(I2C_Packet_Type type, uint8_t value)
        {
            I2C_Packet p{ type, value };
            if (m_server_send && m_server_id != 0)
                m_server_send(m_server_id, &p, sizeof(p));
        }

        std::string m_name;
        int m_sda_pin{ 2 }, m_scl_pin{ 3 }, m_address{ 0x3C }, m_slave_id{ 10 }, m_bus_id{ 12 }, m_remote_port{ 5000 };
        char m_remote_ip[64]{ 0 };
        IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };
        uint32_t m_server_id{ 0 };
        bool m_connected{ false };
        remote_protocol::register_t m_server_register{ nullptr };
        remote_protocol::unregister_t m_server_unregister{ nullptr };
        remote_protocol::send_t m_server_send{ nullptr };
        remote_protocol::init_handshake_t m_server_init_handshake{ nullptr };
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
