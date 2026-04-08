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

#include "fmt/format.h"
#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/RemoteProtocol.hpp"
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

    enum class I2C_State : std::uint8_t
    {
        IDLE,
        ADDRESS,
        READ_BYTE,
        WRITE_BYTE,
        RESPONSE
    };

    struct I2CPayload
    {
        uint8_t proto;
        uint8_t is_master;
        uint32_t bus_id;
    } __attribute__((packed));

}

namespace zero_mate::peripheral
{
    class CRemote_I2C_Master final : public IExternal_Peripheral
    {
    public:
        static constexpr size_t QUEUE_SIZE = 128;

        std::string m_name;
        int m_bus_id{ 1 };
        int m_sda_pin{ 2 };
        int m_scl_pin{ 3 };

        IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        IExternal_Peripheral::Halt_t m_halt;
        IExternal_Peripheral::Start_t m_start;
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };

        int m_server_fd{ -1 };
        int m_stop_pipe[2]{ -1, -1 };
        std::atomic<uint32_t> m_slave_count{ 0 };
        bool m_scl_lvl{ true };
        bool m_sda_lvl{ true };
        bool m_is_read{ false };
        bool m_ack_from_slave{ false };

        I2C_State m_state{ I2C_State::IDLE };
        std::uint8_t m_bit_count{ 0 };
        std::uint8_t m_shift_reg{ 0 };

        remote_protocol::register_t m_server_register{ nullptr };
        remote_protocol::unregister_t m_server_unregister{ nullptr };
        remote_protocol::init_handshake_t m_server_init_handshake{ nullptr };

        TSP::Queue::Buffer<uint8_t, QUEUE_SIZE> m_queue_buf;
        TSP::Queue::Reader<uint8_t, QUEUE_SIZE> m_queue_reader;
        TSP::Queue::Writer<uint8_t, QUEUE_SIZE> m_queue_writer;
        TSP::BF::SemBackoff m_reader_backoff, m_writer_backoff;

        std::mutex m_slaves_mutex;
        std::vector<struct sockaddr_in> m_slave_addrs;

        std::atomic<bool> m_running{ false };
        std::thread m_rx_thread;

        CRemote_I2C_Master(const std::string& name,
                           IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                           IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                           IExternal_Peripheral::Halt_t halt,
                           IExternal_Peripheral::Start_t start,
                           utils::CLogging_System* logging_system)
        : m_name{ name }
        , m_read_pin{ read_pin }
        , m_set_pin{ set_pin }
        , m_halt{ halt }
        , m_start{ start }
        , m_logging_system{ logging_system }
        , m_queue_writer(&m_queue_buf)
        , m_queue_reader(&m_queue_buf)
        , m_reader_backoff(10, 100)
        , m_writer_backoff(10, 100)
        {
            if (pipe(m_stop_pipe) == -1)
            {
                if (m_logging_system)
                {
                    m_logging_system->Error("Remote I2C Master: Failed to create stop pipe");
                }
            }

            void* proc = LIB_SELF();
            m_server_register = (remote_protocol::register_t)LIB_SYM(proc, "server_register_channel");
            m_server_unregister = (remote_protocol::unregister_t)LIB_SYM(proc, "server_unregister_channel");
            m_server_init_handshake = (remote_protocol::init_handshake_t)LIB_SYM(proc, "server_init_handshake");

            if (m_server_register != nullptr)
            {
                m_server_fd = m_server_register("i2c_master",
                                                On_Compare_Static,
                                                On_Disconnect_Static,
                                                On_Handshake_Result_Static,
                                                this);
            }

            if (m_server_fd != -1)
            {
                m_running = true;
                m_rx_thread = std::thread(&CRemote_I2C_Master::RX_Thread, this);
            }
        }

        ~CRemote_I2C_Master() override
        {
            Stop_RX_Thread();

            if (m_server_fd != -1 && (m_server_unregister != nullptr))
            {
                m_server_unregister(m_server_fd);
            }

            if (m_stop_pipe[0] != -1)
            {
                close(m_stop_pipe[0]);
                close(m_stop_pipe[1]);
            }
        }

        void ResetState()
        {
            m_scl_lvl = true;
            m_sda_lvl = true;
            m_is_read = false;
            m_ack_from_slave = false;
            m_state = I2C_State::IDLE;
            m_bit_count = 0;
            m_shift_reg = 0;

            while (m_queue_reader.try_advance())
            {
                m_queue_reader.advance();
            }
            m_reader_backoff.wake();
            m_writer_backoff.wake();

            std::lock_guard<std::mutex> lock(m_slaves_mutex);
            m_slave_addrs.clear();
            m_slave_count = 0;
        }

        void Stop_RX_Thread()
        {
            if (!m_running)
            {
                return;
            }

            m_running = false;
            m_reader_backoff.wake();
            m_writer_backoff.wake();

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

        void GPIO_Subscription_Callback(uint32_t pin_idx) override
        {
            if (m_server_fd == -1)
            {
                return;
            }

            const bool curr_pin_state = m_read_pin(pin_idx);

            if (pin_idx == (uint32_t)m_scl_pin)
            {
                Handle_SCL(curr_pin_state);
            }
            else
            {
                Handle_SDA(curr_pin_state);
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
                ImGui::InputInt("Bus ID", &m_bus_id);
                ImGui::InputInt("SDA Pin", &m_sda_pin);
                ImGui::InputInt("SCL Pin", &m_scl_pin);
                ImGui::Separator();
                ImGui::Text("Registered Slaves: %zu", m_slave_addrs.size());
                if (ImGui::Button("Disconnect All"))
                {
                    if (m_server_fd != -1 && (m_server_unregister != nullptr))
                    {
                        void* proc = LIB_SELF();
                        auto dconn = (remote_protocol::disconnect_t)LIB_SYM(proc, "server_disconnect_channel");
                        if (dconn != nullptr)
                        {
                            dconn(m_server_fd);
                        }
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
        static bool On_Compare_Static(void* context, const void* payload, size_t size)
        {
            return static_cast<CRemote_I2C_Master*>(context)->On_Compare(payload, size);
        }

        bool On_Compare(const void* payload, size_t size) const
        {
            if (size < sizeof(I2CPayload))
            {
                return false;
            }

            const auto* protocol = static_cast<const I2CPayload*>(payload);

            if (protocol->proto != 1)
            {
                return false;
            }

            if (protocol->is_master != 0)
            {
                return false;
            }

            if (protocol->bus_id != m_bus_id)
            {
                return false;
            }

            return true;
        }

        static void On_Disconnect_Static(void* context, const char* remote_ip, uint16_t remote_port)
        {
            auto* master = static_cast<CRemote_I2C_Master*>(context);

            struct sockaddr_in addr{ };
            addr.sin_family = AF_INET;
            addr.sin_port = htons(remote_port);
            inet_pton(AF_INET, remote_ip, &addr.sin_addr);

            std::lock_guard<std::mutex> lock(master->m_slaves_mutex);
            std::erase_if(master->m_slave_addrs, [&](const auto& slave) {
                return slave.sin_addr.s_addr == addr.sin_addr.s_addr && slave.sin_port == addr.sin_port;
            });

            if (master->m_slave_count > 0)
            {
                master->m_slave_count--;
            }

            if (master->m_logging_system != nullptr)
            {
                master->m_logging_system->Info(
                fmt::format("Remote I2C Master: Peer {}:{} disconnected. Remaining slaves: {}",
                            remote_ip,
                            remote_port,
                            master->m_slave_count.load())
                .c_str());
            }
        }

        static void On_Handshake_Result_Static(
        void* context, bool success, int /* fd */, const char* remote_ip, uint16_t remote_port)
        {
            if (success)
            {
                auto* master = static_cast<CRemote_I2C_Master*>(context);
                struct sockaddr_in addr{ };
                addr.sin_family = AF_INET;
                addr.sin_port = htons(remote_port);
                inet_pton(AF_INET, remote_ip, &addr.sin_addr);

                std::lock_guard<std::mutex> lock(master->m_slaves_mutex);
                master->m_slave_addrs.push_back(addr);
            }
        }

        void RX_Thread()
        {
            uint8_t buffer[1024];
            struct sockaddr_in remote_addr{ };
            socklen_t addr_len = sizeof(remote_addr);

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

                ssize_t received =
                recvfrom(m_server_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&remote_addr, &addr_len);
                if (received == (ssize_t)sizeof(I2C_Packet))
                {
                    const auto* packet = reinterpret_cast<const I2C_Packet*>(buffer);
                    if (packet->type == I2C_Packet_Type::I2C_ACK)
                    {
                        m_ack_from_slave = (packet->value != 0);
                        if (!m_queue_writer.insert(m_ack_from_slave ? 0 : 1))
                        {
                            break;
                        }
                        m_reader_backoff.wake();
                        if (m_state == I2C_State::RESPONSE && m_bit_count == 0)
                        {
                            Drive_SDA_From_Queue();
                        }
                    }
                    else if (packet->type == I2C_Packet_Type::I2C_DATA)
                    {
                        for (int i = 7; i >= 0; --i)
                        {
                            if (!m_queue_writer.insert((packet->value >> i) & 0x01U))
                            {
                                break;
                            }
                            m_reader_backoff.wake();
                        }
                        if (m_state == I2C_State::READ_BYTE && m_bit_count == 0)
                        {
                            Drive_SDA_From_Queue();
                        }
                    }
                    m_start();
                }
                else if (received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    if (m_logging_system != nullptr)
                    {
                        m_logging_system->Info("Remote I2C Master: Network connection closed");
                    }
                    break;
                }
            }
        }

        void Handle_SCL(bool is_high)
        {
            bool wait_for_response = false;
            if (!m_scl_lvl && is_high) // Rising
            {
                if (m_state != I2C_State::IDLE)
                {
                    m_bit_count++;
                    if (m_state == I2C_State::ADDRESS || m_state == I2C_State::WRITE_BYTE)
                    {
                        m_shift_reg = static_cast<uint8_t>((m_shift_reg << 1U) | (m_sda_lvl ? 1U : 0U));
                    }
                }
                if (m_state == I2C_State::ADDRESS && m_bit_count == 8)
                {
                    m_is_read = (m_shift_reg & 0x01U);
                }
            }
            else if (m_scl_lvl && !is_high) // Falling
            {
                const bool slave_drives = (m_state == I2C_State::READ_BYTE && m_bit_count < 8) ||
                                          (m_state == I2C_State::RESPONSE && m_ack_from_slave && m_bit_count == 0);
                if (slave_drives)
                {
                    Drive_SDA_From_Queue();
                }
                else
                {
                    m_set_pin(m_sda_pin, true);
                    m_sda_lvl = true;
                }

                if (m_state == I2C_State::ADDRESS && m_bit_count == 8)
                {
                    Send_Packet(I2C_Packet_Type::I2C_ADDRESS, m_shift_reg);
                    m_state = I2C_State::RESPONSE;
                    m_bit_count = 0;
                    wait_for_response = true;
                }
                else if (m_state == I2C_State::WRITE_BYTE && m_bit_count == 8)
                {
                    Send_Packet(I2C_Packet_Type::I2C_WRITE_BYTE, m_shift_reg);
                    m_state = I2C_State::RESPONSE;
                    m_bit_count = 0;
                    wait_for_response = true;
                }
                else if (m_state == I2C_State::READ_BYTE && m_bit_count == 8)
                {
                    m_state = I2C_State::RESPONSE;
                    m_bit_count = 0;
                    m_ack_from_slave = false;
                }
                else if (m_state == I2C_State::RESPONSE)
                {
                    if (m_is_read && m_bit_count == 0)
                    {
                        Send_Packet(I2C_Packet_Type::I2C_ACK, m_sda_lvl ? 0 : 1);
                    }
                    m_state = m_is_read ? I2C_State::READ_BYTE : I2C_State::WRITE_BYTE;
                    m_bit_count = 0;
                    m_shift_reg = 0;
                    if (m_is_read)
                    {
                        Send_Packet(I2C_Packet_Type::I2C_READ_BYTE, 0);
                        wait_for_response = true;
                    }
                }
            }
            m_scl_lvl = is_high;
            if (wait_for_response)
            {
                m_halt();
            }
        }

        void Handle_SDA(bool is_high)
        {
            if (m_scl_lvl)
            {
                if (m_sda_lvl && !is_high)
                { // START
                    while (m_queue_reader.try_advance())
                    {
                        m_queue_reader.advance();
                        m_writer_backoff.wake();
                    }
                    Send_Packet(I2C_Packet_Type::I2C_START, 0);
                    m_state = I2C_State::ADDRESS;
                    m_bit_count = 0;
                    m_shift_reg = 0;
                }
                else if (!m_sda_lvl && is_high)
                { // STOP
                    Send_Packet(I2C_Packet_Type::I2C_STOP, 0);
                    m_state = I2C_State::IDLE;
                }
            }
            m_sda_lvl = is_high;
        }

        void Drive_SDA_From_Queue()
        {
            if (m_queue_reader.try_advance())
            {
                uint8_t val = m_queue_reader.peek();
                m_set_pin(m_sda_pin, static_cast<bool>(val));
                m_sda_lvl = (val != 0);
                m_queue_reader.advance();
                m_writer_backoff.wake();
            }
            else
            {
                m_set_pin(m_sda_pin, true);
                m_sda_lvl = true;
            }
        }

        void Send_Packet(I2C_Packet_Type type, uint8_t value)
        {
            I2C_Packet packet{ .type = type, .value = value };
            std::lock_guard<std::mutex> lock(m_slaves_mutex);
            for (const auto& addr : m_slave_addrs)
            {
                sendto(m_server_fd, &packet, sizeof(packet), 0, (struct sockaddr*)&addr, sizeof(addr));
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
                      zero_mate::IExternal_Peripheral::Halt_t halt,
                      zero_mate::IExternal_Peripheral::Start_t start,
                      zero_mate::utils::CLogging_System* logging_system)
    {
        *peripheral = new (std::nothrow)
        zero_mate::peripheral::CRemote_I2C_Master(name, read_pin, set_pin, halt, start, logging_system);
        return (*peripheral == nullptr) ? zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error
                                        : zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
