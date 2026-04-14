#include <string>
#include <array>
#include <atomic>
#include <cstring>
#include <thread>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>

#include "fmt/format.h"
#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/gpio_server_abi.hpp"
#include "zero_mate/RemoteProtocol.hpp"
#include "CircularBufferQueue.hpp"

namespace
{
    struct UARTPayload
    {
        uint8_t proto;
        uint32_t baud;
        uint32_t net_id;
    } __attribute__((packed));
}

namespace zero_mate::peripheral
{
    class CRemote_UART final : public IExternal_Peripheral
    {
    public:
        static constexpr std::size_t QUEUE_SIZE = 1024;
        static constexpr std::uint32_t Clock_Rate = 250000000;

        enum class UART_State : std::uint8_t
        {
            Idle,
            Start_Bits,
            Data_Bits,
            Stop_Bits
        };

        std::string m_name;
        int m_net_id{ 1 };
        int m_remote_port{ 9000 };
        char m_remote_ip[64]{ 0 };
        int m_fd{ -1 };
        int m_stop_pipe[2]{ -1, -1 };
        bool m_connected{ false };
        struct sockaddr_in m_remote_addr{ };

        int m_tx_pin{ 14 };
        int m_rx_pin{ 15 };

        int m_start_bits{ 1 };
        int m_start_bits_curr{ m_start_bits };

        int m_data_bits{ 8 };
        int m_data_bits_curr{ m_data_bits };

        int m_stop_bits{ 1 };
        int m_stop_bits_curr{ m_stop_bits };

        IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };
        remote_protocol::register_t m_server_register{ nullptr };
        remote_protocol::unregister_t m_server_unregister{ nullptr };
        remote_protocol::disconnect_t m_server_disconnect{ nullptr };
        remote_protocol::init_handshake_t m_server_init_handshake{ nullptr };

        TSP::Queue::Buffer<bool, QUEUE_SIZE> m_rx_buffer_buf;
        TSP::Queue::Reader<bool, QUEUE_SIZE> m_rx_reader;
        TSP::Queue::Writer<bool, QUEUE_SIZE> m_rx_writer;
        TSP::BF::SemBackoff m_rx_backoff;

        std::array<bool, 64> m_tx_buf{ false };
        std::uint8_t m_tx_buf_idx;

        struct UART_Word
        {
            std::uint8_t bit_count{ 0 };
            std::uint32_t word{ 0 };
        } __attribute__((packed));

        // For now just send the words, later on buffer words and send them all at once
        // A word is 64 bit number where first 8 bits signify the number of following bits which are meaningful
        /*
        static constexpr std::size_t WORD_COUNT = 8;
        std::array<UART_Word, WORD_COUNT> m_send_buf;
        std::size_t m_send_buf_idx;
         */

        UART_State m_tx_state{ UART_State::Idle };

        std::uint32_t m_cpu_cycles{ 0 };
        std::uint32_t m_baud_rate{ };
        int m_baudrate_val{ 115200 };

        std::atomic<bool> m_running{ false };
        std::thread m_rx_thread;

        CRemote_UART(const std::string& name,
                     IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                     IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                     utils::CLogging_System* logging_system)
        : m_name{ name }
        , m_read_pin{ read_pin }
        , m_set_pin{ set_pin }
        , m_logging_system{ logging_system }
        , m_tx_buf_idx{ 0 }
        , m_rx_reader(&m_rx_buffer_buf)
        , m_rx_writer(&m_rx_buffer_buf)
        , m_rx_backoff(10, 100)
        {
            if (pipe(m_stop_pipe) == -1)
            {
                if (m_logging_system != nullptr)
                {
                    m_logging_system->Error("Remote UART: Failed to create stop pipe");
                }
            }

            void* proc = LIB_OPEN_SERVER(LIB_NAME("gpio_server"));
            auto get_gs_abi = (TGPIOServerABI (*)())LIB_LOOKUP_SYMBOL(proc, "Get_GPIO_Server_ABI");

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
                m_fd =
                m_server_register("uart", On_Compare_Static, On_Disconnect_Static, On_Handshake_Result_Static, this);
            }

            std::strncpy(m_remote_ip, "127.0.0.1", sizeof(m_remote_ip) - 1);
        }

        ~CRemote_UART() override
        {
            Stop_RX_Thread();

            if (m_fd != -1 && (m_server_unregister != nullptr))
            {
                m_server_unregister(m_fd);
            }

            if (m_stop_pipe[0] != -1)
            {
                close(m_stop_pipe[0]);
                close(m_stop_pipe[1]);
            }
        }

        void ResetState()
        {
            m_tx_state = UART_State::Idle;
            m_tx_buf_idx = 0;
            m_start_bits_curr = m_start_bits;
            m_data_bits_curr = m_data_bits;
            m_stop_bits_curr = m_stop_bits;
            m_cpu_cycles = 0;

            while (m_rx_reader.try_advance())
            {
                m_rx_reader.advance();
            }
            m_rx_backoff.wake();
        }

        void Stop_RX_Thread()
        {
            if (!m_running)
            {
                return;
            }

            m_running = false;
            m_rx_backoff.wake();

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

        void SendWord()
        {
            std::uint32_t word = 0;

            for (std::size_t i = 0; i < m_tx_buf_idx; i++)
            {
                if (i < 32)
                {
                    word |= (m_tx_buf[i] ? 1U : 0U) << i;
                }
            }

            UART_Word u_word = {
                .bit_count = m_tx_buf_idx,
                .word = word,
            };

            send(m_fd, &u_word, sizeof(u_word), 0);

            m_tx_buf_idx = 0;
        }

        void ProcessFromTX()
        {

            const auto bit = m_read_pin(m_tx_pin);

            switch (m_tx_state)
            {
                case UART_State::Idle:
                    if (bit)
                    {
                        m_tx_state = UART_State::Start_Bits;
                    }
                    break;

                case UART_State::Start_Bits:
                    if (!bit)
                    {
                        m_tx_buf[m_tx_buf_idx++] = bit;
                        if (--m_start_bits_curr == 0)
                        {
                            m_start_bits_curr = m_start_bits;
                            m_tx_state = UART_State::Data_Bits;
                        }
                    }
                    else if (m_start_bits_curr < m_start_bits)
                    {
                        // start bits weren't correct
                        // efectively clear the buffer
                        m_start_bits_curr = m_start_bits;
                        m_tx_buf_idx = 0;
                        m_tx_state = UART_State::Idle;
                    }
                    break;

                case UART_State::Data_Bits:
                    m_tx_buf[m_tx_buf_idx++] = bit;

                    if (--m_data_bits_curr == 0)
                    {
                        m_data_bits_curr = m_data_bits;
                        m_tx_state = UART_State::Stop_Bits;
                    }
                    break;

                case UART_State::Stop_Bits:
                    m_tx_buf[m_tx_buf_idx++] = bit;

                    if (--m_stop_bits_curr == 0)
                    {
                        m_stop_bits_curr = m_stop_bits;
                        m_tx_state = UART_State::Idle;
                        SendWord();
                    }
            }
        }

        void DrainToRX()
        {
            if (m_rx_reader.try_advance())
            {
                const auto& bit = static_cast<bool>(m_rx_reader.peek());
                m_rx_reader.advance();
                m_rx_backoff.wake();

                m_set_pin(m_rx_pin, bit);
            }
        }

        void Increment_Passed_Cycles(uint32_t count) override
        {
            m_cpu_cycles += count;

            if (!m_connected)
            {
                return;
            }

            if (m_cpu_cycles >= m_baud_rate)
            {
                m_cpu_cycles = 0;
                ProcessFromTX();
                DrainToRX();
            }
        }

        void Render() override
        {
            if (m_imgui_context)
            {
                ImGui::SetCurrentContext(static_cast<ImGuiContext*>(m_imgui_context));
            }

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

                    ImGui::InputInt("Start Bits", &m_start_bits);
                    ImGui::InputInt("Data Bits", &m_data_bits);
                    ImGui::InputInt("Stop Bits", &m_stop_bits);

                    if (ImGui::Button("Connect"))
                    {
                        ResetState();
                        m_baud_rate = ((Clock_Rate / (uint32_t)m_baudrate_val) / 8) - 1;

                        if (m_fd != -1 && (m_server_init_handshake != nullptr))
                        {
                            const UARTPayload payload = { .proto = 0,
                                                          .baud = (uint32_t)m_baudrate_val,
                                                          .net_id = (uint32_t)m_net_id };
                            m_server_init_handshake(m_fd,
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
                        if (m_fd != -1 && (m_server_disconnect != nullptr))
                        {
                            m_server_disconnect(m_fd);
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
        static bool On_Compare_Static(void* context, const void* payload, size_t size)
        {
            return static_cast<CRemote_UART*>(context)->On_Compare(payload, size);
        }

        bool On_Compare(const void* payload, size_t size) const
        {
            if (size < sizeof(UARTPayload))
            {
                return false;
            }

            if (m_connected)
            {
                return false;
            }

            const auto* protocol = static_cast<const UARTPayload*>(payload);

            if (protocol->proto != 0)
            {
                return false;
            }

            if (protocol->net_id != m_net_id)
            {
                return false;
            }

            return true;
        }

        static void On_Disconnect_Static(void* context, const char* /*remote_ip*/, uint16_t /*remote_port*/)
        {
            auto* uart = static_cast<CRemote_UART*>(context);
            if (uart->m_logging_system != nullptr)
            {
                uart->m_logging_system->Info("Remote UART: Disconnected remotely");
            }
            uart->m_connected = false;
            uart->Stop_RX_Thread();
            uart->ResetState();
        }

        static void
        On_Handshake_Result_Static(void* context, bool success, int fd, const char* remote_ip, uint16_t remote_port)
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
                    m_baud_rate = ((Clock_Rate / (uint32_t)m_baudrate_val) / 8) - 1;

                    if (m_logging_system != nullptr)

                    {
                        m_logging_system->Info(fmt::format("Remote UART: Connected to {}:{} (baud_rate: {})",
                                                           remote_ip,
                                                           remote_port,
                                                           m_baud_rate)
                                               .c_str());
                    }

                    Stop_RX_Thread();
                    m_connected = true;
                    m_running = true;
                    m_rx_thread = std::thread(&CRemote_UART::RX_Thread, this);
                }
                else
                {
                    if (m_logging_system != nullptr)
                    {
                        m_logging_system->Error(
                        fmt::format("Remote UART: Failed to connect to {}:{} (errno {})", remote_ip, remote_port, errno)
                        .c_str());
                    }
                }
            }
        }

        void RX_Thread()
        {
            std::array<std::uint8_t, 1024> buffer{ 0 };
            while (m_running)
            {
                struct pollfd pfds[2];
                pfds[0].fd = m_fd;
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

                const auto received = recv(m_fd, buffer.data(), sizeof(buffer), 0);

                if (received > 0)
                {
                    if (received < (ssize_t)sizeof(UART_Word))
                    {
                        continue;
                    }

                    UART_Word u_word{ };
                    std::memcpy(&u_word, buffer.data(), sizeof(UART_Word));

                    for (std::uint8_t i = 0; i < u_word.bit_count; i++)
                    {
                        const bool bit = (u_word.word & (1U << i)) > 0;
                        if (!m_rx_writer.insert(bit))
                        {
                            break;
                        }
                    }
                }
                else if (received == 0 || (received < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    if (m_logging_system != nullptr)
                    {
                        m_logging_system->Info("Remote UART: Network connection closed");
                    }

                    m_connected = false;
                    break;
                }
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
        *peripheral = new (std::nothrow) zero_mate::peripheral::CRemote_UART(name, read_pin, set_pin, logging_system);
        return (*peripheral == nullptr) ? zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error
                                        : zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
