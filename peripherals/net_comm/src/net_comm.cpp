// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.cpp
/// \author Assistant
/// \brief Implementation of the Remote GPIO peripheral.
// ---------------------------------------------------------------------------------------------------------------------

#include "CircularBufferQueue.hpp"
#include "net_comm.hpp"

#include <cstring>
#include <unistd.h>
#include <sys/socket.h>

// Helper function local to this cpp file
namespace
{

    // constructs a unique id for each connection which is looks like this
    // 0(2bytes) PORT(2bytes) IP(4bytes)
    std::uint64_t get_conn_id(struct sockaddr_in* addr)
    {
        constexpr unsigned int bit_amount = 8;
        constexpr unsigned int shift_amount = sizeof(std::uint32_t) * bit_amount;

        auto ret_val = static_cast<std::uint64_t>(0);
        // this is wrong if the struct isn't 32 bits
        // TODO: add proper conversion to take 1st 4 bytes
        ret_val |= ntohl(std::bit_cast<std::uint32_t>(addr->sin_addr));
        const auto temp = static_cast<std::uint64_t>(ntohs(addr->sin_port));
        ret_val |= (temp << shift_amount);

        return ret_val;
    }

    /*
    std::optional<protocol_info> parse_conf_from_net_msg(const std::vector<std::uint8_t>& msg,
                                                         const struct sockaddr_in& other_side)
    {
        protocol_info prot{};
        std::size_t msg_index{ 0 };
        constexpr std::uint8_t BYTE_WIDTH = 8U;

        if (msg[msg_index++] != MAGIC_BYTE)
        {
            return std::nullopt;
        }
        std::uint16_t opened_port{ 0 };
        opened_port |= static_cast<std::uint16_t>(msg[msg_index++]);
        // why is the warning that this is signed here ??????
        opened_port |= static_cast<std::uint16_t>(msg[msg_index++]) << 8U;

        // use the current address and change the port to the opened port
        struct sockaddr_in other_side_new_port = other_side;
        other_side_new_port.sin_port = htons(opened_port);

        const auto extract_clock_info = [](const std::vector<std::uint8_t>& msg,
                                           std::size_t& index) -> std::tuple<bool, std::uint8_t, std::uint8_t> {
            bool implicit_clock{ false };
            std::uint8_t clock_unit{ 0 };
            std::uint8_t clock_value{ 0 };
            implicit_clock = msg[index++] == 0;
            clock_unit = msg[index++];
            clock_value = msg[index++];

            return { implicit_clock, clock_unit, clock_value };
        };

        const auto protocol_enum_value = static_cast<ProtocolEnum>(msg[msg_index++]);
        std::uint32_t baudrate{ 0 };

        std::tuple<bool, std::uint8_t, std::uint8_t> clock_info;
        bool implicit_clock{ false };
        std::uint8_t clock_unit{ 0 };
        std::uint8_t clock_value{ 0 };

        switch (protocol_enum_value)
        {
            case UART:
                baudrate |= static_cast<std::uint32_t>(msg_index++);
                baudrate |= static_cast<std::uint32_t>(msg_index++) << BYTE_WIDTH;
                baudrate |= static_cast<std::uint32_t>(msg_index++) << BYTE_WIDTH * 2U;
                baudrate |= static_cast<std::uint32_t>(msg_index++) << BYTE_WIDTH * 3U;

                clock_info = extract_clock_info(msg, msg_index);

                prot.info = UART_P{ .baudrate = baudrate, .other_side = other_side_new_port };
                break;

            case I2C:
                // For now assuming it's a master if we are parsing config
                prot.info = I2C_Master_P{ .slave_fds = {} };
                break;

            case SPI:
                prot.info = SPI_P{ .slaves = { other_side_new_port } };
                break;

            case GeneralBuffered:
                prot.info = GeneralBuffered_P{ .other_side = other_side_new_port };
                break;

            case GeneralUnbuffered:
                prot.info = GeneralUnbuffered_P{ .other_side = other_side_new_port };
                break;

            default:
                return std::nullopt;
                break;
        }

        return prot;
    }
    */
}

void GPIOServer::pin_write()
{
    // tries to read from the queue
    // if nothing comes for a long enough time, start sleeping on a semaphore
    while (m_running.load())
    {
        const auto [pin, value] = pin_write_queue_reader.read(backoff_sem);
        if (!m_running.load())
        {
            break;
        }
        func_set_pin(static_cast<std::uint32_t>(pin), value > 0);
    }
}

void GPIOServer::write_to_pin(const std::uint8_t pin, const std::uint8_t value)
{
    pin_write_spinlock.lock();
    pin_write_queue_writer.insert_with_backoff({ pin, value }, backoff_fast);
    // if the time before insertion was too long, wake up the reader
    backoff_sem.wake();
    pin_write_spinlock.unlock();
}

void GPIOServer::route_pin_info(const pin_pair pin_info)
{
    const auto& [pin, value] = pin_info;
    const auto conn_id_idx = pin_to_conn_id.get(pin);

    if (conn_id_idx == UINT8_MAX)
    {
        return;
    }

    auto& writer_ref = out_queue_writers[conn_id_idx];
    while (!writer_ref.try_insert())
    {
        backoff_fast.wait();
    }
    backoff_fast.reset();

    writer_ref.insert(pin_info);
}

GPIOServer::GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin,
                       zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t func_read_pin,
                       zero_mate::IExternal_Peripheral::Halt_t func_halt,
                       zero_mate::IExternal_Peripheral::Start_t func_start)
: pin_write_queue_reader(&pin_write_queue_buf)
, pin_write_queue_writer(&pin_write_queue_buf)
, func_set_pin(func_set_pin)
, func_read_pin(func_read_pin)
, func_halt(func_halt)
, func_start(func_start)
{
    for (std::size_t i = 0; i < out_queue_buffers.size(); i++)
    {
        std::construct_at(&out_queue_writers[i], &out_queue_buffers[i]);
        connection_running[i] = std::make_unique<std::atomic<bool>>(false);
    }
}

void GPIOServer::unmap_connection(std::size_t i)
{
    std::visit(
    [this](auto& p) {
        using T = std::remove_cvref_t<decltype(p)>;
        if constexpr (std::is_same_v<T, UART_P>)
        {
            pin_to_conn_id.set(static_cast<std::uint8_t>(p.tx_pin), UINT8_MAX);
        }
        else if constexpr (std::is_same_v<T, I2C_Master_P> || std::is_same_v<T, I2C_Slave_P>)
        {
            pin_to_conn_id.set(static_cast<std::uint8_t>(p.scl_pin), UINT8_MAX);
            pin_to_conn_id.set(static_cast<std::uint8_t>(p.sda_pin), UINT8_MAX);
        }
    },
    connection_data[i].protocol);
}

void GPIOServer::remove_connection(std::size_t i)
{
    if (i >= MAX_CONNECTION_COUNT || !connection_bit_map[i])
    {
        return;
    }

    connection_running[i]->store(false);
    if (connection_threads[i].joinable())
    {
        connection_threads[i].join();
    }

    unmap_connection(i);
    connection_bit_map[i] = false;
}

void GPIOServer::construct_connection(const conn_info& info)
{
    for (std::size_t i = 0; i < MAX_CONNECTION_COUNT; i++)
    {
        if (!connection_bit_map[i])
        {
            connection_bit_map[i] = true;
            connection_data[i] = info;
            connection_running[i]->store(true);

            // Map pins
            std::visit(
            [this, i](auto& p) {
                using T = std::remove_cvref_t<decltype(p)>;
                if constexpr (std::is_same_v<T, UART_P>)
                {
                    pin_to_conn_id.set(static_cast<std::uint8_t>(p.tx_pin), static_cast<std::uint8_t>(i));
                }
                else if constexpr (std::is_same_v<T, I2C_Master_P> || std::is_same_v<T, I2C_Slave_P>)
                {
                    pin_to_conn_id.set(static_cast<std::uint8_t>(p.scl_pin), static_cast<std::uint8_t>(i));
                    pin_to_conn_id.set(static_cast<std::uint8_t>(p.sda_pin), static_cast<std::uint8_t>(i));
                }
            },
            connection_data[i].protocol);

            connection_threads[i] = std::thread([this, i]() {
                GPIOConnection conn(connection_data[i],
                                    &out_queue_buffers[i],
                                    func_halt,
                                    func_start,
                                    func_read_pin,
                                    *this,
                                    m_running,
                                    *connection_running[i]);
                conn.run();
            });
            return;
        }
    }
}

void GPIOServer::run()
{
    m_pin_write_thread = std::thread([this]() {
        while (m_running.load())
        {
            pin_write();
        }
    });

    // TODO: Main discovery/config loop (UDP recvfrom)
    while (m_running.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void GPIOServer::stop()
{
    m_running.store(false);
    backoff_sem.wake();
    if (m_pin_write_thread.joinable())
    {
        m_pin_write_thread.join();
    }
    for (auto& thread : connection_threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }
}

GPIOServer::~GPIOServer()
{
    stop();
}

GPIOConnection::GPIOConnection(conn_info& info,
                               TSP::Queue::Buffer<pin_pair, GPIOServer::BUFFER_SIZE>* buffer,
                               zero_mate::IExternal_Peripheral::Halt_t halt,
                               zero_mate::IExternal_Peripheral::Start_t start,
                               zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                               GPIOServer& server,
                               std::atomic<bool>& server_running,
                               std::atomic<bool>& connection_running)
: connection(info)
, m_queue_reader(buffer)
, m_socket(-1)
, m_halt(halt)
, m_start(start)
, m_read_pin(read_pin)
, m_server(server)
, m_server_running(server_running)
, m_connection_running(connection_running)
{
    m_socket = socket(AF_INET, SOCK_DGRAM, 0);
    // TODO: bind if needed, but user said handshake later

    auto pin_write_cb = [this](std::uint8_t pin, std::uint8_t val) { this->write_to_pin(pin, val); };
    auto pin_read_cb = [this](std::uint8_t pin) { return static_cast<uint8_t>(m_read_pin(pin)); };

    auto local_info = info;

    if (std::holds_alternative<UART_P>(local_info.protocol))
    {
        auto& uart_p = std::get<UART_P>(local_info.protocol);
        uart_p.other_side_fd = m_socket;
        m_processor = std::make_unique<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE>>(
        std::make_unique<UART_Handler<GPIOServer::BUFFER_SIZE>>(uart_p, pin_write_cb),
        buffer);
    }
    else if (std::holds_alternative<I2C_Master_P>(local_info.protocol))
    {
        auto& i2c_m = std::get<I2C_Master_P>(local_info.protocol);
        // For I2C Master we might need multiple FDs, but for now use the main one if empty
        if (i2c_m.slave_fds.empty() && m_socket != -1)
        {
            i2c_m.slave_fds.push_back(m_socket);
        }
        m_processor = std::make_unique<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE>>(
        std::make_unique<I2C_Master<GPIOServer::BUFFER_SIZE>>(i2c_m, m_halt, m_start, pin_write_cb, pin_read_cb),
        buffer);
    }
    else if (std::holds_alternative<I2C_Slave_P>(local_info.protocol))
    {
        auto& i2c_s = std::get<I2C_Slave_P>(local_info.protocol);
        i2c_s.master_fd = m_socket;
        m_processor = std::make_unique<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE>>(
        std::make_unique<I2C_Slave<GPIOServer::BUFFER_SIZE>>(i2c_s, m_halt, m_start, pin_write_cb, pin_read_cb),
        buffer);
    }
    connection = local_info;
}

GPIOConnection::~GPIOConnection()
{
    if (m_socket != -1)
    {
        close(m_socket);
    }
}

void GPIOConnection::run()
{
    if (m_processor)
    {
        m_processor->start();
        while (m_server_running.load() && m_connection_running.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        m_processor->stop();
    }
}

pin_pair GPIOConnection::read_queue()
{
    TSP::BF::Backoff backoff(100);
    return m_queue_reader.read(backoff);
}

void GPIOConnection::send_to_network(const std::vector<std::uint8_t>& data)
{
    if (m_socket != -1)
    {
        sendto(m_socket,
               data.data(),
               data.size(),
               0,
               reinterpret_cast<struct sockaddr*>(&m_other_side),
               sizeof(m_other_side));
    }
}

void GPIOConnection::write_to_pin(std::uint8_t pin, std::uint8_t value)
{
    m_server.write_to_pin(pin, value);
}

Remote_GPIO::Remote_GPIO(const std::string& name,
                         const std::vector<std::uint32_t>& pins,
                         zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                         zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                         zero_mate::IExternal_Peripheral::Halt_t halt,
                         zero_mate::IExternal_Peripheral::Start_t start,
                         zero_mate::utils::CLogging_System* logging_system)
: name(name)
, pins(pins)
, read_pin(read_pin)
, set_pin(set_pin)
, halt(halt)
, start(start)
, logging_system(logging_system)
, ImGui_context(nullptr)
, server(set_pin, read_pin, halt, start)
, ui_selected_local_pin_idx(0)
, ui_target_net_pin(0)
, ui_selected_net_pin_source(0)
, ui_target_local_pin_idx(0)
{
    // Subscribe to all pins passed in construction
    for (const auto& pin : pins)
    {
        m_gpio_subscription.insert(pin);
    }

    server_thread = std::thread(&GPIOServer::run, &server);
}

Remote_GPIO::~Remote_GPIO()
{
    m_running.store(false);
    server.stop();
    server_thread.join();
}

void Remote_GPIO::Set_ImGui_Context(void* context)
{
    ImGui_context = static_cast<ImGuiContext*>(context);
}

void Remote_GPIO::Increment_Passed_Cycles(std::uint32_t count)
{
}

void Remote_GPIO::GPIO_Subscription_Callback(std::uint32_t pin_idx)
{
    const auto val = static_cast<std::uint8_t>(read_pin(pin_idx));
    const auto pin = static_cast<std::uint8_t>(pin_idx);
    server.route_pin_info({ pin, val });
}

// === Rendering Bellow ===

void Remote_GPIO::Render()
{
    assert(ImGui_context != nullptr);
    ImGui::SetCurrentContext(ImGui_context);

    if (ImGui::Begin(name.c_str()))
    {
        Render_Settings();
        ImGui::Separator();
        Render_Mappings();
    }
    ImGui::End();
}

void Remote_GPIO::Render_Settings()
{
    if (ImGui::Button("Add Connection"))
    {
        ImGui::OpenPopup("Add Connection");
    }

    ImGui::Separator();
    ImGui::Text("Active Connections:");

    const auto& bit_map = server.get_connection_bit_map();
    const auto& data = server.get_connection_data();

    for (std::size_t i = 0; i < bit_map.size(); ++i)
    {
        if (bit_map[i])
        {
            std::string label = "Connection " + std::to_string(i) + ": ";
            std::visit(
            [&label](auto& p) {
                using T = std::remove_cvref_t<decltype(p)>;
                if constexpr (std::is_same_v<T, UART_P>)
                {
                    label += "UART";
                }
                else if constexpr (std::is_same_v<T, I2C_Master_P>)
                {
                    label += "I2C Master";
                }
                else if constexpr (std::is_same_v<T, I2C_Slave_P>)
                {
                    label += "I2C Slave";
                }
            },
            data[i].protocol);

            if (ImGui::Selectable(label.c_str(), false, 0, ImVec2(ImGui::GetWindowWidth() - 50, 0)))
            {
                m_ui_view_idx = static_cast<int>(i);
                ImGui::OpenPopup("View Connection");
            }

            ImGui::SameLine(ImGui::GetWindowWidth() - 35);
            if (ImGui::Button(("x##" + std::to_string(i)).c_str()))
            {
                server.remove_connection(i);
            }
        }
    }

    // Modal: Add Connection
    if (ImGui::BeginPopupModal("Add Connection", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Combo("Protocol", &m_ui_add_state.protocol_type, "UART\0I2C Master\0I2C Slave\0\0");

        if (m_ui_add_state.protocol_type == 0) // UART
        {
            ImGui::InputInt("Baudrate", &m_ui_add_state.baudrate);
            ImGui::InputInt("TX Pin", &m_ui_add_state.tx_pin);
            ImGui::InputInt("RX Pin", &m_ui_add_state.rx_pin);
            ImGui::InputInt("Start Bits", &m_ui_add_state.start_bits);
            ImGui::InputInt("Data Bits", &m_ui_add_state.data_bits);
            ImGui::InputInt("Parity Bits", &m_ui_add_state.parity_bits);
            ImGui::InputInt("Stop Bits", &m_ui_add_state.stop_bits);
            ImGui::InputText("Remote IP", m_ui_add_state.ip, sizeof(m_ui_add_state.ip));
            ImGui::InputInt("Remote Port", &m_ui_add_state.port);
        }
        else if (m_ui_add_state.protocol_type == 1) // I2C Master
        {
            ImGui::InputInt("Bus ID", &m_ui_add_state.i2c_id);
            ImGui::InputInt("SCL Pin", &m_ui_add_state.scl_pin);
            ImGui::InputInt("SDA Pin", &m_ui_add_state.sda_pin);
        }
        else if (m_ui_add_state.protocol_type == 2) // I2C Slave
        {
            ImGui::InputInt("Bus ID", &m_ui_add_state.i2c_id);
            ImGui::InputInt("Address", &m_ui_add_state.address);
            ImGui::InputInt("SCL Pin", &m_ui_add_state.scl_pin);
            ImGui::InputInt("SDA Pin", &m_ui_add_state.sda_pin);
        }

        if (ImGui::Button("OK", ImVec2(120, 0)))
        {
            conn_info info{};
            if (m_ui_add_state.protocol_type == 0)
            {
                UART_P p{};
                p.baudrate = m_ui_add_state.baudrate;
                p.tx_pin = m_ui_add_state.tx_pin;
                p.rx_pin = m_ui_add_state.rx_pin;
                p.start_bits = m_ui_add_state.start_bits;
                p.data_bits = m_ui_add_state.data_bits;
                p.parity_bits = m_ui_add_state.parity_bits;
                p.stop_bits = m_ui_add_state.stop_bits;
                p.other_side.sin_family = AF_INET;
                p.other_side.sin_port = htons(static_cast<uint16_t>(m_ui_add_state.port));
                inet_pton(AF_INET, m_ui_add_state.ip, &p.other_side.sin_addr);
                info.protocol = p;
            }
            else if (m_ui_add_state.protocol_type == 1)
            {
                I2C_Master_P p{};
                p.id = static_cast<uint32_t>(m_ui_add_state.i2c_id);
                p.scl_pin = m_ui_add_state.scl_pin;
                p.sda_pin = m_ui_add_state.sda_pin;
                info.protocol = p;
            }
            else if (m_ui_add_state.protocol_type == 2)
            {
                I2C_Slave_P p{};
                p.id = static_cast<uint32_t>(m_ui_add_state.i2c_id);
                p.address = static_cast<uint8_t>(m_ui_add_state.address);
                p.scl_pin = m_ui_add_state.scl_pin;
                p.sda_pin = m_ui_add_state.sda_pin;
                info.protocol = p;
            }

            server.construct_connection(info);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SetItemDefaultFocus();
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0)))
        {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Modal: View Connection
    if (ImGui::BeginPopupModal("View Connection", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        if (m_ui_view_idx != -1 && bit_map[m_ui_view_idx])
        {
            const auto& conn = data[m_ui_view_idx];
            std::visit(
            [](auto& p) {
                using T = std::remove_cvref_t<decltype(p)>;
                if constexpr (std::is_same_v<T, UART_P>)
                {
                    ImGui::Text("Protocol: UART");
                    ImGui::Text("Baudrate: %u", p.baudrate);
                    ImGui::Text("TX Pin: %u", p.tx_pin);
                    ImGui::Text("RX Pin: %u", p.rx_pin);
                    ImGui::Text("Start Bits: %u", p.start_bits);
                    ImGui::Text("Data Bits: %u", p.data_bits);
                    ImGui::Text("Parity Bits: %u", p.parity_bits);
                    ImGui::Text("Stop Bits: %u", p.stop_bits);
                    ImGui::Text("Remote IP: %s", inet_ntoa(p.other_side.sin_addr));
                    ImGui::Text("Remote Port: %u", ntohs(p.other_side.sin_port));
                }
                else if constexpr (std::is_same_v<T, I2C_Master_P>)
                {
                    ImGui::Text("Protocol: I2C Master");
                    ImGui::Text("Bus ID: %u", p.id);
                    ImGui::Text("SCL Pin: %u", p.scl_pin);
                    ImGui::Text("SDA Pin: %u", p.sda_pin);
                }
                else if constexpr (std::is_same_v<T, I2C_Slave_P>)
                {
                    ImGui::Text("Protocol: I2C Slave");
                    ImGui::Text("Bus ID: %u", p.id);
                    ImGui::Text("Address: 0x%02X", p.address);
                    ImGui::Text("SCL Pin: %u", p.scl_pin);
                    ImGui::Text("SDA Pin: %u", p.sda_pin);
                }
            },
            conn.protocol);
        }

        if (ImGui::Button("Close", ImVec2(120, 0)))
        {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

void Remote_GPIO::Render_Mappings()
{
}

extern "C"
{
    zero_mate::IExternal_Peripheral::NInit_Status
    Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
                      const char* const name,
                      const std::uint32_t* const connection,
                      std::size_t pin_count,
                      zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                      zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                      zero_mate::IExternal_Peripheral::Halt_t halt,
                      zero_mate::IExternal_Peripheral::Start_t start,
                      zero_mate::utils::CLogging_System* logging_system)
    {
        std::vector<std::uint32_t> pins;
        for (std::size_t i = 0; i < pin_count; ++i)
        {
            pins.push_back(connection[i]);
        }

        *peripheral = new (std::nothrow) Remote_GPIO(name, pins, read_pin, set_pin, halt, start, logging_system);

        if (*peripheral == nullptr)
        {
            return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
        }

        return zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
