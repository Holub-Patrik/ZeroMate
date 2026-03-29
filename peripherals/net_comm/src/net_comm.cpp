// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.cpp
/// \author Assistant
/// \brief Implementation of the Remote GPIO peripheral.
// ---------------------------------------------------------------------------------------------------------------------

#include <cstring>
#include <unistd.h>
#include <sys/socket.h>

#include "CircularBufferQueue.hpp"
#include "net_comm.hpp"

namespace
{
    template<class... Ts>
    struct overloads : Ts...
    {
        using Ts::operator()...;
    };
}

void GPIOServer::pin_write(const std::stop_token& stop_token)
{
    // tries to read from the queue
    // if nothing comes for a long enough time, start sleeping on a semaphore
    while (!stop_token.stop_requested())
    {
        if (!pin_write_queue_reader.try_advance())
        {
            pin_write_reader_backoff.wait(
            [this, &stop_token]() { return stop_token.stop_requested() || pin_write_queue_reader.try_advance(); });
            continue;
        }
        pin_write_reader_backoff.reset();

        const auto pair = pin_write_queue_reader.peek();
        pin_write_queue_reader.advance();
        pin_write_writer_backoff.wake();

        func_set_pin(static_cast<std::uint32_t>(pair.first), pair.second > 0);
    }
}

void GPIOServer::write_to_pin(const std::uint8_t pin, const std::uint8_t value)
{
    pin_write_spinlock.lock();
    pin_write_queue_writer.insert_with_backoff({ pin, value }, pin_write_writer_backoff);
    // if the time before insertion was too long, wake up the reader
    pin_write_reader_backoff.wake();
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
    auto& backoffs = m_backoffs[conn_id_idx];

    writer_ref.insert_with_backoff(pin_info, backoffs.out_queue_writer);
    backoffs.out_queue_reader.wake();
}

void GPIOServer::add_connection(const conn_info& info)
{
    const auto index = find_free_index();
    if (index != UINT8_MAX)
    {
        connection_bit_map[index] = true;
        connection_data[index] = info;
        connection_data[index].status = ConnectionStatus::Defined;
        connection_data[index].sockfd = -1;
        m_net_id_to_idx[info.net_id] = index;

        if (std::holds_alternative<I2C_Master_P>(info.protocol))
        {
            const auto& p = std::get<I2C_Master_P>(info.protocol);
            m_bus_id_to_idx[p.bus_id] = index;
            connection_data[index].status = ConnectionStatus::Connected;
            construct_connection(connection_data[index]);
        }
    }
}

GPIOServer::GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin,
                       zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t func_read_pin,
                       zero_mate::IExternal_Peripheral::Halt_t func_halt,
                       zero_mate::IExternal_Peripheral::Start_t func_start,
                       zero_mate::utils::CLogging_System* logging_system,
                       const std::atomic<std::uint64_t>* total_cycles)
: pin_write_queue_reader(&pin_write_queue_buf)
, pin_write_queue_writer(&pin_write_queue_buf)
, func_set_pin(func_set_pin)
, func_read_pin(func_read_pin)
, func_halt(func_halt)
, func_start(func_start)
, logging_system(logging_system)
, m_total_cycles(total_cycles)
{
    for (std::size_t i = 0; i < out_queue_buffers.size(); i++)
    {
        std::construct_at(&out_queue_writers[i], &out_queue_buffers[i]);
        connection_running[i].store(false);
    }
}

void GPIOServer::Init(uint16_t handshake_port)
{
    if (m_handshake_socket != -1)
    {
        return;
    }

    m_handshake_port = handshake_port;
    m_handshake_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_handshake_socket != -1)
    {
        struct sockaddr_in addr{ };
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(m_handshake_port);
        if (bind(m_handshake_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            if (logging_system)
            {
                std::string log_msg = "Handshake port " + std::to_string(m_handshake_port) +
                                      " is already in use. Failed to start GPIOServer.";
                logging_system->Error(log_msg.c_str());
            }
            close(m_handshake_socket);
            m_handshake_socket = -1;
        }
        else if (logging_system)
        {
            std::string log_msg = "Handshake socket listening on port " + std::to_string(m_handshake_port);
            logging_system->Info(log_msg.c_str());
        }
    }
}

bool GPIOServer::Is_Initialized() const noexcept
{
    return m_handshake_socket != -1;
}

void GPIOServer::unmap_connection(const std::size_t conn_index)
{
    const auto unmap_visitor =
    overloads{ [this](const UART_P& protocol) -> void {
                  pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.tx_pin), UINT8_MAX);
              },
               [this](const I2C_Master_P& protocol) -> void {
                   pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.scl_pin), UINT8_MAX);
                   pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.sda_pin), UINT8_MAX);
                   m_bus_id_to_idx.erase(protocol.bus_id);
               },
               [this](const I2C_Slave_P& protocol) -> void {
                   pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.sda_pin), UINT8_MAX);
               } };

    std::visit(unmap_visitor, connection_data[conn_index].protocol);
}

void GPIOServer::remove_connection(std::size_t i)
{
    if (i >= MAX_CONNECTION_COUNT || !connection_bit_map[i])
    {
        return;
    }

    if (connection_data[i].status == ConnectionStatus::Connected)
    {
        handshake::DisconnectMessage d_msg{ };
        const auto& info = connection_data[i];

        d_msg.config.net_id = info.net_id;
        d_msg.config.type = handshake::MessageType::Conf;
        d_msg.config.port = info.opened_port;

        const auto visitor =
        overloads{ [&d_msg](const UART_P& protocol) {
                      d_msg.config.protocol_id = handshake::ProtocolID::UART;
                      d_msg.config.config.uart.baudrate = protocol.baudrate;
                      d_msg.config.config.uart.data_bits = static_cast<uint8_t>(protocol.data_bits);
                      d_msg.config.config.uart.start_bits = static_cast<uint8_t>(protocol.start_bits);
                      d_msg.config.config.uart.parity_bits = static_cast<uint8_t>(protocol.parity_bits);
                      d_msg.config.config.uart.stop_bits = static_cast<uint8_t>(protocol.stop_bits);
                  },
                   [&d_msg](const I2C_Master_P& p) {
                       d_msg.config.protocol_id = handshake::ProtocolID::I2C;
                       d_msg.config.config.i2c.bus_id = p.bus_id;
                       d_msg.config.config.i2c.is_master = 1;
                       d_msg.config.config.i2c.address = 0;
                   },
                   [&d_msg](const I2C_Slave_P& p) {
                       d_msg.config.protocol_id = handshake::ProtocolID::I2C;
                       d_msg.config.config.i2c.bus_id = p.bus_id;
                       d_msg.config.config.i2c.is_master = 0;
                       d_msg.config.config.i2c.address = p.address;
                   } };
        std::visit(visitor, info.protocol);

        struct sockaddr_in remote_addr{ };
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_port = htons(static_cast<uint16_t>(connection_data[i].remote_port));
        inet_pton(AF_INET, connection_data[i].remote_ip.c_str(), &remote_addr.sin_addr);

        sendto(m_handshake_socket,
               &d_msg,
               sizeof(d_msg),
               0,
               reinterpret_cast<const struct sockaddr*>(&remote_addr),
               sizeof(remote_addr));
    }

    connection_running[i].store(false);
    connection_threads[i].request_stop();
    m_backoffs[i].out_queue_reader.wake();

    if (connection_threads[i].joinable())
    {
        connection_threads[i].join();
    }
    active_connections[i].reset();

    if (connection_data[i].sockfd != -1)
    {
        close(connection_data[i].sockfd);
        connection_data[i].sockfd = -1;
    }

    unmap_connection(i);
    m_net_id_to_idx.erase(connection_data[i].net_id);
    connection_bit_map[i] = false;
}

std::uint8_t GPIOServer::find_free_index() const noexcept
{
    std::uint8_t index = 0;
    for (const auto& position : connection_bit_map)
    {
        if (!position)
        {
            return index;
        }
        index++;
    }

    return UINT8_MAX;
}

void GPIOServer::construct_connection(const conn_info& info)
{
    if (!m_net_id_to_idx.contains(info.net_id))
    {
        return;
    }

    const auto idx = m_net_id_to_idx.at(info.net_id);

    const auto construct_visitor = overloads{
        [this, idx](const UART_P& protocol) -> void {
            pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.tx_pin), static_cast<std::uint8_t>(idx));
        },
        [this, idx](const I2C_Master_P& protocol) -> void {
            pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.scl_pin), static_cast<std::uint8_t>(idx));
            pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.sda_pin), static_cast<std::uint8_t>(idx));
        },
        [this, idx](const I2C_Slave_P& protocol) -> void {
            pin_to_conn_id.set(static_cast<std::uint8_t>(protocol.sda_pin), static_cast<std::uint8_t>(idx));
        },
    };

    if (connection_data[idx].status == ConnectionStatus::Connected)
    {
        connection_running[idx].store(true);
        std::visit(construct_visitor, connection_data[idx].protocol);

        active_connections[idx] = std::make_unique<GPIOConnection>(idx, *this);

        connection_threads[idx] =
        std::jthread([this, idx](std::stop_token stop_token) { active_connections[idx]->run(stop_token); });
    }
}

void GPIOServer::add_slave_to_master(std::uint32_t bus_id, int fd, std::uint32_t slave_id)
{
    if (m_bus_id_to_idx.contains(bus_id))
    {
        const auto idx = m_bus_id_to_idx.at(bus_id);
        if (active_connections[idx])
        {
            active_connections[idx]->add_slave(fd, slave_id);
        }
    }
}

void GPIOServer::remove_slave_from_master(std::uint32_t slave_id)
{
    for (std::size_t i = 0; i < MAX_CONNECTION_COUNT; ++i)
    {
        if (connection_bit_map[i] && std::holds_alternative<I2C_Master_P>(connection_data[i].protocol))
        {
            if (active_connections[i])
            {
                active_connections[i]->remove_slave(slave_id);
            }
        }
    }
}

std::size_t GPIOServer::get_slave_count(std::size_t idx) const
{
    if (idx < MAX_CONNECTION_COUNT && active_connections[idx])
    {
        return active_connections[idx]->get_slave_count();
    }
    return 0;
}

void GPIOServer::initiate_handshake(std::size_t idx)
{
    if (m_handshake_socket == -1 || idx >= MAX_CONNECTION_COUNT || !connection_bit_map[idx])
    {
        return;
    }

    auto& info = connection_data[idx];
    handshake::ConfMessage msg{ };
    msg.net_id = info.net_id;
    msg.type = handshake::MessageType::Conf;

    // Port handling
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr{ };
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = 0;
    bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    getsockname(sock, reinterpret_cast<struct sockaddr*>(&addr), &len);
    msg.port = ntohs(addr.sin_port);

    const auto visitor = overloads{ [&msg](const UART_P& protocol) {
                                       msg.protocol_id = handshake::ProtocolID::UART;
                                       msg.config.uart.baudrate = protocol.baudrate;
                                       msg.config.uart.data_bits = static_cast<uint8_t>(protocol.data_bits);
                                       msg.config.uart.start_bits = static_cast<uint8_t>(protocol.start_bits);
                                       msg.config.uart.parity_bits = static_cast<uint8_t>(protocol.parity_bits);
                                       msg.config.uart.stop_bits = static_cast<uint8_t>(protocol.stop_bits);
                                   },
                                    [&msg, &info](const I2C_Master_P& p) {
                                        msg.protocol_id = handshake::ProtocolID::I2C;
                                        msg.config.i2c.bus_id = p.bus_id;
                                        msg.config.i2c.is_master = 1;
                                        msg.config.i2c.address = 0;
                                        msg.net_id = info.net_id;
                                    },
                                    [&msg, &info](const I2C_Slave_P& p) {
                                        msg.protocol_id = handshake::ProtocolID::I2C;
                                        msg.config.i2c.bus_id = p.bus_id;
                                        msg.config.i2c.is_master = 0;
                                        msg.config.i2c.address = p.address;
                                        msg.net_id = info.net_id;
                                    } };

    std::visit(visitor, info.protocol);

    struct sockaddr_in remote_addr{ };
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(static_cast<uint16_t>(info.remote_port));
    inet_pton(AF_INET, info.remote_ip.c_str(), &remote_addr.sin_addr);

    sendto(m_handshake_socket,
           &msg,
           sizeof(msg),
           0,
           reinterpret_cast<struct sockaddr*>(&remote_addr),
           sizeof(remote_addr));

    // Update connection state
    auto& conn = connection_data[idx];
    conn.status = ConnectionStatus::Connecting;
    conn.opened_port = msg.port;
    conn.is_responder = false;
    conn.start_time = std::chrono::steady_clock::now();
    conn.sockfd = sock;
}

void GPIOServer::handle_handshake()
{
    if (m_handshake_socket == -1)
    {
        return;
    }

    struct pollfd pfd = { .fd = m_handshake_socket, .events = POLLIN, .revents = 0 };
    if (poll(&pfd, 1, 10) > 0)
    {
        uint8_t buf[512];
        struct sockaddr_in addr{ };
        socklen_t addr_len = sizeof(addr);
        const auto bytes_received =
        recvfrom(m_handshake_socket, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr*>(&addr), &addr_len);
        if (bytes_received > 0 && buf[0] == handshake::MAGIC_BYTE)
        {
            const auto* msg = reinterpret_cast<const handshake::ConfMessage*>(buf);

            switch (msg->type)
            {
                case handshake::MessageType::Conf:
                    if (bytes_received == sizeof(handshake::ConfMessage))
                    {
                        handle_conf_msg(*msg, addr);
                    }
                    break;

                case handshake::MessageType::Response:
                    if (bytes_received == sizeof(handshake::ResponseMessage))
                    {
                        handle_response_msg(*reinterpret_cast<const handshake::ResponseMessage*>(buf), addr);
                    }
                    break;

                case handshake::MessageType::FinalResponse:
                    if (bytes_received == sizeof(handshake::FinalResponseMessage))
                    {
                        handle_final_response_msg(*reinterpret_cast<const handshake::FinalResponseMessage*>(buf), addr);
                    }
                    break;

                case handshake::MessageType::Disconnect:
                    if (bytes_received == sizeof(handshake::DisconnectMessage))
                    {
                        handle_disconnect_msg(*reinterpret_cast<const handshake::DisconnectMessage*>(buf), addr);
                    }
                    break;

                default:
                    if (logging_system)
                    {
                        logging_system->Warning("Received unknown handshake message type.");
                    }
                    break;
            }
        }
    }
}

void GPIOServer::handle_conf_msg(const handshake::ConfMessage& msg, const struct sockaddr_in& addr)
{
    // Responder side
    handshake::ResponseMessage resp{ };
    resp.status = 0; // Decline by default
    resp.net_id = msg.net_id;

    int found_idx = -1;

    if (msg.protocol_id == handshake::ProtocolID::I2C)
    {
        if (msg.config.i2c.is_master)
        {
            if (logging_system)
            {
                logging_system->Warning("Received I2C Master handshake. Master-to-Master not supported.");
            }
            sendto(m_handshake_socket,
                   &resp,
                   sizeof(resp),
                   0,
                   reinterpret_cast<const struct sockaddr*>(&addr),
                   sizeof(addr));
            return;
        }

        if (m_bus_id_to_idx.contains(msg.config.i2c.bus_id))
        {
            const auto idx = m_bus_id_to_idx.at(msg.config.i2c.bus_id);
            if (connection_bit_map[idx] && std::holds_alternative<I2C_Master_P>(connection_data[idx].protocol))
            {
                found_idx = static_cast<int>(idx);
            }
        }
    }
    else if (msg.protocol_id == handshake::ProtocolID::UART)
    {
        for (std::size_t i = 0; i < MAX_CONNECTION_COUNT; ++i)
        {
            if (connection_bit_map[i] && connection_data[i].net_id == msg.net_id &&
                std::holds_alternative<UART_P>(connection_data[i].protocol))
            {
                found_idx = static_cast<int>(i);
                break;
            }
        }
    }

    if (found_idx == -1)
    {
        if (logging_system)
        {
            std::string log_msg = "Received handshake for unknown Connection/Bus ID: " + std::to_string(msg.net_id);
            logging_system->Warning(log_msg.c_str());
        }
        sendto(m_handshake_socket,
               &resp,
               sizeof(resp),
               0,
               reinterpret_cast<const struct sockaddr*>(&addr),
               sizeof(addr));
        return;
    }

    resp.status = 1; // Accept

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in temp_addr{ };
    temp_addr.sin_family = AF_INET;
    temp_addr.sin_addr.s_addr = INADDR_ANY;
    temp_addr.sin_port = 0;
    bind(sock, reinterpret_cast<struct sockaddr*>(&temp_addr), sizeof(temp_addr));
    socklen_t len = sizeof(temp_addr);
    getsockname(sock, reinterpret_cast<struct sockaddr*>(&temp_addr), &len);
    const uint16_t data_port = ntohs(temp_addr.sin_port);

    // Connect the socket to the remote data port
    struct sockaddr_in remote_data_addr = addr;
    remote_data_addr.sin_port = htons(msg.port);
    connect(sock, reinterpret_cast<const struct sockaddr*>(&remote_data_addr), sizeof(remote_data_addr));

    resp.port = data_port;
    sendto(m_handshake_socket, &resp, sizeof(resp), 0, reinterpret_cast<const struct sockaddr*>(&addr), sizeof(addr));

    if (msg.protocol_id == handshake::ProtocolID::I2C)
    {
        add_slave_to_master(msg.config.i2c.bus_id, sock, msg.net_id);

        if (logging_system)
        {
            std::string log_msg = "Added remote I2C Slave (Port " + std::to_string(msg.port) + ", Address: 0x" +
                                  std::to_string(msg.config.i2c.address) + ", Slave ID: " + std::to_string(msg.net_id) +
                                  ") to Bus ID: " + std::to_string(msg.config.i2c.bus_id);
            logging_system->Info(log_msg.c_str());
        }
    }
    else
    {
        // Update connection state for UART
        auto& conn = connection_data[found_idx];
        conn.status = ConnectionStatus::Connecting;
        conn.opened_port = data_port;
        conn.is_responder = true;
        conn.remote_ip = inet_ntoa(addr.sin_addr);
        conn.remote_port = ntohs(addr.sin_port);
        conn.remote_data_port = msg.port;
        conn.sockfd = sock;
    }
}

void GPIOServer::handle_response_msg(const handshake::ResponseMessage& msg, const struct sockaddr_in& addr)
{
    // Initiator side receives Response from Responder
    for (std::size_t i = 0; i < MAX_CONNECTION_COUNT; ++i)
    {
        if (connection_bit_map[i])
        {
            auto& conn_data = connection_data[i];
            if (conn_data.net_id == msg.net_id && conn_data.status == ConnectionStatus::Connecting &&
                !conn_data.is_responder)
            {
                // Verify it's from the same IP and port
                if (conn_data.remote_ip == inet_ntoa(addr.sin_addr) && conn_data.remote_port == ntohs(addr.sin_port))
                {
                    handshake::FinalResponseMessage final_resp{ };
                    final_resp.status = msg.status;
                    final_resp.net_id = msg.net_id;
                    sendto(m_handshake_socket,
                           &final_resp,
                           sizeof(final_resp),
                           0,
                           reinterpret_cast<const struct sockaddr*>(&addr),
                           sizeof(addr));

                    if (msg.status == 1)
                    {
                        conn_data.remote_data_port = msg.port;
                        conn_data.status = ConnectionStatus::Connected;

                        if (logging_system)
                        {
                            std::string log_msg =
                            "Connected to " + conn_data.remote_ip + ":" + std::to_string(conn_data.remote_port);
                            log_msg += " (Remote data port: " + std::to_string(msg.port) + ")";
                            logging_system->Info(log_msg.c_str());
                        }

                        construct_connection(conn_data);
                    }
                    else
                    {
                        conn_data.status = ConnectionStatus::Failed;
                        if (conn_data.sockfd != -1)
                        {
                            close(conn_data.sockfd);
                            conn_data.sockfd = -1;
                        }
                    }
                    break;
                }
            }
        }
    }
}

void GPIOServer::handle_final_response_msg(const handshake::FinalResponseMessage& msg, const struct sockaddr_in& addr)
{
    for (std::size_t i = 0; i < MAX_CONNECTION_COUNT; ++i)
    {
        if (connection_bit_map[i])
        {
            auto& conn_data = connection_data[i];
            if (conn_data.net_id == msg.net_id && conn_data.status == ConnectionStatus::Connecting &&
                conn_data.is_responder)
            {
                if (conn_data.remote_ip == inet_ntoa(addr.sin_addr) && conn_data.remote_port == ntohs(addr.sin_port))
                {
                    if (msg.status == 1)
                    {
                        conn_data.status = ConnectionStatus::Connected;

                        if (logging_system)
                        {
                            std::string log_msg = "Final ACK received from " + conn_data.remote_ip + ":" +
                                                  std::to_string(conn_data.remote_port);
                            logging_system->Info(log_msg.c_str());
                        }

                        construct_connection(connection_data[i]);
                    }
                    else
                    {
                        if (connection_data[i].sockfd != -1)
                        {
                            close(connection_data[i].sockfd);
                            connection_data[i].sockfd = -1;
                        }
                        connection_data[i].status = ConnectionStatus::Failed;
                    }
                    break;
                }
            }
        }
    }
}

void GPIOServer::handle_disconnect_msg(const handshake::DisconnectMessage& msg, const struct sockaddr_in& addr)
{
    if (msg.config.protocol_id == handshake::ProtocolID::I2C && !msg.config.config.i2c.is_master)
    {
        remove_slave_from_master(msg.config.net_id);
    }

    if (!m_net_id_to_idx.contains(msg.config.net_id))
    {
        return;
    }

    const auto idx = m_net_id_to_idx.at(msg.config.net_id);
    auto& conn = connection_data[idx];

    if (connection_bit_map[idx] && conn.status == ConnectionStatus::Connected)
    {
        if (conn.remote_ip == inet_ntoa(addr.sin_addr) && conn.remote_port == ntohs(addr.sin_port))
        {
            if (logging_system)
            {
                std::string log_msg = "Remote disconnected Connection ID: " + std::to_string(msg.config.net_id);
                logging_system->Info(log_msg.c_str());
            }

            connection_running[idx].store(false);
            m_backoffs[idx].out_queue_reader.wake();
            conn.status = ConnectionStatus::Defined;
            unmap_connection(idx);
        }
    }
}

void GPIOServer::cleanup_finished_connections()
{
    for (std::size_t i = 0; i < MAX_CONNECTION_COUNT; i++)
    {
        if (connection_bit_map[i] && !connection_running[i].load())
        {
            // If it was a responder, or it failed, we might want to remove it
            // For now, if it's not running, we set status to Defined if it was Connected
            if (connection_data[i].status == ConnectionStatus::Connected)
            {
                connection_data[i].status = ConnectionStatus::Defined;
                unmap_connection(i);
            }
        }
    }
}

void GPIOServer::run(const std::stop_token& stop_token)
{
    m_pin_write_thread = std::jthread([this](const std::stop_token& stop_token) { pin_write(stop_token); });

    while (!stop_token.stop_requested())
    {
        handle_handshake();
        cleanup_finished_connections();
        constexpr std::size_t WAIT_TIME = 10;
        std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_TIME));
    }
}

void GPIOServer::stop()
{
    m_server_thread.request_stop();
    m_pin_write_thread.request_stop();
    pin_write_reader_backoff.wake();

    for (std::size_t i = 0; i < connection_threads.size(); i++)
    {
        connection_running[i].store(false);
        connection_threads[i].request_stop();
        m_backoffs[i].out_queue_reader.wake();
    }
}

GPIOServer::~GPIOServer()
{
    stop();
}

GPIOConnection::GPIOConnection(std::size_t idx, GPIOServer& server)
: connection(server.get_connection_info(idx))
, m_backoffs(server.get_backoffs(idx))
, m_queue_reader(server.get_out_queue_buffer(idx))
, m_socket(connection.sockfd)
, m_halt(server.get_halt())
, m_start(server.get_start())
, m_read_pin(server.get_read_pin())
, m_server(server)
, m_server_running(server.is_server_running())
, m_connection_running(server.is_connection_running(idx))
, m_total_cycles(server.get_total_cycles())
{
    connection.sockfd = -1; // Take ownership

    if (m_socket != -1 && connection.remote_data_port != 0)
    {
        struct sockaddr_in remote_addr{ };
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_port = htons(connection.remote_data_port);
        inet_pton(AF_INET, connection.remote_ip.c_str(), &remote_addr.sin_addr);

        if (connect(m_socket, reinterpret_cast<struct sockaddr*>(&remote_addr), sizeof(remote_addr)) == 0)
        {
            m_other_side = remote_addr;
        }
    }

    auto pin_write_cb = [this](std::uint8_t pin, std::uint8_t val) { this->write_to_pin(pin, val); };
    auto pin_read_cb = [this](std::uint8_t pin) { return static_cast<uint8_t>(m_read_pin(pin)); };

    const BitProcessorContext<pin_pair, GPIOServer::BUFFER_SIZE> bp_ctx = {
        .buf = server.get_out_queue_buffer(idx),
        .total_cycles = m_total_cycles,
        .reader_backoff = m_backoffs.out_queue_reader,
        .writer_backoff = m_backoffs.out_queue_writer,
    };

    if (std::holds_alternative<UART_P>(connection.protocol))
    {
        auto& uart_p = std::get<UART_P>(connection.protocol);
        uart_p.other_side_fd = m_socket;

        const UART_HandlerContext handler_ctx = {
            .pin_write = pin_write_cb,
            .total_cycles = m_total_cycles,
        };

        m_processor.emplace(
        std::in_place_type<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE, UART_Handler<GPIOServer::BUFFER_SIZE>>>,
        bp_ctx,
        uart_p,
        handler_ctx);
    }
    else if (std::holds_alternative<I2C_Master_P>(connection.protocol))
    {
        auto& i2c_m = std::get<I2C_Master_P>(connection.protocol);

        const I2C_HandlerContext handler_ctx = {
            .halt = m_halt,
            .start = m_start,
            .pin_write = pin_write_cb,
            .pin_read = pin_read_cb,
            .total_cycles = m_total_cycles,
            .reader_backoff = m_backoffs.handler_queue_reader,
            .writer_backoff = m_backoffs.handler_queue_writer,
        };

        m_processor.emplace(
        std::in_place_type<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE, I2C_Master<GPIOServer::BUFFER_SIZE>>>,
        bp_ctx,
        i2c_m,
        handler_ctx);

        if (m_socket != -1)
        {
            this->add_slave(m_socket, static_cast<uint32_t>(connection.remote_port));
        }
    }
    else if (std::holds_alternative<I2C_Slave_P>(connection.protocol))
    {
        auto& i2c_s = std::get<I2C_Slave_P>(connection.protocol);
        i2c_s.master_fd = m_socket;

        const I2C_HandlerContext handler_ctx = {
            .halt = m_halt,
            .start = m_start,
            .pin_write = pin_write_cb,
            .pin_read = pin_read_cb,
            .total_cycles = m_total_cycles,
            .reader_backoff = m_backoffs.out_queue_reader,
            .writer_backoff = m_backoffs.out_queue_writer,
        };

        m_processor.emplace(
        std::in_place_type<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE, I2C_Slave<GPIOServer::BUFFER_SIZE>>>,
        bp_ctx,
        i2c_s,
        handler_ctx);
    }
}

GPIOConnection::~GPIOConnection()
{
    if (m_socket != -1)
    {
        close(m_socket);
    }
}

void GPIOConnection::run(std::stop_token stop_token)
{
    if (m_processor.has_value())
    {
        std::visit(
        [this, &stop_token](auto& p) {
            p.start();
            while (!stop_token.stop_requested() && m_connection_running.load() && p.is_running())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            p.stop();
        },
        *m_processor);
    }
    m_connection_running.store(false);
}

pin_pair GPIOConnection::read_queue()
{
    constexpr std::size_t wait_cycles = 100;
    TSP::BF::SemBackoff backoff{ wait_cycles, wait_cycles };
    return m_queue_reader.read(backoff);
}

void GPIOConnection::send_to_network(const std::vector<std::uint8_t>& data)
{
    if (m_socket != -1)
    {
        send(m_socket, data.data(), data.size(), 0);
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
, server(set_pin, read_pin, halt, start, logging_system, &m_total_cycles)
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

    server_thread = std::jthread([this](std::stop_token stop_token) { server.run(stop_token); });
}

Remote_GPIO::~Remote_GPIO()
{
    m_running.store(false);
    server.stop();
}

void Remote_GPIO::Set_ImGui_Context(void* context)
{
    ImGui_context = static_cast<ImGuiContext*>(context);
}

void Remote_GPIO::Increment_Passed_Cycles(std::uint32_t count)
{
    m_total_cycles += count;
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
    if (!server.Is_Initialized())
    {
        ImGui::Text("Server Configuration");
        ImGui::InputInt("Handshake Port", &m_ui_handshake_port);
        if (ImGui::Button("Start Server"))
        {
            server.Init(static_cast<uint16_t>(m_ui_handshake_port));
        }
        return;
    }

    ImGui::Text("Server listening on port: %d", m_ui_handshake_port);
    ImGui::Separator();

    if (ImGui::Button("Add Connection"))
    {
        ImGui::OpenPopup("Add Connection");
    }

    ImGui::Separator();
    ImGui::Text("Active Connections:");

    const auto& bit_map = server.get_connection_bit_map();
    auto& data = const_cast<std::array<conn_info, GPIOServer::MAX_CONNECTION_COUNT>&>(server.get_connection_data());

    for (std::size_t i = 0; i < bit_map.size(); ++i)
    {
        if (bit_map[i])
        {
            bool is_i2c_master = std::holds_alternative<I2C_Master_P>(data[i].protocol);
            std::string label = "Connection " + std::to_string(i) + ": ";
            std::visit(
            [this, &label, i](auto& p) {
                using T = std::remove_cvref_t<decltype(p)>;
                if constexpr (std::is_same_v<T, UART_P>)
                {
                    label += "UART";
                }
                else if constexpr (std::is_same_v<T, I2C_Master_P>)
                {
                    label += "I2C Master";
                    label += " (" + std::to_string(this->server.get_slave_count(i)) + " slaves)";
                }
                else if constexpr (std::is_same_v<T, I2C_Slave_P>)
                {
                    label += "I2C Slave";
                }
            },
            data[i].protocol);

            if (data[i].status == ConnectionStatus::Connecting)
            {
                label += " (Connecting...)";
            }
            else if (data[i].status == ConnectionStatus::Defined)
            {
                label += " (Not connected)";
            }
            else if (data[i].status == ConnectionStatus::Failed)
            {
                label += " (Failed)";
            }

            if (ImGui::Selectable(label.c_str(), false, 0, ImVec2(ImGui::GetWindowWidth() - 100, 0)))
            {
                m_ui_view_idx = static_cast<int>(i);
                ImGui::OpenPopup("View Connection");
            }

            ImGui::SameLine(ImGui::GetWindowWidth() - 95);
            if (!is_i2c_master)
            {
                if (data[i].status == ConnectionStatus::Defined || data[i].status == ConnectionStatus::Failed)
                {
                    if (ImGui::Button(("Connect##" + std::to_string(i)).c_str()))
                    {
                        server.initiate_handshake(i);
                    }
                }
                else if (data[i].status == ConnectionStatus::Connecting)
                {
                    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
                                                                         data[i].start_time)
                        .count() > 1)
                    {
                        data[i].status = ConnectionStatus::Failed;
                    }
                }
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
            ImGui::InputInt("Connection ID", &m_ui_add_state.net_id);
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
            ImGui::InputInt("Slave ID", &m_ui_add_state.net_id);
            ImGui::InputInt("Bus ID", &m_ui_add_state.i2c_id);
            ImGui::InputInt("Address", &m_ui_add_state.address);
            ImGui::InputInt("SCL Pin", &m_ui_add_state.scl_pin);
            ImGui::InputInt("SDA Pin", &m_ui_add_state.sda_pin);
            ImGui::InputText("Remote IP", m_ui_add_state.ip, sizeof(m_ui_add_state.ip));
            ImGui::InputInt("Remote Port", &m_ui_add_state.port);
        }

        if (ImGui::Button("OK", ImVec2(120, 0)))
        {
            conn_info info{ };
            info.remote_ip = m_ui_add_state.ip;
            info.remote_port = m_ui_add_state.port;

            if (m_ui_add_state.protocol_type == 0) // UART
            {
                info.net_id = static_cast<uint32_t>(m_ui_add_state.net_id);
                UART_P p{ };
                p.baudrate = m_ui_add_state.baudrate;
                p.tx_pin = m_ui_add_state.tx_pin;
                p.rx_pin = m_ui_add_state.rx_pin;
                p.start_bits = m_ui_add_state.start_bits;
                p.data_bits = m_ui_add_state.data_bits;
                p.parity_bits = m_ui_add_state.parity_bits;
                p.stop_bits = m_ui_add_state.stop_bits;
                info.protocol = p;
            }
            else if (m_ui_add_state.protocol_type == 1) // I2C Master
            {
                info.net_id = static_cast<uint32_t>(m_ui_add_state.i2c_id);
                I2C_Master_P p{ };
                p.bus_id = static_cast<uint32_t>(m_ui_add_state.i2c_id);
                p.scl_pin = m_ui_add_state.scl_pin;
                p.sda_pin = m_ui_add_state.sda_pin;
                info.protocol = p;
                info.remote_ip = "";
                info.remote_port = 0;
            }
            else if (m_ui_add_state.protocol_type == 2) // I2C Slave
            {
                info.net_id = static_cast<uint32_t>(m_ui_add_state.net_id);
                I2C_Slave_P p{ };
                p.bus_id = static_cast<uint32_t>(m_ui_add_state.i2c_id);
                p.address = static_cast<uint8_t>(m_ui_add_state.address);
                p.scl_pin = m_ui_add_state.scl_pin;
                p.sda_pin = m_ui_add_state.sda_pin;
                info.protocol = p;
            }

            server.add_connection(info);
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
            bool is_i2c_master = std::holds_alternative<I2C_Master_P>(conn.protocol);
            bool is_i2c_slave = std::holds_alternative<I2C_Slave_P>(conn.protocol);

            if (!is_i2c_master && !is_i2c_slave)
            {
                ImGui::Text("Remote IP: %s", conn.remote_ip.c_str());
                ImGui::Text("Handshake Port: %d", conn.remote_port);
                ImGui::Text("Connection ID: %u", conn.net_id);
            }
            ImGui::Separator();

            std::visit(
            [conn](auto& p) {
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
                }
                else if constexpr (std::is_same_v<T, I2C_Master_P>)
                {
                    ImGui::Text("Protocol: I2C Master");
                    ImGui::Text("Bus ID: %u", p.bus_id);
                    ImGui::Text("Connection ID: %u", conn.net_id);
                    ImGui::Text("SCL Pin: %u", p.scl_pin);
                    ImGui::Text("SDA Pin: %u", p.sda_pin);
                }
                else if constexpr (std::is_same_v<T, I2C_Slave_P>)
                {
                    ImGui::Text("Protocol: I2C Slave");
                    ImGui::Text("Target Bus ID: %u", p.bus_id);
                    ImGui::Text("Slave ID: %u", conn.net_id);
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
