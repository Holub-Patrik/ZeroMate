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

    bool uart_config_matches(const UART_P& protocol, const handshake::UARTConfig& msg)
    {
        return protocol.baudrate == msg.baudrate && protocol.data_bits == msg.data_bits &&
               protocol.start_bits == msg.start_bits && protocol.parity_bits == msg.parity_bits &&
               protocol.stop_bits == msg.stop_bits;
    }
}

// === GPIOConnection ===

void GPIOConnection::init(std::size_t idx, GPIOServer* server)
{
    m_idx = idx;
    m_server = server;
}

GPIOConnection::~GPIOConnection()
{
    stop_thread();
    if (m_info.sockfd != -1)
    {
        close(m_info.sockfd);
    }
}

void GPIOConnection::clear()
{
    stop_thread();
    if (m_info.sockfd != -1)
    {
        close(m_info.sockfd);
        m_info.sockfd = -1;
    }
    m_info.status = ConnectionStatus::Defined;
    m_info.opened_port = 0;
    m_info.remote_data_port = 0;
    m_handler.reset();
}

void GPIOConnection::update()
{
    auto pin_write_cb = [this](std::uint8_t pin, std::uint8_t val) { this->write_to_pin(pin, val); };
    auto pin_read_cb = [this](std::uint8_t pin) { return static_cast<uint8_t>(m_server->get_read_pin()(pin)); };

    const auto construct_visitor = overloads{
        [this, &pin_write_cb](UART_P& prot) -> void {
            prot.other_side_fd = m_info.sockfd;
            const UART_HandlerContext ctx = { .pin_write = pin_write_cb, .total_cycles = m_server->get_total_cycles() };
            m_handler.emplace(std::in_place_type<UART_Handler<net_comm::BUFFER_SIZE>>, prot, ctx);
        },
        [this, &pin_write_cb, &pin_read_cb](I2C_Master_P& prot) {
            const I2C_HandlerContext ctx = { .halt = m_server->get_halt(),
                                             .start = m_server->get_start(),
                                             .pin_write = pin_write_cb,
                                             .pin_read = pin_read_cb,
                                             .total_cycles = m_server->get_total_cycles(),
                                             .reader_backoff = m_server->get_backoffs(m_idx).handler_queue_reader,
                                             .writer_backoff = m_server->get_backoffs(m_idx).handler_queue_writer };
            m_handler.emplace(std::in_place_type<I2C_Master<net_comm::BUFFER_SIZE>>, prot, ctx);
        },
        [this, &pin_write_cb, &pin_read_cb](I2C_Slave_P& prot) {
            prot.master_fd = m_info.sockfd;
            const I2C_HandlerContext ctx = { .halt = m_server->get_halt(),
                                             .start = m_server->get_start(),
                                             .pin_write = pin_write_cb,
                                             .pin_read = pin_read_cb,
                                             .total_cycles = m_server->get_total_cycles(),
                                             .reader_backoff = m_server->get_backoffs(m_idx).out_queue_reader,
                                             .writer_backoff = m_server->get_backoffs(m_idx).out_queue_writer };
            m_handler.emplace(std::in_place_type<I2C_Slave<net_comm::BUFFER_SIZE>>, prot, ctx);
        }
    };

    std::visit(construct_visitor, m_info.protocol);
}

void GPIOConnection::start_thread()
{
    if (m_running.load())
    {
        return;
    }
    update();
    m_running.store(true);
    m_thread = std::jthread([this](std::stop_token stop_token) { this->run(stop_token); });
}

void GPIOConnection::stop_thread()
{
    m_running.store(false);
    if (m_server)
    {
        m_server->get_backoffs(m_idx).out_queue_reader.wake();
    }
    if (m_thread.joinable())
    {
        m_thread.request_stop();
        m_thread.join();
    }
}

void GPIOConnection::run(std::stop_token stop_token)
{
    if (!m_handler.has_value())
    {
        m_running.store(false);
        return;
    }

    std::visit([](auto& handler) { handler.start_receiver(); }, m_handler.value());

    auto* const queue_buffer = m_server->get_out_queue_buffer(m_idx);
    TSP::Queue::Reader<pin_pair, net_comm::BUFFER_SIZE> queue_reader(queue_buffer);
    auto& backoffs = m_server->get_backoffs(m_idx);

    std::uint64_t last_cycles = m_server->get_total_cycles()->load();

    while (!stop_token.stop_requested() && m_running.load())
    {
        if (!queue_reader.try_advance())
        {
            backoffs.out_queue_reader.wait([this, &stop_token, &queue_reader]() {
                return stop_token.stop_requested() || !m_running.load() || queue_reader.try_advance();
            });
            continue;
        }
        backoffs.out_queue_reader.reset();

        const auto pair = queue_reader.peek();
        queue_reader.advance();
        backoffs.out_queue_writer.wake();

        const std::uint64_t now_cycles = m_server->get_total_cycles()->load();
        const auto delta = static_cast<std::uint32_t>(now_cycles - last_cycles);
        last_cycles = now_cycles;

        std::visit([&pair, &delta](auto& handler) { handler.process_bit(pair, delta); }, m_handler.value());

        if (!std::visit([](auto& handler) { return handler.is_alive(); }, m_handler.value()))
        {
            break;
        }
    }

    std::visit([](auto& handler) { handler.receiver_stop(); }, m_handler.value());
    m_running.store(false);
}

command::Response GPIOConnection::execute(const command::Command& cmd)
{
    if (!m_handler.has_value())
    {
        return { .status = command::ResponseStatus::Fail, .data = 0 };
    }

    const auto cmd_visitor =
    overloads{ [this](const command::AddSlave& cmd) -> command::Response {
                  if (const auto& handler = std::get_if<I2C_Master<net_comm::BUFFER_SIZE>>(&m_handler.value()))
                  {
                      handler->add_slave(cmd.fd, cmd.slave_id);
                      return { .status = command::ResponseStatus::Success, .data = 0 };
                  }
                  return { .status = command::ResponseStatus::Fail, .data = 0 };
              },
               [this](const command::RemoveSlave& c) -> command::Response {
                   if (const auto& handler = std::get_if<I2C_Master<net_comm::BUFFER_SIZE>>(&m_handler.value()))
                   {
                       handler->remove_slave(c.slave_id);
                       return { .status = command::ResponseStatus::Success, .data = 0 };
                   }
                   return { .status = command::ResponseStatus::Fail, .data = 0 };
               },
               [this](const command::GetSlaveCount&) -> command::Response {
                   if (const auto& handler = std::get_if<I2C_Master<net_comm::BUFFER_SIZE>>(&m_handler.value()))
                   {
                       return { .status = command::ResponseStatus::Success,
                                .data = static_cast<std::uint32_t>(handler->get_slave_count()) };
                   }
                   return { .status = command::ResponseStatus::Fail, .data = 0 };
               },
               [this](const command::HasSlave& c) -> command::Response {
                   if (const auto& handler = std::get_if<I2C_Master<net_comm::BUFFER_SIZE>>(&m_handler.value()))
                   {
                       return { .status = command::ResponseStatus::Success,
                                .data = handler->has_slave(c.slave_id) ? 1U : 0U };
                   }
                   return { .status = command::ResponseStatus::Fail, .data = 0 };
               } };

    return std::visit(cmd_visitor, cmd);
}

void GPIOConnection::write_to_pin(std::uint8_t pin, std::uint8_t value)
{
    m_server->write_to_pin(pin, value);
}

// === GPIOServer ===

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
    for (std::size_t i = 0; i < net_comm::MAX_CONNECTION_COUNT; i++)
    {
        std::construct_at(&out_queue_writers[i], &out_queue_buffers[i]);
        m_connections[i].init(i, this);
    }
}

GPIOServer::~GPIOServer()
{
    stop();
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
        addr.sin_port = htons(m_handshake_port);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(m_handshake_socket, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
        {
            close(m_handshake_socket);
            m_handshake_socket = -1;
        }
    }
}

bool GPIOServer::Is_Initialized() const noexcept
{
    return m_handshake_socket != -1;
}

void GPIOServer::pin_write(const std::stop_token& stop_token)
{
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
        auto& conn = m_connections[index];
        conn.get_info() = info;
        conn.get_info().status = ConnectionStatus::Defined;
        conn.get_info().sockfd = -1;
        conn.set_defined(true);

        const auto mapping_visitor =
        overloads{ [this, index](const UART_P& p) {
                      pin_to_conn_id.set(static_cast<std::uint8_t>(p.tx_pin), static_cast<std::uint8_t>(index));
                  },
                   [this, index](const I2C_Master_P& p) {
                       pin_to_conn_id.set(static_cast<std::uint8_t>(p.scl_pin), static_cast<std::uint8_t>(index));
                       pin_to_conn_id.set(static_cast<std::uint8_t>(p.sda_pin), static_cast<std::uint8_t>(index));
                       m_bus_id_to_idx[p.bus_id] = index;
                       m_connections[index].get_info().status = ConnectionStatus::Connected;
                       m_connections[index].start_thread();
                   },
                   [this, index](const I2C_Slave_P& p) {
                       pin_to_conn_id.set(static_cast<std::uint8_t>(p.sda_pin), static_cast<std::uint8_t>(index));
                   } };
        std::visit(mapping_visitor, conn.get_info().protocol);
    }
}

void GPIOServer::remove_connection(std::size_t idx)
{
    if (idx >= net_comm::MAX_CONNECTION_COUNT || !m_connections[idx].is_defined())
    {
        return;
    }

    auto& conn = m_connections[idx];
    if (conn.get_info().status == ConnectionStatus::Connected && m_handshake_socket != -1)
    {
        handshake::DisconnectMessage d_msg{ };
        d_msg.config.net_id = conn.get_info().net_id;

        const auto visitor =
        overloads{ [&d_msg](const UART_P&) { d_msg.config.protocol_id = handshake::ProtocolID::UART; },
                   [&d_msg](const I2C_Master_P& protocol) {
                       d_msg.config.protocol_id = handshake::ProtocolID::I2C;
                       d_msg.config.config.i2c.bus_id = protocol.bus_id;
                       d_msg.config.config.i2c.is_master = 1;
                   },
                   [&d_msg](const I2C_Slave_P& protocol) {
                       d_msg.config.protocol_id = handshake::ProtocolID::I2C;
                       d_msg.config.config.i2c.bus_id = protocol.bus_id;
                       d_msg.config.config.i2c.is_master = 0;
                   } };
        std::visit(visitor, conn.get_info().protocol);

        struct sockaddr_in remote_addr{ };
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_port = htons(static_cast<uint16_t>(conn.get_info().remote_port));
        inet_pton(AF_INET, conn.get_info().remote_ip.c_str(), &remote_addr.sin_addr);

        sendto(m_handshake_socket, &d_msg, sizeof(d_msg), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
    }

    unmap_connection(idx);
    conn.clear();
    conn.set_defined(false);
}

void GPIOServer::unmap_connection(const std::size_t conn_index)
{
    const auto unmap_visitor =
    overloads{ [this](const UART_P& p) { pin_to_conn_id.set(static_cast<std::uint8_t>(p.tx_pin), UINT8_MAX); },
               [this](const I2C_Master_P& p) {
                   pin_to_conn_id.set(static_cast<std::uint8_t>(p.scl_pin), UINT8_MAX);
                   pin_to_conn_id.set(static_cast<std::uint8_t>(p.sda_pin), UINT8_MAX);
                   m_bus_id_to_idx.erase(p.bus_id);
               },
               [this](const I2C_Slave_P& p) { pin_to_conn_id.set(static_cast<std::uint8_t>(p.sda_pin), UINT8_MAX); } };
    std::visit(unmap_visitor, m_connections[conn_index].get_info().protocol);
}

std::uint8_t GPIOServer::find_free_index() const noexcept
{
    for (std::size_t i = 0; i < net_comm::MAX_CONNECTION_COUNT; ++i)
    {
        if (!m_connections[i].is_defined())
        {
            return static_cast<std::uint8_t>(i);
        }
    }
    return UINT8_MAX;
}

void GPIOServer::add_slave_to_master(std::uint32_t bus_id, int fd, std::uint32_t slave_id)
{
    if (m_bus_id_to_idx.contains(bus_id))
    {
        m_connections[m_bus_id_to_idx.at(bus_id)].execute(command::AddSlave{ .fd = fd, .slave_id = slave_id });
    }
}

void GPIOServer::remove_slave_from_master(std::uint32_t slave_id)
{
    for (auto& conn : m_connections)
    {
        if (conn.is_defined() && std::holds_alternative<I2C_Master_P>(conn.get_info().protocol))
        {
            conn.execute(command::RemoveSlave{ slave_id });
        }
    }
}

std::size_t GPIOServer::get_slave_count(std::size_t idx) const
{
    if (idx < net_comm::MAX_CONNECTION_COUNT && m_connections[idx].is_defined())
    {
        auto resp = const_cast<GPIOConnection&>(m_connections[idx]).execute(command::GetSlaveCount{ });
        if (resp.status == command::ResponseStatus::Success)
        {
            return resp.data;
        }
    }
    return 0;
}

void GPIOServer::initiate_handshake(std::size_t idx)
{
    if (m_handshake_socket == -1 || idx >= net_comm::MAX_CONNECTION_COUNT || !m_connections[idx].is_defined())
    {
        return;
    }

    auto& info = m_connections[idx].get_info();
    handshake::ConfMessage msg{ };
    msg.net_id = info.net_id;
    msg.type = handshake::MessageType::Conf;

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr{ .sin_family = AF_INET, .sin_port = 0, .sin_addr = { .s_addr = INADDR_ANY } };
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    socklen_t len = sizeof(addr);
    getsockname(sock, (struct sockaddr*)&addr, &len);
    msg.port = ntohs(addr.sin_port);

    const auto visitor = overloads{ [&msg](const UART_P& p) {
                                       msg.protocol_id = handshake::ProtocolID::UART;
                                       msg.config.uart = { .baudrate = p.baudrate,
                                                           .data_bits = (uint8_t)p.data_bits,
                                                           .start_bits = (uint8_t)p.start_bits,
                                                           .parity_bits = (uint8_t)p.parity_bits,
                                                           .stop_bits = (uint8_t)p.stop_bits };
                                   },
                                    [&msg](const I2C_Master_P& p) {
                                        msg.protocol_id = handshake::ProtocolID::I2C;
                                        msg.config.i2c = { .bus_id = p.bus_id, .is_master = 1, .address = 0 };
                                    },
                                    [&msg](const I2C_Slave_P& p) {
                                        msg.protocol_id = handshake::ProtocolID::I2C;
                                        msg.config.i2c = { .bus_id = p.bus_id, .is_master = 0, .address = p.address };
                                    } };
    std::visit(visitor, info.protocol);

    struct sockaddr_in remote_addr{ };
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(static_cast<uint16_t>(info.remote_port));
    inet_pton(AF_INET, info.remote_ip.c_str(), &remote_addr.sin_addr);

    sendto(m_handshake_socket, &msg, sizeof(msg), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));

    info.status = ConnectionStatus::Connecting;
    info.opened_port = msg.port;
    info.sockfd = sock;
    info.start_time = std::chrono::steady_clock::now();
}

void GPIOServer::handle_handshake()
{
    if (m_handshake_socket == -1)
        return;

    struct pollfd pfd = { .fd = m_handshake_socket, .events = POLLIN };
    if (poll(&pfd, 1, 10) > 0 && (pfd.revents & POLLIN))
    {
        uint8_t buf[512];
        struct sockaddr_in addr{ };
        socklen_t addr_len = sizeof(addr);
        const auto received = recvfrom(m_handshake_socket, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &addr_len);
        if (received > 0 && buf[0] == handshake::MAGIC_BYTE)
        {
            const auto* msg = reinterpret_cast<const handshake::ConfMessage*>(buf);
            switch (msg->type)
            {
                case handshake::MessageType::Conf:
                    handle_conf_msg(*msg, addr);
                    break;
                case handshake::MessageType::Response:
                    handle_response_msg(*(const handshake::ResponseMessage*)buf, addr);
                    break;
                case handshake::MessageType::Disconnect:
                    handle_disconnect_msg(*(const handshake::DisconnectMessage*)buf, addr);
                    break;
                default:
                    break;
            }
        }
    }
}

void GPIOServer::handle_conf_msg(const handshake::ConfMessage& msg, const struct sockaddr_in& addr)
{
    int found_idx = -1;
    handshake::ResponseMessage resp{ .status = 0, .net_id = msg.net_id };

    if (msg.protocol_id == handshake::ProtocolID::I2C)
    {
        if (msg.config.i2c.is_master)
            return; // No Master-to-Master

        if (m_bus_id_to_idx.contains(msg.config.i2c.bus_id))
        {
            found_idx = (int)m_bus_id_to_idx.at(msg.config.i2c.bus_id);

            // Check if slave_id (net_id) already exists on this bus
            auto has_slave_resp = m_connections[found_idx].execute(command::HasSlave{ .slave_id = msg.net_id });
            if (has_slave_resp.status == command::ResponseStatus::Success && has_slave_resp.data == 1U)
            {
                // Already connected
                sendto(m_handshake_socket, &resp, sizeof(resp), 0, (struct sockaddr*)&addr, sizeof(addr));
                return;
            }
        }
    }
    else
    {
        for (std::size_t i = 0; i < net_comm::MAX_CONNECTION_COUNT; ++i)
        {
            if (m_connections[i].is_defined() && m_connections[i].get_info().net_id == msg.net_id &&
                m_connections[i].get_info().status == ConnectionStatus::Defined)
            {
                if (auto* p = std::get_if<UART_P>(&m_connections[i].get_info().protocol))
                {
                    if (uart_config_matches(*p, msg.config.uart))
                    {
                        found_idx = i;
                        break;
                    }
                }
            }
        }
    }

    if (found_idx == -1)
    {
        sendto(m_handshake_socket, &resp, sizeof(resp), 0, (struct sockaddr*)&addr, sizeof(addr));
        return;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in temp_addr{ .sin_family = AF_INET, .sin_port = 0, .sin_addr = { .s_addr = INADDR_ANY } };
    bind(sock, (struct sockaddr*)&temp_addr, sizeof(temp_addr));
    socklen_t len = sizeof(temp_addr);
    getsockname(sock, (struct sockaddr*)&temp_addr, &len);
    resp.port = ntohs(temp_addr.sin_port);
    resp.status = 1;

    struct sockaddr_in remote_data_addr = addr;
    remote_data_addr.sin_port = htons(msg.port);
    connect(sock, (struct sockaddr*)&remote_data_addr, sizeof(remote_data_addr));

    sendto(m_handshake_socket, &resp, sizeof(resp), 0, (struct sockaddr*)&addr, sizeof(addr));

    if (msg.protocol_id == handshake::ProtocolID::I2C)
    {
        add_slave_to_master(msg.config.i2c.bus_id, sock, msg.net_id);
    }
    else
    {
        auto& info = m_connections[found_idx].get_info();
        info.status = ConnectionStatus::Connected;
        info.remote_data_port = msg.port;
        info.sockfd = sock;
        info.remote_ip = inet_ntoa(addr.sin_addr);
        info.remote_port = ntohs(addr.sin_port);
        m_connections[found_idx].start_thread();
    }
}

void GPIOServer::handle_response_msg(const handshake::ResponseMessage& msg, const struct sockaddr_in& addr)
{
    for (auto& conn : m_connections)
    {
        auto& info = conn.get_info();
        if (conn.is_defined() && info.net_id == msg.net_id && info.status == ConnectionStatus::Connecting)
        {
            if (msg.status == 1)
            {
                info.status = ConnectionStatus::Connected;
                info.remote_data_port = msg.port;
                struct sockaddr_in remote_data_addr = addr;
                remote_data_addr.sin_port = htons(msg.port);
                connect(info.sockfd, (struct sockaddr*)&remote_data_addr, sizeof(remote_data_addr));
                conn.start_thread();
            }
            else
            {
                conn.clear();
                info.status = ConnectionStatus::Failed;
            }
            break;
        }
    }
}

void GPIOServer::handle_disconnect_msg(const handshake::DisconnectMessage& msg, const struct sockaddr_in& addr)
{
    if (msg.config.protocol_id == handshake::ProtocolID::I2C)
    {
        if (!msg.config.config.i2c.is_master)
        {
            remove_slave_from_master(msg.config.net_id);
        }
        else
        {
            for (auto& conn : m_connections)
            {
                if (conn.is_defined())
                {
                    if (auto* p = std::get_if<I2C_Slave_P>(&conn.get_info().protocol))
                    {
                        if (p->bus_id == msg.config.config.i2c.bus_id)
                        {
                            conn.clear();
                        }
                    }
                }
            }
        }
    }
    else
    {
        for (auto& conn : m_connections)
        {
            if (conn.is_defined() && conn.get_info().net_id == msg.config.net_id)
            {
                conn.clear();
                break;
            }
        }
    }
}

void GPIOServer::run(const std::stop_token& stop_token)
{
    m_pin_write_thread = std::jthread([this](const std::stop_token& st) { pin_write(st); });
    while (!stop_token.stop_requested())
    {
        handle_handshake();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void GPIOServer::stop()
{
    m_running.store(false);
    m_server_thread.request_stop();
    m_pin_write_thread.request_stop();
    pin_write_reader_backoff.wake();

    for (auto& conn : m_connections)
    {
        conn.stop_thread();
    }
}

// === Remote_GPIO ===

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
{
    for (const auto& pin : pins)
        m_gpio_subscription.insert(pin);
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
    server.route_pin_info({ (uint8_t)pin_idx, (uint8_t)read_pin(pin_idx) });
}

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
            server.Init((uint16_t)m_ui_handshake_port);
        return;
    }

    ImGui::Text("Server listening on port: %d", m_ui_handshake_port);
    ImGui::Separator();

    if (ImGui::Button("Add Connection"))
        ImGui::OpenPopup("Add Connection");

    ImGui::Separator();
    ImGui::Text("Active Connections:");

    auto& connections = server.get_connections();
    for (std::size_t i = 0; i < connections.size(); ++i)
    {
        if (connections[i].is_defined())
        {
            auto& info = connections[i].get_info();
            bool is_i2c_master = std::holds_alternative<I2C_Master_P>(info.protocol);
            std::string label = "Connection " + std::to_string(i) + ": ";

            std::visit(overloads{ [&label](const UART_P&) { label += "UART"; },
                                  [this, &label, i](const I2C_Master_P&) {
                                      label += "I2C Master (" + std::to_string(server.get_slave_count(i)) + " slaves)";
                                  },
                                  [&label](const I2C_Slave_P&) { label += "I2C Slave"; } },
                       info.protocol);

            if (info.status == ConnectionStatus::Connecting)
                label += " (Connecting...)";
            else if (info.status == ConnectionStatus::Defined)
                label += " (Defined)";
            else if (info.status == ConnectionStatus::Failed)
                label += " (Failed)";

            if (ImGui::Selectable(label.c_str(), false, 0, ImVec2(ImGui::GetWindowWidth() - 100, 0)))
            {
                m_ui_view_idx = (int)i;
                ImGui::OpenPopup("View Connection");
            }

            ImGui::SameLine(ImGui::GetWindowWidth() - 95);
            if (!is_i2c_master)
            {
                if (info.status == ConnectionStatus::Defined || info.status == ConnectionStatus::Failed)
                {
                    if (ImGui::Button(("Connect##" + std::to_string(i)).c_str()))
                        server.initiate_handshake(i);
                }
                else if (info.status == ConnectionStatus::Connecting)
                {
                    if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() -
                                                                         info.start_time)
                        .count() > 2)
                    {
                        const_cast<conn_info&>(info).status = ConnectionStatus::Failed;
                    }
                }
            }

            ImGui::SameLine(ImGui::GetWindowWidth() - 35);
            if (ImGui::Button(("x##" + std::to_string(i)).c_str()))
                server.remove_connection(i);
        }
    }

    if (ImGui::BeginPopupModal("Add Connection", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ImGui::Combo("Protocol", &m_ui_add_state.protocol_type, "UART\0I2C Master\0I2C Slave\0\0");
        if (m_ui_add_state.protocol_type == 0)
        {
            ImGui::InputInt("Net ID", &m_ui_add_state.net_id);
            ImGui::InputInt("Baudrate", &m_ui_add_state.baudrate);
            ImGui::InputInt("TX Pin", &m_ui_add_state.tx_pin);
            ImGui::InputInt("RX Pin", &m_ui_add_state.rx_pin);
            ImGui::InputText("Remote IP", m_ui_add_state.ip, sizeof(m_ui_add_state.ip));
            ImGui::InputInt("Remote Port", &m_ui_add_state.port);
        }
        else if (m_ui_add_state.protocol_type == 1)
        {
            ImGui::InputInt("Bus ID", &m_ui_add_state.i2c_id);
            ImGui::InputInt("SCL Pin", &m_ui_add_state.scl_pin);
            ImGui::InputInt("SDA Pin", &m_ui_add_state.sda_pin);
        }
        else if (m_ui_add_state.protocol_type == 2)
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
            conn_info info{ .remote_ip = m_ui_add_state.ip, .remote_port = m_ui_add_state.port };
            if (m_ui_add_state.protocol_type == 0)
            {
                info.net_id = (uint32_t)m_ui_add_state.net_id;
                info.protocol = UART_P{ .baudrate = (uint32_t)m_ui_add_state.baudrate,
                                        .tx_pin = (uint32_t)m_ui_add_state.tx_pin,
                                        .rx_pin = (uint32_t)m_ui_add_state.rx_pin,
                                        .start_bits = 1,
                                        .data_bits = 8,
                                        .parity_bits = 0,
                                        .stop_bits = 1 };
            }
            else if (m_ui_add_state.protocol_type == 1)
            {
                info.net_id = (uint32_t)m_ui_add_state.i2c_id;
                info.protocol = I2C_Master_P{ .bus_id = (uint32_t)m_ui_add_state.i2c_id,
                                              .scl_pin = (uint32_t)m_ui_add_state.scl_pin,
                                              .sda_pin = (uint32_t)m_ui_add_state.sda_pin };
            }
            else if (m_ui_add_state.protocol_type == 2)
            {
                info.net_id = (uint32_t)m_ui_add_state.net_id;
                info.protocol = I2C_Slave_P{ .bus_id = (uint32_t)m_ui_add_state.i2c_id,
                                             .address = (uint8_t)m_ui_add_state.address,
                                             .scl_pin = (uint32_t)m_ui_add_state.scl_pin,
                                             .sda_pin = (uint32_t)m_ui_add_state.sda_pin };
            }
            server.add_connection(info);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0)))
            ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (ImGui::BeginPopupModal("View Connection", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        if (m_ui_view_idx != -1 && connections[m_ui_view_idx].is_defined())
        {
            auto& info = connections[m_ui_view_idx].get_info();
            std::visit(overloads{ [&info](const UART_P& p) {
                                     ImGui::Text("Protocol: UART");
                                     ImGui::Text("Baudrate: %u", p.baudrate);
                                     ImGui::Text("TX Pin: %u", p.tx_pin);
                                     ImGui::Text("RX Pin: %u", p.rx_pin);
                                 },
                                  [&info](const I2C_Master_P& p) {
                                      ImGui::Text("Protocol: I2C Master");
                                      ImGui::Text("Bus ID: %u", p.bus_id);
                                  },
                                  [&info](const I2C_Slave_P& p) {
                                      ImGui::Text("Protocol: I2C Slave");
                                      ImGui::Text("Bus ID: %u", p.bus_id);
                                      ImGui::Text("Address: 0x%02X", p.address);
                                  } },
                       info.protocol);
            ImGui::Text("Status: %d", (int)info.status);
        }
        if (ImGui::Button("Close", ImVec2(120, 0)))
            ImGui::CloseCurrentPopup();
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
            pins.push_back(connection[i]);
        *peripheral = new (std::nothrow)::Remote_GPIO(name, pins, read_pin, set_pin, halt, start, logging_system);
        return *peripheral ? zero_mate::IExternal_Peripheral::NInit_Status::OK
                           : zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
    }
}
