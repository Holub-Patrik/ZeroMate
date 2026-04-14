#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <vector>
#include <array>
#include <algorithm>
#include <cerrno>

#include "fmt/format.h"
#include "imgui.h"
#include "gpio_server.hpp"

constexpr std::uint16_t DEFAULT_PORT = 9000;

namespace zero_mate::peripheral
{
    CGPIO_Server::CGPIO_Server(std::string name, utils::CLogging_System* logging_system)
    : m_name{ std::move(name) }
    , m_handshake_port{ DEFAULT_PORT }
    , m_logging_system{ logging_system }
    , m_ui_port{ static_cast<int>(DEFAULT_PORT) }
    {
        if (s_instance != nullptr)
        {
            throw std::runtime_error("CGPIO_Server instance already exists");
        }

        s_instance = this;
    }

    CGPIO_Server::~CGPIO_Server()
    {
        StopServer();

        // When the server itself is destroyed, we should close all components' FDs.
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto& [fd, comp] : m_components)
        {
            if (fd != -1)
            {
                close(fd);
            }
        }
        m_components.clear();
        m_peer_to_components.clear();

        s_instance = nullptr;
    }

    void CGPIO_Server::StartServer()
    {
        if (m_enabled)
        {
            return;
        }

        m_handshake_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_handshake_sock == -1)
        {
            if (m_logging_system != nullptr)
            {
                m_logging_system->Error(
                fmt::format("CGPIO_Server [{}]: Failed to create handshake socket", m_name).c_str());
            }
            return;
        }

        struct sockaddr_in addr{ };
        addr.sin_family = AF_INET;
        addr.sin_port = htons((uint16_t)m_ui_port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(m_handshake_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            if (m_logging_system != nullptr)
            {
                m_logging_system->Error(
                fmt::format("CGPIO_Server [{}]: Failed to bind handshake socket to port {}", m_name, m_ui_port)
                .c_str());
            }
            close(m_handshake_sock);
            m_handshake_sock = -1;
            return;
        }

        m_running = true;
        m_enabled = true;
        m_networking_thread = std::thread(&CGPIO_Server::Networking_Thread, this);

        if (m_logging_system != nullptr)
        {
            m_logging_system->Info(
            fmt::format("CGPIO_Server [{}]: Server started on port {}", m_name, m_ui_port).c_str());
        }
    }

    void CGPIO_Server::StopServer()
    {
        if (!m_enabled)
        {
            return;
        }

        if (m_logging_system != nullptr)
        {
            m_logging_system->Info(fmt::format("CGPIO_Server [{}]: Stopping server", m_name).c_str());
        }

        m_running = false;
        if (m_networking_thread.joinable())
        {
            m_networking_thread.join();
        }

        if (m_handshake_sock != -1)
        {
            close(m_handshake_sock);
            m_handshake_sock = -1;
        }

        // Components are persistent, so we don't close their FDs or clear m_components.
        // But we do disconnect all current peers.
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto& [fd, comp] : m_components)
        {
            if (m_logging_system != nullptr && !comp.peers.empty())
            {
                m_logging_system->Debug(
                fmt::format("CGPIO_Server [{}]: Disconnecting component FD {} from all peers", m_name, fd).c_str());
            }

            // We notify the component about disconnect from each peer
            if (comp.on_disconnect != nullptr)
            {
                for (const auto& peer : comp.peers)
                {
                    char peer_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &peer.handshake_addr.sin_addr, peer_ip, INET_ADDRSTRLEN);
                    comp.on_disconnect(comp.context, peer_ip, peer.data_port);
                }
            }
            InternalDisconnect(comp);
        }

        m_peer_to_components.clear();
        m_enabled = false;
    }

    void CGPIO_Server::Render()
    {
        if (m_imgui_context != nullptr)
        {
            ImGui::SetCurrentContext(static_cast<ImGuiContext*>(m_imgui_context));
        }

        if (ImGui::Begin(m_name.c_str()))
        {
            ImGui::InputInt("Server Port", &m_ui_port);
            if (m_enabled)
            {
                if (ImGui::Button("Stop Server"))
                {
                    StopServer();
                }
            }
            else
            {
                if (ImGui::Button("Start Server"))
                {
                    StartServer();
                }
            }

            ImGui::Separator();
            ImGui::Text("Status: %s", m_enabled ? "Running" : "Stopped");
            if (m_enabled)
            {
                ImGui::Text("Listening on port %d", m_ui_port);
            }

            ImGui::Separator();
            std::lock_guard<std::mutex> lock(m_mutex);
            ImGui::Text("Registered components: %zu", m_components.size());
            for (auto& [f_d, comp] : m_components)
            {
                ImGui::BulletText("FD: %d [%s], %zu peers connected (Data Port: %u)",
                                  f_d,
                                  comp.protocol.c_str(),
                                  comp.peers.size(),
                                  comp.local_port);
            }
        }
        ImGui::End();
    }

    void CGPIO_Server::Set_ImGui_Context(void* context)
    {
        m_imgui_context = context;
    }

    int CGPIO_Server::Register(const char* protocol,
                               remote_protocol::comparison_func_t comp_func,
                               remote_protocol::disconnect_callback_t on_disconnect,
                               remote_protocol::handshake_result_callback_t on_handshake_result,
                               void* context)
    {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd == -1)
        {
            return -1;
        }

        struct sockaddr_in addr{ };
        addr.sin_family = AF_INET;
        addr.sin_port = 0;
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            close(fd);
            return -1;
        }

        struct sockaddr_in local_addr{ };
        socklen_t addr_len = sizeof(local_addr);
        if (getsockname(fd, (struct sockaddr*)&local_addr, &addr_len) < 0)
        {
            if (m_logging_system != nullptr)
            {
                m_logging_system->Error(
                fmt::format("CGPIO_Server [{}]: getsockname failed for FD {} in Register (errno {})", m_name, fd, errno)
                .c_str());
            }
            close(fd);
            return -1;
        }

        uint16_t local_port = ntohs(local_addr.sin_port);

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_logging_system != nullptr)
        {
            m_logging_system->Debug(
            fmt::format("CGPIO_Server [{}]: Registering component with FD {} and protocol {}. Data Port: {}",
                        m_name,
                        fd,
                        protocol,
                        local_port)
            .c_str());
        }

        m_components[fd] = { .fd = fd,
                             .local_port = local_port,
                             .protocol = protocol,
                             .comp_func = comp_func,
                             .on_disconnect = on_disconnect,
                             .on_handshake_result = on_handshake_result,
                             .context = context,
                             .peers = { } };

        return fd;
    }

    void CGPIO_Server::Unregister(int fd)
    {
        Disconnect(fd);

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_components.contains(fd))
        {
            close(fd);
            m_components.erase(fd);
        }
    }

    void CGPIO_Server::Disconnect(int fd)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_components.contains(fd))
        {
            auto& comp = m_components[fd];

            if (m_logging_system != nullptr)
            {
                m_logging_system->Info(
                fmt::format("CGPIO_Server [{}]: Disconnecting component FD {} (Data Port: {}, {} peers)",
                            m_name,
                            fd,
                            comp.local_port,
                            comp.peers.size())
                .c_str());
            }

            if (m_handshake_sock != -1)
            {
                handshake::DisconnectMessage dconn_msg{ .magic = handshake::MAGIC_BYTE,
                                                        .type = handshake::MessageType::Disconnect,
                                                        .port = comp.local_port };

                for (const auto& peer : comp.peers)
                {
                    char peer_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &peer.handshake_addr.sin_addr, peer_ip, INET_ADDRSTRLEN);

                    if (m_logging_system != nullptr)
                    {
                        m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Sending Disconnect to peer {}:{}",
                                                            m_name,
                                                            peer_ip,
                                                            peer.data_port)
                                                .c_str());
                    }

                    sendto(m_handshake_sock,
                           &dconn_msg,
                           sizeof(dconn_msg),
                           0,
                           (struct sockaddr*)&peer.handshake_addr,
                           sizeof(peer.handshake_addr));

                    // Remove from peer_to_components map
                    PeerKey key{ .ip = peer.handshake_addr.sin_addr.s_addr, .data_port = peer.data_port };
                    if (m_peer_to_components.contains(key))
                    {
                        auto& fds = m_peer_to_components[key];
                        std::erase(fds, fd);
                        if (fds.empty())
                        {
                            m_peer_to_components.erase(key);
                        }
                    }
                }
            }

            InternalDisconnect(comp);
        }
    }

    void CGPIO_Server::InternalDisconnect(ComponentContext& comp)
    {
        if (m_logging_system != nullptr)
        {
            m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: InternalDisconnect for FD {} (Data Port: {})",
                                                m_name,
                                                comp.fd,
                                                comp.local_port)
                                    .c_str());
        }

        comp.peers.clear();

        struct sockaddr unspec_addr{ };
        unspec_addr.sa_family = AF_UNSPEC;
        // This dissolves the association from any previous 'connect' call on the UDP socket.
        if (connect(comp.fd, &unspec_addr, sizeof(unspec_addr)) < 0 && errno != EAFNOSUPPORT)
        {
            if (m_logging_system != nullptr)
                m_logging_system->Error(
                fmt::format("CGPIO_Server [{}]: connect(AF_UNSPEC) failed for FD {} (errno {})", m_name, comp.fd, errno)
                .c_str());
        }
    }

    void CGPIO_Server::Init_Handshake(
    int fd, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_components.contains(fd))
        {
            if (m_logging_system != nullptr)
            {
                m_logging_system->Error(
                fmt::format("CGPIO_Server [{}]: Init_Handshake failed - FD {} not found", m_name, fd).c_str());
            }
            return;
        }

        if (m_handshake_sock == -1)
        {
            if (m_logging_system != nullptr)
            {
                m_logging_system->Error(
                fmt::format("CGPIO_Server [{}]: Cannot init handshake, server not running", m_name).c_str());
            }
            return;
        }

        auto& comp = m_components[fd];

        struct sockaddr_in remote_handshake_addr{ };
        remote_handshake_addr.sin_family = AF_INET;
        remote_handshake_addr.sin_port = htons(remote_port);
        inet_pton(AF_INET, remote_ip, &remote_handshake_addr.sin_addr);

        std::vector<uint8_t> msg_buf(sizeof(handshake::ConfMessage) + size);
        auto* conf = reinterpret_cast<handshake::ConfMessage*>(msg_buf.data());
        conf->magic = handshake::MAGIC_BYTE;
        conf->type = handshake::MessageType::Conf;
        conf->port = comp.local_port;
        conf->payload_size = static_cast<uint16_t>(size);
        std::memcpy(msg_buf.data() + sizeof(handshake::ConfMessage), comp_payload, size);

        if (m_logging_system != nullptr)
        {
            m_logging_system->Debug(
            fmt::format(
            "CGPIO_Server [{}]: Sending Conf message to {}:{} from handshake socket. Initiator data port: {}",
            m_name,
            remote_ip,
            remote_port,
            conf->port)
            .c_str());
        }

        // Control messages are sent from the handshake socket.
        ssize_t sent = sendto(m_handshake_sock,
                              msg_buf.data(),
                              msg_buf.size(),
                              0,
                              (struct sockaddr*)&remote_handshake_addr,
                              sizeof(remote_handshake_addr));

        if (sent < 0 && (m_logging_system != nullptr))
        {
            m_logging_system->Error(
            fmt::format("CGPIO_Server [{}]: Failed to send Conf message (errno {})", m_name, errno).c_str());
        }
    }

    void CGPIO_Server::Networking_Thread()
    {
        while (m_running)
        {
            struct pollfd pfd{ .fd = m_handshake_sock, .events = POLLIN, .revents = 0 };
            int ret = poll(&pfd, 1, 100);
            if (ret > 0)
            {
                if (pfd.revents & POLLIN)
                {
                    Handle_Message(m_handshake_sock);
                }
            }
            else if (ret < 0 && errno != EINTR)
            {
                if (m_logging_system != nullptr)
                {
                    m_logging_system->Error(
                    fmt::format("CGPIO_Server [{}]: poll error (errno {})", m_name, errno).c_str());
                }
                break;
            }
        }
    }

    void CGPIO_Server::Handle_Message(int sock)
    {
        struct sockaddr_in remote_addr{ };
        socklen_t addr_len = sizeof(remote_addr);
        std::array<std::uint8_t, 4096> buffer{ 0 };
        const auto received =
        recvfrom(sock, buffer.data(), buffer.size(), 0, (struct sockaddr*)&remote_addr, &addr_len);

        if (received <= 0)
        {
            return;
        }

        if (buffer[0] != handshake::MAGIC_BYTE)
        {
            return;
        }

        if (received < 2)
        {
            return;
        }

        const auto msg_type = static_cast<handshake::MessageType>(buffer[1]);

        char remote_ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &remote_addr.sin_addr, remote_ip_str, INET_ADDRSTRLEN);

        switch (msg_type)
        {
            case handshake::MessageType::Conf:
                Handle_Conf_Message(sock, remote_addr, buffer, received, remote_ip_str);
                break;

            case handshake::MessageType::Response:
                Handle_Response_Message(remote_addr, buffer, received, remote_ip_str);
                break;

            case handshake::MessageType::Disconnect:
                Handle_Disconnect_Message(remote_addr, buffer, received, remote_ip_str);
                break;

            default:
                break;
        }
    }

    void CGPIO_Server::Handle_Conf_Message(int sock,
                                           const sockaddr_in& remote_addr,
                                           const std::array<std::uint8_t, 4096>& buffer,
                                           ssize_t received,
                                           const char* remote_ip_str)
    {
        if (received < (ssize_t)sizeof(handshake::ConfMessage))
        {
            return;
        }

        const auto* conf = reinterpret_cast<const handshake::ConfMessage*>(buffer.data());

        if (m_logging_system != nullptr)
        {
            m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Received Conf from {}:{} (remote data port {})",
                                                m_name,
                                                remote_ip_str,
                                                ntohs(remote_addr.sin_port),
                                                conf->port)
                                    .c_str());
        }

        bool accepted = false;
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto& [fd, comp] : m_components)
        {
            if (comp.comp_func(comp.context, buffer.data() + sizeof(handshake::ConfMessage), conf->payload_size))
            {
                // Remove existing peer if it matches
                std::erase_if(comp.peers, [&](const auto& p) {
                    return p.handshake_addr.sin_addr.s_addr == remote_addr.sin_addr.s_addr &&
                           p.handshake_addr.sin_port == remote_addr.sin_port && p.data_port == conf->port;
                });

                comp.peers.push_back({ .handshake_addr = remote_addr, .data_port = conf->port });

                // Update peer_to_components map
                PeerKey key{ .ip = remote_addr.sin_addr.s_addr, .data_port = conf->port };
                auto& fds = m_peer_to_components[key];
                if (std::find(fds.begin(), fds.end(), fd) == fds.end())
                {
                    fds.push_back(fd);
                }

                handshake::ResponseMessage resp{ .magic = handshake::MAGIC_BYTE,
                                                 .type = handshake::MessageType::Response,
                                                 .status = 1,
                                                 .port = comp.local_port,
                                                 .initiator_port = conf->port };

                if (m_logging_system != nullptr)
                {
                    m_logging_system->Debug(
                    fmt::format(
                    "CGPIO_Server [{}]: Accepting connection, responder data port {}, initiator data port {}",
                    m_name,
                    resp.port,
                    resp.initiator_port)
                    .c_str());
                }

                sendto(sock, &resp, sizeof(resp), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));

                if (comp.on_handshake_result != nullptr)
                {
                    comp.on_handshake_result(comp.context, true, fd, remote_ip_str, conf->port);
                }

                accepted = true;
                break;
            }
        }

        if (!accepted)
        {
            if (m_logging_system != nullptr)
            {
                m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: No matching component for Conf from {}:{}",
                                                    m_name,
                                                    remote_ip_str,
                                                    ntohs(remote_addr.sin_port))
                                        .c_str());
            }

            handshake::ResponseMessage resp{ .magic = handshake::MAGIC_BYTE,
                                             .type = handshake::MessageType::Response,
                                             .status = 0,
                                             .port = 0,
                                             .initiator_port = conf->port };
            sendto(sock, &resp, sizeof(resp), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
        }
    }

    void CGPIO_Server::Handle_Response_Message(const sockaddr_in& remote_addr,
                                               const std::array<std::uint8_t, 4096>& buffer,
                                               ssize_t received,
                                               const char* remote_ip_str)
    {
        if (received < (ssize_t)sizeof(handshake::ResponseMessage))
        {
            return;
        }

        const auto* resp = reinterpret_cast<const handshake::ResponseMessage*>(buffer.data());
        if (m_logging_system != nullptr)
        {
            m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Received Response from {}:{} (status {}, "
                                                "responder data port {}, initiator data port {})",
                                                m_name,
                                                remote_ip_str,
                                                ntohs(remote_addr.sin_port),
                                                (int)resp->status,
                                                resp->port,
                                                resp->initiator_port)
                                    .c_str());
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        bool found = false;
        for (auto& [fd, comp] : m_components)
        {
            if (comp.local_port == resp->initiator_port)
            {
                if (resp->status == 1)
                {
                    std::erase_if(comp.peers, [&](const auto& p) {
                        return p.handshake_addr.sin_addr.s_addr == remote_addr.sin_addr.s_addr &&
                               p.handshake_addr.sin_port == remote_addr.sin_port && p.data_port == resp->port;
                    });
                    comp.peers.push_back({ .handshake_addr = remote_addr, .data_port = resp->port });

                    // Update peer_to_components map
                    PeerKey key{ .ip = remote_addr.sin_addr.s_addr, .data_port = resp->port };
                    auto& fds = m_peer_to_components[key];
                    if (std::find(fds.begin(), fds.end(), fd) == fds.end())
                    {
                        fds.push_back(fd);
                    }
                }

                if (comp.on_handshake_result != nullptr)
                {
                    comp.on_handshake_result(comp.context, resp->status == 1, fd, remote_ip_str, resp->port);
                }

                found = true;
                break;
            }
        }
        if (!found && (m_logging_system != nullptr))
        {
            m_logging_system->Debug(
            fmt::format("CGPIO_Server [{}]: Could not find component with initiator port {} for Response",
                        m_name,
                        resp->initiator_port)
            .c_str());
        }
    }

    void CGPIO_Server::Handle_Disconnect_Message(const sockaddr_in& remote_addr,
                                                 const std::array<std::uint8_t, 4096>& buffer,
                                                 ssize_t received,
                                                 const char* remote_ip_str)
    {
        if (received < (ssize_t)sizeof(handshake::DisconnectMessage))
        {
            return;
        }

        const auto* dmsg = reinterpret_cast<const handshake::DisconnectMessage*>(buffer.data());
        if (m_logging_system != nullptr)
        {
            m_logging_system->Debug(
            fmt::format("CGPIO_Server [{}]: Received Disconnect from {}:{} (remote data port {})",
                        m_name,
                        remote_ip_str,
                        ntohs(remote_addr.sin_port),
                        dmsg->port)
            .c_str());
        }

        const PeerKey key{ .ip = remote_addr.sin_addr.s_addr, .data_port = dmsg->port };
        const auto fds_to_notify = Update_Peers_On_Disconnect(key, remote_addr, dmsg->port);

        Notify_Components_Of_Disconnect(fds_to_notify, remote_ip_str, dmsg->port);
    }

    std::vector<int> CGPIO_Server::Update_Peers_On_Disconnect(const PeerKey& key,
                                                              const sockaddr_in& remote_addr,
                                                              uint16_t remote_data_port)
    {
        std::vector<int> fds_to_notify;
        std::lock_guard<std::mutex> lock(m_mutex);

        if (m_peer_to_components.contains(key))
        {
            fds_to_notify = m_peer_to_components[key];

            if (m_logging_system != nullptr)
            {
                m_logging_system->Debug(
                fmt::format("CGPIO_Server [{}]: Found {} components to notify", m_name, fds_to_notify.size()).c_str());
            }

            m_peer_to_components.erase(key);

            for (int fd : fds_to_notify)
            {
                if (m_components.contains(fd))
                {
                    auto& comp = m_components[fd];
                    std::erase_if(comp.peers, [&](const auto& p) {
                        return p.handshake_addr.sin_addr.s_addr == remote_addr.sin_addr.s_addr &&
                               p.handshake_addr.sin_port == remote_addr.sin_port && p.data_port == remote_data_port;
                    });

                    if (comp.peers.empty())
                    {
                        if (m_logging_system != nullptr)
                        {
                            m_logging_system->Debug(
                            fmt::format(
                            "CGPIO_Server [{}]: Component FD {} has no more peers, calling InternalDisconnect",
                            m_name,
                            fd)
                            .c_str());
                        }
                        InternalDisconnect(comp);
                    }
                }
            }
        }
        else if (m_logging_system != nullptr)
        {
            m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: No components found for peer IP {} port {}",
                                                m_name,
                                                inet_ntoa(remote_addr.sin_addr),
                                                remote_data_port)
                                    .c_str());
        }

        return fds_to_notify;
    }

    void CGPIO_Server::Notify_Components_Of_Disconnect(const std::vector<int>& fds_to_notify,
                                                       const char* remote_ip_str,
                                                       uint16_t remote_data_port)
    {
        for (int fd : fds_to_notify)
        {
            remote_protocol::disconnect_callback_t on_dconn = nullptr;
            void* ctx = nullptr;

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                if (m_components.contains(fd))
                {
                    on_dconn = m_components[fd].on_disconnect;
                    ctx = m_components[fd].context;
                }
            }

            if (on_dconn != nullptr)
            {
                if (m_logging_system != nullptr)
                {
                    m_logging_system->Debug(
                    fmt::format("CGPIO_Server [{}]: Notifying component FD {} of peer disconnect", m_name, fd).c_str());
                }
                on_dconn(ctx, remote_ip_str, remote_data_port);
            }
        }
    }
}

#include "zero_mate/gpio_server_abi.hpp"

extern "C"
{
    zero_mate::peripheral::TGPIOServerABI Get_GPIO_Server_ABI()
    {
        return { .register_channel =
                 [](const char* protocol,
                    zero_mate::remote_protocol::comparison_func_t comp_func,
                    zero_mate::remote_protocol::disconnect_callback_t on_disconnect,
                    zero_mate::remote_protocol::handshake_result_callback_t on_handshake_result,
                    void* context) {
                     return zero_mate::peripheral::CGPIO_Server::s_instance->Register(protocol,
                                                                                      comp_func,
                                                                                      on_disconnect,
                                                                                      on_handshake_result,
                                                                                      context);
                 },
                 .unregister_channel = [](int fd) { zero_mate::peripheral::CGPIO_Server::s_instance->Unregister(fd); },
                 .disconnect_channel = [](int fd) { zero_mate::peripheral::CGPIO_Server::s_instance->Disconnect(fd); },
                 .init_handshake =
                 [](int fd, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size) {
                     zero_mate::peripheral::CGPIO_Server::s_instance->Init_Handshake(fd,
                                                                                     remote_ip,
                                                                                     remote_port,
                                                                                     comp_payload,
                                                                                     size);
                 } };
    }

    zero_mate::IExternal_Peripheral::NInit_Status
    Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
                      const char* const name,
                      const uint32_t* const /* connection */,
                      size_t /* pin_count */,
                      zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t /* set_pin */,
                      zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t /* read_pin */,
                      zero_mate::IExternal_Peripheral::Halt_t /* halt */,
                      zero_mate::IExternal_Peripheral::Start_t /* start */,
                      zero_mate::utils::CLogging_System* logging_system)
    {
        if (zero_mate::peripheral::CGPIO_Server::s_instance != nullptr)
        {
            return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
        }

        *peripheral = new (std::nothrow) zero_mate::peripheral::CGPIO_Server(name, logging_system);

        return (*peripheral == nullptr) ? zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error
                                        : zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
