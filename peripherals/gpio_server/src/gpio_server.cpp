#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <vector>

#include "fmt/format.h"
#include "imgui.h"
#include "gpio_server.hpp"

namespace zero_mate::peripheral
{
    CGPIO_Server::CGPIO_Server(const std::string& name,
                               uint16_t handshake_port,
                               IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                               IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                               IExternal_Peripheral::Halt_t halt,
                               IExternal_Peripheral::Start_t start,
                               utils::CLogging_System* logging_system)
    : m_name{ name }
    , m_handshake_port{ handshake_port }
    , m_read_pin{ read_pin }
    , m_set_pin{ set_pin }
    , m_halt{ halt }
    , m_start{ start }
    , m_logging_system{ logging_system }
    , m_ui_port{ (int)handshake_port }
    {
        {
            std::lock_guard<std::mutex> lock(s_instances_mutex);
            s_instances.push_back(this);
        }
    }

    CGPIO_Server::~CGPIO_Server()
    {
        StopServer();

        {
            std::lock_guard<std::mutex> lock(s_instances_mutex);
            std::erase(s_instances, this);
            
            auto it = s_fd_to_instance.begin();
            while (it != s_fd_to_instance.end())
            {
                if (it->second == this)
                    it = s_fd_to_instance.erase(it);
                else
                    ++it;
            }
        }
    }

    void CGPIO_Server::StartServer()
    {
        if (m_enabled)
            return;

        m_handshake_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (m_handshake_sock == -1)
        {
            if (m_logging_system)
                m_logging_system->Error(fmt::format("CGPIO_Server [{}]: Failed to create handshake socket", m_name).c_str());
            return;
        }

        struct sockaddr_in addr{ .sin_family = AF_INET,
                                 .sin_port = htons((uint16_t)m_ui_port),
                                 .sin_addr = { .s_addr = INADDR_ANY } };
        if (bind(m_handshake_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            if (m_logging_system)
                m_logging_system->Error(fmt::format("CGPIO_Server [{}]: Failed to bind handshake socket to port {}", m_name, m_ui_port).c_str());
            close(m_handshake_sock);
            m_handshake_sock = -1;
            return;
        }

        m_running = true;
        m_enabled = true;
        m_networking_thread = std::thread(&CGPIO_Server::Networking_Thread, this);
        
        if (m_logging_system)
            m_logging_system->Info(fmt::format("CGPIO_Server [{}]: Server started on port {}", m_name, m_ui_port).c_str());
    }

    void CGPIO_Server::StopServer()
    {
        if (!m_enabled)
            return;

        if (m_logging_system)
            m_logging_system->Info(fmt::format("CGPIO_Server [{}]: Stopping server", m_name).c_str());

        m_running = false;
        if (m_networking_thread.joinable())
            m_networking_thread.join();

        if (m_handshake_sock != -1)
        {
            close(m_handshake_sock);
            m_handshake_sock = -1;
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        std::vector<int> fds_to_remove;
        for (auto& [fd, comp] : m_components)
        {
            if (m_logging_system)
                m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Disconnecting component FD {}", m_name, fd).c_str());
            
            if (comp.on_disconnect)
                comp.on_disconnect(comp.context);
            
            if (fd != -1)
                close(fd);
            fds_to_remove.push_back(fd);
        }

        {
            std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
            for (int fd : fds_to_remove)
                s_fd_to_instance.erase(fd);
        }
        m_components.clear();
        m_enabled = false;
    }

    void CGPIO_Server::Render()
    {
        if (m_imgui_context)
            ImGui::SetCurrentContext(static_cast<ImGuiContext*>(m_imgui_context));

        if (ImGui::Begin(m_name.c_str()))
        {
            ImGui::InputInt("Server Port", &m_ui_port);
            if (m_enabled)
            {
                if (ImGui::Button("Stop Server"))
                    StopServer();
            }
            else
            {
                if (ImGui::Button("Start Server"))
                    StartServer();
            }

            ImGui::Separator();
            ImGui::Text("Status: %s", m_enabled ? "Running" : "Stopped");
            if (m_enabled)
                ImGui::Text("Listening on port %d", m_ui_port);
            
            ImGui::Separator();
            std::lock_guard<std::mutex> lock(m_mutex);
            ImGui::Text("Registered components: %zu", m_components.size());
            for (auto& [fd, comp] : m_components)
            {
                ImGui::BulletText("FD: %d [%s], %s", fd, comp.protocol.c_str(), comp.remote_data_port != 0 ? "Connected" : "Waiting");
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
                               remote_protocol::receive_callback_t on_receive,
                               remote_protocol::disconnect_callback_t on_disconnect,
                               remote_protocol::handshake_result_callback_t on_handshake_result,
                               void* context)
    {
        std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
        if (s_instances.empty())
            return -1;

        CGPIO_Server* instance = s_instances.back();
        
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd == -1)
            return -1;

        struct sockaddr_in addr{ .sin_family = AF_INET, .sin_port = 0, .sin_addr = { .s_addr = INADDR_ANY } };
        if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            close(fd);
            return -1;
        }

        std::lock_guard<std::mutex> lock(instance->m_mutex);
        if (instance->m_logging_system)
            instance->m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Registering component with FD {} and protocol {}", instance->m_name, fd, protocol).c_str());
        
        instance->m_components[fd] = { fd, protocol, comp_func, on_receive, on_disconnect, on_handshake_result, context };
        s_fd_to_instance[fd] = instance;
        
        struct sockaddr_in local_addr{ };
        socklen_t addr_len = sizeof(local_addr);
        getsockname(fd, (struct sockaddr*)&local_addr, &addr_len);

        if (instance->m_logging_system)
            instance->m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Component registered. FD: {}, Protocol: {}, Local Data Port: {}", 
                                              instance->m_name, fd, protocol, ntohs(local_addr.sin_port)).c_str());

        return fd;
    }

    void CGPIO_Server::Unregister(int fd)
    {
        std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
        if (s_fd_to_instance.contains(fd))
        {
            CGPIO_Server* instance = s_fd_to_instance[fd];
            std::lock_guard<std::mutex> lock(instance->m_mutex);
            if (instance->m_components.contains(fd))
            {
                auto& comp = instance->m_components[fd];
                if (comp.remote_data_port != 0 && instance->m_handshake_sock != -1)
                {
                    struct sockaddr_in local_data_addr{ };
                    socklen_t data_addr_len = sizeof(local_data_addr);
                    getsockname(fd, (struct sockaddr*)&local_data_addr, &data_addr_len);

                    handshake::DisconnectMessage dm{ .magic = handshake::MAGIC_BYTE,
                                                     .type = handshake::MessageType::Disconnect,
                                                     .port = ntohs(local_data_addr.sin_port) };
                    sendto(instance->m_handshake_sock,
                           &dm,
                           sizeof(dm),
                           0,
                           (struct sockaddr*)&comp.remote_handshake_addr,
                           sizeof(comp.remote_handshake_addr));
                }
                close(fd);
                instance->m_components.erase(fd);
            }
            s_fd_to_instance.erase(fd);
        }
    }

    void CGPIO_Server::Send(int fd, const void* data, size_t size)
    {
        send(fd, data, size, 0);
    }

    void CGPIO_Server::Init_Handshake(
    int fd, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size)
    {
        CGPIO_Server* instance = nullptr;
        {
            std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
            if (!s_fd_to_instance.contains(fd))
                return;
            instance = s_fd_to_instance[fd];
        }

        std::lock_guard<std::mutex> lock(instance->m_mutex);
        if (!instance->m_components.contains(fd))
            return;

        if (instance->m_handshake_sock == -1)
        {
            if (instance->m_logging_system)
                instance->m_logging_system->Error(fmt::format("CGPIO_Server [{}]: Cannot init handshake, server not running", instance->m_name).c_str());
            return;
        }

        auto& comp = instance->m_components[fd];

        comp.remote_handshake_addr.sin_family = AF_INET;
        comp.remote_handshake_addr.sin_port = htons(remote_port);
        inet_pton(AF_INET, remote_ip, &comp.remote_handshake_addr.sin_addr);

        struct sockaddr_in local_data_addr{ };
        socklen_t data_addr_len = sizeof(local_data_addr);
        getsockname(fd, (struct sockaddr*)&local_data_addr, &data_addr_len);

        std::vector<uint8_t> msg_buf(sizeof(handshake::ConfMessage) + size);
        auto* conf = reinterpret_cast<handshake::ConfMessage*>(msg_buf.data());
        conf->magic = handshake::MAGIC_BYTE;
        conf->type = handshake::MessageType::Conf;
        conf->port = ntohs(local_data_addr.sin_port);
        conf->payload_size = static_cast<uint16_t>(size);
        std::memcpy(msg_buf.data() + sizeof(handshake::ConfMessage), comp_payload, size);

        if (instance->m_logging_system)
            instance->m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Sending Conf message to {}:{} from handshake socket (initiator port {})", 
                                              instance->m_name, remote_ip, remote_port, conf->port).c_str());

        // Control messages are sent from the handshake socket.
        ssize_t sent = sendto(instance->m_handshake_sock,
                              msg_buf.data(),
                              msg_buf.size(),
                              0,
                              (struct sockaddr*)&comp.remote_handshake_addr,
                              sizeof(comp.remote_handshake_addr));
        
        if (sent < 0 && instance->m_logging_system)
            instance->m_logging_system->Error(fmt::format("CGPIO_Server [{}]: Failed to send Conf message (errno {})", instance->m_name, errno).c_str());
    }

    void CGPIO_Server::Networking_Thread()
    {
        while (m_running)
        {
            struct pollfd pfd{ m_handshake_sock, POLLIN, 0 };
            int ret = poll(&pfd, 1, 100);
            if (ret > 0)
            {
                if (pfd.revents & POLLIN)
                    Handle_Handshake(m_handshake_sock);
            }
            else if (ret < 0 && errno != EINTR)
            {
                if (m_logging_system)
                    m_logging_system->Error(fmt::format("CGPIO_Server [{}]: poll error (errno {})", m_name, errno).c_str());
                break;
            }
        }
    }

    void CGPIO_Server::Handle_Handshake(int sock)
    {
        struct sockaddr_in remote_addr{ };
        socklen_t addr_len = sizeof(remote_addr);
        uint8_t buffer[4096];
        ssize_t received = recvfrom(sock, buffer, sizeof(buffer), 0, (struct sockaddr*)&remote_addr, &addr_len);
        if (received < (ssize_t)sizeof(handshake::ConfMessage))
            return;

        auto* conf = reinterpret_cast<handshake::ConfMessage*>(buffer);
        if (conf->magic != handshake::MAGIC_BYTE)
            return;

        char remote_ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &remote_addr.sin_addr, remote_ip_str, INET_ADDRSTRLEN);

        if (conf->type == handshake::MessageType::Conf)
        {
            if (m_logging_system)
                m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Received Conf from {}:{} (remote data port {})", m_name, remote_ip_str, ntohs(remote_addr.sin_port), conf->port).c_str());

            bool accepted = false;
            std::lock_guard<std::mutex> lock(m_mutex);
            for (auto& [fd, comp] : m_components)
            {
                if (comp.comp_func(comp.context, buffer + sizeof(handshake::ConfMessage), conf->payload_size))
                {
                    comp.remote_handshake_addr = remote_addr;
                    comp.remote_data_port = conf->port;

                    struct sockaddr_in local_data_addr{ };
                    socklen_t data_addr_len = sizeof(local_data_addr);
                    getsockname(fd, (struct sockaddr*)&local_data_addr, &data_addr_len);

                    handshake::ResponseMessage resp{ .magic = handshake::MAGIC_BYTE,
                                                     .type = handshake::MessageType::Response,
                                                     .status = 1,
                                                     .port = ntohs(local_data_addr.sin_port),
                                                     .initiator_port = conf->port };
                    
                    if (m_logging_system)
                        m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Accepting connection, responder data port {}, initiator data port {}", m_name, resp.port, resp.initiator_port).c_str());

                    sendto(sock, &resp, sizeof(resp), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));

                    if (comp.on_handshake_result)
                        comp.on_handshake_result(comp.context, true, fd, remote_ip_str, conf->port);
                    
                    accepted = true;
                    break;
                }
            }

            if (!accepted)
            {
                if (m_logging_system)
                    m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: No matching component for Conf from {}:{}", m_name, remote_ip_str, ntohs(remote_addr.sin_port)).c_str());

                handshake::ResponseMessage resp{ .magic = handshake::MAGIC_BYTE,
                                                 .type = handshake::MessageType::Response,
                                                 .status = 0,
                                                 .port = 0,
                                                 .initiator_port = conf->port };
                sendto(sock, &resp, sizeof(resp), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
            }
        }
        else if (conf->type == handshake::MessageType::Response)
        {
            auto* resp = reinterpret_cast<handshake::ResponseMessage*>(buffer);
            if (m_logging_system)
                m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Received Response from {}:{} (status {}, responder data port {}, initiator data port {})", 
                                        m_name, remote_ip_str, ntohs(remote_addr.sin_port), (int)resp->status, resp->port, resp->initiator_port).c_str());

            std::lock_guard<std::mutex> lock(m_mutex);
            bool found = false;
            for (auto& [fd, comp] : m_components)
            {
                struct sockaddr_in local_data_addr{ };
                socklen_t data_addr_len = sizeof(local_data_addr);
                getsockname(fd, (struct sockaddr*)&local_data_addr, &data_addr_len);

                if (ntohs(local_data_addr.sin_port) == resp->initiator_port)
                {
                    if (resp->status == 1)
                    {
                        comp.remote_data_port = resp->port;
                        comp.remote_handshake_addr = remote_addr;
                    }
                    
                    if (comp.on_handshake_result)
                        comp.on_handshake_result(comp.context, resp->status == 1, fd, remote_ip_str, resp->port);
                    
                    found = true;
                    break;
                }
            }
            if (!found && m_logging_system)
                m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Could not find component with initiator port {} for Response", m_name, resp->initiator_port).c_str());
        }
        else if (conf->type == handshake::MessageType::Disconnect)
        {
            auto* dm = reinterpret_cast<handshake::DisconnectMessage*>(buffer);
            if (m_logging_system)
                m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Received Disconnect from {}:{} (remote data port {})", m_name, remote_ip_str, ntohs(remote_addr.sin_port), dm->port).c_str());

            std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
            std::lock_guard<std::mutex> lock(m_mutex);
            for (auto& [fd, comp] : m_components)
            {
                if (comp.remote_handshake_addr.sin_addr.s_addr == remote_addr.sin_addr.s_addr &&
                    comp.remote_handshake_addr.sin_port == remote_addr.sin_port && comp.remote_data_port == dm->port)
                {
                    if (m_logging_system)
                        m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Component FD {} disconnected remotely", m_name, fd).c_str());

                    if (comp.on_disconnect)
                        comp.on_disconnect(comp.context);
                    
                    s_fd_to_instance.erase(fd);
                    close(fd);
                    m_components.erase(fd);
                    break;
                }
            }
        }
    }
}

extern "C"
{
    int server_register_channel(const char* protocol,
                                zero_mate::remote_protocol::comparison_func_t comp_func,
                                zero_mate::remote_protocol::receive_callback_t on_receive,
                                zero_mate::remote_protocol::disconnect_callback_t on_disconnect,
                                zero_mate::remote_protocol::handshake_result_callback_t on_handshake_result,
                                void* context)
    {
        return zero_mate::peripheral::CGPIO_Server::Register(protocol,
                                                             comp_func,
                                                             on_receive,
                                                             on_disconnect,
                                                             on_handshake_result,
                                                             context);
    }

    void server_unregister_channel(int fd)
    {
        zero_mate::peripheral::CGPIO_Server::Unregister(fd);
    }
    void server_send_data(int fd, const void* data, size_t size)
    {
        zero_mate::peripheral::CGPIO_Server::Send(fd, data, size);
    }
    void server_init_handshake(
    int fd, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size)
    {
        zero_mate::peripheral::CGPIO_Server::Init_Handshake(fd, remote_ip, remote_port, comp_payload, size);
    }

    zero_mate::IExternal_Peripheral::NInit_Status
    Create_Peripheral(zero_mate::IExternal_Peripheral** peripheral,
                      const char* const name,
                      const uint32_t* const connection,
                      size_t pin_count,
                      zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                      zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                      zero_mate::IExternal_Peripheral::Halt_t halt,
                      zero_mate::IExternal_Peripheral::Start_t start,
                      zero_mate::utils::CLogging_System* logging_system)
    {
        if (pin_count != 1)
            return zero_mate::IExternal_Peripheral::NInit_Status::GPIO_Mismatch;
        *peripheral = new (std::nothrow)
        zero_mate::peripheral::CGPIO_Server(name, connection[0], read_pin, set_pin, halt, start, logging_system);
        return (*peripheral == nullptr) ? zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error
                                        : zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
