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
    {
        {
            std::lock_guard<std::mutex> lock(s_instances_mutex);
            s_instances.push_back(this);
        }
        m_networking_thread = std::thread(&CGPIO_Server::Networking_Thread, this);
    }

    CGPIO_Server::~CGPIO_Server()
    {
        m_running = false;
        if (m_networking_thread.joinable())
        {
            m_networking_thread.join();
        }

        {
            std::lock_guard<std::mutex> lock(s_instances_mutex);
            std::erase(s_instances, this);
            
            // Remove all IDs belonging to this instance from the global map
            auto it = s_id_to_instance.begin();
            while (it != s_id_to_instance.end())
            {
                if (it->second == this)
                    it = s_id_to_instance.erase(it);
                else
                    ++it;
            }
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto& [id, comp] : m_components)
        {
            for (int fd : comp.sockfds)
                if (fd != -1)
                    close(fd);
        }
    }

    void CGPIO_Server::Render()
    {
        if (m_imgui_context)
            ImGui::SetCurrentContext(static_cast<ImGuiContext*>(m_imgui_context));

        if (ImGui::Begin(m_name.c_str()))
        {
            ImGui::Text("GPIOServer on port %d", m_handshake_port);
            ImGui::Separator();
            
            std::lock_guard<std::mutex> lock(m_mutex);
            ImGui::Text("Registered components: %zu", m_components.size());
            for (auto& [id, comp] : m_components)
            {
                ImGui::BulletText("ID: %u [%s], Conns: %zu", id, comp.protocol.c_str(), comp.sockfds.size());
            }
        }
        ImGui::End();
    }

    void CGPIO_Server::Set_ImGui_Context(void* context)
    {
        m_imgui_context = context;
    }

    uint32_t CGPIO_Server::Register(const char* protocol,
                                    remote_protocol::comparison_func_t comp_func,
                                    remote_protocol::receive_callback_t on_receive,
                                    remote_protocol::disconnect_callback_t on_disconnect,
                                    remote_protocol::handshake_result_callback_t on_handshake_result,
                                    void* context)
    {
        std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
        if (s_instances.empty())
            return 0;

        // Since we can't easily tell which instance is which from the C API,
        // we'll use the last created instance for registration.
        // In a typical ZeroMate setup, the peripheral is created right after the server.
        CGPIO_Server* instance = s_instances.back();
        
        std::lock_guard<std::mutex> lock(instance->m_mutex);
        uint32_t id = instance->m_next_id++;
        
        // Ensure ID is globally unique across instances by using instance pointer bits
        // or just a global counter. Let's use a global counter for simplicity.
        static uint32_t s_global_next_id = 1;
        uint32_t global_id = s_global_next_id++;

        if (instance->m_logging_system)
            instance->m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Registering component {} with protocol {}", instance->m_name, global_id, protocol).c_str());
        
        instance->m_components[global_id] = { global_id, protocol, comp_func, on_receive, on_disconnect, on_handshake_result, context };
        s_id_to_instance[global_id] = instance;
        
        return global_id;
    }

    void CGPIO_Server::Unregister(uint32_t id)
    {
        std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
        if (s_id_to_instance.contains(id))
        {
            CGPIO_Server* instance = s_id_to_instance[id];
            std::lock_guard<std::mutex> lock(instance->m_mutex);
            if (instance->m_components.contains(id))
            {
                for (int fd : instance->m_components[id].sockfds)
                    if (fd != -1)
                        close(fd);
                instance->m_components.erase(id);
            }
            s_id_to_instance.erase(id);
        }
    }

    void CGPIO_Server::Send(uint32_t id, const void* data, size_t size)
    {
        ComponentContext comp_copy;
        CGPIO_Server* instance = nullptr;

        {
            std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
            if (!s_id_to_instance.contains(id))
                return;
            instance = s_id_to_instance[id];
        }

        {
            std::lock_guard<std::mutex> lock(instance->m_mutex);
            if (!instance->m_components.contains(id))
                return;
            comp_copy = instance->m_components[id];
        }

        if (instance->m_logging_system)
        {
            std::string hex_dump;
            const auto* bytes = static_cast<const uint8_t*>(data);
            for (size_t i = 0; i < size; ++i)
                hex_dump += fmt::format("{:02X} ", bytes[i]);
            
            instance->m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Sending {} bytes to component {} [{}]: {}", 
                                              instance->m_name, size, id, comp_copy.protocol, hex_dump).c_str());
        }

        for (size_t i = 0; i < comp_copy.sockfds.size(); ++i)
        {
            if (comp_copy.sockfds[i] != -1)
            {
                sendto(comp_copy.sockfds[i],
                       data,
                       size,
                       0,
                       (struct sockaddr*)&comp_copy.remote_addrs[i],
                       sizeof(comp_copy.remote_addrs[i]));
            }
        }
    }

    void CGPIO_Server::Init_Handshake(
    uint32_t id, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size)
    {
        CGPIO_Server* instance = nullptr;
        {
            std::lock_guard<std::mutex> lock_instances(s_instances_mutex);
            if (!s_id_to_instance.contains(id))
                return;
            instance = s_id_to_instance[id];
        }

        std::lock_guard<std::mutex> lock(instance->m_mutex);
        if (!instance->m_components.contains(id))
            return;
        auto& comp = instance->m_components[id];

        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock == -1)
            return;

        struct sockaddr_in local_addr{ .sin_family = AF_INET, .sin_port = 0, .sin_addr = { .s_addr = INADDR_ANY } };
        bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr));

        socklen_t addr_len = sizeof(local_addr);
        getsockname(sock, (struct sockaddr*)&local_addr, &addr_len);

        struct sockaddr_in remote_addr{ };
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_port = htons(remote_port);
        inet_pton(AF_INET, remote_ip, &remote_addr.sin_addr);

        std::vector<uint8_t> msg_buf(sizeof(handshake::ConfMessage) + size);
        auto* conf = reinterpret_cast<handshake::ConfMessage*>(msg_buf.data());
        conf->magic = handshake::MAGIC_BYTE;
        conf->type = handshake::MessageType::Conf;
        conf->port = ntohs(local_addr.sin_port);
        conf->payload_size = static_cast<uint16_t>(size);
        std::memcpy(msg_buf.data() + sizeof(handshake::ConfMessage), comp_payload, size);

        if (instance->m_logging_system)
            instance->m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Sending Conf message to {}:{}", instance->m_name, remote_ip, remote_port).c_str());

        sendto(sock,
               msg_buf.data(),
               msg_buf.size(),
               0,
               (struct sockaddr*)&remote_addr,
               sizeof(remote_addr));

        comp.sockfds.push_back(sock);
        comp.remote_addrs.push_back(remote_addr);
        comp.connected = false;
    }

    void CGPIO_Server::Networking_Thread()
    {
        int handshake_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (handshake_sock == -1)
            return;

        struct sockaddr_in addr{ .sin_family = AF_INET,
                                 .sin_port = htons(m_handshake_port),
                                 .sin_addr = { .s_addr = INADDR_ANY } };
        if (bind(handshake_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        {
            close(handshake_sock);
            return;
        }

        while (m_running)
        {
            std::vector<struct pollfd> fds;
            fds.push_back({ handshake_sock, POLLIN, 0 });
            std::vector<std::pair<uint32_t, size_t>> fd_to_comp;
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                for (auto& [id, comp] : m_components)
                {
                    for (size_t i = 0; i < comp.sockfds.size(); ++i)
                    {
                        if (comp.sockfds[i] != -1)
                        {
                            fds.push_back({ comp.sockfds[i], POLLIN, 0 });
                            fd_to_comp.push_back({ id, i });
                        }
                    }
                }
            }

            if (poll(fds.data(), fds.size(), 10) > 0)
            {
                if (fds[0].revents & POLLIN)
                    Handle_Handshake(handshake_sock);
                for (size_t i = 1; i < fds.size(); ++i)
                    if (fds[i].revents & POLLIN)
                        Handle_Data(fds[i].fd, fd_to_comp[i - 1].first);
            }
        }
        close(handshake_sock);
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
        if (conf->magic != handshake::MAGIC_BYTE || conf->type != handshake::MessageType::Conf)
            return;

        bool accepted = false;
        std::lock_guard<std::mutex> lock(m_mutex);
        for (auto& [id, comp] : m_components)
        {
            if (comp.comp_func(comp.context, buffer + sizeof(handshake::ConfMessage), conf->payload_size))
            {
                struct sockaddr_in remote_data_addr = remote_addr;
                remote_data_addr.sin_port = htons(conf->port);

                int data_sock = socket(AF_INET, SOCK_DGRAM, 0);
                struct sockaddr_in local_data_addr{ .sin_family = AF_INET,
                                                    .sin_port = 0,
                                                    .sin_addr = { .s_addr = INADDR_ANY } };
                bind(data_sock, (struct sockaddr*)&local_data_addr, sizeof(local_data_addr));
                socklen_t data_addr_len = sizeof(local_data_addr);
                getsockname(data_sock, (struct sockaddr*)&local_data_addr, &data_addr_len);

                handshake::ResponseMessage resp{ .magic = handshake::MAGIC_BYTE,
                                                 .type = handshake::MessageType::Response,
                                                 .status = 1,
                                                 .port = ntohs(local_data_addr.sin_port) };
                sendto(sock, &resp, sizeof(resp), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));

                comp.sockfds.push_back(data_sock);
                comp.remote_addrs.push_back(remote_data_addr);
                if (comp.on_handshake_result)
                    comp.on_handshake_result(comp.context, true);
                accepted = true;
                break;
            }
        }

        if (!accepted)
        {
            handshake::ResponseMessage resp{ .magic = handshake::MAGIC_BYTE,
                                             .type = handshake::MessageType::Response,
                                             .status = 0,
                                             .port = 0 };
            sendto(sock, &resp, sizeof(resp), 0, (struct sockaddr*)&remote_addr, sizeof(remote_addr));
        }
    }

    void CGPIO_Server::Handle_Data(int sock, uint32_t component_id)
    {
        uint8_t buffer[4096];
        ssize_t received = recv(sock, buffer, sizeof(buffer), 0);
        if (received <= 0)
            return;

        ComponentContext comp_copy;
        bool found = false;
        size_t sock_idx = 0;

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (!m_components.contains(component_id))
                return;

            auto& comp = m_components[component_id];
            for (size_t i = 0; i < comp.sockfds.size(); ++i)
            {
                if (comp.sockfds[i] == sock)
                {
                    found = true;
                    sock_idx = i;
                    break;
                }
            }

            if (!found)
                return;

            // Check if this is a control message that needs state update under lock
            auto* resp = reinterpret_cast<handshake::ResponseMessage*>(buffer);
            if (received >= (ssize_t)sizeof(handshake::ResponseMessage) && resp->magic == handshake::MAGIC_BYTE &&
                resp->type == handshake::MessageType::Response)
            {
                if (resp->status == 1)
                {
                    comp.connected = true;
                    comp.remote_addrs[sock_idx].sin_port = htons(resp->port);
                }
            }

            // Copy necessary info for callbacks
            comp_copy = comp;
        }

        if (m_logging_system)
        {
            std::string hex_dump;
            for (ssize_t i = 0; i < received; ++i)
            {
                hex_dump += fmt::format("{:02X} ", buffer[i]);
            }

            m_logging_system->Debug(fmt::format("CGPIO_Server [{}]: Received {} bytes for component {} [{}]: {}", 
                                    m_name, received, component_id, comp_copy.protocol, hex_dump).c_str());
        }

        // Now we can call callbacks without holding the lock
        auto* resp = reinterpret_cast<handshake::ResponseMessage*>(buffer);
        if (received >= (ssize_t)sizeof(handshake::ResponseMessage) && resp->magic == handshake::MAGIC_BYTE &&
            resp->type == handshake::MessageType::Response)
        {
            if (comp_copy.on_handshake_result)
                comp_copy.on_handshake_result(comp_copy.context, resp->status == 1);
        }
        else
        {
            auto* dm = reinterpret_cast<handshake::DisconnectMessage*>(buffer);
            if (received >= (ssize_t)sizeof(handshake::DisconnectMessage) && dm->magic == handshake::MAGIC_BYTE &&
                dm->type == handshake::MessageType::Disconnect)
            {
                if (comp_copy.on_disconnect)
                    comp_copy.on_disconnect(comp_copy.context);
            }
            else if (comp_copy.on_receive)
            {
                comp_copy.on_receive(comp_copy.context, buffer, received);
            }
        }
    }
}

extern "C"
{
    uint32_t server_register_channel(const char* protocol,
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

    void server_unregister_channel(uint32_t id)
    {
        zero_mate::peripheral::CGPIO_Server::Unregister(id);
    }
    void server_send_data(uint32_t id, const void* data, size_t size)
    {
        zero_mate::peripheral::CGPIO_Server::Send(id, data, size);
    }
    void server_init_handshake(
    uint32_t id, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size)
    {
        zero_mate::peripheral::CGPIO_Server::Init_Handshake(id, remote_ip, remote_port, comp_payload, size);
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
