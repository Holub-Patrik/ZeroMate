#pragma once

#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <netinet/in.h>

#include "zero_mate/external_peripheral.hpp"
#include "zero_mate/RemoteProtocol.hpp"
#include "zero_mate/Protocol.hpp"

namespace zero_mate::peripheral
{
    class CGPIO_Server final : public IExternal_Peripheral
    {
    public:
        static inline CGPIO_Server* s_instance{ nullptr };

        CGPIO_Server(std::string name, utils::CLogging_System* logging_system);

        ~CGPIO_Server() override;

        void Render() override;
        void Set_ImGui_Context(void* context) override;

        // Exported symbols
        int Register(const char* protocol,
                     remote_protocol::comparison_func_t comp_func,
                     remote_protocol::disconnect_callback_t on_disconnect,
                     remote_protocol::handshake_result_callback_t on_handshake_result,
                     void* context);

        void Unregister(int fd);
        void Disconnect(int fd);
        void Init_Handshake(int fd, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size);

    private:
        struct PeerKey
        {
            uint32_t ip;
            uint16_t data_port;

            bool operator==(const PeerKey& other) const
            {
                return ip == other.ip && data_port == other.data_port;
            }
        };

        struct PeerKeyHash
        {
            std::size_t operator()(const PeerKey& key) const
            {
                return std::hash<uint32_t>{ }(key.ip) ^ (std::hash<uint16_t>{ }(key.data_port) << 1);
            }
        };

        struct ComponentContext
        {
            struct PeerInfo
            {
                struct sockaddr_in handshake_addr;
                uint16_t data_port;
            };

            int fd;
            uint16_t local_port;
            std::string protocol;
            remote_protocol::comparison_func_t comp_func;
            remote_protocol::disconnect_callback_t on_disconnect;
            remote_protocol::handshake_result_callback_t on_handshake_result;
            void* context;

            std::vector<PeerInfo> peers;
        };

        void Networking_Thread();
        void Handle_Message(int sock);
        void Handle_Conf_Message(int sock,
                                 const sockaddr_in& remote_addr,
                                 const std::array<std::uint8_t, 4096>& buffer,
                                 ssize_t received,
                                 const char* remote_ip_str);
        void Handle_Response_Message(const sockaddr_in& remote_addr,
                                     const std::array<std::uint8_t, 4096>& buffer,
                                     ssize_t received,
                                     const char* remote_ip_str);
        void Handle_Disconnect_Message(const sockaddr_in& remote_addr,
                                       const std::array<std::uint8_t, 4096>& buffer,
                                       ssize_t received,
                                       const char* remote_ip_str);
        [[nodiscard]] std::vector<int>
        Update_Peers_On_Disconnect(const PeerKey& key, const sockaddr_in& remote_addr, uint16_t remote_data_port);
        void Notify_Components_Of_Disconnect(const std::vector<int>& fds_to_notify,
                                             const char* remote_ip_str,
                                             uint16_t remote_data_port);
        void InternalDisconnect(ComponentContext& comp);
        void StartServer();
        void StopServer();

        std::string m_name;
        uint16_t m_handshake_port;
        int m_handshake_sock{ -1 };
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };

        std::atomic<bool> m_running{ false };
        bool m_enabled{ false };
        int m_ui_port;
        std::thread m_networking_thread;

        std::mutex m_mutex;
        // fd -> Context
        std::unordered_map<int, ComponentContext> m_components;
        // ip + port -> vec of ports
        std::unordered_map<PeerKey, std::vector<int>, PeerKeyHash> m_peer_to_components;
    };
}
