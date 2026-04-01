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
        CGPIO_Server(const std::string& name,
                     uint16_t handshake_port,
                     IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                     IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                     IExternal_Peripheral::Halt_t halt,
                     IExternal_Peripheral::Start_t start,
                     utils::CLogging_System* logging_system);

        ~CGPIO_Server() override;

        void Render() override;
        void Set_ImGui_Context(void* context) override;

        // Exported symbols
        static uint32_t Register(const char* protocol,
                                 remote_protocol::comparison_func_t comp_func,
                                 remote_protocol::receive_callback_t on_receive,
                                 remote_protocol::disconnect_callback_t on_disconnect,
                                 remote_protocol::handshake_result_callback_t on_handshake_result,
                                 void* context);

        static void Unregister(uint32_t id);
        static void Send(uint32_t id, const void* data, size_t size);
        static void
        Init_Handshake(uint32_t id, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size);

    private:
        struct ComponentContext
        {
            uint32_t id;
            std::string protocol;
            remote_protocol::comparison_func_t comp_func;
            remote_protocol::receive_callback_t on_receive;
            remote_protocol::disconnect_callback_t on_disconnect;
            remote_protocol::handshake_result_callback_t on_handshake_result;
            void* context;

            // Networking info (UDP) - multiple endpoints support
            std::vector<int> sockfds;
            std::vector<struct sockaddr_in> remote_addrs;
            bool connected{ false }; // Initiator connection status
        };

        void Networking_Thread();
        void Handle_Handshake(int sock);
        void Handle_Data(int sock, uint32_t component_id);

        std::string m_name;
        uint16_t m_handshake_port;
        IExternal_Peripheral::Read_GPIO_Pin_t m_read_pin;
        IExternal_Peripheral::Set_GPIO_Pin_t m_set_pin;
        IExternal_Peripheral::Halt_t m_halt;
        IExternal_Peripheral::Start_t m_start;
        utils::CLogging_System* m_logging_system;
        void* m_imgui_context{ nullptr };

        std::atomic<bool> m_running{ true };
        std::thread m_networking_thread;

        std::mutex m_mutex;
        std::unordered_map<uint32_t, ComponentContext> m_components;
        uint32_t m_next_id{ 1 };

        static inline std::mutex s_instances_mutex;
        static inline std::vector<CGPIO_Server*> s_instances;
        static inline std::unordered_map<uint32_t, CGPIO_Server*> s_id_to_instance;
    };
}
