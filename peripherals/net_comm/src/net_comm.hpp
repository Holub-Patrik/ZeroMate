// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.hpp
/// \brief Defines a remote GPIO peripheral communicating via UDP
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

#include <memory>
#include <thread>
#include <atomic>
#include <utility>
#include <variant>
#include <vector>
#include <string>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <array>

#include "imgui.h"
#include "zero_mate/external_peripheral.hpp"
#include "CircularBufferQueue.hpp"
#include "Util.hpp"
#include "UART_Handler.hpp"
#include "I2C_Handler.hpp"

using Protocol = std::variant<UART_P, I2C_Master_P, I2C_Slave_P>;

using conn_info = struct conn_info_struct
{
    Protocol protocol;

    bool explicit_clock{};
    std::int8_t clock_unit{};
    std::uint32_t clock_value{};

    in_port_t opened_port{}; // receiver port
    std::uint32_t net_id{};
};

using conn_id = std::uint64_t;
using pin_pair = std::pair<std::uint8_t, std::uint8_t>;

using protocol_variant = std::variant<UART_Handler, I2C_Master, I2C_Slave>;

template<typename Type, std::size_t Size>
class BitProcessor final
{
private:
    protocol_variant handler;
    TSP::Queue::Reader<Type, Size> queue_reader;
    TSP::BF::Backoff backoff; // simple for now, can be changed to sem later

    std::atomic<bool> running{ false };
    std::thread sender;
    std::thread receiver;

public:
    BitProcessor(protocol_variant&& variant, TSP::Queue::Buffer<Type, Size>* buf)
    : handler(std::move(variant))
    , queue_reader(buf)
    {
    }

    ~BitProcessor()
    {
        stop();
    }

    BitProcessor(const BitProcessor&) = delete;
    BitProcessor& operator=(const BitProcessor&) = delete;

    BitProcessor(BitProcessor&&) = delete;
    BitProcessor& operator=(BitProcessor&&) = delete;

    void start()
    {
        running = true;
        sender = std::thread{ &BitProcessor<Type, Size>::run_sender, this };
        // Is this legal?
        std::visit(
        [this](auto& handler) -> void { receiver = std::thread(decltype(handler)::receiver_thread, handler); });
    }

    void stop()
    {
        if (running)
        {
            running = false;

            if (sender.joinable())
            {
                sender.join();
            }

            std::visit([](auto& handler) -> void { handler.receiver_stop(); });
        }
    }

private:
    void run_sender()
    {
        auto last_time = std::chrono::high_resolution_clock::now();

        while (running)
        {
            const auto& [bit, pin] = queue_reader.read(backoff);
            const auto now = std::chrono::high_resolution_clock::now();
            const auto delta =
            static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_time).count());
            last_time = now;
            std::visit([bit, delta](auto& handler) -> void { handler.process_bit(bit, delta); });
        }
    }
};

class GPIOServer final
{
public:
    static constexpr std::uint64_t BACKOFF_CYCLES = 1000;
    static constexpr std::uint64_t BACKOFF_CYCLES_RELAXED = 20'000;
    static constexpr auto NET_WAIT_TIME = std::chrono::microseconds{ 100 };
    static constexpr std::size_t MAX_CONNECTION_COUNT = 16;
    static constexpr std::size_t BUFFER_COUNT = MAX_CONNECTION_COUNT;
    static constexpr std::size_t BUFFER_SIZE = 512;
    static constexpr std::size_t QUEUE_SIZE = 64;

private:
    // Pin write entirely here since there will be multiple writers, so spinlock is added to ensure safety
    TSP::Queue::Buffer<pin_pair, QUEUE_SIZE> pin_write_queue_buf{};
    TSP::Queue::Reader<pin_pair, QUEUE_SIZE> pin_write_queue_reader;
    TSP::Queue::Writer<pin_pair, QUEUE_SIZE> pin_write_queue_writer;
    Spinlock pin_write_spinlock;

    TSP::BF::Backoff backoff_fast{ BACKOFF_CYCLES };
    TSP::BF::SemBackoff backoff_sem{ BACKOFF_CYCLES, BACKOFF_CYCLES_RELAXED };

    // a bit map lookup might be best to assign new threads
    std::array<TSP::Queue::Buffer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_buffers;
    std::array<TSP::Queue::Writer<pin_pair, BUFFER_SIZE>, BUFFER_COUNT> out_queue_writers;

    // this could be converted into a bit map, but bit instruction are extra instructions
    std::array<bool, MAX_CONNECTION_COUNT> connection_bit_map{};

    // abuse std::destroy_at{} and std::construct_at{} to use arrays
    std::array<std::thread, MAX_CONNECTION_COUNT> connection_threads;
    std::array<conn_info, MAX_CONNECTION_COUNT> connection_data;

    FastMap pin_to_conn_id;
    FastMap net_id_to_conn_id;

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin;
    zero_mate::IExternal_Peripheral::Halt_t func_halt;
    zero_mate::IExternal_Peripheral::Start_t func_start;

    std::atomic<bool> m_running{ true };
    std::thread m_pin_write_thread;

    void pin_write();

public:
    GPIOServer() = delete;

    explicit GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin,
                        zero_mate::IExternal_Peripheral::Halt_t func_halt,
                        zero_mate::IExternal_Peripheral::Start_t func_start)
    : pin_write_queue_reader(&pin_write_queue_buf)
    , pin_write_queue_writer(&pin_write_queue_buf)
    , func_set_pin(func_set_pin)
    , func_halt(func_halt)
    , func_start(func_start)
    {
        for (std::size_t i = 0; i < out_queue_buffers.size(); i++)
        {
            std::construct_at(&out_queue_writers[i], &out_queue_buffers[i]);
        }
    }
    ~GPIOServer();

    GPIOServer(const GPIOServer& other) = delete;
    GPIOServer& operator=(const GPIOServer& other) = delete;

    GPIOServer(GPIOServer&& other) = delete;
    GPIOServer& operator=(GPIOServer&& other) = delete;

    void write_to_pin(const std::uint8_t pin, const std::uint8_t value);
    void route_pin_info(const pin_pair pin_info);
    void construct_connection(const conn_info& info);
    void run();
    void stop();

    // used by GPIOConnection
    [[nodiscard]] zero_mate::IExternal_Peripheral::Halt_t get_halt() const
    {
        return func_halt;
    }
    [[nodiscard]] zero_mate::IExternal_Peripheral::Start_t get_start() const
    {
        return func_start;
    }
    [[nodiscard]] zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t get_set_pin() const
    {
        return func_set_pin;
    }
};

class GPIOConnection final
{
private:
    conn_info connection;

    TSP::Queue::Reader<pin_pair, GPIOServer::BUFFER_SIZE> m_queue_reader;

    int m_socket;
    struct sockaddr_in m_other_side;

    zero_mate::IExternal_Peripheral::Halt_t m_halt;
    zero_mate::IExternal_Peripheral::Start_t m_start;
    GPIOServer& m_server;

    std::atomic<bool>& m_server_running;
    std::unique_ptr<BitProcessor<pin_pair, GPIOServer::BUFFER_SIZE>> m_processor;

public:
    GPIOConnection() = delete;
    GPIOConnection(const conn_info& info,
                   TSP::Queue::Buffer<pin_pair, GPIOServer::BUFFER_SIZE>* buffer,
                   zero_mate::IExternal_Peripheral::Halt_t halt,
                   zero_mate::IExternal_Peripheral::Start_t start,
                   GPIOServer& server,
                   std::atomic<bool>& server_running);

    ~GPIOConnection();

    GPIOConnection(const GPIOConnection& other) = delete;
    void run();

    // Helper for SM
    pin_pair read_queue();
    void send_to_network(const std::vector<std::uint8_t>& data);
    void write_to_pin(std::uint8_t pin, std::uint8_t value);
    void halt()
    {
        m_halt();
    }
    void start()
    {
        m_start();
    }
    [[nodiscard]] bool is_running() const
    {
        return m_server_running.load();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
/// \class CRemote_GPIO
/// \brief External peripheral for UDP-based remote GPIO control.
// ---------------------------------------------------------------------------------------------------------------------
class Remote_GPIO final : public zero_mate::IExternal_Peripheral
{
public:
    Remote_GPIO() = delete;
    Remote_GPIO(const Remote_GPIO&) = delete;
    Remote_GPIO& operator=(const Remote_GPIO&) = delete;
    Remote_GPIO(Remote_GPIO&&) = delete;
    Remote_GPIO& operator=(Remote_GPIO&&) = delete;

    explicit Remote_GPIO(const std::string& name,
                         const std::vector<std::uint32_t>& pins,
                         zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                         zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                         zero_mate::IExternal_Peripheral::Halt_t halt,
                         zero_mate::IExternal_Peripheral::Start_t start,
                         zero_mate::utils::CLogging_System* logging_system);

    ~Remote_GPIO() final;

    void Render() final;
    void Set_ImGui_Context(void* context) final;
    void Increment_Passed_Cycles(std::uint32_t count) final;
    void GPIO_Subscription_Callback(std::uint32_t pin_idx) final;

private:
    // UI Rendering
    void Render_Settings();
    void Render_Mappings();

    // IExternal dependencies
    std::string name;
    std::vector<std::uint32_t> pins; // Available local pins
    zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin;
    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin;
    zero_mate::IExternal_Peripheral::Halt_t halt;
    zero_mate::IExternal_Peripheral::Start_t start;
    zero_mate::utils::CLogging_System* logging_system;
    ImGuiContext* ImGui_context;

    std::atomic<bool> m_running{ true };
    GPIOServer server;
    std::thread server_thread;

    // UI Helpers
    int ui_selected_local_pin_idx;
    int ui_target_net_pin;
    int ui_selected_net_pin_source;
    int ui_target_local_pin_idx;
};
