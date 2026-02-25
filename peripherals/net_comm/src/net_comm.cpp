// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.cpp
/// \author Assistant
/// \brief Implementation of the Remote GPIO peripheral.
// ---------------------------------------------------------------------------------------------------------------------

#include "net_comm.hpp"
#include "CircularBufferQueue.hpp"

#include <unistd.h>
#include <cstring>

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
                // hehe, here's an issue >:(
                prot.info = I2C_P{ .slaves = { other_side_new_port } };
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
}

void GPIOServer::pin_write()
{
    // tries to read from the queue
    // if nothing comes for a long enough time, start sleeping on a semaphore
    const auto [pin, value] = pin_write_queue_reader.read(backoff_sem);
    func_set_pin(static_cast<std::uint32_t>(pin), value > 0);
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
    const auto& conn_id = pin_to_conn_id.get(pin);
    auto& writer_ref = out_queue_writers[conn_id];
    while (!writer_ref.try_insert())
    {
        backoff_fast.wait();
    }
    backoff_fast.reset();

    writer_ref.insert(pin_info);
}

void GPIOServer::construct_connection(const conn_info& info)
{
}

void GPIOServer::run()
{
}

Remote_GPIO::Remote_GPIO(const std::string& name,
                         const std::vector<std::uint32_t>& pins,
                         zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                         zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                         zero_mate::utils::CLogging_System* logging_system)
: name(name)
, pins(pins)
, read_pin(read_pin)
, set_pin(set_pin)
, logging_system(logging_system)
, ImGui_context(nullptr)
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
}

Remote_GPIO::~Remote_GPIO() = default;

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
    ImGui::Text("Network Configuration");
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
                      zero_mate::utils::CLogging_System* logging_system)
    {
        std::vector<std::uint32_t> pins;
        for (std::size_t i = 0; i < pin_count; ++i)
        {
            pins.push_back(connection[i]);
        }

        *peripheral = new (std::nothrow) Remote_GPIO(name, pins, read_pin, set_pin, logging_system);

        if (*peripheral == nullptr)
        {
            return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
        }

        return zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
