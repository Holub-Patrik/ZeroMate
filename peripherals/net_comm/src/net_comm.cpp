// ---------------------------------------------------------------------------------------------------------------------
/// \file net_comm.cpp
/// \author Assistant
/// \brief Implementation of the Remote GPIO peripheral.
// ---------------------------------------------------------------------------------------------------------------------

#include "net_comm.hpp"
#include "CircularBufferQueue.hpp"
#include "Parser.hpp"

#include <map>
#include <unistd.h>
#include <cstring>
#include <variant>

constexpr int RECV_BUF_SIZE = 64;
constexpr std::size_t QUEUE_SIZE = 32;

constexpr std::uint8_t MAGIC_BYTE = 60; // 0x00111100
constexpr std::uint8_t WRITE_ONE = UINT8_MAX;
constexpr std::uint8_t WRITE_ZERO = 0;

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

    // descriptive typedef
    using conn_id = std::uint64_t;
}

using UART_P = struct UART_ProtocolInfo
{

    std::uint32_t baudrate{ UINT32_MAX };
    std::uint32_t tx_pin{ UINT32_MAX }; // transmit
    std::uint32_t rx_pin{ UINT32_MAX }; // receive (ignore the callback on this one)
    // Point-to-point connection
    struct sockaddr_in other_side{};
};

using I2C_P = struct I2C_ProtocolInfo
{
    std::uint32_t scl_pin{ UINT32_MAX };
    std::uint32_t sda_pin{ UINT32_MAX };
    // Broadcast address and intent everywhere
    // START signal | 7 bit address | 1 bit intent (0 - Write | 1 - Read)
    std::vector<struct sockaddr_in> slaves;
};

using SPI_P = struct SPI_ProtocolInfo
{
    std::uint32_t sclk_pin{ UINT32_MAX };
    std::uint32_t mosi_pin{ UINT32_MAX }; // master out slave in
    std::uint32_t miso_pin{ UINT32_MAX }; // master out slave in
    // Chip select here works basically as an index into the array
    // Of course chip select signal has to sent into the slave first
    std::uint32_t chip_select{ UINT32_MAX };
    std::vector<struct sockaddr_in> slaves;
};

// General buffered and unbuffered protocols
// They are general for a reason and they might be inadequate
// But they can be useful when speeds are low
using GeneralBuffered_P = struct GeneralBuffered_ProtocolInfo
{
    int buf_length{ -1 };
    struct sockaddr_in other_side{};

    GPIOMap net_to_local;
    GPIOMap local_to_net;
    GPIOLastState state_map;
};

using GeneralUnbuffered_P = struct GeneralUnbuffered_ProtocolInfo
{
    struct sockaddr_in other_side{};
    GPIOMap net_to_local;
    GPIOMap local_to_net;
    GPIOLastState state_map;
};

using ProtocolUnion = std::variant<UART_P, I2C_P, SPI_P, GeneralBuffered_P, GeneralUnbuffered_P>;

using protocol_info = struct protocol_info
{
    ProtocolEnum p{ UART };
    ProtocolUnion info;
};

namespace
{
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

using conn_info = struct
{
    bool explicit_clock;
    std::int8_t clock_unit;
    std::uint32_t clock_value;

    in_port_t opened_port;
    protocol_info protocol;

    std::string name;
};

// Defines the logic and data for accepting new connections
class GPIOServer final
{
private:
    static constexpr std::uint64_t BACKOFF_CYCLES = 1024;
    static constexpr auto NET_WAIT_TIME = std::chrono::microseconds{ 100 };

    std::map<conn_id, conn_info> connection_map;
    // convert this one into a directo lookup later
    // This might a good usecase for the 3 array datastructure
    // - The 3 array solution might be good since connections opening/closing isn't guaranteed to be in order
    // This map is needed due to callback nature of handling the pin callback
    std::map<std::uint8_t, conn_id> pin_to_connection;
    std::map<conn_id, CB::Writer<std::uint8_t, QUEUE_SIZE>> connection_to_queue_map;

    CB::Buffer<std::pair<std::uint8_t, std::uint8_t>, QUEUE_SIZE> pin_write_queue_buf{};
    CB::Reader<std::pair<std::uint8_t, std::uint8_t>, QUEUE_SIZE> pin_write_queue_reader;
    CB::Writer<std::pair<std::uint8_t, std::uint8_t>, QUEUE_SIZE> pin_write_queue_writer;

    Spinlock pin_write_spinlock;
    Backoff backoff_fast{ BACKOFF_CYCLES };
    SleepBackoff<std::chrono::microseconds> backoff_net{ BACKOFF_CYCLES,
                                                         (BACKOFF_CYCLES * BACKOFF_CYCLES),
                                                         NET_WAIT_TIME };

    zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin;

    void pin_write()
    {
        while (!pin_write_queue_reader.try_advance())
        {
            backoff_net.wait();
        }
        backoff_net.reset();

        const auto [pin, value] = pin_write_queue_reader.peek();
        pin_write_queue_reader.advance();

        func_set_pin(static_cast<std::uint32_t>(pin), value > 0);
    }

public:
    GPIOServer() = delete;
    explicit GPIOServer(zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t func_set_pin)
    : pin_write_queue_reader(pin_write_queue_buf)
    , pin_write_queue_writer(pin_write_queue_buf)
    , func_set_pin(func_set_pin)
    {
    }
    ~GPIOServer() = default;

    GPIOServer(const GPIOServer& other) = delete;
    GPIOServer& operator=(const GPIOServer& other) = delete;

    GPIOServer(GPIOServer&& other) = delete;
    GPIOServer& operator=(GPIOServer&& other) = delete;

    void write_to_pin(const std::uint8_t pin, const std::uint8_t value)
    {
        pin_write_spinlock.lock();
        while (!pin_write_queue_writer.try_insert())
        {
            backoff_fast.wait();
        }
        backoff_fast.reset();

        pin_write_queue_writer.insert({ pin, value });

        pin_write_spinlock.unlock();
    }

    void writer_thread()
    {
        while (!pin_write_queue_reader.try_advance())
        {
            // within net backoff, a condition variable might be better to sleep until woken up by
            // a callback/reader thread
            backoff_net.wait();
        }
    }

    // bind socket and start listening
    // a parser should be defined
    void run()
    {
    }
};

// Defines the logic and data for each individual connection
class GPIOConnection final
{
    conn_info connection;
    std::unique_ptr<Parser> parser;

    CB::Reader<std::uint8_t, QUEUE_SIZE> bit_queue;
};

CRemote_GPIO::CRemote_GPIO(const std::string& name,
                           const std::vector<std::uint32_t>& pins,
                           zero_mate::IExternal_Peripheral::Read_GPIO_Pin_t read_pin,
                           zero_mate::IExternal_Peripheral::Set_GPIO_Pin_t set_pin,
                           zero_mate::utils::CLogging_System* logging_system)
: m_name(name)
, m_pins(pins)
, m_read_pin(read_pin)
, m_set_pin(set_pin)
, m_logging_system(logging_system)
, m_ImGui_context(nullptr)
, m_sockfd(-1)
, m_running(false)
, m_remote_port(8081)
, m_local_port(8080)
, m_connected(false)
, m_ui_selected_local_pin_idx(0)
, m_ui_target_net_pin(0)
, m_ui_selected_net_pin_source(0)
, m_ui_target_local_pin_idx(0)
{
    std::strncpy(m_remote_ip_buffer, "127.0.0.1", sizeof(m_remote_ip_buffer));

    // Subscribe to all pins passed in construction
    for (const auto& pin : m_pins)
    {
        m_gpio_subscription.insert(pin);
    }
}

CRemote_GPIO::~CRemote_GPIO()
{
    Stop_Listening_Thread();
    if (m_sockfd >= 0)
    {
        close(m_sockfd);
    }
}

void CRemote_GPIO::Set_ImGui_Context(void* context)
{
    m_ImGui_context = static_cast<ImGuiContext*>(context);
}

void CRemote_GPIO::Init_Socket()
{
    if (m_sockfd >= 0)
    {
        close(m_sockfd);
    }

    m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (m_sockfd < 0)
    {
        m_logging_system->Error("Failed to create UDP socket");
        return;
    }

    // Set non-blocking might be useful, but using a thread for recvfrom is cleaner
    // Bind Local
    std::memset(&m_local_addr, 0, sizeof(m_local_addr));
    m_local_addr.sin_family = AF_INET;
    m_local_addr.sin_addr.s_addr = INADDR_ANY;
    m_local_addr.sin_port = htons(m_local_port);

    if (bind(m_sockfd, (const struct sockaddr*)&m_local_addr, sizeof(m_local_addr)) < 0)
    {
        m_logging_system->Error("Failed to bind UDP socket");
        return;
    }

    // Setup Remote (Target)
    std::memset(&m_remote_addr, 0, sizeof(m_remote_addr));
    m_remote_addr.sin_family = AF_INET;
    m_remote_addr.sin_port = htons(m_remote_port);
    inet_pton(AF_INET, m_remote_ip_buffer, &m_remote_addr.sin_addr);

    m_connected = true;
    Init_Listening_Thread();
    const auto msg = std::string{ "UDP Socket Initialized on fd: " } + std::to_string(m_sockfd);
    m_logging_system->Info(msg.c_str());
}

void CRemote_GPIO::Init_Listening_Thread()
{
    m_running = true;
    m_listener_thread = std::thread(&CRemote_GPIO::Listening_Loop, this);
}

void CRemote_GPIO::Start_Listening_Thread()
{
    Stop_Listening_Thread(); // Ensure previous is stopped
    m_running = true;
    m_listener_thread = std::thread(&CRemote_GPIO::Listening_Loop, this);
}

void CRemote_GPIO::Stop_Listening_Thread()
{
    m_running = false;
    if (m_sockfd >= 0)
    {
        shutdown(m_sockfd, SHUT_RDWR);
        close(m_sockfd);
        m_sockfd = -1;
    }

    m_listener_thread.join();
    m_connected = false;
}

void CRemote_GPIO::Listening_Loop()
{
    std::array<std::uint8_t, RECV_BUF_SIZE> buffer{}; // Small buffer is fine for byte payloads
    struct sockaddr_in cliaddr{};
    socklen_t len = sizeof(cliaddr);

    while (m_running)
    {
        if (m_sockfd < 0)
        {
            break;
        }

        ssize_t bytes_received = recvfrom(m_sockfd, buffer.data(), sizeof(buffer), 0, (struct sockaddr*)&cliaddr, &len);
        if (bytes_received < 3)
        {
            continue;
        }

        // check magic
        if (buffer[0] != MAGIC_BYTE)
        {
            continue;
        }

        // retrieve pin
        const std::uint8_t net_pin = buffer[1];
        const bool value = buffer[2] == WRITE_ONE;
        if (m_map_net_to_local.contains(net_pin))
        {
            const auto local_pin = static_cast<std::uint32_t>(m_map_net_to_local.get(net_pin));
            m_set_pin(local_pin, value);
        }
    }
}

inline void CRemote_GPIO::Send_UDP_Packet(const uint8_t value, const uint8_t source_pin)
{
    if (!m_connected || m_sockfd < 0)
    {
        return;
    }

    const auto net_pin = m_map_local_to_net.get(source_pin);
    const std::array<std::uint8_t, 3> payload = { MAGIC_BYTE, net_pin, value };
    sendto(m_sockfd, &payload, sizeof(payload), 0, (const struct sockaddr*)&m_remote_addr, sizeof(m_remote_addr));
}

void CRemote_GPIO::Increment_Passed_Cycles(std::uint32_t count)
{
}

void CRemote_GPIO::GPIO_Subscription_Callback(std::uint32_t pin_idx)
{
    const auto pin_idx_u8 = static_cast<std::uint8_t>(pin_idx);
    if (m_map_local_to_net.contains(pin_idx_u8))
    {
        const std::uint8_t net_pin = m_map_local_to_net.get(pin_idx_u8);
        const std::uint8_t state = m_read_pin(pin_idx) ? WRITE_ONE : WRITE_ZERO;
        // attempt to reduce cals
        if (state != m_state_map.get(pin_idx_u8))
        {
            Send_UDP_Packet(state, pin_idx_u8);
            m_state_map.set(pin_idx_u8, state);
        }
    }
    std::chrono::high_resolution_clock::now();
}

void CRemote_GPIO::Render()
{
    assert(m_ImGui_context != nullptr);
    ImGui::SetCurrentContext(m_ImGui_context);

    if (ImGui::Begin(m_name.c_str()))
    {
        Render_Settings();
        ImGui::Separator();
        Render_Mappings();
    }
    ImGui::End();
}

void CRemote_GPIO::Render_Settings()
{
    ImGui::Text("Network Configuration");

    ImGui::InputText("Remote IP", m_remote_ip_buffer, sizeof(m_remote_ip_buffer));
    ImGui::InputInt("Remote Port (TX)", &m_remote_port);
    ImGui::InputInt("Local Port (RX)", &m_local_port);

    if (!m_connected)
    {
        if (ImGui::Button("Connect / Bind"))
        {
            Init_Socket();
        }
    }
    else
    {
        if (ImGui::Button("Disconnect / Stop"))
        {
            Stop_Listening_Thread();
        }
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "Active");
    }
}

void CRemote_GPIO::Render_Mappings()
{
    ImGui::Text("Outbound Mapping (Local GPIO -> Net Pin)");

    // Dropdown for Local Pins (Source)
    if (ImGui::BeginCombo("##LocalSource", ("GPIO " + std::to_string(m_pins[m_ui_selected_local_pin_idx])).c_str()))
    {
        for (int i = 0; i < m_pins.size(); ++i)
        {
            bool is_selected = (m_ui_selected_local_pin_idx == i);
            if (ImGui::Selectable(("GPIO " + std::to_string(m_pins[i])).c_str(), is_selected))
            {
                m_ui_selected_local_pin_idx = i;
            }
            if (is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();
    ImGui::Text("->");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##NetTarget", &m_ui_target_net_pin);

    ImGui::SameLine();
    if (ImGui::Button("Add##Out"))
    {
        const auto local_pin = m_pins[m_ui_selected_local_pin_idx];
        m_map_local_to_net.set(static_cast<std::uint8_t>(local_pin), static_cast<std::uint8_t>(m_ui_target_net_pin));
    }

    // List Outbound Mappings
    if (ImGui::BeginTable("OutboundMaps", 2, ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Local GPIO");
        ImGui::TableSetupColumn("Net Pin ID");
        ImGui::TableHeadersRow();

        std::size_t index = 0;
        for (const std::uint8_t mapping : m_map_local_to_net._get_arr())
        {
            if (mapping != UINT8_MAX)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%zu", index);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d", mapping);
                ImGui::SameLine();
                if (ImGui::SmallButton(("X##Out" + std::to_string(index)).c_str()))
                {
                    m_map_local_to_net.set(index, UINT8_MAX);
                }
            }
            index++;
        }
        ImGui::EndTable();
    }

    ImGui::Separator();

    ImGui::Text("Inbound Mapping (Net Pin -> Local GPIO)");

    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("##NetSource", &m_ui_selected_net_pin_source);
    ImGui::SameLine();
    ImGui::Text("->");
    ImGui::SameLine();

    // Dropdown for Local Pins (Target)
    if (ImGui::BeginCombo("##LocalTarget", ("GPIO " + std::to_string(m_pins[m_ui_target_local_pin_idx])).c_str()))
    {
        for (int i = 0; i < m_pins.size(); ++i)
        {
            bool is_selected = (m_ui_target_local_pin_idx == i);
            if (ImGui::Selectable(("GPIO " + std::to_string(m_pins[i])).c_str(), is_selected))
            {
                m_ui_target_local_pin_idx = i;
            }
            if (is_selected)
                ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }

    ImGui::SameLine();
    if (ImGui::Button("Add##In"))
    {
        if (m_ui_selected_net_pin_source >= 0 && m_ui_selected_net_pin_source < GPIOMapSize)
        {
            m_map_net_to_local.set(static_cast<std::uint8_t>(m_ui_selected_net_pin_source),
                                   static_cast<std::uint8_t>(m_pins[m_ui_target_local_pin_idx]));
        }
    }

    // List Inbound Mappings
    if (ImGui::BeginTable("InboundMaps", 2, ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Net Pin ID");
        ImGui::TableSetupColumn("Local GPIO");
        ImGui::TableHeadersRow();

        std::size_t index = 0;
        for (const std::uint8_t mapping : m_map_net_to_local._get_arr())
        {
            if (mapping != UINT8_MAX)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("%zu", index);
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%d", mapping);
                ImGui::SameLine();
                if (ImGui::SmallButton(("X##In" + std::to_string(index)).c_str()))
                {
                    m_map_net_to_local.set(index, UINT8_MAX);
                }
            }
            index++;
        }
        ImGui::EndTable();
    }
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

        *peripheral = new (std::nothrow) CRemote_GPIO(name, pins, read_pin, set_pin, logging_system);

        if (*peripheral == nullptr)
        {
            return zero_mate::IExternal_Peripheral::NInit_Status::Allocation_Error;
        }

        return zero_mate::IExternal_Peripheral::NInit_Status::OK;
    }
}
