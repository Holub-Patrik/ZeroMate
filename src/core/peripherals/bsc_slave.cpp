// ---------------------------------------------------------------------------------------------------------------------
/// \file bsc_slave.cpp
/// \date 10. 04. 2026
/// \author Patrik Holub (23bulohp@gmail.com)
///
/// \brief This file implements the BSC peripheral used in BCM2835.
///
/// To find more information about this peripheral, please visit
/// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf (chapter 11)
// ---------------------------------------------------------------------------------------------------------------------

// STL imports (excluded from Doxygen)
/// \cond
#include <bit>
#include <algorithm>
#include <utility>
/// \endcond

// Project file imports

#include "bsc_slave.hpp"
#include "zero_mate/utils/singleton.hpp"

namespace zero_mate::peripheral
{
    CBSC_Slave::CBSC_Slave(std::shared_ptr<CGPIO_Manager> gpio)
    : m_gpio{ std::move(gpio) }
    , m_regs{ }
    , m_transaction{ }
    , m_clock{ 0 }
    , m_sda_rising_edge_timestamp{ 0 }
    , m_scl_rising_edge_timestamp{ 0 }
    , m_sda_prev_state{ true } // Pull-ups
    , m_scl_prev_state{ true }
    , m_sda_rising_edge{ false }
    , m_scl_rising_edge{ false }
    , m_logging_system{ *utils::CSingleton<utils::CLogging_System>::Get_Instance() }
    {
        m_gpio_subscription.insert(SDA_Pin_Idx);
        m_gpio_subscription.insert(SCL_Pin_Idx);
        Reset();
    }

    void CBSC_Slave::Reset() noexcept
    {
        std::fill(m_regs.begin(), m_regs.end(), 0);

        while (!m_rx_fifo.empty())
        {
            m_rx_fifo.pop();
        }
        while (!m_tx_fifo.empty())
        {
            m_tx_fifo.pop();
        }

        // Set initial flags: RX FIFO Empty and TX FIFO Empty
        m_regs[static_cast<std::uint32_t>(NRegister::Flag)] =
        (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::RX_FIFO_Empty)) |
        (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::TX_FIFO_Empty));

        m_clock = 0;
        m_transaction.state = NState_Machine::Start_Bit;
    }

    std::uint32_t CBSC_Slave::Get_Size() const noexcept
    {
        return static_cast<std::uint32_t>(sizeof(m_regs));
    }

    void CBSC_Slave::Write(std::uint32_t addr, const char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr / Reg_Size;
        const auto reg_type = static_cast<NRegister>(reg_idx);

        // Write data to the peripheral's registers.
        std::copy_n(data, size, &std::bit_cast<char*>(m_regs.data())[addr]);

        switch (reg_type)
        {
            case NRegister::Data:
                // Writing to Data register adds a byte to the TX FIFO.
                if (m_tx_fifo.size() < FIFO_Depth)
                {
                    m_tx_fifo.push(
                    static_cast<std::uint8_t>(m_regs[static_cast<std::uint32_t>(NRegister::Data)] & DATA_Mask));
                }
                Update_Status_Flags();
                break;

            case NRegister::Control:
                // Handle Break
                if (m_regs[static_cast<std::uint32_t>(NRegister::Control)] &
                    (1U << static_cast<std::uint32_t>(NControl_Flags::Break)))
                {
                    while (!m_rx_fifo.empty())
                    {
                        m_rx_fifo.pop();
                    }
                    while (!m_tx_fifo.empty())
                    {
                        m_tx_fifo.pop();
                    }
                    // Clear break bit after action
                    m_regs[static_cast<std::uint32_t>(NRegister::Control)] &=
                    ~(1U << static_cast<std::uint32_t>(NControl_Flags::Break));
                    Update_Status_Flags();
                }
                break;

            case NRegister::Status_And_Error_Clear:
                // Writing to this register clears error bits
                m_regs[static_cast<std::uint32_t>(NRegister::Status_And_Error_Clear)] = 0;
                break;

            default:
                break;
        }
    }

    void CBSC_Slave::Read(std::uint32_t addr, char* data, std::uint32_t size)
    {
        const std::size_t reg_idx = addr / Reg_Size;
        const auto reg_type = static_cast<NRegister>(reg_idx);

        switch (reg_type)
        {
            case NRegister::Data:
                // Reading from Data register pops a byte from the RX FIFO.
                if (!m_rx_fifo.empty())
                {
                    m_regs[static_cast<std::uint32_t>(NRegister::Data)] = m_rx_fifo.front();
                    m_rx_fifo.pop();
                }
                else
                {
                    m_regs[static_cast<std::uint32_t>(NRegister::Data)] = 0;
                }
                Update_Status_Flags();
                break;

            default:
                break;
        }

        // Read data from the peripheral's registers.
        std::copy_n(&std::bit_cast<char*>(m_regs.data())[addr], size, data);
    }

    void CBSC_Slave::Increment_Passed_Cycles(std::uint32_t count)
    {
        m_clock += count;
    }

    void CBSC_Slave::GPIO_Subscription_Callback(std::uint32_t pin_idx)
    {
        const bool curr_pin_state = (m_gpio->Read_GPIO_Pin(pin_idx) == CGPIO_Manager::CPin::NState::High);

        if (pin_idx == SDA_Pin_Idx)
        {
            SDA_Pin_Change_Callback(curr_pin_state);
        }
        else if (pin_idx == SCL_Pin_Idx)
        {
            SCL_Pin_Change_Callback(curr_pin_state);
        }

        if (m_scl_prev_state && m_sda_rising_edge)
        {
            m_transaction.state = NState_Machine::Start_Bit;
            Update_Status_Flags();
        }
    }

    void CBSC_Slave::SCL_Pin_Change_Callback(bool curr_pin_state)
    {
        m_scl_rising_edge = false;

        if (!m_scl_prev_state && curr_pin_state) // Rising edge
        {
            m_scl_rising_edge = true;
            m_scl_rising_edge_timestamp = m_clock;
            I2C_Update();
        }
        else if (m_scl_prev_state && !curr_pin_state) // Falling edge
        {
            if (m_transaction.state == NState_Machine::Data && m_transaction.read)
            {
                I2C_Send_Data();
            }
            else if (m_transaction.state == NState_Machine::ACK_1 && m_transaction.read)
            {
                // Prepare the first bit of the first byte for read transactions
                I2C_Send_Data();
            }
            else if (m_transaction.state == NState_Machine::ACK_2 && m_transaction.read)
            {
                if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) == CGPIO_Manager::CPin::NState::Low)
                {
                    // Master sent ACK, prepare first bit of next byte
                    I2C_Send_Data();
                }
            }
            else if (m_transaction.state == NState_Machine::Data || m_transaction.state == NState_Machine::Address ||
                     m_transaction.state == NState_Machine::RW)
            {
                // Release SDA for ACK/NACK or master bits
                static_cast<void>(m_gpio->Set_Pin_State(SDA_Pin_Idx, CGPIO_Manager::CPin::NState::High));
            }
        }

        m_scl_prev_state = curr_pin_state;
    }

    void CBSC_Slave::SDA_Pin_Change_Callback(bool curr_pin_state)
    {
        m_sda_rising_edge = false;

        if (!m_sda_prev_state && curr_pin_state)
        {
            m_sda_rising_edge = true;
            m_sda_rising_edge_timestamp = m_clock;
            
            // Stop bit detection
            if (m_scl_prev_state)
            {
                m_transaction.state = NState_Machine::Start_Bit;
                Update_Status_Flags();
            }
        }

        // Start bit detection
        if (m_sda_prev_state && !curr_pin_state && m_scl_prev_state)
        {
            Init_Transaction();
            m_transaction.state = NState_Machine::Address;
        }

        m_sda_prev_state = curr_pin_state;
    }

    void CBSC_Slave::Init_Transaction()
    {
        m_transaction.address = 0;
        m_transaction.data = 0;
        m_transaction.addr_idx = Slave_Addr_Length;
        m_transaction.data_idx = Data_Length;
        m_transaction.read = false;
    }

    void CBSC_Slave::I2C_Update()
    {
        switch (m_transaction.state)
        {
            case NState_Machine::Address:
                I2C_Receive_Address();
                break;

            case NState_Machine::RW:
                I2C_Receive_RW_Bit();
                break;

            case NState_Machine::ACK_1:
                if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) == CGPIO_Manager::CPin::NState::Low)
                {
                    m_transaction.state = NState_Machine::Data;
                    if (m_transaction.read)
                    {
                        if (!m_tx_fifo.empty())
                        {
                            m_transaction.data = m_tx_fifo.front();
                            m_tx_fifo.pop();
                        }
                        else
                        {
                            m_transaction.data = 0xFF;
                        }
                    }
                }
                else
                {
                    m_transaction.state = NState_Machine::Start_Bit;
                }
                break;

            case NState_Machine::Data:
                if (m_transaction.read)
                {
                    if (m_transaction.data_idx == 0)
                    {
                        m_transaction.state = NState_Machine::ACK_2;
                    }
                }
                else
                {
                    I2C_Receive_Data();
                }
                break;

            case NState_Machine::ACK_2:
                if (m_transaction.read)
                {
                    if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) == CGPIO_Manager::CPin::NState::Low)
                    {
                        if (!m_tx_fifo.empty())
                        {
                            m_transaction.data = m_tx_fifo.front();
                            m_tx_fifo.pop();
                        }
                        else
                        {
                            m_transaction.data = 0xFF;
                        }
                        m_transaction.data_idx = Data_Length;
                        m_transaction.state = NState_Machine::Data;
                    }
                    else
                    {
                        m_transaction.state = NState_Machine::Start_Bit;
                    }
                }
                else
                {
                    m_transaction.data = 0;
                    m_transaction.data_idx = Data_Length;
                    m_transaction.state = NState_Machine::Data;
                }
                break;
            default:
                break;
        }
        Update_Status_Flags();
    }

    void CBSC_Slave::I2C_Receive_Address()
    {
        --m_transaction.addr_idx;
        if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) == CGPIO_Manager::CPin::NState::High)
        {
            m_transaction.address |= (1U << m_transaction.addr_idx);
        }

        if (m_transaction.addr_idx == 0)
        {
            m_transaction.state = NState_Machine::RW;
        }
    }

    void CBSC_Slave::I2C_Receive_RW_Bit()
    {
        m_transaction.read = (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) == CGPIO_Manager::CPin::NState::High);

        const std::uint32_t my_addr = m_regs[static_cast<std::uint32_t>(NRegister::Slave_Address)] & 0x7FU;
        if (m_transaction.address == my_addr)
        {
            Send_ACK();
            m_transaction.state = NState_Machine::ACK_1;
        }
        else
        {
            m_transaction.state = NState_Machine::Start_Bit;
        }
    }

    void CBSC_Slave::I2C_Receive_Data()
    {
        --m_transaction.data_idx;
        if (m_gpio->Read_GPIO_Pin(SDA_Pin_Idx) == CGPIO_Manager::CPin::NState::High)
        {
            m_transaction.data |= (1U << m_transaction.data_idx);
        }

        if (m_transaction.data_idx == 0)
        {
            if (m_rx_fifo.size() < FIFO_Depth)
            {
                m_rx_fifo.push(m_transaction.data);
            }
            Send_ACK();
            m_transaction.state = NState_Machine::ACK_2;
        }
    }

    void CBSC_Slave::I2C_Send_Data()
    {
        if (m_transaction.data_idx > 0)
        {
            --m_transaction.data_idx;
            const bool bit = (m_transaction.data >> m_transaction.data_idx) & 0b1U;
            static_cast<void>(m_gpio->Set_Pin_State(SDA_Pin_Idx, static_cast<CGPIO_Manager::CPin::NState>(bit)));
        }
    }

    void CBSC_Slave::Send_ACK()
    {
        static_cast<void>(m_gpio->Set_Pin_State(SDA_Pin_Idx, CGPIO_Manager::CPin::NState::Low));
    }

    void CBSC_Slave::Update_Status_Flags()
    {
        std::uint32_t flags = 0;

        if (m_rx_fifo.empty())
        {
            flags |= (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::RX_FIFO_Empty));
        }
        if (m_rx_fifo.size() >= FIFO_Depth)
        {
            flags |= (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::RX_FIFO_Full));
        }

        if (m_tx_fifo.empty())
        {
            flags |= (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::TX_FIFO_Empty));
        }
        if (m_tx_fifo.size() >= FIFO_Depth)
        {
            flags |= (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::TX_FIFO_Full));
        }

        // FIFO levels
        flags |= (static_cast<std::uint32_t>(m_tx_fifo.size()) & 0x1FU)
                 << static_cast<std::uint32_t>(NFlag_Bit_Positions::TX_FIFO_Level_Start);
        flags |= (static_cast<std::uint32_t>(m_rx_fifo.size()) & 0x1FU)
                 << static_cast<std::uint32_t>(NFlag_Bit_Positions::RX_FIFO_Level_Start);

        if (m_transaction.state != NState_Machine::Start_Bit)
        {
            if (m_transaction.read)
                flags |= (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::TX_Busy));
            else
                flags |= (1U << static_cast<std::uint32_t>(NFlag_Bit_Positions::RX_Busy));
        }

        m_regs[static_cast<std::uint32_t>(NRegister::Flag)] = flags;

        // Update Data register bits based on NData_Sections
        m_regs[static_cast<std::uint32_t>(NRegister::Data)] &= 0xFFU;
        if (m_tx_fifo.size() >= FIFO_Depth)
        {
            m_regs[static_cast<std::uint32_t>(NRegister::Data)] |=
            (1U << static_cast<std::uint32_t>(NData_Sections::TX_FIFO_Full));
        }
        if (m_rx_fifo.size() >= FIFO_Depth)
        {
            m_regs[static_cast<std::uint32_t>(NRegister::Data)] |=
            (1U << static_cast<std::uint32_t>(NData_Sections::RX_FIFO_Full));
        }
        if (m_tx_fifo.empty())
        {
            m_regs[static_cast<std::uint32_t>(NRegister::Data)] |=
            (1U << static_cast<std::uint32_t>(NData_Sections::TX_FIFO_Empty));
        }
        if (m_rx_fifo.empty())
        {
            m_regs[static_cast<std::uint32_t>(NRegister::Data)] |=
            (1U << static_cast<std::uint32_t>(NData_Sections::RX_FIFO_Empty));
        }

        m_regs[static_cast<std::uint32_t>(NRegister::Data)] |=
        (static_cast<std::uint32_t>(m_tx_fifo.size()) & 0x1FU)
        << static_cast<std::uint32_t>(NData_Sections::TX_FIFO_Level_Start);
        m_regs[static_cast<std::uint32_t>(NRegister::Data)] |=
        (static_cast<std::uint32_t>(m_rx_fifo.size()) & 0x1FU)
        << static_cast<std::uint32_t>(NData_Sections::RX_FIFO_Level_Start);
    }

} // namespace zero_mate::peripheral
