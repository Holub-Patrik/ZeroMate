// ---------------------------------------------------------------------------------------------------------------------
/// \file bsc_slave.hpp
/// \date 10. 04. 2026
/// \author Patrik Holub (23bulohp@gmail.com)
///
/// \brief This file defines the BSC peripheral used in BCM2835.
///
/// To find more information about this peripheral, please visit
/// https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf (chapter 11)
// ---------------------------------------------------------------------------------------------------------------------

#pragma once

// STL imports (excluded from Doxygen)
/// \cond
#include <array>
#include <queue>
/// \endcond

// Project file imports

#include "gpio.hpp"
#include "peripheral.hpp"
#include "system_clock_listener.hpp"
#include "zero_mate/utils/logging_system.hpp"
#include "zero_mate/external_peripheral.hpp"

namespace zero_mate::peripheral
{
    // -----------------------------------------------------------------------------------------------------------------
    /// \class CBSC_Slave
    /// \brief This class represents the BSC peripheral used in BCM2835.
    // -----------------------------------------------------------------------------------------------------------------
    class CBSC_Slave final : public IPeripheral, public ISystem_Clock_Listener, public IExternal_Peripheral
    {
    public:
        /// I2C SDA (data) slave pin on the Raspberry Pi Zero board
        static constexpr std::uint32_t SDA_Pin_Idx = 18;

        /// I2C SCL (clock) slave pin on the Raspberry Pi Zero board
        static constexpr std::uint32_t SCL_Pin_Idx = 19;

        /// Slave address length
        static constexpr std::uint8_t Slave_Addr_Length = 7;

        /// Data length
        static constexpr std::uint8_t Data_Length = 8;

        // Clock speed
        static constexpr std::uint32_t CPU_Cycles_Per_Update = 30;

        // FIFO depth
        static constexpr std::size_t FIFO_Depth = 16;

        // Data Bit Mask
        static constexpr std::uint32_t DATA_Mask = 0xFFU;

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NRegister
        /// \brief Enumeration of the BSC registers.
        // -------------------------------------------------------------------------------------------------------------
        enum class NRegister : std::uint32_t
        {
            Data = 0,               ///< Data register (used for FIFO access)
            Status_And_Error_Clear, ///< Operation status register and error clear register
            Slave_Address,          ///< Holds the I2C Slave register
            Control,                ///< Control register used to configure I2C/SPI Slave
            Flag,                   ///< Flag register (FIFO levels and status)
            Interrupt_FIFO_Level_Select,
            Interrupt_Mask_Set_Clear,
            Raw_Interrupt_Status,
            Masked_Interrupt_Status,
            Interrupt_Clear,
            DMA_Control_Register, ///< Unsupported
            Test_Data_Register,
            GPUSTAT, ///< GPU Status
            Host_Control,
            Debug_1, ///< I2C Debug Register
            Debug_2, ///< SPI Debug Register
            Count
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NData_Sections
        /// \brief Enumeration of bit positions within the Data Register.
        // -------------------------------------------------------------------------------------------------------------
        enum class NData_Sections : std::uint32_t
        {
            Data_Start = 0,
            Data_End = 7,
            RX_Overrun = 8,
            TX_Underrun = 9,
            // 10 - 15 is reserved
            TX_Busy = 16,
            RX_FIFO_Empty = 17,
            TX_FIFO_Full = 18,
            RX_FIFO_Full = 19,
            TX_FIFO_Empty = 20,
            RX_Busy = 21,
            TX_FIFO_Level_Start = 22,
            TX_FIFO_Level_End = 26,
            RX_FIFO_Level_Start = 27,
            RX_FIFO_Level_End = 31,
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NError_Bit_Positions
        /// \brief Enumeration of bit positions for flags in the error register.
        // -------------------------------------------------------------------------------------------------------------
        enum class NError_Bit_Positions : std::uint32_t
        {
            RX_Overrun_Error = 0, ///< Reset 0
            TX_Underrun_Error = 1 ///< Reset 0
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NControl_Flags
        /// \brief Enumeration of bit positions in the control register.
        // -------------------------------------------------------------------------------------------------------------
        enum class NControl_Flags : std::uint32_t
        {
            Enable_Device = 0,
            Enable_SPI = 1,
            Enable_I2C = 2,
            Clock_Phase = 3,
            Clock_Polarity = 4,
            Enable_Status_8bit_Register = 5,
            Enable_Control_8bit_Register = 6,
            Break = 7,     ///< Stop operation and clear all FIFOs
            TX_Enable = 8, ///< Transmit mode enable
            RX_Enable = 9, ///< Receive mode enable
            Inverse_RX_Status_Flag = 10,
            Test_FIFO = 11,
            Enable_Control_For_Host = 12,
            Inverse_TX_Status_Flag = 13,
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NFlag_Bit_Positions
        /// \brief Enumeration of bit positions in the flag register.
        // -------------------------------------------------------------------------------------------------------------
        enum class NFlag_Bit_Positions : std::uint32_t
        {
            TX_Busy = 0,
            RX_FIFO_Empty = 1, // Reset to 1
            TX_FIFO_Full = 2,
            RX_FIFO_Full = 3,
            TX_FIFO_Empty = 4, // Reset to 1
            RX_Busy = 5,
            TX_FIFO_Level_Start = 6,
            TX_FIFO_Level_End = 10,
            RX_FIFO_Level_Start = 11,
            RX_FIFO_Level_End = 15
        };

        /// Total number of the peripheral's registers
        static constexpr auto Number_Of_Registers = static_cast<std::size_t>(NRegister::Count);

        /// Size of a single register
        static constexpr auto Reg_Size = static_cast<std::uint32_t>(sizeof(std::uint32_t));

    public:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Creates an instance of the class.
        /// \param gpio Reference to a GPIO manager (changing the state of GPIO pins)
        // -------------------------------------------------------------------------------------------------------------
        explicit CBSC_Slave(std::shared_ptr<CGPIO_Manager> gpio);

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Resets/re-initializes the peripheral (IPeripheral interface).
        // -------------------------------------------------------------------------------------------------------------
        void Reset() noexcept override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Returns the size of the peripheral (IPeripheral interface).
        /// \return number of register * register size
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] std::uint32_t Get_Size() const noexcept override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Writes data to the peripheral (IPeripheral interface).
        /// \param addr Relative address (from the peripheral's perspective) where the data will be written
        /// \param data Pointer to the data to be written to the peripheral
        /// \param size Size of the data to be written to the peripheral [B]
        // -------------------------------------------------------------------------------------------------------------
        void Write(std::uint32_t addr, const char* data, std::uint32_t size) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Reads data from the peripheral (IPeripheral interface).
        /// \param addr Relative address (from the peripheral's perspective) from which the data will be read
        /// \param data Pointer to a buffer the data will be copied into
        /// \param size Size of the data to read from the peripheral [B]
        // -------------------------------------------------------------------------------------------------------------
        void Read(std::uint32_t addr, char* data, std::uint32_t size) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Notifies the peripheral about how many CPU cycles have passed (ISystem_Clock_Listener interface).
        /// \param count Number of CPU cycles it took to execute the last instruction
        // -------------------------------------------------------------------------------------------------------------
        void Increment_Passed_Cycles(std::uint32_t count) override;

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Notifies the peripheral that the state of one of the pins it subscribes to has changed.
        /// \param pin_idx Index of the GPIO pin whose state has been changed
        // -------------------------------------------------------------------------------------------------------------
        void GPIO_Subscription_Callback(std::uint32_t pin_idx) override;

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \enum NState_Machine
        /// \brief Enumeration of different states of the I2C state machine.
        // -------------------------------------------------------------------------------------------------------------
        enum class NState_Machine : std::uint8_t
        {
            Start_Bit, ///< Receive the start bit
            Address,   ///< Receive the address of the target machine
            RW,        ///< Receive the RW bit
            ACK_1,     ///< Send the ACK_1 bit
            Data,      ///< Receive/Send the data payload
            ACK_2      ///< Receive/Send the ACK_2 bit
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum TTransaction
        /// \brief Representation of a single data transaction.
        // -------------------------------------------------------------------------------------------------------------
        struct TTransaction
        {
            NState_Machine state{ NState_Machine::Start_Bit }; ///< Current state of the state machine
            std::uint32_t address{ 0x0 };                      ///< Slave address
            std::uint8_t data{ 0 };                            ///< Current byte
            std::uint8_t addr_idx{ Slave_Addr_Length };        ///< Index of the current bit of the slave's address
            std::uint8_t data_idx{ Data_Length };              ///< Index of the current bit of the current data payload
            bool read{ false };                                ///< Is the device being read from?
        };

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Updates the I2C state machine.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Update();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Receives the address of the target device.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Receive_Address();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Receives the RW bit.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Receive_RW_Bit();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Reads from I2C
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Read();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Receives a data payload.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Receive_Data();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends a data payload (Master reading from us).
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Send_Data();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief SCL pin change callback.
        /// \param curr_pin_state Current state of the SCL pin
        // -------------------------------------------------------------------------------------------------------------
        inline void SCL_Pin_Change_Callback(bool curr_pin_state);

        // -------------------------------------------------------------------------------------------------------------
        /// \brief SDA pin change callback.
        /// \param curr_pin_state Current state of the SDA pin
        // -------------------------------------------------------------------------------------------------------------
        inline void SDA_Pin_Change_Callback(bool curr_pin_state);

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Initializes a new transaction.
        // -------------------------------------------------------------------------------------------------------------
        inline void Init_Transaction();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends an ACK bit back to the master device.
        // -------------------------------------------------------------------------------------------------------------
        inline void Send_ACK();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Update the status and flag registers based on FIFO states.
        // -------------------------------------------------------------------------------------------------------------
        inline void Update_Status_Flags();

    private:
        std::shared_ptr<CGPIO_Manager> m_gpio;                 ///< GPIO manager
        std::array<std::uint32_t, Number_Of_Registers> m_regs; ///< Peripheral's registers
        std::queue<std::uint8_t> m_rx_fifo;                    ///< RX Data FIFO
        std::queue<std::uint8_t> m_tx_fifo;                    ///< TX Data FIFO

        TTransaction m_transaction;                ///< Ongoing transaction
        std::uint32_t m_clock;                     ///< Emulation of the current time
        std::uint32_t m_sda_rising_edge_timestamp; ///< Timestamp of SDA going from LOW to HIGH
        std::uint32_t m_scl_rising_edge_timestamp; ///< Timestamp of SCL going from LOW to HIGH
        bool m_sda_prev_state;                     ///< Previous state of the SDA pin
        bool m_scl_prev_state;                     ///< Previous state of the SCL pin
        bool m_sda_rising_edge;                    ///< Rising edge detected on SDA?
        bool m_scl_rising_edge;                    ///< Rising edge detected on SCL?

        utils::CLogging_System& m_logging_system; ///< Logging system
    };

} // namespace zero_mate::peripheral
