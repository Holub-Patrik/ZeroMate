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

namespace zero_mate::peripheral
{
    // -----------------------------------------------------------------------------------------------------------------
    /// \class CBSC
    /// \brief This class represents the BSC peripheral used in BCM2835.
    // -----------------------------------------------------------------------------------------------------------------
    class CBSC final : public IPeripheral, public ISystem_Clock_Listener
    {
    public:
        /// I2C SDA (data) pin on the Raspberry Pi Zero board
        static constexpr std::uint32_t SDA_Pin_Idx = 2;

        /// I2C SCL (clock) pin on the Raspberry Pi Zero board
        static constexpr std::uint32_t SCL_Pin_Idx = 3;

        /// Slave address length
        static constexpr std::uint8_t Slave_Addr_Length = 7;

        /// Data length
        static constexpr std::uint8_t Data_Length = 8;

        // Clock speed
        static constexpr std::uint32_t CPU_Cycles_Per_Update = 30;

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NRegister
        /// \brief Enumeration of the BSC registers.
        // -------------------------------------------------------------------------------------------------------------
        enum class NRegister : std::uint32_t
        {
            Data,
            Status_And_Error_Clear, /// Operation status register and error clear register
            Slave_Address,          /// Holds the I2C Slave register
            Control,                /// Control register used to configure I2C/SPI Slave
            Flag,
            Interrupt_FIFO_Level_Select,
            Interrupt_Mask_Set_Clear,
            Raw_Interrupt_Status,
            Masked_Interrupt_Status,
            Interrupt_Clear,
            DMA_Control_Register, /// Unsupported
            Test_Data_Register,
            GPUSTAT, // GPU Status
            Host_Control,
            Debug_1, // I2C Debug Register
            Debug_2, // SPI Debug Register
            Count
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NData_Sections
        /// \brief Enumation of the Data Register bit position of different sections within Data Register
        // -------------------------------------------------------------------------------------------------------------
        enum class NData_Sections : std::uint8_t
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
        /// \enum NError_Bit_position
        /// \brief Enumeration of different bit position for flags of the error register.
        // -------------------------------------------------------------------------------------------------------------
        enum class NError_Bit_Positions : std::uint8_t
        {
            RX_Overrun_Error = 0, ///< Reset 0
            TX_Underrun_Error = 1 ///< Reset 0
                                  // rest of bit are unsupported or reserved
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NControl_Flags
        /// \brief Enumeration of control flags in the control register
        // -------------------------------------------------------------------------------------------------------------
        enum class NControl_Flags : std::uint32_t
        {
            Enable_Device = 0b1U << 0U,
            Enable_SPI = 0b1U << 1U,
            Enable_I2C = 0b1U << 2U,
            Clock_Phase = 0b1U << 3U,
            Clock_Polarity = 0b1U << 4U,
            // 0 = implies ordinary I2C mode
            // 1 = status register will be transfered as 1st data character to the bus
            Enable_Status_8bit_Register = 0b1U << 5U,
            // 0 = implies ordinary I2C mode
            // 1 = control register will be received as 1st data character from the bus
            Enable_Control_8bit_Register = 0b1U << 6U,
            Break = 0b1U << 7U,     // Stop operation and clear all FIFOs
            TX_Enable = 0b1U << 8U, // Transmit mode enable
            RX_Enable = 0b1U << 9U, // Receive mode enable
            // 0 = Default status flag bit 6 will be reset to 0 -> RX FIFO Full
            // 1 = Inverse status flag bit 6 will be reset to 1 -> RX FIFO Empty
            Inverse_RX_Status_Flag = 0b1U << 10U,
            Test_FIFO = 0b1U << 11U,
            // Allows host to request GPUSTAT or HCTRL register
            // The same can be achieved using ENSTAT and ENCTRL
            Enable_Control_For_Host = 0b1U << 12U,
            // 0 = Default status flag bit 6 will be reset to 1 -> TX FIFO Empty
            // 1 = Inverse status flag bit 6 will be reset to 0 -> TX FIFO Full
            Inverse_TX_Status_Flag = 0b1U << 13U,
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NFlag_Bit_Positions
        /// \brief Enumeration of bit position of different flags
        // -------------------------------------------------------------------------------------------------------------
        enum class NFlag_Bit_Positions : std::uint8_t
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
        explicit CBSC(std::shared_ptr<CGPIO_Manager> gpio);

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Resets/re-initializes the interrupt controller (IPeripheral interface).
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

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \enum NState_Machine
        /// \brief Enumeration of different states of the I2C state machine.
        // -------------------------------------------------------------------------------------------------------------
        enum class NState_Machine : std::uint8_t
        {
            Start_Bit, ///< Send a start bit (start of a transaction)
            Address,   ///< Send slave's address
            RW,        ///< Are we going to read from the device or write to it?
            ACK_1,     ///< The slave device is supposed to send ACK_1
            Data,      ///< Data payload
            ACK_2,     ///< The slave device is supposed to send ACK_2
            Stop_Bit   ///< Stop bit (end of a transaction)
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum TTransaction
        /// \brief Representation of a single data transaction.
        // -------------------------------------------------------------------------------------------------------------
        struct TTransaction
        {
            NState_Machine state{ NState_Machine::Start_Bit }; ///< Current state of the state machine
            std::uint32_t address{ 0x0 };                      ///< Slave address
            std::uint32_t length{ 0 };                         ///< Total number of bytes
            std::uint8_t addr_idx{ Slave_Addr_Length };        ///< Index of the current bit of the slave's address
            std::uint8_t data_idx{ Data_Length };              ///< Index of the current bit of the current data payload
            bool read{ false };                                ///< Are we going to read from the device or write to it?
        };

        // -------------------------------------------------------------------------------------------------------------
        /// \enum NSCL_State
        /// \brief State of the clock signal.
        // -------------------------------------------------------------------------------------------------------------
        enum class NSCL_State
        {
            SDA_Change, ///< SDA shall be updated
            SCL_Low,    ///< SCL shall go low
            SCL_High    ///< SCL shall go high
        };

    private:
        // -------------------------------------------------------------------------------------------------------------
        /// \brief Adds data to the FIFO.
        // -------------------------------------------------------------------------------------------------------------
        inline void Add_Data_To_FIFO();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Performs actions after the control register has been written to.
        // -------------------------------------------------------------------------------------------------------------
        inline void Control_Reg_Callback();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Clears the FIFO.
        // -------------------------------------------------------------------------------------------------------------
        inline void Clear_FIFO();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether a new data transfer (transaction) should begin.
        /// \return true, if a new transaction should begin. false otherwise.
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] inline bool Should_Transaction_Begin();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether the FIFO should be cleared or not.
        /// \return true, if the FIFO should be cleared. false otherwise.
        // -------------------------------------------------------------------------------------------------------------
        [[nodiscard]] inline bool Should_FIFO_Be_Cleared();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sets the state of a given GPIO pin.
        /// \param pin_idx Index of the GPIO pin whose state will be set
        /// \param set Should the state of the pin be set to high or low?
        // -------------------------------------------------------------------------------------------------------------
        inline void Set_GPIO_pin(std::uint8_t pin_idx, bool set);

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Terminates an ongoing transaction (stop bit).
        // -------------------------------------------------------------------------------------------------------------
        inline void Terminate_Transaction();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Updates the I2C state machine.
        // -------------------------------------------------------------------------------------------------------------
        void I2C_Update();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends a start bit (start of a frame).
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Send_Start_Bit();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends another bit of the slave's address.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Send_Slave_Address();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends the RW bit.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Send_RW_Bit();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether the slave device has sent ACK_1 as expected.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Receive_ACK_1();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends another bit of the data payload.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Send_Data();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Checks whether the slave device has sent ACK_2 as expected.
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Receive_ACK_2();

        // -------------------------------------------------------------------------------------------------------------
        /// \brief Sends a stop bit (end of a frame).
        // -------------------------------------------------------------------------------------------------------------
        inline void I2C_Send_Stop_Bit();

    private:
        std::shared_ptr<CGPIO_Manager> m_gpio;                 ///< GPIO manager
        std::array<std::uint32_t, Number_Of_Registers> m_regs; ///< Peripheral's registers
        std::queue<std::uint8_t> m_fifo;                       ///< Data FIFO
        std::uint32_t m_cpu_cycles;                            ///< Number of passed CPU cycles
        bool m_transaction_in_progress;                        ///< Is there an ongoing transaction?
        TTransaction m_transaction;                            ///< Current transaction
        NSCL_State m_SCL_state;                                ///< State of the clock line (SCL)
        utils::CLogging_System& m_logging_system;              ///< Logging system
    };

} // namespace zero_mate::peripheral
