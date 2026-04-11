#pragma once

#include <hal/peripherals.h>

class CI2C_Slave
{
private:
    volatile uint32_t* const mBase;

    uint32_t mSDA_Pin;
    uint32_t mSCL_Pin;

public:
    enum class Control_Flags : uint32_t
    {
        RXE = (1 << 9),
        TXE = (1 << 8),
        BRK = (1 << 7),
        I2C = (1 << 2),
        EN = (1 << 0)
    };

    enum class Flag_Flags : uint32_t
    {
        TXFF = (1 << 2),
        RXFE = (1 << 3),
        TXCOUNT_SHIFT = 6,
        TXCOUNT_MASK = (0x1F << 6),
        RXCOUNT_SHIFT = 11,
        RXCOUNT_MASK = (0x1F << 11)
    };

    enum class Status_Flags : uint32_t
    {
        TXUE = (1 << 1),
        RXOE = (1 << 0)
    };

    CI2C_Slave(unsigned long base, uint32_t pin_sda, uint32_t pin_scl);

    void Open(uint8_t addr);
    void Close();

    bool Is_RX_Empty() const;
    uint8_t Read();

    bool Is_TX_Full() const;
    void Write(uint8_t val);

    uint32_t Get_TX_Count() const;
    uint32_t Get_RX_Count() const;

    void Clear_Status();
    uint32_t Get_Status() const;

protected:
    volatile uint32_t& Reg(hal::BSCSL_Reg reg) const;
};

extern CI2C_Slave sI2C_Slave;
