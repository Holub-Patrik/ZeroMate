#include <drivers/i2c_slave.h>
#include <drivers/gpio.h>

CI2C_Slave sI2C_Slave(hal::BSCSL_Base, 18, 19);

CI2C_Slave::CI2C_Slave(unsigned long base, uint32_t pin_sda, uint32_t pin_scl)
: mBase(reinterpret_cast<volatile uint32_t*>(base))
, mSDA_Pin(pin_sda)
, mSCL_Pin(pin_scl)
{
    //
}

volatile uint32_t& CI2C_Slave::Reg(hal::BSCSL_Reg reg) const
{
    return mBase[static_cast<uint32_t>(reg)];
}

void CI2C_Slave::Open(uint8_t addr)
{
    // Configure GPIOs
    sGPIO.Set_GPIO_Function(mSDA_Pin, NGPIO_Function::Alt_3);
    sGPIO.Set_GPIO_Function(mSCL_Pin, NGPIO_Function::Alt_3);

    // Initial reset
    Reg(hal::BSCSL_Reg::Control) = static_cast<uint32_t>(Control_Flags::BRK);
    Reg(hal::BSCSL_Reg::Control) = 0;
    Reg(hal::BSCSL_Reg::Status_And_Error_Clear) = 0;
    Reg(hal::BSCSL_Reg::Interrupt_Clear) = 0xF;

    // Set slave address
    Reg(hal::BSCSL_Reg::Slave_Address) = addr;

    // Enable I2C, RX and TX
    Reg(hal::BSCSL_Reg::Control) = static_cast<uint32_t>(Control_Flags::RXE) |
                                   static_cast<uint32_t>(Control_Flags::TXE) |
                                   static_cast<uint32_t>(Control_Flags::I2C) |
                                   static_cast<uint32_t>(Control_Flags::EN);
}

void CI2C_Slave::Close()
{
    Reg(hal::BSCSL_Reg::Control) = 0;
    sGPIO.Set_GPIO_Function(mSDA_Pin, NGPIO_Function::Input);
    sGPIO.Set_GPIO_Function(mSCL_Pin, NGPIO_Function::Input);
}

bool CI2C_Slave::Is_RX_Empty() const
{
    return (mBase[static_cast<uint32_t>(hal::BSCSL_Reg::Flag)] & static_cast<uint32_t>(Flag_Flags::RXFE));
}

uint8_t CI2C_Slave::Read()
{
    return static_cast<uint8_t>(Reg(hal::BSCSL_Reg::Data) & 0xFF);
}

bool CI2C_Slave::Is_TX_Full() const
{
    return (mBase[static_cast<uint32_t>(hal::BSCSL_Reg::Flag)] & static_cast<uint32_t>(Flag_Flags::TXFF));
}

void CI2C_Slave::Write(uint8_t val)
{
    Reg(hal::BSCSL_Reg::Data) = val;
}

uint32_t CI2C_Slave::Get_TX_Count() const
{
    return (Reg(hal::BSCSL_Reg::Flag) & static_cast<uint32_t>(Flag_Flags::TXCOUNT_MASK)) >> static_cast<uint32_t>(Flag_Flags::TXCOUNT_SHIFT);
}

uint32_t CI2C_Slave::Get_RX_Count() const
{
    return (Reg(hal::BSCSL_Reg::Flag) & static_cast<uint32_t>(Flag_Flags::RXCOUNT_MASK)) >> static_cast<uint32_t>(Flag_Flags::RXCOUNT_SHIFT);
}

void CI2C_Slave::Clear_Status()
{
    Reg(hal::BSCSL_Reg::Status_And_Error_Clear) = 0;
}

uint32_t CI2C_Slave::Get_Status() const
{
    return Reg(hal::BSCSL_Reg::Status_And_Error_Clear);
}
