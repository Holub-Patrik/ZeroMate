#include <drivers/gpio.h>
#include <hal/peripherals.h>

CGPIO_Handler sGPIO(hal::GPIO_Base);

CGPIO_Handler::CGPIO_Handler(unsigned int gpio_base_addr)
: mGPIO(reinterpret_cast<unsigned int*>(gpio_base_addr))
{
}

bool CGPIO_Handler::Get_GPFSEL_Location(uint32_t pin, uint32_t& reg, uint32_t& bit_idx) const
{
    if (pin >= hal::GPIO_Pin_Count)
        return false;

    reg = pin / 10;
    bit_idx = (pin % 10) * 3;
    return true;
}

bool CGPIO_Handler::Get_GPCLR_Location(uint32_t pin, uint32_t& reg, uint32_t& bit_idx) const
{
    if (pin >= hal::GPIO_Pin_Count)
        return false;

    reg = static_cast<uint32_t>(hal::GPIO_Reg::GPCLR0) + pin / 32;
    bit_idx = pin % 32;
    return true;
}

bool CGPIO_Handler::Get_GPSET_Location(uint32_t pin, uint32_t& reg, uint32_t& bit_idx) const
{
    if (pin >= hal::GPIO_Pin_Count)
        return false;

    reg = static_cast<uint32_t>(hal::GPIO_Reg::GPSET0) + pin / 32;
    bit_idx = pin % 32;
    return true;
}

bool CGPIO_Handler::Get_GPLEV_Location(uint32_t pin, uint32_t& reg, uint32_t& bit_idx) const
{
    if (pin >= hal::GPIO_Pin_Count)
        return false;

    reg = static_cast<uint32_t>(hal::GPIO_Reg::GPLEV0) + pin / 32;
    bit_idx = pin % 32;
    return true;
}

bool CGPIO_Handler::Get_GPEDS_Location(uint32_t pin, uint32_t& reg, uint32_t& bit_idx) const
{
    if (pin >= hal::GPIO_Pin_Count)
        return false;

    reg = static_cast<uint32_t>(hal::GPIO_Reg::GPEDS0) + pin / 32;
    bit_idx = pin % 32;
    return true;
}

void CGPIO_Handler::Set_GPIO_Function(uint32_t pin, NGPIO_Function func)
{
    uint32_t reg, bit;
    if (!Get_GPFSEL_Location(pin, reg, bit))
        return;

    mGPIO[reg] = (mGPIO[reg] & ~(7 << bit)) | (static_cast<uint32_t>(func) << bit);
}

void CGPIO_Handler::Set_Output(uint32_t pin, bool set)
{
    uint32_t reg, bit;
    if (set)
    {
        if (Get_GPSET_Location(pin, reg, bit))
            mGPIO[reg] = (1 << bit);
    }
    else
    {
        if (Get_GPCLR_Location(pin, reg, bit))
            mGPIO[reg] = (1 << bit);
    }
}

void CGPIO_Handler::Enable_Event_Detect(uint32_t pin, NGPIO_Interrupt_Type type)
{
    uint32_t reg, bit;
    if (!Get_GP_IRQ_Detect_Location(pin, type, reg, bit))
        return;

    mGPIO[reg] |= (1 << bit);
}

void CGPIO_Handler::Disable_Event_Detect(uint32_t pin, NGPIO_Interrupt_Type type)
{
    uint32_t reg, bit;
    if (!Get_GP_IRQ_Detect_Location(pin, type, reg, bit))
        return;

    mGPIO[reg] &= ~(1 << bit);
}

void CGPIO_Handler::Clear_Detected_Event(uint32_t pin)
{
    uint32_t reg, bit;
    if (Get_GPEDS_Location(pin, reg, bit))
        mGPIO[reg] = (1 << bit);
}

bool CGPIO_Handler::Get_GP_IRQ_Detect_Location(uint32_t pin, NGPIO_Interrupt_Type type, uint32_t& reg, uint32_t& bit_idx) const
{
    if (pin >= hal::GPIO_Pin_Count)
        return false;

    bit_idx = pin % 32;

    switch (type)
    {
        case NGPIO_Interrupt_Type::Rising_Edge:
            reg = static_cast<uint32_t>(hal::GPIO_Reg::GPREN0) + pin / 32;
            break;
        case NGPIO_Interrupt_Type::Falling_Edge:
            reg = static_cast<uint32_t>(hal::GPIO_Reg::GPFEN0) + pin / 32;
            break;
        case NGPIO_Interrupt_Type::High:
            reg = static_cast<uint32_t>(hal::GPIO_Reg::GPHEN0) + pin / 32;
            break;
        case NGPIO_Interrupt_Type::Low:
            reg = static_cast<uint32_t>(hal::GPIO_Reg::GPLEN0) + pin / 32;
            break;
        default:
            return false;
    }

    return true;
}
