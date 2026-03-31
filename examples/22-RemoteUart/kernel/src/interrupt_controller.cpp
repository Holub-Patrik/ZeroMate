#include <interrupt_controller.h>

CInterrupt_Controller sInterruptCtl(hal::Interrupt_Controller_Base);

CInterrupt_Controller::CInterrupt_Controller(unsigned long base)
: mInterrupt_Regs(reinterpret_cast<unsigned int*>(base))
{
}

volatile unsigned int& CInterrupt_Controller::Regs(hal::Interrupt_Controller_Reg reg)
{
    return mInterrupt_Regs[static_cast<uint32_t>(reg)];
}

void CInterrupt_Controller::Enable_Basic_IRQ(hal::IRQ_Basic_Source source_idx)
{
    Regs(hal::Interrupt_Controller_Reg::IRQ_Basic_Enable) = (1 << static_cast<uint32_t>(source_idx));
}

void CInterrupt_Controller::Disable_Basic_IRQ(hal::IRQ_Basic_Source source_idx)
{
    Regs(hal::Interrupt_Controller_Reg::IRQ_Basic_Disable) = (1 << static_cast<uint32_t>(source_idx));
}

void CInterrupt_Controller::Enable_IRQ(hal::IRQ_Source source_idx)
{
    const uint32_t idx = static_cast<uint32_t>(source_idx);
    if (idx < 32)
        Regs(hal::Interrupt_Controller_Reg::IRQ_Enable_1) = (1 << idx);
    else
        Regs(hal::Interrupt_Controller_Reg::IRQ_Enable_2) = (1 << (idx - 32));
}

void CInterrupt_Controller::Disable_IRQ(hal::IRQ_Source source_idx)
{
    const uint32_t idx = static_cast<uint32_t>(source_idx);
    if (idx < 32)
        Regs(hal::Interrupt_Controller_Reg::IRQ_Disable_1) = (1 << idx);
    else
        Regs(hal::Interrupt_Controller_Reg::IRQ_Disable_2) = (1 << (idx - 32));
}
