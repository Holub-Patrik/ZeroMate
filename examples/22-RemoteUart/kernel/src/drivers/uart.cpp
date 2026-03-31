#include <drivers/uart.h>
#include <drivers/bcm_aux.h>

CUART sUART0(sAUX);

CUART::CUART(CAUX& aux)
: mAUX(aux)
{
    mAUX.Enable(hal::AUX_Peripherals::MiniUART);
    mAUX.Set_Register(hal::AUX_Reg::MU_IIR, 0);
    mAUX.Set_Register(hal::AUX_Reg::MU_IER, 0);
    mAUX.Set_Register(hal::AUX_Reg::MU_MCR, 0);
    mAUX.Set_Register(hal::AUX_Reg::MU_CNTL, 3); // RX and TX enabled
}

void CUART::Set_Char_Length(NUART_Char_Length len)
{
    mAUX.Set_Register(hal::AUX_Reg::MU_LCR,
                      (mAUX.Get_Register(hal::AUX_Reg::MU_LCR) & 0xFFFFFFFE) | static_cast<unsigned int>(len));
}

void CUART::Set_Baud_Rate(NUART_Baud_Rate rate)
{
    const unsigned int val = ((hal::Default_Clock_Rate / static_cast<unsigned int>(rate)) / 8) - 1;

    mAUX.Set_Register(hal::AUX_Reg::MU_CNTL, 0);
    mAUX.Set_Register(hal::AUX_Reg::MU_BAUD, val);
    mAUX.Set_Register(hal::AUX_Reg::MU_CNTL, 3);
}

void CUART::Write(char c)
{
    while (!(mAUX.Get_Register(hal::AUX_Reg::MU_LSR) & (1 << 5)))
        ;

    mAUX.Set_Register(hal::AUX_Reg::MU_IO, c);
}

void CUART::Write(const char* str)
{
    for (int i = 0; str[i] != '\0'; i++)
        Write(str[i]);
}

void CUART::Write(const char* str, unsigned int len)
{
    for (unsigned int i = 0; i < len; i++)
        Write(str[i]);
}

void CUART::Write(unsigned int num)
{
    char buf[16];
    int i = 0;
    if (num == 0)
    {
        Write('0');
        return;
    }
    while (num > 0)
    {
        buf[i++] = (num % 10) + '0';
        num /= 10;
    }
    while (--i >= 0)
        Write(buf[i]);
}

void CUART::Write(int num)
{
    if (num < 0)
    {
        Write('-');
        num = -num;
    }
    Write(static_cast<unsigned int>(num));
}

void CUART::Write_Hex(unsigned int num)
{
    char buf[16];
    int i = 0;
    if (num == 0)
    {
        Write('0');
        return;
    }
    while (num > 0)
    {
        unsigned int rem = num % 16;
        if (rem < 10)
            buf[i++] = rem + '0';
        else
            buf[i++] = rem - 10 + 'A';
        num /= 16;
    }
    while (--i >= 0)
        Write(buf[i]);
}

void CUART::Read(char* c)
{
    while (!(mAUX.Get_Register(hal::AUX_Reg::MU_LSR) & 0x01))
        ;

    *c = static_cast<char>(mAUX.Get_Register(hal::AUX_Reg::MU_IO) & 0xFF);
}

void CUART::Enable_Receive_Int()
{
    mAUX.Set_Register(hal::AUX_Reg::MU_IER, mAUX.Get_Register(hal::AUX_Reg::MU_IER) | 0x01);
}

void CUART::Enable_Transmit_Int()
{
    mAUX.Set_Register(hal::AUX_Reg::MU_IER, mAUX.Get_Register(hal::AUX_Reg::MU_IER) | 0x02);
}
