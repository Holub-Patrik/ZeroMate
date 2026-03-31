#include <drivers/gpio.h>
#include <drivers/uart.h>

extern "C" void _irq_handler(void)
{
}

extern "C" void _fiq_handler(void)
{
}

extern "C" int _kernel_main(void)
{
    sUART0.Set_Baud_Rate(NUART_Baud_Rate::BR_115200);
    sUART0.Set_Char_Length(NUART_Char_Length::Char_8);

    while (1)
    {
        char c1, op, c2;
        sUART0.Read(&c1);
        sUART0.Read(&op);
        sUART0.Read(&c2);

        int v1 = c1 - '0';
        int v2 = c2 - '0';
        int res = 0;

        if (op == '+')
            res = v1 + v2;
        else if (op == '-')
            res = v1 - v2;
        else if (op == '*')
            res = v1 * v2;

        if (res < 0)
            res = 0;
        if (res > 99)
            res = 99;

        sUART0.Write(static_cast<char>((res / 10) + '0'));
        sUART0.Write(static_cast<char>((res % 10) + '0'));
    }

    return 0;
}
