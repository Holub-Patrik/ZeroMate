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
        char value_1 = 0;
        char operation = 0;
        char value_2 = 0;
        char res = 0;

        sUART0.Read(&value_1);
        sUART0.Read(&operation);
        sUART0.Read(&value_2);

        value_1 -= '0';
        value_2 -= '0';

        if (operation == '+')
        {
            res = static_cast<char>(value_1 + value_2);
        }
        // calculate absolute difference so that I don't deal with negative
        else if (operation == '-')
        {
            if (value_1 > value_2)
            {
                res = static_cast<char>(value_1 - value_2);
            }
            else
            {
                res = static_cast<char>(value_2 - value_1);
            }
        }
        else if (operation == '*')
        {
            res = static_cast<char>(value_1 * value_2);
        }

        sUART0.Write(static_cast<char>(res + '0'));
    }

    return 0;
}
