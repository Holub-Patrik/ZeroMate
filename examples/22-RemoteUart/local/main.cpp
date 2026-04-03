#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <interrupt_controller.h>

// Pins for Tens display
#define TENS_LATCH 2
#define TENS_DATA 3
#define TENS_CLOCK 4

// Pins for Units display
#define UNITS_LATCH 7
#define UNITS_DATA 8
#define UNITS_CLOCK 9

// Map for segments (active low)
static const uint8_t sDigit_Map[] = {
    0x84, // 0
    0xD7, // 1
    0x64, // 2
    0x46, // 3
    0x13, // 4
    0x0A, // 5
    0x08, // 6
    0xD4, // 7
    0x00, // 8
    0x02, // 9
};

char op1 = 1;
char op2 = 2;
char op = 0;

void Display_Digit(uint32_t latch, uint32_t data, uint32_t clock, uint8_t value)
{
    for (int i = 0; i < 8; ++i)
    {
        sGPIO.Set_Output(data, (value >> i) & 1);
        sGPIO.Set_Output(clock, true);
        sGPIO.Set_Output(clock, false);
    }
    sGPIO.Set_Output(latch, false);
    sGPIO.Set_Output(latch, true);
}

void Update_Display(int result)
{
    if (result < 0)
    {
        result = 0;
    }
    if (result > 99)
    {
        result = 99;
    }

    int tens = result / 10;
    int units = result % 10;

    Display_Digit(TENS_LATCH, TENS_DATA, TENS_CLOCK, sDigit_Map[tens]);
    Display_Digit(UNITS_LATCH, UNITS_DATA, UNITS_CLOCK, sDigit_Map[units]);
}

extern "C" void _irq_handler(void)
{
    sGPIO.Clear_Detected_Event(5);

    sUART0.Write(static_cast<char>(op1 + '0'));
    if (op == 0)
    {
        sUART0.Write(static_cast<char>('+'));
    }
    else if (op == 1)
    {
        sUART0.Write(static_cast<char>('-'));
    }
    else if (op == 2)
    {
        sUART0.Write(static_cast<char>('*'));
    }
    sUART0.Write(static_cast<char>(op2 + '0'));

    op1 = (op1 + 1) % 10;
    op2 = (op2 + 2) % 10;
    op = (op + 1) % 3;
}

extern "C" void _fiq_handler(void)
{
}

extern "C" int _kernel_main(void)
{
    sUART0.Set_Baud_Rate(NUART_Baud_Rate::BR_115200);
    sUART0.Set_Char_Length(NUART_Char_Length::Char_8);

    sGPIO.Set_GPIO_Function(5, NGPIO_Function::Input);
    sGPIO.Enable_Event_Detect(5, NGPIO_Interrupt_Type::Falling_Edge);

    sGPIO.Set_GPIO_Function(TENS_LATCH, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(TENS_DATA, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(TENS_CLOCK, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(UNITS_LATCH, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(UNITS_DATA, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(UNITS_CLOCK, NGPIO_Function::Output);

    sInterruptCtl.Enable_IRQ(hal::IRQ_Source::GPIO_0);

    Update_Display(0);

    enable_irq();

    while (1)
    {
        char res;
        sUART0.Read(&res);

        Update_Display(res - '0');
    }

    return 0;
}
