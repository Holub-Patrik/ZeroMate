#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <interrupt_controller.h>

// I2C
#define I2C_SLAVE_ADDRESS 0x74

// Pins for Tens display
#define TENS_LATCH 10
#define TENS_DATA 11
#define TENS_CLOCK 12

// Pins for Units display
#define UNITS_LATCH 13
#define UNITS_DATA 14
#define UNITS_CLOCK 15

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
char op = 0; // 0: +, 1: -, 2: *

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
        result = -result;
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

void delay(uint32_t count)
{
    for (volatile uint32_t i = 0; i < count; ++i)
    {
        asm volatile("nop");
    }
}

extern "C" void _irq_handler(void)
{
    sGPIO.Clear_Detected_Event(5);

    // Send equation: op1, op, op2
    char tx_data[3];
    tx_data[0] = op1;
    tx_data[1] = op;
    tx_data[2] = op2;
    sI2C1.Send(I2C_SLAVE_ADDRESS, tx_data, 3);

    // Give the slave some time to process
    delay(1000);

    // Receive result
    char result = 0;
    sI2C1.Receive(I2C_SLAVE_ADDRESS, (char*)&result, 1);

    Update_Display((int)result);

    op1 = (op1 + 1) % 10;
    op2 = (op2 + 2) % 10;
    op = (op + 1) % 3;
}

extern "C" void _fiq_handler(void)
{
}

extern "C" int _kernel_main(void)
{
    // Button Init (GPIO 5)
    sGPIO.Set_GPIO_Function(5, NGPIO_Function::Input);
    sGPIO.Enable_Event_Detect(5, NGPIO_Interrupt_Type::Falling_Edge);

    // 7-Seg Init
    sGPIO.Set_GPIO_Function(TENS_LATCH, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(TENS_DATA, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(TENS_CLOCK, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(UNITS_LATCH, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(UNITS_DATA, NGPIO_Function::Output);
    sGPIO.Set_GPIO_Function(UNITS_CLOCK, NGPIO_Function::Output);

    sInterruptCtl.Enable_IRQ(hal::IRQ_Source::GPIO_0);

    sI2C1.Open();

    Update_Display(0);

    enable_irq();

    while (1)
    {
        asm volatile("wfe");
    }

    return 0;
}
