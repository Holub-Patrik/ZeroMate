#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <interrupt_controller.h>

// GPIO
#define GPSET1 0x20200020
#define GPCLR1 0x2020002C

// I2C
#define I2C_SLAVE_ADDRESS 0x74

void write32(unsigned int addr, unsigned int value)
{
    *((volatile unsigned int*)addr) = value;
}

unsigned int read32(unsigned int addr)
{
    return *((volatile unsigned int*)addr);
}

void active_sleep(unsigned int ticks)
{
    for (volatile unsigned int i = 0; i < ticks; i++)
    {
        asm volatile("nop");
    };
}

bool g_led_on = false;
void led_toggle(void)
{
    g_led_on = !g_led_on;
    if (g_led_on)
    {
        write32(GPCLR1, 1 << (47 - 32)); // LED ON (Active Low)
    }
    else
    {
        write32(GPSET1, 1 << (47 - 32)); // LED OFF
    }
}

extern "C" void _irq_handler(void)
{
}
extern "C" void _fiq_handler(void)
{
}

extern "C" int _kernel_main(void)
{
    sGPIO.Set_GPIO_Function(47, NGPIO_Function::Output);
    write32(GPSET1, 1 << (47 - 32)); // Start OFF

    sI2C1.Open();

    unsigned char tx_val = 0;

    while (1)
    {
        sI2C1.Send(I2C_SLAVE_ADDRESS, reinterpret_cast<const char*>(&tx_val), 1);
        active_sleep(0x10000);

        char rx_val = 0;
        sI2C1.Receive(I2C_SLAVE_ADDRESS, &rx_val, 1);

        // We can't easily check for errors with current CI2C::Receive as it doesn't return anything
        // but for this example it should be fine as it will just receive 0xFF or something if it fails
        led_toggle();

        tx_val++;
        active_sleep(0x100000);
    }

    return 0;
}
