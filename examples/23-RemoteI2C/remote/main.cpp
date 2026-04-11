#include <drivers/gpio.h>
#include <drivers/i2c_slave.h>
#include <interrupt_controller.h>

// GPIO
#define GPSET1 0x20200020
#define GPCLR1 0x2020002C

// I2C Slave
#define I2C_SLAVE_ADDRESS 0x74

void write32(unsigned int addr, unsigned int value)
{
    *((volatile unsigned int*)addr) = value;
}

unsigned int read32(unsigned int addr)
{
    return *((volatile unsigned int*)addr);
}

bool g_led_on = false;
void led_toggle(void)
{
    g_led_on = !g_led_on;
    if (g_led_on)
    {
        write32(GPCLR1, 1 << (47 - 32)); // LED ON
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

    sI2C_Slave.Open(I2C_SLAVE_ADDRESS);

    unsigned char last_val = 0;
    bool first_receive = true;

    while (1)
    {
        // Check for errors (TX Underrun or RX Overrun)
        if (sI2C_Slave.Get_Status() != 0)
        {
            sI2C_Slave.Clear_Status();
        }

        // Check if RX FIFO is not empty
        if (!sI2C_Slave.Is_RX_Empty())
        {
            unsigned char new_val = sI2C_Slave.Read();
            if (first_receive || new_val != last_val)
            {
                last_val = new_val;
                led_toggle();
                first_receive = false;
            }
        }

        // If TX FIFO is empty, queue the response
        if (sI2C_Slave.Get_TX_Count() == 0)
        {
            sI2C_Slave.Write((last_val + 1) & 0xFF);
        }
    }

    return 0;
}
