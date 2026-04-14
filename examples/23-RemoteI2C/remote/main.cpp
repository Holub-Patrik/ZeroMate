#include <drivers/gpio.h>
#include <interrupt_controller.h>

// I2C Slave
#define I2C_SLAVE_ADDRESS 0x74

// I2C Slave Registers
#define BSCSL_BASE 0x20214000
#define BSCSL_DATA (BSCSL_BASE + 0x00)
#define BSCSL_STAT (BSCSL_BASE + 0x04)
#define BSCSL_ADDR (BSCSL_BASE + 0x08)
#define BSCSL_CTRL (BSCSL_BASE + 0x0C)
#define BSCSL_FLAG (BSCSL_BASE + 0x10)

// BSCSL Control Bits
#define BSCSL_CTRL_RXE (1 << 9)
#define BSCSL_CTRL_TXE (1 << 8)
#define BSCSL_CTRL_BRK (1 << 7)
#define BSCSL_CTRL_I2C (1 << 2)
#define BSCSL_CTRL_EN (1 << 0)

// BSCSL Flag Bits
#define BSCSL_FLAG_RXFE (1 << 1)
#define BSCSL_FLAG_TXFF (1 << 2)
#define BSCSL_FLAG_TXCOUNT_MASK (0x1F << 6)
#define BSCSL_FLAG_TXCOUNT_SHIFT 6

void write32(unsigned int addr, unsigned int value)
{
    *((volatile unsigned int*)addr) = value;
}

unsigned int read32(unsigned int addr)
{
    return *((volatile unsigned int*)addr);
}

extern "C" void _irq_handler(void)
{
}
extern "C" void _fiq_handler(void)
{
}

extern "C" int _kernel_main(void)
{
    // Configure I2C Slave pins (Alt 3 for GPIO 18, 19)
    sGPIO.Set_GPIO_Function(18, NGPIO_Function::Alt_3);
    sGPIO.Set_GPIO_Function(19, NGPIO_Function::Alt_3);

    // Reset and initialize I2C Slave
    write32(BSCSL_CTRL, BSCSL_CTRL_BRK);
    write32(BSCSL_CTRL, 0);
    write32(BSCSL_STAT, 0);
    write32(BSCSL_ADDR, I2C_SLAVE_ADDRESS);
    write32(BSCSL_CTRL, BSCSL_CTRL_RXE | BSCSL_CTRL_TXE | BSCSL_CTRL_I2C | BSCSL_CTRL_EN);

    char equation[3];
    int bytes_received = 0;

    while (1)
    {
        // Check for errors
        if (read32(BSCSL_STAT) != 0)
        {
            write32(BSCSL_STAT, 0);
        }

        // Check if RX FIFO is not empty
        if (!(read32(BSCSL_FLAG) & BSCSL_FLAG_RXFE))
        {
            equation[bytes_received++] = (char)(read32(BSCSL_DATA) & 0xFF);

            if (bytes_received == 3)
            {
                // Calculate result
                char operand_1 = equation[0];
                char operation = equation[1];
                char operand_2 = equation[2];
                char result = 0;

                if (operation == 0)
                {
                    result = (char)(operand_1 + operand_2);
                }
                else if (operation == 1)
                {
                    if (operand_1 > operand_2)
                    {
                        result = (char)(operand_1 - operand_2);
                    }
                    else
                    {
                        result = (char)(operand_2 - operand_1);
                    }
                }
                else if (operation == 2)
                {
                    result = (char)(operand_1 * operand_2);
                }

                // Send result back (wait for TX FIFO space)
                while (read32(BSCSL_FLAG) & BSCSL_FLAG_TXFF)
                {
                    ;
                }
                write32(BSCSL_DATA, (int)result);

                bytes_received = 0;
            }
        }
    }

    return 0;
}
