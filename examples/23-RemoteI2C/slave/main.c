extern void dummy(unsigned int);

void data_memory_barrier(void)
{
    asm volatile ("mcr p15, 0, %0, c7, c10, 5" : : "r" (0) : "memory");
}

void write32(unsigned int addr, unsigned int value)
{
    data_memory_barrier();
    *((volatile unsigned int*)addr) = value;
    data_memory_barrier();
}

unsigned int read32(unsigned int addr)
{
    unsigned int value = *((volatile unsigned int*)addr);
    data_memory_barrier();
    return value;
}

#define GPFSEL0    0x20200000
#define GPFSEL4    0x20200010
#define GPSET1     0x20200020
#define GPCLR1     0x2020002C
#define GPEDS0     0x20200040
#define GPREN0     0x2020004C
#define GPFEN0     0x20200058
#define GPPUD      0x20200094
#define GPPUDCLK0  0x20200098
#define ST_CLO     0x20003004

#define BSCSL_DR   0x20214000
#define BSCSL_RSR  0x20214004
#define BSCSL_SLV  0x20214008
#define BSCSL_CR   0x2021400C
#define BSCSL_FR   0x20214010
#define BSCSL_ICR  0x20214024

#define GPIO_INPUT   0
#define GPIO_ALT3    7

#define I2C_SLAVE_SDA_PIN 18
#define I2C_SLAVE_SCL_PIN 19
#define I2C_SLAVE_ADDRESS 0x74
#define I2C_REPLY_BYTE    1
#define LED_TOGGLE_INTERVAL_US 100000

#define BSCSL_CR_RXE      (1 << 9)
#define BSCSL_CR_TXE      (1 << 8)
#define BSCSL_CR_BRK      (1 << 7)
#define BSCSL_CR_I2C      (1 << 2)
#define BSCSL_CR_EN       (1 << 0)

#define BSCSL_FR_TXFF     (1 << 2)
#define BSCSL_FR_RXCOUNT_SHIFT 11
#define BSCSL_FR_RXCOUNT_MASK  (0x1F << BSCSL_FR_RXCOUNT_SHIFT)
#define BSCSL_FR_TXCOUNT_SHIFT 6
#define BSCSL_FR_TXCOUNT_MASK  (0x1F << BSCSL_FR_TXCOUNT_SHIFT)

#define BSCSL_RSR_TXUE    (1 << 1)
#define BSCSL_RSR_RXOE    (1 << 0)

void active_sleep(unsigned int ticks)
{
    unsigned int ra;
    for (ra = 0; ra < ticks; ra++)
        dummy(ra);
}

void gpio_set_function(unsigned int pin, unsigned int func)
{
    unsigned int reg = GPFSEL0 + ((pin / 10) * 4);
    unsigned int shift = (pin % 10) * 3;
    unsigned int value = read32(reg);

    value &= ~(7 << shift);
    value |= func << shift;
    write32(reg, value);
}

void enable_pullup(unsigned int pin)
{
    write32(GPPUD, 2);
    active_sleep(150);
    write32(GPPUDCLK0, 1 << pin);
    active_sleep(150);
    write32(GPPUD, 0);
    write32(GPPUDCLK0, 0);
}

unsigned int timer_get_low(void)
{
    return read32(ST_CLO);
}

void led_set(unsigned int on)
{
    if (on)
        write32(GPCLR1, 1 << (47 - 32));
    else
        write32(GPSET1, 1 << (47 - 32));
}

void init_led(void)
{
    unsigned int ra = read32(GPFSEL4);

    ra &= ~(7 << 21);
    ra |= 1 << 21;
    write32(GPFSEL4, ra);
    led_off();
}

void bsc_slave_disable(void)
{
    write32(BSCSL_CR, BSCSL_CR_BRK);
    write32(BSCSL_CR, 0);
}

void bsc_slave_enable(void)
{
    write32(BSCSL_CR,
        BSCSL_CR_RXE |
        BSCSL_CR_TXE |
        BSCSL_CR_I2C |
        BSCSL_CR_EN
    );
}

void bsc_slave_clear_errors(void)
{
    if (read32(BSCSL_RSR) & (BSCSL_RSR_TXUE | BSCSL_RSR_RXOE))
        write32(BSCSL_RSR, 0);
}

unsigned int bsc_slave_rx_count(void)
{
    return (read32(BSCSL_FR) & BSCSL_FR_RXCOUNT_MASK) >> BSCSL_FR_RXCOUNT_SHIFT;
}

unsigned int bsc_slave_tx_count(void)
{
    return (read32(BSCSL_FR) & BSCSL_FR_TXCOUNT_MASK) >> BSCSL_FR_TXCOUNT_SHIFT;
}

void bsc_slave_queue_tx_byte(unsigned int value)
{
    if ((read32(BSCSL_FR) & BSCSL_FR_TXFF) == 0)
        write32(BSCSL_DR, value & 0xFF);
}

void init_i2c_slave(void)
{
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_INPUT);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_INPUT);
    enable_pullup(I2C_SLAVE_SDA_PIN);
    enable_pullup(I2C_SLAVE_SCL_PIN);

    bsc_slave_disable();
    bsc_slave_clear_errors();

    write32(BSCSL_SLV, I2C_SLAVE_ADDRESS);
    write32(BSCSL_ICR, 0xF);

    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_ALT3);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_ALT3);

    write32(GPEDS0, (1 << I2C_SLAVE_SDA_PIN) | (1 << I2C_SLAVE_SCL_PIN));
    write32(GPREN0, read32(GPREN0) | (1 << I2C_SLAVE_SDA_PIN) | (1 << I2C_SLAVE_SCL_PIN));
    write32(GPFEN0, read32(GPFEN0) | (1 << I2C_SLAVE_SDA_PIN) | (1 << I2C_SLAVE_SCL_PIN));

    bsc_slave_enable();
    bsc_slave_queue_tx_byte(I2C_REPLY_BYTE);
}

int blinker_main(void)
{
    unsigned int led_state = 0;
    unsigned int last_toggle_time = 0;

    init_led();
    init_i2c_slave();

    while (1)
    {
        unsigned int gpio_events;
        unsigned int rx_count;

        rx_count = bsc_slave_rx_count();
        if (rx_count != 0)
        {
            while (rx_count != 0)
            {
                (void)(read32(BSCSL_DR) & 0xFF);
                rx_count--;
            }

            led_state ^= 1;
            led_set(led_state);
        }

        if (read32(BSCSL_RSR) & (BSCSL_RSR_TXUE | BSCSL_RSR_RXOE))
            bsc_slave_clear_errors();

        if (bsc_slave_tx_count() == 0)
            bsc_slave_queue_tx_byte(I2C_REPLY_BYTE);

        gpio_events = read32(GPEDS0) & ((1 << I2C_SLAVE_SDA_PIN) | (1 << I2C_SLAVE_SCL_PIN));
        if (gpio_events != 0)
        {
            write32(GPEDS0, gpio_events);

            if ((timer_get_low() - last_toggle_time) >= LED_TOGGLE_INTERVAL_US)
            {
                led_state ^= 1;
                led_set(led_state);
                last_toggle_time = timer_get_low();
            }
        }
    }

    return 0;
}
