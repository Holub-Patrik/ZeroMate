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

#define GPFSEL0 0x20200000
#define GPFSEL4 0x20200010
#define GPSET1  0x20200020
#define GPCLR1  0x2020002C
#define GPPUD   0x20200094
#define GPPUDCLK0 0x20200098
#define ST_CLO  0x20003004
#define BSC1_C  0x20804000
#define BSC1_S  0x20804004
#define BSC1_DLEN 0x20804008
#define BSC1_A  0x2080400C
#define BSC1_FIFO 0x20804010
#define BSC1_DIV 0x20804014
#define BSC1_CLKT 0x2080401C

#define GPIO_INPUT  0
#define GPIO_ALT0   4

#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define I2C_SLAVE_ADDRESS 0x74
#define I2C_TRANSFER_TIMEOUT_US 50000

#define BSC_C_I2CEN (1 << 15)
#define BSC_C_ST    (1 << 7)
#define BSC_C_CLEAR (3 << 4)
#define BSC_C_READ  (1 << 0)

#define BSC_S_CLKT  (1 << 9)
#define BSC_S_ERR   (1 << 8)
#define BSC_S_DONE  (1 << 1)

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

unsigned int timer_get_low(void)
{
    return read32(ST_CLO);
}

int wait_bsc_done(void)
{
    unsigned int start = timer_get_low();

    while ((timer_get_low() - start) < I2C_TRANSFER_TIMEOUT_US)
    {
        unsigned int status = read32(BSC1_S);
        if (status & (BSC_S_DONE | BSC_S_ERR | BSC_S_CLKT))
            return (int)status;
    }

    return -1;
}

void init_i2c_lines(void)
{
    write32(GPPUD, 2);
    active_sleep(150);
    write32(GPPUDCLK0, (1 << I2C_SDA_PIN) | (1 << I2C_SCL_PIN));
    active_sleep(150);
    write32(GPPUD, 0);
    write32(GPPUDCLK0, 0);

    gpio_set_function(I2C_SDA_PIN, GPIO_ALT0);
    gpio_set_function(I2C_SCL_PIN, GPIO_ALT0);

    write32(BSC1_DIV, 150000);
    write32(BSC1_CLKT, 0);
    write32(BSC1_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
}

int send_i2c(unsigned int addr, unsigned int testbyte)
{
    int status;

    write32(BSC1_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
    write32(BSC1_A, addr);
    write32(BSC1_DLEN, 1);
    write32(BSC1_FIFO, testbyte);
    write32(BSC1_C, BSC_C_I2CEN | BSC_C_CLEAR | BSC_C_ST);

    status = wait_bsc_done();
    if ((status < 0) || (status & (BSC_S_ERR | BSC_S_CLKT)))
    {
        write32(BSC1_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
        return 0;
    }

    write32(BSC1_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
    return 1;
}

int recv_i2c(unsigned int addr)
{
    int status;

    write32(BSC1_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
    write32(BSC1_A, addr);
    write32(BSC1_DLEN, 1);
    write32(BSC1_C, BSC_C_I2CEN | BSC_C_CLEAR | BSC_C_ST | BSC_C_READ);

    status = wait_bsc_done();
    if ((status < 0) || (status & (BSC_S_ERR | BSC_S_CLKT)))
    {
        write32(BSC1_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
        return -1;
    }

    status = (int)(read32(BSC1_FIFO) & 0xFF);
    write32(BSC1_S, BSC_S_CLKT | BSC_S_ERR | BSC_S_DONE);
    return status;
}

void blink_led_once(void)
{
    write32(GPCLR1,1<<(47-32));
    active_sleep(0x80000);
    write32(GPSET1,1<<(47-32));
}

int blinker_main(void)
{
    unsigned int ra;
    unsigned int tx_value = 0;
    int reply;

    ra = read32(GPFSEL4);
    ra &= ~(7<<21);
    ra |= 1<<21;
    write32(GPFSEL4,ra);
    write32(GPSET1,1<<(47-32));

    init_i2c_lines();
    blink_led_once();
    active_sleep(0x100000);

    while (1)
    {
        (void)send_i2c(I2C_SLAVE_ADDRESS, tx_value);
        active_sleep(0x20000);

        reply = recv_i2c(I2C_SLAVE_ADDRESS);
        if (reply >= 0)
            blink_led_once();

        tx_value = (tx_value + 1) & 0xFF;
        active_sleep(0x300000);
    }

    return 0;
}
