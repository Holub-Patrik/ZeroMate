#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <gtest/gtest.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <functional>
#include <cstdio>

#include "../../peripherals/net_comm/src/I2C_Handler.hpp"
#include "../../peripherals/net_comm/src/UART_Handler.hpp"

// --- Mocks with Timeouts ---

struct Mock_CPU
{
    std::mutex mtx;
    std::condition_variable cv;
    int balance = 0;

    void halt()
    {
        std::unique_lock<std::mutex> lock(mtx);
        balance++;
        cv.notify_all();
    }

    void start()
    {
        std::unique_lock<std::mutex> lock(mtx);
        balance--;
        cv.notify_all();
    }

    bool wait_for_ready(std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
    {
        std::unique_lock<std::mutex> lock(mtx);
        return cv.wait_for(lock, timeout, [this] { return balance <= 0; });
    }

    void reset()
    {
        balance = 0;
    }
};

Mock_CPU g_master_cpu;
void master_halt()
{
    g_master_cpu.halt();
}
void master_start()
{
    g_master_cpu.start();
}

// --- Mock Peripheral ---

struct Mock_I2C_Peripheral
{
    uint8_t address;
    uint32_t sda_pin;
    uint32_t scl_pin;
    std::function<void(uint32_t, bool)> set_pin_func;

    uint8_t shift_reg = 0;
    int bit_count = 0;
    bool matched = false;
    bool is_read = false;
    bool scl_lvl = true;
    bool sda_lvl = true;

    std::vector<uint8_t> received_data;
    std::vector<uint8_t> data_to_send;
    size_t send_idx = 0;
    int bit_count_internal = 0;

    enum class State
    {
        IDLE,
        ADDR,
        DATA,
        ACK
    };
    State m_state = State::IDLE;

    Mock_I2C_Peripheral(uint8_t addr, uint32_t sda, uint32_t scl, std::function<void(uint32_t, bool)> set_pin)
    : address(addr)
    , sda_pin(sda)
    , scl_pin(scl)
    , set_pin_func(set_pin)
    {
    }

    void on_gpio_change(uint32_t pin, bool val)
    {
        if (pin == sda_pin)
        {
            if (scl_lvl)
            {
                if (sda_lvl && !val) // START
                {
                    m_state = State::ADDR;
                    bit_count = 0;
                    shift_reg = 0;
                    matched = false;
                    bit_count_internal = 0;
                }
                else if (!sda_lvl && val) // STOP
                {
                    m_state = State::IDLE;
                }
            }
            sda_lvl = val;
        }
        else if (pin == scl_pin)
        {
            if (!scl_lvl && val) // Rising edge
            {
                if (m_state == State::ADDR || m_state == State::DATA)
                {
                    bit_count++;
                    if (bit_count <= 8)
                    {
                        shift_reg = (shift_reg << 1) | (sda_lvl ? 1 : 0);
                    }
                }
            }
            else if (scl_lvl && !val) // Falling edge
            {
                if (bit_count == 8)
                {
                    if (m_state == State::ADDR)
                    {
                        matched = (shift_reg >> 1) == address;
                        is_read = shift_reg & 1;
                        if (matched)
                        {
                            set_pin_func(sda_pin, false); // ACK
                            if (is_read)
                            {
                                prepare_next_data_bit(); // Bit 0
                            }
                        }
                        m_state = State::ACK;
                    }
                    else if (m_state == State::DATA)
                    {
                        if (matched)
                        {
                            if (!is_read)
                            {
                                received_data.push_back(shift_reg);
                                set_pin_func(sda_pin, false); // ACK
                            }
                        }
                        m_state = State::ACK;
                    }
                    bit_count = 0;
                    shift_reg = 0;
                }
                else if (m_state == State::ACK)
                {
                    m_state = State::DATA;
                    if (is_read && matched)
                    {
                        prepare_next_data_bit(); // Bits 1-7
                    }
                }
                else if (m_state == State::DATA && is_read && matched)
                {
                    prepare_next_data_bit();
                }
            }
            scl_lvl = val;
        }
    }

    void prepare_next_data_bit()
    {
        if (send_idx < data_to_send.size())
        {
            bool bit = (data_to_send[send_idx] >> (7 - bit_count_internal)) & 1;
            set_pin_func(sda_pin, bit);
            bit_count_internal++;
            if (bit_count_internal == 8)
            {
                bit_count_internal = 0;
                send_idx++;
            }
        }
    }

    void reset()
    {
        m_state = State::IDLE;
        bit_count = 0;
        bit_count_internal = 0;
        shift_reg = 0;
        matched = false;
        send_idx = 0;
        received_data.clear();
    }
};

// --- Tests ---

TEST(net_comm, i2c_combined_interaction)
{
    g_master_cpu.reset();

    int sv[2];
    ASSERT_GE(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    const uint32_t SCL = 3;
    const uint32_t SDA = 2;
    const uint8_t SLAVE_ADDR = 0x50;

    I2C_Master_P m_cfg;
    m_cfg.scl_pin = SCL;
    m_cfg.sda_pin = SDA;
    m_cfg.slave_fds.push_back(sv[0]);

    I2C_Slave_P s_cfg;
    s_cfg.address = SLAVE_ADDR;
    s_cfg.scl_pin = SCL;
    s_cfg.sda_pin = SDA;
    s_cfg.master_fd = sv[1];

    bool current_sda = true;
    bool peripheral_sda = true;

    I2C_Master<128> master(m_cfg, master_halt, master_start, [&](uint8_t pin, uint8_t val) {
        if (pin == SDA)
        {
            current_sda = (val != 0);
        }
    }, [&](uint8_t pin) { 
        if (pin == SDA) return (current_sda && peripheral_sda) ? (uint8_t)1 : (uint8_t)0;
        return (uint8_t)1;
    });

    Mock_I2C_Peripheral peripheral(SLAVE_ADDR, SDA, SCL, nullptr);
    peripheral.data_to_send = { 0x55 };

    I2C_Slave<128> slave(
    s_cfg,
    []() { },
    []() { },
    [&](uint8_t pin, uint8_t val) { 
        if (pin == SDA) current_sda = (val != 0); // Slave proxy driving local bus
        peripheral.on_gpio_change(pin, val != 0); 
    },
    [&](uint8_t pin) { 
        if (pin == SDA) return (current_sda && peripheral_sda) ? (uint8_t)1 : (uint8_t)0;
        return (uint8_t)1;
    });

    peripheral.set_pin_func = [&](uint32_t pin, bool val) { 
        if (pin == SDA) peripheral_sda = val;
    };

    master.start_receiver();
    slave.start_receiver();

    auto emit_master_bit = [&](bool val, bool is_scl) {
        master.process_bit({ val ? 1 : 0, is_scl ? SCL : SDA }, 1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };

    // 1. WRITE Transaction (0xAA)
    emit_master_bit(true, true);
    emit_master_bit(true, false);
    emit_master_bit(false, false);

    uint8_t addr_byte = 0xA0;
    for (int i = 7; i >= 0; --i)
    {
        emit_master_bit(false, true);
        emit_master_bit((addr_byte >> i) & 1, false);
        emit_master_bit(true, true);
    }
    emit_master_bit(false, true);

    ASSERT_TRUE(g_master_cpu.wait_for_ready());

    emit_master_bit(true, true); // ACK rising
    EXPECT_EQ(current_sda, false);
    emit_master_bit(false, true); // ACK falling

    ASSERT_TRUE(g_master_cpu.wait_for_ready());

    uint8_t data_byte = 0xAA;
    for (int i = 7; i >= 0; --i)
    {
        emit_master_bit(false, true);
        emit_master_bit((data_byte >> i) & 1, false);
        emit_master_bit(true, true);
    }
    emit_master_bit(false, true);

    ASSERT_TRUE(g_master_cpu.wait_for_ready());

    emit_master_bit(true, true); // ACK rising
    EXPECT_EQ(current_sda, false);
    emit_master_bit(false, true); // ACK falling

    ASSERT_FALSE(peripheral.received_data.empty());
    EXPECT_EQ(peripheral.received_data[0], 0xAA);

    // 2. REPEATED START
    emit_master_bit(true, false);
    emit_master_bit(true, true);
    emit_master_bit(false, false);

    // 3. READ Transaction (expecting 0x55)
    addr_byte = 0xA1;
    for (int i = 7; i >= 0; --i)
    {
        emit_master_bit(false, true);
        emit_master_bit((addr_byte >> i) & 1, false);
        emit_master_bit(true, true);
    }
    emit_master_bit(false, true);

    ASSERT_TRUE(g_master_cpu.wait_for_ready());

    emit_master_bit(true, true); // ACK rising
    EXPECT_EQ(current_sda, false);
    emit_master_bit(false, true); // ACK falling

    ASSERT_TRUE(g_master_cpu.wait_for_ready());

    uint8_t master_read_byte = 0;
    for (int i = 0; i < 8; ++i)
    {
        emit_master_bit(true, true);
        master_read_byte = (master_read_byte << 1) | (current_sda ? 1 : 0);
        emit_master_bit(false, true);

        ASSERT_TRUE(g_master_cpu.wait_for_ready());
    }

    EXPECT_EQ(master_read_byte, 0x55);

    emit_master_bit(true, true);  // Master ACK rising
    emit_master_bit(false, true); // Master ACK falling

    // 4. STOP
    emit_master_bit(false, false);
    emit_master_bit(true, true);
    emit_master_bit(true, false);

    master.receiver_stop();
    slave.receiver_stop();
    close(sv[0]);
    close(sv[1]);
}

TEST(net_comm, i2c_read_interaction)
{
    g_master_cpu.reset();

    int sv[2];
    ASSERT_GE(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    const uint32_t SCL = 3;
    const uint32_t SDA = 2;
    const uint8_t SLAVE_ADDR = 0x50;

    I2C_Master_P m_cfg;
    m_cfg.scl_pin = SCL;
    m_cfg.sda_pin = SDA;
    m_cfg.slave_fds.push_back(sv[0]);

    I2C_Slave_P s_cfg;
    s_cfg.address = SLAVE_ADDR;
    s_cfg.scl_pin = SCL;
    s_cfg.sda_pin = SDA;
    s_cfg.master_fd = sv[1];

    bool current_sda = true;
    bool peripheral_sda = true;

    I2C_Master<128> master(m_cfg, master_halt, master_start, [&](uint8_t pin, uint8_t val) {
        if (pin == SDA)
        {
            current_sda = (val != 0);
        }
    }, [&](uint8_t pin) { 
        if (pin == SDA) return (current_sda && peripheral_sda) ? (uint8_t)1 : (uint8_t)0;
        return (uint8_t)1;
    });

    Mock_I2C_Peripheral peripheral(SLAVE_ADDR, SDA, SCL, nullptr);
    peripheral.data_to_send = { 0x55 };

    I2C_Slave<128> slave(
    s_cfg,
    []() { },
    []() { },
    [&](uint8_t pin, uint8_t val) { 
        if (pin == SDA) current_sda = (val != 0); // Slave proxy driving local bus
        peripheral.on_gpio_change(pin, val != 0); 
    },
    [&](uint8_t pin) { 
        if (pin == SDA) return (current_sda && peripheral_sda) ? (uint8_t)1 : (uint8_t)0;
        return (uint8_t)1;
    });

    peripheral.set_pin_func = [&](uint32_t pin, bool val) { 
        if (pin == SDA) peripheral_sda = val;
    };

    master.start_receiver();
    slave.start_receiver();

    auto emit_master_bit = [&](bool val, bool is_scl) {
        master.process_bit({ val ? 1 : 0, is_scl ? SCL : SDA }, 1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    };

    // 1. START
    emit_master_bit(true, true);
    emit_master_bit(true, false);
    emit_master_bit(false, false);

    // 2. Address 0xA1 (Read)
    uint8_t addr_byte = 0xA1;
    for (int i = 7; i >= 0; --i)
    {
        emit_master_bit(false, true);
        emit_master_bit((addr_byte >> i) & 1, false);
        emit_master_bit(true, true);
    }
    emit_master_bit(false, true);

    ASSERT_TRUE(g_master_cpu.wait_for_ready());

    emit_master_bit(true, true); // ACK rising
    EXPECT_EQ(current_sda, false);
    emit_master_bit(false, true); // ACK falling

    ASSERT_TRUE(g_master_cpu.wait_for_ready());

    uint8_t master_read_byte = 0;
    for (int i = 0; i < 8; ++i)
    {
        emit_master_bit(true, true);
        master_read_byte = (master_read_byte << 1) | (current_sda ? 1 : 0);
        emit_master_bit(false, true);

        ASSERT_TRUE(g_master_cpu.wait_for_ready());
    }

    EXPECT_EQ(master_read_byte, 0x55);

    emit_master_bit(true, true);  // Master ACK rising
    emit_master_bit(false, true); // Master ACK falling

    // 5. STOP
    emit_master_bit(false, false);
    emit_master_bit(true, true);
    emit_master_bit(true, false);

    master.receiver_stop();
    slave.receiver_stop();
    close(sv[0]);
    close(sv[1]);
}

TEST(net_comm, uart_basic)
{
    int sv[2];
    ASSERT_GE(socketpair(AF_UNIX, SOCK_DGRAM, 0, sv), 0);

    UART_P config;
    config.baudrate = 115200;
    config.tx_pin = 14;
    config.rx_pin = 15;
    config.start_bits = 1;
    config.data_bits = 8;
    config.parity_bits = 0;
    config.stop_bits = 1;
    config.other_side_fd = sv[0];

    auto pin_write_mock = [](uint8_t, uint8_t) { };
    UART_Handler<64> uart(config, pin_write_mock);
    uart.start_receiver();

    std::vector<bool> bits = { 0, 1, 0, 1, 0, 1, 0, 1, 0, 1 };
    for (bool b : bits)
    {
        uart.process_bit({ b ? 1 : 0, config.tx_pin }, 1000);
    }

    std::array<uint32_t, 64> recv_buf;
    ssize_t n = recv(sv[1], recv_buf.data(), recv_buf.size() * sizeof(uint32_t), MSG_DONTWAIT);
    ASSERT_GT(n, 0);
    EXPECT_EQ(n / sizeof(uint32_t), 10);

    std::vector<bool> rx_bits = { 0, 0, 1, 0, 1, 0, 1, 0, 1, 1 };
    std::vector<uint32_t> packed_rx;
    for (bool b : rx_bits)
    {
        uint32_t p = 1000;
        if (b)
            p |= (1U << 31U);
        packed_rx.push_back(p);
    }
    send(sv[1], packed_rx.data(), packed_rx.size() * sizeof(uint32_t), 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    uart.receiver_stop();
    close(sv[0]);
    close(sv[1]);
}
