#pragma once

#include <chrono>
#include <vector>
#include <cstdint>
#include <array>

class Parser
{
public:
    Parser() = default;

    Parser(const Parser&) = delete;
    Parser& operator=(const Parser&) = delete;

    Parser(Parser&&) = delete;
    Parser& operator=(Parser&&) = delete;

    virtual ~Parser() = default;

    virtual bool accumulate_bit(const std::uint8_t bit) = 0;
    virtual std::vector<std::uint8_t> get_packet_data(bool implicit_clock) = 0;
};

class UARTParser final : public Parser
{
private:
    static constexpr std::uint64_t PROTOCOL_MSG_BIT_LENGTH = 8;

    enum phase_enum : std::uint8_t
    {
        START,
        DATA,
        PARITY,
        STOP
    } phase{ START };

    struct config
    {
        int start_bits, data_length, parity_bits, stop_bits;
    } conf;

    struct parse_progress
    {
        int start_bits, data_length, parity_bits, stop_bits;
    } progress;

    std::chrono::time_point<std::chrono::high_resolution_clock> time_point{ std::chrono::high_resolution_clock::now() };
    std::vector<std::tuple<std::uint8_t, std::uint32_t>> accumulator;

public:
    UARTParser(int start_bits, int data_length, int parity_bits, int stop_bits)
    : accumulator(PROTOCOL_MSG_BIT_LENGTH)
    , conf({ .start_bits = start_bits, .data_length = data_length, .parity_bits = parity_bits, .stop_bits = stop_bits })
    , progress({ .start_bits = 0, .data_length = 0, .parity_bits = 0, .stop_bits = 0 })
    {
    }

    bool accumulate_bit(const std::uint8_t bit) final
    {
        const auto now = std::chrono::high_resolution_clock::now();
        const auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - time_point).count();

        switch (phase)
        {
            case START:
                progress.start_bits++;
                time_point = std::chrono::high_resolution_clock::now();

                if (progress.start_bits >= conf.start_bits)
                {
                    phase = DATA;
                }
                return false;

            case DATA:
                progress.data_length++;
                accumulator.emplace_back(bit, diff);

                if (progress.data_length >= conf.data_length)
                {
                    phase = PARITY;
                }
                break;

            case PARITY:
                progress.parity_bits++;
                accumulator.emplace_back(bit, diff);
                if (progress.parity_bits >= conf.parity_bits)
                {
                    phase = STOP;
                }
                break;

            case STOP:
                progress.start_bits++;
                accumulator.emplace_back(bit, diff);

                if (progress.stop_bits >= conf.start_bits)
                {
                    phase = START;
                }
                return true;
        }
        return false;
    }

    std::vector<std::uint8_t> get_packet_data(bool implicit_clock) final
    {
        std::vector<std::uint8_t> packet{};

        if (implicit_clock)
        {
            packet.reserve(accumulator.size());
            for (const auto [bit, diff] : accumulator)
            {
                packet.emplace_back(bit);
            }
        }
        else
        {
            packet.reserve(accumulator.size() * sizeof(std::uint32_t));
            for (const auto [bit, diff] : accumulator)
            {
                constexpr unsigned int byte_width = 8;
                constexpr unsigned int u32_width = (sizeof(std::uint32_t) * byte_width) - 1;
                std::uint32_t bit_and_diff = diff;
                bit_and_diff = bit_and_diff & ~(1U << u32_width);  // clear top bit
                bit_and_diff |= (bit == 0 ? 0U : 1U) << u32_width; // set bit to top bit

                const auto bytes = std::bit_cast<std::array<std::uint8_t, sizeof(std::uint32_t)>>(bit_and_diff);
                for (const auto& byte : bytes)
                {
                    packet.emplace_back(byte);
                }
            }
        }

        return packet;
    }
};

class I2CParser final : public Parser
{
public:
    I2CParser()
    {
    }

    bool accumulate_bit(const std::uint8_t bit) final
    {
    }

    std::vector<std::uint8_t> get_packet_data(bool implicit_clock) final
    {
    }
};

class SPIParser final : public Parser
{
    SPIParser()
    {
    }

    bool accumulate_bit(const std::uint8_t bit) final
    {
    }

    std::vector<std::uint8_t> get_packet_data(bool implicit_clock) final
    {
    }
};
