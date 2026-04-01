#pragma once

#include <cstdint>

namespace handshake
{
    static constexpr std::uint8_t MAGIC_BYTE = 0x5A;

    enum class MessageType : std::uint8_t
    {
        Conf = 0,
        Response = 1,
        Disconnect = 3,
    };

    enum class ProtocolID : std::uint8_t
    {
        UART = 0,
        I2C = 1,
    };

    struct UARTConfig
    {
        std::uint32_t baudrate;
        std::uint8_t data_bits;
        std::uint8_t start_bits;
        std::uint8_t parity_bits;
        std::uint8_t stop_bits;
    } __attribute__((packed));

    struct I2CConfig
    {
        std::uint32_t bus_id;
        std::uint8_t is_master;
        std::uint8_t address;
    } __attribute__((packed));

    struct ConfMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Conf;
        ProtocolID protocol_id;
        std::uint16_t port;
        std::uint32_t net_id;
        union {
            UARTConfig uart;
            I2CConfig i2c;
        } config;
    } __attribute__((packed));

    struct ResponseMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Response;
        std::uint8_t status; // 1: Accept, 0: Decline
        std::uint16_t port;
        std::uint32_t net_id;
    } __attribute__((packed));

    struct DisconnectMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Disconnect;
        ConfMessage config;
    } __attribute__((packed));
}
