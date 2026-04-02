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

    // Generic envelope for component-driven handshakes
    struct ConfMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Conf;
        std::uint16_t port; // Port opened by the sender for data
        std::uint16_t payload_size;
        // Followed by payload_size bytes of component-specific data
    } __attribute__((packed));

    struct ResponseMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Response;
        std::uint8_t status;      // 1: Accept, 0: Decline
        std::uint16_t port;           // Port opened by the responder for data
        std::uint16_t initiator_port; // Port of the initiator's data socket
    } __attribute__((packed));

    struct DisconnectMessage
    {
        std::uint8_t magic = MAGIC_BYTE;
        MessageType type = MessageType::Disconnect;
        std::uint16_t port; // Port of the sender's data socket
    } __attribute__((packed));
}
