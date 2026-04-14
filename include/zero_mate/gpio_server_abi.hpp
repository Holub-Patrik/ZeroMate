#pragma once

#include "zero_mate/RemoteProtocol.hpp"

namespace zero_mate::peripheral
{
    struct TGPIOServerABI
    {
        remote_protocol::register_t register_channel;
        remote_protocol::unregister_t unregister_channel;
        remote_protocol::disconnect_t disconnect_channel;
        remote_protocol::init_handshake_t init_handshake;
    };
}

extern "C"
{
    zero_mate::peripheral::TGPIOServerABI Get_GPIO_Server_ABI();
}
