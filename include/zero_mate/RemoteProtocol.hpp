#pragma once

#include <cstdint>
#include <cstddef>

#ifdef _WIN32
    #include <windows.h>
    #define LIB_SELF() GetModuleHandle(NULL)
    #define LIB_SYM(h, name) (void*)GetProcAddress((HMODULE)h, name)
#else
    #include <dlfcn.h>
    #define LIB_SELF() dlopen(NULL, RTLD_NOW)
    #define LIB_SYM(h, name) dlsym(h, name)
#endif

namespace zero_mate::remote_protocol
{
    // Handshake result callback: called to notify if the connection was successful or not.
    using handshake_result_callback_t =
    void (*)(void* context, bool success, int fd, const char* remote_ip, uint16_t remote_port);

    // Disconnect callback: called when the remote side disconnects or the connection is lost.
    using disconnect_callback_t = void (*)(void* context);

    // Comparison function: returns true if the incoming component-specific handshake payload matches.
    // The server will pass the payload extracted from the ConfMessage.
    using comparison_func_t = bool (*)(void* context, const void* payload, size_t size);

    // Registration function exported by GPIOServer
    using register_t = int (*)(const char* protocol,
                               comparison_func_t comp_func,
                               disconnect_callback_t on_disconnect,
                               handshake_result_callback_t on_handshake_result,
                               void* context);

    // Unregistration function exported by GPIOServer
    using unregister_t = void (*)(int fd);

    // Handshake initiation exported by GPIOServer
    // remote_ip/port: where to send the ConfMessage
    // comp_payload: component-specific data (e.g. protocol ID, baudrate, net_id)
    using init_handshake_t =
    void (*)(int fd, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size);
}
