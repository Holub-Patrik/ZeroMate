#pragma once

#include <cstdint>
#include <cstddef>

#ifdef _WIN32
    #include <windows.h>
    #define LIB_OPEN_SERVER(name) GetModuleHandle(name)
    #define LIB_LOOKUP_SYMBOL(h, name) (void*)GetProcAddress((HMODULE)h, name)
    #define LIB_NAME(name) name ".dll"
#else
    #include <dlfcn.h>
    #define LIB_OPEN_SERVER(name) dlopen(name, RTLD_NOW | RTLD_NOLOAD)
    #define LIB_LOOKUP_SYMBOL(h, name) dlsym(h, name)
    #define LIB_NAME(name) "lib" name ".so"
#endif

namespace zero_mate::remote_protocol
{
    // Handshake result callback: called to notify if the connection was successful or not.
    using handshake_result_callback_t =
    void (*)(void* context, bool success, int fd, const char* remote_ip, uint16_t remote_port);

    // Disconnect callback: called when the remote side disconnects or the connection is lost.
    using disconnect_callback_t = void (*)(void* context, const char* remote_ip, uint16_t remote_port);

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

    // Disconnect function exported by GPIOServer
    using disconnect_t = void (*)(int fd);
}
