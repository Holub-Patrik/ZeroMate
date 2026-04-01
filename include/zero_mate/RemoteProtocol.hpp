#pragma once

#include <cstdint>
#include <cstddef>

#ifdef _WIN32
  #include <windows.h>
  #define LIB_SELF()        GetModuleHandle(NULL)
  #define LIB_SYM(h, name)  (void*)GetProcAddress((HMODULE)h, name)
#else
  #include <dlfcn.h>
  #define LIB_SELF()        dlopen(NULL, RTLD_NOW)
  #define LIB_SYM(h, name)  dlsym(h, name)
#endif

namespace zero_mate::remote_protocol
{
    // Handshake result callback: called to notify if the connection was successful or not.
    typedef void (*handshake_result_callback_t)(void* context, bool success);

    // Receive callback: called when generic data arrives from the remote side.
    typedef void (*receive_callback_t)(void* context, const void* data, size_t size);

    // Disconnect callback: called when the remote side disconnects or the connection is lost.
    typedef void (*disconnect_callback_t)(void* context);

    // Comparison function: returns true if the incoming component-specific handshake payload matches.
    // The server will pass the payload extracted from the ConfMessage.
    typedef bool (*comparison_func_t)(void* context, const void* payload, size_t size);

    // Registration function exported by GPIOServer
    typedef uint32_t (*register_t)(const char* protocol, 
                                   comparison_func_t comp_func,
                                   receive_callback_t on_receive, 
                                   disconnect_callback_t on_disconnect,
                                   handshake_result_callback_t on_handshake_result,
                                   void* context);

    // Unregistration function exported by GPIOServer
    typedef void (*unregister_t)(uint32_t id);

    // Sending function exported by GPIOServer
    typedef void (*send_t)(uint32_t id, const void* data, size_t size);

    // Handshake initiation exported by GPIOServer
    // remote_ip/port: where to send the ConfMessage
    // comp_payload: component-specific data (e.g. protocol ID, baudrate, net_id)
    typedef void (*init_handshake_t)(uint32_t id, const char* remote_ip, uint16_t remote_port, const void* comp_payload, size_t size);
}
