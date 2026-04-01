include_guard()

cmake_minimum_required(VERSION 3.5)

# Add the remote_uart peripheral as a shared library
add_library(remote_uart SHARED)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(remote_uart
    PRIVATE 
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(remote_uart
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
)

# Specify the source file making up the remote_uart peripheral
target_sources(remote_uart
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/remote_uart.cpp
)

# Link imgui_glfw, logging_system, and fmt
target_link_libraries(remote_uart
    PRIVATE
        imgui_glfw
        logging_system
        fmt
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET remote_uart
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:remote_uart> ${output_directory}
)
