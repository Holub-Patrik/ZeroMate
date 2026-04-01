include_guard()

cmake_minimum_required(VERSION 3.5)

# Add the gpio_server peripheral as a shared library
add_library(gpio_server SHARED)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(gpio_server
    PRIVATE 
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(gpio_server
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
)

# Specify the source file making up the gpio_server peripheral
target_sources(gpio_server
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio_server.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio_server.cpp
)

# Link imgui_glfw, logging_system, and fmt
target_link_libraries(gpio_server
    PRIVATE
        imgui_glfw
        logging_system
        fmt
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET gpio_server
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:gpio_server> ${output_directory}
)
