include_guard()

cmake_minimum_required(VERSION 3.5)

# Add the net_comm peripheral as a shared library
add_library(net_comm SHARED)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(net_comm
    PRIVATE 
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(net_comm
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
)

# Specify the source file making up the net_comm peripheral
target_sources(net_comm
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/net_comm.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/net_comm.cpp
)

# Link imgui_glfw and logging_system to the net_comm library so the peripheral can
# render itself and log messages to the logging system
target_link_libraries(net_comm
    PRIVATE
        imgui_glfw
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET net_comm
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:net_comm> ${output_directory}
)
