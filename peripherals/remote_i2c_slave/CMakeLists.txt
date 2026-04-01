include_guard()

cmake_minimum_required(VERSION 3.5)

# Add the remote_i2c_slave peripheral as a shared library
add_library(remote_i2c_slave SHARED)

# Define the ZM_EXTERNAL_PERIPHERAL_EXPORT macro
target_compile_definitions(remote_i2c_slave
    PRIVATE 
        ZM_EXTERNAL_PERIPHERAL_EXPORT
)

# Specify the public and private include directories
target_include_directories(remote_i2c_slave
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/imgui
)

# Specify the source file making up the remote_i2c_slave peripheral
target_sources(remote_i2c_slave
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/remote_i2c_slave.cpp
)

# Link imgui_glfw and logging_system
target_link_libraries(remote_i2c_slave
    PRIVATE
        imgui_glfw
        logging_system
)

# Copy the shared library into the output/peripherals directory
set(output_directory ${PROJECT_SOURCE_DIR}/output/peripherals)

add_custom_command(TARGET remote_i2c_slave
    POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${output_directory}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:remote_i2c_slave> ${output_directory}
)
