# @file CMakeLists.txt
# @brief 本板固件的 CMake 入口
# @board 底盘板
# @vehicle_function 把应用代码、CubeMX 代码和启动文件链接为本板固件
# @inputs CMake 预设、工具链文件及工程源文件列表
# @outputs CMake 固件构建目标
# @dependencies CMake、cmake/stm32cubemx 和 Arm 工具链
# @note 重构说明：本文件行为与原代码保持一致。手写模块通过顶层 target_sources 登记。

cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/chassis_control_loop.c
    Core/Src/chassis_motion_control.c
    Core/Src/chassis_wheel_kinematics.c
    Core/Src/chassis_motor_driver.c
    Core/Src/chassis_can_bus.c
    Core/Src/chassis_imu_feedback.c
    Core/Src/chassis_remote_communication.c
    Core/Src/chassis_remote_input.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    m          
)
