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
set(CMAKE_PROJECT_NAME M3508test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/Bsp/can/bsp_can.c
        User/Bsp/can/bsp_can.h
        User/Bsp/usb/bsp_usb.c
        User/Bsp/usb/bsp_usb.h
        User/Bsp/bmi088/bsp_BMI088driver.c
        User/Bsp/bmi088/bsp_BMI088driver.h
        User/Bsp/struct_typedef.h

        User/Module/pid/module_pid.c
        User/Module/pid/module_pid.h
        User/Module/bmi088/module_BMI088reg.h
        User/Module/bmi088/module_BMI088Middleware.c
        User/Module/bmi088/module_BMI088Middleware.h

        User/Lib/lib_MahonyAHRS.h
        User/Lib/lib_MahonyAHRS.c
        User/Lib/lib_Config.c
        User/Lib/lib_Config.h
        User/Lib/lib_AKF.c
        User/Lib/lib_AKF.h

        User/App/app_sentry_common.h
        User/App/app_sentry_globals.c
        User/App/app_sentry_globals.h
        User/App/imu_task/app_imu_task.c
        User/App/imu_task/app_imu_task.h
        User/App/autoaim_task/app_autoaim_task.c
        User/App/autoaim_task/app_autoaim_task.h
        User/App/sentry_task/app_sentry_task.c
        User/App/sentry_task/app_sentry_task.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        User/Bsp
        User/Module
        User/Lib
        User/App
        User/Bsp/bmi088

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
)