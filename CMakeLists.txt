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
set(CMAKE_PROJECT_NAME intelligent_rescue)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        MyLib/DRV8701/drv8701.c
        MyLib/DRV8701/drv8701.h


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
        MyLib/Inc/button.h
        MyLib/Inc/pid.h
        MyLib/Inc/qmath.h
        MyLib/Inc/si2c.h
        MyLib/Inc/task.h

        MyLib/Src/button.c
        MyLib/Src/pid.c
        MyLib/Src/qmath.c
        MyLib/Src/si2c.c

        User_gx/Inc/app_button.h
        User_gx/Inc/app_encoder.h
        User_gx/Inc/app_pwm.h
        User_gx/Inc/app_usart.h
        User_gx/Inc/gx_delay.h
        User_gx/Inc/JY901S.h

        User_gx/Src/app_button.c
        User_gx/Src/app_encoder.c
        User_gx/Src/app_pwm.c
        User_gx/Src/app_usart.c
        User_gx/Src/gx_delay.c
        User_gx/Src/JY901S.c
        User_gx/Src/irqHandlers.c

        User_gx/App_Motor/app_motor.c
        User_gx/App_Motor/app_motor.h

        User_gx/App_Motor/app_speed.c
        User_gx/App_Motor/app_speed.h

        User_gx/App_Servo/app_servo.c
        User_gx/App_Servo/app_servo.h

        User_gx/App_Protocol/rescue_protocol.c
        User_gx/App_Protocol/rescue_usart2.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        User_gx
        User_gx/Inc
        User_gx/Src
        User_gx/App_Motor
        User_gx/App_Servo
        User_gx/App_Protocol
        MyLib
        MyLib/Inc
        MyLib/Src
        MyLib/DRV8701
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
# 启用浮点数支持
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -u _printf_float")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -u _printf_float")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float -u _printf_long_long")
