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
set(CMAKE_PROJECT_NAME smart_car)

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

# Keep critical runtime fixes outside CubeMX-generated files.
set(CUBEMX_OVERRIDDEN_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32f4xx_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32f4xx_hal_timebase_tim.c
)
set_source_files_properties(${CUBEMX_OVERRIDDEN_SOURCES} PROPERTIES HEADER_FILE_ONLY TRUE)
target_compile_definitions(stm32cubemx INTERFACE HSE_VALUE=8000000U)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    ${CMAKE_SOURCE_DIR}/Core/AI
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/font.c
    Core/Src/oled.c
    Core/App/Platform/stm32f4xx_hal_timebase_tim_user.c
    Core/App/Platform/stm32f4xx_it_user.c
    Core/App/Platform/runtime_diag.c
    Core/App/Tasks/CtrlTask.c
    Core/App/Tasks/DriverTask.c
    Core/App/Tasks/HCSR04.c
    Core/App/Tasks/LedTask.c
    Core/App/Tasks/OledTask.c
    Core/App/Tasks/RfidTask.c
    Core/App/Tasks/SensorTask.c
    Core/App/Tasks/UartTask.c
    Core/App/Tasks/WifiTask.c
    Core/App/Tasks/AITask.c
    Core/App/Driver/Aht20.c
    Core/App/Driver/motor.c
    Core/App/Driver/pid.c
    Core/App/Driver/RfidReader.c
    Core/App/Module/BatteryCtrl.c
    Core/App/Module/Encoder.c
    Core/App/Module/ObstacleCtrl.c
    Core/App/Module/SelfTest.c
    Core/App/Module/ThermalCtrl.c
    Core/App/Module/TrackCtrl.c
    Core/App/Module/WifiComm.c
    Core/App/Module/AIStatus.c
    Core/App/Module/AIAnomalyDetect.c
    Core/App/Module/VoltageDetect.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/App/Tasks
    ${CMAKE_SOURCE_DIR}/Core/App/Driver
    ${CMAKE_SOURCE_DIR}/Core/App/Module
    ${CMAKE_SOURCE_DIR}/Core/App/Platform
    ${CMAKE_SOURCE_DIR}/Core/AI
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    neai
    m
)
