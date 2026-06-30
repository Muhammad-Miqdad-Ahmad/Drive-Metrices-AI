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
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME Drive-Metrics-AI)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

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
    Core/Src/debug_log.cc
    Core/Src/micro_time.cc
    Core/Src/model_tflite.cc
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    tflite-micro
    tflite-micro/tensorflow/lite/micro/tools/make/downloads/flatbuffers/include
    tflite-micro/tensorflow/lite/micro/tools/make/downloads/kissfft
    tflite-micro/tensorflow/lite/micro/tools/make/downloads/ruy
    CMSIS-DSP/Include
    CMSIS_NN
    tflite-micro/tensorflow/lite/micro/tools/make/downloads/gemmlowp
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
# libtflm.a must come before libcmsis-nn.a: GNU ld scans left-to-right, so
# the library that creates undefined references (libtflm) must precede the one
# that satisfies them (libcmsis-nn), otherwise the linker won't rescan.
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${CMAKE_SOURCE_DIR}/Lib/libtflm.a
    ${CMAKE_SOURCE_DIR}/Lib/libcmsis-nn.a
    ${CMAKE_SOURCE_DIR}/Lib/libCMSISDSP.a
    # Add user defined libraries
)
