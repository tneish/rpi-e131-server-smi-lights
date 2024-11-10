# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(rpi_neopixel_smi)

# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Specify the source files
set(SOURCES 
    rpi_pixleds.c 
    rpi_dma_utils.c
)

# Add the executable
add_executable(rpi_neopixel_smi ${SOURCES})

