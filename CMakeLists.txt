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

# Set the build type to Debug 
set(CMAKE_BUILD_TYPE Debug)

# Set compiler flags for the Debug build type 
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

# Add the executable
add_executable(rpi_neopixel_smi ${SOURCES})

project(dma_test)
# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Specify the source files
set(TESTSOURCES
	rpi_dma_test.c
	pi_hw/pi_util.c
	pi_hw/mailbox.c
)
# Set the build type to Debug 
set(CMAKE_BUILD_TYPE Debug)

# Set compiler flags for the Debug build type 
set(CMAKE_C_FLAGS_DEBUG "-g -O0")


# Add the executable
add_executable(rpi_dma_test ${TESTSOURCES})

target_link_libraries(rpi_dma_test bcm_host)






