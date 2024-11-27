# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

project(basic_dma_test)
# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Specify the source files
set(BASICTESTSOURCES
	basic_dma_test.c
	mailbox.c
	mem_utils.c
	pi_board.c
)
# Set the build type to Debug 
set(CMAKE_BUILD_TYPE Debug)

# Set compiler flags for the Debug build type 
set(CMAKE_C_FLAGS_DEBUG "-g -O0")


# Add the executable
add_executable(basic_dma_test ${BASICTESTSOURCES})

target_link_libraries(basic_dma_test bcm_host)


project(basic_pixels_test)
# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Specify the source files
set(BASICPIXELTESTSOURCES
	basic_pixels_test.c
	mailbox.c
	mem_utils.c
	pi_board.c
)
# Set the build type to Debug 
set(CMAKE_BUILD_TYPE Debug)

# Set compiler flags for the Debug build type 
set(CMAKE_C_FLAGS_DEBUG "-g -O0")


# Add the executable
add_executable(basic_pixels_test ${BASICPIXELTESTSOURCES})

target_link_libraries(basic_pixels_test bcm_host)


project(rpi_pixels_receiver)
# Set the C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Specify the source files
set(PIXELRECEIVERSOURCES
	rpi_pixels_receiver.c
	mailbox.c
	mem_utils.c
	pi_board.c
	ringbuf.c
)
# Set the build type to Debug 
set(CMAKE_BUILD_TYPE Debug)

# Set compiler flags for the Debug build type 
set(CMAKE_C_FLAGS_DEBUG "-g -O0")


# Add the executable
add_executable(rpi_pixels_receiver ${PIXELRECEIVERSOURCES})

target_link_libraries(rpi_pixels_receiver bcm_host)

