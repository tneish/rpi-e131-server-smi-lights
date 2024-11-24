# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

project(basic_dma_test)
# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Specify the source files
set(BASICTESTSOURCES
	basic_dma_test.cpp
	mailbox.cpp
	mem_utils.cpp
	pi_board.cpp
)
# Set the build type to Debug 
if(NOT CMAKE_BUILD_TYPE) 
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Set compiler flags for the Debug build type 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")


# Add the executable
add_executable(basic_dma_test ${BASICTESTSOURCES})

target_link_libraries(basic_dma_test bcm_host)



