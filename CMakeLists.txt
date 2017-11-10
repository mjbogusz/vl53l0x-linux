cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)

project(VL53L0X_linux)

### Setup compiler
# Set warning flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat=2 -Wall -Wextra")
# Set optimalization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

### Main library target
# Set source files
set(VL53L0X_SOURCE_FILES
        VL53L0X.cpp)

# Set up target
add_library(vl53l0x SHARED ${VL53L0X_SOURCE_FILES})
set_target_properties(vl53l0x PROPERTIES
        VERSION ${LIB_VERSION}
        SOVERSION ${LIB_VERSION_MAJOR})

### WiringPi
target_link_libraries(vl53l0x LINK_PUBLIC wiringPi)
add_dependencies(vl53l0x LinuxDuino)