cmake_minimum_required(VERSION 3.25)

project(SO_ARM_101_Controller)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# define header files
include_directories(
  include
  drivers/linux_serial
  drivers/sts3215
)

# define source files
set(SOURCES
  src/main.cpp
  drivers/linux_serial/linux_serial.cpp
  drivers/sts3215/sts3215.cpp
)

# the executable
add_executable(servo_test ${SOURCES})

# enabling compile warnings
target_compile_options(servo_test PRIVATE -Wall -Wextra - Wpedantic)
