cmake_minimum_required(VERSION 3.25)

project(SO_ARM_101_Controller)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# define header files
include_directories(
	cfg
	drivers/linux_serial
	drivers/sts3215
	include
)

# define source files
set(SOURCES
	src/arm_calibration.cpp
	src/arm_kinematics.cpp
	src/main.cpp
	drivers/linux_serial/linux_serial.cpp
	drivers/sts3215/sts3215.cpp
)

# the executable
add_executable(servo_test ${SOURCES})

# enabling compile warnings
target_compile_options(servo_test PRIVATE -Wall -Wextra -Wpedantic)

# MuJoCo SO-ARM101 loader
set(MUJOCO_ROOT "$ENV{HOME}/.mujoco/mujoco-3.6.0")
set(SO101_SCENE_XML "${CMAKE_CURRENT_SOURCE_DIR}/sim/scene.xml")

add_executable(mujoco_loader src/mujoco_loader.cpp)
target_include_directories(mujoco_loader PRIVATE ${MUJOCO_ROOT}/include)
target_link_libraries(mujoco_loader PRIVATE ${MUJOCO_ROOT}/lib/libmujoco.so)
target_compile_definitions(mujoco_loader PRIVATE
    SCENE_XML_PATH="${SO101_SCENE_XML}"
    MUJOCO_PLUGIN_DIR="${MUJOCO_ROOT}/bin/mujoco_plugin"
)
target_compile_options(mujoco_loader PRIVATE -Wall -Wextra -Wpedantic)
