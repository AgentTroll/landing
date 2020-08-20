cmake_minimum_required(VERSION 3.17)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/Arduino-CMake-Toolchain/Arduino-toolchain.cmake")
project(landing
        VERSION 1.0
        LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)

set(ARDUINO_BOARD_OPTIONS_FILE "${CMAKE_SOURCE_DIR}/Arduino-CMake-Toolchain/BoardOptions.cmake")

add_executable(landing main.cpp)
target_link_arduino_libraries(landing
        PRIVATE core
        PRIVATE FreeRTOS)
target_enable_arduino_upload(landing)

set(DEPLOY_PORT "/dev/ttyUSB0" CACHE STRING "The default port for the deploy command")
add_custom_target(deploy
        "${CMAKE_COMMAND}"
        "--build" "${CMAKE_BINARY_DIR}"
        "--target" "upload" "SERIAL_PORT=${DEPLOY_PORT}")
