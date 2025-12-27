cmake_minimum_required(VERSION 3.16)
project(Servo2040 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(OUTPUT_NAME ghost-servo2040)
add_executable(${OUTPUT_NAME} ghost-servo2040.cpp
        main.h
)

target_link_libraries(${OUTPUT_NAME}
        pico_stdlib
        servo2040
        pico_multicore
        analogmux
        analog
        button
)

# enable usb output, disable uart output (so it doesn't confuse any connected servos)
pico_enable_stdio_usb(${OUTPUT_NAME} 1)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)

pico_add_extra_outputs(${OUTPUT_NAME})
