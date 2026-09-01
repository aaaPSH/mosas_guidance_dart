cmake_minimum_required(VERSION 3.16)
project(serial_package LANGUAGES CXX)

find_package(Threads REQUIRED)

add_library(serial_package STATIC
    src/serial_port.cpp
)

target_compile_features(serial_package PUBLIC cxx_std_17)
target_include_directories(serial_package
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(serial_package PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
target_link_libraries(serial_package PUBLIC Threads::Threads)

add_executable(serial_port_probe
    examples/serial_port_probe.cpp
)
target_compile_features(serial_port_probe PRIVATE cxx_std_17)
target_compile_options(serial_port_probe PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
target_link_libraries(serial_port_probe PRIVATE serial_package)

install(TARGETS serial_package serial_port_probe
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
