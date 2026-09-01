cmake_minimum_required(VERSION 3.16)
project(mosas_guidance LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_EXTENSIONS OFF)

function(configure_mosas_target target)
    target_compile_features(${target} PUBLIC cxx_std_17)
    target_compile_options(${target} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
    )
    target_include_directories(${target}
        PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    )
endfunction()

add_library(mosas_dart STATIC
    src/dart/dart_condition.cpp
)
configure_mosas_target(mosas_dart)

add_library(mosas_vision STATIC
    src/vision/vision_recognizer.cpp
    src/vision/vision_visualizer.cpp
)
configure_mosas_target(mosas_vision)

add_library(mosas_guidance STATIC
    src/guidance/guidance_estimator.cpp
)
configure_mosas_target(mosas_guidance)
target_link_libraries(mosas_guidance
    PUBLIC
        mosas_dart
        mosas_vision
)

find_package(Threads REQUIRED)
add_library(serial_package STATIC
    src/serial_package/serial_port.cpp
)
configure_mosas_target(serial_package)
target_link_libraries(serial_package PUBLIC Threads::Threads)

add_executable(serial_port_probe
    examples/serial_port_probe.cpp
)
configure_mosas_target(serial_port_probe)
target_link_libraries(serial_port_probe PRIVATE serial_package)

install(TARGETS
    mosas_dart
    mosas_vision
    mosas_guidance
    serial_package
    serial_port_probe
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
