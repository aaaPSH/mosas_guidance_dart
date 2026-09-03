cmake_minimum_required(VERSION 3.16)
project(mosas_guidance LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)

option(MOSAS_BUILD_WIRELESS "构建 RTSP 无线图传模块" ON)

set(MOSAS_OPENCV_COMPONENTS core imgproc videoio calib3d)
if(MOSAS_BUILD_WIRELESS)
    list(APPEND MOSAS_OPENCV_COMPONENTS imgcodecs)
endif()
find_package(OpenCV REQUIRED COMPONENTS ${MOSAS_OPENCV_COMPONENTS})
find_package(Threads REQUIRED)

if(MOSAS_BUILD_WIRELESS)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
        libavcodec
        libavformat
        libavutil
        libswscale
    )
endif()

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
target_include_directories(mosas_vision
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mosas_vision PUBLIC ${OpenCV_LIBS})

add_library(mosas_guidance STATIC
    src/guidance/guidance_estimator.cpp
)
configure_mosas_target(mosas_guidance)
target_link_libraries(mosas_guidance
    PUBLIC
        mosas_dart
        mosas_vision
)

add_library(mosas_runtime STATIC
    src/runtime/flight_phase_detector.cpp
    src/runtime/imu_state_history.cpp
    src/runtime/guidance_runtime.cpp
)
configure_mosas_target(mosas_runtime)
target_include_directories(mosas_runtime
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mosas_runtime
    PUBLIC
        mosas_guidance
        Threads::Threads
)

if(MOSAS_BUILD_WIRELESS)
    add_library(mosas_wireless STATIC
        src/wireless/rtsp_frame_sink.cpp
        src/wireless/rtsp_streamer.cpp
    )
    configure_mosas_target(mosas_wireless)
    target_include_directories(mosas_wireless
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(mosas_wireless
        PUBLIC
            mosas_vision
            ${OpenCV_LIBS}
            PkgConfig::FFMPEG
    )
endif()

add_library(mosas_app STATIC
    src/app/app_config.cpp
    src/app/host_adapters.cpp
)
configure_mosas_target(mosas_app)
target_include_directories(mosas_app
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mosas_app
    PUBLIC
        mosas_runtime
        ${OpenCV_LIBS}
)
if(MOSAS_BUILD_WIRELESS)
    target_link_libraries(mosas_app PUBLIC mosas_wireless)
endif()

add_executable(mosas_guidance_app
    examples/mosas_guidance_app.cpp
)
configure_mosas_target(mosas_guidance_app)
target_link_libraries(mosas_guidance_app PRIVATE mosas_app)
if(MOSAS_BUILD_WIRELESS)
    target_compile_definitions(mosas_guidance_app PRIVATE MOSAS_APP_HAS_WIRELESS=1)
endif()

if(BUILD_TESTING)
    add_executable(vision_opencv_test
        tests/vision_opencv_test.cpp
    )
    configure_mosas_target(vision_opencv_test)
    target_link_libraries(vision_opencv_test PRIVATE mosas_vision)
    add_test(NAME vision_opencv_test COMMAND vision_opencv_test)

    add_executable(png_guidance_test
        tests/png_guidance_test.cpp
    )
    configure_mosas_target(png_guidance_test)
    target_link_libraries(png_guidance_test PRIVATE mosas_guidance)
    add_test(NAME png_guidance_test COMMAND png_guidance_test)

    add_executable(guidance_runtime_test
        tests/guidance_runtime_test.cpp
    )
    configure_mosas_target(guidance_runtime_test)
    target_link_libraries(guidance_runtime_test PRIVATE mosas_runtime)
    add_test(NAME guidance_runtime_test COMMAND guidance_runtime_test)

    add_executable(app_config_test
        tests/app_config_test.cpp
    )
    configure_mosas_target(app_config_test)
    target_include_directories(app_config_test
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(app_config_test PRIVATE mosas_app)
    add_test(NAME app_config_test COMMAND app_config_test)

    add_executable(host_adapters_test
        tests/host_adapters_test.cpp
    )
    configure_mosas_target(host_adapters_test)
    target_link_libraries(host_adapters_test PRIVATE mosas_app)
    add_test(NAME host_adapters_test COMMAND host_adapters_test)

    add_test(
        NAME mosas_guidance_app_cli_test
        COMMAND bash
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/mosas_guidance_app_cli_test.sh
            $<TARGET_FILE:mosas_guidance_app>
    )

    add_test(
        NAME run_guidance_script_test
        COMMAND bash
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_guidance_script_test.sh
            ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_guidance.sh
            ${CMAKE_CURRENT_SOURCE_DIR}
    )

    if(MOSAS_BUILD_WIRELESS)
        add_executable(wireless_streamer_test
            tests/wireless_streamer_test.cpp
        )
        configure_mosas_target(wireless_streamer_test)
        target_link_libraries(wireless_streamer_test PRIVATE mosas_wireless)
        add_test(NAME wireless_streamer_test COMMAND wireless_streamer_test)

        add_test(
            NAME wireless_test_frame_generator_fallback_test
            COMMAND bash
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/wireless_test_frame_generator_fallback_test.sh
                $<TARGET_FILE:wireless_test_frame_generator>
        )
    endif()
endif()

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

add_executable(guidance_timing_probe
    examples/guidance_timing_probe.cpp
)
configure_mosas_target(guidance_timing_probe)
target_link_libraries(guidance_timing_probe PRIVATE mosas_guidance)

if(MOSAS_BUILD_WIRELESS)
    add_executable(wireless_stream_probe
        examples/wireless_stream_probe.cpp
    )
    configure_mosas_target(wireless_stream_probe)
    target_link_libraries(wireless_stream_probe PRIVATE mosas_wireless)

    add_executable(wireless_test_frame_generator
        examples/wireless_test_frame_generator.cpp
    )
    configure_mosas_target(wireless_test_frame_generator)
    target_include_directories(wireless_test_frame_generator
        PRIVATE
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(wireless_test_frame_generator PRIVATE ${OpenCV_LIBS})
endif()

set(MOSAS_INSTALL_TARGETS
    mosas_dart
    mosas_vision
    mosas_guidance
    mosas_runtime
    mosas_app
    mosas_guidance_app
    serial_package
    serial_port_probe
    guidance_timing_probe
)
if(MOSAS_BUILD_WIRELESS)
    list(APPEND MOSAS_INSTALL_TARGETS
        mosas_wireless
        wireless_stream_probe
        wireless_test_frame_generator
    )
endif()

install(TARGETS ${MOSAS_INSTALL_TARGETS}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES config/mosas_guidance.conf
    DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/mosas
)
install(PROGRAMS scripts/run_guidance.sh
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
