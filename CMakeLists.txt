cmake_minimum_required(VERSION 3.16)
project(mosas_guidance LANGUAGES CXX)

include(GNUInstallDirs)
include(CTest)

option(MOSAS_BUILD_WIRELESS "构建 RTSP 无线图传模块" ON)

# 根据目标处理器选择 SDK 的库目录，并在配置阶段拒绝不完整的 SDK。
set(MOSAS_CAMERA_SDK_ROOT "${PROJECT_SOURCE_DIR}/camera_SDk" CACHE PATH
    "Nori 相机 SDK 根目录")
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" MOSAS_CAMERA_SDK_PROCESSOR)
if(MOSAS_CAMERA_SDK_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(MOSAS_CAMERA_SDK_ARCH arm64)
elseif(MOSAS_CAMERA_SDK_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(MOSAS_CAMERA_SDK_ARCH x64)
else()
    message(FATAL_ERROR
        "未知的 Nori SDK 目标架构: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(MOSAS_CAMERA_SDK_INCLUDE_DIR "${MOSAS_CAMERA_SDK_ROOT}/Includes")
set(MOSAS_CAMERA_SDK_API_INCLUDE_DIR
    "${MOSAS_CAMERA_SDK_INCLUDE_DIR}/Nori_Xvision_API")
set(MOSAS_CAMERA_SDK_REQUIRED_HEADERS
    Nori_Error_Define.h
    Nori_Public.h
    Nori_Xvision_API.h
)
set(MOSAS_CAMERA_SDK_LIBRARY_DIR
    "${MOSAS_CAMERA_SDK_ROOT}/Libraries/${MOSAS_CAMERA_SDK_ARCH}")
set(MOSAS_CAMERA_SDK_LIBRARY
    "${MOSAS_CAMERA_SDK_LIBRARY_DIR}/libNori_Xvision_Std.so")
if(NOT IS_DIRECTORY "${MOSAS_CAMERA_SDK_ROOT}")
    message(FATAL_ERROR
        "Nori SDK 根目录不存在: ${MOSAS_CAMERA_SDK_ROOT}")
endif()
foreach(MOSAS_CAMERA_SDK_HEADER IN LISTS MOSAS_CAMERA_SDK_REQUIRED_HEADERS)
    if(NOT EXISTS "${MOSAS_CAMERA_SDK_API_INCLUDE_DIR}/${MOSAS_CAMERA_SDK_HEADER}")
        message(FATAL_ERROR
            "Nori SDK 头文件不存在: ${MOSAS_CAMERA_SDK_API_INCLUDE_DIR}/${MOSAS_CAMERA_SDK_HEADER}")
    endif()
endforeach()
if(NOT EXISTS "${MOSAS_CAMERA_SDK_LIBRARY}")
    message(FATAL_ERROR "Nori SDK 库不存在: ${MOSAS_CAMERA_SDK_LIBRARY}")
endif()

# 让所有使用 SDK 的构建目标在构建树和安装树中找到正确的库。
set(CMAKE_BUILD_RPATH "${MOSAS_CAMERA_SDK_LIBRARY_DIR}")
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")

add_library(mosas_camera_sdk SHARED IMPORTED GLOBAL)
set_target_properties(mosas_camera_sdk PROPERTIES
    IMPORTED_LOCATION "${MOSAS_CAMERA_SDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MOSAS_CAMERA_SDK_INCLUDE_DIR}"
)

set(MOSAS_OPENCV_COMPONENTS core imgproc videoio)
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

add_library(serial_package STATIC
    src/serial_package/serial_port.cpp
    src/serial_package/serial_protocol.cpp
)
configure_mosas_target(serial_package)
target_link_libraries(serial_package PUBLIC Threads::Threads)

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
        mosas_vision
)

add_library(mosas_runtime STATIC
    src/runtime/attitude_estimator.cpp
    src/runtime/attitude_history.cpp
    src/runtime/clock.cpp
    src/runtime/frame_timing.cpp
    src/runtime/imu_angular_velocity_history.cpp
    src/runtime/async_info_logger.cpp
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
        src/wireless/rtsp_streamer.cpp
    )
    configure_mosas_target(mosas_wireless)
    target_include_directories(mosas_wireless
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(mosas_wireless
        PUBLIC
            ${OpenCV_LIBS}
            PkgConfig::FFMPEG
    )
endif()

add_library(mosas_output STATIC
    src/output/local_recorder.cpp
    src/output/output_controller.cpp
)
configure_mosas_target(mosas_output)
target_include_directories(mosas_output
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mosas_output
    PUBLIC
        mosas_runtime
        mosas_vision
        ${OpenCV_LIBS}
)
if(MOSAS_BUILD_WIRELESS)
    target_compile_definitions(mosas_output
        PUBLIC MOSAS_OUTPUT_HAS_WIRELESS=1
    )
    target_link_libraries(mosas_output PUBLIC mosas_wireless)
endif()

add_library(mosas_app STATIC
    src/app/app_config.cpp
    src/app/host_adapters.cpp
    src/app/imu_stream_decoder.cpp
    src/app/nori_camera_sdk.cpp
    src/app/nori_camera_source.cpp
    src/app/raw_imu_receive_buffer.cpp
    src/app/serial_imu_source.cpp
)
configure_mosas_target(mosas_app)
target_include_directories(mosas_app
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mosas_app
    PUBLIC
        mosas_runtime
        mosas_output
        serial_package
        mosas_camera_sdk
        atomic
        ${OpenCV_LIBS}
)
if(MOSAS_BUILD_WIRELESS)
    target_link_libraries(mosas_app PUBLIC mosas_wireless)
endif()

if(BUILD_TESTING)
    function(mosas_add_test test_name)
        add_test(NAME ${test_name} COMMAND ${test_name})
        set_tests_properties(${test_name} PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        )
    endfunction()

    # 姿态、时钟、时序与运行时测试
    foreach(MOSAS_RUNTIME_TEST IN ITEMS
            attitude_history_test
            attitude_launch_window_test
            attitude_estimator_test
            attitude_state_snapshot_test
            clock_test
            frame_timing_test
            imu_angular_velocity_history_test
            spsc_ring_test
            runtime_logging_test)
        add_executable(${MOSAS_RUNTIME_TEST}
            tests/${MOSAS_RUNTIME_TEST}.cpp
        )
        configure_mosas_target(${MOSAS_RUNTIME_TEST})
        target_link_libraries(${MOSAS_RUNTIME_TEST} PRIVATE mosas_runtime)
        mosas_add_test(${MOSAS_RUNTIME_TEST})
    endforeach()

    # 导引与视觉测试
    foreach(MOSAS_GUIDANCE_TEST IN ITEMS
            guidance_body_frame_test
            png_guidance_formula_test)
        add_executable(${MOSAS_GUIDANCE_TEST}
            tests/${MOSAS_GUIDANCE_TEST}.cpp
        )
        configure_mosas_target(${MOSAS_GUIDANCE_TEST})
        target_link_libraries(${MOSAS_GUIDANCE_TEST} PRIVATE mosas_guidance)
        mosas_add_test(${MOSAS_GUIDANCE_TEST})
    endforeach()

    add_executable(vision_recognizer_debug_test
        tests/vision_recognizer_debug_test.cpp
    )
    configure_mosas_target(vision_recognizer_debug_test)
    target_link_libraries(vision_recognizer_debug_test PRIVATE mosas_vision ${OpenCV_LIBS})
    mosas_add_test(vision_recognizer_debug_test)

    # 输出模式与本地录像测试
    add_executable(output_config_test
        tests/output_config_test.cpp
    )
    configure_mosas_target(output_config_test)
    target_link_libraries(output_config_test PRIVATE mosas_app)
    mosas_add_test(output_config_test)

    add_executable(local_recorder_test
        tests/local_recorder_test.cpp
    )
    configure_mosas_target(local_recorder_test)
    target_link_libraries(local_recorder_test PRIVATE mosas_output mosas_vision ${OpenCV_LIBS})
    mosas_add_test(local_recorder_test)

    add_executable(output_controller_test
        tests/output_controller_test.cpp
    )
    configure_mosas_target(output_controller_test)
    target_link_libraries(output_controller_test PRIVATE mosas_output)
    mosas_add_test(output_controller_test)

    # 串口、配置与相机数据源测试
    add_executable(serial_port_receive_test
        tests/serial_port_receive_test.cpp
    )
    configure_mosas_target(serial_port_receive_test)
    target_link_libraries(serial_port_receive_test PRIVATE serial_package)
    mosas_add_test(serial_port_receive_test)

    foreach(MOSAS_APP_TEST IN ITEMS
            attitude_config_test
            raw_serial_callback_test
            imu_stream_decoder_test
            serial_imu_source_async_test
            async_output_test)
        add_executable(${MOSAS_APP_TEST}
            tests/${MOSAS_APP_TEST}.cpp
        )
        configure_mosas_target(${MOSAS_APP_TEST})
        target_link_libraries(${MOSAS_APP_TEST} PRIVATE mosas_app)
        mosas_add_test(${MOSAS_APP_TEST})
    endforeach()

    add_executable(nori_camera_source_test
        tests/nori_camera_source_test.cpp
    )
    configure_mosas_target(nori_camera_source_test)
    target_link_libraries(nori_camera_source_test PRIVATE mosas_app mosas_camera_sdk)
    mosas_add_test(nori_camera_source_test)
endif()

add_executable(mosas_guidance_app
    examples/mosas_guidance_app.cpp
)
configure_mosas_target(mosas_guidance_app)
target_link_libraries(mosas_guidance_app PRIVATE mosas_app)
if(MOSAS_BUILD_WIRELESS)
    target_compile_definitions(mosas_guidance_app PRIVATE MOSAS_APP_HAS_WIRELESS=1)
endif()

set(MOSAS_INSTALL_TARGETS
    serial_package
    mosas_vision
    mosas_guidance
    mosas_runtime
    mosas_output
    mosas_app
    mosas_guidance_app
)
if(MOSAS_BUILD_WIRELESS)
    list(APPEND MOSAS_INSTALL_TARGETS
        mosas_wireless
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
install(FILES "${MOSAS_CAMERA_SDK_LIBRARY}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
