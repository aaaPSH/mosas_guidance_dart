cmake_minimum_required(VERSION 3.16)
project(mosas_guidance LANGUAGES CXX)

include(GNUInstallDirs)

option(MOSAS_BUILD_WIRELESS "构建 RTSP 无线图传模块" ON)
option(MOSAS_BUILD_APPLICATION "构建 Nori/视觉设备应用" ON)
option(MOSAS_BUILD_OPENCV_ADAPTERS "构建 OpenCV 视觉和输出适配器（无需相机 SDK）" OFF)
option(MOSAS_BUILD_SIMULATION_APPLICATION "构建 Unix Socket 仿真适配器和应用（无需 Nori/ROS）" OFF)
include(CTest)

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

add_library(mosas_contract_core STATIC
    src/domain/imu_time_model.cpp
    src/runtime/clock.cpp
    src/runtime/frame_timing.cpp
    src/serial_package/serial_protocol.cpp
)
configure_mosas_target(mosas_contract_core)

find_package(Threads REQUIRED)

if(MOSAS_BUILD_APPLICATION OR MOSAS_BUILD_OPENCV_ADAPTERS)
    set(MOSAS_OPENCV_COMPONENTS core imgproc videoio)
    if(MOSAS_BUILD_APPLICATION)
        list(APPEND MOSAS_OPENCV_COMPONENTS imgcodecs)
    endif()
    find_package(OpenCV REQUIRED COMPONENTS ${MOSAS_OPENCV_COMPONENTS})

    add_library(mosas_vision STATIC
        src/vision/vision_recognizer.cpp
        src/vision/vision_visualizer.cpp
        src/vision/opencv_image_frame.cpp
        src/vision/opencv_vision_detector.cpp
    )
    configure_mosas_target(mosas_vision)
    target_include_directories(mosas_vision PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(mosas_vision PUBLIC ${OpenCV_LIBS})
endif()

add_library(mosas_guidance STATIC
    src/guidance/guidance_estimator.cpp
    src/guidance/default_algorithm_adapters.cpp
    src/guidance/body_vector_pn_guidance.cpp
    src/guidance/body_vector_los_filter.cpp
)
configure_mosas_target(mosas_guidance)

add_library(mosas_runtime STATIC
    src/runtime/attitude_estimator.cpp
    src/runtime/legacy_flight_state_adapter.cpp
    src/runtime/attitude_history.cpp
    src/runtime/imu_angular_velocity_history.cpp
    src/runtime/async_info_logger.cpp
    src/runtime/frame_pipeline.cpp
    src/runtime/runtime_supervisor.cpp
    src/runtime/imu_pipeline.cpp
    src/runtime/output_dispatcher.cpp
)
configure_mosas_target(mosas_runtime)
target_link_libraries(mosas_runtime
    PUBLIC
        mosas_contract_core
        Threads::Threads
)

# 输出适配器及其测试不依赖相机 SDK。
if(MOSAS_BUILD_APPLICATION OR MOSAS_BUILD_OPENCV_ADAPTERS)
    if(MOSAS_BUILD_WIRELESS)
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
            libavcodec
            libavformat
            libavutil
            libswscale
        )
        add_library(mosas_wireless STATIC src/wireless/rtsp_streamer.cpp)
        configure_mosas_target(mosas_wireless)
        target_include_directories(mosas_wireless PUBLIC ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(mosas_wireless
            PUBLIC ${OpenCV_LIBS} PkgConfig::FFMPEG
        )
    endif()

    add_library(mosas_output STATIC
        src/output/imu_csv_recorder.cpp
        src/output/local_recording_session.cpp
        src/output/local_video_recorder.cpp
        src/output/telemetry_csv_recorder.cpp
        src/output/output_mapping.cpp
        src/output/recording_paths.cpp
    )
    configure_mosas_target(mosas_output)
    target_include_directories(mosas_output PUBLIC ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(mosas_output
        PUBLIC mosas_runtime mosas_vision ${OpenCV_LIBS}
    )
    if(MOSAS_BUILD_WIRELESS)
        target_sources(mosas_output PRIVATE src/output/rtsp_visualization_sink.cpp)
        target_compile_definitions(mosas_output PUBLIC MOSAS_OUTPUT_HAS_WIRELESS=1)
        target_link_libraries(mosas_output PUBLIC mosas_wireless)
    endif()
endif()

# 配置解析和算法组装属于宿主逻辑，不执行相机或串口 I/O。
if(MOSAS_BUILD_APPLICATION OR MOSAS_BUILD_OPENCV_ADAPTERS)
    add_library(mosas_app_core STATIC
        src/app/app_config.cpp
        src/app/camera_imu_sync_session.cpp
        src/app/camera_imu_sync_sources.cpp
        src/app/config_parsing.cpp
        src/app/simulation_config.cpp
        src/app/default_guidance_algorithms.cpp
        src/app/output_factory.cpp
    )
    configure_mosas_target(mosas_app_core)
    target_link_libraries(mosas_app_core PUBLIC mosas_runtime mosas_guidance mosas_vision mosas_output)
endif()

if(MOSAS_BUILD_SIMULATION_APPLICATION)
    if(NOT TARGET mosas_vision)
        message(FATAL_ERROR "Simulation requires MOSAS_BUILD_OPENCV_ADAPTERS=ON")
    endif()
    find_package(dart_sim_transport REQUIRED CONFIG)
    add_library(mosas_simulation STATIC
        src/adapters/simulation_session.cpp
        src/adapters/simulation_sources.cpp
        src/adapters/simulation_command_sink.cpp
        src/adapters/simulation_pre_acquisition.cpp
        src/adapters/simulation_audit.cpp
        src/app/simulation_runner.cpp
        src/app/simulation_contract.cpp)
    configure_mosas_target(mosas_simulation)
    target_link_libraries(mosas_simulation PUBLIC mosas_app_core dart_sim_transport::dart_sim_transport)
    add_executable(mosas_simulation_app examples/mosas_simulation_app.cpp)
    configure_mosas_target(mosas_simulation_app)
    target_link_libraries(mosas_simulation_app PRIVATE mosas_simulation)
    install(TARGETS mosas_simulation_app RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES config/mosas_simulation.conf DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/mosas)
endif()

if(MOSAS_BUILD_APPLICATION)
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

add_library(serial_package STATIC
    src/serial_package/serial_port.cpp
)
configure_mosas_target(serial_package)
target_link_libraries(serial_package PUBLIC mosas_contract_core Threads::Threads)

add_library(mosas_app STATIC
    src/adapters/command_adapter.cpp
    src/adapters/host_adapters.cpp
    src/adapters/imu_stream_decoder.cpp
    src/adapters/nori_camera_sdk.cpp
    src/adapters/nori_camera_source.cpp
    src/adapters/raw_imu_receive_buffer.cpp
    src/adapters/serial_imu_source.cpp
)
configure_mosas_target(mosas_app)
target_include_directories(mosas_app
    PUBLIC
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(mosas_app
    PUBLIC
        mosas_app_core
        mosas_runtime
        mosas_guidance
        mosas_output
        serial_package
        mosas_camera_sdk
        atomic
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

set(MOSAS_INSTALL_TARGETS
    mosas_contract_core
    serial_package
    mosas_vision
    mosas_guidance
    mosas_runtime
    mosas_output
    mosas_app_core
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
install(PROGRAMS scripts/run_guidance.sh scripts/calibrate_camera_imu_sync.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES "${MOSAS_CAMERA_SDK_LIBRARY}"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
endif()  # MOSAS_BUILD_APPLICATION

if(BUILD_TESTING)
    # 仅部署 C++ 程序的主机无需安装离线 Python 分析依赖。
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        execute_process(COMMAND ${Python3_EXECUTABLE} -c "import numpy; import cv2"
            RESULT_VARIABLE MOSAS_SYNC_PYTHON_AVAILABLE
            OUTPUT_QUIET ERROR_QUIET)
        if(MOSAS_SYNC_PYTHON_AVAILABLE EQUAL 0)
            add_test(NAME camera_imu_sync_calibration_test
                COMMAND ${Python3_EXECUTABLE} -B
                    ${PROJECT_SOURCE_DIR}/tests/camera_imu_sync_calibration_test.py)
            set_tests_properties(camera_imu_sync_calibration_test PROPERTIES TIMEOUT 60)
        endif()
    endif()
    if(TARGET mosas_simulation)
        foreach(test simulation_sources_test simulation_command_sink_test simulation_sync_test simulation_runner_test simulation_contract_test simulation_recording_test simulation_pre_acquisition_test simulation_audit_test)
            add_executable(${test} tests/${test}.cpp)
            configure_mosas_target(${test})
            target_compile_definitions(${test} PRIVATE MOSAS_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
            target_link_libraries(${test} PRIVATE mosas_simulation)
            add_test(NAME ${test} COMMAND ${test})
            set_tests_properties(${test} PROPERTIES TIMEOUT 15)
        endforeach()
    endif()
    add_executable(host_build_smoke_test
        tests/host_build_smoke_test.cpp
    )
    configure_mosas_target(host_build_smoke_test)
    target_link_libraries(host_build_smoke_test PRIVATE mosas_contract_core)
    add_test(NAME host_build_smoke_test COMMAND host_build_smoke_test)

    add_executable(domain_contracts_test tests/domain_contracts_test.cpp)
    configure_mosas_target(domain_contracts_test)
    target_link_libraries(domain_contracts_test PRIVATE mosas_contract_core)
    add_test(NAME domain_contracts_test COMMAND domain_contracts_test)

    add_executable(image_frame_test tests/image_frame_test.cpp)
    configure_mosas_target(image_frame_test)
    target_link_libraries(image_frame_test PRIVATE mosas_contract_core)
    add_test(NAME image_frame_test COMMAND image_frame_test)

    add_executable(algorithm_port_contracts_test
        tests/algorithm_port_contracts_test.cpp
    )
    configure_mosas_target(algorithm_port_contracts_test)
    target_link_libraries(algorithm_port_contracts_test PRIVATE mosas_contract_core)
    add_test(NAME algorithm_port_contracts_test
        COMMAND algorithm_port_contracts_test)

    add_executable(camera_time_test tests/camera_time_test.cpp)
    configure_mosas_target(camera_time_test)
    target_link_libraries(camera_time_test PRIVATE mosas_contract_core)
    add_test(NAME camera_time_test COMMAND camera_time_test)

    add_executable(serial_protocol_test tests/serial_protocol_test.cpp)
    configure_mosas_target(serial_protocol_test)
    target_link_libraries(serial_protocol_test PRIVATE mosas_contract_core)
    add_test(NAME serial_protocol_test COMMAND serial_protocol_test)

    add_executable(imu_time_model_test
        tests/imu_time_model_test.cpp
        src/adapters/imu_stream_decoder.cpp
    )
    configure_mosas_target(imu_time_model_test)
    target_link_libraries(imu_time_model_test PRIVATE mosas_contract_core)
    add_test(NAME imu_time_model_test COMMAND imu_time_model_test)

    add_executable(raw_imu_receive_buffer_test
        tests/raw_imu_receive_buffer_test.cpp
        src/adapters/raw_imu_receive_buffer.cpp
    )
    configure_mosas_target(raw_imu_receive_buffer_test)
    target_link_libraries(raw_imu_receive_buffer_test PRIVATE Threads::Threads)
    add_test(NAME raw_imu_receive_buffer_test COMMAND raw_imu_receive_buffer_test)
    set_tests_properties(raw_imu_receive_buffer_test PROPERTIES TIMEOUT 10)

    add_executable(body_vector_los_filter_test tests/body_vector_los_filter_test.cpp)
    configure_mosas_target(body_vector_los_filter_test)
    target_link_libraries(body_vector_los_filter_test PRIVATE mosas_guidance)
    add_test(NAME body_vector_los_filter_test COMMAND body_vector_los_filter_test)

    add_executable(body_vector_pn_guidance_test tests/body_vector_pn_guidance_test.cpp)
    configure_mosas_target(body_vector_pn_guidance_test)
    target_link_libraries(body_vector_pn_guidance_test PRIVATE mosas_guidance)
    add_test(NAME body_vector_pn_guidance_test COMMAND body_vector_pn_guidance_test)

    add_executable(png_calculation_trace_test tests/png_calculation_trace_test.cpp)
    configure_mosas_target(png_calculation_trace_test)
    target_link_libraries(png_calculation_trace_test PRIVATE mosas_guidance)
    add_test(NAME png_calculation_trace_test COMMAND png_calculation_trace_test)

    add_executable(guidance_adapter_test tests/guidance_adapter_test.cpp)
    configure_mosas_target(guidance_adapter_test)
    target_link_libraries(guidance_adapter_test PRIVATE mosas_guidance)
    add_test(NAME guidance_adapter_test COMMAND guidance_adapter_test)

    if(MOSAS_BUILD_APPLICATION OR MOSAS_BUILD_OPENCV_ADAPTERS)
        add_executable(vision_adapter_test tests/vision_adapter_test.cpp)
        configure_mosas_target(vision_adapter_test)
        target_link_libraries(vision_adapter_test PRIVATE mosas_vision)
        add_test(NAME vision_adapter_test COMMAND vision_adapter_test)

        add_executable(image_adapter_test tests/image_adapter_test.cpp)
        configure_mosas_target(image_adapter_test)
        target_link_libraries(image_adapter_test PRIVATE mosas_vision)
        add_test(NAME image_adapter_test COMMAND image_adapter_test)
    endif()

    add_executable(attitude_adapter_test tests/attitude_adapter_test.cpp)
    configure_mosas_target(attitude_adapter_test)
    target_link_libraries(attitude_adapter_test PRIVATE mosas_runtime)
    add_test(NAME attitude_adapter_test COMMAND attitude_adapter_test)

    add_executable(launch_detection_test tests/launch_detection_test.cpp)
    configure_mosas_target(launch_detection_test)
    target_link_libraries(launch_detection_test PRIVATE mosas_runtime)
    add_test(NAME launch_detection_test COMMAND launch_detection_test)

    add_executable(body_vector_runtime_test
        tests/body_vector_runtime_test.cpp
        tests/fakes/fake_camera_source.cpp
        tests/fakes/fake_imu_source.cpp
    )
    configure_mosas_target(body_vector_runtime_test)
    target_link_libraries(body_vector_runtime_test PRIVATE mosas_runtime mosas_guidance)
    add_test(NAME body_vector_runtime_test COMMAND body_vector_runtime_test)

    add_executable(runtime_supervisor_test
        tests/runtime_supervisor_test.cpp
        tests/fakes/fake_camera_source.cpp
        tests/fakes/fake_imu_source.cpp
    )
    configure_mosas_target(runtime_supervisor_test)
    target_link_libraries(runtime_supervisor_test PRIVATE mosas_runtime)
    add_test(NAME runtime_supervisor_test COMMAND runtime_supervisor_test)

    add_executable(runtime_supervisor_lifecycle_test
        tests/runtime_supervisor_lifecycle_test.cpp
    )
    configure_mosas_target(runtime_supervisor_lifecycle_test)
    target_link_libraries(runtime_supervisor_lifecycle_test PRIVATE mosas_runtime)
    add_test(NAME runtime_supervisor_lifecycle_test
        COMMAND runtime_supervisor_lifecycle_test)

    add_executable(runtime_output_isolation_test
        tests/runtime_output_isolation_test.cpp
        tests/fakes/fake_imu_source.cpp
    )
    configure_mosas_target(runtime_output_isolation_test)
    target_link_libraries(runtime_output_isolation_test PRIVATE mosas_runtime)
    add_test(NAME runtime_output_isolation_test
        COMMAND runtime_output_isolation_test)

    add_executable(output_contracts_test tests/output_contracts_test.cpp)
    configure_mosas_target(output_contracts_test)
    target_link_libraries(output_contracts_test PRIVATE mosas_contract_core)
    add_test(NAME output_contracts_test COMMAND output_contracts_test)

    add_executable(multi_output_dispatcher_test
        tests/multi_output_dispatcher_test.cpp
    )
    configure_mosas_target(multi_output_dispatcher_test)
    target_include_directories(multi_output_dispatcher_test PRIVATE
        ${PROJECT_SOURCE_DIR}/src/runtime
    )
    target_link_libraries(multi_output_dispatcher_test PRIVATE mosas_runtime)
    add_test(NAME multi_output_dispatcher_test
        COMMAND multi_output_dispatcher_test)

    add_executable(output_dispatcher_test tests/output_dispatcher_test.cpp)
    configure_mosas_target(output_dispatcher_test)
    target_include_directories(output_dispatcher_test PRIVATE
        ${PROJECT_SOURCE_DIR}/src/runtime
    )
    target_link_libraries(output_dispatcher_test PRIVATE mosas_runtime)
    add_test(NAME output_dispatcher_test COMMAND output_dispatcher_test)

    add_executable(imu_pipeline_test
        tests/imu_pipeline_test.cpp
        tests/fakes/fake_imu_source.cpp
    )
    configure_mosas_target(imu_pipeline_test)
    target_include_directories(imu_pipeline_test PRIVATE
        ${PROJECT_SOURCE_DIR}/src/runtime
    )
    target_link_libraries(imu_pipeline_test PRIVATE mosas_runtime)
    add_test(NAME imu_pipeline_test COMMAND imu_pipeline_test)

    add_executable(simulation_time_test tests/simulation_time_test.cpp
        tests/fakes/fake_imu_source.cpp tests/fakes/fake_camera_source.cpp)
    configure_mosas_target(simulation_time_test)
    target_include_directories(simulation_time_test PRIVATE ${PROJECT_SOURCE_DIR}/src/runtime)
    target_link_libraries(simulation_time_test PRIVATE mosas_runtime)
    add_test(NAME simulation_time_test COMMAND simulation_time_test)

    if(MOSAS_BUILD_APPLICATION OR MOSAS_BUILD_OPENCV_ADAPTERS)
        add_executable(output_mapping_test tests/output_mapping_test.cpp)
        configure_mosas_target(output_mapping_test)
        target_link_libraries(output_mapping_test PRIVATE mosas_output)
        add_test(NAME output_mapping_test COMMAND output_mapping_test)

        add_executable(local_recorders_test tests/local_recorders_test.cpp)
        configure_mosas_target(local_recorders_test)
        target_link_libraries(local_recorders_test PRIVATE mosas_output)
        add_test(NAME local_recorders_test COMMAND local_recorders_test)

        add_executable(output_sink_diagnostics_test
            tests/output_sink_diagnostics_test.cpp
        )
        configure_mosas_target(output_sink_diagnostics_test)
        target_link_libraries(output_sink_diagnostics_test PRIVATE mosas_output)
        add_test(NAME output_sink_diagnostics_test
            COMMAND output_sink_diagnostics_test)

        add_executable(camera_imu_sync_capture_test tests/camera_imu_sync_capture_test.cpp)
        configure_mosas_target(camera_imu_sync_capture_test)
        target_include_directories(camera_imu_sync_capture_test PRIVATE ${PROJECT_SOURCE_DIR}/src/app)
        target_link_libraries(camera_imu_sync_capture_test PRIVATE mosas_app_core)
        add_test(NAME camera_imu_sync_capture_test COMMAND camera_imu_sync_capture_test)
        set_tests_properties(camera_imu_sync_capture_test PROPERTIES TIMEOUT 15)

        add_executable(camera_imu_sync_composition_test tests/camera_imu_sync_composition_test.cpp)
        configure_mosas_target(camera_imu_sync_composition_test)
        target_link_libraries(camera_imu_sync_composition_test PRIVATE mosas_app_core)
        add_test(NAME camera_imu_sync_composition_test COMMAND camera_imu_sync_composition_test)
        set_tests_properties(camera_imu_sync_composition_test PROPERTIES TIMEOUT 15)

        add_executable(mode_composition_test tests/mode_composition_test.cpp)
        configure_mosas_target(mode_composition_test)
        target_link_libraries(mode_composition_test PRIVATE mosas_app_core)
        add_test(NAME mode_composition_test COMMAND mode_composition_test)
    endif()

    if(MOSAS_BUILD_APPLICATION)
        add_executable(command_adapter_test tests/command_adapter_test.cpp)
        configure_mosas_target(command_adapter_test)
        target_include_directories(command_adapter_test PRIVATE
            ${PROJECT_SOURCE_DIR}/src/adapters
        )
        target_link_libraries(command_adapter_test PRIVATE mosas_app)
        add_test(NAME command_adapter_test COMMAND command_adapter_test)

        add_executable(serial_command_sink_test tests/serial_command_sink_test.cpp)
        configure_mosas_target(serial_command_sink_test)
        target_link_libraries(serial_command_sink_test PRIVATE mosas_app)
        add_test(NAME serial_command_sink_test COMMAND serial_command_sink_test)
    endif()

    if(TARGET mosas_app_core)
        add_executable(simulation_config_test tests/simulation_config_test.cpp)
        configure_mosas_target(simulation_config_test)
        target_compile_definitions(simulation_config_test PRIVATE MOSAS_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
        target_link_libraries(simulation_config_test PRIVATE mosas_app_core)
        add_test(NAME simulation_config_test COMMAND simulation_config_test)

        add_executable(default_guidance_algorithms_test
            tests/default_guidance_algorithms_test.cpp
        )
        configure_mosas_target(default_guidance_algorithms_test)
        target_compile_definitions(default_guidance_algorithms_test PRIVATE
            MOSAS_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
        )
        target_link_libraries(default_guidance_algorithms_test PRIVATE mosas_app_core)
        add_test(NAME default_guidance_algorithms_test
            COMMAND default_guidance_algorithms_test)
    endif()
endif()
