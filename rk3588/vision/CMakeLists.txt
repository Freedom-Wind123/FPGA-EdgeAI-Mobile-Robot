cmake_minimum_required(VERSION 3.16)
project(rk3588_vision CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

find_path(RKNN_INCLUDE_DIR rknn_api.h
    PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
        /usr/include
        /usr/local/include
        /usr/include/rknn
        /opt/rknn/include
)

find_library(RKNN_LIB rknnrt
    PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
        /usr/lib
        /usr/lib/aarch64-linux-gnu
        /usr/local/lib
)

if(NOT RKNN_INCLUDE_DIR)
    message(FATAL_ERROR "未找到 rknn_api.h，请安装 librknnrt-dev 或设置 RKNN_INCLUDE_DIR")
endif()

if(NOT RKNN_LIB)
    message(FATAL_ERROR "未找到 librknnrt.so，请安装 librknnrt 或设置 RKNN_LIB")
endif()

add_executable(vision
    src/main.cpp
    src/camera.cpp
    src/detector.cpp
    src/detector_pool.cpp
    src/detector_utils.cpp
    src/split_head_detector.cpp
    src/yolo_world_detector.cpp
    src/multi_model.cpp
    src/vision_qa.cpp
    src/tracker.cpp
    src/lidar.cpp
    src/uart_link.cpp
    src/voice_uart.cpp
    semantic/command_parser.cpp
    semantic/perception_adapter.cpp
    semantic/world_model.cpp
    semantic/target_resolver.cpp
    semantic/task_fsm.cpp
    semantic/action_interface.cpp
    semantic/semantic_engine.cpp
)

target_include_directories(vision PRIVATE
    include
    semantic
    ${OpenCV_INCLUDE_DIRS}
    ${RKNN_INCLUDE_DIR}
)

target_link_libraries(vision PRIVATE
    ${OpenCV_LIBS}
    ${RKNN_LIB}
    Threads::Threads
)
