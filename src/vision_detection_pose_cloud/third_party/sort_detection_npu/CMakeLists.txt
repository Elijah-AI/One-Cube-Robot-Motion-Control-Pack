cmake_minimum_required(VERSION 3.15) #设置cmake最低要求

project(sort_detection)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(STATUS "OpenMP found, enabling support")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty 3rdparty)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/utils utils)

find_package(OpenCV 4 REQUIRED)
find_package(PCL  REQUIRED)


get_filename_component(PROJECT_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_DIR}/bin)

include_directories(${PROJECT_DIR}/include/
                    ${PROJECT_DIR}/include/onnxruntime
                    ${PROJECT_DIR}/API/
                    ${PCL_INCLUDE_DIRS}
                    /home/neardi/ros2_ws/src/vision_detection_pose_cloud/third_party/sort_detection_npu/3rdparty/rknpu2/include
                    )
file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

#dma
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/dma)

#drm
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/drm)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# 为 RK3588 设置 RKNN 库路径
set(RKNN_API_PATH /home/neardi/ros2_ws/src/vision_detection_pose_cloud/third_party/sort_detection_npu/3rdparty/rknpu2/Linux)  # 替换为实际路径
find_library(RKNNRT_LIB rknnrt ${RKNN_API_PATH}/aarch64)  # 修正拼写 aarch64
if(NOT RKNNRT_LIB)
    message(FATAL_ERROR "RKNNRT_LIB not found, check RKNN_API_PATH: ${RKNN_API_PATH}")
endif()


file(GLOB_RECURSE head_srcs 
        ${PROJECT_SOURCE_DIR}/include/*.h
        ${PROJECT_SOURCE_DIR}/API/*.h
        )
file(GLOB_RECURSE cpp_srcs 
        ${PROJECT_SOURCE_DIR}/src/*.cpp
        ${PROJECT_SOURCE_DIR}/src/*.cc
        ${PROJECT_SOURCE_DIR}/API/*.cpp
        )

add_library(${PROJECT_NAME} SHARED 
            ${head_srcs}
            ${cpp_srcs}
            )


target_link_libraries(${PROJECT_NAME}
            ${OpenCV_LIBRARIES}
            ${PCL_LIBRARIES}
            fileutils
            imageutils
            imagedrawing
            ${RKNNRT_LIB}
            OpenMP::OpenMP_CXX
            )

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64" )
    message("x86_64-- Current cpu frame: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
    include_directories(${PROJECT_SOURCE_DIR}/../2_x86_all/include)
    link_directories(${PROJECT_SOURCE_DIR}/../2_x86_all/lib)

else(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("aarch64-- Current cpu frame: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
    include_directories(${PROJECT_SOURCE_DIR}/../1_arm_all/include)
    link_directories(${PROJECT_SOURCE_DIR}/../1_arm_all/lib)

endif()

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION 1.1.0.1 SOVERSION 1)
