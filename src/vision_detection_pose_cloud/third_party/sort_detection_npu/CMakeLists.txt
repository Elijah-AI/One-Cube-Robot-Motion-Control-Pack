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
                    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rknpu2/include
                    )
file(GLOB SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)

#dma
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/dma)

#drm
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/drm)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

# 为 RK3588 设置 RKNN 库路径
set(RKNN_API_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rknpu2/Linux)  # 替换为实际路径
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

# 未设 CMAKE_BUILD_TYPE 时 gcc 默认 -O0: Eigen 表达式无内联/向量化,
# 点云配准(ICP 法方程/PLY 解析)实测比 -O2 慢 10~50 倍, 故强制开优化
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CXX_FLAGS MATCHES "-O[123s]")
    target_compile_options(${PROJECT_NAME} PRIVATE -O2)
endif()


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

set(SORT_DETECTION_MODEL_DIR "${CMAKE_INSTALL_PREFIX}/share/vision_detection_pose_cloud/models"
    CACHE PATH "Installed NPU models and labels")
set(SORT_DETECTION_DEBUG_DIR "/tmp/vision_detection_pose_cloud/"
    CACHE PATH "Writable point-cloud debug output")
target_compile_definitions(${PROJECT_NAME} PRIVATE
    SORT_DETECTION_MODEL_DIR="${SORT_DETECTION_MODEL_DIR}"
    SORT_DETECTION_DEBUG_DIR="${SORT_DETECTION_DEBUG_DIR}"
)
target_link_libraries(${PROJECT_NAME} glog gflags)
