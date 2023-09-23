cmake_minimum_required(VERSION 3.5)
project(sensor_fusion)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(sensor_fusion main.cpp
    EkfModel.cpp
    EkfModel.h
    UKF.cpp
    UKF.h
    Types.cpp
    Types.h
)

# build with O3
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

target_link_libraries(sensor_fusion Eigen3::Eigen)