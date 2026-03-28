cmake_minimum_required(VERSION 3.22)

project(mono_geo_loc)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

find_package(absl REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})

link_libraries(Ceres::ceres spdlog::spdlog)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

add_executable(main_node main.cpp)

add_executable(serial_test serial_test.cpp)
