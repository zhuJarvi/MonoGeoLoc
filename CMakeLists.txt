cmake_minimum_required(VERSION 3.22)

project(mono_geo_loc)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_package(cserialport REQUIRED)

# find_package(absl REQUIRED)
# find_package(Ceres REQUIRED)

include_directories(${CMAKE_SOURCE_DIR})

link_libraries(${OpenCV_LIBS}  spdlog::spdlog)
include_directories(${OpenCV_INCLUDE_DIRS} ${CSerialPort_INCLUDE_DIRS})

message(STATUS ${cserialport})
message(STATUS ${CSerialPort_INCLUDE_DIRS})
message(STATUS ${CSerialPort_LIBS})

add_executable(main_node main.cpp)

add_executable(serial_test serial_test.cpp)
target_link_libraries(serial_test ${OpenCV_LIBS} /usr/local/lib/libcserialport.so spdlog::spdlog)
add_executable(camera_config_test camera_config_test.cpp)
