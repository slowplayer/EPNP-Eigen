cmake_minimum_required(VERSION 2.8)
project(EPnP-Eigen)

set(CMAKE_BUILD_TYPE  "Realease")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)