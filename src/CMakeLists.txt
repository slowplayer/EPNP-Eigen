cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Realease")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

#eigen 
include_directories("/usr/include/eigen3")

add_executable(eigen-pnp main.cpp pnp_solver.cpp)