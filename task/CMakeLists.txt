cmake_minimum_required(VERSION 3.16)

project(freq)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -O3")

add_executable(${PROJECT_NAME} main.cpp)
