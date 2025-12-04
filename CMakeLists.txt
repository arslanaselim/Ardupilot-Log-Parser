cmake_minimum_required(VERSION 3.10)
project(LogP)

set(CMAKE_CXX_STANDARD 17)

find_package(Python3 COMPONENTS Development NumPy REQUIRED)

add_executable(LogP main.cpp)

target_link_libraries(LogP Python3::Python Python3::NumPy)
