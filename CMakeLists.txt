cmake_minimum_required(VERSION 3.10)
project(Mission_Systems)

set(CMAKE_CXX_STANDARD 17)

find_package(Python3 COMPONENTS Development NumPy REQUIRED)

add_executable(LogParser Task1_LogAnalysis/main.cpp)

target_link_libraries(LogParser Python3::Python Python3::NumPy)

add_executable(Lawnmower Task2_SearchPattern/lawnmower.cpp)