cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
project(Hello_OMPL VERSION 1.0)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (ompl REQUIRED)

add_executable(hello_ompl main.cpp)

target_link_libraries(hello_ompl Eigen3::Eigen ompl)
