cmake_minimum_required(VERSION 2.8.3)
project(hello)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (ompl REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mp2d src/mp2d.cpp src/VisualNode.cpp)
target_link_libraries(mp2d Eigen3::Eigen ompl ${catkin_LIBRARIES})