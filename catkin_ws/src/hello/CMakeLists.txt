cmake_minimum_required(VERSION 2.8.3)
project(hello)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (ompl REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(geometric_feasible_2d src/geometric_feasible_2d.cpp src/VisualNode.cpp)
target_link_libraries(geometric_feasible_2d Eigen3::Eigen ompl ${catkin_LIBRARIES})

add_executable(geometric_optimal_2d src/geometric_optimal_2d.cpp src/VisualNode.cpp)
target_link_libraries(geometric_optimal_2d Eigen3::Eigen ompl ${catkin_LIBRARIES})