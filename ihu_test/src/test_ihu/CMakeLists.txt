cmake_minimum_required(VERSION 3.0.2)
project(ihu_test)


find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found at:" ${EIGEN3_INCLUDE_DIR})

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(ihu_test main.cpp)
target_link_libraries(ihu_test ${catkin_LIBRARIES})