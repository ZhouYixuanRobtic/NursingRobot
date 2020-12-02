cmake_minimum_required(VERSION 3.17)
project(NursingRobot)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories (
        SYSTEM
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(SE3 src/SE3.cpp include/SE3.h)

add_executable(NursingRobot src/main.cpp)
target_link_libraries(NursingRobot SE3)
