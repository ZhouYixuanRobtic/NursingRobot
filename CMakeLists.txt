cmake_minimum_required(VERSION 3.17)
project(NursingRobot)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED yaml-cpp)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

include_directories (
        SYSTEM
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_INCLUDE_DIRS}
)
link_directories(
        /user/local/lib
        ${YAML_LIBRARIES}
)

add_library(SE3 src/SE3.cpp include/SE3.h)

add_library(RobotModel src/RobotModel.cpp)
target_link_libraries(RobotModel SE3 ${YAML_LIBRARIES})

add_executable(NursingRobot src/main.cpp)
target_link_libraries(NursingRobot SE3 RobotModel ${YAML_LIBRARIES})
