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

# include cmake files in the 'cmake folder'
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Flann REQUIRED)
add_library(Flann STATIC IMPORTED)


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

set(lib_SRC
        src/StateSpace/SO3.cpp include/StateSpace/SO3.hpp
        src/StateSpace/SE3.cpp include/StateSpace/SE3.hpp
        include/StateSpace/JointSpace.hpp include/StateSpace/rn.hpp)

# add_library("RRT" STATIC ${lib_SRC})
add_library("StateSpace" ${lib_SRC})


add_library(RobotModel src/RobotModel.cpp)
target_link_libraries(RobotModel StateSpace ${YAML_LIBRARIES})

add_library(RRT src/RRT.cpp)
target_link_libraries(RRT StateSpace RobotModel)

add_library(PlanningInterface src/PlanningInterface.cpp)
target_link_libraries(PlanningInterface StateSpace RobotModel)

add_executable(NursingRobot src/main.cpp)
target_link_libraries(NursingRobot StateSpace RobotModel ${YAML_LIBRARIES})
