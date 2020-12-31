cmake_minimum_required(VERSION 3.17)
project(NursingRobot)

set(CMAKE_CXX_STANDARD 11)
# include cmake files in the 'cmake folder'
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED yaml-cpp)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

#fcl
find_package(fcl REQUIRED)

find_package(Flann REQUIRED)
add_library(Flann STATIC IMPORTED)

include_directories (
        SYSTEM
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}

)
link_directories(
        /user/local/lib
        ${YAML_LIBRARIES}
        ${FCL_LIBRARIES}
)

set(lib_SRC
        include/StateSpace/SO3.hpp
        include/StateSpace/SE3.hpp
        include/StateSpace/JointSpace.hpp
        include/StateSpace/rn.hpp
        src/StateSpace/state_space.cpp)

add_library("StateSpace" STATIC  ${lib_SRC})

set(lib_RM
        include/RobotModel/RobotModel.h
        include/RobotModel/joint_model.h
        include/RobotModel/link_model.h
        include/RobotModel/aabb.h
        src/RobotModel/joint_model.cpp
        src/RobotModel/link_model.cpp
        src/RobotModel/RobotModel.cpp
        src/RobotModel/aabb.cpp)

add_library(Kinematics src/Kinematics/Kinematics.cpp include/Kinematics/custom_kinematics.hpp)
target_link_libraries(Kinematics StateSpace ${YAML_LIBRARIES})

add_library(RRT include/planner/RRT.hpp)
target_link_libraries(RRT StateSpace Kinematics)

add_library(PlanningInterface src/planner/PlanningInterface.cpp)
target_link_libraries(PlanningInterface StateSpace Kinematics)

add_library(RobotModel ${lib_RM})
target_link_libraries(RobotModel Kinematics StateSpace ${YAML_LIBRARIES})


#executale
add_executable(NursingRobot src/main.cpp)
target_link_libraries(NursingRobot )

add_executable(6dTest test/6d_rrt_test.cpp)
target_link_libraries(6dTest StateSpace RRT)

add_executable(kTest test/kinematics_test.cpp)
target_link_libraries(kTest StateSpace Kinematics)

add_executable(RTest test/robot_model_test.cpp)
target_link_libraries(RTest StateSpace Kinematics RobotModel)