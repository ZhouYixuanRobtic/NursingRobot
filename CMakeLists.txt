cmake_minimum_required(VERSION 3.0.2)
project(NursingRobot)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_STANDARD 11)
# include cmake files in the 'cmake folder'
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_compile_options(-std=c++11)

set(MKL_INCLUDE_DIRS "/opt/intel/oneapi/mkl/latest/include")
set(MKL_LIB_DIRS "/opt/intel/oneapi/mkl/latest/lib/intel64")

find_package(Eigen3 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs roscpp rospy std_msgs
        interactive_markers moveit_core moveit_visual_tools moveit_ros_planning moveit_ros_planning_interface
        pluginlib geometric_shapes pcl_ros pcl_conversions rosbag tf2_ros tf2_eigen tf2_geometry_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED yaml-cpp)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if (NEW_YAMLCPP_FOUND)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif (NEW_YAMLCPP_FOUND)

#fcl
find_package(fcl REQUIRED)

find_package(Flann REQUIRED)
add_library(Flann STATIC IMPORTED)

#glog
include(./cmake/glog.cmake)

catkin_package()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${MKL_INCLUDE_DIRS}
)
link_directories(
        /user/local/lib
        ${YAML_LIBRARIES}
        ${FCL_LIBRARIES}
        ${catkin_LIBRARY_DIRS}
        ${GLOG_LIBRARIES}
        ${MKL_LIB_DIRS}
)

set(lib_SRC
        include/StateSpace/SO3.hpp
        include/StateSpace/SE3.hpp
        include/StateSpace/JointSpace.hpp
        include/StateSpace/rn.hpp
        src/StateSpace/state_space.cpp)

add_library("StateSpace" STATIC ${lib_SRC})

add_library(Kinematics src/Kinematics/Kinematics.cpp include/Kinematics/custom_kinematics.hpp)
target_link_libraries(Kinematics StateSpace ${YAML_LIBRARIES} ${GLOG_LIBRARIES})

add_library(RRT include/planner/RRT.hpp)
target_link_libraries(RRT StateSpace Kinematics)

add_library(collision_helper src/collision_detection/MoveItCollisionHelperImpl.cpp)
target_link_libraries(collision_helper Kinematics StateSpace ${catkin_LIBRARIES})

add_library(cartesianPlanner src/planner/CartesianPlanner.cpp)
target_link_libraries(cartesianPlanner Kinematics collision_helper StateSpace)

add_library(PlanningInterface src/planner/PlanningInterface.cpp)
target_link_libraries(PlanningInterface collision_helper cartesianPlanner)

#executale
add_executable(NursingRobot src/main.cpp)
target_link_libraries(NursingRobot ${GLOG_LIBRARIES} ${catkin_LIBRARIES} collision_helper cartesianPlanner)

catkin_add_gtest(6dTest test/rrt_test.cpp)
target_link_libraries(6dTest StateSpace RRT ${catkin_LIBRARIES} collision_helper)

catkin_add_gtest(kTest test/kinematics_test.cpp)
target_link_libraries(kTest ${catkin_LIBRARIES} collision_helper)

catkin_add_gtest(cTest test/moveit_collision_helper_test.cpp)
target_link_libraries(cTest ${catkin_LIBRARIES} collision_helper  )

catkin_add_gtest(carTest test/cartesian_test.cpp)
target_link_libraries(carTest ${catkin_LIBRARIES} cartesianPlanner)

catkin_add_gtest(planning_interface_test test/planning_interface_test.cpp)
target_link_libraries(planning_interface_test ${catkin_LIBRARIES} PlanningInterface)