#define EIGEN_USE_MKL_ALL

#include "util/logger.hpp"
#include <ros/ros.h>
#include "collision_detection/MoveItCollisionHelper.h"
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include "Kinematics/custom_kinematics.hpp"
#include "planner/CartesianPlanner.h"
#include "planner/PlannerMethod.hpp"
int main(int argc, char **argv)
{
    /*
    logger a( argv[0]);
    ros::init(argc, argv, "NursingRobot");
    ros::AsyncSpinner spinner(2);
    spinner.start();
        my_collision_detection::MoveItCollisionHelperPtr moveItCollisionHelper(new my_collision_detection::MoveItCollisionHelper("manipulator_i5",
                                                                        "/home/xcy/WorkSpace/src/NursingRobot/config/aubo_i5.yaml",
                                                                        my_kinematics::aubo_i5_analytical_IK));

    const auto & kinematic_ptr = moveItCollisionHelper->getKinematicsPtr();
    auto current_pose = kinematic_ptr->fk(state_space::JointSpace::Zero());
    state_space::vector_SE3 cartesian_path;
    planner::CartesianPlanner::getCartesianCircle(cartesian_path,state_space::SE3(state_space::SO3::Zero(),Eigen::Vector3d{-0.05,0,0}),
                                                  state_space::SE3(state_space::SO3::Zero(),Eigen::Vector3d{-0.25,0,0}),
                                                  -2*M_PI);
    state_space::vector_JointSpace trajectory;
    double percentage = planner::CartesianPlanner::computeCartesianPath(state_space::JointSpace::Zero(),
                                                                        cartesian_path,
                                                                        trajectory,
                                                                        planner::MaxEEFStep(0.0,0.01),
                                                                        planner::JumpThreshold::MIN(),
                                                                        kinematic_ptr->getEndEffectorName(),
                                                                        moveItCollisionHelper);
    std::cout<<percentage<<std::endl;
    EigenSTL::vector_Affine3d waypoints;
    for(const auto & item:trajectory)
    {
        waypoints.emplace_back(Eigen::Affine3d{kinematic_ptr->fk(item).SE3Matrix()});
    }
    /*
    std::size_t iter= 0;
    while(++iter<trajectory.size()){
        std::cout<<planner::distance(trajectory[iter-1],trajectory[iter])<<std::endl;
    }*
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.loadRemoteControl();

    visual_tools.publishAxisLabeled(waypoints[0], " start", rvt::LARGE);
    visual_tools.publishAxisLabeled(waypoints.back(), " end ", rvt::LARGE);
    //visual_tools.publishAxisLabeled(Eigen::Affine3d(center.SE3Matrix())," center ",rvt::LARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (const auto & waypoint : waypoints)
        visual_tools.publishAxis(waypoint, rvt::SMALL);
    visual_tools.trigger();
    spinner.stop();*/
    auto lazy_rrt_ptr = planner::createPlanner<state_space::JointSpace>(planner::LAZY_RRT,6);

    return 0;
}
